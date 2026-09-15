"""Radio-profile selector + auto policy + encoder confirmation loop.

Covers the 2026-07-25 selector work:
  * AutoRadioPolicy state machine (pure, synthetic clock)
  * GET/POST /api/settings/radio_profile (validation, persistence,
    retained publish, auto handoff)
  * /api/encode_mode/current confirmation fields (tractor ack + rx codec)
  * MQTT ack caching for status/radio_profile/{tx,rx} + status/encode_mode

Same import recipe as test_web_ui_auth.py: patch the paho client class,
reload web_ui so the module-level broker connect binds to the mock.
"""

from __future__ import annotations

import json
import os
import sys
import tempfile
import time
import unittest
from pathlib import Path
from unittest import mock

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

try:
    import paho.mqtt.client  # noqa: F401
    import fastapi           # noqa: F401
    from fastapi.testclient import TestClient  # noqa: F401
except ImportError:
    raise unittest.SkipTest(
        "paho-mqtt + fastapi required for radio-profile tests")


class _Msg:
    def __init__(self, topic: str, obj) -> None:
        self.topic = topic
        self.payload = json.dumps(obj).encode()


class RadioProfileTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls._tmp = tempfile.TemporaryDirectory()
        os.environ["LIFETRAC_PIN"] = "424242"
        os.environ["LIFETRAC_ENCODE_MODE_STORE"] = str(
            Path(cls._tmp.name) / ".encode_mode")
        os.environ["LIFETRAC_RADIO_PROFILE_STORE"] = str(
            Path(cls._tmp.name) / ".radio_profile")
        with mock.patch("paho.mqtt.client.Client") as mqtt_class:
            instance = mqtt_class.return_value
            instance.connect = mock.MagicMock()
            instance.loop_start = mock.MagicMock()
            instance.subscribe = mock.MagicMock()
            pub_info = mock.MagicMock()
            pub_info.rc = 0
            instance.publish = mock.MagicMock(return_value=pub_info)
            import importlib
            import web_ui
            importlib.reload(web_ui)
            cls.web_ui = web_ui
            cls.client = TestClient(web_ui.app)
            cls.mqtt = instance

    @classmethod
    def tearDownClass(cls) -> None:
        cls._tmp.cleanup()

    def setUp(self) -> None:
        self.client.post("/api/login", json={"pin": "424242"})
        self.mqtt.publish.reset_mock()

    # ---- AutoRadioPolicy (pure state machine) ----

    def _policy(self, profile: int = 2, now: float = 0.0):
        return self.web_ui.AutoRadioPolicy(initial_profile=profile, now=now)

    def test_policy_holds_dts_while_healthy(self):
        p = self._policy()
        for t in range(0, 300, 5):
            self.assertIsNone(p.evaluate(
                now=float(t), sample_age_s=2.0, timeouts_per_10s=0.0))
        self.assertEqual(p.profile, 2)

    def test_policy_degrades_on_stale_link_after_min_gap(self):
        p = self._policy()
        # inside the min-switch gap → hold even though unhealthy
        self.assertIsNone(p.evaluate(
            now=30.0, sample_age_s=None, timeouts_per_10s=0.0))
        self.assertEqual(p.evaluate(
            now=61.0, sample_age_s=None, timeouts_per_10s=0.0), 1)
        self.assertEqual(p.profile, 1)

    def test_policy_degrades_on_timeout_rate(self):
        p = self._policy()
        self.assertEqual(p.evaluate(
            now=100.0, sample_age_s=2.0, timeouts_per_10s=10.0), 1)

    def test_policy_promotes_after_healthy_dwell(self):
        p = self._policy()
        self.assertEqual(p.evaluate(
            now=61.0, sample_age_s=None, timeouts_per_10s=0.0), 1)
        self.assertIsNone(p.evaluate(
            now=70.0, sample_age_s=2.0, timeouts_per_10s=0.0))
        self.assertIsNone(p.evaluate(
            now=125.0, sample_age_s=2.0, timeouts_per_10s=0.0))
        self.assertEqual(p.evaluate(
            now=131.0, sample_age_s=2.0, timeouts_per_10s=0.0), 2)

    def test_policy_health_dwell_resets_on_blip(self):
        p = self._policy()
        self.assertEqual(p.evaluate(
            now=61.0, sample_age_s=None, timeouts_per_10s=0.0), 1)
        self.assertIsNone(p.evaluate(
            now=70.0, sample_age_s=2.0, timeouts_per_10s=0.0))
        # unhealthy blip at t=100 resets the promote dwell...
        self.assertIsNone(p.evaluate(
            now=100.0, sample_age_s=2.0, timeouts_per_10s=9.0))
        # ...so health restarting at t=155 must wait a FULL dwell again:
        self.assertIsNone(p.evaluate(
            now=155.0, sample_age_s=2.0, timeouts_per_10s=0.0))
        self.assertIsNone(p.evaluate(
            now=210.0, sample_age_s=2.0, timeouts_per_10s=0.0))
        self.assertEqual(p.evaluate(
            now=216.0, sample_age_s=2.0, timeouts_per_10s=0.0), 2)

    def test_policy_never_selects_bench_profile(self):
        p = self._policy(profile=0)   # bad seed coerced to DTS
        self.assertEqual(p.profile, 2)

    # ---- RS-12.16 (2026-09-15): dead-air + loss-rate inputs ----
    # Leg S replay: 39 % loss with 53 s + 22 s lock-loss blackouts, yet the
    # age/timeout inputs read the whole leg HEALTHY (peak 1.0 timeouts per
    # 10 s; samples never stale because the daemon publishes zero-samples
    # through dead air). Only the fragment counter going flat sees it.

    def _healthy_kw(self, **extra):
        kw = dict(sample_age_s=2.0, timeouts_per_10s=0.0)
        kw.update(extra)
        return kw

    def test_policy_dead_air_after_streaming_degrades(self):
        p = self._policy()
        for t, n in ((0.0, 0), (5.0, 40), (10.0, 80), (15.0, 120)):
            self.assertIsNone(p.evaluate(now=t, frags_seen=n,
                                         **self._healthy_kw()))
        # blackout: counter frozen at 120 while samples keep arriving
        self.assertIsNone(p.evaluate(now=20.0, frags_seen=120,
                                     **self._healthy_kw()))
        # 10 s silent => unhealthy, but inside MIN_SWITCH_GAP => hold
        self.assertIsNone(p.evaluate(now=25.0, frags_seen=120,
                                     **self._healthy_kw()))
        # past the gap, still silent => degrade DTS -> FHSS
        self.assertEqual(p.evaluate(now=61.0, frags_seen=120,
                                    **self._healthy_kw()), 1)

    def test_policy_dead_air_resets_promote_dwell(self):
        """On FHSS, a blackout mid-dwell must restart the 60 s health
        clock exactly like a timeout blip does."""
        p = self._policy(profile=1)
        p.evaluate(now=0.0, frags_seen=0, **self._healthy_kw())
        for t, n in ((5.0, 10), (30.0, 60), (55.0, 110)):
            self.assertIsNone(p.evaluate(now=t, frags_seen=n,
                                         **self._healthy_kw()))
        # frozen from t=55: dead air at t=70 resets the dwell
        self.assertIsNone(p.evaluate(now=70.0, frags_seen=110,
                                     **self._healthy_kw()))
        # stream resumes at t=75; promote only after a FULL fresh dwell
        self.assertIsNone(p.evaluate(now=75.0, frags_seen=111,
                                     **self._healthy_kw()))
        self.assertIsNone(p.evaluate(now=130.0, frags_seen=200,
                                     **self._healthy_kw()))
        self.assertEqual(p.evaluate(now=136.0, frags_seen=210,
                                    **self._healthy_kw()), 2)

    def test_policy_idle_from_boot_is_not_dead_air(self):
        """A counter that never advanced never started a silence clock:
        an idle tractor must not degrade or flap the profile."""
        p = self._policy()
        for t in range(0, 300, 5):
            self.assertIsNone(p.evaluate(now=float(t), frags_seen=0,
                                         **self._healthy_kw()))
        self.assertEqual(p.profile, 2)

    def test_policy_long_silence_becomes_idle(self):
        """Beyond STREAM_MEMORY_S the link is idle, not sick: an absent
        tractor must not pin the policy unhealthy forever."""
        p = self._policy()
        p.evaluate(now=0.0, frags_seen=0, **self._healthy_kw())
        p.evaluate(now=5.0, frags_seen=50, **self._healthy_kw())
        self.assertTrue(p._note_frags(20.0, 50))      # 15 s: dead air
        self.assertFalse(p._note_frags(75.0, 50))     # 70 s: idle
        self.assertIsNone(p.evaluate(now=80.0, frags_seen=50,
                                     **self._healthy_kw()))

    def test_policy_stream_resuming_clears_dead_air(self):
        p = self._policy()
        p.evaluate(now=0.0, frags_seen=0, **self._healthy_kw())
        p.evaluate(now=5.0, frags_seen=50, **self._healthy_kw())
        self.assertTrue(p._note_frags(20.0, 50))      # silent 15 s
        self.assertFalse(p._note_frags(21.0, 51))     # one fragment: alive
        self.assertFalse(p._note_frags(25.0, 51))     # 4 s: under DEAD_AIR_S

    def test_policy_counter_reset_counts_as_alive(self):
        """A daemon restart drops rx_frames_seen; a CHANGE is a live
        stream, not silence."""
        p = self._policy()
        p.evaluate(now=0.0, frags_seen=500, **self._healthy_kw())
        p.evaluate(now=5.0, frags_seen=600, **self._healthy_kw())
        self.assertFalse(p._note_frags(20.0, 3))

    def test_policy_loss_rate_above_max_degrades(self):
        p = self._policy()
        self.assertEqual(p.evaluate(now=100.0, loss_rate=0.39,
                                    **self._healthy_kw()), 1)

    def test_policy_bench_floor_loss_holds(self):
        """1-8 % fragment loss is the healthy profile-1 bench floor."""
        p = self._policy()
        for t in range(0, 300, 5):
            self.assertIsNone(p.evaluate(now=float(t), loss_rate=0.08,
                                         **self._healthy_kw()))
        self.assertEqual(p.profile, 2)

    def test_policy_loss_none_is_window_too_small(self):
        p = self._policy()
        self.assertIsNone(p.evaluate(now=100.0, loss_rate=None,
                                     **self._healthy_kw()))

    # ---- RS-12.18 (leg V, 2026-09-15): re-sync to the daemon's actual profile ----

    def test_policy_resyncs_after_sustained_daemon_revert(self):
        """Leg V: policy degraded to 1 at t=0; the daemon reverted to 2 at
        t=46 (no frames on the new profile). The policy must adopt 2 once
        the disagreement has persisted RESYNC_AFTER_S, and treat it as a
        switch: no further degrade until a fresh MIN_SWITCH_GAP."""
        p = self._policy(profile=1)
        p._last_switch_t = 0.0
        self.assertFalse(p.observe_active(2, 46.0))     # first sighting
        self.assertFalse(p.observe_active(2, 56.0))     # 10 s: still inside the handshake window
        self.assertTrue(p.observe_active(2, 66.0))      # 20 s: re-sync
        self.assertEqual(p.profile, 2)
        # a revert is a switch: dead air right after it must NOT degrade
        # until MIN_SWITCH_GAP has elapsed from the re-sync
        p.evaluate(now=70.0, frags_seen=0, **self._healthy_kw())
        p.evaluate(now=75.0, frags_seen=50, **self._healthy_kw())
        self.assertIsNone(p.evaluate(now=100.0, frags_seen=50,
                                     **self._healthy_kw()))   # dead air, gap not ok
        self.assertEqual(p.evaluate(now=127.0, frags_seen=50,
                                    **self._healthy_kw()), 1)  # gap ok -> degrade again

    def test_policy_ignores_transient_mismatch_during_handshake(self):
        """Policy pins 1; the daemon still reports 2 until the tractor ACKs
        (up to 12 s). That is not a revert."""
        p = self._policy(profile=2)
        p._last_switch_t = 0.0
        self.assertEqual(p.evaluate(now=61.0, sample_age_s=None,
                                    timeouts_per_10s=0.0), 1)
        self.assertFalse(p.observe_active(2, 62.0))     # daemon not switched yet
        self.assertFalse(p.observe_active(2, 70.0))     # 8 s later, still handshaking
        self.assertFalse(p.observe_active(1, 75.0))     # daemon switched: agreement
        self.assertEqual(p.profile, 1)
        self.assertIsNone(p._mismatch_since)

    def test_policy_observe_none_or_bench_profile_is_noop(self):
        p = self._policy(profile=2)
        self.assertFalse(p.observe_active(None, 10.0))
        self.assertFalse(p.observe_active(0, 40.0))     # bench 915 never adopted
        self.assertFalse(p.observe_active("2", 70.0))
        self.assertEqual(p.profile, 2)

    def test_policy_pre_rs1216_call_shape_unchanged(self):
        p = self._policy()
        self.assertIsNone(p.evaluate(now=10.0, sample_age_s=2.0,
                                     timeouts_per_10s=0.0))

    # ---- endpoints ----

    def test_get_shape(self):
        r = self.client.get("/api/settings/radio_profile")
        self.assertEqual(r.status_code, 200)
        body = r.json()
        self.assertEqual(body["choices"], ["auto", "0", "1", "2"])
        self.assertIn(body["current"], body["choices"])
        self.assertIn("acks", body)
        self.assertIn("active_profile", body)

    def test_post_concrete_publishes_retained_and_persists(self):
        r = self.client.post("/api/settings/radio_profile",
                             json={"profile": "1"})
        self.assertEqual(r.status_code, 200)
        self.assertTrue(r.json()["ok"])
        calls = [c for c in self.mqtt.publish.call_args_list
                 if c.args[0] == self.web_ui._RADIO_PROFILE_TOPIC]
        self.assertTrue(calls, "no publish on the control topic")
        args, kwargs = calls[-1]
        self.assertTrue(kwargs.get("retain"))
        self.assertEqual(json.loads(args[1])["profile"], 1)
        self.assertEqual(self.web_ui._load_radio_profile(), "1")

    def test_post_rejects_garbage(self):
        self.assertEqual(self.client.post(
            "/api/settings/radio_profile",
            json={"profile": "3"}).status_code, 422)
        self.assertEqual(self.client.post(
            "/api/settings/radio_profile",
            json={"profile": "fhss"}).status_code, 422)

    def test_post_auto_seeds_policy_and_commands_start(self):
        r = self.client.post("/api/settings/radio_profile",
                             json={"profile": "auto"})
        self.assertEqual(r.status_code, 200)
        with self.web_ui._radio_profile_lock:
            policy = self.web_ui._radio_auto_policy
        self.assertIsNotNone(policy)
        self.assertIn(policy.profile, (1, 2))
        payloads = [json.loads(c.args[1])
                    for c in self.mqtt.publish.call_args_list
                    if c.args[0] == self.web_ui._RADIO_PROFILE_TOPIC]
        self.assertTrue(payloads, "auto must command a starting profile")
        self.assertEqual(payloads[-1]["source"], "auto-start")

    def test_encode_mode_current_carries_confirmation(self):
        self.web_ui._encode_mode_ack = {
            "requested": 4, "effective": 1,
            "effective_name": "y_only", "clamped": True}
        self.web_ui._image_publisher.link_stats = {
            "rx_codec": 4, "rx_codec_name": "webp_luma",
            "ts": time.time()}
        r = self.client.get("/api/encode_mode/current")
        self.assertEqual(r.status_code, 200)
        body = r.json()
        self.assertTrue(body["tractor"]["clamped"])
        self.assertEqual(body["rx_codec_name"], "webp_luma")

    # ---- MQTT ack caching ----

    def test_mqtt_ack_caching(self):
        wu = self.web_ui
        wu._on_mqtt_message(None, None, _Msg(
            "lifetrac/v25/status/radio_profile/tx",
            {"ok": True, "profile": 2}))
        wu._on_mqtt_message(None, None, _Msg(
            "lifetrac/v25/status/radio_profile/rx",
            {"ok": False, "profile": 1}))
        wu._on_mqtt_message(None, None, _Msg(
            "lifetrac/v25/status/encode_mode",
            {"requested": 6, "effective": 6,
             "effective_name": "mono_g4", "clamped": False}))
        with wu._radio_profile_lock:
            self.assertEqual(wu._radio_acks["tx"]["profile"], 2)
            self.assertFalse(wu._radio_acks["rx"]["ok"])
        self.assertEqual(wu._encode_mode_ack["effective_name"], "mono_g4")


if __name__ == "__main__":
    unittest.main()
