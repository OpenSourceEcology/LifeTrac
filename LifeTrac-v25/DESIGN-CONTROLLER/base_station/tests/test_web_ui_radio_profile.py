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
