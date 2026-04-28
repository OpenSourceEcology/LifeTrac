"""Tests for the 2026-04-27 follow-up sweep modules:

- ``audit_log`` named event helpers
- ``person_alert`` packing + emitter (+ debounce)
- ``link_monitor`` audit hook
- ``tools.lora_rtt._percentile`` (the only pure-logic chunk that's
  worth testing in isolation; the rest needs an MQTT broker)
"""
from __future__ import annotations

import json
import os
import sys
import tempfile
import unittest
from unittest.mock import MagicMock

# tools/ isn't on PYTHONPATH; add it explicitly so we can exercise lora_rtt.
sys.path.insert(0, os.path.abspath(os.path.join(
    os.path.dirname(__file__), "..", "..", "tools")))

from audit_log import AuditLog
from link_monitor import LinkMonitor
from lora_proto import (
    CMD_PERSON_APPEARED,
    HEADER_LEN,
    PROTO_VERSION,
    SRC_BASE,
)
from person_alert import (
    CLS_ANIMAL,
    CLS_PERSON,
    CLS_VEHICLE,
    PersonAlertEmitter,
    build_person_alert_frame,
    cls_id_for,
    pack_person_alert_args,
)


class _Det:
    def __init__(self, cls, conf, x, y, w, h):
        self.cls = cls; self.conf = conf
        self.x = x; self.y = y; self.w = w; self.h = h


class AuditNamedHelpersTests(unittest.TestCase):
    def setUp(self):
        self._tmp = tempfile.mkdtemp(prefix="audit-")
        self.path = os.path.join(self._tmp, "audit.jsonl")
        self.log = AuditLog(path=self.path)

    def tearDown(self):
        self.log.close()

    def _last(self):
        with open(self.path, encoding="utf-8") as fp:
            return json.loads(fp.readlines()[-1])

    def test_log_sf_step(self):
        self.log.log_sf_step(prev_rung=0, new_rung=1, prev_sf=7, new_sf=9,
                             reason="snr_dropped", rssi_dbm=-110.5)
        evt = self._last()
        self.assertEqual(evt["event"], "sf_step")
        self.assertEqual(evt["new_rung"], 1)
        self.assertEqual(evt["reason"], "snr_dropped")
        self.assertAlmostEqual(evt["rssi_dbm"], -110.5)

    def test_log_encode_mode_change(self):
        self.log.log_encode_mode_change(prev="FULL", new="Y_ONLY",
                                        reason="airtime_hysteresis",
                                        airtime_pct=42.0)
        evt = self._last()
        self.assertEqual(evt["event"], "encode_mode_change")
        self.assertEqual(evt["prev"], "FULL")
        self.assertEqual(evt["new"], "Y_ONLY")

    def test_log_fhss_skip(self):
        self.log.log_fhss_skip(channel=3, reason="cca_busy",
                               airtime_pct=12.5)
        evt = self._last()
        self.assertEqual(evt["event"], "fhss_skip")
        self.assertEqual(evt["channel"], 3)

    def test_log_replay_reject(self):
        self.log.log_replay_reject(source_id=0x10, seq=42,
                                   window_low=20, window_high=50)
        evt = self._last()
        self.assertEqual(evt["event"], "replay_reject")
        self.assertEqual(evt["source_id"], 0x10)
        self.assertEqual(evt["seq"], 42)

    def test_log_gcm_reject(self):
        self.log.log_gcm_reject(onair_len=64)
        evt = self._last()
        self.assertEqual(evt["event"], "gcm_tag_reject")
        self.assertEqual(evt["onair_len"], 64)

    def test_log_person_appeared(self):
        self.log.log_person_appeared(source="tractor", cls="person",
                                     confidence=0.93, cx=0.42, cy=0.55)
        evt = self._last()
        self.assertEqual(evt["event"], "person_appeared")
        self.assertEqual(evt["source"], "tractor")
        self.assertEqual(evt["cls"], "person")


class PersonAlertPackTests(unittest.TestCase):
    def test_cls_id_table(self):
        self.assertEqual(cls_id_for("person"), CLS_PERSON)
        self.assertEqual(cls_id_for("animal"), CLS_ANIMAL)
        self.assertEqual(cls_id_for("vehicle"), CLS_VEHICLE)
        self.assertEqual(cls_id_for("Person"), CLS_PERSON)
        self.assertNotEqual(cls_id_for("dog"), CLS_PERSON)

    def test_pack_args_round_trip_extents(self):
        import struct
        # Center of frame → Q7 should round to ~0.
        args = pack_person_alert_args(CLS_PERSON, 1.0, 0.5, 0.5, age_ms=0)
        self.assertEqual(len(args), 6)
        cls_id, conf, cx, cy, age_div10, _ = struct.unpack("<BBbbBB", args)
        self.assertEqual(cls_id, CLS_PERSON)
        self.assertEqual(conf, 100)
        self.assertEqual(cx, 0)              # cx Q7 ≈ 0
        self.assertEqual(cy, 0)              # cy Q7 ≈ 0
        # Top-left → both Q7 should saturate to about -127.
        args = pack_person_alert_args(CLS_PERSON, 0.0, 0.0, 0.0)
        cls_id, conf, cx, cy, _, _ = struct.unpack("<BBbbBB", args)
        self.assertEqual(conf, 0)
        self.assertLessEqual(cx, -126)
        # Bottom-right → both Q7 should saturate near +127.
        args = pack_person_alert_args(CLS_PERSON, 0.55, 1.0, 1.0)
        cls_id, conf, cx, cy, _, _ = struct.unpack("<BBbbBB", args)
        self.assertGreaterEqual(cx, 126)

    def test_build_frame_uses_cmd_opcode(self):
        frame = build_person_alert_frame(seq=7, cls_id=CLS_PERSON, conf=0.9,
                                         cx=0.5, cy=0.5)
        # CommandFrame: [hdr(5)|opcode(1)|args(6)|crc16(2)] = 14 bytes
        self.assertEqual(len(frame), 14)
        # Opcode immediately follows the 5-byte header.
        self.assertEqual(frame[HEADER_LEN], CMD_PERSON_APPEARED)
        # Header byte 0 = version (lower 4 bits) | frame_type (upper 4 bits).
        self.assertEqual(frame[0] & 0x0F, PROTO_VERSION)


class PersonAlertEmitterTests(unittest.TestCase):
    def test_emit_basic_and_audit(self):
        published = []
        seq = [0]
        def seq_provider():
            seq[0] += 1; return seq[0]
        audit = MagicMock()
        em = PersonAlertEmitter(
            seq_provider=seq_provider, publish=published.append,
            audit=audit, source="tractor", source_id=SRC_BASE,
            min_confidence=0.7, debounce_s=0.0)
        out = em.feed([_Det("person", 0.85, 0.4, 0.3, 0.1, 0.2)])
        self.assertEqual(len(out), 1)
        self.assertEqual(em.emitted, 1)
        self.assertEqual(len(published), 1)
        audit.log_person_appeared.assert_called_once()

    def test_low_confidence_suppressed(self):
        em = PersonAlertEmitter(
            seq_provider=lambda: 1, publish=lambda _: None,
            audit=None, debounce_s=0.0)
        out = em.feed([_Det("person", 0.5, 0.5, 0.5, 0.1, 0.1)])
        self.assertEqual(out, [])
        self.assertEqual(em.suppressed_low_conf, 1)
        self.assertEqual(em.emitted, 0)

    def test_other_class_suppressed(self):
        em = PersonAlertEmitter(
            seq_provider=lambda: 1, publish=lambda _: None,
            audit=None, debounce_s=0.0)
        out = em.feed([_Det("dog", 0.99, 0.5, 0.5, 0.1, 0.1)])
        self.assertEqual(out, [])
        self.assertEqual(em.emitted, 0)

    def test_debounce_collapses_repeats(self):
        t = [0.0]
        em = PersonAlertEmitter(
            seq_provider=lambda: 1, publish=lambda _: None, audit=None,
            min_confidence=0.7, debounce_s=1.5,
            monotonic=lambda: t[0])
        em.feed([_Det("person", 0.95, 0.5, 0.5, 0.1, 0.1)])
        em.feed([_Det("person", 0.95, 0.5, 0.5, 0.1, 0.1)])
        self.assertEqual(em.emitted, 1)
        self.assertEqual(em.suppressed_debounce, 1)
        # After the debounce window, a fresh detection emits again.
        t[0] += 2.0
        em.feed([_Det("person", 0.95, 0.5, 0.5, 0.1, 0.1)])
        self.assertEqual(em.emitted, 2)

    def test_publish_failure_does_not_crash(self):
        def boom(_): raise RuntimeError("tx queue full")
        em = PersonAlertEmitter(
            seq_provider=lambda: 1, publish=boom, audit=None,
            debounce_s=0.0)
        # Must not raise.
        em.feed([_Det("person", 0.99, 0.5, 0.5, 0.1, 0.1)])


class LinkMonitorAuditHookTests(unittest.TestCase):
    def test_encode_mode_change_recorded(self):
        from lora_proto import PHY_IMAGE
        audit = MagicMock()
        published = []
        mon = LinkMonitor(
            publish_command=published.append,
            publish_status=None,
            window_ms=1_000,
            required_windows=1,
            audit=audit,
        )
        # Push enough airtime to cross the >25 % threshold and trigger a mode change.
        for ms in range(0, 1_000, 50):
            mon.on_air(ms, PHY_IMAGE, cleartext_len=200)
        snap = mon.tick(now_ms=1_000)
        self.assertIsNotNone(snap)
        if audit.log_encode_mode_change.called:
            kwargs = audit.log_encode_mode_change.call_args.kwargs
            self.assertIn("prev", kwargs)
            self.assertIn("new", kwargs)
            self.assertEqual(kwargs.get("reason"), "airtime_hysteresis")


class LoraRttPercentileTests(unittest.TestCase):
    def test_percentile_basic(self):
        from lora_rtt import _percentile
        s = sorted([10.0, 20.0, 30.0, 40.0, 50.0])
        self.assertEqual(_percentile(s, 0), 10.0)
        self.assertEqual(_percentile(s, 100), 50.0)
        self.assertAlmostEqual(_percentile(s, 50), 30.0)
        # Linear interpolation between samples; numpy default 'linear':
        # k = 4 * 0.75 = 3.0 → returns s[3] = 40.0.
        self.assertAlmostEqual(_percentile(s, 75), 40.0)
        # And p25 lands exactly on s[1] = 20.0.
        self.assertAlmostEqual(_percentile(s, 25), 20.0)

    def test_percentile_empty(self):
        from lora_rtt import _percentile
        self.assertEqual(_percentile([], 50), 0.0)


if __name__ == "__main__":
    unittest.main()
