"""Unit tests for S1.3 helpers: NewestFrameWinsCounter + topic_0x10_decoder."""

from __future__ import annotations

import json
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from newest_frame_wins import (  # noqa: E402
    NewestFrameWinsCounter,
    _is_newer_seq,
)
from topic_0x10_decoder import (  # noqa: E402
    PerSourceLink,
    SourceActiveSnapshot,
    decode_topic_0x10,
)


class IsNewerSeqTests(unittest.TestCase):
    """RFC-1982 16-bit serial-number arithmetic edge cases."""

    def test_forward_in_range(self) -> None:
        self.assertTrue(_is_newer_seq(11, 10))
        self.assertTrue(_is_newer_seq(1000, 1))

    def test_equal_is_not_newer(self) -> None:
        self.assertFalse(_is_newer_seq(10, 10))

    def test_backward_is_not_newer(self) -> None:
        self.assertFalse(_is_newer_seq(10, 11))
        self.assertFalse(_is_newer_seq(1, 1000))

    def test_wraparound_forward(self) -> None:
        # last=0xFFFF, candidate=0 → +1 ahead, must be newer.
        self.assertTrue(_is_newer_seq(0, 0xFFFF))
        self.assertTrue(_is_newer_seq(5, 0xFFFE))

    def test_wraparound_backward(self) -> None:
        # last=0, candidate=0xFFFF → 1 behind in wraparound space.
        self.assertFalse(_is_newer_seq(0xFFFF, 0))


class NewestFrameWinsCounterTests(unittest.TestCase):
    def test_initial_state_is_zero(self) -> None:
        c = NewestFrameWinsCounter()
        d = c.stats_dict()
        self.assertEqual(d["accepted"], 0)
        self.assertEqual(d["stale_dropped"], 0)
        self.assertEqual(d["duplicate_dropped"], 0)
        self.assertFalse(d["first_frame_accepted"])
        self.assertIsNone(d["last_accepted_seq"])

    def test_first_frame_always_accepted(self) -> None:
        c = NewestFrameWinsCounter()
        self.assertTrue(c.observe(42))
        d = c.stats_dict()
        self.assertEqual(d["accepted"], 1)
        self.assertTrue(d["first_frame_accepted"])
        self.assertEqual(d["last_accepted_seq"], 42)

    def test_monotonic_stream_all_accepted(self) -> None:
        c = NewestFrameWinsCounter()
        for seq in (10, 11, 12, 20, 100, 101):
            self.assertTrue(c.observe(seq))
        d = c.stats_dict()
        self.assertEqual(d["accepted"], 6)
        self.assertEqual(d["stale_dropped"], 0)
        self.assertEqual(d["last_accepted_seq"], 101)

    def test_stale_frame_dropped(self) -> None:
        c = NewestFrameWinsCounter()
        self.assertTrue(c.observe(100))
        self.assertFalse(c.observe(99))   # stale
        self.assertFalse(c.observe(50))   # stale
        self.assertTrue(c.observe(101))   # newer again
        d = c.stats_dict()
        self.assertEqual(d["accepted"], 2)
        self.assertEqual(d["stale_dropped"], 2)
        self.assertEqual(d["duplicate_dropped"], 0)

    def test_duplicate_dropped(self) -> None:
        c = NewestFrameWinsCounter()
        c.observe(7)
        self.assertFalse(c.observe(7))
        self.assertFalse(c.observe(7))
        d = c.stats_dict()
        self.assertEqual(d["accepted"], 1)
        self.assertEqual(d["duplicate_dropped"], 2)
        self.assertEqual(d["stale_dropped"], 0)

    def test_wraparound_acceptance(self) -> None:
        c = NewestFrameWinsCounter()
        c.observe(0xFFFE)
        self.assertTrue(c.observe(0xFFFF))   # +1, newer
        self.assertTrue(c.observe(0))        # wrap, +1 newer
        self.assertTrue(c.observe(5))        # +5 newer
        self.assertFalse(c.observe(0xFFF0))  # ~16 behind in wrap space — stale
        d = c.stats_dict()
        self.assertEqual(d["accepted"], 4)
        self.assertEqual(d["stale_dropped"], 1)

    def test_high_bits_are_masked(self) -> None:
        # base_seq is 16-bit; values past 0xFFFF must be truncated, not
        # treated as ever-increasing 32-bit ints.
        c = NewestFrameWinsCounter()
        self.assertTrue(c.observe(0x10005))   # masks to 5
        self.assertTrue(c.observe(0x20006))   # masks to 6 — newer
        d = c.stats_dict()
        self.assertEqual(d["last_accepted_seq"], 6)
        self.assertEqual(d["accepted"], 2)


class DecodeTopic0x10Tests(unittest.TestCase):
    def test_legacy_single_byte_payload(self) -> None:
        snap = decode_topic_0x10(b"\x02")  # 0x02 = base (per lora_proto.SRC_BASE)
        self.assertEqual(snap.active_source, "base")
        self.assertEqual(snap.raw_legacy_byte, 2)
        self.assertIsNone(snap.sf_rung)
        self.assertEqual(snap.sources, {})

    def test_legacy_single_byte_handheld(self) -> None:
        # 0x01 is handheld (canonical lora_proto.SRC_HANDHELD).
        snap = decode_topic_0x10(b"\x01")
        self.assertEqual(snap.active_source, "handheld")
        self.assertEqual(snap.raw_legacy_byte, 1)

    def test_legacy_single_byte_none(self) -> None:
        # 0xFF is the "no active source" sentinel.
        snap = decode_topic_0x10(b"\xFF")
        self.assertEqual(snap.active_source, "none")
        self.assertEqual(snap.raw_legacy_byte, 0xFF)

    def test_legacy_unknown_enum(self) -> None:
        snap = decode_topic_0x10(b"\xFE")
        # Unknown enum should not crash; active_source stays None but the
        # raw byte is preserved for forensics.
        self.assertIsNone(snap.active_source)
        self.assertEqual(snap.raw_legacy_byte, 0xFE)

    def test_full_json_dict(self) -> None:
        payload = json.dumps({
            "active_source": "base",
            "sf_rung": 2,
            "link_unstable": False,
            "u_image": 0.31,
            "u_telemetry": 0.18,
            "u_total": 0.49,
            "sources": {
                "base": {"rssi_dbm": -82.0, "snr_db": 7.5, "loss_rate": 0.01, "last_heard_ms": 12345},
                "handheld": {"rssi_dbm": -95.0, "snr_db": -3.5, "loss_rate": 0.12},
            },
        })
        snap = decode_topic_0x10(payload)
        self.assertEqual(snap.active_source, "base")
        self.assertEqual(snap.sf_rung, 2)
        self.assertFalse(snap.link_unstable)
        self.assertAlmostEqual(snap.u_image or 0.0, 0.31)
        self.assertAlmostEqual(snap.u_telemetry or 0.0, 0.18)
        self.assertAlmostEqual(snap.u_total or 0.0, 0.49)
        self.assertIn("base", snap.sources)
        self.assertEqual(snap.sources["base"].rssi_dbm, -82.0)
        self.assertEqual(snap.sources["base"].last_heard_ms, 12345)
        self.assertEqual(snap.sources["handheld"].snr_db, -3.5)
        self.assertIsNone(snap.sources["handheld"].last_heard_ms)

    def test_legacy_uppercase_U_keys_still_decode(self) -> None:
        snap = decode_topic_0x10({"U_image": 0.1, "U_telemetry": 0.2, "U_total": 0.3})
        self.assertEqual(snap.u_image, 0.1)
        self.assertEqual(snap.u_telemetry, 0.2)
        self.assertEqual(snap.u_total, 0.3)

    def test_active_source_as_int_decodes_to_name(self) -> None:
        snap = decode_topic_0x10({"active_source": 4})
        self.assertEqual(snap.active_source, "autonomy")

    def test_link_unstable_flag(self) -> None:
        snap = decode_topic_0x10({"active_source": "base", "link_unstable": True})
        self.assertTrue(snap.link_unstable)

    def test_sources_list_form_also_works(self) -> None:
        snap = decode_topic_0x10({
            "sources": [
                {"name": "base", "rssi_dbm": -70},
                {"source": "handheld", "snr_db": 5.0},
                {"rssi_dbm": -99},  # nameless — dropped silently
            ],
        })
        self.assertEqual(set(snap.sources), {"base", "handheld"})
        self.assertEqual(snap.sources["base"].rssi_dbm, -70.0)
        self.assertEqual(snap.sources["handheld"].snr_db, 5.0)

    def test_extras_preserves_unknown_keys(self) -> None:
        snap = decode_topic_0x10({"active_source": "base", "future_field": "xyz", "n": 7})
        self.assertEqual(snap.extras, {"future_field": "xyz", "n": 7})

    def test_garbage_inputs_fail_closed(self) -> None:
        # All of these should return an empty snapshot, never raise.
        for bad in [b"", b"not json at all but more than 1 byte",
                    "plain text", "{not-json", 12345, None]:
            snap = decode_topic_0x10(bad)  # type: ignore[arg-type]
            self.assertIsInstance(snap, SourceActiveSnapshot)
            self.assertIsNone(snap.active_source)
            self.assertEqual(snap.sources, {})

    def test_bool_is_not_treated_as_numeric(self) -> None:
        # JSON doesn't have a separate int type; ensure True/False don't
        # contaminate float fields.
        snap = decode_topic_0x10({"sf_rung": True, "u_image": False})
        self.assertIsNone(snap.sf_rung)
        self.assertIsNone(snap.u_image)


if __name__ == "__main__":
    unittest.main()
