"""Encode-to-fit rate control (2026-07-27).

Pins the three encode-to-fit fixes so the single-fragment TDMA schedule can
rely on them:

1. BUDGET EXACTNESS — a byte_budget of N yields a WIRE payload (header +
   bitmap + tile bodies) of at most N bytes, so a "243 B" frame is truly one
   fragment. Before this the 6 B header + bitmap were charged OUTSIDE the
   cap (261 B for a "243" budget = 2 fragments), and a single oversized
   first tile could bust any cap via the `and kept` bypass.
2. QUALITY-ONLY CHANGE — does NOT force a keyframe (quality is not on the
   wire; tiles are self-describing). A MODE change still does.
3. PROBE OPCODE — CMD_OP_PROBE / CMD_OP_PROBE_ECHO pack + parse round-trip.

Uses the same import shim + _encode_tile monkeypatch idiom as
test_data_saving_measures.py so no PIL is needed.
"""

from __future__ import annotations

import importlib.util
import os
import sys
import threading
import unittest
from unittest import mock


_HERE = os.path.dirname(__file__)
_X8_DIR = os.path.normpath(os.path.join(
    _HERE, "..", "..", "firmware", "tractor_x8"))
_BS_DIR = os.path.normpath(os.path.join(_HERE, ".."))

_FF_PATH = os.path.join(_BS_DIR, "image_pipeline", "frame_format.py")
_spec = importlib.util.spec_from_file_location("_bs_frame_format_e2f", _FF_PATH)
assert _spec is not None and _spec.loader is not None
_bs_frame_format = importlib.util.module_from_spec(_spec)
sys.modules["_bs_frame_format_e2f"] = _bs_frame_format
_spec.loader.exec_module(_bs_frame_format)
parse_tile_delta_frame = _bs_frame_format.parse_tile_delta_frame

if _X8_DIR not in sys.path:
    sys.path.insert(0, _X8_DIR)

import camera_service  # noqa: E402


def _fixed_tile(blob_size: int):
    """_encode_tile stand-in returning a fixed-size blob (no PIL)."""
    def _fake(rgb_canvas, tx, ty, quality=None, encode_mode=None, is_key=False):
        body = bytes([tx & 0xFF, ty & 0xFF, (quality or 0) & 0xFF])
        return body + b"\xAA" * max(0, blob_size - len(body))
    return _fake


class BudgetExactnessTests(unittest.TestCase):
    """The wire payload must never exceed byte_budget (header+bitmap counted)."""

    def _wire_len_for(self, budget, blob_size):
        cam = camera_service.SyntheticCamera()
        accum = camera_service.FrameAccum()
        with mock.patch.object(camera_service, "_encode_tile",
                               _fixed_tile(blob_size)):
            payload = camera_service._build_frame(
                cam, accum, force_keyframe=True, byte_budget=budget)
        return payload

    def test_wire_payload_within_budget_small(self):
        # A 243 B budget must produce <= 243 B on the wire, INCLUDING the
        # 6 B header + changed bitmap — the exact single-fragment property.
        for budget in (60, 120, 243, 486):
            payload = self._wire_len_for(budget, blob_size=26)
            self.assertLessEqual(
                len(payload), budget,
                f"budget={budget}: wire payload {len(payload)} B exceeds cap")

    def test_oversized_first_tile_cannot_bust_cap(self):
        # A single tile larger than the whole budget must NOT be admitted
        # (the removed `and kept` bypass used to ship it anyway).
        payload = self._wire_len_for(budget=60, blob_size=250)
        self.assertLessEqual(len(payload), 60)
        # It parses cleanly with zero or few tiles, not a 250 B body.
        parsed = parse_tile_delta_frame(payload)
        self.assertIsNotNone(parsed)

    def test_greedy_continue_fits_smaller_lower_priority_tiles(self):
        # With uniform tile sizes the packer fills the budget densely;
        # assert it keeps more than one tile when the budget allows several.
        payload = self._wire_len_for(budget=243, blob_size=20)
        parsed = parse_tile_delta_frame(payload)
        self.assertGreaterEqual(len(parsed.changed_indices), 2)
        self.assertLessEqual(len(payload), 243)

    def test_no_budget_is_unbounded(self):
        # byte_budget=None keeps the legacy "encode everything" behaviour.
        cam = camera_service.SyntheticCamera()
        accum = camera_service.FrameAccum()
        with mock.patch.object(camera_service, "_encode_tile",
                               _fixed_tile(20)):
            payload = camera_service._build_frame(
                cam, accum, force_keyframe=True, byte_budget=None)
        n_tiles = camera_service.GRID_W * camera_service.GRID_H
        parsed = parse_tile_delta_frame(payload)
        self.assertEqual(len(parsed.changed_indices), n_tiles)


class QualityKeyframeTests(unittest.TestCase):
    """Quality-only changes must not force a keyframe; mode changes must."""

    def setUp(self):
        self._saved_mode = camera_service.ENCODE_MODE
        self._saved_q = camera_service.WEBP_QUALITY
        self._saved_client = camera_service._MQTT_CLIENT
        camera_service._MQTT_CLIENT = None   # skip the ack publish path

    def tearDown(self):
        camera_service.ENCODE_MODE = self._saved_mode
        camera_service.WEBP_QUALITY = self._saved_q
        camera_service._MQTT_CLIENT = self._saved_client

    def test_quality_only_change_does_not_force_keyframe(self):
        camera_service.ENCODE_MODE = camera_service.ENCODE_MODE_Y_ONLY
        evt = threading.Event()
        camera_service._apply_encode_mode(
            camera_service.ENCODE_MODE_Y_ONLY, "lora_cmd", evt, quality=40)
        self.assertFalse(evt.is_set(),
                         "quality-only change wrongly forced a keyframe")

    def test_mode_change_forces_keyframe(self):
        camera_service.ENCODE_MODE = camera_service.ENCODE_MODE_FULL
        evt = threading.Event()
        camera_service._apply_encode_mode(
            camera_service.ENCODE_MODE_Y_ONLY, "lora_cmd", evt)
        self.assertTrue(evt.is_set(),
                        "mode change should force a keyframe")

    def test_boot_forces_keyframe(self):
        evt = threading.Event()
        camera_service._apply_encode_mode(
            camera_service.ENCODE_MODE, "boot", evt)
        self.assertTrue(evt.is_set(), "boot should force an initial keyframe")

    def test_quality_applied_even_without_keyframe(self):
        camera_service.ENCODE_MODE = camera_service.ENCODE_MODE_Y_ONLY
        evt = threading.Event()
        camera_service._apply_encode_mode(
            camera_service.ENCODE_MODE_Y_ONLY, "lora_cmd", evt, quality=44)
        self.assertEqual(camera_service.WEBP_QUALITY, 44)


class ProbeOpcodeTests(unittest.TestCase):
    """CMD_OP_PROBE / CMD_OP_PROBE_ECHO pack + parse round-trip."""

    def setUp(self):
        import importlib.util as _ilu
        lp_path = os.path.join(_BS_DIR, "lora_proto.py")
        spec = _ilu.spec_from_file_location("_lp_probe", lp_path)
        self.lp = _ilu.module_from_spec(spec)
        sys.modules["_lp_probe"] = self.lp
        spec.loader.exec_module(self.lp)

    def test_probe_roundtrip(self):
        seq = 0x01020304
        phase = 80
        args = seq.to_bytes(4, "little") + phase.to_bytes(2, "little")
        frame = self.lp.pack_command_frame(self.lp.CMD_OP_PROBE, args)
        op, out = self.lp.parse_command_frame(frame)
        self.assertEqual(op, self.lp.CMD_OP_PROBE)
        self.assertEqual(int.from_bytes(out[0:4], "little"), seq)
        self.assertEqual(int.from_bytes(out[4:6], "little"), phase)

    def test_probe_echo_roundtrip(self):
        seq = 0xAABBCCDD
        frame = self.lp.pack_command_frame(
            self.lp.CMD_OP_PROBE_ECHO, seq.to_bytes(4, "little"))
        op, out = self.lp.parse_command_frame(frame)
        self.assertEqual(op, self.lp.CMD_OP_PROBE_ECHO)
        self.assertEqual(int.from_bytes(out[0:4], "little"), seq)

    def test_probe_opcodes_are_distinct_and_known(self):
        self.assertEqual(self.lp.CMD_OP_PROBE, 0x69)
        self.assertEqual(self.lp.CMD_OP_PROBE_ECHO, 0x6A)
        self.assertIn(self.lp.CMD_OP_PROBE, self.lp._CMD_OPS)
        self.assertIn(self.lp.CMD_OP_PROBE_ECHO, self.lp._CMD_OPS)


if __name__ == "__main__":
    unittest.main()
