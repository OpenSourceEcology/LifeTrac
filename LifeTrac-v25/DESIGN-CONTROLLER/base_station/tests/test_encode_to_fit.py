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

    def test_aged_oversized_tile_eventually_ships(self):
        # Liveness valve: a tile whose smallest blob exceeds tile_cap must
        # not be starved forever. Drive many P-frames at a tight budget with
        # one tile always larger than the cap; assert it ships within a
        # bounded number of frames (age escalation + over-budget valve).
        cam = camera_service.SyntheticCamera()
        accum = camera_service.FrameAccum()
        n_tiles = camera_service.GRID_W * camera_service.GRID_H
        big_idx = 0

        def _sized(rgb, tx, ty, quality=None, encode_mode=None, is_key=False):
            i = ty * camera_service.GRID_W + tx
            # big tile always 200 B (> any tight tile_cap); others 8 B.
            n = 200 if i == big_idx else 8
            return bytes([tx & 0xFF, ty & 0xFF]) + b"\xAA" * (n - 2)

        shipped_big = False
        with mock.patch.object(camera_service, "_encode_tile", _sized):
            # First frame keyframe to seed last_canvas + tile_last_seq.
            camera_service._build_frame(cam, accum, force_keyframe=True,
                                        byte_budget=120)
            # Then P-frames. The synth changes tiles each frame; the big
            # tile can never fit 120 B, so only the valve ships it.
            escalate = camera_service.TILE_AGE_ESCALATE_FRAMES
            for _ in range(escalate + 5):
                payload = camera_service._build_frame(
                    cam, accum, force_keyframe=False, byte_budget=120)
                parsed = parse_tile_delta_frame(payload)
                if big_idx in parsed.changed_indices:
                    shipped_big = True
                    break
        self.assertTrue(shipped_big,
                        "aged oversized tile was starved forever")

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


class EnvContractTests(unittest.TestCase):
    """Env-var contracts flagged in PR review (2026-07-27).

    Bench boxes edit these by hand, so a typo must degrade to the default
    rather than stop a daemon from starting, and a documented "0 disables"
    must actually disable.
    """

    def test_env_int_defaults_on_garbage(self):
        os.environ["_LT_TEST_BAD"] = "not-an-int"
        try:
            self.assertEqual(
                camera_service._env_int("_LT_TEST_BAD", 40), 40)
        finally:
            del os.environ["_LT_TEST_BAD"]

    def test_env_int_defaults_on_empty_and_blank(self):
        for raw in ("", "   "):
            os.environ["_LT_TEST_EMPTY"] = raw
            try:
                self.assertEqual(
                    camera_service._env_int("_LT_TEST_EMPTY", 7), 7)
            finally:
                del os.environ["_LT_TEST_EMPTY"]

    def test_env_int_clamps(self):
        os.environ["_LT_TEST_CLAMP"] = "-5"
        try:
            self.assertEqual(
                camera_service._env_int("_LT_TEST_CLAMP", 6, lo=1), 1)
            self.assertEqual(
                camera_service._env_int("_LT_TEST_CLAMP", 6, lo=0), 0)
        finally:
            del os.environ["_LT_TEST_CLAMP"]
        os.environ["_LT_TEST_CLAMP2"] = "999"
        try:
            self.assertEqual(
                camera_service._env_int("_LT_TEST_CLAMP2", 6, hi=100), 100)
        finally:
            del os.environ["_LT_TEST_CLAMP2"]

    def test_age_escalate_zero_actually_disables(self):
        # Documented contract: 0 disables. Before the fix, `age > 0` made a
        # value of 0 escalate EVERY aged tile — the opposite of disabled.
        cam = camera_service.SyntheticCamera()
        accum = camera_service.FrameAccum()
        saved = camera_service.TILE_AGE_ESCALATE_FRAMES
        camera_service.TILE_AGE_ESCALATE_FRAMES = 0
        try:
            with mock.patch.object(camera_service, "_encode_tile",
                                   _fixed_tile(20)):
                camera_service._build_frame(cam, accum, force_keyframe=True,
                                            byte_budget=120)
                # Age every tile well past any threshold, then confirm the
                # escalation path is not taken (no crash, budget respected,
                # and the over-budget valve stays shut).
                accum.sweep_seq += 10_000
                with mock.patch.object(camera_service, "_encode_tile",
                                       _fixed_tile(250)):
                    payload = camera_service._build_frame(
                        cam, accum, force_keyframe=False, byte_budget=120)
            # Valve disabled -> no over-budget admission.
            self.assertLessEqual(len(payload), 120)
        finally:
            camera_service.TILE_AGE_ESCALATE_FRAMES = saved


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

    def test_ditto_roundtrip_and_free_quantization(self):
        f = self.lp.pack_ctrl_ditto(0x1234)
        op, args = self.lp.parse_command_frame(f)
        self.assertEqual(op, self.lp.CMD_OP_CTRL_DITTO)
        self.assertEqual(self.lp.parse_ctrl_ditto(args), 0x1234)
        # The 2 B ref must be FREE: a ditto and a bare skip frame must sit
        # in the same LoRa symbol bucket (9..12 B payload incl 8 B hop hdr).
        toa = self.lp.lora_time_on_air_ms
        phy = self.lp.PHY_IMAGE_BW500
        skip_b = 8 + 1
        ditto_b = 8 + len(f)          # hop hdr + magic+opcode+2 B ref
        self.assertLessEqual(ditto_b, 12)
        self.assertEqual(toa(ditto_b, phy), toa(skip_b, phy))

    def test_ditto_only_applies_on_exact_seq_match(self):
        # THE anti-desync contract: anything but an exact match must fall
        # through to the deadman rather than replay a stale command.
        self.assertTrue(self.lp.ditto_applies(7, 7))
        self.assertFalse(self.lp.ditto_applies(7, 6))      # missed a frame
        self.assertFalse(self.lp.ditto_applies(7, 8))      # ahead somehow
        self.assertFalse(self.lp.ditto_applies(7, None))   # nothing applied
        self.assertFalse(self.lp.ditto_applies(None, 7))   # malformed ditto
        # u16 wrap compares on the low 16 bits, both sides.
        self.assertTrue(self.lp.ditto_applies(0x10001, 1))

    def test_malformed_ditto_args_rejected(self):
        for bad in (b"", b"\x01"):
            self.assertIsNone(self.lp.parse_ctrl_ditto(bad))
            self.assertFalse(
                self.lp.ditto_applies(self.lp.parse_ctrl_ditto(bad), 1))

    def test_ditto_saves_exactly_one_symbol_step_when_authenticated(self):
        # Honest accounting for the design doc: with D13 (+12 B) a ditto is
        # 23 B vs a 38 B control frame -> one quantization step (5.12 ms at
        # BW500). Pin it so the doc's number cannot silently drift.
        toa = self.lp.lora_time_on_air_ms
        phy = self.lp.PHY_IMAGE_BW500
        ditto_d13 = toa(8 + 2 + 2 + 12, phy)     # hdr+magic/op+ref+tag
        ctrl_d13 = toa(8 + 2 + 16 + 12, phy)     # hdr+magic/op+ControlFrame+tag
        self.assertAlmostEqual(ctrl_d13 - ditto_d13, 5.12, places=2)

    def test_probe_opcodes_are_distinct_and_known(self):
        self.assertEqual(self.lp.CMD_OP_PROBE, 0x69)
        self.assertEqual(self.lp.CMD_OP_PROBE_ECHO, 0x6A)
        self.assertIn(self.lp.CMD_OP_PROBE, self.lp._CMD_OPS)
        self.assertIn(self.lp.CMD_OP_PROBE_ECHO, self.lp._CMD_OPS)


if __name__ == "__main__":
    unittest.main()
