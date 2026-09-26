"""ENCODE_MODE (0x63) ack-matching + pending-command supersede semantics.

Guards the RS-1.5 convergence contract after the 2026-07-26 control-plane
fix batch. Two failure modes are covered, both of which shipped as real
bugs and were caught in review:

  1. ANY 0x68 cleared the pending 0x63 — so the stale retained ack a
     tractor re-radiates on MQTT reconnect "confirmed" a command that
     never landed.
  2. Matching on the mode byte alone — so an in-flight ack from the
     PREVIOUS (same mode, old quality) command confirmed a new
     quality-only change, silently dropping it.

The class methods under test are pure, so no radio/broker is needed.
"""
from __future__ import annotations

import os
import sys
import types
import unittest

_HERE = os.path.dirname(os.path.abspath(__file__))
_BASE = os.path.abspath(os.path.join(_HERE, ".."))
for _p in (_BASE, os.path.join(_BASE, "..", "firmware", "x8_lora_bootloader_helper")):
    _p = os.path.abspath(_p)
    if _p not in sys.path:
        sys.path.insert(0, _p)

from image_rx_daemon import ImageRxDaemon  # noqa: E402
from lora_proto import (  # noqa: E402
    CMD_OP_ENCODE_MODE,
    COMMAND_FRAME_MAGIC,
    EncodeMode,
    pack_command_frame,
)


def _ack(**fields) -> bytes:
    import json
    return json.dumps(fields).encode("utf-8")


class ParseEncodeAckTests(unittest.TestCase):
    def test_parses_requested_and_quality(self):
        got = ImageRxDaemon._parse_encode_ack(
            _ack(requested=1, effective=1, quality=55))
        self.assertEqual(got, {"mode": 1, "quality": 55})

    def test_camera_service_ack_shape_is_understood(self):
        # Exact field set camera_service._apply_encode_mode publishes.
        got = ImageRxDaemon._parse_encode_ack(_ack(
            requested=4, effective=1, effective_name="y_only", clamped=True,
            codec=4, quality=40, source="lora_cmd", ts=123.4))
        self.assertEqual(got["mode"], 4)      # requested wins over effective
        self.assertEqual(got["quality"], 40)

    def test_missing_quality_is_none_not_an_error(self):
        got = ImageRxDaemon._parse_encode_ack(_ack(requested=0))
        self.assertEqual(got, {"mode": 0, "quality": None})

    def test_unparseable_bodies_return_none(self):
        for bad in (b"", b"not json", b"[]", b'{"no_mode": 1}',
                    b"\xff\xfe\x00", _ack(requested="one")):
            self.assertIsNone(ImageRxDaemon._parse_encode_ack(bad), bad)


class AckMatchesBodyTests(unittest.TestCase):
    def _body(self, mode: int, quality: int | None = None) -> bytes:
        args = bytes([mode]) if quality is None else bytes([mode, quality])
        return pack_command_frame(CMD_OP_ENCODE_MODE, args)

    def test_mode_only_command_matches_on_mode(self):
        body = self._body(2)
        self.assertEqual(body[:2], bytes([COMMAND_FRAME_MAGIC, 0x63]))
        self.assertTrue(ImageRxDaemon._ack_matches_body(
            {"mode": 2, "quality": 55}, body))

    def test_mode_only_command_ignores_quality_differences(self):
        # We did not command a quality, so whatever the tractor reports
        # must not block convergence.
        body = self._body(2)
        for q in (20, 55, 100, None):
            self.assertTrue(ImageRxDaemon._ack_matches_body(
                {"mode": 2, "quality": q}, body))

    def test_wrong_mode_never_matches(self):
        body = self._body(1)
        self.assertFalse(ImageRxDaemon._ack_matches_body(
            {"mode": 6, "quality": 55}, body))

    def test_quality_command_requires_matching_quality(self):
        body = self._body(0, 80)
        self.assertTrue(ImageRxDaemon._ack_matches_body(
            {"mode": 0, "quality": 80}, body))

    def test_stale_ack_for_previous_quality_does_not_match(self):
        # The regression this test exists for: operator sets q=50, then
        # q=80; the in-flight q=50 ack must NOT confirm the q=80 command.
        body = self._body(0, 80)
        self.assertFalse(ImageRxDaemon._ack_matches_body(
            {"mode": 0, "quality": 50}, body))

    def test_quality_compared_against_tractor_clamp(self):
        # camera_service clamps into [20, 100] and echoes the CLAMPED
        # value, so the base must clamp identically or a legal low
        # request would never converge.
        body = self._body(1, 5)
        self.assertTrue(ImageRxDaemon._ack_matches_body(
            {"mode": 1, "quality": ImageRxDaemon.TRACTOR_QUALITY_MIN}, body))
        self.assertFalse(ImageRxDaemon._ack_matches_body(
            {"mode": 1, "quality": 5}, body))

    def test_vector_quality_uses_the_1_to_100_clamp(self):
        # VECTOR (9) takes the whole byte: bands 60-100/40-59/20-39/1-19 are
        # the V0..V3 levels (VECTOR_SCENE.md s4.5.4), and camera_service lifts
        # its 20 floor for that mode; the base must clamp identically or a
        # V3 request (1-19) would retry until cool-down.
        body = self._body(int(EncodeMode.VECTOR), 12)
        self.assertTrue(ImageRxDaemon._ack_matches_body(
            {"mode": int(EncodeMode.VECTOR), "quality": 12}, body))
        self.assertFalse(ImageRxDaemon._ack_matches_body(
            {"mode": int(EncodeMode.VECTOR), "quality": 20}, body))
        # tile modes keep the 20 floor
        self.assertFalse(ImageRxDaemon._ack_matches_body(
            {"mode": 1, "quality": 12}, self._body(1, 12)))

    def test_old_tractor_without_quality_field_does_not_confirm(self):
        # A build that cannot report quality cannot prove it applied one;
        # keep retrying (and eventually GAVE UP) rather than claim it did.
        body = self._body(1, 70)
        self.assertFalse(ImageRxDaemon._ack_matches_body(
            {"mode": 1, "quality": None}, body))

    def test_truncated_body_is_never_confirmed(self):
        for body in (b"", bytes([COMMAND_FRAME_MAGIC]),
                     bytes([COMMAND_FRAME_MAGIC, 0x63])):
            self.assertFalse(ImageRxDaemon._ack_matches_body(
                {"mode": 0, "quality": 55}, body))


class SetPendingSupersedeTests(unittest.TestCase):
    """_set_pending must replace a superseded body, not discard it."""

    def setUp(self):
        self.d = ImageRxDaemon.__new__(ImageRxDaemon)      # no I/O in __init__
        import threading
        self.d._lock = threading.Lock()
        self.d._pending_cmds = {}

    def _pending(self):
        return self.d._pending_cmds[CMD_OP_ENCODE_MODE]

    def test_identical_retrigger_keeps_attempt_budget(self):
        body = pack_command_frame(CMD_OP_ENCODE_MODE, bytes([1]))
        self.d._set_pending(CMD_OP_ENCODE_MODE, body)
        self._pending()["attempts"] = 7
        self.d._set_pending(CMD_OP_ENCODE_MODE, body)
        self.assertEqual(self._pending()["attempts"], 7)
        self.assertEqual(self._pending()["body"], body)

    def test_changed_body_replaces_and_resets_budget(self):
        first = pack_command_frame(CMD_OP_ENCODE_MODE, bytes([1]))
        second = pack_command_frame(CMD_OP_ENCODE_MODE, bytes([6]))
        self.d._set_pending(CMD_OP_ENCODE_MODE, first)
        self._pending()["attempts"] = 12
        self.d._set_pending(CMD_OP_ENCODE_MODE, second)
        self.assertEqual(self._pending()["body"], second)
        self.assertEqual(self._pending()["attempts"], 0)

    def test_quality_only_change_is_a_new_command(self):
        q40 = pack_command_frame(CMD_OP_ENCODE_MODE, bytes([0, 40]))
        q80 = pack_command_frame(CMD_OP_ENCODE_MODE, bytes([0, 80]))
        self.d._set_pending(CMD_OP_ENCODE_MODE, q40)
        self.d._set_pending(CMD_OP_ENCODE_MODE, q80)
        self.assertEqual(self._pending()["body"], q80)
        self.assertEqual(self._pending()["attempts"], 0)


class EncodeModeOverrideAcceptsTractorModesTests(unittest.TestCase):
    """control/encode_mode_override must put every tractor-implemented
    mode on the air. RAWSTREAM (8) used to die here: EncodeMode stopped
    at ADAPTIVE (7), so the daemon logged "outside EncodeMode" and the
    operator could only reach the mode through tractor env vars."""

    def setUp(self):
        self.d = ImageRxDaemon.__new__(ImageRxDaemon)      # no I/O in __init__
        import threading
        self.d._lock = threading.Lock()
        self.d._pending_cmds = {}

    def _override(self, **body):
        import json
        self.d._pending_cmds.clear()
        msg = types.SimpleNamespace(payload=json.dumps(body).encode("utf-8"))
        self.d._on_encode_mode_msg(None, None, msg)
        cur = self.d._pending_cmds.get(CMD_OP_ENCODE_MODE)
        return None if cur is None else cur["body"]

    def test_rawstream_by_name_and_by_value_goes_on_the_air(self):
        # Wire byte 8 == camera_service.ENCODE_MODE_RAWSTREAM (parity is
        # pinned by test_encode_mode_parity_sil).
        want = pack_command_frame(CMD_OP_ENCODE_MODE, bytes([8]))
        for requested in ("rawstream", "RAWSTREAM", 8):
            self.assertEqual(self._override(mode=requested), want, requested)

    def test_rawstream_ack_from_tractor_converges(self):
        # Exact ack shape camera_service._apply_encode_mode publishes for
        # an implemented mode: requested == effective, clamped False.
        body = self._override(mode="rawstream", quality=55)
        ack = ImageRxDaemon._parse_encode_ack(_ack(
            requested=8, effective=8, effective_name="rawstream",
            clamped=False, codec=5, quality=55, source="back_channel",
            ts=1.0))
        self.assertTrue(ImageRxDaemon._ack_matches_body(ack, body))

    def test_vector_by_name_and_by_value_goes_on_the_air(self):
        # Wire byte 9 == camera_service.ENCODE_MODE_VECTOR (parity is pinned
        # by test_encode_mode_parity_sil).
        want = pack_command_frame(CMD_OP_ENCODE_MODE, bytes([9]))
        for requested in ("vector", "VECTOR", 9):
            self.assertEqual(self._override(mode=requested), want, requested)

    def test_vector_v3_ack_from_tractor_converges(self):
        # A V3 request (quality 12) is acked unclamped by a mode-9 tractor.
        body = self._override(mode="vector", quality=12)
        ack = ImageRxDaemon._parse_encode_ack(_ack(
            requested=9, effective=9, effective_name="vector",
            clamped=False, codec=6, quality=12, source="back_channel",
            ts=1.0))
        self.assertTrue(ImageRxDaemon._ack_matches_body(ack, body))

    def test_values_outside_encode_mode_are_still_rejected(self):
        for bad in (int(max(EncodeMode)) + 1, 99, 255):
            self.assertIsNone(self._override(mode=bad), bad)


if __name__ == "__main__":
    unittest.main()
