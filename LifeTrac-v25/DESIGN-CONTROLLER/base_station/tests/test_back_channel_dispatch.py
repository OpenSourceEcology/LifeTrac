"""IP-104 follow-up: integration test for the M7 → X8 back-channel.

The back-channel is the path that turns a web-UI ``/cmd/req_keyframe``
into an actual I-frame on the X8 encoder. End-to-end:

  web_ui  ──MQTT──▶  lora_bridge  ──LoRa──▶  M7  ──UART KISS──▶  X8

This suite covers the X8-side dispatcher (``dispatch_back_channel``)
exhaustively over the four opcodes the M7 forwards (``CMD_REQ_KEYFRAME``,
``CMD_CAMERA_SELECT``, ``CMD_CAMERA_QUALITY``, ``CMD_ENCODE_MODE``)
plus malformed / truncated / over-long / wrong-topic frames.

Catches before the bench session:
* missing ``force_key_evt.set()`` on REQ_KEYFRAME
* WEBP quality clamp not engaging on out-of-range args
* wrong-topic frames leaking through (would cause spurious keyframes)
* short frames crashing the dispatcher
"""

from __future__ import annotations

import os
import sys
import threading
import unittest


# Make the X8 service importable from the base_station tests without
# having to install it as a package. (The real X8 image sets PYTHONPATH
# via systemd; tests just adjust sys.path.)
_X8_DIR = os.path.normpath(os.path.join(
    os.path.dirname(__file__), "..", "..", "firmware", "tractor_x8"))
if _X8_DIR not in sys.path:
    sys.path.insert(0, _X8_DIR)

import camera_service  # noqa: E402  (sys.path tweak above)
from camera_service import (  # noqa: E402
    CMD_CAMERA_QUALITY,
    CMD_CAMERA_SELECT,
    CMD_ENCODE_MODE,
    CMD_REQ_KEYFRAME,
    X8_CMD_TOPIC,
    dispatch_back_channel,
)


class BackChannelDispatchTests(unittest.TestCase):

    def setUp(self) -> None:
        self.evt = threading.Event()
        # Snapshot module-level WEBP_QUALITY so we can restore it.
        self._saved_q = camera_service.WEBP_QUALITY

    def tearDown(self) -> None:
        camera_service.WEBP_QUALITY = self._saved_q

    # ---- happy-path opcodes -----------------------------------------

    def test_req_keyframe_sets_event(self) -> None:
        dispatch_back_channel(bytes([X8_CMD_TOPIC, CMD_REQ_KEYFRAME]), self.evt)
        self.assertTrue(self.evt.is_set())

    def test_camera_quality_clamps_into_range(self) -> None:
        for raw, expected in [(0, 20), (10, 20), (20, 20),
                              (55, 55), (100, 100), (200, 100), (255, 100)]:
            self.evt.clear()
            dispatch_back_channel(
                bytes([X8_CMD_TOPIC, CMD_CAMERA_QUALITY, raw]), self.evt)
            self.assertEqual(camera_service.WEBP_QUALITY, expected,
                             f"raw={raw} → {camera_service.WEBP_QUALITY}, want {expected}")
            # IP-104 contract: every reconfig op also forces a keyframe.
            self.assertTrue(self.evt.is_set(),
                            f"CMD_CAMERA_QUALITY raw={raw} did not force keyframe")

    def test_camera_select_forces_keyframe(self) -> None:
        for cam_idx in range(4):
            self.evt.clear()
            dispatch_back_channel(
                bytes([X8_CMD_TOPIC, CMD_CAMERA_SELECT, cam_idx]), self.evt)
            self.assertTrue(self.evt.is_set(),
                            f"CMD_CAMERA_SELECT cam={cam_idx} did not force keyframe")

    def test_encode_mode_forces_keyframe(self) -> None:
        for mode in (0, 1, 2, 0xFF):
            self.evt.clear()
            dispatch_back_channel(
                bytes([X8_CMD_TOPIC, CMD_ENCODE_MODE, mode]), self.evt)
            self.assertTrue(self.evt.is_set())

    # ---- malformed inputs -------------------------------------------

    def test_wrong_topic_byte_is_ignored(self) -> None:
        for bad_topic in (0x00, 0x01, 0xBF, 0xC1, 0xFF):
            dispatch_back_channel(
                bytes([bad_topic, CMD_REQ_KEYFRAME]), self.evt)
            self.assertFalse(self.evt.is_set(),
                             f"topic 0x{bad_topic:02x} leaked through")

    def test_too_short_frames_do_not_crash(self) -> None:
        for short in (b"", b"\xc0"):
            dispatch_back_channel(short, self.evt)
            self.assertFalse(self.evt.is_set())

    def test_unknown_opcode_is_silent(self) -> None:
        for bad_op in (0x00, 0x05, 0x10, 0x55, 0x90, 0xFE):
            self.evt.clear()
            dispatch_back_channel(
                bytes([X8_CMD_TOPIC, bad_op, 0xAA, 0xBB]), self.evt)
            self.assertFalse(self.evt.is_set(),
                             f"unknown opcode 0x{bad_op:02x} unexpectedly forced keyframe")

    def test_camera_quality_without_arg_is_ignored(self) -> None:
        # No arg byte → we leave WEBP_QUALITY untouched and do not force
        # a keyframe (no reconfig actually happened).
        before = camera_service.WEBP_QUALITY
        dispatch_back_channel(
            bytes([X8_CMD_TOPIC, CMD_CAMERA_QUALITY]), self.evt)
        self.assertEqual(camera_service.WEBP_QUALITY, before)

    def test_oversize_quality_payload_uses_first_arg_byte_only(self) -> None:
        dispatch_back_channel(
            bytes([X8_CMD_TOPIC, CMD_CAMERA_QUALITY, 75, 0xDE, 0xAD]), self.evt)
        self.assertEqual(camera_service.WEBP_QUALITY, 75)

    def test_repeated_req_keyframe_is_idempotent(self) -> None:
        for _ in range(5):
            dispatch_back_channel(
                bytes([X8_CMD_TOPIC, CMD_REQ_KEYFRAME]), self.evt)
        self.assertTrue(self.evt.is_set())


if __name__ == "__main__":
    unittest.main()
