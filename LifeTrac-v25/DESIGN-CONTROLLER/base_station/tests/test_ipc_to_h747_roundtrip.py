"""W2-02 follow-on: end-to-end pipe test for the X8->M7->base_station chain.

Covers two gaps that previously had no test:

1. ``ipc_to_h747.decode_one_ipc_frame`` / ``iter_ipc_frames`` (the Python
   counterpart to the C++ decode the M7 firmware does on its UART RX
   path). Driven directly with crafted byte buffers including leading
   junk, a deliberately bad-CRC frame, and a truncated tail.

2. The whole encode pipe: ``camera_service._build_frame`` (with
   :class:`SyntheticCamera`) -> ``encode_ipc_frame`` -> bytes-on-the-wire
   -> ``iter_ipc_frames`` -> ``frame_format.parse_tile_delta_frame``.
   The keyframe assertion pins that all 96 tiles ride through with their
   WebP blobs intact.

The integration test is skipped when Pillow isn't installed so this file
runs cleanly on a host without the WebP encoder.
"""

from __future__ import annotations

import importlib.util
import os
import sys
import unittest


_HERE = os.path.dirname(__file__)
_X8_DIR = os.path.normpath(os.path.join(
    _HERE, "..", "..", "firmware", "tractor_x8"))
_BS_DIR = os.path.normpath(os.path.join(_HERE, ".."))

# Both the tractor and the base station ship a package named
# ``image_pipeline`` — putting them both on sys.path would let one mask
# the other. Load the base-station ``frame_format`` directly from its
# file so the parser is unambiguous, then put the tractor side on
# sys.path for ``import image_pipeline.ipc_to_h747`` and ``camera_service``.
_FF_PATH = os.path.join(_BS_DIR, "image_pipeline", "frame_format.py")
_spec = importlib.util.spec_from_file_location("_bs_frame_format", _FF_PATH)
assert _spec is not None and _spec.loader is not None
_bs_frame_format = importlib.util.module_from_spec(_spec)
# Register before exec so @dataclass (Py 3.14) can resolve cls.__module__.
sys.modules["_bs_frame_format"] = _bs_frame_format
_spec.loader.exec_module(_bs_frame_format)
parse_tile_delta_frame = _bs_frame_format.parse_tile_delta_frame

if _X8_DIR not in sys.path:
    sys.path.insert(0, _X8_DIR)

# Tractor-side modules.
from image_pipeline.ipc_to_h747 import (  # noqa: E402
    FLAG_MOTION,
    FLAG_WIREFRAME,
    FRAME_MARKER,
    IpcFrameError,
    crc8_smbus,
    decode_one_ipc_frame,
    encode_ipc_frame,
    iter_ipc_frames,
)


class IpcFrameDecoderTests(unittest.TestCase):
    """Direct decoder coverage — no PIL, no camera, just bytes."""

    def test_round_trip_basic_payload(self) -> None:
        wire = encode_ipc_frame(b"hello world", is_keyframe=True)
        frame, consumed = decode_one_ipc_frame(wire)
        self.assertIsNotNone(frame)
        assert frame is not None  # for type checker
        self.assertEqual(consumed, len(wire))
        self.assertEqual(frame.payload, b"hello world")
        self.assertTrue(frame.is_keyframe)
        self.assertFalse(frame.is_motion)
        self.assertFalse(frame.is_wireframe)

    def test_motion_and_wireframe_flag_round_trip(self) -> None:
        wire = encode_ipc_frame(b"\x00\x01\x02",
                                is_motion=True, is_wireframe=True)
        frame, _ = decode_one_ipc_frame(wire)
        assert frame is not None
        self.assertFalse(frame.is_keyframe)
        self.assertTrue(frame.is_motion)
        self.assertTrue(frame.is_wireframe)
        self.assertEqual(frame.flags, FLAG_MOTION | FLAG_WIREFRAME)

    def test_empty_payload_round_trips(self) -> None:
        wire = encode_ipc_frame(b"", is_keyframe=True)
        frame, consumed = decode_one_ipc_frame(wire)
        assert frame is not None
        self.assertEqual(frame.payload, b"")
        self.assertEqual(consumed, len(wire))

    def test_partial_buffer_returns_none(self) -> None:
        wire = encode_ipc_frame(b"abcdef")
        # Truncate before the trailing CRC.
        frame, consumed = decode_one_ipc_frame(wire[:-1])
        self.assertIsNone(frame)
        self.assertEqual(consumed, 0)

    def test_partial_header_returns_none(self) -> None:
        # Just the marker.
        frame, consumed = decode_one_ipc_frame(bytes([FRAME_MARKER]))
        self.assertIsNone(frame)
        self.assertEqual(consumed, 0)

    def test_leading_junk_is_skipped(self) -> None:
        wire = encode_ipc_frame(b"payload", is_keyframe=True)
        framed = b"\x00\x11\x22" + wire
        frame, consumed = decode_one_ipc_frame(framed)
        assert frame is not None
        self.assertEqual(frame.payload, b"payload")
        self.assertEqual(consumed, len(framed))

    def test_bad_crc_raises(self) -> None:
        wire = bytearray(encode_ipc_frame(b"abc", is_keyframe=True))
        wire[-1] ^= 0xFF  # corrupt the CRC
        with self.assertRaises(IpcFrameError):
            decode_one_ipc_frame(bytes(wire))

    def test_iter_skips_bad_crc_frame_and_keeps_going(self) -> None:
        good1 = encode_ipc_frame(b"first", is_keyframe=True)
        bad = bytearray(encode_ipc_frame(b"poisoned"))
        bad[-1] ^= 0x5A  # break CRC
        good2 = encode_ipc_frame(b"second", is_motion=True)
        # Sprinkle junk between frames to verify resync each time.
        stream = b"\x99" + good1 + b"\x00\x00" + bytes(bad) + good2 + b"\x77"
        frames = list(iter_ipc_frames(stream))
        self.assertEqual(len(frames), 2)
        self.assertEqual(frames[0].payload, b"first")
        self.assertTrue(frames[0].is_keyframe)
        self.assertEqual(frames[1].payload, b"second")
        self.assertTrue(frames[1].is_motion)

    def test_iter_stops_on_truncated_tail(self) -> None:
        full = encode_ipc_frame(b"alpha", is_keyframe=True)
        partial = encode_ipc_frame(b"omega")[:-2]
        stream = full + partial
        frames = list(iter_ipc_frames(stream))
        self.assertEqual(len(frames), 1)
        self.assertEqual(frames[0].payload, b"alpha")

    def test_crc_is_smbus_polynomial(self) -> None:
        # Sanity check the CRC-8/SMBUS implementation against a known
        # vector: CRC-8/SMBUS over "123456789" == 0xF4.
        self.assertEqual(crc8_smbus(b"123456789"), 0xF4)


def _pil_available() -> bool:
    try:
        import PIL  # noqa: F401
        return True
    except ImportError:
        return False


@unittest.skipUnless(_pil_available(),
                     "Pillow not installed; skipping full encode pipe test")
class CameraToParseEndToEndTests(unittest.TestCase):
    """Full pipe: SyntheticCamera -> _build_frame -> encode_ipc_frame ->
    iter_ipc_frames -> parse_tile_delta_frame."""

    def test_keyframe_round_trips_through_ipc_framing(self) -> None:
        import camera_service  # imported lazily so the module-import side
                                # effects don't run unless we need them
        cam = camera_service.SyntheticCamera()
        accum = camera_service.FrameAccum()

        # First call with force_keyframe=True -> every tile encoded.
        payload = camera_service._build_frame(cam, accum, force_keyframe=True)
        self.assertGreater(len(payload), 0)

        # Wrap exactly the way camera_service.main() would.
        wire = encode_ipc_frame(payload, is_keyframe=True)

        # Sandwich with junk to prove the decoder resyncs on the marker.
        stream = b"\xde\xad\xbe\xef" + wire + b"\x00\x00"
        frames = list(iter_ipc_frames(stream))
        self.assertEqual(len(frames), 1)
        framed = frames[0]
        self.assertTrue(framed.is_keyframe)
        self.assertEqual(framed.payload, payload)

        decoded = parse_tile_delta_frame(framed.payload)
        self.assertTrue(decoded.is_keyframe)
        self.assertEqual(decoded.grid_w, camera_service.GRID_W)
        self.assertEqual(decoded.grid_h, camera_service.GRID_H)
        self.assertEqual(decoded.tile_px, camera_service.TILE_PX)
        # Keyframe -> all 96 tiles present, in row-major order.
        n_tiles = camera_service.GRID_W * camera_service.GRID_H
        self.assertEqual(decoded.changed_indices, list(range(n_tiles)))
        self.assertEqual(len(decoded.tiles), n_tiles)
        # Each tile blob must be a non-empty, <=256 B WebP-ish payload.
        for tile in decoded.tiles:
            self.assertGreater(len(tile.blob), 0)
            self.assertLessEqual(len(tile.blob), 256)

    def test_two_frames_in_one_buffer_decode_in_order(self) -> None:
        import camera_service
        cam = camera_service.SyntheticCamera()
        accum = camera_service.FrameAccum()

        p1 = camera_service._build_frame(cam, accum, force_keyframe=True)
        p2 = camera_service._build_frame(cam, accum, force_keyframe=False)
        stream = (encode_ipc_frame(p1, is_keyframe=True)
                  + encode_ipc_frame(p2))

        frames = list(iter_ipc_frames(stream))
        self.assertEqual(len(frames), 2)
        self.assertTrue(frames[0].is_keyframe)
        self.assertFalse(frames[1].is_keyframe)
        self.assertEqual(frames[0].payload, p1)
        self.assertEqual(frames[1].payload, p2)


if __name__ == "__main__":
    unittest.main()
