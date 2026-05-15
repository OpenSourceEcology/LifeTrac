"""W2-03: data-saving measures wired into the encoder hot path.

Pins three behaviours that previously lived only as building blocks:

1. ROI-aware per-tile WebP quality \u2014 :class:`RoiPlanner` inside-tiles get
   :data:`camera_service.ROI_QUALITY_INSIDE`, outside-tiles get
   :data:`ROI_QUALITY_OUTSIDE`.
2. Byte-budget cap on the encoded body \u2014 outside-ROI tiles drop first;
   the bitmap is rebuilt from the kept set so the parser stays in sync
   (verified end-to-end via ``parse_tile_delta_frame``).
3. ``CMD_ROI_HINT`` back-channel opcode forwards into ``RoiPlanner``.

Pure-Python; no PIL needed because ``_encode_tile`` is monkey-patched to
record the per-tile quality and emit a deterministic blob whose size is
controlled by the test.
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

# Load base-station ``frame_format`` directly to dodge the package-name
# collision with the tractor's ``image_pipeline`` package.
_FF_PATH = os.path.join(_BS_DIR, "image_pipeline", "frame_format.py")
_spec = importlib.util.spec_from_file_location("_bs_frame_format", _FF_PATH)
assert _spec is not None and _spec.loader is not None
_bs_frame_format = importlib.util.module_from_spec(_spec)
sys.modules["_bs_frame_format"] = _bs_frame_format
_spec.loader.exec_module(_bs_frame_format)
parse_tile_delta_frame = _bs_frame_format.parse_tile_delta_frame

if _X8_DIR not in sys.path:
    sys.path.insert(0, _X8_DIR)

import camera_service  # noqa: E402
from image_pipeline.roi import RoiPlanner  # noqa: E402


def _fake_encode_tile_factory(blob_size: int = 50,
                              calls: list | None = None):
    """Returns a stand-in for ``camera_service._encode_tile`` that ignores
    the canvas, returns a fixed-size blob, and records ``(tx, ty, quality)``
    so tests can assert per-tile quality decisions."""
    if calls is None:
        calls = []

    def _fake(rgb_canvas, tx, ty, quality=None):
        calls.append((tx, ty, quality))
        # Tile content includes (tx, ty) so different tiles round-trip
        # to distinct blobs through parse_tile_delta_frame.
        body = bytes([tx & 0xFF, ty & 0xFF, (quality or 0) & 0xFF])
        pad_len = max(0, blob_size - len(body))
        return body + b"\xAA" * pad_len

    return _fake, calls


def _decode_indices(payload: bytes) -> list[int]:
    return parse_tile_delta_frame(payload).changed_indices


class EncoderHotPathDefaultsTests(unittest.TestCase):
    """No ROI planner, no byte budget \u2192 every changed tile encoded with
    the global :data:`WEBP_QUALITY`."""

    def test_keyframe_uses_global_quality_for_every_tile(self) -> None:
        cam = camera_service.SyntheticCamera()
        accum = camera_service.FrameAccum()
        fake, calls = _fake_encode_tile_factory(blob_size=20)
        with mock.patch.object(camera_service, "_encode_tile", fake):
            payload = camera_service._build_frame(cam, accum,
                                                  force_keyframe=True)
        n_tiles = camera_service.GRID_W * camera_service.GRID_H
        self.assertEqual(len(calls), n_tiles)
        # Every call must have used the legacy global WEBP_QUALITY since
        # no ROI planner was wired in.
        self.assertTrue(
            all(q == camera_service.WEBP_QUALITY for _tx, _ty, q in calls),
            f"unexpected per-tile qualities: {set(q for _, _, q in calls)}")
        decoded = parse_tile_delta_frame(payload)
        self.assertEqual(decoded.changed_indices, list(range(n_tiles)))


class RoiPerTileQualityTests(unittest.TestCase):
    """A configured :class:`RoiPlanner` must split per-tile quality."""

    def test_inside_and_outside_quality_split(self) -> None:
        # 3-wide ROI in upper-left so we know which indices are inside.
        roi = RoiPlanner(grid_w=camera_service.GRID_W,
                         grid_h=camera_service.GRID_H)
        roi.apply_hint(col_lo=0, col_hi=2, row_lo=0, row_hi=1)
        cam = camera_service.SyntheticCamera()
        accum = camera_service.FrameAccum()
        fake, calls = _fake_encode_tile_factory(blob_size=20)
        with mock.patch.object(camera_service, "_encode_tile", fake):
            payload = camera_service._build_frame(
                cam, accum, force_keyframe=True, roi_planner=roi)

        # Map (tx,ty) \u2192 quality from the recorded calls.
        per_tile = {(tx, ty): q for tx, ty, q in calls}
        # Inside ROI: cols 0..2 \xd7 rows 0..1 \u2192 6 tiles.
        for ty in range(0, 2):
            for tx in range(0, 3):
                self.assertEqual(per_tile[(tx, ty)],
                                 camera_service.ROI_QUALITY_INSIDE,
                                 f"tile ({tx},{ty}) should be inside-ROI quality")
        # Outside ROI tiles should all use the lower quality.
        n_outside = sum(1 for v in per_tile.values()
                        if v == camera_service.ROI_QUALITY_OUTSIDE)
        self.assertEqual(n_outside,
                         camera_service.GRID_W * camera_service.GRID_H - 6)

        # Parser still happy; bitmap unchanged because no budget cap.
        decoded = parse_tile_delta_frame(payload)
        self.assertEqual(len(decoded.tiles),
                         camera_service.GRID_W * camera_service.GRID_H)


class ByteBudgetDropsTests(unittest.TestCase):
    """Tight budget \u2192 outside-ROI tiles dropped first; bitmap is rebuilt
    so the parser stays in sync."""

    def test_outside_tiles_drop_before_inside(self) -> None:
        roi = RoiPlanner(grid_w=camera_service.GRID_W,
                         grid_h=camera_service.GRID_H)
        # Inside-ROI = 6 tiles in upper-left.
        roi.apply_hint(col_lo=0, col_hi=2, row_lo=0, row_hi=1)
        cam = camera_service.SyntheticCamera()
        accum = camera_service.FrameAccum()

        # Each fake tile body is 20 B + 1 B size prefix = 21 B per tile.
        # Budget = 21 * 6 = 126 B \u2192 exactly the 6 inside tiles fit.
        fake, _calls = _fake_encode_tile_factory(blob_size=20)
        with mock.patch.object(camera_service, "_encode_tile", fake):
            payload = camera_service._build_frame(
                cam, accum, force_keyframe=True,
                roi_planner=roi, byte_budget=21 * 6)

        decoded = parse_tile_delta_frame(payload)
        # Indices 0,1,2 (row 0 cols 0\u20132) and 12,13,14 (row 1 cols 0\u20132).
        self.assertEqual(decoded.changed_indices, [0, 1, 2, 12, 13, 14])
        self.assertEqual(len(decoded.tiles), 6)
        # Tile bodies must be in ascending index order regardless of the
        # encoding order used internally.
        self.assertEqual([t.index for t in decoded.tiles], [0, 1, 2, 12, 13, 14])

    def test_budget_above_total_drops_nothing(self) -> None:
        roi = RoiPlanner(grid_w=camera_service.GRID_W,
                         grid_h=camera_service.GRID_H)
        roi.apply_hint(col_lo=0, col_hi=2, row_lo=0, row_hi=1)
        cam = camera_service.SyntheticCamera()
        accum = camera_service.FrameAccum()
        fake, _calls = _fake_encode_tile_factory(blob_size=10)
        with mock.patch.object(camera_service, "_encode_tile", fake):
            payload = camera_service._build_frame(
                cam, accum, force_keyframe=True,
                roi_planner=roi, byte_budget=100_000)
        decoded = parse_tile_delta_frame(payload)
        self.assertEqual(len(decoded.tiles),
                         camera_service.GRID_W * camera_service.GRID_H)

    def test_no_roi_planner_with_budget_keeps_lowest_indices_first(self) -> None:
        cam = camera_service.SyntheticCamera()
        accum = camera_service.FrameAccum()
        fake, _calls = _fake_encode_tile_factory(blob_size=20)
        # Budget for 4 tiles \xd7 21 B = 84 B.
        with mock.patch.object(camera_service, "_encode_tile", fake):
            payload = camera_service._build_frame(
                cam, accum, force_keyframe=True, byte_budget=84)
        decoded = parse_tile_delta_frame(payload)
        self.assertEqual(decoded.changed_indices, [0, 1, 2, 3])


class CmdRoiHintDispatchTests(unittest.TestCase):
    """``CMD_ROI_HINT`` must forward into the planner and force a keyframe."""

    def test_roi_hint_updates_planner_and_forces_keyframe(self) -> None:
        roi = RoiPlanner(grid_w=camera_service.GRID_W,
                         grid_h=camera_service.GRID_H)
        evt = threading.Event()
        # CMD_ROI_HINT payload: topic, opcode, col_lo, col_hi, row_lo, row_hi.
        frame = bytes([camera_service.X8_CMD_TOPIC,
                       camera_service.CMD_ROI_HINT,
                       4, 9, 2, 6])
        camera_service.dispatch_back_channel(frame, evt, roi_planner=roi)
        self.assertTrue(evt.is_set())
        self.assertEqual(roi.current_roi(), (4, 9, 2, 6))

    def test_roi_hint_without_planner_is_silent(self) -> None:
        evt = threading.Event()
        frame = bytes([camera_service.X8_CMD_TOPIC,
                       camera_service.CMD_ROI_HINT,
                       0, 1, 0, 1])
        # No planner \u2192 must not raise; still forces a keyframe.
        camera_service.dispatch_back_channel(frame, evt, roi_planner=None)
        self.assertTrue(evt.is_set())

    def test_roi_hint_too_short_is_ignored(self) -> None:
        roi = RoiPlanner()
        evt = threading.Event()
        frame = bytes([camera_service.X8_CMD_TOPIC,
                       camera_service.CMD_ROI_HINT, 1, 2])  # only 2 of 4 args
        camera_service.dispatch_back_channel(frame, evt, roi_planner=roi)
        # No hint applied \u2192 planner falls back to default-mode ROI.
        self.assertEqual(roi.current_roi()[0:2], (0, 11))  # idle default


class ResolveByteBudgetTests(unittest.TestCase):

    def test_blank_env_returns_none(self) -> None:
        with mock.patch.dict(os.environ, {"LIFETRAC_FRAGMENT_BUDGET": ""},
                             clear=False):
            self.assertIsNone(camera_service._resolve_byte_budget())

    def test_zero_or_negative_returns_none(self) -> None:
        with mock.patch.dict(os.environ, {"LIFETRAC_FRAGMENT_BUDGET": "0"},
                             clear=False):
            self.assertIsNone(camera_service._resolve_byte_budget())
        with mock.patch.dict(os.environ, {"LIFETRAC_FRAGMENT_BUDGET": "-3"},
                             clear=False):
            self.assertIsNone(camera_service._resolve_byte_budget())

    def test_positive_count_returns_byte_cap(self) -> None:
        with mock.patch.dict(os.environ, {"LIFETRAC_FRAGMENT_BUDGET": "4"},
                             clear=False):
            cap = camera_service._resolve_byte_budget()
        self.assertIsNotNone(cap)
        self.assertGreater(cap, 0)


if __name__ == "__main__":
    unittest.main()
