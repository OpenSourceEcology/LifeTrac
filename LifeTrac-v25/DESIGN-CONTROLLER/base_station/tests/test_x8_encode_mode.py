"""IP-W2-07 unit tests: X8 honours ``CMD_ENCODE_MODE`` (opcode 0x63).

Until W2-07 the X8 dispatcher logged ``CMD_ENCODE_MODE`` and force-keyframed
but didn't actually change the encode strategy, so the base-station's
auto-fallback ladder (``EncodeModeController``) had no measurable effect on
the wire. These tests pin the new behaviour: dispatch mutates the
``camera_service.ENCODE_MODE`` global; ``_encode_tile`` strips chroma for
modes 1/2/3 and applies a per-mode quality ceiling; the W2-05 tile encode
cache key includes the mode byte so a flip invalidates per-tile entries.

The PIL dependency is gated to a single test class so the rest of the
suite can run on stripped Python images.
"""

from __future__ import annotations

import importlib.util
import os
import sys
import threading
import unittest


_THIS_DIR = os.path.dirname(os.path.abspath(__file__))
_X8_DIR = os.path.normpath(os.path.join(
    _THIS_DIR, "..", "..", "firmware", "tractor_x8"))
if _X8_DIR not in sys.path:
    sys.path.insert(0, _X8_DIR)

# Use a unique sys.modules key so we don't collide with the
# base_station.image_pipeline package that ships under the same name.
_SPEC = importlib.util.spec_from_file_location(
    "x8_camera_service_w2_07",
    os.path.join(_X8_DIR, "camera_service.py"))
camera_service = importlib.util.module_from_spec(_SPEC)
sys.modules[_SPEC.name] = camera_service
_SPEC.loader.exec_module(camera_service)  # type: ignore[union-attr]


class EncodeModeDispatchTests(unittest.TestCase):
    """The dispatcher mutates ``ENCODE_MODE`` and force-keyframes."""

    def setUp(self) -> None:
        self._saved = camera_service.ENCODE_MODE
        self.evt = threading.Event()

    def tearDown(self) -> None:
        camera_service.ENCODE_MODE = self._saved

    def test_valid_modes_update_global_and_force_keyframe(self) -> None:
        for mode, name in enumerate(camera_service.ENCODE_MODE_NAMES):
            self.evt.clear()
            camera_service.dispatch_back_channel(
                bytes([camera_service.X8_CMD_TOPIC,
                       camera_service.CMD_ENCODE_MODE,
                       mode]),
                self.evt)
            self.assertEqual(camera_service.ENCODE_MODE, mode,
                             f"mode {mode} ({name}) not committed")
            self.assertTrue(self.evt.is_set(),
                            f"mode {mode} ({name}) did not force keyframe")

    def test_out_of_range_mode_rejected_but_keyframe_still_forced(self) -> None:
        # Operator-visible: a malformed mode shouldn't silently change
        # the encoder, but we still want a fresh keyframe so the bridge
        # surfaces "I heard the request" instantly.
        camera_service.ENCODE_MODE = camera_service.ENCODE_MODE_FULL
        for bad in (4, 7, 99, 0xFF):
            self.evt.clear()
            camera_service.dispatch_back_channel(
                bytes([camera_service.X8_CMD_TOPIC,
                       camera_service.CMD_ENCODE_MODE,
                       bad]),
                self.evt)
            self.assertEqual(camera_service.ENCODE_MODE,
                             camera_service.ENCODE_MODE_FULL,
                             f"bad mode {bad} mutated ENCODE_MODE")
            self.assertTrue(self.evt.is_set(),
                            f"bad mode {bad} did not force keyframe")

    def test_truncated_frame_does_not_crash(self) -> None:
        # Frame missing the mode byte: dispatcher must ignore.
        camera_service.ENCODE_MODE = camera_service.ENCODE_MODE_Y_ONLY
        camera_service.dispatch_back_channel(
            bytes([camera_service.X8_CMD_TOPIC,
                   camera_service.CMD_ENCODE_MODE]),
            self.evt)
        # Mode unchanged; force_evt still set per the IP-104 contract.
        self.assertEqual(camera_service.ENCODE_MODE,
                         camera_service.ENCODE_MODE_Y_ONLY)
        self.assertTrue(self.evt.is_set())


class EncodeModeQualityCeilingTests(unittest.TestCase):
    """Pure-arithmetic helper, no PIL needed."""

    def test_full_passes_quality_through(self) -> None:
        for q in (10, 30, 55, 90, 100):
            self.assertEqual(
                camera_service._apply_encode_mode_quality(q, camera_service.ENCODE_MODE_FULL),
                q)

    def test_y_only_passes_quality_through(self) -> None:
        for q in (10, 30, 55, 90, 100):
            self.assertEqual(
                camera_service._apply_encode_mode_quality(q, camera_service.ENCODE_MODE_Y_ONLY),
                q)

    def test_motion_only_caps_at_motion_quality(self) -> None:
        cap = camera_service.MOTION_ONLY_QUALITY
        # Below the cap, pass through; at or above, clamp.
        self.assertEqual(
            camera_service._apply_encode_mode_quality(20, camera_service.ENCODE_MODE_MOTION_ONLY),
            min(20, cap))
        self.assertEqual(
            camera_service._apply_encode_mode_quality(80, camera_service.ENCODE_MODE_MOTION_ONLY),
            cap)
        self.assertEqual(
            camera_service._apply_encode_mode_quality(cap, camera_service.ENCODE_MODE_MOTION_ONLY),
            cap)

    def test_wireframe_caps_at_wireframe_quality(self) -> None:
        cap = camera_service.WIREFRAME_QUALITY
        self.assertEqual(
            camera_service._apply_encode_mode_quality(80, camera_service.ENCODE_MODE_WIREFRAME),
            cap)
        # Wireframe cap should be tighter than motion_only.
        self.assertLessEqual(camera_service.WIREFRAME_QUALITY,
                             camera_service.MOTION_ONLY_QUALITY)


def _have_pil() -> bool:
    try:
        import PIL  # noqa: F401
        return True
    except Exception:
        return False


@unittest.skipUnless(_have_pil(), "PIL not installed")
class EncodeModeWebPSizeTests(unittest.TestCase):
    """End-to-end: mode flip changes the encoded WebP byte count."""

    def setUp(self) -> None:
        self._saved_mode = camera_service.ENCODE_MODE
        # Build one synthetic canvas via the in-tree SyntheticCamera so
        # the test doesn't need a real frame source.
        cam = camera_service.SyntheticCamera()
        self.canvas = cam.grab_rgb()

    def tearDown(self) -> None:
        camera_service.ENCODE_MODE = self._saved_mode

    def test_y_only_smaller_than_full_for_synthetic_canvas(self) -> None:
        # SyntheticCamera renders coloured patterns; chroma drop should
        # save at least a few bytes per tile on average. We average
        # across the whole grid to smooth out per-tile noise.
        camera_service.ENCODE_MODE = camera_service.ENCODE_MODE_FULL
        full_total = 0
        for ty in range(camera_service.GRID_H):
            for tx in range(camera_service.GRID_W):
                full_total += len(camera_service._encode_tile(self.canvas, tx, ty))
        camera_service.ENCODE_MODE = camera_service.ENCODE_MODE_Y_ONLY
        y_total = 0
        for ty in range(camera_service.GRID_H):
            for tx in range(camera_service.GRID_W):
                y_total += len(camera_service._encode_tile(self.canvas, tx, ty))
        self.assertLess(y_total, full_total,
                        f"y_only={y_total}B not < full={full_total}B")

    def test_wireframe_smaller_than_motion_only(self) -> None:
        camera_service.ENCODE_MODE = camera_service.ENCODE_MODE_MOTION_ONLY
        motion_total = 0
        for ty in range(camera_service.GRID_H):
            for tx in range(camera_service.GRID_W):
                motion_total += len(camera_service._encode_tile(self.canvas, tx, ty))
        camera_service.ENCODE_MODE = camera_service.ENCODE_MODE_WIREFRAME
        wire_total = 0
        for ty in range(camera_service.GRID_H):
            for tx in range(camera_service.GRID_W):
                wire_total += len(camera_service._encode_tile(self.canvas, tx, ty))
        self.assertLessEqual(wire_total, motion_total,
                             f"wireframe={wire_total}B > motion_only={motion_total}B")

    def test_explicit_encode_mode_kwarg_overrides_global(self) -> None:
        camera_service.ENCODE_MODE = camera_service.ENCODE_MODE_FULL
        full_blob = camera_service._encode_tile(
            self.canvas, 0, 0,
            encode_mode=camera_service.ENCODE_MODE_FULL)
        wire_blob = camera_service._encode_tile(
            self.canvas, 0, 0,
            encode_mode=camera_service.ENCODE_MODE_WIREFRAME)
        # Global is still FULL; the kwarg path took precedence.
        self.assertEqual(camera_service.ENCODE_MODE,
                         camera_service.ENCODE_MODE_FULL)
        self.assertLessEqual(len(wire_blob), len(full_blob))


@unittest.skipUnless(_have_pil(), "PIL not installed")
class EncodeCacheInvalidatesOnModeChangeTests(unittest.TestCase):
    """W2-05 cache key now includes ENCODE_MODE; flip invalidates entries."""

    def setUp(self) -> None:
        self._saved_mode = camera_service.ENCODE_MODE
        cam = camera_service.SyntheticCamera()
        self.canvas = cam.grab_rgb()

    def tearDown(self) -> None:
        camera_service.ENCODE_MODE = self._saved_mode

    def _replay_cam(self):
        canvas = self.canvas

        class _R:
            def grab_rgb(self_inner):
                return canvas

        return _R()

    def test_mode_flip_forces_full_re_encode(self) -> None:
        from image_pipeline.tile_cache import TileEncodeCache
        n_tiles = camera_service.GRID_W * camera_service.GRID_H
        cache = TileEncodeCache(n_tiles=n_tiles, history=2)

        # Stub encoder so we can count calls without paying PIL cost.
        calls = []
        original = camera_service._encode_tile

        def _stub(rgb, tx, ty, quality=None, encode_mode=None):
            calls.append((tx, ty, quality, encode_mode,
                          camera_service.ENCODE_MODE))
            head = bytes([tx & 0xFF, ty & 0xFF,
                          (quality or 0) & 0xFF,
                          camera_service.ENCODE_MODE & 0xFF])
            return head + b"\x55" * 56

        camera_service._encode_tile = _stub
        try:
            camera_service.ENCODE_MODE = camera_service.ENCODE_MODE_FULL
            accum = camera_service.FrameAccum()
            camera_service._build_frame(self._replay_cam(), accum,
                                        force_keyframe=True,
                                        encode_cache=cache)
            first_pass = len(calls)
            self.assertEqual(first_pass, n_tiles)

            # Same canvas, same mode -> all hits.
            calls.clear()
            accum2 = camera_service.FrameAccum()
            camera_service._build_frame(self._replay_cam(), accum2,
                                        force_keyframe=True,
                                        encode_cache=cache)
            self.assertEqual(len(calls), 0,
                             "expected all cache hits on identical mode")

            # Flip mode -> cache key changes -> all misses again.
            calls.clear()
            camera_service.ENCODE_MODE = camera_service.ENCODE_MODE_Y_ONLY
            accum3 = camera_service.FrameAccum()
            camera_service._build_frame(self._replay_cam(), accum3,
                                        force_keyframe=True,
                                        encode_cache=cache)
            self.assertEqual(len(calls), n_tiles,
                             f"mode flip should invalidate cache; got "
                             f"{len(calls)} encodes (want {n_tiles})")
        finally:
            camera_service._encode_tile = original


if __name__ == "__main__":
    unittest.main()
