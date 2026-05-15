"""IP-W2-05: per-tile WebP encode cache on the X8.

Pins three things:

1. The standalone :class:`TileEncodeCache` semantics (hit/miss/eviction).
2. ``_build_frame(encode_cache=...)`` skips ``_encode_tile`` for tiles
   whose raw RGB slice round-trips to a recently encoded blob, while
   producing a byte-identical wire payload to the no-cache path.
3. Quality is part of the cache key — re-running the same canvas at a
   different ROI quality forces a re-encode.
"""

from __future__ import annotations

import importlib.util
import os
import sys
import unittest
from unittest import mock


_HERE = os.path.dirname(__file__)
_X8_DIR = os.path.normpath(os.path.join(
    _HERE, "..", "..", "firmware", "tractor_x8"))
_BS_DIR = os.path.normpath(os.path.join(_HERE, ".."))


# Load base-station ``frame_format`` directly to dodge the package-name
# collision with the tractor's ``image_pipeline`` package — same trick
# the W2-03 test file uses.
_FF_PATH = os.path.join(_BS_DIR, "image_pipeline", "frame_format.py")
_spec = importlib.util.spec_from_file_location("_bs_frame_format_w2_05", _FF_PATH)
assert _spec is not None and _spec.loader is not None
_bs_frame_format = importlib.util.module_from_spec(_spec)
sys.modules["_bs_frame_format_w2_05"] = _bs_frame_format
_spec.loader.exec_module(_bs_frame_format)
parse_tile_delta_frame = _bs_frame_format.parse_tile_delta_frame

if _X8_DIR not in sys.path:
    sys.path.insert(0, _X8_DIR)

import camera_service  # noqa: E402
from image_pipeline.tile_cache import (  # noqa: E402
    DEFAULT_HISTORY,
    TileEncodeCache,
)


def _fake_encode_factory(blob_size: int = 24, calls: list | None = None):
    """Recording stub for ``camera_service._encode_tile``."""
    if calls is None:
        calls = []

    def _fake(rgb_canvas, tx, ty, quality=None):
        calls.append((tx, ty, quality))
        body = bytes([tx & 0xFF, ty & 0xFF, (quality or 0) & 0xFF])
        pad = max(0, blob_size - len(body))
        return body + b"\xCC" * pad

    return _fake, calls


class TileEncodeCacheUnitTests(unittest.TestCase):

    def test_lookup_miss_then_store_then_hit(self) -> None:
        c = TileEncodeCache(n_tiles=4, history=2)
        self.assertIsNone(c.lookup(0, b"pixels-A"))
        self.assertEqual(c.stats.misses, 1)
        c.store(0, b"pixels-A", b"BLOBA")
        self.assertEqual(c.lookup(0, b"pixels-A"), b"BLOBA")
        self.assertEqual(c.stats.hits, 1)

    def test_different_pixels_are_separate_entries(self) -> None:
        c = TileEncodeCache(n_tiles=4, history=4)
        c.store(0, b"pix-A", b"BLOB1")
        c.store(0, b"pix-B", b"BLOB2")
        self.assertEqual(c.lookup(0, b"pix-A"), b"BLOB1")
        self.assertEqual(c.lookup(0, b"pix-B"), b"BLOB2")

    def test_buckets_are_per_tile(self) -> None:
        c = TileEncodeCache(n_tiles=4, history=2)
        c.store(0, b"pix-A", b"BLOB-T0")
        c.store(1, b"pix-A", b"BLOB-T1")
        # Same key bytes, different tile bucket → independent entries.
        self.assertEqual(c.lookup(0, b"pix-A"), b"BLOB-T0")
        self.assertEqual(c.lookup(1, b"pix-A"), b"BLOB-T1")

    def test_lru_evicts_oldest_when_history_exceeded(self) -> None:
        c = TileEncodeCache(n_tiles=1, history=2)
        c.store(0, b"A", b"blobA")
        c.store(0, b"B", b"blobB")
        c.store(0, b"C", b"blobC")               # evicts "A"
        self.assertEqual(c.stats.evictions, 1)
        self.assertIsNone(c.lookup(0, b"A"))
        self.assertEqual(c.lookup(0, b"B"), b"blobB")
        self.assertEqual(c.lookup(0, b"C"), b"blobC")

    def test_lookup_refreshes_lru_position(self) -> None:
        c = TileEncodeCache(n_tiles=1, history=2)
        c.store(0, b"A", b"blobA")
        c.store(0, b"B", b"blobB")
        # Touching A should make B the LRU victim now.
        self.assertEqual(c.lookup(0, b"A"), b"blobA")
        c.store(0, b"C", b"blobC")
        self.assertIsNone(c.lookup(0, b"B"))
        self.assertEqual(c.lookup(0, b"A"), b"blobA")
        self.assertEqual(c.lookup(0, b"C"), b"blobC")

    def test_store_with_existing_key_updates_blob_in_place(self) -> None:
        c = TileEncodeCache(n_tiles=1, history=2)
        c.store(0, b"A", b"old")
        c.store(0, b"A", b"new")
        self.assertEqual(c.lookup(0, b"A"), b"new")
        self.assertEqual(c.stats.evictions, 0)

    def test_constructor_rejects_invalid_sizes(self) -> None:
        with self.assertRaises(ValueError):
            TileEncodeCache(n_tiles=0)
        with self.assertRaises(ValueError):
            TileEncodeCache(n_tiles=4, history=0)

    def test_default_history_constant(self) -> None:
        self.assertGreaterEqual(DEFAULT_HISTORY, 2)

    def test_reset_clears_stats_and_buckets(self) -> None:
        c = TileEncodeCache(n_tiles=2, history=2)
        c.store(0, b"A", b"x")
        c.lookup(0, b"A")
        c.reset()
        self.assertEqual(c.stats.hits, 0)
        self.assertEqual(c.stats.misses, 0)
        self.assertIsNone(c.lookup(0, b"A"))


class BuildFrameWithCacheTests(unittest.TestCase):
    """Cache integration through ``camera_service._build_frame``."""

    def _run_keyframe(self, cam, accum, encode_cache, fake):
        with mock.patch.object(camera_service, "_encode_tile", fake):
            return camera_service._build_frame(
                cam, accum, force_keyframe=True, encode_cache=encode_cache)

    def test_first_keyframe_misses_every_tile(self) -> None:
        cam = camera_service.SyntheticCamera()
        accum = camera_service.FrameAccum()
        cache = TileEncodeCache(
            n_tiles=camera_service.GRID_W * camera_service.GRID_H)
        fake, calls = _fake_encode_factory()
        self._run_keyframe(cam, accum, cache, fake)
        n_tiles = camera_service.GRID_W * camera_service.GRID_H
        self.assertEqual(len(calls), n_tiles)
        self.assertEqual(cache.stats.misses, n_tiles)
        self.assertEqual(cache.stats.hits, 0)

    def test_second_keyframe_with_identical_canvas_is_full_cache_hit(self) -> None:
        # SyntheticCamera returns a deterministic canvas keyed on frame
        # number — to get an identical canvas across two encodes we hold
        # one frame and replay it via a thin wrapper.
        base_cam = camera_service.SyntheticCamera()
        canvas = base_cam.grab_rgb()

        class _ReplayCam:
            def grab_rgb(self_inner) -> bytes:
                return canvas

        cam = _ReplayCam()
        accum = camera_service.FrameAccum()
        cache = TileEncodeCache(
            n_tiles=camera_service.GRID_W * camera_service.GRID_H)
        fake, calls = _fake_encode_factory()
        self._run_keyframe(cam, accum, cache, fake)
        n_tiles = camera_service.GRID_W * camera_service.GRID_H
        self.assertEqual(len(calls), n_tiles)
        # Second keyframe on byte-identical canvas: zero re-encodes.
        accum2 = camera_service.FrameAccum()
        calls.clear()
        self._run_keyframe(cam, accum2, cache, fake)
        self.assertEqual(len(calls), 0,
                         "cache did not absorb the redundant encode")
        self.assertEqual(cache.stats.hits, n_tiles)

    def test_wire_payload_byte_identical_with_and_without_cache(self) -> None:
        base_cam = camera_service.SyntheticCamera()
        canvas = base_cam.grab_rgb()

        class _ReplayCam:
            def grab_rgb(self_inner) -> bytes:
                return canvas

        # Run no-cache path.
        accum_a = camera_service.FrameAccum()
        fake_a, _ = _fake_encode_factory()
        with mock.patch.object(camera_service, "_encode_tile", fake_a):
            payload_a = camera_service._build_frame(
                _ReplayCam(), accum_a, force_keyframe=True)
        # Run cache path against the same canvas; payload bytes must match.
        cache = TileEncodeCache(
            n_tiles=camera_service.GRID_W * camera_service.GRID_H)
        accum_b = camera_service.FrameAccum()
        fake_b, _ = _fake_encode_factory()
        with mock.patch.object(camera_service, "_encode_tile", fake_b):
            payload_b = camera_service._build_frame(
                _ReplayCam(), accum_b, force_keyframe=True,
                encode_cache=cache)
        self.assertEqual(payload_a, payload_b)
        self.assertEqual(parse_tile_delta_frame(payload_a).changed_indices,
                         parse_tile_delta_frame(payload_b).changed_indices)

    def test_quality_change_invalidates_cached_blob(self) -> None:
        # Same pixels, different ROI quality must miss the cache because
        # quality is folded into the key.
        base_cam = camera_service.SyntheticCamera()
        canvas = base_cam.grab_rgb()

        class _ReplayCam:
            def grab_rgb(self_inner) -> bytes:
                return canvas

        from image_pipeline.roi import RoiPlanner
        roi = RoiPlanner(grid_w=camera_service.GRID_W,
                         grid_h=camera_service.GRID_H)
        # Phase 1: no ROI → uniform WEBP_QUALITY for every tile.
        cache = TileEncodeCache(
            n_tiles=camera_service.GRID_W * camera_service.GRID_H)
        accum = camera_service.FrameAccum()
        fake, calls = _fake_encode_factory()
        with mock.patch.object(camera_service, "_encode_tile", fake):
            camera_service._build_frame(_ReplayCam(), accum,
                                        force_keyframe=True,
                                        encode_cache=cache)
        n_tiles = camera_service.GRID_W * camera_service.GRID_H
        self.assertEqual(len(calls), n_tiles)
        # Phase 2: same canvas, but split into ROI inside/outside qualities.
        roi.apply_hint(col_lo=0, col_hi=2, row_lo=0, row_hi=1)
        accum2 = camera_service.FrameAccum()
        calls.clear()
        with mock.patch.object(camera_service, "_encode_tile", fake):
            camera_service._build_frame(_ReplayCam(), accum2,
                                        force_keyframe=True,
                                        encode_cache=cache,
                                        roi_planner=roi)
        # Every tile should re-encode because quality changed for each.
        # (inside-ROI: WEBP_QUALITY → ROI_QUALITY_INSIDE; outside-ROI:
        # WEBP_QUALITY → ROI_QUALITY_OUTSIDE).
        if (camera_service.ROI_QUALITY_INSIDE != camera_service.WEBP_QUALITY
                and camera_service.ROI_QUALITY_OUTSIDE != camera_service.WEBP_QUALITY):
            self.assertEqual(len(calls), n_tiles)


if __name__ == "__main__":
    unittest.main()
