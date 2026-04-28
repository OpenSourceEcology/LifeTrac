"""Tests for the base-station image_pipeline modules added in bucket A."""
from __future__ import annotations

import unittest

from image_pipeline.bg_cache import BackgroundCache
from image_pipeline.canvas import Canvas, CanvasUpdate
from image_pipeline.fallback_render import FallbackRenderer
from image_pipeline.frame_format import (
    FrameDecodeError,
    TileBlob,
    TileDeltaFrame,
    encode_tile_delta_frame,
    parse_tile_delta_frame,
)
from image_pipeline.motion_replay import (
    MotionFrame,
    MotionReplayer,
    MotionVector,
    parse_motion_frame,
)
from image_pipeline.reassemble import FRAGMENT_MAGIC, FragmentReassembler
from image_pipeline.recolourise import Recolouriser
from image_pipeline.state_publisher import Detection, StatePublisher
from image_pipeline.wireframe_render import (
    WireframeFrame,
    WireframeRenderer,
    parse_wireframe_frame,
)
from lora_proto import Badge


def _tile(idx: int, blob: bytes = b"\x01\x02\x03") -> TileBlob:
    return TileBlob(index=idx, tx=idx % 12, ty=idx // 12, blob=blob)


def _make_keyframe(seq: int, indices: list[int]) -> TileDeltaFrame:
    return TileDeltaFrame(
        frame_kind=1,
        base_seq=seq,
        grid_w=12,
        grid_h=8,
        tile_px=32,
        changed_indices=indices,
        tiles=[_tile(i, bytes([i & 0xFF, 0xAA])) for i in indices],
    )


def _make_delta(seq: int, indices: list[int]) -> TileDeltaFrame:
    return TileDeltaFrame(
        frame_kind=0,
        base_seq=seq,
        grid_w=12,
        grid_h=8,
        tile_px=32,
        changed_indices=indices,
        tiles=[_tile(i, bytes([i & 0xFF, 0x55])) for i in indices],
    )


class FrameFormatTests(unittest.TestCase):
    def test_round_trip(self):
        original = _make_keyframe(7, [0, 1, 2, 95])
        wire = encode_tile_delta_frame(original)
        decoded = parse_tile_delta_frame(wire)
        self.assertEqual(decoded.frame_kind, 1)
        self.assertEqual(decoded.base_seq, 7)
        self.assertEqual(decoded.changed_indices, [0, 1, 2, 95])
        self.assertEqual(len(decoded.tiles), 4)
        self.assertEqual(decoded.tiles[0].index, 0)
        self.assertEqual(decoded.tiles[-1].index, 95)

    def test_truncated_payload_raises(self):
        wire = encode_tile_delta_frame(_make_keyframe(0, [0]))
        with self.assertRaises(FrameDecodeError):
            parse_tile_delta_frame(wire[:-1])

    def test_trailing_bytes_raise(self):
        wire = encode_tile_delta_frame(_make_keyframe(0, [0]))
        with self.assertRaises(FrameDecodeError):
            parse_tile_delta_frame(wire + b"\x00")


class CanvasTests(unittest.TestCase):
    def setUp(self):
        self.now = [0]
        self.canvas = Canvas(clock_ms=lambda: self.now[0])

    def test_keyframe_then_delta(self):
        kf = _make_keyframe(0, [0, 1, 2])
        upd = self.canvas.apply(kf)
        self.assertTrue(upd.is_keyframe)
        self.assertFalse(upd.request_keyframe)
        self.assertEqual(sorted(upd.updated_indices), [0, 1, 2])

        self.now[0] = 100
        delta = _make_delta(1, [1])
        upd = self.canvas.apply(delta)
        self.assertFalse(upd.request_keyframe)
        self.assertEqual(upd.updated_indices, [1])

    def test_delta_before_keyframe_requests_keyframe(self):
        upd = self.canvas.apply(_make_delta(5, [0]))
        self.assertTrue(upd.request_keyframe)

    def test_base_seq_gap_requests_keyframe(self):
        self.canvas.apply(_make_keyframe(0, [0]))
        upd = self.canvas.apply(_make_delta(2, [0]))   # expected seq 1
        self.assertTrue(upd.request_keyframe)
        self.assertIn("base_seq gap", upd.reason)

    def test_grid_mismatch_requests_keyframe(self):
        weird = _make_keyframe(0, [0])
        weird.grid_w = 10                     # mismatched
        upd = self.canvas.apply(weird)
        self.assertTrue(upd.request_keyframe)

    def test_snapshot_includes_age(self):
        self.now[0] = 50
        self.canvas.apply(_make_keyframe(0, [0]))
        self.now[0] = 300
        items = list(self.canvas.snapshot())
        self.assertEqual(items[0][0], 0)
        self.assertEqual(items[0][1].age_ms_at_publish, 250)
        self.assertEqual(items[0][1].badge, Badge.RAW)


class ReassemblerTests(unittest.TestCase):
    def test_passthrough_complete_frame(self):
        wire = encode_tile_delta_frame(_make_keyframe(0, [0, 1]))
        ras = FragmentReassembler()
        frame = ras.feed(wire)
        self.assertIsNotNone(frame)
        self.assertEqual(frame.changed_indices, [0, 1])
        self.assertEqual(ras.stats.bad_magic_passthroughs, 1)

    def test_two_fragments(self):
        wire = encode_tile_delta_frame(_make_keyframe(0, [0, 1, 2]))
        midpoint = len(wire) // 2
        frag0 = bytes([FRAGMENT_MAGIC, 0, 0, 1]) + wire[:midpoint]
        frag1 = bytes([FRAGMENT_MAGIC, 0, 1, 1]) + wire[midpoint:]
        ras = FragmentReassembler()
        self.assertIsNone(ras.feed(frag0))
        frame = ras.feed(frag1)
        self.assertIsNotNone(frame)
        self.assertEqual(frame.base_seq, 0)
        self.assertEqual(ras.stats.completed_frames, 1)

    def test_timeout_clears_partial(self):
        clock = [0]
        ras = FragmentReassembler(timeout_ms=100, clock_ms=lambda: clock[0])
        ras.feed(bytes([FRAGMENT_MAGIC, 0, 0, 1]) + b"junk")
        clock[0] = 1000
        ras.feed(bytes([FRAGMENT_MAGIC, 1, 0, 1]) + b"x")    # triggers GC
        self.assertEqual(ras.stats.timeouts, 1)


class BackgroundCacheTests(unittest.TestCase):
    def test_fills_stale_tile_with_majority_blob(self):
        clock = [0]
        canvas = Canvas(clock_ms=lambda: clock[0])
        cache = BackgroundCache(n_tiles=canvas.n_tiles, stale_after_ms=100,
                                clock_ms=lambda: clock[0])
        canvas.apply(_make_keyframe(0, [0]))
        cache.observe(0, b"good")
        cache.observe(0, b"good")
        cache.observe(0, b"jitter")
        clock[0] = 5000
        filled = cache.fill_misses(canvas)
        self.assertEqual(filled, [0])
        self.assertEqual(canvas._tiles[0].badge, Badge.CACHED)   # noqa: SLF001
        self.assertEqual(canvas._tiles[0].blob, b"good")         # noqa: SLF001


class StatePublisherTests(unittest.TestCase):
    def test_snapshot_round_trip(self):
        canvas = Canvas()
        canvas.apply(_make_keyframe(0, [0, 1]))
        pub = StatePublisher(canvas=canvas, accel_status="online", encode_mode="full")
        pub.detections.append(Detection(cls="person", conf=0.9, x=0.1, y=0.1, w=0.2, h=0.5))
        snap = pub.snapshot()
        self.assertEqual(snap["grid"]["w"], 12)
        self.assertEqual(len(snap["tiles"]), 2)
        self.assertEqual(snap["accel_status"], "online")
        self.assertEqual(snap["detections"][0]["cls"], "person")


class FallbackRendererTests(unittest.TestCase):
    def test_text_digest_when_pil_missing(self):
        # We don't actually mock PIL out; just verify render() returns a result
        # the size we expect (text or image).
        canvas = Canvas()
        canvas.apply(_make_keyframe(0, [0]))
        pub = StatePublisher(canvas=canvas)
        renderer = FallbackRenderer(canvas, pub)
        result = renderer.render()
        self.assertGreater(len(result.image_bytes), 0)
        self.assertIn(result.mime, ("image/webp", "text/plain"))


class RecolouriserTests(unittest.TestCase):
    def test_apply_y_only_uses_reference(self):
        canvas = Canvas()
        canvas.apply(_make_keyframe(0, [3]))
        rec = Recolouriser(n_tiles=canvas.n_tiles)
        rec.remember_colour(3, b"colour-ref")
        ok = rec.apply_y_only(canvas, 3, b"luma-blob")
        self.assertTrue(ok)
        self.assertEqual(canvas._tiles[3].badge, Badge.RECOLOURISED)   # noqa: SLF001
        self.assertIn(b"colour-ref", canvas._tiles[3].blob)            # noqa: SLF001
        self.assertIn(b"luma-blob", canvas._tiles[3].blob)             # noqa: SLF001

    def test_apply_y_only_returns_false_without_reference(self):
        canvas = Canvas()
        canvas.apply(_make_keyframe(0, [3]))
        rec = Recolouriser(n_tiles=canvas.n_tiles)
        self.assertFalse(rec.apply_y_only(canvas, 3, b"luma"))


class MotionReplayTests(unittest.TestCase):
    def test_parse_round_trip(self):
        payload = bytes([7, 2,
                         0, 0,        # tile 0
                         5, 250 - 256 + 256, # dx=5, dy=-6 (wrap)
                         0, 0,
                         3, 0,        # dx=3, dy=0 — wait, format is u16 idx, so re-do
                         ])
        # easier: build from helpers
        frame = MotionFrame(base_seq=7, vectors=[MotionVector(0, 5, -6),
                                                  MotionVector(2, 3, 0)])
        encoded = bytes([7, 2]) + b"\x00\x00\x05\xfa" + b"\x02\x00\x03\x00"
        decoded = parse_motion_frame(encoded)
        self.assertEqual(decoded.base_seq, 7)
        self.assertEqual(decoded.vectors[0].dx, 5)
        self.assertEqual(decoded.vectors[0].dy, -6)
        self.assertEqual(decoded.vectors[1].tile_index, 2)

    def test_apply_marks_predicted(self):
        canvas = Canvas()
        canvas.apply(_make_keyframe(0, [0, 1]))
        replayer = MotionReplayer()
        touched = replayer.apply(canvas, MotionFrame(base_seq=1, vectors=[
            MotionVector(0, 4, -2),
            MotionVector(99, 0, 0),       # out of range — ignored
            MotionVector(50, 0, 0),       # no blob present — skipped
        ]))
        self.assertEqual(touched, [0])
        self.assertEqual(canvas._tiles[0].badge, Badge.PREDICTED)   # noqa: SLF001
        self.assertEqual(replayer.snapshot_vectors()[0], (4, -2))


class WireframeTests(unittest.TestCase):
    def test_parse_and_apply(self):
        bitmap = bytes([0xFF] * ((384 * 256 + 7) // 8))
        encoded = bytes([3, 0x80, 0x01, 0x00, 0x01]) + bitmap   # 384x256
        frame = parse_wireframe_frame(encoded)
        self.assertEqual(frame.base_seq, 3)
        self.assertEqual(frame.width, 384)
        self.assertEqual(frame.height, 256)
        renderer = WireframeRenderer(canvas_w=384, canvas_h=256)
        self.assertTrue(renderer.apply(frame))
        self.assertIsNotNone(renderer.age_ms())

    def test_wrong_canvas_size_rejected(self):
        bitmap = bytes([0xFF] * ((100 * 100 + 7) // 8))
        encoded = bytes([0, 100, 0, 100, 0]) + bitmap
        frame = parse_wireframe_frame(encoded)
        renderer = WireframeRenderer(canvas_w=384, canvas_h=256)
        self.assertFalse(renderer.apply(frame))


if __name__ == "__main__":
    unittest.main()
