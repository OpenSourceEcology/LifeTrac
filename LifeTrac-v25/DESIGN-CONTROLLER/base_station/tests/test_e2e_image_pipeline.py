"""End-to-end image-pipeline contract + V2 latency gate (Phase 5).

Closes the last open Phase-5 base-station gap: an automated test that pins
the producer (`camera_service.py` on the tractor X8) against the renderer
(`base_station/web/img/canvas_renderer.js`) without bringing up MQTT, the
serial bridge, or a real radio. The full pipe under test is:

    TileDeltaFrame (WebP tiles, PIL-encoded for realism)
      → encode_tile_delta_frame  (frame_format.py, producer side)
      → pack_telemetry_fragments (lora_proto.py R-6, on-air chunker)
      → TelemetryReassembler     (lora_bridge.py R-6 join layer)
      → FragmentReassembler      (image_pipeline/reassemble.py 0xFE join)
      → Canvas.apply             (image_pipeline/canvas.py persistent store)
      → StatePublisher.snapshot  (image_pipeline/state_publisher.py)

The browser-contract test asserts that every key `canvas_renderer.js` reads
(`grid.{w,h,tile_px}`, `tiles[].{tx,ty,blob_b64}`, `accel_status`,
`encode_mode`, `needs_keyframe`) is present in the snapshot — drift on
either side breaks the operator console silently, so we make it a
compile-time gate.

The V2 latency gate (`IMAGE_PIPELINE.md` § validation) puts a 500 ms p99
ceiling on capture → repaint. We don't have a real camera in CI, so this
test exercises the pure-Python portion of that budget (everything *after*
the WebP encoder hands bytes to the IPC framer); the camera + LoRa
airtime add a known constant on top, measured separately on the bench.

Catches before bench / before deploy:
  * Frame-format drift between producer and base parser.
  * Canvas-snapshot key drift breaking the browser painter.
  * StatePublisher dropping a field that an overlay JS module relies on.
  * Pure-Python pipeline regressing past the V2 budget on stock CPython.
"""

from __future__ import annotations

import io
import os
import statistics
import sys
import time
import unittest
from pathlib import Path

_BS_DIR = os.path.normpath(os.path.join(os.path.dirname(__file__), ".."))
if _BS_DIR not in sys.path:
    sys.path.insert(0, _BS_DIR)

from image_pipeline.canvas import Canvas                     # noqa: E402
from image_pipeline.frame_format import (                    # noqa: E402
    FRAME_KIND_DELTA,
    FRAME_KIND_KEY,
    TileBlob,
    TileDeltaFrame,
    encode_tile_delta_frame,
)
from image_pipeline.reassemble import FragmentReassembler    # noqa: E402
from image_pipeline.state_publisher import StatePublisher    # noqa: E402
from lora_proto import (                                     # noqa: E402
    PHY_IMAGE,
    TelemetryReassembler,
    pack_telemetry_fragments,
    parse_telemetry_fragment,
)


# ---------------------------------------------------------------------------
# Test config — keep small so the latency gate runs in <2 s on a laptop.
# ---------------------------------------------------------------------------
GRID_W = 12
GRID_H = 8
TILE_PX = 32
N_TILES = GRID_W * GRID_H

# V2 budget per IMAGE_PIPELINE.md: capture → operator-canvas-repaint p99 ≤ 500 ms.
# This test exercises the post-encoder pure-Python portion only; the camera
# + LoRa airtime add a known constant on top, measured on the bench.
V2_PYTHON_PORTION_P99_MS = 80
LATENCY_TRIALS = 60


def _have_pil() -> bool:
    try:
        import PIL  # noqa: F401
        return True
    except Exception:
        return False


def _make_webp_tile(fill_rgb: tuple[int, int, int]) -> bytes:
    from PIL import Image  # type: ignore
    img = Image.new("RGB", (TILE_PX, TILE_PX), fill_rgb)
    buf = io.BytesIO()
    img.save(buf, format="WEBP", quality=60, method=4)
    return buf.getvalue()


def _make_keyframe_wire(seq: int = 0) -> bytes:
    """Synthesise a full-canvas keyframe with deterministic per-tile colours."""
    tiles = []
    for idx in range(N_TILES):
        ty, tx = divmod(idx, GRID_W)
        colour = ((tx * 21) & 0xFF, (ty * 31) & 0xFF, ((tx + ty) * 17) & 0xFF)
        tiles.append(TileBlob(index=idx, tx=tx, ty=ty,
                              blob=_make_webp_tile(colour)))
    frame = TileDeltaFrame(
        frame_kind=FRAME_KIND_KEY, base_seq=seq,
        grid_w=GRID_W, grid_h=GRID_H, tile_px=TILE_PX,
        changed_indices=list(range(N_TILES)),
        tiles=tiles)
    return encode_tile_delta_frame(frame)


def _make_delta_wire(seq: int, indices: list[int],
                     fill_rgb: tuple[int, int, int]) -> bytes:
    tiles = []
    for idx in indices:
        ty, tx = divmod(idx, GRID_W)
        tiles.append(TileBlob(index=idx, tx=tx, ty=ty,
                              blob=_make_webp_tile(fill_rgb)))
    frame = TileDeltaFrame(
        frame_kind=FRAME_KIND_DELTA, base_seq=seq,
        grid_w=GRID_W, grid_h=GRID_H, tile_px=TILE_PX,
        changed_indices=indices, tiles=tiles)
    return encode_tile_delta_frame(frame)


def _push_through_bridge_join(wire: bytes,
                              telem_ra: TelemetryReassembler,
                              frag_seq: int,
                              source_id: int = 3,
                              topic_id: int = 0x25) -> bytes:
    """Mirror the lora_bridge bridge-join layer (R-6 telemetry chunker).

    Splits ``wire`` into ≤25 ms-airtime fragments via
    ``pack_telemetry_fragments`` and feeds them through a
    ``TelemetryReassembler`` keyed by (source_id, topic_id), returning the
    joined payload that the bridge would publish on
    ``lifetrac/v25/video/tile_delta``. Asserts the join completes.
    """
    fragments = pack_telemetry_fragments(wire, frag_seq=frag_seq, profile=PHY_IMAGE)
    joined = None
    for i, frag in enumerate(fragments):
        joined = telem_ra.feed(source_id=source_id, topic_id=topic_id,
                               body=frag, now_ms=i + 1)
    if joined is None:
        raise AssertionError(
            f"telemetry reassembly never completed across {len(fragments)} fragments")
    return joined


# ---------------------------------------------------------------------------
# 1. Browser-contract: snapshot shape matches canvas_renderer.js expectations
# ---------------------------------------------------------------------------
@unittest.skipUnless(_have_pil(), "PIL not installed")
class BrowserSnapshotContractTests(unittest.TestCase):
    """Pin StatePublisher.snapshot() against canvas_renderer.js."""

    def setUp(self) -> None:
        self.canvas = Canvas(grid_w=GRID_W, grid_h=GRID_H, tile_px=TILE_PX)
        self.publisher = StatePublisher(canvas=self.canvas)
        self.image_ra = FragmentReassembler()
        self.telem_ra = TelemetryReassembler()

    def _ingest(self, wire: bytes, frag_seq: int) -> None:
        joined = _push_through_bridge_join(wire, self.telem_ra, frag_seq)
        frame = self.image_ra.feed(joined)
        self.assertIsNotNone(frame, "image fragment reassembly failed")
        self.canvas.apply(frame)

    def test_snapshot_has_browser_top_level_keys(self) -> None:
        self._ingest(_make_keyframe_wire(seq=0), frag_seq=0)
        snap = self.publisher.snapshot()
        # Every key canvas_renderer.js / overlay modules read.
        for key in ("ts_ms", "grid", "tiles", "accel_status", "encode_mode",
                    "detections", "safety_verdict", "needs_keyframe",
                    "last_keyframe_reason"):
            self.assertIn(key, snap, f"missing browser-contract key: {key}")

    def test_snapshot_grid_matches_canvas_renderer_ensure_surfaces(self) -> None:
        self._ingest(_make_keyframe_wire(seq=0), frag_seq=0)
        snap = self.publisher.snapshot()
        # canvas_renderer.js does: gridW = snap.grid.w; ...; visible.width = gridW * tilePx
        self.assertEqual(snap["grid"], {"w": GRID_W, "h": GRID_H, "tile_px": TILE_PX})

    def test_snapshot_tiles_have_browser_per_tile_keys(self) -> None:
        self._ingest(_make_keyframe_wire(seq=0), frag_seq=0)
        snap = self.publisher.snapshot()
        self.assertEqual(len(snap["tiles"]), N_TILES,
                         "keyframe must populate every tile slot")
        for tile in snap["tiles"]:
            # canvas_renderer.js reads .tx, .ty, .blob_b64; overlays read .age_ms, .badge.
            for key in ("i", "tx", "ty", "age_ms", "badge", "blob_b64"):
                self.assertIn(key, tile, f"missing per-tile key: {key}")
            self.assertIsInstance(tile["blob_b64"], str)
            self.assertEqual(tile["i"], tile["ty"] * GRID_W + tile["tx"],
                             "per-tile index must be row-major")

    def test_delta_after_keyframe_persists_unchanged_tiles(self) -> None:
        """Browser draws each snapshot in full; delta tiles must overwrite
        only their changed slots while everything else keeps its last blob."""
        self._ingest(_make_keyframe_wire(seq=0), frag_seq=0)
        snap_before = self.publisher.snapshot()
        before_by_idx = {t["i"]: t["blob_b64"] for t in snap_before["tiles"]}

        delta_indices = [0, 5, 13, 95]
        self._ingest(_make_delta_wire(seq=1, indices=delta_indices,
                                      fill_rgb=(240, 16, 16)), frag_seq=1)
        snap_after = self.publisher.snapshot()
        after_by_idx = {t["i"]: t["blob_b64"] for t in snap_after["tiles"]}

        self.assertEqual(len(after_by_idx), N_TILES,
                         "delta must not erase the canvas")
        for idx in delta_indices:
            self.assertNotEqual(before_by_idx[idx], after_by_idx[idx],
                                f"tile {idx} should have been replaced by delta")
        for idx in set(range(N_TILES)) - set(delta_indices):
            self.assertEqual(before_by_idx[idx], after_by_idx[idx],
                             f"tile {idx} should have persisted across delta")

    def test_orphan_delta_requests_keyframe(self) -> None:
        """A delta arriving with no prior keyframe must surface
        ``needs_keyframe=True`` so the browser can show the 'awaiting key'
        state and the bridge can fire CMD_REQ_KEYFRAME."""
        wire = _make_delta_wire(seq=0, indices=[0],
                                fill_rgb=(255, 255, 255))
        joined = _push_through_bridge_join(wire, self.telem_ra, frag_seq=0)
        frame = self.image_ra.feed(joined)
        self.assertIsNotNone(frame)
        update = self.canvas.apply(frame)
        self.assertTrue(update.request_keyframe)
        self.publisher.needs_keyframe = update.request_keyframe
        self.publisher.last_keyframe_reason = update.reason
        snap = self.publisher.snapshot()
        self.assertTrue(snap["needs_keyframe"])
        self.assertIn("keyframe", snap["last_keyframe_reason"].lower())


# ---------------------------------------------------------------------------
# 2. V2 latency gate — pure-Python portion ≤ 80 ms p99
# ---------------------------------------------------------------------------
@unittest.skipUnless(_have_pil(), "PIL not installed")
class V2LatencyGateTests(unittest.TestCase):
    """The V2 budget per IMAGE_PIPELINE.md is 500 ms p99 capture → repaint.

    On stock CPython the bridge-join + reassemble + canvas-apply +
    snapshot portion should sit well under 80 ms; the camera + LoRa air
    consume the rest. We measure repeated round trips so transient GC
    pauses don't fail the gate."""

    def test_keyframe_then_delta_p99_under_budget(self) -> None:
        # Pre-encode wire bytes ONCE so we measure only the receive side
        # — the producer's WebP-encode time is gated by a separate test
        # (test_x8_tile_encode_cache.py).
        keyframe_wire = _make_keyframe_wire(seq=0)
        delta_wires = [
            _make_delta_wire(seq=(i + 1) & 0xFF,
                             indices=[(i * 7) % N_TILES, (i * 11 + 3) % N_TILES],
                             fill_rgb=((i * 37) & 0xFF, (i * 53) & 0xFF, (i * 71) & 0xFF))
            for i in range(LATENCY_TRIALS)
        ]

        samples_ms: list[float] = []
        for trial in range(LATENCY_TRIALS):
            canvas = Canvas(grid_w=GRID_W, grid_h=GRID_H, tile_px=TILE_PX)
            publisher = StatePublisher(canvas=canvas)
            image_ra = FragmentReassembler()
            telem_ra = TelemetryReassembler()

            t_start = time.perf_counter()
            joined_kf = _push_through_bridge_join(
                keyframe_wire, telem_ra, frag_seq=0)
            kf = image_ra.feed(joined_kf)
            self.assertIsNotNone(kf)
            canvas.apply(kf)
            joined_d = _push_through_bridge_join(
                delta_wires[trial], telem_ra, frag_seq=(trial + 1) & 0xFF)
            df = image_ra.feed(joined_d)
            self.assertIsNotNone(df)
            canvas.apply(df)
            _ = publisher.snapshot()
            t_end = time.perf_counter()

            samples_ms.append((t_end - t_start) * 1000.0)

        samples_ms.sort()
        p50 = samples_ms[len(samples_ms) // 2]
        p99_idx = max(0, int(len(samples_ms) * 0.99) - 1)
        p99 = samples_ms[p99_idx]
        worst = samples_ms[-1]
        # Keep the assertion quiet on success; on failure surface the
        # whole histogram so we know whether it's a tail-only regression
        # or a systemic slowdown.
        self.assertLessEqual(
            p99, V2_PYTHON_PORTION_P99_MS,
            f"V2 python-portion p99 latency budget blown: "
            f"p50={p50:.2f} ms p99={p99:.2f} ms worst={worst:.2f} ms "
            f"(budget={V2_PYTHON_PORTION_P99_MS} ms over {LATENCY_TRIALS} trials)")


# ---------------------------------------------------------------------------
# 3. Bridge-join ↔ image-join interoperability sanity check
# ---------------------------------------------------------------------------
class BridgeJoinSanityTests(unittest.TestCase):
    """Catch drift between the lora_proto telemetry chunker and the
    image_pipeline.reassemble fragment reassembler that lives one layer
    up. Both use 0xFE-magic 4-byte headers but are *separate* implementations
    operating on different wire formats — the bridge joins R-6 telemetry
    fragments, the image reassembler joins on-air image fragments. A
    payload that survives one must remain bit-identical after the round
    trip."""

    def test_payload_round_trips_through_telemetry_fragment_layer(self) -> None:
        payload = bytes(range(256)) * 3  # 768 bytes; forces multi-fragment
        fragments = pack_telemetry_fragments(payload, frag_seq=42, profile=PHY_IMAGE)
        self.assertGreater(len(fragments), 1, "test needs multi-fragment payload")
        # Each fragment must round-trip through parse_telemetry_fragment.
        for f in fragments:
            seq, idx, total, _data = parse_telemetry_fragment(f)
            self.assertEqual(seq, 42)
            self.assertEqual(total, len(fragments))
            self.assertLess(idx, total)
        ra = TelemetryReassembler()
        joined = None
        for i, f in enumerate(fragments):
            joined = ra.feed(source_id=3, topic_id=0x25, body=f, now_ms=i + 1)
        self.assertEqual(joined, payload,
                         "payload must round-trip bit-identically through bridge-join")


if __name__ == "__main__":  # pragma: no cover
    unittest.main()
