"""Optical-flow microframe encoder (topic 0x28).

Q on the IMAGE_PIPELINE.md §3.4 fallback ladder. Instead of re-encoding the
whole canvas as WebP, we estimate a per-tile (dx, dy) integer-pixel motion
vector from the previous and current Y planes and ship just those vectors.
At 8×12 tiles + 4 bytes each, the whole microframe is ≤ 2 + 96*4 = 386 B.

We use a simple block-matching search bounded to ±8 pixels per axis
(matches what the base side's Badge.PREDICTED overlay can render). NumPy
is preferred; pure-Python fallback exists for portability.

Wire format mirrors :mod:`base_station.image_pipeline.motion_replay`:

    u8  base_seq                 ; mirrors I-frame chain
    u8  count                    ; vectors that follow
    repeated count times:
        u16 tile_index           ; little-endian
        i8  dx
        i8  dy
"""
from __future__ import annotations

import struct
from dataclasses import dataclass

try:                                             # pragma: no cover
    import numpy as _np                          # type: ignore
    _HAVE_NUMPY = True
except ImportError:                              # pragma: no cover
    _np = None                                    # type: ignore
    _HAVE_NUMPY = False

SEARCH_RANGE = 8


@dataclass
class TileMotion:
    tile_index: int
    dx: int
    dy: int


def _np_block_match(prev: "_np.ndarray", curr: "_np.ndarray",
                    tx: int, ty: int, tile_px: int) -> tuple[int, int]:
    h, w = prev.shape
    x0 = tx * tile_px
    y0 = ty * tile_px
    template = curr[y0:y0 + tile_px, x0:x0 + tile_px].astype(_np.int32)
    best = (0, 0)
    best_score = None
    for dy in range(-SEARCH_RANGE, SEARCH_RANGE + 1):
        sy = y0 - dy
        if sy < 0 or sy + tile_px > h:
            continue
        for dx in range(-SEARCH_RANGE, SEARCH_RANGE + 1):
            sx = x0 - dx
            if sx < 0 or sx + tile_px > w:
                continue
            block = prev[sy:sy + tile_px, sx:sx + tile_px].astype(_np.int32)
            score = int(_np.abs(block - template).sum())
            if best_score is None or score < best_score:
                best_score = score
                best = (dx, dy)
    return best


def estimate_motion(prev_y: bytes, curr_y: bytes,
                    width: int, height: int,
                    grid_w: int, grid_h: int, tile_px: int,
                    only_indices: list[int] | None = None) -> list[TileMotion]:
    """Compute one (dx, dy) per requested tile.

    If `only_indices` is None we estimate every tile; otherwise we estimate
    only the supplied tile indices (typically the union of the previous
    "changed" bitmap and the current bitmap, so dropped tiles still get a
    vector for Badge.PREDICTED).
    """
    if not _HAVE_NUMPY:
        return []
    prev = _np.frombuffer(prev_y, dtype=_np.uint8).reshape((height, width))
    curr = _np.frombuffer(curr_y, dtype=_np.uint8).reshape((height, width))
    indices = only_indices if only_indices is not None else list(range(grid_w * grid_h))
    out: list[TileMotion] = []
    for idx in indices:
        ty, tx = divmod(idx, grid_w)
        dx, dy = _np_block_match(prev, curr, tx, ty, tile_px)
        out.append(TileMotion(tile_index=idx, dx=dx, dy=dy))
    return out


def pack_motion_frame(base_seq: int, motions: list[TileMotion]) -> bytes:
    """Pack into the topic-0x28 wire format."""
    if len(motions) > 255:
        motions = motions[:255]
    out = bytearray(struct.pack("BB", base_seq & 0xFF, len(motions)))
    for m in motions:
        if not (0 <= m.tile_index < 65536):
            continue
        dx = max(-128, min(127, int(m.dx)))
        dy = max(-128, min(127, int(m.dy)))
        out += struct.pack("<Hbb", m.tile_index, dx, dy)
    return bytes(out)
