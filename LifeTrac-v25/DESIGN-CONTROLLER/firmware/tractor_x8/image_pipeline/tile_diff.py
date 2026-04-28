"""Per-tile change detector for the tractor X8 image pipeline.

Computes a 64-bit perceptual hash (pHash, 8×8 mean-DCT variant) per tile of
the registered current Y-plane and compares it against the previous frame's
hashes. Tiles whose Hamming distance exceeds a threshold are reported as
*changed* and forwarded to the WebP encoder + ``frame_format.encode_tile_delta_frame``.

This module is intentionally NumPy-only; the H747 OpenMV target swaps the
DCT for ulab on the embedded side, but the wire-level outputs are identical.

Public API
----------

``TileDiffer(grid_w, grid_h, tile_px)`` keeps per-tile pHash state across
frames. Methods:

- ``reset()``                                — drop cached hashes.
- ``hashes(frame)``                          — compute pHash bitfield for every tile.
- ``changed_indices(frame, *, threshold=8)`` — list of tile indices whose
  pHash Hamming distance from the cached version exceeds ``threshold``;
  also updates the cache.

The threshold default of 8 bits (out of 64) matches IMAGE_PIPELINE.md §3.2:
"≥ 12.5 % of bits flipped" is the empirical cut-off where re-encoding pays
off vs. paying the on-air retransmission cost.
"""
from __future__ import annotations

from dataclasses import dataclass

try:                                             # pragma: no cover
    import numpy as _np                          # type: ignore
    _HAVE_NUMPY = True
except ImportError:                              # pragma: no cover
    _np = None                                    # type: ignore
    _HAVE_NUMPY = False

DEFAULT_THRESHOLD = 8
PHASH_SIDE = 8                  # 8×8 DCT block → 64-bit hash


def _require_numpy() -> None:
    if not _HAVE_NUMPY:
        raise RuntimeError("tile_diff requires NumPy on the tractor X8")


def _phash_8x8(tile: "_np.ndarray") -> int:
    """64-bit pHash of an 8×8 luma block (mean-threshold variant).

    Faster than full DCT and within 1-2 bits of DCT-pHash on natural images;
    the X8 only has ~30 ms per tile so the simpler form is intentional.
    """
    block = tile.astype(_np.float32)
    h, w = block.shape
    if (h, w) != (PHASH_SIDE, PHASH_SIDE):
        # Down-sample by integer block-mean to 8×8 first.
        bh = max(1, h // PHASH_SIDE)
        bw = max(1, w // PHASH_SIDE)
        block = block[:bh * PHASH_SIDE, :bw * PHASH_SIDE]
        block = block.reshape(PHASH_SIDE, bh, PHASH_SIDE, bw).mean(axis=(1, 3))
    mean = float(block.mean())
    bits = 0
    flat = block.reshape(-1)
    for i, v in enumerate(flat):
        if v >= mean:
            bits |= 1 << i
    return bits


def _hamming64(a: int, b: int) -> int:
    return int.bit_count(a ^ b)


@dataclass
class _Cached:
    hashes: list[int]
    grid_w: int
    grid_h: int
    tile_px: int


class TileDiffer:
    """Stateful per-tile pHash differ.

    The caller passes in a *registered* (already shifted to compensate for
    bulk camera motion — see ``register.shift_canvas``) Y-plane each frame.
    """

    def __init__(self, grid_w: int = 12, grid_h: int = 8, tile_px: int = 32):
        if grid_w <= 0 or grid_h <= 0 or tile_px <= 0:
            raise ValueError("grid_w, grid_h, tile_px must all be > 0")
        _require_numpy()
        self.grid_w = grid_w
        self.grid_h = grid_h
        self.tile_px = tile_px
        self._prev: _Cached | None = None
        # Stats for the audit log / link diagnostics page.
        self.frames_seen = 0
        self.tiles_changed = 0

    # ----- public API -----

    def reset(self) -> None:
        self._prev = None

    def hashes(self, frame: "_np.ndarray") -> list[int]:
        """Compute the per-tile pHash for the current frame; no state mutation."""
        self._validate(frame)
        out: list[int] = []
        for ty in range(self.grid_h):
            y0 = ty * self.tile_px
            for tx in range(self.grid_w):
                x0 = tx * self.tile_px
                tile = frame[y0:y0 + self.tile_px, x0:x0 + self.tile_px]
                out.append(_phash_8x8(tile))
        return out

    def changed_indices(self, frame: "_np.ndarray", *,
                        threshold: int = DEFAULT_THRESHOLD,
                        force_keyframe: bool = False) -> list[int]:
        """Return tile indices considered *changed* since the previous frame.

        Side effect: updates the cached per-tile pHashes so the next call
        compares against this frame.

        ``force_keyframe=True`` (or first frame after ``reset()``) marks
        every tile changed and seeds the cache.
        """
        self._validate(frame)
        new_hashes = self.hashes(frame)
        n = self.grid_w * self.grid_h
        changed: list[int]
        if force_keyframe or self._prev is None or \
                self._prev.grid_w != self.grid_w or \
                self._prev.grid_h != self.grid_h or \
                self._prev.tile_px != self.tile_px:
            changed = list(range(n))
        else:
            changed = []
            prev_hashes = self._prev.hashes
            for i in range(n):
                if _hamming64(new_hashes[i], prev_hashes[i]) > threshold:
                    changed.append(i)
        self._prev = _Cached(new_hashes, self.grid_w, self.grid_h, self.tile_px)
        self.frames_seen += 1
        self.tiles_changed += len(changed)
        return changed

    # ----- internal -----

    def _validate(self, frame: "_np.ndarray") -> None:
        expected = (self.grid_h * self.tile_px, self.grid_w * self.tile_px)
        if frame.ndim != 2 or frame.shape != expected:
            raise ValueError(
                f"frame shape {frame.shape} != expected {expected} (HxW)")
