"""Y-only luma + 30 s colour reference recolouriser (scheme Z).

Per IMAGE_PIPELINE.md §8: when the encoder drops to ``y_only`` we receive
a single-channel luma blob per tile. The base side keeps the most recent
*colour* tile blob we ever saw for that index (refreshed at most every
30 s) and fuses the two: luma from the wire, chroma from the cache. The
result is overlaid into the canvas with ``Badge.RECOLOURISED`` so the
operator knows the colour is held-over.

We deliberately store the colour reference as the raw WebP blob rather
than decoding it: the actual fusion happens browser-side once the canvas
is rendered. This module is responsible for the *bookkeeping* — picking
the right reference tile, attaching the badge, and pruning references
older than ``reference_ttl_ms``.
"""
from __future__ import annotations

import time
from dataclasses import dataclass
from typing import Callable

from lora_proto import Badge

from .canvas import Canvas

DEFAULT_REFERENCE_TTL_MS = 30_000


@dataclass
class _Reference:
    blob: bytes
    captured_ms: int


class Recolouriser:
    """Hold a per-tile colour reference and apply it as needed."""

    def __init__(
        self,
        n_tiles: int,
        reference_ttl_ms: int = DEFAULT_REFERENCE_TTL_MS,
        clock_ms: Callable[[], int] | None = None,
    ) -> None:
        self.reference_ttl_ms = reference_ttl_ms
        self._refs: list[_Reference | None] = [None] * n_tiles
        self._clock_ms = clock_ms or (lambda: int(time.monotonic() * 1000))

    def remember_colour(self, tile_index: int, blob: bytes) -> None:
        """Cache the latest known-colour blob for this tile (called when a
        FULL-mode tile arrives)."""
        if not (0 <= tile_index < len(self._refs)):
            return
        ref = self._refs[tile_index]
        now = self._clock_ms()
        if ref is None or (now - ref.captured_ms) >= self.reference_ttl_ms:
            self._refs[tile_index] = _Reference(blob=blob, captured_ms=now)

    def apply_y_only(self, canvas: Canvas, tile_index: int, luma_blob: bytes) -> bool:
        """Tag ``canvas[tile_index]`` with ``Badge.RECOLOURISED`` once we
        have a fresh-enough colour reference. Returns True iff the tile was
        overlaid (caller decides whether to fall through to RAW handling).

        The fused payload prepended with a 1-byte sentinel ``0x59`` ('Y')
        carries: ``0x59 | luma_blob | 0xC0 | colour_ref_blob``. The browser's
        ``badge_renderer.js`` peels these apart and uses WebGL2 to do the
        actual luma↔chroma fuse.
        """
        if not (0 <= tile_index < len(self._refs)):
            return False
        ref = self._refs[tile_index]
        if ref is None:
            return False
        now = self._clock_ms()
        if (now - ref.captured_ms) > 2 * self.reference_ttl_ms:
            # Reference too stale — fall back to whatever the canvas had.
            return False
        fused = b"\x59" + luma_blob + b"\xC0" + ref.blob
        canvas.overlay(tile_index, fused, badge=Badge.RECOLOURISED, arrived_ms=now)
        return True
