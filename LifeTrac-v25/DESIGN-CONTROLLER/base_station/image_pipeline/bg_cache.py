"""Rolling per-tile median cache.

Per IMAGE_PIPELINE.md Phase 5A. When a tile times out (no fresh delta in
``stale_after_ms`` and no keyframe expected for a while), we paint the
canvas slot with the median of the last ``window`` good blobs we observed
for that index. The median is taken byte-wise — *we are not decoding the
WebP* — which means the cache only really makes sense for a tile that has
been visually stable. For dynamic tiles the median collapses to "the most
common encoded result of the last few frames", which is still better than
the alternative (showing nothing or a black hole).

The overlay sets `Badge.CACHED` so the operator never confuses a held-over
tile with a fresh one. ``state_publisher`` carries the age separately.
"""
from __future__ import annotations

import time
from collections import Counter, deque
from dataclasses import dataclass
from typing import Callable

from lora_proto import Badge

from .canvas import Canvas

DEFAULT_WINDOW = 8
DEFAULT_STALE_AFTER_MS = 3000


@dataclass
class _TileHistory:
    samples: deque[bytes]


class BackgroundCache:
    """Per-tile rolling history; can fill stale slots into a Canvas."""

    def __init__(
        self,
        n_tiles: int,
        window: int = DEFAULT_WINDOW,
        stale_after_ms: int = DEFAULT_STALE_AFTER_MS,
        clock_ms: Callable[[], int] | None = None,
    ) -> None:
        self.window = window
        self.stale_after_ms = stale_after_ms
        self._history: list[_TileHistory] = [
            _TileHistory(samples=deque(maxlen=window)) for _ in range(n_tiles)
        ]
        self._clock_ms = clock_ms or (lambda: int(time.monotonic() * 1000))

    def observe(self, tile_index: int, blob: bytes) -> None:
        """Record a fresh tile blob for the median pool."""
        if 0 <= tile_index < len(self._history) and blob:
            self._history[tile_index].samples.append(blob)

    def fill_misses(self, canvas: Canvas) -> list[int]:
        """Walk the canvas and overlay any stale tile slot with a cached
        blob (mode of the rolling window — byte-equality-based median).

        Returns the list of tile indices that were filled. Tiles with no
        history at all are left untouched so the operator sees a real gap
        rather than a synthesised one.
        """
        now = self._clock_ms()
        filled: list[int] = []
        # Iterate the canvas via its internal layout so we can spot empty
        # slots too (snapshot() skips slots that never received anything).
        for index in range(canvas.n_tiles):
            slot = canvas._tiles[index]  # noqa: SLF001 - canvas is sibling module
            age_ms = now - slot.arrived_ms if slot.arrived_ms else None
            if age_ms is not None and age_ms < self.stale_after_ms:
                # Tile is fresh; record it for the median pool and move on.
                if slot.badge == Badge.RAW:
                    self.observe(index, slot.blob)
                continue
            samples = self._history[index].samples
            if not samples:
                continue
            best, _count = Counter(samples).most_common(1)[0]
            canvas.overlay(index, best, badge=Badge.CACHED, arrived_ms=now)
            filled.append(index)
        return filled
