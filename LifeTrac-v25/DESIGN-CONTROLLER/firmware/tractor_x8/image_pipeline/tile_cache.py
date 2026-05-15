"""IP-W2-05: per-tile WebP encode cache for the X8 encoder.

Tile diffs are byte-coarse: a single noisy pixel re-flips the bit and we
re-encode the whole 32×32 RGB slice through PIL/WebP even though the
result is byte-identical to a frame we encoded a few hundred ms ago.
This module maintains a tiny per-tile LRU keyed by the *raw pixel hash*
of the slice — when the hash matches a recent entry the cached blob is
returned and the WebP encoder is skipped entirely.

This is purely a CPU saver. The wire payload is unchanged; the base
station's :class:`image_pipeline.bg_cache.BackgroundCache` is what
decides to *reuse* the rendered tile across frames. We do not drop a
tile from ``kept`` here — the encoder owns that decision via the byte
budget — we only avoid spending CPU on a re-encode that would round-trip
to the same blob.

Cache is opt-in: ``_build_frame(encode_cache=None)`` is the historical
behaviour and matches what the W2-03 / W2-04 tests pin.

Hash function: ``hashlib.blake2b(digest_size=16)``. Cheap, available in
the Python 3.10+ stdlib that ships on the LmP X8, and 128 bits is
plenty to keep collision probability below 2⁻⁶⁴ at our cache sizes
(4–8 entries × 96 tiles).
"""
from __future__ import annotations

import hashlib
from collections import OrderedDict
from dataclasses import dataclass


DEFAULT_HISTORY = 4


@dataclass
class TileCacheStats:
    hits: int = 0
    misses: int = 0
    evictions: int = 0


class TileEncodeCache:
    """Per-tile LRU mapping ``raw_pixel_hash -> encoded_blob``.

    The cache is bucketed by ``tile_index`` so a tile's history doesn't
    get pushed out by a different tile's churn — at 96 tiles × 4 entries
    that's ≈ 100 KiB worst case for 256 B blobs, well within the X8 RAM
    envelope.
    """

    def __init__(self, n_tiles: int, history: int = DEFAULT_HISTORY) -> None:
        if n_tiles <= 0:
            raise ValueError("n_tiles must be positive")
        if history <= 0:
            raise ValueError("history must be positive")
        self._history = history
        # OrderedDict gives O(1) move-to-end + popitem(last=False) for LRU.
        self._buckets: list[OrderedDict[bytes, bytes]] = [
            OrderedDict() for _ in range(n_tiles)
        ]
        self.stats = TileCacheStats()

    @staticmethod
    def hash_pixels(raw_pixels: bytes) -> bytes:
        return hashlib.blake2b(raw_pixels, digest_size=16).digest()

    def lookup(self, tile_index: int, raw_pixels: bytes) -> bytes | None:
        """Return the cached blob for these pixels, or ``None`` on miss."""
        bucket = self._buckets[tile_index]
        key = self.hash_pixels(raw_pixels)
        blob = bucket.get(key)
        if blob is None:
            self.stats.misses += 1
            return None
        bucket.move_to_end(key)
        self.stats.hits += 1
        return blob

    def store(self, tile_index: int, raw_pixels: bytes, blob: bytes) -> None:
        """Record an encoded blob; evicts LRU when bucket is full."""
        bucket = self._buckets[tile_index]
        key = self.hash_pixels(raw_pixels)
        if key in bucket:
            bucket.move_to_end(key)
            bucket[key] = blob
            return
        bucket[key] = blob
        while len(bucket) > self._history:
            bucket.popitem(last=False)
            self.stats.evictions += 1

    def reset(self) -> None:
        for b in self._buckets:
            b.clear()
        self.stats = TileCacheStats()
