"""Persistent tile canvas + base_seq mismatch detector.

The canvas keeps the most-recent encoded blob per tile plus the wall-clock
time it arrived (for the staleness overlay). It also tracks the keyframe
chain: every delta frame must build on the keyframe that started its `base_seq`
counter; if we miss a keyframe (e.g. the radio dropped the I-frame), the
canvas surfaces a "needs keyframe" condition so the bridge can publish
`CMD_REQ_KEYFRAME` (opcode `0x62`).

Per IMAGE_PIPELINE.md §3.3 every published tile must carry a Badge enum.
We default to ``Badge.RAW`` for tiles arriving direct from the tractor;
``bg_cache.fill_misses()`` and ``recolourise.apply()`` overlay the other
badges in their own modules.
"""
from __future__ import annotations

import logging
import time
from collections import OrderedDict
from dataclasses import dataclass, field
from typing import Callable, Iterator

from lora_proto import Badge

from .codec_decode import CodecDecodeError, transcode_to_webp
from .frame_format import CODEC_MONO_G4, CODEC_WEBP, CODEC_WEBP_LUMA, TileDeltaFrame

LOG = logging.getLogger(__name__)

# Per-frame codec -> default Badge applied to every tile from that frame.
# CODEC_WEBP is the colour baseline (RAW); CODEC_WEBP_LUMA carries a
# luma-only WebP that the operator should see tagged as RECOLOURISED so
# they know the colour is held over (Plan A / IMAGE_PIPELINE.md §8). The
# 1-bit dither codec lands in the WIREFRAME bucket per the same §3.3.
_CODEC_BADGE: dict[int, Badge] = {
    CODEC_WEBP:       Badge.RAW,
    CODEC_WEBP_LUMA:  Badge.RECOLOURISED,
    CODEC_MONO_G4:    Badge.WIREFRAME,
}

# Max distinct (codec, blob) tuples to remember when transcoding non-WebP
# tiles. 256 covers ~2.6 full keyframes worth of unique tiles, which is
# more than enough to absorb a static scene re-shipping the same blobs.
_TRANSCODE_CACHE_MAX = 256


@dataclass
class TileState:
    """Per-tile canvas slot."""
    blob: bytes = b""
    arrived_ms: int = 0
    badge: Badge = Badge.RAW
    age_ms_at_publish: int = 0


@dataclass
class CanvasUpdate:
    """Result of `Canvas.apply()` — fed straight into `state_publisher`."""
    base_seq: int
    is_keyframe: bool
    updated_indices: list[int] = field(default_factory=list)
    request_keyframe: bool = False
    reason: str = ""


class Canvas:
    """In-memory tile store. One per active camera."""

    def __init__(
        self,
        grid_w: int = 12,
        grid_h: int = 8,
        tile_px: int = 32,
        clock_ms: Callable[[], int] | None = None,
    ) -> None:
        self.grid_w = grid_w
        self.grid_h = grid_h
        self.tile_px = tile_px
        self._tiles: list[TileState] = [TileState() for _ in range(grid_w * grid_h)]
        self._clock_ms = clock_ms or (lambda: int(time.monotonic() * 1000))
        self._last_base_seq: int | None = None
        self._has_keyframe = False
        # (codec, blob) -> transcoded WebP blob. LRU-trimmed.
        self._transcode_cache: "OrderedDict[tuple[int, bytes], bytes]" = OrderedDict()

    @property
    def n_tiles(self) -> int:
        return self.grid_w * self.grid_h

    @property
    def has_keyframe(self) -> bool:
        return self._has_keyframe

    def apply(self, frame: TileDeltaFrame) -> CanvasUpdate:
        """Merge a parsed frame into the canvas.

        Returns a :class:`CanvasUpdate` describing what changed and whether a
        keyframe must be requested. The caller (bridge or pipeline driver)
        is responsible for actually publishing `CMD_REQ_KEYFRAME` on MQTT —
        we keep this module free of broker dependencies.
        """
        # Reject grids that don't match. The camera_service guarantees a
        # fixed grid; if it ever changes we want a keyframe before honouring
        # the new layout.
        if (frame.grid_w, frame.grid_h, frame.tile_px) != (
                self.grid_w, self.grid_h, self.tile_px):
            return CanvasUpdate(
                base_seq=frame.base_seq,
                is_keyframe=frame.is_keyframe,
                request_keyframe=True,
                reason=f"grid mismatch ({frame.grid_w}x{frame.grid_h}@{frame.tile_px}px)",
            )

        update = CanvasUpdate(
            base_seq=frame.base_seq,
            is_keyframe=frame.is_keyframe,
        )
        now = self._clock_ms()

        if frame.is_keyframe:
            self._has_keyframe = True
            self._last_base_seq = frame.base_seq
        else:
            if not self._has_keyframe:
                update.request_keyframe = True
                update.reason = "delta arrived before any keyframe"
                return update
            expected = ((self._last_base_seq or 0) + 1) & 0xFF
            if frame.base_seq != expected:
                # IP-PlanRev gap-tolerant merge: a base_seq gap means one or
                # more intermediate P-frames were lost in transport (LoRa
                # reassembler timeout / fragment drop). The tiles inside
                # THIS frame are still valid pixels — they're just snapshots
                # of those tile positions at a later moment. Refusing to
                # apply them (the pre-IP-PlanRev behaviour) used to starve
                # the canvas down to keyframe-only coverage whenever the
                # link was lossy, since every gap caused a full-frame drop.
                # Now we apply the tiles AND request a keyframe in parallel
                # so the operator still sees motion immediately.
                update.request_keyframe = True
                update.reason = (
                    f"base_seq gap: got {frame.base_seq}, expected {expected}")
            self._last_base_seq = frame.base_seq

        for tile in frame.tiles:
            try:
                webp_blob = self._transcoded(frame.codec, tile.blob)
            except Exception as exc:
                # One bad tile must never kill the frame: an exception
                # escaping here unwinds AFTER _last_base_seq was adopted
                # and BEFORE the remaining tiles apply, silently freezing
                # the canvas while it appears to track the stream (seen on
                # air 2026-08-01, F10 acceptance §6). transcode_to_webp
                # folds transcoder failures into CodecDecodeError; this
                # broad catch is the invariant's backstop.
                LOG.warning("canvas: dropping tile %d codec=%d: %s",
                            tile.index, frame.codec, exc)
                update.request_keyframe = True
                update.reason = update.reason or f"codec_decode_error: {exc}"
                continue
            slot = self._tiles[tile.index]
            slot.blob = webp_blob
            slot.arrived_ms = now
            slot.badge = _CODEC_BADGE.get(frame.codec, Badge.RAW)
            update.updated_indices.append(tile.index)

        return update

    def _transcoded(self, codec: int, blob: bytes) -> bytes:
        """Return the browser-WebP form of ``blob`` for the given codec.

        Caches by ``(codec, blob)`` so a static scene re-shipping the
        same tile bytes doesn't re-pay the PIL decode cost on every
        refresh. CODEC_WEBP short-circuits the cache because identity
        transcode is already free.
        """
        if codec == CODEC_WEBP:
            return blob
        key = (codec, blob)
        hit = self._transcode_cache.get(key)
        if hit is not None:
            self._transcode_cache.move_to_end(key)
            return hit
        out = transcode_to_webp(codec, blob, self.tile_px)
        self._transcode_cache[key] = out
        if len(self._transcode_cache) > _TRANSCODE_CACHE_MAX:
            self._transcode_cache.popitem(last=False)
        return out

    def snapshot(self) -> Iterator[tuple[int, TileState]]:
        """Yield ``(index, TileState)`` for every populated tile.

        The state-publisher uses this to build its outgoing WS payload.
        ``age_ms_at_publish`` is filled in here so downstream consumers
        (browser canvas) never compute their own clocks.
        """
        now = self._clock_ms()
        for idx, slot in enumerate(self._tiles):
            if slot.arrived_ms == 0:
                continue
            slot.age_ms_at_publish = max(0, now - slot.arrived_ms)
            yield idx, slot

    def overlay(self, index: int, blob: bytes, badge: Badge,
                arrived_ms: int | None = None) -> None:
        """Used by ``bg_cache.fill_misses`` / ``recolourise.apply`` etc.

        Caller passes a non-RAW badge so the trust boundary in IMAGE_PIPELINE
        §6.1 is preserved (the browser must refuse-to-display tiles with no
        badge, and treat non-RAW as "do not auto-act on this pixel").
        """
        if not (0 <= index < self.n_tiles):
            raise IndexError(index)
        if badge == Badge.RAW:
            raise ValueError("overlay() must use a non-RAW badge")
        slot = self._tiles[index]
        slot.blob = blob
        slot.arrived_ms = arrived_ms if arrived_ms is not None else self._clock_ms()
        slot.badge = badge

    def reset(self) -> None:
        """Drop everything (e.g. on camera switch)."""
        for slot in self._tiles:
            slot.blob = b""
            slot.arrived_ms = 0
            slot.badge = Badge.RAW
        self._last_base_seq = None
        self._has_keyframe = False
