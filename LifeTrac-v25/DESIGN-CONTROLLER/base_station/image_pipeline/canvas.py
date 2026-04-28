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

import time
from dataclasses import dataclass, field
from typing import Callable, Iterator

from lora_proto import Badge

from .frame_format import TileDeltaFrame


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
                update.request_keyframe = True
                update.reason = (
                    f"base_seq gap: got {frame.base_seq}, expected {expected}")
                # Don't mutate canvas on a bad chain — wait for keyframe.
                return update
            self._last_base_seq = frame.base_seq

        for tile in frame.tiles:
            slot = self._tiles[tile.index]
            slot.blob = tile.blob
            slot.arrived_ms = now
            slot.badge = Badge.RAW
            update.updated_indices.append(tile.index)

        return update

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
