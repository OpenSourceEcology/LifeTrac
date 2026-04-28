"""Apply ``video/motion_vectors`` (topic 0x28) to the existing canvas.

When the link is bad enough that we drop to ``motion_only`` mode (Q on the
fallback ladder, IMAGE_PIPELINE.md §3.4), the tractor sends a per-tile dx,dy
vector pair instead of a re-encoded WebP. The base side replays those
vectors against whatever blob was already in the canvas slot, painting the
result back with ``Badge.PREDICTED`` so the operator knows the pixels are
extrapolated, not freshly observed.

Wire format for one motion-vector frame (matches LORA_PROTOCOL.md §0x28):

    u8  base_seq                 ; rolls 0..255, mirrors the I-frame chain
    u8  count                    ; number of (idx, dx, dy) triplets to follow
    repeated count times:
        u16 tile_index           ; little-endian, < grid_w*grid_h
        i8  dx                   ; -64..63 px
        i8  dy
"""
from __future__ import annotations

import struct
import time
from dataclasses import dataclass
from typing import Callable, Iterable

from lora_proto import Badge

from .canvas import Canvas


@dataclass
class MotionVector:
    tile_index: int
    dx: int
    dy: int


@dataclass
class MotionFrame:
    base_seq: int
    vectors: list[MotionVector]


class MotionFrameDecodeError(ValueError):
    pass


def parse_motion_frame(payload: bytes) -> MotionFrame:
    if len(payload) < 2:
        raise MotionFrameDecodeError("payload < 2 bytes")
    base_seq, count = payload[0], payload[1]
    expected = 2 + count * 4
    if len(payload) < expected:
        raise MotionFrameDecodeError(
            f"truncated: need {expected} bytes for {count} vectors, got {len(payload)}")
    vectors: list[MotionVector] = []
    cursor = 2
    for _ in range(count):
        idx, dx, dy = struct.unpack_from("<Hbb", payload, cursor)
        vectors.append(MotionVector(tile_index=idx, dx=dx, dy=dy))
        cursor += 4
    return MotionFrame(base_seq=base_seq, vectors=vectors)


class MotionReplayer:
    """Apply vectors into the canvas with ``Badge.PREDICTED``.

    We do *not* actually transform the pixels here — that would require
    decoding/re-encoding the WebP every refresh, which is the whole reason
    the encoder dropped to motion_only in the first place. Instead we
    annotate the slot with the latest vector and bump arrived_ms so the
    staleness overlay reflects "I have new information about this tile, even
    if it's only the motion of the previous frame's pixels". The browser's
    ``canvas_renderer.js`` reads the per-tile dx/dy out of state_publisher
    and applies a CSS transform — pure display polish, never autopilot.
    """

    def __init__(
        self,
        clock_ms: Callable[[], int] | None = None,
    ) -> None:
        self._clock_ms = clock_ms or (lambda: int(time.monotonic() * 1000))
        # Per-tile latest motion vector for the state publisher to expose.
        self.latest_vectors: dict[int, tuple[int, int]] = {}

    def apply(self, canvas: Canvas, frame: MotionFrame) -> list[int]:
        """Returns the list of tile indices that were touched."""
        touched: list[int] = []
        now = self._clock_ms()
        for vec in frame.vectors:
            if not (0 <= vec.tile_index < canvas.n_tiles):
                continue
            slot = canvas._tiles[vec.tile_index]   # noqa: SLF001
            if not slot.blob:
                # Nothing to predict from yet; skip.
                continue
            self.latest_vectors[vec.tile_index] = (vec.dx, vec.dy)
            canvas.overlay(vec.tile_index, slot.blob,
                            badge=Badge.PREDICTED, arrived_ms=now)
            touched.append(vec.tile_index)
        return touched

    def snapshot_vectors(self) -> dict[int, tuple[int, int]]:
        return dict(self.latest_vectors)
