"""Render PiDiNet wireframe overlays (topic 0x29).

P on the fallback ladder. The tractor sends a single packed bitmap of
edge pixels at canvas resolution; the base side overlays it on top of
whatever the canvas currently shows (typically last good keyframe + cache).

Wire format for one wireframe frame (matches LORA_PROTOCOL.md §0x29):

    u8  base_seq                 ; mirrors the I-frame chain
    u16 width                    ; little-endian, must equal canvas width
    u16 height                   ; ditto for height
    u8  bitmap[ceil(w*h/8)]      ; 1 = edge pixel, packed little-endian

We store the most-recent bitmap so ``state_publisher`` can hand it to the
browser as a single base64 blob; ``wireframe_render.js`` blits it as an
alpha-blended overlay. The canvas slots themselves are not modified —
wireframes are always supplemental.
"""
from __future__ import annotations

import struct
import time
from dataclasses import dataclass
from typing import Callable


class WireframeDecodeError(ValueError):
    pass


@dataclass
class WireframeFrame:
    base_seq: int
    width: int
    height: int
    bitmap: bytes


def parse_wireframe_frame(payload: bytes) -> WireframeFrame:
    if len(payload) < 5:
        raise WireframeDecodeError("payload < 5 bytes")
    base_seq, width, height = struct.unpack_from("<BHH", payload, 0)
    bitmap_len = (width * height + 7) // 8
    if len(payload) < 5 + bitmap_len:
        raise WireframeDecodeError(
            f"bitmap truncated: need {bitmap_len}, got {len(payload) - 5}")
    return WireframeFrame(base_seq=base_seq, width=width, height=height,
                          bitmap=bytes(payload[5:5 + bitmap_len]))


class WireframeRenderer:
    """Thin wrapper holding the most recent wireframe for the publisher."""

    def __init__(
        self,
        canvas_w: int,
        canvas_h: int,
        clock_ms: Callable[[], int] | None = None,
    ) -> None:
        self.canvas_w = canvas_w
        self.canvas_h = canvas_h
        self._clock_ms = clock_ms or (lambda: int(time.monotonic() * 1000))
        self.latest: WireframeFrame | None = None
        self.received_ms: int = 0

    def apply(self, frame: WireframeFrame) -> bool:
        if frame.width != self.canvas_w or frame.height != self.canvas_h:
            return False
        self.latest = frame
        self.received_ms = self._clock_ms()
        return True

    def age_ms(self) -> int | None:
        if self.latest is None:
            return None
        return max(0, self._clock_ms() - self.received_ms)
