"""PiDiNet edge encoder for video/wireframe (topic 0x29).

P on the IMAGE_PIPELINE.md §3.4 fallback ladder. When the link is so bad
that even motion vectors don't fit, we ship a packed bitmap of edge pixels
at the canvas resolution. The tractor X8 doesn't actually run the PiDiNet
neural network here — that lives in a separate accelerated process. This
module formats whatever single-channel edge buffer it is given into the
on-air payload.

Wire format mirrors :mod:`base_station.image_pipeline.wireframe_render`:

    u8  base_seq                 ; mirrors I-frame chain
    u16 width                    ; little-endian, must equal canvas width
    u16 height                   ; ditto
    u8  bitmap[ceil(w*h/8)]      ; 1 = edge, packed little-endian
"""
from __future__ import annotations

import struct
from typing import Iterable


def pack_wireframe_from_threshold(base_seq: int, edge_plane: bytes,
                                  width: int, height: int,
                                  threshold: int = 64) -> bytes:
    """Build a topic-0x29 payload by thresholding a single-channel edge map.

    `edge_plane` is `width*height` bytes in row-major order; pixels with
    value > `threshold` are marked as edge.
    """
    if len(edge_plane) != width * height:
        raise ValueError(f"edge_plane size {len(edge_plane)} != {width*height}")
    bitmap_len = (width * height + 7) // 8
    bitmap = bytearray(bitmap_len)
    for i, v in enumerate(edge_plane):
        if v > threshold:
            bitmap[i // 8] |= (1 << (i % 8))
    return struct.pack("<BHH", base_seq & 0xFF, width, height) + bytes(bitmap)


def pack_wireframe_from_indices(base_seq: int, edge_indices: Iterable[int],
                                width: int, height: int) -> bytes:
    """Build a topic-0x29 payload from a list of edge-pixel linear indices."""
    bitmap_len = (width * height + 7) // 8
    bitmap = bytearray(bitmap_len)
    n = width * height
    for i in edge_indices:
        if 0 <= i < n:
            bitmap[i // 8] |= (1 << (i % 8))
    return struct.pack("<BHH", base_seq & 0xFF, width, height) + bytes(bitmap)
