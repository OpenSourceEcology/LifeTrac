"""Shared parser for the on-air TileDeltaFrame payload.

Wire format (matches ``firmware/tractor_x8/camera_service.py:_build_frame``
and ``LORA_PROTOCOL.md § TileDeltaFrame``):

    header (5 + bitmap_bytes):
        u8  frame_kind      ; 1=keyframe (I), 0=delta (P)
        u8  base_seq        ; rolls 0..255
        u8  grid_w          ; tiles across (12)
        u8  grid_h          ; tiles down  (8)
        u8  tile_px         ; pixels per tile edge (32)
        u8  changed_bitmap[(grid_w*grid_h + 7)//8]

    body (zero or more, in row-major order over the SET bits):
        u8  tile_size_minus1     ; webp blob length is this+1, max 256 B
        u8  webp_blob[size]

This module deliberately has zero dependencies on PIL/cryptography/paho so it
can be used inside the bridge process, the web_ui process, the offline
``replay_pcap.py`` tool, and the unit tests without pulling in the full
image stack.
"""
from __future__ import annotations

import struct
from dataclasses import dataclass, field
from typing import Iterable

FRAME_KIND_KEY = 1
FRAME_KIND_DELTA = 0
HEADER_FIXED_LEN = 5


@dataclass
class TileBlob:
    """One encoded tile inside a TileDeltaFrame."""
    index: int        # row-major index into the (grid_w, grid_h) grid
    tx: int
    ty: int
    blob: bytes       # raw WebP bytes, ready to hand to a decoder


@dataclass
class TileDeltaFrame:
    """Decoded TileDeltaFrame ready for the canvas / state publisher."""
    frame_kind: int
    base_seq: int
    grid_w: int
    grid_h: int
    tile_px: int
    changed_indices: list[int] = field(default_factory=list)
    tiles: list[TileBlob] = field(default_factory=list)

    @property
    def is_keyframe(self) -> bool:
        return self.frame_kind == FRAME_KIND_KEY

    @property
    def n_tiles(self) -> int:
        return self.grid_w * self.grid_h

    @property
    def canvas_w(self) -> int:
        return self.grid_w * self.tile_px

    @property
    def canvas_h(self) -> int:
        return self.grid_h * self.tile_px


class FrameDecodeError(ValueError):
    """Raised when a TileDeltaFrame is malformed."""


def parse_tile_delta_frame(payload: bytes) -> TileDeltaFrame:
    """Parse a complete TileDeltaFrame payload into a :class:`TileDeltaFrame`.

    Raises :class:`FrameDecodeError` on truncation, oversize tiles, or
    grid dimensions outside of the v25 envelope.
    """
    if len(payload) < HEADER_FIXED_LEN:
        raise FrameDecodeError(f"payload too short: {len(payload)} < {HEADER_FIXED_LEN}")
    frame_kind, base_seq, grid_w, grid_h, tile_px = struct.unpack_from(
        "BBBBB", payload, 0)
    if frame_kind not in (FRAME_KIND_KEY, FRAME_KIND_DELTA):
        raise FrameDecodeError(f"bad frame_kind {frame_kind}")
    if not (1 <= grid_w <= 32 and 1 <= grid_h <= 32):
        raise FrameDecodeError(f"grid {grid_w}x{grid_h} out of range")
    if tile_px == 0:
        raise FrameDecodeError("tile_px must be > 0")

    n_tiles = grid_w * grid_h
    bitmap_len = (n_tiles + 7) // 8
    bitmap_off = HEADER_FIXED_LEN
    body_off = bitmap_off + bitmap_len
    if len(payload) < body_off:
        raise FrameDecodeError(
            f"payload truncated: need {body_off} for header+bitmap, got {len(payload)}")

    bitmap = payload[bitmap_off:body_off]
    changed: list[int] = []
    for i in range(n_tiles):
        if bitmap[i // 8] & (1 << (i % 8)):
            changed.append(i)

    tiles: list[TileBlob] = []
    cursor = body_off
    for tile_index in changed:
        if cursor >= len(payload):
            raise FrameDecodeError(
                f"missing tile blob at index {tile_index}, cursor {cursor}")
        size = payload[cursor] + 1
        cursor += 1
        if cursor + size > len(payload):
            raise FrameDecodeError(
                f"tile blob {tile_index} truncated: need {size} from {cursor}, "
                f"have {len(payload) - cursor}")
        ty, tx = divmod(tile_index, grid_w)
        tiles.append(TileBlob(tile_index, tx, ty, bytes(payload[cursor:cursor + size])))
        cursor += size

    if cursor != len(payload):
        # Trailing bytes are unexpected; surface as malformed so the bridge
        # can request a keyframe.
        raise FrameDecodeError(
            f"trailing bytes after tiles: {len(payload) - cursor}")

    return TileDeltaFrame(
        frame_kind=frame_kind,
        base_seq=base_seq,
        grid_w=grid_w,
        grid_h=grid_h,
        tile_px=tile_px,
        changed_indices=changed,
        tiles=tiles,
    )


def encode_tile_delta_frame(frame: TileDeltaFrame) -> bytes:
    """Round-trip helper used by tests and the synthetic-injector in
    fallback_render so the canvas/reassembler can be exercised without a
    real tractor."""
    n_tiles = frame.grid_w * frame.grid_h
    bitmap = bytearray((n_tiles + 7) // 8)
    seen = set()
    for tile in frame.tiles:
        if not (0 <= tile.index < n_tiles):
            raise FrameDecodeError(f"tile index {tile.index} out of range")
        if tile.index in seen:
            raise FrameDecodeError(f"duplicate tile index {tile.index}")
        seen.add(tile.index)
        bitmap[tile.index // 8] |= (1 << (tile.index % 8))
    out = bytearray()
    out += struct.pack("BBBBB",
                       frame.frame_kind, frame.base_seq,
                       frame.grid_w, frame.grid_h, frame.tile_px)
    out += bytes(bitmap)
    # Tiles must be emitted in ascending index order (matches the camera).
    for tile in sorted(frame.tiles, key=lambda t: t.index):
        if len(tile.blob) == 0 or len(tile.blob) > 256:
            raise FrameDecodeError(f"tile blob length {len(tile.blob)} out of [1,256]")
        out.append(len(tile.blob) - 1)
        out += tile.blob
    return bytes(out)
