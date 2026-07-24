"""Per-frame codec dispatch for the base-station canvas.

The tractor publishes ``TileDeltaFrame`` payloads tagged with a 1 B codec
id (``frame_format.CODEC_*``). The browser only knows WebP, so any non-
WebP codec must be *transcoded* by the base station before the tile blob
enters the canvas / WebSocket stream.

This module is the single dispatch point. New codecs land here and in
``firmware/tractor_x8/camera_service.py`` together; the rest of the
canvas/state-publisher stack stays codec-agnostic.

Codec format details
====================

CODEC_WEBP (0)
    Pass-through. Blob is already a WebP container the browser can paint.

CODEC_MONO_G4 (1)
    1-bit Floyd-Steinberg-dithered tile, packed MSB-first row-major,
    optionally zlib-compressed. Wire layout::

        u8  flag        ; bit0: 0 = raw bits follow, 1 = zlib payload
        u8  payload[]   ; (tile_px*tile_px + 7)//8 raw bytes, or zlib

    Decoded to a 32x32 grayscale image (0=black, 255=white) and re-
    encoded as a WebP RGB tile so the browser doesn't need a new path.

CODEC_BTC4_PER_TILE (2)
    Not yet implemented (lands with the BTC4 vertical slice).

CODEC_BTC4_PER_FRAME (3)
    Not yet implemented (palette prefix lives in the frame body, not the
    per-tile blob, so this codec needs both a frame-level prelude
    parser and a per-tile decoder).
"""
from __future__ import annotations

import io
import zlib
from typing import Callable

from .frame_format import (
    CODEC_BTC4_PER_FRAME,
    CODEC_BTC4_PER_TILE,
    CODEC_MONO_G4,
    CODEC_WEBP,
    CODEC_WEBP_LUMA,
    CODEC_WEBP_RAWSTREAM,
)


class CodecDecodeError(Exception):
    """Raised when a tile blob cannot be decoded for its declared codec."""


def _decode_mono_g4(blob: bytes, tile_px: int) -> bytes:
    """Return the grayscale WebP for a MONO_G4 tile blob."""
    if len(blob) < 1:
        raise CodecDecodeError("mono_g4 blob empty")
    flag = blob[0]
    payload = blob[1:]
    bits_needed = tile_px * tile_px
    bytes_needed = (bits_needed + 7) // 8
    if flag & 0x01:
        try:
            bits_packed = zlib.decompress(payload)
        except zlib.error as exc:
            raise CodecDecodeError(f"mono_g4 zlib: {exc}") from exc
    else:
        bits_packed = payload
    if len(bits_packed) < bytes_needed:
        raise CodecDecodeError(
            f"mono_g4 underrun: have {len(bits_packed)} need {bytes_needed}")
    # Expand 1bpp -> 8bpp grayscale (0 or 255), then RGB triplicate.
    out = bytearray(bits_needed)
    for i in range(bits_needed):
        byte = bits_packed[i >> 3]
        bit = (byte >> (7 - (i & 7))) & 0x01
        out[i] = 255 if bit else 0
    from PIL import Image  # type: ignore  # lazy: not needed for codec=0
    img = Image.frombytes("L", (tile_px, tile_px), bytes(out)).convert("RGB")
    buf = io.BytesIO()
    img.save(buf, format="WEBP", quality=70, method=4)
    return buf.getvalue()


def _decode_webp_luma(blob: bytes, tile_px: int) -> bytes:  # noqa: ARG001
    """Pass-through for Y_ONLY tiles.

    A CODEC_WEBP_LUMA blob is a valid grayscale WebP container; every
    modern browser paints it natively, so no transcode is required. The
    distinct codec id exists so the canvas / state-publisher can tag the
    tile with ``Badge.RECOLOURISED`` and so future base-side fusion
    (Recolouriser GLSL composite) can take over without changing the
    wire format.
    """
    return blob


def rewrap_webp(raw: bytes) -> bytes:
    """Re-wrap a stripped VP8/VP8L rawstream into a valid RIFF WebP container."""
    if not raw:
        raise CodecDecodeError("rawstream empty")
    fourcc = b"VP8L" if raw[:1] == b"\x2f" else b"VP8 "
    chunk = fourcc + len(raw).to_bytes(4, "little") + raw + (b"\x00" if len(raw) & 1 else b"")
    return b"RIFF" + (4 + len(chunk)).to_bytes(4, "little") + b"WEBP" + chunk


def _decode_webp_rawstream(blob: bytes, tile_px: int) -> bytes:  # noqa: ARG001
    """Re-wrap a stripped VP8/VP8L rawstream tile into a RIFF WebP container."""
    return rewrap_webp(blob)


# Codec id -> transcoder. Each transcoder takes (blob, tile_px) and
# returns a WebP blob the browser can paint. Codecs that are not yet
# implemented are absent so ``transcode_to_webp`` can raise a precise
# error instead of silently swallowing them.
_TRANSCODERS: dict[int, Callable[[bytes, int], bytes]] = {
    CODEC_MONO_G4: _decode_mono_g4,
    CODEC_WEBP_LUMA: _decode_webp_luma,
    CODEC_WEBP_RAWSTREAM: _decode_webp_rawstream,
}


def transcode_to_webp(codec: int, blob: bytes, tile_px: int) -> bytes:
    """Return a browser-renderable WebP blob for a tile.

    For ``CODEC_WEBP`` the input blob is passed through unchanged. For
    every other registered codec, the matching transcoder converts the
    blob to a small grayscale WebP. Unknown / unimplemented codecs raise
    :class:`CodecDecodeError`; the caller (canvas) is expected to drop
    the tile and request a keyframe so the link recovers.
    """
    if codec == CODEC_WEBP:
        return blob
    transcoder = _TRANSCODERS.get(codec)
    if transcoder is None:
        raise CodecDecodeError(
            f"codec id {codec} not implemented in this base-station build")
    return transcoder(blob, tile_px)
