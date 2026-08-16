"""IPC ring-buffer hand-off from the X8 (Linux side) to the H747 M7 firmware.

The M7 reads encoded image fragments out of a UART (or USB-CDC) ring buffer
and pushes them onto the LoRa radio at P3 priority. From Linux we open the
character device and stream length-prefixed frames over it. The protocol is
deliberately tiny so a microcontroller can parse it with no allocations:

    repeated:
        u8  marker            ; 0xA5 — start of frame
        u8  flags             ; bit0 = is_keyframe, bit1 = is_motion, bit2 = is_wireframe
        u16 length            ; little-endian, payload length in bytes
        u8  payload[length]
        u8  crc8              ; CRC-8/SMBUS over flags|length|payload

The payload itself is a TileDeltaFrame (or motion / wireframe variant) that
the M7 then chops into ≤25 ms-airtime LoRa fragments using the protocol's
fragment header.

Pure Python; opens the device path on construction and never blocks longer
than a single write. If the device is missing (e.g. running on a laptop)
the writer falls back to `/dev/null` and logs a warning.
"""
from __future__ import annotations

import logging
import os
import struct
from dataclasses import dataclass
from typing import BinaryIO, Iterator

LOG = logging.getLogger("ipc_to_h747")

FRAME_MARKER = 0xA5
FLAG_KEYFRAME = 0x01
FLAG_MOTION = 0x02
FLAG_WIREFRAME = 0x04


def crc8_smbus(data: bytes, init: int = 0x00) -> int:
    """CRC-8/SMBUS (poly 0x07, init 0x00, no refin/refout, no xorout)."""
    crc = init & 0xFF
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = ((crc << 1) ^ 0x07) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
    return crc


def encode_ipc_frame(payload: bytes, *, is_keyframe: bool = False,
                     is_motion: bool = False, is_wireframe: bool = False) -> bytes:
    if len(payload) > 0xFFFF:
        raise ValueError(f"payload too large: {len(payload)} > 65535")
    flags = 0
    if is_keyframe: flags |= FLAG_KEYFRAME
    if is_motion: flags |= FLAG_MOTION
    if is_wireframe: flags |= FLAG_WIREFRAME
    body = struct.pack("<BH", flags, len(payload)) + payload
    return bytes([FRAME_MARKER]) + body + bytes([crc8_smbus(body)])


class IpcWriter:
    """Open a TTY (or any file path) and stream encoded frames to it."""

    def __init__(self, device: str = "/dev/ttymxc1") -> None:
        self.device = device
        self._fh: BinaryIO | None = None

    def open(self) -> None:
        if self._fh is not None:
            return
        try:
            self._fh = open(self.device, "wb", buffering=0)
        except OSError as exc:
            LOG.warning("ipc_to_h747: cannot open %s (%s); falling back to /dev/null",
                        self.device, exc)
            self._fh = open(os.devnull, "wb", buffering=0)

    def close(self) -> None:
        if self._fh is not None:
            try:
                self._fh.close()
            finally:
                self._fh = None

    def write(self, payload: bytes, *, is_keyframe: bool = False,
              is_motion: bool = False, is_wireframe: bool = False) -> int:
        if self._fh is None:
            self.open()
        frame = encode_ipc_frame(payload,
                                  is_keyframe=is_keyframe,
                                  is_motion=is_motion,
                                  is_wireframe=is_wireframe)
        assert self._fh is not None
        return self._fh.write(frame)


# ---- decoder side (used by tests + the offline replay tools) ---------
#
# The M7 firmware does this decode in C++ on the UART RX path. The Python
# implementation here is byte-for-byte equivalent so the host-side tests
# can verify that what camera_service emits will actually round-trip
# through the framing the M7 expects. Tolerant of leading garbage and of
# being handed a partial buffer mid-frame: the decoder resyncs on the
# next 0xA5 marker, drops any frame whose CRC doesn't match, and reports
# how many input bytes it consumed so a streaming caller can advance its
# read cursor.


class IpcFrameError(ValueError):
    """Raised by :func:`decode_one_ipc_frame` when the buffer holds a
    malformed-but-fully-bounded frame (bad CRC or impossible length).
    Truncation returns ``(None, 0)`` instead so the caller can wait for
    more bytes."""


@dataclass
class DecodedIpcFrame:
    flags: int
    payload: bytes

    @property
    def is_keyframe(self) -> bool:
        return bool(self.flags & FLAG_KEYFRAME)

    @property
    def is_motion(self) -> bool:
        return bool(self.flags & FLAG_MOTION)

    @property
    def is_wireframe(self) -> bool:
        return bool(self.flags & FLAG_WIREFRAME)


def decode_one_ipc_frame(buf: bytes) -> tuple[DecodedIpcFrame | None, int]:
    """Try to decode one frame from the head of ``buf``.

    Returns ``(frame, consumed)``:
      * ``frame=None, consumed=0`` — not enough bytes yet (caller should
        accumulate more and retry).
      * ``frame=DecodedIpcFrame, consumed=N`` — successfully parsed, drop
        the first ``N`` bytes from the buffer.
      * Raises :class:`IpcFrameError` when a fully-bounded frame fails
        CRC. The caller should advance one byte past the bad marker and
        retry; :func:`iter_ipc_frames` does that for you.
    """
    # Skip junk until we find a marker.
    i = 0
    while i < len(buf) and buf[i] != FRAME_MARKER:
        i += 1
    if i >= len(buf):
        # All junk; consume nothing — wait for more.
        return None, 0
    # Need at least marker + flags + length(2) + crc = 5 bytes before payload.
    if len(buf) - i < 5:
        return None, 0
    flags = buf[i + 1]
    length = struct.unpack_from("<H", buf, i + 2)[0]
    end = i + 4 + length + 1  # +1 for trailing CRC
    if len(buf) < end:
        return None, 0
    payload = bytes(buf[i + 4:i + 4 + length])
    body = bytes(buf[i + 1:i + 4 + length])  # flags|length|payload
    expected = buf[i + 4 + length]
    actual = crc8_smbus(body)
    if expected != actual:
        raise IpcFrameError(
            f"crc mismatch at offset {i}: expected 0x{expected:02x}, got 0x{actual:02x}")
    return DecodedIpcFrame(flags=flags, payload=payload), end


def iter_ipc_frames(buf: bytes) -> Iterator[DecodedIpcFrame]:
    """Yield every well-formed frame in ``buf``. Bad-CRC frames are
    silently skipped (logged at debug level) and the scan resumes one
    byte past the failed marker — same behaviour the M7 has on a
    physical-layer glitch."""
    cursor = 0
    while cursor < len(buf):
        try:
            frame, consumed = decode_one_ipc_frame(buf[cursor:])
        except IpcFrameError as exc:
            LOG.debug("ipc_to_h747: dropping bad frame at %d: %s", cursor, exc)
            # Skip this marker byte and keep looking.
            cursor += 1
            continue
        if frame is None:
            return
        yield frame
        cursor += consumed
