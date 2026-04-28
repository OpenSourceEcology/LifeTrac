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
from typing import BinaryIO

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
