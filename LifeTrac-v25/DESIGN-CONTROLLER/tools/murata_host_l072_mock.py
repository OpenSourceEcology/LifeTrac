#!/usr/bin/env python3
"""Simple L072 host-protocol mock for PTY loopback tests."""

from __future__ import annotations

import struct
from dataclasses import dataclass
from typing import Callable, List, Optional

try:
    import mh_wire_constants as mh
except ImportError as exc:  # pragma: no cover - guided by tooling
    raise RuntimeError(
        "mh_wire_constants.py not found. Run tools/gen_mh_wire_py.py first."
    ) from exc


def crc16_ccitt(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


def cobs_encode(data: bytes) -> bytes:
    out = bytearray()
    code_index = 0
    out.append(0)
    code = 1

    for byte in data:
        if byte == 0:
            out[code_index] = code
            code_index = len(out)
            out.append(0)
            code = 1
        else:
            out.append(byte)
            code += 1
            if code == 0xFF:
                out[code_index] = code
                code_index = len(out)
                out.append(0)
                code = 1

    out[code_index] = code
    return bytes(out)


def cobs_decode(data: bytes) -> Optional[bytes]:
    out = bytearray()
    i = 0
    n = len(data)

    while i < n:
        code = data[i]
        if code == 0:
            return None
        i += 1
        end = i + code - 1
        if end > n:
            return None

        out.extend(data[i:end])
        i = end
        if code < 0xFF and i < n:
            out.append(0)

    return bytes(out)


@dataclass
class InnerFrame:
    ver: int
    type: int
    flags: int
    seq: int
    payload: bytes


def build_inner(frame_type: int, flags: int, seq: int, payload: bytes, ver: Optional[int] = None) -> bytes:
    if ver is None:
        ver = int(mh.HOST_PROTOCOL_VER)
    header = struct.pack("<BBBHH", ver, frame_type, flags, seq & 0xFFFF, len(payload))
    inner_wo_crc = header + payload
    crc = crc16_ccitt(inner_wo_crc)
    return inner_wo_crc + struct.pack("<H", crc)


def parse_inner(inner: bytes) -> Optional[InnerFrame]:
    if len(inner) < int(mh.HOST_HEADER_LEN + mh.HOST_CRC_LEN):
        return None

    ver, frame_type, flags, seq, payload_len = struct.unpack_from("<BBBHH", inner, 0)
    expected = int(mh.HOST_HEADER_LEN + payload_len + mh.HOST_CRC_LEN)
    if expected != len(inner):
        return None

    crc_frame = struct.unpack_from("<H", inner, len(inner) - 2)[0]
    crc_calc = crc16_ccitt(inner[:-2])
    if crc_frame != crc_calc:
        return None

    payload = inner[int(mh.HOST_HEADER_LEN) : int(mh.HOST_HEADER_LEN) + payload_len]
    return InnerFrame(ver=ver, type=frame_type, flags=flags, seq=seq, payload=payload)


def wrap_outer(inner: bytes) -> bytes:
    return cobs_encode(inner) + b"\x00"


class MurataL072Mock:
    def __init__(self) -> None:
        self._stats_counter = 1
        self._emit_additive = False
        self._chunk_index = 0

    def startup_fuzz_frames(self) -> List[bytes]:
        frames: List[bytes] = []

        bad_crc = bytearray(build_inner(int(mh.HOST_TYPE_VER_URC), 0, 1, b"V"))
        bad_crc[-1] ^= 0x5A
        frames.append(wrap_outer(bytes(bad_crc)))

        bad_len = bytearray(build_inner(int(mh.HOST_TYPE_UID_URC), 0, 2, b"ABCD"))
        bad_len[5] = 8  # payload_len LSB tampered; CRC intentionally stale
        frames.append(wrap_outer(bytes(bad_len)))

        wrong_ver = build_inner(int(mh.HOST_TYPE_VER_URC), 0, 3, b"X", ver=int(mh.HOST_PROTOCOL_VER) + 1)
        frames.append(wrap_outer(wrong_ver))

        return frames

    def _build_stats_payload(self) -> bytes:
        if self._emit_additive:
            payload_len = int(mh.HOST_STATS_PAYLOAD_LEN)
        else:
            payload_len = int(mh.HOST_STATS_LEGACY_PAYLOAD_LEN)

        payload = bytearray(payload_len)
        max_offset = payload_len - 4
        for offset in range(0, max_offset + 1, 4):
            struct.pack_into("<I", payload, offset, self._stats_counter + offset)

        self._stats_counter += 11
        self._emit_additive = not self._emit_additive
        return bytes(payload)

    def handle_inner(self, inner: bytes) -> List[bytes]:
        parsed = parse_inner(inner)
        if parsed is None:
            return []

        if parsed.type == int(mh.HOST_TYPE_PING_REQ):
            return [wrap_outer(build_inner(parsed.type, parsed.flags, parsed.seq, parsed.payload))]

        if parsed.type == int(mh.HOST_TYPE_CFG_GET_REQ):
            key = parsed.payload[0] if parsed.payload else 0
            payload = bytes([key, 1, 0])
            return [wrap_outer(build_inner(int(mh.HOST_TYPE_CFG_DATA_URC), 0, parsed.seq, payload))]

        if parsed.type == int(mh.HOST_TYPE_STATS_DUMP_REQ):
            payload = self._build_stats_payload()
            return [wrap_outer(build_inner(int(mh.HOST_TYPE_STATS_URC), 0, parsed.seq, payload))]

        # Unknown request type: return protocol error URC.
        err_payload = bytes([parsed.type])
        return [wrap_outer(build_inner(int(mh.HOST_TYPE_ERR_PROTO_URC), 0, parsed.seq, err_payload))]

    def write_chunked(self, write_part: Callable[[bytes], object], data: bytes) -> None:
        pattern = [1, 3, 11, 64]
        offset = 0
        while offset < len(data):
            n = pattern[self._chunk_index % len(pattern)]
            self._chunk_index += 1
            part = data[offset : offset + n]
            if not part:
                break
            write_part(part)
            offset += len(part)
