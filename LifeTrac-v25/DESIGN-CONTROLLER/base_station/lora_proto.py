"""Shared LifeTrac v25 LoRa wire helpers for base-station services.

This mirrors ``firmware/common/lora_proto`` closely enough for the Python
bridge, web UI, bench tools, and unit tests to share one protocol surface.
Hardware radio control stays outside this module.
"""

from __future__ import annotations

import math
import os
import struct
import time
from dataclasses import dataclass
from enum import IntEnum
from typing import Iterable

PROTO_VERSION = 0x01

SRC_HANDHELD = 0x01
SRC_BASE = 0x02
SRC_TRACTOR = 0x03
SRC_AUTONOMY = 0x04
SRC_NONE = 0xFF

FT_CONTROL = 0x10
FT_TELEMETRY = 0x20
FT_COMMAND = 0x30
FT_HEARTBEAT = 0x40

CMD_ESTOP = 0x01
CMD_CLEAR_ESTOP = 0x02
CMD_CAMERA_SELECT = 0x03
CMD_CAMERA_QUALITY = 0x04
CMD_PLAN_COMMIT = 0x10
CMD_LINK_HINT = 0x20
CMD_LINK_TUNE = 0x21
CMD_PERSON_APPEARED = 0x60
CMD_ROI_HINT = 0x61
CMD_REQ_KEYFRAME = 0x62
CMD_ENCODE_MODE = 0x63

TOPIC_BY_ID = {
    0x01: "lifetrac/v25/telemetry/gps",
    0x02: "lifetrac/v25/telemetry/engine",
    0x03: "lifetrac/v25/telemetry/battery",
    0x04: "lifetrac/v25/telemetry/hydraulics",
    0x05: "lifetrac/v25/telemetry/mode",
    0x06: "lifetrac/v25/telemetry/errors",
    0x07: "lifetrac/v25/telemetry/imu",
    0x08: "lifetrac/v25/telemetry/sensor_faults",
    0x10: "lifetrac/v25/control/source_active",
    0x20: "lifetrac/v25/video/thumbnail",
    0x21: "lifetrac/v25/video/thumbnail_rear",
    0x22: "lifetrac/v25/video/active_camera",
    0x23: "lifetrac/v25/video/thumbnail_implement",
    0x24: "lifetrac/v25/telemetry/crop_health",
    0x25: "lifetrac/v25/video/tile_delta",
    0x26: "lifetrac/v25/video/detections",
    0x27: "lifetrac/v25/video/audio_event",
    0x28: "lifetrac/v25/video/motion_vectors",
    0x29: "lifetrac/v25/video/wireframe",
    0x2A: "lifetrac/v25/video/semantic_map",  # reserved for v26
}

TOPIC_ID_BY_NAME = {value: key for key, value in TOPIC_BY_ID.items()}


class Badge(IntEnum):
    RAW = 0
    CACHED = 1
    ENHANCED = 2
    RECOLOURISED = 3
    PREDICTED = 4
    SYNTHETIC = 5
    WIREFRAME = 6


class EncodeMode(IntEnum):
    FULL = 0
    Y_ONLY = 1
    MOTION_ONLY = 2
    WIREFRAME = 3


KISS_FEND = 0xC0
KISS_FESC = 0xDB
KISS_TFEND = 0xDC
KISS_TFESC = 0xDD

HEADER_LEN = 5
CTRL_FRAME_LEN = 16
HB_FRAME_LEN = 10
TELEM_HEADER_LEN = HEADER_LEN + 2
# IP-306: reconciled with C side. The on-wire ``payload[120]`` array in
# ``firmware/common/lora_proto/lora_proto.h`` stores the trailing 2-byte
# CRC inside the same buffer, so the maximum *usable* payload is 118.
# Keeping the Python side at 120 silently allowed frames the tractor would
# truncate. See LORA_PROTOCOL.md telemetry frame layout.
TELEM_MAX_PAYLOAD = 118
CRC_LEN = 2
GCM_NONCE_LEN = 12
GCM_TAG_LEN = 16


@dataclass(frozen=True)
class LoraHeader:
    version: int
    source_id: int
    frame_type: int
    sequence_num: int


@dataclass(frozen=True)
class PhyProfile:
    name: str
    sf: int
    bw_khz: int
    cr_den: int
    preamble_len: int = 8


PHY_CONTROL_SF7 = PhyProfile("control_sf7", 7, 250, 5, 8)
PHY_CONTROL_SF8 = PhyProfile("control_sf8", 8, 125, 5, 8)
PHY_CONTROL_SF9 = PhyProfile("control_sf9", 9, 125, 5, 8)
PHY_TELEMETRY = PhyProfile("telemetry", 9, 250, 8, 12)
PHY_IMAGE = PhyProfile("image", 7, 500, 5, 8)

PHY_BY_NAME = {
    profile.name: profile
    for profile in (PHY_CONTROL_SF7, PHY_CONTROL_SF8, PHY_CONTROL_SF9, PHY_TELEMETRY, PHY_IMAGE)
}


def crc16_ccitt(data: bytes) -> int:
    """CRC-16/CCITT-FALSE: poly 0x1021, init 0xFFFF."""
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def verify_crc(frame: bytes) -> bool:
    if len(frame) < CRC_LEN + 1:
        return False
    expected = crc16_ccitt(frame[:-CRC_LEN])
    actual = struct.unpack("<H", frame[-CRC_LEN:])[0]
    return expected == actual


def kiss_encode(data: bytes) -> bytes:
    out = bytearray([KISS_FEND])
    for byte in data:
        if byte == KISS_FEND:
            out.extend([KISS_FESC, KISS_TFEND])
        elif byte == KISS_FESC:
            out.extend([KISS_FESC, KISS_TFESC])
        else:
            out.append(byte)
    out.append(KISS_FEND)
    return bytes(out)


class KissDecoder:
    def __init__(self, max_len: int = 512) -> None:
        self.max_len = max_len
        self.buf = bytearray()
        self.in_frame = False
        self.escape = False

    def feed(self, byte: int) -> Iterable[bytes]:
        if byte == KISS_FEND:
            if self.in_frame and self.buf:
                frame = bytes(self.buf)
                self.buf.clear()
                self.in_frame = False
                self.escape = False
                yield frame
            else:
                self.in_frame = True
                self.buf.clear()
                self.escape = False
            return
        if not self.in_frame:
            return
        if self.escape:
            if byte == KISS_TFEND:
                byte = KISS_FEND
            elif byte == KISS_TFESC:
                byte = KISS_FESC
            self.escape = False
        elif byte == KISS_FESC:
            self.escape = True
            return
        self.buf.append(byte)
        if len(self.buf) > self.max_len:
            self.buf.clear()
            self.in_frame = False
            self.escape = False


def parse_header(frame: bytes) -> LoraHeader | None:
    if len(frame) < HEADER_LEN:
        return None
    return LoraHeader(*struct.unpack("<BBBH", frame[:HEADER_LEN]))


def pack_control(seq: int, lhx: int, lhy: int, rhx: int, rhy: int,
                 buttons: int = 0, flags: int = 0, hb: int = 0,
                 source_id: int = SRC_BASE) -> bytes:
    body = struct.pack(
        "<BBBHbbbbHBBB",
        PROTO_VERSION,
        source_id,
        FT_CONTROL,
        seq & 0xFFFF,
        _clip_i8(lhx),
        _clip_i8(lhy),
        _clip_i8(rhx),
        _clip_i8(rhy),
        buttons & 0xFFFF,
        flags & 0xFF,
        hb & 0xFF,
        0,
    )
    return body + struct.pack("<H", crc16_ccitt(body))


def pack_heartbeat(seq: int, source_id: int, priority_request: int, flags: int = 0) -> bytes:
    body = struct.pack(
        "<BBBHBBB",
        PROTO_VERSION,
        source_id,
        FT_HEARTBEAT,
        seq & 0xFFFF,
        priority_request & 0xFF,
        flags & 0xFF,
        0,
    )
    return body + struct.pack("<H", crc16_ccitt(body))


def pack_command(seq: int, opcode: int, args: bytes = b"", source_id: int = SRC_BASE) -> bytes:
    if len(args) > 8:
        raise ValueError("command args are limited to 8 bytes")
    body = struct.pack("<BBBHB", PROTO_VERSION, source_id, FT_COMMAND, seq & 0xFFFF, opcode & 0xFF) + args
    return body + struct.pack("<H", crc16_ccitt(body))


def build_nonce(source_id: int, seq: int, now_s: int | None = None, random_tail: bytes | None = None) -> bytes:
    if now_s is None:
        now_s = int(time.time())
    if random_tail is None:
        random_tail = os.urandom(5)
    if len(random_tail) != 5:
        raise ValueError("random_tail must be exactly 5 bytes")
    return bytes([source_id & 0xFF]) + struct.pack("<H", seq & 0xFFFF) + struct.pack("<I", now_s & 0xFFFFFFFF) + random_tail


def encrypt_frame(key: bytes, source_id: int, seq: int, plaintext: bytes) -> bytes:
    from cryptography.hazmat.primitives.ciphers.aead import AESGCM

    nonce = build_nonce(source_id, seq)
    return nonce + AESGCM(key).encrypt(nonce, plaintext, None)


def decrypt_frame(key: bytes, onair: bytes) -> bytes | None:
    if len(onair) < GCM_NONCE_LEN + GCM_TAG_LEN:
        return None
    from cryptography.hazmat.primitives.ciphers.aead import AESGCM

    nonce = onair[:GCM_NONCE_LEN]
    ciphertext = onair[GCM_NONCE_LEN:]
    try:
        return AESGCM(key).decrypt(nonce, ciphertext, None)
    except Exception:
        return None


def lora_time_on_air_ms(payload_len: int, profile: PhyProfile | None = None,
                        *, sf: int | None = None, bw_khz: int | None = None,
                        cr_den: int | None = None, preamble_len: int | None = None) -> float:
    """Semtech LoRa packet airtime estimate for explicit-header payloads."""
    if profile is not None:
        sf = profile.sf
        bw_khz = profile.bw_khz
        cr_den = profile.cr_den
        preamble_len = profile.preamble_len
    if sf is None or bw_khz is None or cr_den is None:
        raise ValueError("profile or sf/bw_khz/cr_den is required")
    if preamble_len is None:
        preamble_len = 8
    low_data_rate_opt = 1 if sf >= 11 and bw_khz <= 125 else 0
    tsym_ms = (2 ** sf) / (bw_khz * 1000.0) * 1000.0
    cr = cr_den - 4
    numerator = 8 * payload_len - 4 * sf + 28 + 16
    denominator = 4 * (sf - 2 * low_data_rate_opt)
    payload_symbols = 8 + max(math.ceil(numerator / denominator) * (cr + 4), 0)
    return (preamble_len + 4.25 + payload_symbols) * tsym_ms


def encrypted_payload_len(cleartext_len: int) -> int:
    return GCM_NONCE_LEN + cleartext_len + GCM_TAG_LEN


def fhss_channel_index(key_id: int, hop_counter: int) -> int:
    perm = list(range(8))
    rnd = _xorshift32((key_id & 0xFFFFFFFF) ^ ((hop_counter // 8) * 0x9E3779B9))
    for idx in range(7, 0, -1):
        rnd = _xorshift32(rnd)
        swap = rnd % (idx + 1)
        perm[idx], perm[swap] = perm[swap], perm[idx]
    return perm[hop_counter & 0x07]


def fhss_channel_hz(key_id: int, hop_counter: int) -> int:
    return 902_000_000 + fhss_channel_index(key_id, hop_counter) * 3_250_000


# CSMA "skip-busy" wrapper. Per LORA_IMPLEMENTATION.md §8 row 2 (FHSS), before
# transmitting on the deterministic hop we sample the carrier and, if the
# channel reads above `busy_threshold_dbm`, advance to the next hop in the
# deterministic sequence and try again. Bounded by `max_skips` so we never
# spend more than ~1 dwell window hunting for a clear channel; if every probed
# hop is busy we fall back to the final candidate so the frame still goes out
# (a control frame missing its dwell hurts safety more than a small CCA miss).
CSMA_DEFAULT_BUSY_DBM = -90
CSMA_DEFAULT_MAX_SKIPS = 4


def pick_csma_hop(key_id: int, start_hop: int,
                  sample_rssi_dbm,
                  busy_threshold_dbm: int = CSMA_DEFAULT_BUSY_DBM,
                  max_skips: int = CSMA_DEFAULT_MAX_SKIPS) -> tuple[int, int]:
    """Walk the FHSS sequence forward from `start_hop` until a candidate hop
    reads RSSI <= busy_threshold_dbm or we have skipped `max_skips` hops.

    `sample_rssi_dbm(channel_hz)` must return a signed dBm reading.
    Returns `(chosen_hop_counter, skips)`. `skips` == 0 means the start hop
    was already clear; `skips` == max_skips means every probe was busy and
    we fell through to the last candidate (TX anyway, log the event).
    """
    if max_skips < 0:
        raise ValueError("max_skips must be >= 0")
    hop = start_hop
    for skips in range(max_skips + 1):
        rssi = sample_rssi_dbm(fhss_channel_hz(key_id, hop))
        if rssi <= busy_threshold_dbm:
            return hop, skips
        hop += 1
    # Every candidate up to max_skips was busy. Caller must log.
    return hop - 1, max_skips


def topic_name(topic_id: int) -> str:
    return TOPIC_BY_ID.get(topic_id, f"lifetrac/v25/raw/{topic_id:02x}")


# Topic IDs that ride the image-link PHY profile (per LORA_PROTOCOL.md PHY table
# and IMAGE_PIPELINE.md §13.1 Revisit-1). Anything else under FT_TELEMETRY rides
# the telemetry profile.
IMAGE_TOPIC_IDS = frozenset({0x25, 0x28, 0x29})


def attribute_phy(frame_type: int, topic_id: int | None = None) -> PhyProfile:
    """Infer which PHY profile a cleartext frame would have ridden on the air.

    Used by the bridge's airtime ledger so utilization is attributed correctly
    without each call site having to know the PHY split. Mirrors the per-frame
    retune policy in LORA_IMPLEMENTATION.md §3 / LORA_PROTOCOL.md PHY table.
    """
    if frame_type == FT_TELEMETRY and topic_id in IMAGE_TOPIC_IDS:
        return PHY_IMAGE
    if frame_type == FT_TELEMETRY:
        return PHY_TELEMETRY
    # ControlFrame / Heartbeat / Command all ride the control profile. The
    # adaptive SF ladder does the SF7→SF8→SF9 step-down separately; until that
    # ladder lands we attribute to SF7 as the default rung.
    return PHY_CONTROL_SF7


# Priority class definitions per LORA_IMPLEMENTATION.md §4.
# Lower number = higher priority (so heapq orders correctly).
PRIO_P0 = 0   # ControlFrame, Heartbeat, CMD_ESTOP, CMD_LINK_TUNE, CMD_PERSON_APPEARED
PRIO_P1 = 1   # CMD_CLEAR_ESTOP, CMD_ROI_HINT, CMD_REQ_KEYFRAME, CMD_CAMERA_SELECT, CMD_ENCODE_MODE
PRIO_P2 = 2   # telemetry (non-image)
PRIO_P3 = 3   # image fragments (topics 0x25/0x28/0x29)

_P0_OPCODES = frozenset({CMD_ESTOP, CMD_LINK_TUNE, CMD_PERSON_APPEARED})
_P1_OPCODES = frozenset({CMD_CLEAR_ESTOP, CMD_ROI_HINT, CMD_REQ_KEYFRAME,
                         CMD_CAMERA_SELECT, CMD_ENCODE_MODE})


def classify_priority(frame_type: int, opcode: int | None = None,
                      topic_id: int | None = None) -> int:
    """Map a cleartext frame to a P0..P3 class.

    `opcode` only matters for FT_COMMAND; `topic_id` only matters for
    FT_TELEMETRY. Returns one of the PRIO_P* constants. Defaults to P2 for
    anything unrecognised so the queue never silently demotes a control frame.
    """
    if frame_type in (FT_CONTROL, FT_HEARTBEAT):
        return PRIO_P0
    if frame_type == FT_COMMAND:
        if opcode in _P0_OPCODES:
            return PRIO_P0
        if opcode in _P1_OPCODES:
            return PRIO_P1
        return PRIO_P1   # unknown command: assume operator-issued, P1
    if frame_type == FT_TELEMETRY:
        if topic_id in IMAGE_TOPIC_IDS:
            return PRIO_P3
        return PRIO_P2
    return PRIO_P2


# -------------------------------------------------------------------------
# Replay-defence sliding window (mirror of LpReplayWindow in lora_proto.h).
#
# Used by the bridge's per-source RX path so a base station that observes
# tractor TX can also reject replays. The bit layout matches the C code so
# integration tests can cross-validate.
# -------------------------------------------------------------------------
REPLAY_WINDOW_BITS = 64


class ReplayWindow:
    __slots__ = ("high_water", "bitmap", "primed")

    def __init__(self) -> None:
        self.high_water = 0
        self.bitmap = 0
        self.primed = False

    def check_and_update(self, seq: int) -> bool:
        seq &= 0xFFFF
        if not self.primed:
            self.high_water = seq
            self.bitmap = 1
            self.primed = True
            return True
        # Signed 16-bit delta handles wrap.
        raw = (seq - self.high_water) & 0xFFFF
        delta = raw - 0x10000 if raw & 0x8000 else raw
        if delta > 0:
            if delta >= REPLAY_WINDOW_BITS:
                self.bitmap = 1
            else:
                self.bitmap = ((self.bitmap << delta) | 1) & ((1 << REPLAY_WINDOW_BITS) - 1)
            self.high_water = seq
            return True
        age = -delta
        if age >= REPLAY_WINDOW_BITS:
            return False
        mask = 1 << age
        if self.bitmap & mask:
            return False
        self.bitmap |= mask
        return True


def _clip_i8(value: int) -> int:
    return max(-127, min(127, int(value)))


def _xorshift32(value: int) -> int:
    value &= 0xFFFFFFFF
    if value == 0:
        value = 0x6D2B79F5
    value ^= (value << 13) & 0xFFFFFFFF
    value ^= value >> 17
    value ^= (value << 5) & 0xFFFFFFFF
    return value & 0xFFFFFFFF


# -------------------------------------------------------------------------
# R-6 — Telemetry fragmentation.
#
# Per IMAGE_PIPELINE.md §13.3 R-6 / MASTER_PLAN.md §8.17, P2 telemetry obeys
# the same ≤25 ms-per-fragment airtime cap as image fragments. When a
# topic payload exceeds the per-PHY fragment budget we chop it into
# `TileDeltaFrame`-style fragments using the 4-byte 0xFE-magic header that
# `base_station.image_pipeline.reassemble.FragmentReassembler` already
# understands.
#
# Wire format (matches reassemble.py):
#     u8 magic = 0xFE
#     u8 frag_seq        ; same value across all fragments of one logical
#                        ; payload, increments per logical payload mod 256
#     u8 frag_idx        ; 0..total-1
#     u8 total_minus_one ; total fragments - 1 (so single-fragment payloads
#                        ; would still use 0)
#     payload bytes
#
# Each TelemetryFrame body (header + topic_id + length + fragment bytes +
# CRC) must satisfy `lora_time_on_air_ms(...) <= max_air_ms`. We size the
# fragments by binary search up front — measured once per (profile,
# topic_id) and cached, since the PHY profile is per-topic_id stable.
# -------------------------------------------------------------------------
TELEMETRY_FRAGMENT_MAGIC = 0xFE
TELEMETRY_FRAGMENT_HEADER_LEN = 4
TELEMETRY_FRAGMENT_MAX_AIRTIME_MS = 25.0


def max_telemetry_fragment_payload(profile: PhyProfile,
                                   max_air_ms: float = TELEMETRY_FRAGMENT_MAX_AIRTIME_MS) -> int:
    """Largest fragment-body byte count whose on-air time ≤ ``max_air_ms``.

    The returned value is the count of *fragment header + fragment data*
    bytes that go inside the TelemetryFrame body; the caller still pays
    the TelemetryFrame envelope (5-byte header + 2-byte topic/length tag +
    2-byte CRC), which we account for by adding a constant 9 here.
    """
    envelope = TELEM_HEADER_LEN + CRC_LEN  # 5-byte header + 2-byte CRC
    # Binary search.
    lo, hi = 1, TELEM_MAX_PAYLOAD
    best = 0
    while lo <= hi:
        mid = (lo + hi) // 2
        air = lora_time_on_air_ms(envelope + mid, profile)
        if air <= max_air_ms:
            best = mid
            lo = mid + 1
        else:
            hi = mid - 1
    return max(0, best - TELEMETRY_FRAGMENT_HEADER_LEN)


def pack_telemetry_fragments(payload: bytes, frag_seq: int,
                             profile: PhyProfile,
                             max_air_ms: float = TELEMETRY_FRAGMENT_MAX_AIRTIME_MS) -> list[bytes]:
    """Split a logical telemetry payload into ≤25 ms-air fragment bodies.

    Returns the list of fragment bodies (header + data) that the caller
    wraps into TelemetryFrames. If the payload already fits in a single
    fragment under ``max_air_ms`` the returned list has length 1; the
    fragment is still tagged so the receiver's reassembler treats it
    uniformly.
    """
    chunk = max_telemetry_fragment_payload(profile, max_air_ms)
    if chunk <= 0:
        raise ValueError(f"profile {profile.name} cannot fit any fragment in {max_air_ms} ms")
    total = max(1, (len(payload) + chunk - 1) // chunk)
    if total > 256:
        raise ValueError(f"payload requires {total} fragments; max 256")
    out: list[bytes] = []
    for idx in range(total):
        body = payload[idx * chunk:(idx + 1) * chunk]
        header = bytes([TELEMETRY_FRAGMENT_MAGIC,
                        frag_seq & 0xFF,
                        idx & 0xFF,
                        (total - 1) & 0xFF])
        out.append(header + body)
    return out


def parse_telemetry_fragment(body: bytes) -> tuple[int, int, int, bytes] | None:
    """Inverse of pack_telemetry_fragments(); returns (seq, idx, total, data).

    Returns None when the magic byte doesn't match (caller should treat
    the body as a complete unfragmented payload — keeps backward compat
    with telemetry topics whose payloads always fit under 25 ms).
    """
    if len(body) < TELEMETRY_FRAGMENT_HEADER_LEN:
        return None
    if body[0] != TELEMETRY_FRAGMENT_MAGIC:
        return None
    frag_seq = body[1]
    frag_idx = body[2]
    total = body[3] + 1
    if frag_idx >= total:
        return None
    return frag_seq, frag_idx, total, bytes(body[TELEMETRY_FRAGMENT_HEADER_LEN:])


@dataclass
class _TelemPartial:
    total: int
    received_ms: int
    parts: dict[int, bytes]


class TelemetryReassembler:
    """Reassemble fragments produced by ``pack_telemetry_fragments``.

    Mirrors :class:`base_station.image_pipeline.reassemble.FragmentReassembler`
    but is keyed on (source_id, topic_id, frag_seq) so two topics sharing a
    seq window can't be confused. Time-based GC drops partial assemblies
    older than ``timeout_ms``.
    """

    def __init__(self, timeout_ms: int = 1500) -> None:
        self.timeout_ms = timeout_ms
        self._partials: dict[tuple[int, int, int], _TelemPartial] = {}
        self.completed = 0
        self.timeouts = 0
        self.duplicates = 0
        self.bad_magic_passthroughs = 0

    def feed(self, source_id: int, topic_id: int, body: bytes,
             now_ms: int) -> bytes | None:
        self._gc(now_ms)
        parsed = parse_telemetry_fragment(body)
        if parsed is None:
            self.bad_magic_passthroughs += 1
            return body  # passthrough — body is itself a complete payload
        frag_seq, frag_idx, total, data = parsed
        key = (source_id & 0xFF, topic_id & 0xFF, frag_seq)
        slot = self._partials.get(key)
        if slot is None:
            slot = _TelemPartial(total=total, received_ms=now_ms, parts={})
            self._partials[key] = slot
        elif slot.total != total:
            # Total changed mid-stream — start over.
            slot = _TelemPartial(total=total, received_ms=now_ms, parts={})
            self._partials[key] = slot
        if frag_idx in slot.parts:
            self.duplicates += 1
        else:
            slot.parts[frag_idx] = data
            slot.received_ms = now_ms
        if len(slot.parts) == slot.total:
            del self._partials[key]
            self.completed += 1
            return b"".join(slot.parts[i] for i in range(slot.total))
        return None

    def _gc(self, now_ms: int) -> None:
        expired = [k for k, p in self._partials.items()
                   if now_ms - p.received_ms > self.timeout_ms]
        for k in expired:
            del self._partials[k]
            self.timeouts += 1

    def pending_keys(self) -> list[tuple[int, int, int]]:
        return list(self._partials.keys())