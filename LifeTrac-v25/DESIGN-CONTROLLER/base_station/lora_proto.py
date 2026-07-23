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
CMD_LINK_PROFILE = 0x64

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
    # Stable wire values — DO NOT renumber. The first four are the original
    # auto-fallback ladder; values 4..7 are the bench-rotation candidates
    # from `AI NOTES/2026-05-25_Grayscale_Quantization_Encoding_Research_Copilot_v1_0.md`.
    FULL = 0           # Plan A0 baseline: full color WebP per tile
    Y_ONLY = 1         # Plan A: luma-only WebP, chroma re-estimated at base
    MOTION_ONLY = 2    # legacy: luma-only WebP at MOTION_ONLY_QUALITY
    WIREFRAME = 3      # legacy: luma-only WebP at WIREFRAME_QUALITY
    BTC4_PER_TILE = 4  # Plan C-2bit: per-tile 4-level palette + LZ4
    BTC4_PER_FRAME = 5 # Plan C-2: one 4-level palette per frame + LZ4
    MONO_G4 = 6        # Plan D: 1-bit Floyd-Steinberg dither + Group-4 fax
    ADAPTIVE = 7       # Plan H: per-tile entropy heuristic picks among 1/4/256


# Resilience ladder — most bandwidth-hungry (least resilient) first.
# Used by `link_monitor.EncodeModeController` to clamp the auto-fallback
# ladder under the operator-selected ceiling, and to define the auto-
# demotion order when the link degrades. ADAPTIVE is intentionally NOT
# in the ladder — when the operator pins it the auto-fallback layer
# leaves it alone unless the link goes critical, then jumps to the floor.
ENCODE_MODE_LADDER: tuple["EncodeMode", ...] = (
    EncodeMode.FULL,            # ceiling — best quality
    EncodeMode.Y_ONLY,
    EncodeMode.BTC4_PER_TILE,
    EncodeMode.BTC4_PER_FRAME,
    EncodeMode.MONO_G4,         # floor — survives the worst link
)


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
PHY_IMAGE_BW250 = PhyProfile("image_bw250", 7, 250, 5, 8)   # what the radio RUNS today
PHY_IMAGE_BW500 = PhyProfile("image_bw500", 7, 500, 5, 8)   # valid after Phase 2.4
PHY_IMAGE = PhyProfile("image", 7, 250, 5, 8)                # alias to BW250 (fixes F1)

PHY_BY_NAME = {
    profile.name: profile
    for profile in (PHY_CONTROL_SF7, PHY_CONTROL_SF8, PHY_CONTROL_SF9, PHY_TELEMETRY, PHY_IMAGE, PHY_IMAGE_BW250, PHY_IMAGE_BW500)
}

# IP-W2-04: shared ordered tuple used as the wire-side index space for
# CMD_LINK_PROFILE (opcode 0x64). The base-station emitter and the X8
# decoder MUST agree on this order; the X8 mirrors it as
# ``camera_service.LINK_PHY_NAMES``.
LINK_PHY_NAMES: tuple[str, ...] = (
    "image", "telemetry", "control_sf9", "control_sf8", "control_sf7", "image_bw250", "image_bw500"
)


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


# --- D13 AES-GCM-64 implicit-nonce codec (proposed) ------------------------
#
# Wire layout (replaces explicit 12-B nonce + 16-B tag = +28 B):
#
#   on_air = seq_be32 (4 B) ‖ ciphertext (len = cleartext) ‖ tag (8 B)
#
# RX reconstructs the AES-GCM 12-B nonce from receiver-side state:
#
#   nonce = src (1 B) ‖ boot_ctr (4 B, big-endian) ‖ seq (4 B, big-endian)
#                       ‖ zero_pad (3 B)
#
# Both sides must agree on (src, boot_ctr) out-of-band. `boot_ctr` is
# persisted to flash and incremented on every TX-side boot to guarantee
# nonce uniqueness across reboots even if the seq counter resets. The
# RX learns the new boot_ctr from an explicit handshake frame (out of
# scope of this codec — see DECISIONS.md D13 §3 boot_ctr roll protocol).
#
# Tag is the first 8 bytes of the full 16-B AES-GCM tag (NIST SP 800-38D
# §5.2.1.2 allows truncation to ≥ 32 bits; we use 64 bits). The §19.6
# rate-limited MAC-failure floor budgets the residual forgery risk.

GCM64_SEQ_LEN = 4
GCM64_TAG_LEN = 8
GCM64_BOOT_CTR_LEN = 4
GCM64_OVERHEAD = GCM64_SEQ_LEN + GCM64_TAG_LEN  # 12 B (matches CRYPTO_GCM64_IMPLICIT.overhead_bytes)


def build_implicit_nonce(source_id: int, boot_ctr: int, seq: int) -> bytes:
    """Reconstruct the 12-B AES-GCM nonce for the D13 implicit-nonce profile.

    Layout: src(1) ‖ boot_ctr_be32(4) ‖ seq_be32(4) ‖ zero_pad(3) = 12 B.
    All three integer fields are masked to their wire width so callers
    cannot accidentally inject upper bits that would silently desync
    sender and receiver."""
    return (
        bytes([source_id & 0xFF])
        + struct.pack(">I", boot_ctr & 0xFFFFFFFF)
        + struct.pack(">I", seq & 0xFFFFFFFF)
        + b"\x00\x00\x00"
    )


def encrypt_frame_gcm64_implicit(key: bytes, source_id: int, boot_ctr: int,
                                 seq: int, plaintext: bytes) -> bytes:
    """D13 codec: seq_be32 ‖ ciphertext ‖ tag8. Overhead = +12 B."""
    from cryptography.hazmat.primitives.ciphers import Cipher, algorithms, modes

    nonce = build_implicit_nonce(source_id, boot_ctr, seq)
    encryptor = Cipher(algorithms.AES(key), modes.GCM(nonce)).encryptor()
    ciphertext = encryptor.update(plaintext) + encryptor.finalize()
    tag8 = encryptor.tag[:GCM64_TAG_LEN]
    return struct.pack(">I", seq & 0xFFFFFFFF) + ciphertext + tag8


def decrypt_frame_gcm64_implicit(key: bytes, source_id: int, boot_ctr: int,
                                 onair: bytes) -> tuple[int, bytes] | None:
    """D13 RX: parse seq, reconstruct nonce, decrypt + verify 8-B tag.

    Returns (seq, plaintext) on success, None on any failure
    (length too short, MAC mismatch). Caller is responsible for
    feeding `seq` into the per-source replay window AFTER this returns
    a non-None value (per §19.6 the replay check runs after MAC verify
    to avoid a side-channel on the replay state itself)."""
    if len(onair) < GCM64_SEQ_LEN + GCM64_TAG_LEN:
        return None
    from cryptography.hazmat.primitives.ciphers import Cipher, algorithms, modes

    seq = struct.unpack(">I", onair[:GCM64_SEQ_LEN])[0]
    tag8 = onair[-GCM64_TAG_LEN:]
    ciphertext = onair[GCM64_SEQ_LEN:-GCM64_TAG_LEN]
    nonce = build_implicit_nonce(source_id, boot_ctr, seq)
    try:
        decryptor = Cipher(
            algorithms.AES(key),
            modes.GCM(nonce, tag8, min_tag_length=GCM64_TAG_LEN),
        ).decryptor()
        plaintext = decryptor.update(ciphertext) + decryptor.finalize()
    except Exception:
        return None
    return seq, plaintext


# --- D14 split-trust image fragment framer (plaintext + CRC32) -------------
#
# Wire layout (replaces +28 B GCM-128 envelope for P3 image fragments):
#
#   on_air = seq_be32 (4 B) ‖ cleartext (variable) ‖ crc16_be (2 B)
#
# Where crc16 = (zlib.crc32(seq_be32 ‖ cleartext) & 0xFFFF). We use
# the lower 16 bits of CRC32 (not CRC16-CCITT) deliberately: it gives
# a different polynomial domain than the PHY-layer CRC16-CCITT, so a
# burst error that defeats the PHY CRC is unlikely to also satisfy
# this app-layer CRC. There is intentionally NO MAC (D14 split-trust
# rationale §20.4): a forged image fragment has zero kinetic-control
# authority, and the host-boundary class-tag enforcer (below) prevents
# reclassification into a P0/P1/P2 path.

IMAGE_PLAIN_SEQ_LEN = 4
IMAGE_PLAIN_CRC_LEN = 2
IMAGE_PLAIN_OVERHEAD = IMAGE_PLAIN_SEQ_LEN + IMAGE_PLAIN_CRC_LEN  # 6 B


def _image_plain_crc(seq_bytes: bytes, payload: bytes) -> int:
    import zlib
    return zlib.crc32(seq_bytes + payload) & 0xFFFF


def pack_image_fragment_plain(seq: int, payload: bytes) -> bytes:
    """D14 TX: seq_be32 ‖ payload ‖ crc16_be. Overhead = +6 B, NO MAC."""
    seq_bytes = struct.pack(">I", seq & 0xFFFFFFFF)
    crc = _image_plain_crc(seq_bytes, payload)
    return seq_bytes + payload + struct.pack(">H", crc)


def unpack_image_fragment_plain(onair: bytes) -> tuple[int, bytes] | None:
    """D14 RX: parse seq, verify CRC, return (seq, payload). None on failure.

    Caller MUST treat the returned payload as UNAUTHENTICATED and route
    it ONLY into the image pipeline. See `enforce_class_tag_boundary`
    below for the host-side guard that makes this safe."""
    if len(onair) < IMAGE_PLAIN_OVERHEAD:
        return None
    seq_bytes = onair[:IMAGE_PLAIN_SEQ_LEN]
    payload = onair[IMAGE_PLAIN_SEQ_LEN:-IMAGE_PLAIN_CRC_LEN]
    crc_recv = struct.unpack(">H", onair[-IMAGE_PLAIN_CRC_LEN:])[0]
    if _image_plain_crc(seq_bytes, payload) != crc_recv:
        return None
    seq = struct.unpack(">I", seq_bytes)[0]
    return seq, payload


# --- S5.3 Host-boundary class-tag enforcer (X8 ↔ L072) ---------------------
#
# §20 / §21.3-3 class-downgrade-only invariant: a frame arriving at the
# X8 ↔ L072 host boundary carries a declared traffic class (P0/P1/P2/P3)
# AND a crypto profile (which may or may not have a MAC). The enforcer
# rejects any path that would let an unauthenticated frame (no MAC)
# drive a kinetic effector, and rejects any reclassification that would
# raise the privilege level of the frame mid-pipeline.
#
# This module-level guard is the single point that makes the D14
# split-trust image profile safe: even if an attacker forges a
# CRYPTO_IMAGE_PLAIN_CRC32 fragment that decodes cleanly, it cannot be
# laundered into a P0/P1/P2 path because (a) it has no MAC and (b) any
# attempt to relabel its class to a higher-privilege bucket raises.

class ClassTagViolation(Exception):
    """Raised when a frame's (class, profile) tuple violates §21.3-3."""


# P0/P1/P2 require an authenticated profile (has_mac == True). P3 does not.
# Constants inlined here (not `PRIO_P0` etc.) because those names are
# defined later in this module; the integer wire values are stable per
# LORA_PROTOCOL.md so the inline literals are safe.
_CLASSES_REQUIRING_MAC = frozenset({0, 1, 2})  # P0, P1, P2


def enforce_class_tag_boundary(declared_class: int, profile: CryptoProfile) -> None:
    """Reject frames whose (class, crypto profile) tuple violates §21.3-3.

    Specifically: any class in {P0, P1, P2} that carries an unauthenticated
    profile (has_mac=False) is rejected — this is the guard that prevents
    a forged D14 image fragment from being laundered into a kinetic-control
    path at the X8 ↔ L072 host boundary. P3 is allowed without a MAC
    (that is the whole point of the D14 split-trust profile)."""
    if declared_class in _CLASSES_REQUIRING_MAC and not profile.has_mac:
        raise ClassTagViolation(
            f"class {declared_class} (P0/P1/P2) requires an authenticated "
            f"crypto profile; got profile={profile.name!r} (has_mac=False). "
            f"See §21.3-3 class-downgrade-only invariant."
        )


def enforce_class_downgrade_only(src_class: int, dst_class: int) -> None:
    """Reject reclassification that raises the privilege level of a frame.

    Class numbers are inverted-priority (P0 = 0 = highest privilege,
    P3 = 3 = lowest). A frame may be downgraded (dst_class >= src_class)
    but never upgraded. This blocks the "image fragment laundered into
    a control frame" attack at the host boundary."""
    if dst_class < src_class:
        raise ClassTagViolation(
            f"class-downgrade-only invariant violated: cannot reclassify "
            f"P{src_class} frame as P{dst_class} (would raise privilege). "
            f"See §21.3-3."
        )


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


# --- Crypto-profile-aware on-air overhead ----------------------------------
#
# Per the 2026-05-18 TX-Power Adaptation design doc (§19 / §20 / D13 / D14),
# we need to predict on-air bytes under three different crypto profiles:
#
#   * The SHIPPED profile (today's code): explicit 12 B nonce + AES-GCM-128
#     16 B tag = +28 B overhead. Used by all classes right now.
#   * The PROPOSED control/telemetry profile (D13): implicit nonce derived
#     from `(src ‖ boot_ctr ‖ seq)` so it does not ride on the air + a
#     truncated 8 B AES-GCM tag + a 4 B explicit `seq` field that the RX
#     uses to reconstruct the nonce = +12 B overhead. Cuts ~16 B per frame
#     versus the shipped profile and is the source of the ~1 s/s airtime
#     reclaim claimed in §17 / §18.
#   * The PROPOSED image fragment profile (D14, "split-trust"): plaintext
#     payload + 4 B monotonic `seq` + 2 B CRC32 truncated to 16 bits at the
#     application layer = +6 B overhead with no cryptographic MAC. Safe
#     because image fragments have zero kinetic-effector authority and
#     because the host-boundary class tag (§20) prevents an unauth frame
#     from being reclassified into a control path.
#
# `encrypted_payload_len(cleartext_len)` keeps its old single-arg signature
# (= SHIPPED profile, +28 B) so every existing caller keeps working. New
# callers pass an explicit `CryptoProfile` to predict the proposed profiles.

@dataclass(frozen=True)
class CryptoProfile:
    """Per-class on-air overhead model. `overhead_bytes` is the constant
    delta between the application-layer cleartext payload and the on-air
    LoRa payload that gets fed to `lora_time_on_air_ms`. `has_mac` records
    whether the profile carries a cryptographic authentication tag, which
    matters for the "may this class drive a kinetic effector?" check at
    the host boundary (§20 D14)."""
    name: str
    overhead_bytes: int
    has_mac: bool
    description: str = ""


# Shipped profile: 12 B nonce + 16 B GCM-128 tag.
CRYPTO_GCM128_EXPLICIT = CryptoProfile(
    name="gcm128_explicit",
    overhead_bytes=GCM_NONCE_LEN + GCM_TAG_LEN,  # 28
    has_mac=True,
    description="Shipped 2026 profile: 12 B explicit nonce + AES-GCM-128 16 B tag.",
)

# Proposed D13: implicit nonce (reconstructed from src/boot_ctr/seq on RX) +
# 4 B explicit seq + 8 B truncated GCM tag. 12 B on the wire.
CRYPTO_GCM64_IMPLICIT = CryptoProfile(
    name="gcm64_implicit",
    overhead_bytes=4 + 8,  # 12
    has_mac=True,
    description=(
        "D13 proposal: 4 B explicit seq + 8 B truncated AES-GCM tag; nonce "
        "is reconstructed from (src ‖ boot_ctr ‖ seq) on RX. Saves 16 B "
        "vs CRYPTO_GCM128_EXPLICIT. Use only for classes whose §19.6 "
        "rate-limited MAC-failure floor is acceptable."
    ),
)

# Proposed D14: split-trust image fragment. No crypto, just integrity tag.
CRYPTO_IMAGE_PLAIN_CRC32 = CryptoProfile(
    name="image_plain_crc32",
    overhead_bytes=4 + 2,  # 4 B seq + 2 B CRC32-truncated-to-16
    has_mac=False,
    description=(
        "D14 split-trust profile for P3 image fragments only: 4 B seq + "
        "2 B integrity tag, no MAC. The host boundary MUST enforce that "
        "this profile can never be reclassified into a kinetic-control "
        "path (§20 §21.3-3 class-downgrade-only invariant)."
    ),
)

# Default to the shipped profile so existing single-arg callers are unchanged.
CRYPTO_PROFILE_DEFAULT = CRYPTO_GCM128_EXPLICIT

CRYPTO_PROFILES = {
    p.name: p for p in (
        CRYPTO_GCM128_EXPLICIT,
        CRYPTO_GCM64_IMPLICIT,
        CRYPTO_IMAGE_PLAIN_CRC32,
    )
}


def encrypted_payload_len(cleartext_len: int,
                          profile: CryptoProfile = CRYPTO_PROFILE_DEFAULT) -> int:
    """Predict on-air payload bytes for a cleartext frame under `profile`.

    Backward compatible: single-arg calls use the SHIPPED GCM-128 explicit
    profile (+28 B). Pass `profile=CRYPTO_GCM64_IMPLICIT` to predict the
    D13 reclaim, or `profile=CRYPTO_IMAGE_PLAIN_CRC32` for D14 image
    fragments."""
    if cleartext_len < 0:
        raise ValueError("cleartext_len must be >= 0")
    return cleartext_len + profile.overhead_bytes


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
                         CMD_CAMERA_SELECT, CMD_ENCODE_MODE, CMD_LINK_PROFILE})


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
# W2-02 P0c (2026-05-20): v2 wire format with explicit redundancy. The
# 5-byte header is identical to v1 except for the magic and one extra
# trailing byte encoding (total_copies<<4)|copy_idx (nibbles). v2 is
# emitted only by image-pipeline payloads with redundancy >= 2; telemetry
# topics continue to use v1 (no airtime budget for redundancy under
# TELEMETRY_FRAGMENT_MAX_AIRTIME_MS). The reassembler in
# image_pipeline/reassemble.py recognises both magics and treats v2
# copies as dedup-by-(seq, idx) with first-copy-wins semantics.
TELEMETRY_FRAGMENT_MAGIC_V2 = 0xFD
TELEMETRY_FRAGMENT_HEADER_LEN_V2 = 5
TELEMETRY_FRAGMENT_MAX_AIRTIME_MS = 25.0

# Image strict-path fragmentation constants
LORA_HOP_HDR_LEN = 8            # lora_pkt_hdr.h LORA_PKT_HDR_LEN
TX_FRAME_BODY_MAX = 255 - LORA_HOP_HDR_LEN   # 247: F7-safe hard ceiling
IMAGE_FRAG_AIR_CAP_MS = float(os.environ.get("LIFETRAC_FRAG_AIR_CAP_MS", "170.0"))


def max_image_fragment_body(profile: PhyProfile = PHY_IMAGE_BW250,
                            max_air_ms: float = IMAGE_FRAG_AIR_CAP_MS,
                            body_ceiling: int = TX_FRAME_BODY_MAX) -> int:
    """Largest TX_FRAME_REQ body whose ON-AIR time (body + hop header)
    fits max_air_ms. Unlike max_telemetry_fragment_payload() this does
    NOT inherit the 118 B TelemetryFrame clamp (F13) and does NOT add
    the 9 B telemetry envelope the strict path never sends."""
    lo, hi, best = 1, body_ceiling, 0
    while lo <= hi:
        mid = (lo + hi) // 2
        if lora_time_on_air_ms(mid + LORA_HOP_HDR_LEN, profile) <= max_air_ms:
            best, lo = mid, mid + 1
        else:
            hi = mid - 1
    return best


def pack_image_fragments(payload: bytes, frag_seq: int,
                         profile: PhyProfile = PHY_IMAGE_BW250,
                         max_air_ms: float = IMAGE_FRAG_AIR_CAP_MS) -> list[bytes]:
    """pack_telemetry_fragments twin for the image strict path (0xFE v1
    header, same reassembler) sized by max_image_fragment_body()."""
    chunk = max_image_fragment_body(profile, max_air_ms) - TELEMETRY_FRAGMENT_HEADER_LEN
    if chunk <= 0:
        raise ValueError(f"{profile.name}: no fragment fits {max_air_ms} ms")
    total = max(1, (len(payload) + chunk - 1) // chunk)
    if total > 256:
        raise ValueError(f"payload needs {total} fragments; max 256")
    out: list[bytes] = []
    for i in range(total):
        body = payload[i * chunk:(i + 1) * chunk]
        header = bytes([TELEMETRY_FRAGMENT_MAGIC,
                        frag_seq & 0xFF,
                        i & 0xFF,
                        (total - 1) & 0xFF])
        out.append(header + body)
    return out


def pack_image_fragments_v2(payload: bytes, frag_seq: int,
                            profile: PhyProfile = PHY_IMAGE_BW250,
                            max_air_ms: float = IMAGE_FRAG_AIR_CAP_MS,
                            copies: int = 2) -> list[bytes]:
    """Wrap fragments in the v2 0xFD redundancy header that
    image_pipeline/reassemble.py ALREADY dedups (first copy wins, W2-02
    P0c). Unlike a TX_DONE retry (local-only), independent copies
    repair AIR loss: P(fragment lost) = p^copies."""
    if not 1 <= copies <= 15:
        raise ValueError("copies must be 1..15 (4-bit nibble)")
    base = pack_image_fragments(payload, frag_seq, profile, max_air_ms)
    if copies == 1:
        return base
    out: list[bytes] = []
    for body in base:                       # body = 0xFE hdr(4) + data
        _, seq, idx, total_m1 = body[0], body[1], body[2], body[3]
        for copy_idx in range(copies):
            out.append(bytes([TELEMETRY_FRAGMENT_MAGIC_V2, seq, idx, total_m1,
                              ((copies & 0x0F) << 4) | copy_idx]) + body[4:])
    return out


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

    Accepts both v1 (0xFE, 4-byte header) and v2 (0xFD, 5-byte header
    with extra redundancy nibble byte). For v2 the redundancy byte is
    silently dropped here -- callers that care about copy_idx use the
    image_pipeline FragmentReassembler directly. Returns None when no
    known magic matches (caller should treat the body as a complete
    unfragmented payload -- keeps backward compat with telemetry topics
    whose payloads always fit under 25 ms).
    """
    if len(body) < TELEMETRY_FRAGMENT_HEADER_LEN:
        return None
    magic = body[0]
    if magic == TELEMETRY_FRAGMENT_MAGIC:
        header_len = TELEMETRY_FRAGMENT_HEADER_LEN
    elif magic == TELEMETRY_FRAGMENT_MAGIC_V2:
        if len(body) < TELEMETRY_FRAGMENT_HEADER_LEN_V2:
            return None
        red_byte = body[4]
        total_copies = (red_byte >> 4) & 0x0F
        copy_idx = red_byte & 0x0F
        if total_copies == 0 or copy_idx >= total_copies:
            return None
        header_len = TELEMETRY_FRAGMENT_HEADER_LEN_V2
    else:
        return None
    frag_seq = body[1]
    frag_idx = body[2]
    total = body[3] + 1
    if frag_idx >= total:
        return None
    return frag_seq, frag_idx, total, bytes(body[header_len:])


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