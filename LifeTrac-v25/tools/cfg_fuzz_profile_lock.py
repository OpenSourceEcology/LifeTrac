#!/usr/bin/env python3
"""FCC-EVID-D6-2-b: wire-level fuzz orchestrator for the
cfg_set(CFG_KEY_REG_PROFILE) two-phase-commit dispatcher.

Drives one cfg_set transaction per host_cfg_profile_reject_t reject
class that is wire-reachable through the dispatcher introduced in
D6-2-a-iii (firmware/murata_l072/host/host_cfg.c). For each case the
orchestrator:

  1. Optionally writes prerequisite cfg keys (e.g. a bad channel mask
     or out-of-FCC antenna gain) so the synthesised
     ``host_cfg_profile_req_t`` will trip the targeted REJECT.
  2. Issues ``CFG_SET_REQ(CFG_KEY_REG_PROFILE, <profile_id>)``.
  3. Reads the ``CFG_OK_URC`` payload status byte and asserts it equals
     the expected ``cfg_status_t`` from
     ``host_cfg_profile_reject_to_cfg_status()`` (table mirrored in
     this module's :data:`REJECT_TO_STATUS`).
  4. Issues ``CFG_GET_REQ(CFG_KEY_REG_PROFILE)`` to confirm the stored
     profile byte either updated (happy path) or stayed at the prior
     value (reject path).
  5. Restores the cfg key it perturbed to its default so the next
     case starts from a known baseline (orchestrator-side cleanup;
     the firmware itself has no transactional rollback for the
     prerequisite writes).

CLI:

    py -3 cfg_fuzz_profile_lock.py --self-test
    py -3 cfg_fuzz_profile_lock.py --dev COM7 --baud 115200 \
        --out-log <path>.log [--stamp]

``--self-test`` runs the HW-free golden-vector tests for
:func:`build_cfg_set_payload`, :func:`parse_cfg_ok_payload`, and the
:data:`REJECT_TO_STATUS` table. No serial port is opened. Exits 0 on
clean pass, non-zero on any mismatch.

Live run (``--dev`` + ``--baud``) executes the full fuzz table against
a running firmware. ``--out-log`` writes a transcript of every case +
the canonical ``RUNTIME_PROFILE_ENUM=<N>`` line so FCC-B3-3 can verify
the runtime profile matches the artifact group. ``--stamp`` post-stamps
the log with ``artifact_header.py stamp --profile-enum <N>`` (subprocess
call) so the log is FCC-B2-b-conformant for bench-evidence submission.

Exit codes:
  0 = all cases passed (status byte + stored profile both matched)
  1 = at least one case failed (per-case diff printed to stdout/log)
  2 = transport setup failure (port open, BOOT_URC missing, etc.)

References:
  - LifeTrac-v25/TODO.md FCC-EVID-D6-2-b
  - host/host_cfg.c (cfg_set REG_PROFILE dispatcher, D6-2-a-iii)
  - host/host_cfg_profile.c (host_cfg_profile_reject_to_cfg_status)
  - bench/host_proto/cfg_profile_wire.c (the host-TU mirror of these
    same cases — golden source for expected_status values)
"""

from __future__ import annotations

import argparse
import dataclasses
import subprocess
import sys
import time
from pathlib import Path

# Re-use the SerialRPC framing from the x8 bootloader helper directory.
# Path layout: this file is .../LifeTrac-v25/tools/cfg_fuzz_profile_lock.py;
# the HostLink helper lives in
# .../LifeTrac-v25/DESIGN-CONTROLLER/firmware/x8_lora_bootloader_helper/.
_HELPER_DIR = (Path(__file__).resolve().parent.parent
               / "DESIGN-CONTROLLER" / "firmware"
               / "x8_lora_bootloader_helper")
if str(_HELPER_DIR) not in sys.path:
    sys.path.insert(0, str(_HELPER_DIR))


# ---------------------------------------------------------------------------
# Wire constants — mirrored from firmware/murata_l072/include/host_types.h
# and include/host_cfg.h / include/host_cfg_keys.h. These MUST stay in sync
# with the firmware; the self-test cross-checks the REJECT_TO_STATUS table
# against the firmware enum values pinned in cfg_profile.c.
# ---------------------------------------------------------------------------

HOST_TYPE_CFG_SET_REQ = 0x20
HOST_TYPE_CFG_GET_REQ = 0x21
HOST_TYPE_CFG_OK_URC = 0xA0
HOST_TYPE_CFG_DATA_URC = 0xA1

CFG_KEY_FHSS_CHANNEL_MASK = 0x07
CFG_KEY_REG_PROFILE       = 0x14
CFG_KEY_ANTENNA_GAIN_DBI  = 0x15
CFG_KEY_HW_CEILING_DBM    = 0x16

# Per host_cfg_keys.h.
CFG_KEY_VALUE_LEN = {
    CFG_KEY_FHSS_CHANNEL_MASK: 8,
    CFG_KEY_REG_PROFILE:       1,
    CFG_KEY_ANTENNA_GAIN_DBI:  1,
    CFG_KEY_HW_CEILING_DBM:    1,
}

# cfg_status_t values — see include/host_cfg.h.
CFG_STATUS_OK                              = 0
CFG_STATUS_UNKNOWN_KEY                     = 1
CFG_STATUS_BAD_LENGTH                      = 2
CFG_STATUS_OUT_OF_RANGE                    = 3
CFG_STATUS_READ_ONLY                       = 4
CFG_STATUS_APPLY_FAILED                    = 5
CFG_STATUS_DEFERRED                        = 6
CFG_STATUS_PROFILE_UNROUTED                = 7
CFG_STATUS_PROFILE_REJECT_MASK_POPCOUNT    = 8
CFG_STATUS_PROFILE_REJECT_MASK_OUT_OF_TABLE = 9
CFG_STATUS_PROFILE_REJECT_BW_MISMATCH      = 10
CFG_STATUS_PROFILE_REJECT_ANTENNA_OUT_OF_RANGE = 11
CFG_STATUS_PROFILE_REJECT_NO_POWER_HEADROOM    = 12
CFG_STATUS_PROFILE_REJECT_NOT_STAGED       = 13
CFG_STATUS_PROFILE_REJECT_NULL_ARG         = 14

# host_cfg_profile_reject_t -> cfg_status_t. Mirrors
# host/host_cfg_profile.c::host_cfg_profile_reject_to_cfg_status. The
# self-test asserts every entry; the firmware host TU
# bench/host_proto/cfg_profile.c pins the same table via direct C calls.
REJECT_TO_STATUS = {
    "NONE":                 CFG_STATUS_OK,
    "BAD_PROFILE":          CFG_STATUS_OUT_OF_RANGE,
    "UNROUTED":             CFG_STATUS_PROFILE_UNROUTED,
    "MASK_POPCOUNT":        CFG_STATUS_PROFILE_REJECT_MASK_POPCOUNT,
    "MASK_OUT_OF_TABLE":    CFG_STATUS_PROFILE_REJECT_MASK_OUT_OF_TABLE,
    "BW_MISMATCH":          CFG_STATUS_PROFILE_REJECT_BW_MISMATCH,
    "ANTENNA_OUT_OF_RANGE": CFG_STATUS_PROFILE_REJECT_ANTENNA_OUT_OF_RANGE,
    "NO_POWER_HEADROOM":    CFG_STATUS_PROFILE_REJECT_NO_POWER_HEADROOM,
    "NOT_STAGED":           CFG_STATUS_PROFILE_REJECT_NOT_STAGED,
    "NULL_ARG":             CFG_STATUS_PROFILE_REJECT_NULL_ARG,
}

REG_PROFILE_BENCH_ONLY_FIXED_915     = 0
REG_PROFILE_FCC_15_247_FHSS_50CH_BW250 = 1
REG_PROFILE_FCC_15_247_DTS_BW500     = 2
REG_PROFILE_MAX                       = 2

# Defaults — mirror host/host_cfg.c.
DEFAULT_FHSS_CHANNEL_MASK = 0x00000000000000FF
DEFAULT_ANTENNA_GAIN_DBI  = 2     # int8
DEFAULT_HW_CEILING_DBM    = 17

# host_cfg_profile.h: bits 0..49 = REQUIRED for FHSS profile.
FHSS_50CH_REQUIRED_MASK = (1 << 50) - 1


# ---------------------------------------------------------------------------
# Pure helpers (HW-free; covered by --self-test)
# ---------------------------------------------------------------------------

def u64_to_le_bytes(value: int) -> bytes:
    """Pack a uint64 in little-endian (matches read_u64_le in host_cfg.c)."""
    if value < 0 or value > 0xFFFFFFFFFFFFFFFF:
        raise ValueError(f"u64 out of range: {value!r}")
    return value.to_bytes(8, "little")


def i8_to_byte(value: int) -> bytes:
    """Pack a signed 8-bit value as the single-byte two's-complement
    encoding the firmware reads via (int8_t)bytes[0]."""
    if value < -128 or value > 127:
        raise ValueError(f"i8 out of range: {value!r}")
    return (value & 0xFF).to_bytes(1, "little")


def u8_to_byte(value: int) -> bytes:
    if value < 0 or value > 255:
        raise ValueError(f"u8 out of range: {value!r}")
    return value.to_bytes(1, "little")


def build_cfg_set_payload(key: int, value: bytes) -> bytes:
    """Build the CFG_SET_REQ frame payload per host_cmd.c::handle_cfg_set:
    payload = [key (u8), value_len (u8), value_bytes...]. The wire
    framing layer (HostLink.send) prepends type/seq/ver and appends the
    CRC + COBS, so this helper only emits the inner cfg payload.
    """
    if not 0 <= key <= 0xFF:
        raise ValueError(f"key out of range: 0x{key:X}")
    if len(value) > 0xFF:
        raise ValueError(f"value too long: {len(value)}")
    expected_len = CFG_KEY_VALUE_LEN.get(key)
    if expected_len is not None and len(value) != expected_len:
        raise ValueError(
            f"key 0x{key:02X} expects value_len={expected_len}, "
            f"got {len(value)}"
        )
    return bytes([key, len(value)]) + value


def parse_cfg_ok_payload(payload: bytes) -> dict:
    """Parse CFG_OK_URC payload per host/host_cfg_wire.c::
    host_cfg_wire_encode_ok: [key, status, actual_len, 0]. Returns a
    dict; raises ValueError on truncated input.
    """
    if len(payload) < 4:
        raise ValueError(
            f"CFG_OK_URC payload too short ({len(payload)}, "
            f"need >=4): {payload.hex()}"
        )
    return {
        "key":        payload[0],
        "status":     payload[1],
        "actual_len": payload[2],
        "reserved":   payload[3],
    }


def parse_cfg_data_payload(payload: bytes) -> dict:
    """Parse CFG_DATA_URC payload per host_cfg_wire_encode_data:
    [key, value_len, value_bytes...]. Returns dict with 'value' bytes."""
    if len(payload) < 2:
        raise ValueError(
            f"CFG_DATA_URC payload too short: {payload.hex()}"
        )
    key = payload[0]
    vlen = payload[1]
    if len(payload) < 2 + vlen:
        raise ValueError(
            f"CFG_DATA_URC truncated: got {len(payload)} need {2 + vlen}"
        )
    return {"key": key, "value_len": vlen,
            "value": bytes(payload[2:2 + vlen])}


# ---------------------------------------------------------------------------
# Fuzz table
# ---------------------------------------------------------------------------

@dataclasses.dataclass
class FuzzCase:
    name: str
    reject_class: str   # key into REJECT_TO_STATUS (or "OK" for happy paths)
    setup_writes: list  # list[(key, value_bytes)] applied via cfg_set in order
    profile_set: int    # profile_id byte to write via cfg_set(REG_PROFILE)
    expected_status: int
    # Profile byte cfg_get should return AFTER the case. None = "don't check".
    expected_stored_profile: int
    # Documentation-only — which reject class would naturally be reached
    # in firmware compiled WITH LIFETRAC_FHSS_TX_ROUTED. The wire reachable
    # set differs between routed/unrouted builds; cases that exercise the
    # routed-only paths are skipped on unrouted firmware (see is_skipped).
    requires_routed: bool = False


def build_cases() -> list:
    """Construct the canonical case table. Mirrors
    bench/host_proto/cfg_profile_wire.c 1:1 so a firmware-side or
    Python-side drift will fail one of the two TUs.
    """
    return [
        FuzzCase(
            name="bench_happy_default",
            reject_class="NONE",
            setup_writes=[],
            profile_set=REG_PROFILE_BENCH_ONLY_FIXED_915,
            expected_status=CFG_STATUS_OK,
            expected_stored_profile=REG_PROFILE_BENCH_ONLY_FIXED_915,
        ),
        FuzzCase(
            name="profile_unknown_oor",
            reject_class="BAD_PROFILE",  # mapped to OUT_OF_RANGE alias
            setup_writes=[],
            profile_set=REG_PROFILE_MAX + 1,
            expected_status=CFG_STATUS_OUT_OF_RANGE,
            expected_stored_profile=REG_PROFILE_BENCH_ONLY_FIXED_915,
        ),
        FuzzCase(
            name="antenna_out_of_range",
            reject_class="ANTENNA_OUT_OF_RANGE",
            setup_writes=[(CFG_KEY_ANTENNA_GAIN_DBI, i8_to_byte(31))],
            profile_set=REG_PROFILE_BENCH_ONLY_FIXED_915,
            expected_status=CFG_STATUS_PROFILE_REJECT_ANTENNA_OUT_OF_RANGE,
            expected_stored_profile=REG_PROFILE_BENCH_ONLY_FIXED_915,
        ),
        FuzzCase(
            name="no_power_headroom",
            reject_class="NO_POWER_HEADROOM",
            setup_writes=[(CFG_KEY_ANTENNA_GAIN_DBI, i8_to_byte(30))],
            profile_set=REG_PROFILE_BENCH_ONLY_FIXED_915,
            expected_status=CFG_STATUS_PROFILE_REJECT_NO_POWER_HEADROOM,
            expected_stored_profile=REG_PROFILE_BENCH_ONLY_FIXED_915,
        ),
        FuzzCase(
            name="fhss_unrouted_when_unrouted_build",
            reject_class="UNROUTED",
            setup_writes=[],
            profile_set=REG_PROFILE_FCC_15_247_FHSS_50CH_BW250,
            expected_status=CFG_STATUS_PROFILE_UNROUTED,
            expected_stored_profile=REG_PROFILE_BENCH_ONLY_FIXED_915,
        ),
        FuzzCase(
            name="dts_unrouted_when_unrouted_build",
            reject_class="UNROUTED",
            setup_writes=[],
            profile_set=REG_PROFILE_FCC_15_247_DTS_BW500,
            expected_status=CFG_STATUS_PROFILE_UNROUTED,
            expected_stored_profile=REG_PROFILE_BENCH_ONLY_FIXED_915,
        ),
        # ---- Routed-build-only cases (skipped on unrouted firmware) ----
        FuzzCase(
            name="fhss_happy_when_routed",
            reject_class="NONE",
            setup_writes=[(CFG_KEY_FHSS_CHANNEL_MASK,
                           u64_to_le_bytes(FHSS_50CH_REQUIRED_MASK))],
            profile_set=REG_PROFILE_FCC_15_247_FHSS_50CH_BW250,
            expected_status=CFG_STATUS_OK,
            expected_stored_profile=REG_PROFILE_FCC_15_247_FHSS_50CH_BW250,
            requires_routed=True,
        ),
        FuzzCase(
            name="fhss_mask_popcount",
            reject_class="MASK_POPCOUNT",
            setup_writes=[(CFG_KEY_FHSS_CHANNEL_MASK,
                           u64_to_le_bytes(0x1))],
            profile_set=REG_PROFILE_FCC_15_247_FHSS_50CH_BW250,
            expected_status=CFG_STATUS_PROFILE_REJECT_MASK_POPCOUNT,
            expected_stored_profile=REG_PROFILE_BENCH_ONLY_FIXED_915,
            requires_routed=True,
        ),
        FuzzCase(
            name="fhss_mask_out_of_table",
            reject_class="MASK_OUT_OF_TABLE",
            setup_writes=[(CFG_KEY_FHSS_CHANNEL_MASK,
                           u64_to_le_bytes((1 << 50) | 0x1))],
            profile_set=REG_PROFILE_FCC_15_247_FHSS_50CH_BW250,
            expected_status=CFG_STATUS_PROFILE_REJECT_MASK_OUT_OF_TABLE,
            expected_stored_profile=REG_PROFILE_BENCH_ONLY_FIXED_915,
            requires_routed=True,
        ),
    ]


# ---------------------------------------------------------------------------
# Self-test (HW-free)
# ---------------------------------------------------------------------------

def _self_test_payload_builders() -> int:
    fails = 0

    def check(label: str, got, expected) -> None:
        nonlocal fails
        if got == expected:
            print(f"SELF_TEST: PASS  {label}")
        else:
            fails += 1
            print(f"SELF_TEST: FAIL  {label}")
            print(f"  expected: {expected!r}")
            print(f"  got:      {got!r}")

    # ---- u64/i8/u8 packers ----
    check("u64 le 0x0000_0000_0000_00FF",
          u64_to_le_bytes(0xFF),
          bytes([0xFF, 0, 0, 0, 0, 0, 0, 0]))
    check("u64 le FHSS_50CH_REQUIRED_MASK",
          u64_to_le_bytes(FHSS_50CH_REQUIRED_MASK),
          bytes([0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x03, 0]))
    check("u64 le (1<<50)|1",
          u64_to_le_bytes((1 << 50) | 1),
          bytes([0x01, 0, 0, 0, 0, 0, 0x04, 0]))
    check("i8 -5 -> 0xFB", i8_to_byte(-5), bytes([0xFB]))
    check("i8 30 -> 0x1E",  i8_to_byte(30),  bytes([0x1E]))
    check("u8 17 -> 0x11", u8_to_byte(17), bytes([0x11]))

    # ---- build_cfg_set_payload ----
    check("cfg_set REG_PROFILE=0",
          build_cfg_set_payload(CFG_KEY_REG_PROFILE, bytes([0])),
          bytes([0x14, 0x01, 0x00]))
    check("cfg_set ANTENNA_GAIN_DBI=31",
          build_cfg_set_payload(CFG_KEY_ANTENNA_GAIN_DBI, i8_to_byte(31)),
          bytes([0x15, 0x01, 0x1F]))
    check("cfg_set FHSS_CHANNEL_MASK=REQUIRED",
          build_cfg_set_payload(
              CFG_KEY_FHSS_CHANNEL_MASK,
              u64_to_le_bytes(FHSS_50CH_REQUIRED_MASK)),
          bytes([0x07, 0x08, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x03, 0]))

    # ---- length-mismatch rejection ----
    try:
        build_cfg_set_payload(CFG_KEY_REG_PROFILE, bytes([0, 1]))
    except ValueError as exc:
        check("len-mismatch raises", True, True)
        check("len-mismatch text mentions key",
              "0x14" in str(exc), True)
    else:
        check("len-mismatch raises", False, True)

    # ---- parse_cfg_ok_payload ----
    check("parse OK payload",
          parse_cfg_ok_payload(bytes([0x14, 0x00, 0x01, 0x00])),
          {"key": 0x14, "status": 0, "actual_len": 1, "reserved": 0})
    check("parse OK reject MASK_POPCOUNT",
          parse_cfg_ok_payload(bytes([0x14, 0x08, 0x01, 0x00])),
          {"key": 0x14, "status": 0x08, "actual_len": 1, "reserved": 0})
    try:
        parse_cfg_ok_payload(bytes([0x14, 0x00]))
    except ValueError:
        check("parse OK truncated raises", True, True)
    else:
        check("parse OK truncated raises", False, True)

    # ---- parse_cfg_data_payload ----
    check("parse DATA payload REG_PROFILE=2",
          parse_cfg_data_payload(bytes([0x14, 0x01, 0x02])),
          {"key": 0x14, "value_len": 1, "value": bytes([0x02])})

    # ---- REJECT_TO_STATUS table ----
    # Pin every numeric value so the firmware enum (host_cfg.h) and
    # this Python mirror cannot drift silently.
    expected_table = {
        "NONE": 0,
        "BAD_PROFILE": 3,
        "UNROUTED": 7,
        "MASK_POPCOUNT": 8,
        "MASK_OUT_OF_TABLE": 9,
        "BW_MISMATCH": 10,
        "ANTENNA_OUT_OF_RANGE": 11,
        "NO_POWER_HEADROOM": 12,
        "NOT_STAGED": 13,
        "NULL_ARG": 14,
    }
    check("REJECT_TO_STATUS table matches firmware enum",
          REJECT_TO_STATUS, expected_table)

    # ---- case table sanity ----
    cases = build_cases()
    case_names = [c.name for c in cases]
    check("case names unique",
          len(case_names) == len(set(case_names)), True)
    for c in cases:
        check(f"case {c.name!r} expected_status matches table",
              c.expected_status, REJECT_TO_STATUS[c.reject_class])

    return 0 if fails == 0 else 1


def run_self_test() -> int:
    rc = _self_test_payload_builders()
    if rc == 0:
        print("SELF_TEST: all golden vectors passed")
    else:
        print(f"SELF_TEST: {rc} failure(s)")
    return rc


# ---------------------------------------------------------------------------
# Live run
# ---------------------------------------------------------------------------

def _do_cfg_set(link, key: int, value: bytes,
                timeout: float = 1.0) -> dict:
    """Send CFG_SET_REQ and parse the matching CFG_OK_URC. Returns the
    parsed payload dict (key/status/actual_len/reserved)."""
    payload = build_cfg_set_payload(key, value)
    frame = link.request(HOST_TYPE_CFG_SET_REQ, HOST_TYPE_CFG_OK_URC,
                         payload, timeout=timeout)
    return parse_cfg_ok_payload(frame["payload"])


def _do_cfg_get(link, key: int, timeout: float = 1.0) -> dict:
    frame = link.request(HOST_TYPE_CFG_GET_REQ, HOST_TYPE_CFG_DATA_URC,
                         bytes([key]), timeout=timeout)
    return parse_cfg_data_payload(frame["payload"])


def _restore_defaults(link, log) -> None:
    """Best-effort restore of perturbed cfg keys. Any failure is logged
    but does not abort the run — the orchestrator's only contract is to
    report per-case status; a residual mask/antenna value just means
    the next case may see a different starting point, which the
    per-case ``setup_writes`` handle explicitly."""
    for key, value in (
        (CFG_KEY_FHSS_CHANNEL_MASK,
         u64_to_le_bytes(DEFAULT_FHSS_CHANNEL_MASK)),
        (CFG_KEY_ANTENNA_GAIN_DBI,
         i8_to_byte(DEFAULT_ANTENNA_GAIN_DBI)),
        (CFG_KEY_HW_CEILING_DBM,
         u8_to_byte(DEFAULT_HW_CEILING_DBM)),
    ):
        try:
            _do_cfg_set(link, key, value)
        except Exception as exc:
            log(f"WARN: restore key 0x{key:02X} failed: "
                f"{type(exc).__name__}: {exc}")


def _read_runtime_profile_enum(link, log) -> int:
    """Read CFG_KEY_REG_PROFILE and return the enum byte (or -1 on
    failure). Always emits a canonical RUNTIME_PROFILE_ENUM=... line
    to satisfy FCC-B3-3."""
    try:
        parsed = _do_cfg_get(link, CFG_KEY_REG_PROFILE)
        if parsed["value_len"] != 1:
            log(f"RUNTIME_PROFILE_ENUM=ERR wrong_value_len:"
                f"{parsed['value_len']}")
            return -1
        enum_val = parsed["value"][0]
        log(f"RUNTIME_PROFILE_ENUM={enum_val}")
        return enum_val
    except Exception as exc:
        log(f"RUNTIME_PROFILE_ENUM=ERR request_failed:"
            f"{type(exc).__name__}")
        return -1


class _PySerialHostLink:
    """Cross-platform (Windows-compatible) HostLink shim.

    method_g_stage1_probe.HostLink uses POSIX ``os.open`` + ``stty`` and
    only runs on Linux/macOS. This shim provides the subset of HostLink
    that the orchestrator needs (``request()`` + ``close()``) using
    pyserial, so the live fuzz can run from the Windows bench host
    without WSL/usbipd round-tripping. Wire framing
    (``build_frame``/``parse_frame``) is imported from the same module
    so both paths agree on COBS+CRC layout.
    """

    def __init__(self, dev: str, baud) -> None:
        import serial  # type: ignore
        from method_g_stage1_probe import (  # type: ignore
            build_frame, parse_frame, HOST_TYPE_ERR_PROTO_URC,
        )
        self._build_frame = build_frame
        self._parse_frame = parse_frame
        self._ERR_PROTO = HOST_TYPE_ERR_PROTO_URC
        self.ser = serial.Serial(dev, int(baud), timeout=0.1,
                                 write_timeout=1.0)
        self.rx_buf = bytearray()
        self.seq = 1
        self.urc_queue = []

    def close(self) -> None:
        try:
            self.ser.close()
        except Exception:
            pass

    def _next_seq(self) -> int:
        seq = self.seq
        self.seq = (self.seq + 1) & 0xFFFF
        if self.seq == 0:
            self.seq = 1
        return seq

    def _read_frames(self, timeout: float):
        """Return list of parsed frames received within ``timeout`` s,
        plus any previously queued URCs."""
        frames = []
        if self.urc_queue:
            frames.extend(self.urc_queue)
            self.urc_queue.clear()
            return frames
        deadline = time.time() + timeout
        while time.time() < deadline:
            chunk = self.ser.read(1024)
            if chunk:
                self.rx_buf.extend(chunk)
                while True:
                    try:
                        start = self.rx_buf.index(0)
                    except ValueError:
                        self.rx_buf.clear()
                        break
                    if start > 0:
                        del self.rx_buf[:start]
                    try:
                        end = self.rx_buf.index(0, 1)
                    except ValueError:
                        break
                    payload = bytes(self.rx_buf[1:end])
                    del self.rx_buf[: end + 1]
                    if not payload:
                        continue
                    try:
                        frames.append(self._parse_frame(payload))
                    except ValueError as exc:
                        print(f"WARNING: dropped malformed frame: {exc}")
                        continue
                if frames:
                    return frames
        return frames

    def request(self, req_type: int, rsp_type: int,
                payload: bytes = b"", timeout: float = 1.0) -> dict:
        seq = self._next_seq()
        self.ser.write(self._build_frame(req_type, seq, payload, 0))
        deadline = time.time() + timeout
        pending = list(self.urc_queue)
        self.urc_queue.clear()
        while True:
            if not pending:
                if time.time() >= deadline:
                    raise TimeoutError(
                        f"timeout waiting for response type "
                        f"0x{rsp_type:02X} to req 0x{req_type:02X}"
                    )
                pending = self._read_frames(0.1)
            keep = []
            matched = None
            for idx, frame in enumerate(pending):
                if frame["type"] == rsp_type and frame["seq"] == seq:
                    matched = (idx, frame)
                    break
                if (frame["type"] == self._ERR_PROTO
                        and frame["seq"] == seq):
                    self.urc_queue.extend(keep + pending[idx + 1:])
                    raise RuntimeError(
                        f"ERR_PROTO for req 0x{req_type:02X}"
                    )
                if frame["type"] == rsp_type:
                    # Stale type-match from prior timed-out request.
                    continue
                keep.append(frame)
            if matched is not None:
                idx, frame = matched
                leftovers = keep + pending[idx + 1:]
                if leftovers:
                    self.urc_queue.extend(leftovers)
                return frame
            if keep:
                self.urc_queue.extend(keep)
            pending = []


def run_live(dev: str, baud: str, out_log_path,
             stamp: bool) -> int:
    """Open the link, run every case, write the transcript, optionally
    stamp the log with the FCC-B2-b artifact header. Returns the
    process exit code."""
    # Use the pyserial-backed shim so the orchestrator runs on both
    # POSIX (where the original HostLink works) and Windows (where it
    # does not — see _PySerialHostLink docstring).
    HostLink = _PySerialHostLink  # noqa: N806

    log_lines = []

    def log(msg: str) -> None:
        print(msg)
        log_lines.append(msg)

    log("# FCC-EVID-D6-2-b cfg_set(REG_PROFILE) fuzz orchestrator")
    log(f"# dev={dev} baud={baud} ts={int(time.time())}")

    rc = 0
    link = None
    try:
        link = HostLink(dev, baud)
    except Exception as exc:
        print(f"ERROR: cannot open link {dev}@{baud}: "
              f"{type(exc).__name__}: {exc}", file=sys.stderr)
        return 2

    try:
        # FCC-B3-1: emit the canonical RUNTIME_PROFILE_ENUM line first so
        # the artifact-header stamp + FCC-B3-3 gate sees it regardless of
        # how many cases run / fail.
        runtime_enum = _read_runtime_profile_enum(link, log)

        cases = build_cases()
        # If runtime is the bench/unrouted build, skip cases marked
        # requires_routed. If runtime is a routed build, skip the
        # "*_unrouted_when_unrouted_build" cases (they would fail).
        # We have no direct readback for LIFETRAC_FHSS_TX_ROUTED, so
        # we infer: bench-only runtime (enum=0) is the only
        # build whose unrouted-case expectations are valid; any
        # production runtime is treated as routed.
        runtime_is_routed = (runtime_enum != REG_PROFILE_BENCH_ONLY_FIXED_915
                             and runtime_enum != -1)

        passes = 0
        fails = 0
        skipped = 0
        for c in cases:
            if c.requires_routed and not runtime_is_routed:
                log(f"SKIP {c.name}: requires_routed build "
                    f"(runtime_enum={runtime_enum})")
                skipped += 1
                continue
            if (not c.requires_routed
                    and c.reject_class == "UNROUTED"
                    and runtime_is_routed):
                log(f"SKIP {c.name}: unrouted-only case on routed build")
                skipped += 1
                continue

            try:
                for (sk, sv) in c.setup_writes:
                    sr = _do_cfg_set(link, sk, sv)
                    if sr["status"] != CFG_STATUS_OK:
                        log(f"FAIL {c.name}: setup write key 0x{sk:02X} "
                            f"returned status={sr['status']}")
                        fails += 1
                        _restore_defaults(link, log)
                        continue

                got = _do_cfg_set(link, CFG_KEY_REG_PROFILE,
                                  bytes([c.profile_set]))
                stored = _do_cfg_get(link, CFG_KEY_REG_PROFILE)
                stored_byte = (stored["value"][0]
                               if stored["value_len"] == 1 else -1)

                ok_status = got["status"] == c.expected_status
                ok_stored = (c.expected_stored_profile is None
                             or stored_byte == c.expected_stored_profile)

                if ok_status and ok_stored:
                    log(f"PASS {c.name}: status={got['status']} "
                        f"stored=0x{stored_byte:02X}")
                    passes += 1
                else:
                    fails += 1
                    log(f"FAIL {c.name}:")
                    log(f"  expected_status={c.expected_status} "
                        f"got_status={got['status']}")
                    log(f"  expected_stored="
                        f"{c.expected_stored_profile} "
                        f"got_stored={stored_byte}")
            except Exception as exc:
                fails += 1
                log(f"FAIL {c.name}: exception "
                    f"{type(exc).__name__}: {exc}")
            finally:
                _restore_defaults(link, log)

        log(f"# SUMMARY passes={passes} fails={fails} skipped={skipped}")
        rc = 0 if fails == 0 else 1
    finally:
        try:
            if link is not None:
                link.close()
        except Exception:
            pass

    if out_log_path is not None:
        out_log_path = Path(out_log_path)
        out_log_path.parent.mkdir(parents=True, exist_ok=True)
        out_log_path.write_text("\n".join(log_lines) + "\n",
                                encoding="utf-8", newline="")
        print(f"# wrote {out_log_path}")

        if stamp:
            stamp_rc = _stamp_log(out_log_path, runtime_enum)
            if stamp_rc != 0:
                print(f"WARN: artifact_header stamp returned {stamp_rc}",
                      file=sys.stderr)

    return rc


def _stamp_log(log_path: Path, profile_enum: int) -> int:
    """Shell out to tools/artifact_header.py stamp. Kept as a
    subprocess (not in-process import) so this orchestrator stays
    decoupled from the stamper's import-time side-effects (it
    harvests firmware sources at module load)."""
    if profile_enum < 0:
        print("WARN: profile_enum unknown — skipping stamp",
              file=sys.stderr)
        return 1
    stamper = Path(__file__).resolve().parent / "artifact_header.py"
    cmd = [sys.executable, str(stamper), "stamp",
           "--input", str(log_path),
           "--output", str(log_path),
           "--profile-enum", str(profile_enum)]
    try:
        return subprocess.call(cmd)
    except OSError as exc:
        print(f"WARN: stamp subprocess failed: {exc}", file=sys.stderr)
        return 1


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def main(argv) -> int:
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument("--dev", help="serial device (e.g. COM7 or "
                                  "/dev/ttyUSB0)")
    p.add_argument("--baud", default="115200",
                   help="baud rate (default 115200)")
    p.add_argument("--out-log", help="path to write the run transcript")
    p.add_argument("--self-test", action="store_true",
                   help="run HW-free golden-vector tests and exit")
    p.add_argument("--stamp", action="store_true",
                   help="post-stamp --out-log with the FCC-B2-b "
                        "artifact header (subprocess to "
                        "artifact_header.py stamp)")
    args = p.parse_args(argv)

    if args.self_test:
        return run_self_test()

    if not args.dev:
        p.error("--dev is required for a live run "
                "(or use --self-test)")
    return run_live(args.dev, args.baud, args.out_log, args.stamp)


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
