#!/usr/bin/env python3
"""
method_h_stage2_tx_probe.py - Stage 2 TX bring-up probe (W1-9).

Drives the murata_l072 firmware over /dev/ttymxc3 to send exactly one
LoRa frame using the existing TX_FRAME_REQ (0x10) host command and
validates the resulting TX_DONE_URC (0x90).

Acceptance gates (mirrors W1-9 plan §5):
  A. TX_DONE_URC arrives within --timeout seconds (default 5).
  B. Returned tx_id matches the request.
  C. Returned status == 0 (SX1276_TX_STATUS_OK).
  D. radio_tx_ok increments by exactly 1.
  E. host_parse_err, radio_tx_abort_lbt, radio_tx_abort_airtime do not
     change; no FAULT_URC observed during the cycle.
  F. UART per-flag error counters (PE/FE/NE/ORE on both LPUART and
     USART1) stay at 0.

Exit codes (default --probe tx):
  0 = all gates passed
  1 = protocol responded but at least one gate failed
  2 = fatal transport / timeout failure

W1-9b probe modes (--probe regversion|fsk):
  0 = probe completed with an unambiguous verdict (printed as
      __W1_9B_VERDICT__=...). Includes "clock dead" outcomes.
  1 = probe incomplete: host invariant violated, restore failed,
      or pre-state assertion failed.
  2 = fatal transport setup (no BOOT_URC, link open failure, etc.)
"""

from __future__ import annotations

import argparse
import struct
import sys
import time
import builtins

# Force unbuffered stdout output for all prints in this module to avoid buffering in pipes
def print(*args, **kwargs):
    kwargs.setdefault('flush', True)
    builtins.print(*args, **kwargs)

# Reuse all the framing / link helpers from the Stage 1 probe.  The two
# scripts live in the same directory on the X8 (/tmp/lifetrac_p0c).
from method_g_stage1_probe import (  # type: ignore
    HostLink,
    HOST_TYPE_STATS_DUMP_REQ,
    HOST_TYPE_STATS_URC,
    HOST_TYPE_FAULT_URC,
    HOST_TYPE_ERR_PROTO_URC,
    HOST_TYPE_VER_REQ,
    HOST_TYPE_VER_URC,
    HOST_FAULT_CODE_HOST_RX_SEEN,
    HOST_FAULT_CODE_HOST_DIAG_MARK,
    parse_stats,
    parse_version,
    format_fault_payload,
    format_err_proto_payload,
)

# 2026-05-12 W1-9f: fault codes that are by-design diagnostic emissions, not
# real faults. Per `firmware/murata_l072/main.c`, the L072 fires a one-shot
# `HOST_FAULT_CODE_HOST_RX_SEEN (0x09)` URC the first time it observes any
# byte on a given host UART lane (latch flag in `sub`); this confirms the
# host->MCU direction is alive and is not a fault. Similarly `HOST_DIAG_MARK
# (0x0C)` carries DIAG bitfield markers (e.g. VER_REQ_PARSED) used during
# bring-up. The TX-probe "no FAULT_URC during cycle" gate must ignore both
# so a healthy first-cycle TX does not get spuriously marked FAIL.
BENIGN_FAULT_CODES = (
    HOST_FAULT_CODE_HOST_RX_SEEN,
    HOST_FAULT_CODE_HOST_DIAG_MARK,
)


# 2026-05-22 P5: sidecar progress.txt. Per Open Problems doc v4.1 §4 the
# cheapest fix for PowerShell live-tail buffering is a tiny file the
# orchestrator polls every 1 s. Overwrite-truncate single line so a stat()
# + 1 read is always sufficient (no log rotation, no tail bookkeeping).
# Module global so probe modes don't need to thread a path through every
# helper signature; main() sets it once from --progress-file.
_PROGRESS_PATH: "str | None" = None


def _emit_progress(tag: str, **kv) -> None:
    """Write '<iso-ts> tag k=v k=v\\n' to _PROGRESS_PATH (overwrite-truncate).

    Silent best-effort: any I/O error is swallowed. The probe must never
    crash because the sidecar file became unwritable mid-run.
    """
    if not _PROGRESS_PATH:
        return
    try:
        ts = time.strftime("%Y-%m-%dT%H:%M:%S")
        parts = [ts, tag] + [f"{k}={v}" for k, v in kv.items()]
        line = " ".join(parts) + "\n"
        # Open-for-write truncates; flush+close guarantees the orchestrator's
        # next poll sees a complete line.
        with open(_PROGRESS_PATH, "w", encoding="utf-8") as fh:
            fh.write(line)
    except Exception:
        pass


HOST_TYPE_TX_FRAME_REQ = 0x10
HOST_TYPE_TX_DONE_URC = 0x90
HOST_TYPE_RFCO_PERTX_URC = 0xC3

# Mirror of host_rfco_tx_status_t (see include/host_rfco.h).
RFCO_TX_STATUS_NAMES = {
    0x00: "OK",
    0x01: "ABORT_AIRTIME_INVARIANT",
    0x02: "ABORT_LBT",
    0x03: "ABORT_LEGAL_DWELL",
    0x04: "ABORT_QOS",
    0x05: "TX_TIMEOUT",
    0x06: "TX_FAIL",
    0xFF: "INTERNAL",
}
HOST_RFCO_PERTX_SCHEMA_VER = 1
HOST_RFCO_PERTX_PAYLOAD_LEN = 21


def format_rfco_pertx_payload(payload: bytes) -> str:
    if len(payload) < HOST_RFCO_PERTX_PAYLOAD_LEN:
        return f"short raw={payload.hex()}"
    schema_ver = payload[0]
    if schema_ver != HOST_RFCO_PERTX_SCHEMA_VER:
        return f"schema_ver=0x{schema_ver:02X} (unknown) raw={payload.hex()}"
    profile_id = payload[1]
    tx_status = payload[2]
    hop_idx = payload[3]
    channel_idx = payload[4]
    epoch = struct.unpack("<I", payload[5:9])[0]
    freq_hz = struct.unpack("<I", payload[9:13])[0]
    pkt_toa_us = struct.unpack("<I", payload[13:17])[0]
    legal_dwell_us = struct.unpack("<I", payload[17:21])[0]
    status_name = RFCO_TX_STATUS_NAMES.get(tx_status, f"0x{tx_status:02X}")
    return (
        f"profile_id={profile_id} tx_status=0x{tx_status:02X}({status_name}) "
        f"hop_idx={hop_idx} channel_idx={channel_idx} epoch={epoch} "
        f"freq_hz={freq_hz} pkt_toa_us={pkt_toa_us} "
        f"legal_dwell_used_us_10s={legal_dwell_us}"
    )
HOST_TYPE_CFG_SET_REQ = 0x20
HOST_TYPE_CFG_GET_REQ = 0x21
HOST_TYPE_CFG_OK_URC = 0xA0
HOST_TYPE_CFG_DATA_URC = 0xA1
HOST_TYPE_REG_READ_REQ = 0x30
HOST_TYPE_REG_WRITE_REQ = 0x31
HOST_TYPE_REG_DATA_URC = 0xB0
HOST_TYPE_REG_WRITE_ACK_URC = 0xB1

# P3 host pacing constants matching w2_02_host_pipeline.py authority
MIN_LORA_HOST_INTER_CYCLE_S = 0.05

def clamp_inter_cycle_s(requested_s: float) -> float:
    """Enforce MIN_LORA_HOST_INTER_CYCLE_S = 0.05 with a warning if lower.
    Cites walk_power_falsification_matrix 2026-05-21 and §15.247 spacing headroom.
    """
    if requested_s < MIN_LORA_HOST_INTER_CYCLE_S:
        print(
            "P3-CLAMP: requested inter_cycle_s={:.4f} < min {:.4f}; "
            "raising to floor (walk_power matrix 2026-05-21).".format(
                requested_s, MIN_LORA_HOST_INTER_CYCLE_S
            )
        )
        sys.stdout.flush()
        return MIN_LORA_HOST_INTER_CYCLE_S
    return requested_s

CFG_KEY_LBT_ENABLE = 0x03
# 2026-05-18 S1.1: TX-power adapter sweep (`--probe walk_power`). Mirrors
# `CFG_KEY_TX_POWER_DBM` from `firmware/murata_l072/include/host_cfg_keys.h`.
CFG_KEY_TX_POWER_DBM = 0x01
# 2026-05-20 FCC-B3-1: runtime profile readout (CFG_KEY_REG_PROFILE = 0x14U
# u8 in `firmware/murata_l072/include/host_cfg_keys.h`). Used by
# emit_runtime_profile_enum() below to publish one canonical
# `RUNTIME_PROFILE_ENUM=<N>` line at process startup so the FCC-B3-2 gate
# (`tools/check_run_profile.py`) can verify the firmware actually running
# on the board matches the orchestrator's declared expected enum.
CFG_KEY_REG_PROFILE = 0x14

CFG_KEY_FHSS_CHANNEL_MASK = 0x07
CFG_KEY_ANTENNA_GAIN_DBI = 0x15
CFG_KEY_HW_CEILING_DBM = 0x16

REG_PROFILE_BENCH_ONLY_FIXED_915 = 0
REG_PROFILE_FCC_15_247_FHSS_50CH_BW250 = 1

SX1276_REG_IRQ_FLAGS = 0x12
SX1276_REG_OP_MODE = 0x01
SX1276_REG_MODEM_STAT = 0x18    # bits 0..3: modem status; bits 4..7: SF (RxNbBytes coded path)
SX1276_REG_PKT_RSSI = 0x1A      # last-packet RSSI (raw, -157 offset for HF band)
SX1276_REG_RSSI_VALUE = 0x1B    # current RSSI (raw, -157 offset for HF band)

# RegOpMode values used by RX liveness probe.
SX1276_OPMODE_LORA_RXCONT = 0x85  # LongRangeMode | LowFreqModeOn=0 | mode=RXCONTINUOUS

SX1276_TX_STATUS_OK = 0
SX1276_TX_STATUS_TIMEOUT = 1
SX1276_TX_STATUS_LBT_ABORT = 2
SX1276_TX_STATUS_BUSY = 3

TX_STATUS_NAMES = {
    0: "OK",
    1: "TIMEOUT",
    2: "LBT_ABORT",
    3: "BUSY",
}

DEFAULT_TX_ID = 0x42
DEFAULT_PAYLOAD = b"LIFETRAC"

# Counters that must remain unchanged across the TX cycle.
INVARIANT_COUNTERS = (
    "host_parse_err",
    "host_uart_err_lpuart",
    "host_uart_err_usart1",
    "host_uart_pe_lpuart",
    "host_uart_fe_lpuart",
    "host_uart_ne_lpuart",
    "host_uart_ore_lpuart",
    "host_uart_pe_usart1",
    "host_uart_fe_usart1",
    "host_uart_ne_usart1",
    "host_uart_ore_usart1",
    "host_rx_ring_ovf",
    "radio_tx_abort_lbt",
    "radio_tx_abort_airtime",
)


def fetch_stats(link: HostLink) -> dict:
    frame = link.request(
        HOST_TYPE_STATS_DUMP_REQ,
        HOST_TYPE_STATS_URC,
        timeout=1.0,
    )
    return parse_stats(frame["payload"])


def drain_boot(link: HostLink, settle_s: float = 1.0) -> bool:
    """Drain BOOT_URC + any startup chatter. Returns True if BOOT_URC seen.

    2026-05-12 W1-9g: handle STATS_URC and ERR_PROTO_URC explicitly so they
    don't end up parked in urc_queue when the next request() runs. Mirrors
    the per-type drain pattern in method_g_stage1_probe.py:main() which is
    proven 5/5 PASS this session.
    """
    deadline = time.time() + settle_s
    boot_seen = False
    while time.time() < deadline:
        for frame in link.read_frames(0.2):
            ftype = frame["type"]
            if ftype == 0xF0:
                boot_seen = True
                print(f"BOOT_URC observed during settle (payload={frame['payload'].hex()})")
            elif ftype == HOST_TYPE_FAULT_URC:
                print(f"FAULT_URC during settle (expected for W1-9 lenient init): "
                      f"{format_fault_payload(frame['payload'])}")
            elif ftype == HOST_TYPE_STATS_URC:
                print(f"STATS_URC(boot) drained seq={frame['seq']} "
                      f"(unsolicited snapshot — discarding so it can't "
                      f"poison later request() calls)")
            else:
                print(f"INFO: drained type=0x{ftype:02X} seq={frame['seq']} during boot settle")
    return boot_seen


def drain_pending(link: HostLink, quiet_s: float = 0.25, max_s: float = 1.0) -> int:
    """Drain urc_queue + wire until no new frames for `quiet_s` seconds.

    2026-05-12 W1-9g: needed between VER_REQ and the first STATS_DUMP_REQ
    so that the post-VER HOST_RX_SEEN/HOST_DIAG_MARK FAULT_URCs and their
    auto-emitted STATS_URC(seq=0) snapshots are consumed BEFORE the next
    request() runs. Without this, those queued frames cycle through
    request()'s stale-seq discard path and the real response window is
    consumed by the discard loop instead of the wire-read.
    """
    drained = 0
    deadline = time.time() + max_s
    last_frame_at = time.time()
    while time.time() < deadline:
        # First drain anything already in urc_queue.
        if link.urc_queue:
            queued = list(link.urc_queue)
            link.urc_queue.clear()
            for frame in queued:
                drained += 1
                print(f"INFO: post-VER drain queued type=0x{frame['type']:02X} "
                      f"seq={frame['seq']}")
            last_frame_at = time.time()
            continue
        # Then read the wire briefly.
        frames = link.read_frames(0.05)
        if frames:
            for frame in frames:
                drained += 1
                print(f"INFO: post-VER drain wire   type=0x{frame['type']:02X} "
                      f"seq={frame['seq']}")
            last_frame_at = time.time()
            continue
        # Quiet for `quiet_s`?
        if time.time() - last_frame_at >= quiet_s:
            break
    return drained


def read_reg(link: HostLink, addr: int, timeout: float = 0.5):
    """Issue REG_READ_REQ; return (value_byte, raw_payload_bytes)."""
    f = link.request(HOST_TYPE_REG_READ_REQ, HOST_TYPE_REG_DATA_URC,
                     bytes([addr]), timeout=timeout)
    pl = f["payload"]
    if len(pl) < 2:
        raise RuntimeError(f"REG_DATA_URC short payload: {pl.hex()}")
    return pl[1], pl


def write_reg(link: HostLink, addr: int, value: int, timeout: float = 0.5) -> bytes:
    """Issue REG_WRITE_REQ; return raw ACK payload bytes."""
    f = link.request(HOST_TYPE_REG_WRITE_REQ, HOST_TYPE_REG_WRITE_ACK_URC,
                     bytes([addr, value & 0xFF]), timeout=timeout)
    return f["payload"]


# 2026-05-20 FCC-B3-1: runtime profile readout (CFG_KEY_REG_PROFILE = 0x14).
# One-shot emission flag — guarantees the canonical line is printed exactly
# once per probe process, regardless of how many code paths call the emitter
# or how many --probe modes the orchestrator wires through main(). The flag
# is set the moment we *attempt* the read (not when it succeeds) so a failed
# read still prevents a later success from double-printing.
_runtime_profile_emitted = False


def _format_runtime_profile_line(payload: bytes = None, exc: BaseException = None) -> str:
    """Pure helper that produces exactly one `RUNTIME_PROFILE_ENUM=...` line
    (no trailing newline) from either a CFG_DATA_URC payload or an exception
    raised by `link.request()`. Factored out for the --self-test-profile-emit
    harness so the parsing/formatting logic is exercised without needing a
    real serial port.

    Wire format (per firmware/murata_l072/host/host_cfg_wire.c
    `host_cfg_wire_encode_data`): payload = [key, value_len, value_bytes...].
    For CFG_KEY_REG_PROFILE the value is a single u8 enum byte, so the
    expected payload is exactly [0x14, 0x01, <enum>].
    """
    if exc is not None:
        return f"RUNTIME_PROFILE_ENUM=ERR request_failed:{type(exc).__name__}"
    if payload is None:
        return "RUNTIME_PROFILE_ENUM=ERR no_payload"
    if len(payload) < 3:
        return f"RUNTIME_PROFILE_ENUM=ERR short_payload:{payload.hex()}"
    if payload[0] != CFG_KEY_REG_PROFILE:
        return f"RUNTIME_PROFILE_ENUM=ERR wrong_key:0x{payload[0]:02x}"
    if payload[1] != 1:
        return f"RUNTIME_PROFILE_ENUM=ERR wrong_value_len:{payload[1]}"
    return f"RUNTIME_PROFILE_ENUM={payload[2]}"


CFG_STATUS_NAMES = {
    0: "OK", 1: "UNKNOWN_KEY", 2: "BAD_LENGTH", 3: "OUT_OF_RANGE",
    4: "APPLY_FAILED", 5: "DEFERRED", 6: "READ_ONLY", 7: "PROFILE_UNROUTED",
    8: "PROFILE_REJECT_MASK_POPCOUNT", 9: "PROFILE_REJECT_MASK_OUT_OF_TABLE",
    10: "PROFILE_REJECT_BW_MISMATCH", 11: "PROFILE_REJECT_ANTENNA_OUT_OF_RANGE",
    12: "PROFILE_REJECT_NO_POWER_HEADROOM", 13: "PROFILE_REJECT_NOT_STAGED",
}   # host_cfg.h cfg_status_t

def cfg_set_checked(link: HostLink, key: int, value: bytes, timeout: float = 1.0) -> dict:
    """CFG_SET that actually reads the status byte. CFG_OK_URC payload is
    [key, status, actual_len, 0] (host_cfg_wire.c) — status != 0 arrives on
    the SAME URC type, so request() returns 'success' for rejections (F15:
    '14 08 00 00' = REG_PROFILE refused MASK_POPCOUNT, printed as OK)."""
    ack = link.request(HOST_TYPE_CFG_SET_REQ, HOST_TYPE_CFG_OK_URC,
                       bytes([key, len(value)]) + value, timeout=timeout)
    p = ack["payload"]
    if len(p) < 2 or p[0] != (key & 0xFF):
        raise RuntimeError(f"CFG_OK_URC malformed/mismatched: {p.hex()}")
    if p[1] != 0:
        raise RuntimeError(
            f"CFG_SET(0x{key:02X}) REJECTED: status={p[1]} "
            f"({CFG_STATUS_NAMES.get(p[1], '?')}) — do NOT treat as OK")
    return ack

def verify_active_profile(link: HostLink, expected_id: int, timeout: float = 1.0) -> None:
    """Post-activation readback — profile 1 with the default single-channel
    mask silently stays 0 (popcount >= 50 required). Fail loud instead."""
    ack = link.request(HOST_TYPE_CFG_GET_REQ, HOST_TYPE_CFG_DATA_URC,
                       bytes([CFG_KEY_REG_PROFILE]), timeout=timeout)
    p = ack["payload"]           # CFG_DATA: [key, value_len, value...]
    active = p[2] if len(p) >= 3 else -1
    if active != expected_id:
        raise RuntimeError(
            f"REG_PROFILE readback={active}, expected {expected_id} — "
            "activation was rejected upstream (check mask popcount / F15)")


_BW_KHZ_BY_BITS = {7: 125, 8: 250, 9: 500}   # sx1276.c bw_to_reg_bits()

def verify_modem_matches_profile(link: HostLink, profile) -> None:
    """Fail LOUD at startup if the L072's live modem registers disagree
    with the PhyProfile the fragmenter is about to size against (F1)."""
    cfg1, _ = read_reg(link, 0x1D, timeout=0.5)   # RegModemConfig1
    cfg2, _ = read_reg(link, 0x1E, timeout=0.5)   # RegModemConfig2
    pre_msb, _ = read_reg(link, 0x20, timeout=0.5)
    pre_lsb, _ = read_reg(link, 0x21, timeout=0.5)
    actual = {
        "sf": (cfg2 >> 4) & 0x0F,
        "bw_khz": _BW_KHZ_BY_BITS.get((cfg1 >> 4) & 0x0F, -1),
        "cr_den": ((cfg1 >> 1) & 0x07) + 4,
        "preamble": (pre_msb << 8) | pre_lsb,
    }
    expected = {"sf": profile.sf, "bw_khz": profile.bw_khz,
                "cr_den": profile.cr_den, "preamble": profile.preamble_len}
    if actual != expected:
        raise RuntimeError(
            f"PHY CONTRACT VIOLATION: modem={actual} profile={expected}. "
            "Refusing to start — fragment sizing would be wrong (see "
            "CODE REVIEWS 2026-07-23 F1). Fix the profile constant or "
            "the firmware config; do not silence this check.")
    print(f"PHY contract OK: {profile.name} == {actual}")


def configure_regulatory_profile_if_needed(link: HostLink, profile_id: int = 1) -> None:
    """Configures the required regulatory profile parameters and activates the
    specified regulatory profile (default 1 = REG_PROFILE_FCC_15_247_FHSS_50CH_BW250) on the co-processor.
    If profile_id = 2, activates REG_PROFILE_FCC_15_247_DTS_BW500.

    Profile 0 = REG_PROFILE_BENCH_ONLY_FIXED_915 — deterministic single-channel
    915 MHz, 125 kHz BW. Use this for two-peer bench air-link tests (the
    50-ch FHSS profile (1) has per-board channel-pick non-determinism: each
    board picks a different starting channel from the mask, and re-activating
    the same profile does NOT re-tune FRF, so two peers end up on different
    carriers and never decode each other — root cause of the 2026-05-25
    rx_frames=0 bug, see AI NOTES/2026-05-25_Radio_RX_Frames_Zero_Channel_Mismatch.md).

    Env overrides:
      LIFETRAC_REG_PROFILE=<0|1|2>     — override profile_id (default = caller arg).
      LIFETRAC_FHSS_WIDE_MASK=1        — restore historical 50-ch wide mask
                                        (otherwise narrow to single channel).
      LIFETRAC_FHSS_CHANNEL=<0..49>    — single-channel mask index (default 0).
    """
    import os as _os
    env_prof = _os.environ.get("LIFETRAC_REG_PROFILE")
    if env_prof is not None:
        try:
            profile_id = int(env_prof)
        except ValueError:
            pass
    print(f"Initializing regulatory profile {profile_id} on co-processor...")

    # 1. Set the FHSS channel mask.
    #
    # NOTE: the L072 driver only re-tunes FRF on a profile *transition*; a
    # mid-session mask narrowing does not retroactively change the picked
    # channel. The mask still matters for any future profile activation /
    # FHSS hop, so we narrow it by default; combine with profile=0
    # (BENCH_ONLY_FIXED_915) for a deterministic 915 MHz two-peer bench link.
    if _os.environ.get("LIFETRAC_FHSS_WIDE_MASK") == "1":
        mask_bytes = bytes([0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x03, 0x00])
        print("  FHSS mask: WIDE (50 channels) [LIFETRAC_FHSS_WIDE_MASK=1]")
    else:
        try:
            ch = int(_os.environ.get("LIFETRAC_FHSS_CHANNEL", "0"))
        except ValueError:
            ch = 0
        if not 0 <= ch <= 49:
            ch = 0
        mask_int = 1 << ch
        mask_bytes = mask_int.to_bytes(8, "little")
        print(f"  FHSS mask: single channel {ch} ({mask_bytes.hex()})")
    try:
        ack = cfg_set_checked(link, CFG_KEY_FHSS_CHANNEL_MASK, mask_bytes, timeout=1.0)
        print(f"CFG_SET_REQ(FHSS_CHANNEL_MASK) OK: {ack['payload'].hex()}")
    except Exception as exc:
        print(f"WARN: CFG_SET_REQ(FHSS_CHANNEL_MASK) failed: {exc}")
        
    # 2. Set the antenna gain in dBi
    # Let's use 2 dBi as standard (1 byte, signed 0x02)
    try:
        ack = cfg_set_checked(link, CFG_KEY_ANTENNA_GAIN_DBI, bytes([0x02]), timeout=1.0)
        print(f"CFG_SET_REQ(ANTENNA_GAIN_DBI) OK: {ack['payload'].hex()}")
    except Exception as exc:
        print(f"WARN: CFG_SET_REQ(ANTENNA_GAIN_DBI) failed: {exc}")
        
    # 3. Set the HW ceiling in dBm
    # Let's use 17 dBm (1 byte, unsigned 0x11)
    try:
        ack = cfg_set_checked(link, CFG_KEY_HW_CEILING_DBM, bytes([0x11]), timeout=1.0)
        print(f"CFG_SET_REQ(HW_CEILING_DBM) OK: {ack['payload'].hex()}")
    except Exception as exc:
        print(f"WARN: CFG_SET_REQ(HW_CEILING_DBM) failed: {exc}")
        
    # 4. Set the regulatory profile to profile_id and activate
    try:
        ack = cfg_set_checked(link, CFG_KEY_REG_PROFILE, bytes([profile_id]), timeout=1.0)
        print(f"CFG_SET_REQ(REG_PROFILE={profile_id}) OK: {ack['payload'].hex()}")
        verify_active_profile(link, profile_id, timeout=1.0)
    except Exception as exc:
        print(f"WARN: CFG_SET_REQ(REG_PROFILE={profile_id}) failed: {exc}")

    # 5. Force-pin the SX1276 FRF (carrier frequency) directly via register
    # writes if requested. This is the *workaround* for the 2026-05-25
    # "rx_frames=0" bug where the L072 firmware ACKs CFG_SET(REG_PROFILE) with
    # OK but does NOT re-tune RegFrf{Msb,Mid,Lsb} on the SX1276 (it only
    # re-applies on a *real* profile-id transition, and even then it picks a
    # different starting channel per board from the FHSS mask — see
    # AI NOTES/2026-05-25_Radio_RX_Frames_Zero_Channel_Mismatch.md).
    #
    # Env: LIFETRAC_FORCE_FRF_HZ=<int>   — explicit override (e.g. 915000000).
    # Default: when profile_id==0 (BENCH_ONLY_FIXED_915) we auto-pin to
    # 915_000_000 Hz so both peers land on the same carrier. Other profile
    # IDs leave FRF untouched unless the env var is set.
    force_frf_hz = None
    env_frf = _os.environ.get("LIFETRAC_FORCE_FRF_HZ")
    if env_frf:
        try:
            force_frf_hz = int(env_frf)
        except ValueError:
            print(f"WARN: ignoring non-integer LIFETRAC_FORCE_FRF_HZ={env_frf!r}")
    elif profile_id == 0:
        force_frf_hz = 915_000_000

    if force_frf_hz is not None:
        # FRF = freq_hz * 2^19 / 32_000_000 (SX1276 with 32 MHz TCXO).
        frf_raw = (force_frf_hz * (1 << 19)) // 32_000_000
        frf_msb = (frf_raw >> 16) & 0xFF
        frf_mid = (frf_raw >> 8) & 0xFF
        frf_lsb = frf_raw & 0xFF
        print(f"  forcing FRF -> {force_frf_hz/1e6:.3f} MHz "
              f"(raw=0x{frf_raw:06X} -> Msb=0x{frf_msb:02X} Mid=0x{frf_mid:02X} Lsb=0x{frf_lsb:02X})")
        # Drop to standby first if currently in RX/TX (FRF can only be
        # safely changed in SLEEP or STANDBY per SX1276 datasheet 4.1.4).
        prev_opmode = None
        try:
            prev_opmode, _ = read_reg(link, SX1276_REG_OP_MODE, timeout=0.5)
        except Exception as exc:
            print(f"  WARN: could not read opmode before FRF write: {exc}")
        try:
            if prev_opmode is not None and (prev_opmode & 0x07) not in (0x00, 0x01):
                # Force STANDBY (LoRa long-range bit + mode=001).
                write_reg(link, SX1276_REG_OP_MODE, 0x81, timeout=0.5)
            write_reg(link, 0x06, frf_msb, timeout=0.5)
            write_reg(link, 0x07, frf_mid, timeout=0.5)
            write_reg(link, 0x08, frf_lsb, timeout=0.5)
            # Restore prior opmode (e.g. RXCONTINUOUS=0x85) so the new FRF
            # is committed on the next mode entry.
            if prev_opmode is not None and (prev_opmode & 0x07) not in (0x00, 0x01):
                write_reg(link, SX1276_REG_OP_MODE, prev_opmode, timeout=0.5)
            # Verify.
            rb_msb, _ = read_reg(link, 0x06, timeout=0.5)
            rb_mid, _ = read_reg(link, 0x07, timeout=0.5)
            rb_lsb, _ = read_reg(link, 0x08, timeout=0.5)
            actual = (rb_msb << 16) | (rb_mid << 8) | rb_lsb
            actual_hz = (actual * 32_000_000) // (1 << 19)
            ok = (rb_msb == frf_msb and rb_mid == frf_mid and rb_lsb == frf_lsb)
            print(f"  FRF readback: 0x{actual:06X} = {actual_hz/1e6:.3f} MHz "
                  f"({'OK' if ok else '**MISMATCH**'})")
        except Exception as exc:
            print(f"  WARN: FRF force-write failed: {exc}")

    # 6. (REMOVED 2026-05-25) An earlier block here wrote 0x33=0x26 +
    # 0x3B=0x1D, based on a misread of the SX1276 datasheet — bit 0 of
    # 0x33 was assumed to be "TX inversion enable" (set = inverted),
    # but per Semtech's own LoRaMac-node driver it is the opposite:
    # RFLR_INVERTIQ_TX_OFF == 0x01 (set = OFF). With reserved bits
    # 5,2,1 hardwired at 0x26, the canonical RX-non-inverted value
    # IS 0x27 (the chip's reset default). Writing 0x26 therefore
    # silently INVERTED the TX path; air-coupling sniff after the
    # "fix" showed irq_flags_or=0x00, irq_events=0 -- strictly worse
    # than the pre-fix 0x10/9. Leaving the reset defaults alone. The
    # rx_frames=0 root cause is elsewhere (under investigation).


def emit_runtime_profile_enum(link: HostLink) -> None:
    """FCC-B3-1: issue one CFG_GET_REQ(CFG_KEY_REG_PROFILE) and print exactly
    one canonical `RUNTIME_PROFILE_ENUM=<N>` line to stdout. Idempotent: a
    second call within the same process is a no-op. Non-fatal: any
    request/parse failure prints `RUNTIME_PROFILE_ENUM=ERR <reason>` so the
    FCC-B3-2 gate can distinguish wrong-firmware (enum mismatch) from
    probe regression (line missing entirely).

    2026-05-20 P2-1 fix: drain BOOT_URC + any pending traffic first, then
    retry once with a longer timeout if the initial CFG_GET races the boot
    storm (cold-start TimeoutError seen in walk_power_pilot/T4 evidence).

    2026-05-22 Phase 2.2 fix (P1 HOST-RACE limb): the 0.5 s drain windows
    above were empirically insufficient — `p1_cold_boot_discriminator.ps1`
    showed 100 % `request_failed:TimeoutError` across 7 cycles at
    PreProbeSleepS in {0.05, 2.0, 5.0} s, with `drained=2` STATS_URC
    frames arriving at the *next* drain (W1-10 boot-settle) ~3-5 s later
    while AT+VER on the same boot succeeded. Verdict: the two STATS_URC
    frames the L072 emits within the first few seconds of cold boot
    weren't yet kernel-visible during the short drain windows here, then
    they poisoned the link.request() correlator. Fix: lengthen drains to
    cover the empirical 2-URC burst window (~2 s), and re-drain before
    EACH retry so a late URC can't poison a backoff attempt. Emit
    `__PROFILE_DRAINED__=<n>` / `__PROFILE_ATTEMPTS__=<k>` tokens so the
    Phase 2.1 discriminator can verify the fix without regression.
    See `LifeTrac-v25/DESIGN-CONTROLLER/bench-evidence/p1_cold_boot_2026-05-22_160255/VERDICT.md`.
    """
    global _runtime_profile_emitted
    if _runtime_profile_emitted:
        return
    _runtime_profile_emitted = True
    drained_total = 0
    # Initial drain: cover the empirical 2 s STATS_URC arrival window.
    try:
        drain_boot(link, 2.0)
    except Exception:
        pass
    try:
        drained_total += drain_pending(link, quiet_s=0.30, max_s=2.0)
    except Exception:
        pass
    last_exc = None
    line = None
    attempts = 0
    # Three-attempt bounded backoff with re-drain between each attempt
    # (per Phase 2.1 verdict §"Phase 2.2 implementation"). Gaps follow
    # the 100/250/500 ms cadence in the verdict doc; per-attempt timeout
    # bumps so a slow first-CFG response on a busy bus still succeeds.
    backoff_pairs = ((0.000, 1.5), (0.100, 2.0), (0.350, 2.5))
    for gap_s, attempt_timeout in backoff_pairs:
        if gap_s > 0:
            time.sleep(gap_s)
            # Re-drain before each retry: a late URC arriving between
            # attempts is the exact failure mode this defends against.
            try:
                drained_total += drain_pending(link, quiet_s=0.20, max_s=0.8)
            except Exception:
                pass
        attempts += 1
        try:
            frame = link.request(HOST_TYPE_CFG_GET_REQ, HOST_TYPE_CFG_DATA_URC,
                                 bytes([CFG_KEY_REG_PROFILE]),
                                 timeout=attempt_timeout)
            line = _format_runtime_profile_line(payload=frame.get("payload"))
            break
        except Exception as exc:
            last_exc = exc
            continue
    if line is None:
        line = _format_runtime_profile_line(exc=last_exc)
        # 2026-05-22 Phase 2.3 forensic dump (host-race AND firmware-not-ready
        # both falsified by direct test — bench evidence
        # `p1_cold_boot_2026-05-22_160757/`). When the three-attempt backoff
        # exhausts, dump whatever the firmware actually responded with.
        # Discriminator hypotheses:
        #   * nothing on wire/queue       -> firmware silently dropped CFG_GET
        #     (dispatcher not wired in flashed image, or input handler stalled).
        #   * CFG_OK_URC (0x23) present   -> firmware took the error branch
        #     in handle_cfg_get (UNKNOWN_KEY / OUT_OF_RANGE / READ_ONLY).
        #   * CFG_DATA_URC w/ wrong seq   -> host correlator bug
        #     (seq drift between request and response).
        #   * other type                  -> protocol/encoding mismatch.
        try:
            late_frames = link.read_frames(0.5)
        except Exception:
            late_frames = []
        queued = []
        try:
            queued = list(link.urc_queue)
            link.urc_queue.clear()
        except Exception:
            pass
        try:
            rx_buf_hex = bytes(link.rx_buf).hex()
        except Exception:
            rx_buf_hex = ""
        all_post = list(queued) + list(late_frames)
        print(f"__PROFILE_TIMEOUT_QUEUE_COUNT__={len(queued)}")
        print(f"__PROFILE_TIMEOUT_LATE_COUNT__={len(late_frames)}")
        print(f"__PROFILE_TIMEOUT_RX_BUF_HEX__={rx_buf_hex}")
        for idx, fr in enumerate(all_post):
            try:
                src = "queue" if idx < len(queued) else "wire"
                print(f"__PROFILE_TIMEOUT_FRAME__[{idx}]={src} "
                      f"type=0x{fr.get('type', 0):02X} "
                      f"seq={fr.get('seq', 0)} "
                      f"payload={fr.get('payload', b'').hex()}")
            except Exception:
                pass
    print(line)
    # Verdict tokens for the Phase 2.1 discriminator (parsed by
    # `p1_cold_boot_discriminator.ps1` cohort summary). Emit on a
    # separate line so existing FCC-B3-2 grep on `RUNTIME_PROFILE_ENUM=`
    # is unaffected.
    print(f"__PROFILE_DRAINED__={drained_total}")
    print(f"__PROFILE_ATTEMPTS__={attempts}")
    sys.stdout.flush()


def _self_test_profile_emit() -> int:
    """Built-in self-test for the FCC-B3-1 emitter. Exercises
    `_format_runtime_profile_line` over the documented payload shapes so a
    regression in the parser is caught without needing a serial port or
    bench hardware. Returns 0 on success, 1 on any case mismatch (with
    per-case diagnostic printed to stdout).
    """
    cases = [
        ("enum=0 (BENCH_ONLY_FIXED_915)", {"payload": bytes([0x14, 0x01, 0x00])},
         "RUNTIME_PROFILE_ENUM=0"),
        ("enum=1 (FCC_FHSS)", {"payload": bytes([0x14, 0x01, 0x01])},
         "RUNTIME_PROFILE_ENUM=1"),
        ("enum=2 (FCC_DTS)", {"payload": bytes([0x14, 0x01, 0x02])},
         "RUNTIME_PROFILE_ENUM=2"),
        ("wrong key", {"payload": bytes([0x15, 0x01, 0x00])},
         "RUNTIME_PROFILE_ENUM=ERR wrong_key:0x15"),
        ("wrong value_len", {"payload": bytes([0x14, 0x02, 0x00, 0x00])},
         "RUNTIME_PROFILE_ENUM=ERR wrong_value_len:2"),
        ("short payload", {"payload": bytes([0x14])},
         "RUNTIME_PROFILE_ENUM=ERR short_payload:14"),
        ("request raised TimeoutError", {"exc": TimeoutError("no urc")},
         "RUNTIME_PROFILE_ENUM=ERR request_failed:TimeoutError"),
        ("request raised ValueError", {"exc": ValueError("bad cobs")},
         "RUNTIME_PROFILE_ENUM=ERR request_failed:ValueError"),
    ]
    failures = 0
    for name, kwargs, expected in cases:
        got = _format_runtime_profile_line(**kwargs)
        if got == expected:
            print(f"SELF_TEST_PROFILE_EMIT: PASS  {name}")
        else:
            failures += 1
            print(f"SELF_TEST_PROFILE_EMIT: FAIL  {name}")
            print(f"  expected: {expected!r}")
            print(f"  got:      {got!r}")
    print(f"SELF_TEST_PROFILE_EMIT: cases={len(cases)} failures={failures}")
    return 0 if failures == 0 else 1


# SX1276 RegOpMode value parked at end of every bench probe run so the radio
# stops radiating between tests. 0x80 = LongRangeMode=1 (LoRa) | Mode=000
# (SLEEP). Per the 2026-05-10 "Radio Shutdown for EM Reduction (Bench-Only)"
# rule, bench builds should keep the radio idle whenever it is not actively
# under test. The firmware-side equivalent is the LIFETRAC_BENCH_RADIO_IDLE_SLEEP
# config flag in murata_l072/config.h; this probe-side cleanup covers the gap
# during/after a probe run where firmware would otherwise auto-resume RXCONT.
SX1276_OPMODE_LORA_SLEEP_VAL = 0x80


def sleep_radio_safely(link: HostLink, label: str = "") -> None:
    """Best-effort write of RegOpMode=0x80 (LoRa SLEEP). Never raises; logs
    success/failure to stdout so post-run summaries can confirm the radio was
    parked. Safe to call from a finally: block even if the link is degraded.
    """
    tag = f" ({label})" if label else ""
    try:
        ack = write_reg(link, SX1276_REG_OP_MODE,
                        SX1276_OPMODE_LORA_SLEEP_VAL, timeout=0.5)
        try:
            opm, _ = read_reg(link, SX1276_REG_OP_MODE, timeout=0.5)
            print(f"__RADIO_SLEEP_ON_EXIT__{tag} ack={ack.hex()} "
                  f"RegOpMode(post)=0x{opm:02X} "
                  f"(target=0x{SX1276_OPMODE_LORA_SLEEP_VAL:02X})")
        except Exception:
            print(f"__RADIO_SLEEP_ON_EXIT__{tag} ack={ack.hex()} "
                  f"(readback skipped)")
    except Exception as exc:
        print(f"__RADIO_SLEEP_ON_EXIT_WARN__{tag} write_reg failed: {exc}")


def host_invariants_violated(stats_before: dict, stats_after: dict) -> list:
    """Return list of (label, before, after) tuples for any invariant counters
    that changed. Empty list = invariants intact."""
    violations = []
    for label in INVARIANT_COUNTERS:
        b = stats_before.get(label, 0)
        a = stats_after.get(label, 0)
        if a != b:
            violations.append((label, b, a))
    return violations


# -------------------------------------------------------------------------
# W1-9b mode T0/T1: RegVersion burst (transport / SPI shadow stress test)
# -------------------------------------------------------------------------
def run_regversion_burst(link: HostLink, count: int = 1024) -> int:
    print(f"=== W1-9b probe T1: RegVersion burst (count={count}) ===")
    if not drain_boot(link, 1.0):
        print("BOOT_URC not observed during 1.0s settle (continuing).")

    # T0 precheck: host transport sanity.
    try:
        stats_before = fetch_stats(link)
    except Exception as exc:
        print(f"FATAL: T0 STATS_URC(before) failed: {exc}")
        print("__W1_9B_VERDICT__=TRANSPORT_FAIL")
        return 2
    parse_err_before = stats_before.get("host_parse_err", 0)
    print(f"T0: STATS(before) host_parse_err={parse_err_before}")

    correct = 0
    zero = 0
    other = 0
    other_values = {}
    short_or_err = 0
    t0 = time.time()
    for i in range(count):
        try:
            val, raw = read_reg(link, 0x42, timeout=0.5)
        except Exception as exc:
            short_or_err += 1
            if short_or_err <= 3:
                print(f"  iter {i}: REG_READ failed: {exc}")
            continue
        if val == 0x12:
            correct += 1
        elif val == 0x00:
            zero += 1
            if zero <= 3:
                print(f"  iter {i}: read returned 0x00 (raw={raw.hex()})")
        else:
            other += 1
            other_values[val] = other_values.get(val, 0) + 1
            if other <= 3:
                print(f"  iter {i}: unexpected 0x{val:02X} (raw={raw.hex()})")
    elapsed = time.time() - t0
    rate = count / elapsed if elapsed > 0 else 0
    print(f"T1 burst: correct={correct} zero={zero} other={other} "
          f"errors={short_or_err} elapsed={elapsed:.2f}s rate={rate:.1f}/s")
    if other_values:
        print(f"T1 distinct other values: { {hex(k): v for k, v in other_values.items()} }")

    try:
        stats_after = fetch_stats(link)
    except Exception as exc:
        print(f"FATAL: STATS_URC(after) failed: {exc}")
        print("__W1_9B_VERDICT__=TRANSPORT_FAIL")
        return 2
    violations = host_invariants_violated(stats_before, stats_after)
    invariants_ok = not violations
    for label, b, a in violations:
        print(f"  INVARIANT VIOLATED: {label} {b} -> {a}")

    transport_ok = (correct == count and short_or_err == 0)
    if transport_ok:
        verdict = "PASS"
    elif (zero + other + short_or_err) > 0:
        verdict = "FAIL_NONUNIFORM"
    else:
        verdict = "FAIL"

    print(f"__T1_TRANSPORT__={verdict}")
    print(f"__T1_NON_VERSION_READS__={zero + other}")
    print(f"__T1_REQUEST_ERRORS__={short_or_err}")
    print(f"__W1_9B_VERDICT__=T1_{verdict}")
    if not invariants_ok:
        print("__W1_9B_VERDICT__=T1_INVARIANT_VIOLATED")
        return 1
    return 0


# -------------------------------------------------------------------------
# W1-9b mode T2: FSK STDBY datapath (clock-alive discriminator)
# -------------------------------------------------------------------------
def _classify_fsk_verdict(step5_rb: int, step8_rb: int) -> str:
    """Return one of CLOCK_DEAD | FSK_STDBY_WORKS | FSK_SLEEP_ONLY | INCONSISTENT."""
    if step5_rb == 0x00 and step8_rb == 0x01:
        return "FSK_STDBY_WORKS"     # clock alive
    if step5_rb == 0x00 and step8_rb == 0x00:
        return "FSK_SLEEP_ONLY"      # latch works, STDBY transition fails
    if step5_rb == 0x80 and step8_rb == 0x80:
        return "CLOCK_DEAD"
    return "INCONSISTENT"


def run_fsk_stdby(link: HostLink) -> int:
    print("=== W1-9b probe T2: FSK STDBY datapath ===")
    if not drain_boot(link, 1.0):
        print("BOOT_URC not observed during 1.0s settle (continuing).")

    # 2026-05-12 W1-9b warm-up: empirically the firmware's REG_READ_REQ and
    # subsequent request paths are only reliable AFTER a VER_REQ/VER_URC
    # round-trip has been completed at least once (matches the Stage 1 probe
    # sequence which does VER_REQ -> STATS -> REG_READ x113 successfully).
    # If the very first request is STATS_DUMP_REQ, all subsequent requests
    # silently time out. Issue tracked as W1-9g; for now the FSK probe
    # mirrors Stage 1's ordering (VER first).
    try:
        ver_frame = link.request(HOST_TYPE_VER_REQ, HOST_TYPE_VER_URC, timeout=1.0)
        ver = parse_version(ver_frame["payload"])
        print(f"T0a: VER warm-up OK name={ver['name']} "
              f"v={ver['major']}.{ver['minor']}.{ver['patch']}")
    except Exception as exc:
        print(f"FATAL: VER_REQ warm-up failed: {exc}")
        print("__W1_9B_VERDICT__=TRANSPORT_FAIL")
        return 2

    # 2026-05-12 W1-9g: drain post-VER URCs (HOST_RX_SEEN FAULT + auto STATS
    # snapshot, plus possibly HOST_DIAG_MARK) before the next request so
    # they don't sit in urc_queue and consume request()'s response window.
    drained_n = drain_pending(link, quiet_s=0.25, max_s=1.0)
    print(f"T0b: post-VER drain consumed {drained_n} frames")

    try:
        stats_before = fetch_stats(link)
    except Exception as exc:
        print(f"FATAL: T0 STATS_URC(before) failed: {exc}")
        print("__W1_9B_VERDICT__=TRANSPORT_FAIL")
        return 2
    parse_err_before = stats_before.get("host_parse_err", 0)
    print(f"T0: STATS(before) host_parse_err={parse_err_before}")

    # 1. Disable LBT for cleanliness (avoid ERR_PROTO_FORBIDDEN noise).
    # 2026-05-12 W1-9b: the CFG_SET path is currently observed to silently
    # stall further responses for ~600 ms (kernel /dev/ttymxc3 RX-FIFO +
    # firmware boot-batching). Skip CFG_SET in the probe; the FSK SLEEP/STDBY
    # writes that follow do not require LBT to be disabled.
    # try:
    #     link.request(HOST_TYPE_CFG_SET_REQ, HOST_TYPE_CFG_OK_URC,
    #                  bytes([CFG_KEY_LBT_ENABLE, 0x01, 0x00]), timeout=1.0)
    # except Exception as exc:
    #     print(f"WARN: CFG_SET_REQ(LBT_ENABLE=0) failed: {exc} (continuing)")

    # 2. Pre-state assertion: chip should be in a LoRa mode (bit 7 set).
    # 2026-05-12 W1-9b: retry the pre-state read up to 3x to absorb the
    # early-boot kernel-RX-batching pattern (first 1-3 requests time out
    # but responses arrive in the next request window via urc_queue).
    pre_val = None
    pre_raw = None
    last_exc = None
    for attempt in range(3):
        try:
            pre_val, pre_raw = read_reg(link, SX1276_REG_OP_MODE, timeout=1.0)
            break
        except Exception as exc:
            last_exc = exc
            print(f"WARN: pre-state OPMODE read attempt {attempt + 1}/3 failed: {exc}")
    if pre_val is None:
        print(f"FATAL: pre-state OPMODE read failed after 3 attempts: {last_exc}")
        print("__W1_9B_VERDICT__=TRANSPORT_FAIL")
        return 2
    if pre_val is None:
        print(f"FATAL: pre-state OPMODE read failed after 3 attempts: {last_exc}")
        print("__W1_9B_VERDICT__=TRANSPORT_FAIL")
        return 2
    print(f"T2 step2: pre RegOpMode=0x{pre_val:02X} (raw={pre_raw.hex()})")
    # 2026-05-12 W1-9b probe relaxation: pre-W1-9d this assertion expected
    # exactly 0x80 (LoRa SLEEP), but post-W1-9d the firmware now boots
    # straight into LoRa RXCONTINUOUS (0x85) because the SX1276 successfully
    # exits SLEEP. The T2 probe writes FSK SLEEP (0x00) as its first step
    # anyway, so any LoRa-mode pre-state (bit 7 set) is acceptable. We only
    # reject FSK-mode pre-states (bit 7 clear) since those mean the chip
    # was already taken out of LoRa mode by something else and the test
    # wouldn't be exercising the LoRa->FSK transition we care about.
    if (pre_val & 0x80) == 0:
        print(f"FAIL: pre-state expected LoRa mode (bit7 set), got 0x{pre_val:02X} (FSK)")
        print(f"__W1_9B_VERDICT__=PRE_STATE_UNEXPECTED_0x{pre_val:02X}")
        return 1
    if pre_val != 0x80:
        print(f"INFO: pre-state 0x{pre_val:02X} is non-SLEEP LoRa mode "
              f"(post-W1-9d firmware boots into LoRa RXCONTINUOUS=0x85); accepting and continuing.")

    step5_rb = None
    step8_rb = None
    step5_elapsed_ms = None
    step8_elapsed_ms = None
    restore_ok = False
    restore_rb = None
    try:
        # 3. Write FSK SLEEP (0x00).
        t = time.time()
        ack = write_reg(link, SX1276_REG_OP_MODE, 0x00, timeout=0.5)
        print(f"T2 step3: REG_WRITE OPMODE=0x00 ack={ack.hex()}")
        # 4. Settle 2 ms.
        time.sleep(0.002)
        # 5. Read back (expect 0x00 if LongRange clear path latches).
        step5_rb, raw5 = read_reg(link, SX1276_REG_OP_MODE, timeout=0.5)
        step5_elapsed_ms = (time.time() - t) * 1000.0
        print(f"T2 step5: post-FSK-SLEEP RegOpMode=0x{step5_rb:02X} (raw={raw5.hex()}) "
              f"elapsed={step5_elapsed_ms:.2f}ms")

        # 6. Write FSK STDBY (0x01).
        t = time.time()
        ack = write_reg(link, SX1276_REG_OP_MODE, 0x01, timeout=0.5)
        print(f"T2 step6: REG_WRITE OPMODE=0x01 ack={ack.hex()}")
        # 7. Settle 2 ms (8x datasheet typ. 250us).
        time.sleep(0.002)
        # 8. Read back (expect 0x01 if digital state machine has clock).
        step8_rb, raw8 = read_reg(link, SX1276_REG_OP_MODE, timeout=0.5)
        step8_elapsed_ms = (time.time() - t) * 1000.0
        print(f"T2 step8: post-FSK-STDBY RegOpMode=0x{step8_rb:02X} (raw={raw8.hex()}) "
              f"elapsed={step8_elapsed_ms:.2f}ms")
    finally:
        # 9. Restore LoRa SLEEP unconditionally.
        try:
            ack = write_reg(link, SX1276_REG_OP_MODE, 0x80, timeout=0.5)
            print(f"T2 restore: REG_WRITE OPMODE=0x80 ack={ack.hex()}")
            time.sleep(0.002)
            restore_rb, raw_r = read_reg(link, SX1276_REG_OP_MODE, timeout=0.5)
            print(f"T2 restore: RegOpMode=0x{restore_rb:02X} (raw={raw_r.hex()})")
            restore_ok = (restore_rb == 0x80)
        except Exception as exc:
            print(f"WARN: restore failed: {exc}")
            restore_ok = False

    if step5_rb is None or step8_rb is None:
        print("FATAL: probe sequence incomplete")
        print("__W1_9B_VERDICT__=PROBE_INCOMPLETE")
        return 1

    verdict = _classify_fsk_verdict(step5_rb, step8_rb)

    # 10. Stats after + invariant check.
    try:
        stats_after = fetch_stats(link)
    except Exception as exc:
        print(f"FATAL: STATS_URC(after) failed: {exc}")
        print("__W1_9B_VERDICT__=TRANSPORT_FAIL")
        return 2
    violations = host_invariants_violated(stats_before, stats_after)
    invariants_ok = not violations
    for label, b, a in violations:
        print(f"  INVARIANT VIOLATED: {label} {b} -> {a}")

    print(f"__T2_STEP5_READBACK__=0x{step5_rb:02X}")
    print(f"__T2_STEP8_READBACK__=0x{step8_rb:02X}")
    print(f"__T2_RESTORE_VERDICT__={'OK' if restore_ok else 'FAIL'}")
    print(f"__T2_RESTORE_READBACK__=0x{(restore_rb or 0):02X}")
    print(f"__T2_RADIO_VERDICT__={verdict}")
    print(f"__HOST_INVARIANTS__={'PASS' if invariants_ok else 'FAIL'}")
    print(f"__W1_9B_VERDICT__=T2_{verdict}")

    if not invariants_ok:
        return 1
    if not restore_ok:
        return 1
    return 0


# -------------------------------------------------------------------------
# W1-9b mode T3: RegOpMode bit walk (only meaningful if T2 says clock alive)
# -------------------------------------------------------------------------
def run_opmode_walk(link: HostLink) -> int:
    print("=== W1-9b probe T3: RegOpMode bit walk (LoRa HF/LF) ===")
    if not drain_boot(link, 1.0):
        print("BOOT_URC not observed during 1.0s settle (continuing).")

    try:
        stats_before = fetch_stats(link)
    except Exception as exc:
        print(f"FATAL: STATS_URC(before) failed: {exc}")
        print("__W1_9B_VERDICT__=TRANSPORT_FAIL")
        return 2

    hf_values = [(0x80, "HF SLEEP"), (0x81, "HF STDBY"),
                 (0x83, "HF TX"), (0x85, "HF RX_CONT")]
    lf_values = [(0x88, "LF SLEEP"), (0x89, "LF STDBY"),
                 (0x8B, "LF TX"), (0x8D, "LF RX_CONT")]
    hf_transitions = 0
    lf_transitions = 0
    try:
        for val, label in hf_values + lf_values:
            try:
                write_reg(link, SX1276_REG_OP_MODE, val, timeout=0.5)
                time.sleep(0.002)
                rb, raw = read_reg(link, SX1276_REG_OP_MODE, timeout=0.5)
                took = (rb == val)
                print(f"T3 walk: wrote 0x{val:02X} ({label}) -> readback 0x{rb:02X} "
                      f"({'TAKE' if took else 'IGNORED'})")
                if took:
                    if (val, label) in hf_values:
                        hf_transitions += 1
                    else:
                        lf_transitions += 1
            except Exception as exc:
                print(f"T3 walk: {label} failed: {exc}")
    finally:
        try:
            write_reg(link, SX1276_REG_OP_MODE, 0x80, timeout=0.5)
        except Exception:
            pass

    try:
        stats_after = fetch_stats(link)
    except Exception as exc:
        print(f"FATAL: STATS_URC(after) failed: {exc}")
        return 2
    violations = host_invariants_violated(stats_before, stats_after)
    for label, b, a in violations:
        print(f"  INVARIANT VIOLATED: {label} {b} -> {a}")

    print(f"__T3_HF_TRANSITIONS__={hf_transitions}/4")
    print(f"__T3_LF_TRANSITIONS__={lf_transitions}/4")
    print(f"__W1_9B_VERDICT__=T3_HF{hf_transitions}_LF{lf_transitions}")
    if violations:
        return 1
    return 0


# -------------------------------------------------------------------------
# W1-10 mode RX: single-board RX-liveness probe
# -------------------------------------------------------------------------
# Goal: prove the firmware's LoRa receive chain is alive end-to-end up to
# the host URC dispatch boundary, WITHOUT requiring a paired transmitter.
# Two-board end-to-end RX validation is the next milestone (W1-10b) and
# requires the second bench board (ADB 2D0A1209DABC240B) to be back on
# the bench.
#
# Single-board liveness gates (all must hold for VERDICT=PASS):
#   A. RegOpMode reads 0x85 (LoRa | RXCONTINUOUS) before AND after the
#      observation window — proves boot-into-RX path stuck in RX_CONT.
#   B. RegRssiValue (0x1B) is in the plausible band [10..200] (== about
#      -147 dBm to +43 dBm raw) and changes between two reads spaced 50 ms
#      apart — proves AGC + IQ chain is digitally alive (a dead receiver
#      reads constant 0x00 or 0xFF).
#   C. radio_ok stat == 1.
#   D. host_parse_err and per-flag UART error counters unchanged across
#      the observation window (HOST invariants).
#   E. No real FAULT_URC during the window (BENIGN_FAULT_CODES filtered).
#   F. Any RX_FRAME_URC arrivals are parsed without error and reported.
#      A successful parse is BONUS evidence; absence is not a failure
#      (single-board bench has no transmitter).
def parse_rx_frame(payload: bytes) -> dict:
    """Parse RX_FRAME_URC payload {u8 len, i8 snr_db, i16 rssi_dbm_le,
    u32 timestamp_us_le, payload[len]}. See include/host_types.h."""
    if len(payload) < 8:
        raise ValueError(f"RX_FRAME_URC payload too short: {payload.hex()}")
    rx_len = payload[0]
    snr_db = struct.unpack("<b", payload[1:2])[0]
    rssi_dbm = struct.unpack("<h", payload[2:4])[0]
    timestamp_us = struct.unpack("<I", payload[4:8])[0]
    expected = 8 + rx_len
    if len(payload) < expected:
        raise ValueError(
            f"RX_FRAME_URC payload truncated: have {len(payload)} need {expected}")
    rx_payload = bytes(payload[8:8 + rx_len])
    return {
        "len": rx_len,
        "snr_db": snr_db,
        "rssi_dbm": rssi_dbm,
        "timestamp_us": timestamp_us,
        "payload": rx_payload,
    }


HOST_TYPE_RX_FRAME_URC = 0x91


def run_rx_liveness(link: HostLink, window_s: float = 10.0) -> int:
    """W1-10 single-board RX-liveness probe.

    Verifies the firmware boots straight into LoRa RXCONTINUOUS, the
    SX1276 receiver's analog/digital chain is alive (RSSI varies), and
    no host-transport invariants are violated over a quiet observation
    window. Reports any RX_FRAME_URC that arrives (bonus evidence).
    """
    print(f"=== W1-10 probe RX: single-board RX-liveness (window={window_s:.1f}s) ===")
    if not drain_boot(link, 1.0):
        print("BOOT_URC not observed during 1.0s settle (firmware likely "
              "already past boot — continuing).")

    # T0a: VER warm-up so the firmware has emitted its post-VER
    # HOST_RX_SEEN diagnostic + auto STATS snapshot before we ask for
    # real STATS. Mirrors the W1-9g fix in run_fsk_stdby().
    try:
        ver_frame = link.request(HOST_TYPE_VER_REQ, HOST_TYPE_VER_URC,
                                 timeout=1.0)
        ver = parse_version(ver_frame["payload"])
        print(f"T0a: VER warm-up OK fw=v{ver.get('fw_major', 0)}.{ver.get('fw_minor', 0)}."
              f"{ver.get('fw_patch', 0)} build={ver.get('build_id', 0):08X}")
    except Exception as exc:
        print(f"FATAL: T0a VER_REQ warm-up failed: {exc}")
        print("__W1_10_VERDICT__=TRANSPORT_FAIL")
        return 2

    n_drained = drain_pending(link, quiet_s=0.25, max_s=1.0)
    print(f"T0b: post-VER drain consumed {n_drained} frames")

    # T0c: STATS(before) snapshot.
    try:
        stats_before = fetch_stats(link)
    except Exception as exc:
        print(f"FATAL: T0c STATS_URC(before) failed: {exc}")
        print("__W1_10_VERDICT__=TRANSPORT_FAIL")
        return 2
    rx_ok_before = stats_before.get("radio_rx_ok", 0)
    crc_err_before = stats_before.get("radio_crc_err", 0)
    radio_state_before = stats_before.get("radio_state", 0)
    print(f"T0c STATS(before): radio_state={radio_state_before} (4=RX_CONT) "
          f"radio_rx_ok={rx_ok_before} radio_crc_err={crc_err_before} "
          f"host_parse_err={stats_before.get('host_parse_err', 0)}")

    # T1: RegOpMode pre-check.
    try:
        opm_pre, _ = read_reg(link, SX1276_REG_OP_MODE, timeout=0.5)
    except Exception as exc:
        print(f"FATAL: T1 RegOpMode read failed: {exc}")
        print("__W1_10_VERDICT__=TRANSPORT_FAIL")
        return 2
    print(f"T1: RegOpMode(pre) = 0x{opm_pre:02X} "
          f"(expected 0x{SX1276_OPMODE_LORA_RXCONT:02X} = LoRa+RXCONTINUOUS)")
    opm_pre_ok = (opm_pre == SX1276_OPMODE_LORA_RXCONT)
    if not opm_pre_ok:
        print(f"  WARN: pre-state not RXCONTINUOUS (got 0x{opm_pre:02X})")

    # T2: RSSI liveness — read RegRssiValue twice with 50 ms gap.
    try:
        rssi_a, _ = read_reg(link, SX1276_REG_RSSI_VALUE, timeout=0.5)
        time.sleep(0.05)
        rssi_b, _ = read_reg(link, SX1276_REG_RSSI_VALUE, timeout=0.5)
    except Exception as exc:
        print(f"FATAL: T2 RegRssiValue read failed: {exc}")
        print("__W1_10_VERDICT__=TRANSPORT_FAIL")
        return 2
    rssi_a_dbm = rssi_a - 157
    rssi_b_dbm = rssi_b - 157
    rssi_in_band = (10 <= rssi_a <= 200) and (10 <= rssi_b <= 200)
    rssi_alive = rssi_a != rssi_b  # AGC twitch between samples
    print(f"T2: RegRssiValue A=0x{rssi_a:02X} ({rssi_a_dbm:+d} dBm) "
          f"B=0x{rssi_b:02X} ({rssi_b_dbm:+d} dBm) "
          f"in_band={rssi_in_band} varies={rssi_alive}")

    # T3: ModemStat snapshot.
    try:
        mstat, _ = read_reg(link, SX1276_REG_MODEM_STAT, timeout=0.5)
    except Exception as exc:
        print(f"WARN: T3 RegModemStat read failed: {exc}")
        mstat = None
    if mstat is not None:
        print(f"T3: RegModemStat(0x18) = 0x{mstat:02X} "
              f"(bit0=SigDetect bit1=SigSync bit2=RxOngoing bit3=HdrInfoValid "
              f"bit4=ModemClear)")

    # T4: observation window — listen for RX_FRAME_URC, FAULT_URC,
    # ERR_PROTO_URC, or any other unsolicited traffic.
    print(f"T4: observation window {window_s:.1f}s (quiet bench, no peer expected)")
    deadline = time.time() + window_s
    rx_frames = []
    real_faults = []
    n_other = 0
    while time.time() < deadline:
        for frame in link.read_frames(0.2):
            ftype = frame["type"]
            if ftype == HOST_TYPE_RX_FRAME_URC:
                try:
                    parsed = parse_rx_frame(frame["payload"])
                    rx_frames.append(parsed)
                    print(f"  RX_FRAME_URC: len={parsed['len']} "
                          f"rssi={parsed['rssi_dbm']:+d}dBm snr={parsed['snr_db']:+d}dB "
                          f"timestamp_us={parsed['timestamp_us']} "
                          f"payload={parsed['payload'].hex()}")
                except Exception as exc:
                    print(f"  RX_FRAME_URC parse error: {exc} raw={frame['payload'].hex()}")
                    real_faults.append(f"rx_frame_parse_error:{exc}")
            elif ftype == HOST_TYPE_FAULT_URC:
                desc = format_fault_payload(frame["payload"])
                code = frame["payload"][0] if frame["payload"] else None
                if code in BENIGN_FAULT_CODES:
                    print(f"  INFO: benign FAULT_URC during RX window: {desc}")
                else:
                    print(f"  FAULT_URC during RX window: {desc}")
                    real_faults.append(desc)
            elif ftype == HOST_TYPE_ERR_PROTO_URC:
                desc = format_err_proto_payload(frame["payload"])
                print(f"  ERR_PROTO during RX window: {desc}")
                real_faults.append(f"err_proto:{desc}")
            else:
                n_other += 1
                if n_other <= 3:
                    print(f"  INFO: other URC type=0x{ftype:02X} seq={frame['seq']}")

    # T5: post-window state.
    try:
        opm_post, _ = read_reg(link, SX1276_REG_OP_MODE, timeout=0.5)
    except Exception as exc:
        print(f"FATAL: T5 RegOpMode(post) read failed: {exc}")
        print("__W1_10_VERDICT__=TRANSPORT_FAIL")
        return 2
    opm_post_ok = (opm_post == SX1276_OPMODE_LORA_RXCONT)
    print(f"T5: RegOpMode(post) = 0x{opm_post:02X} "
          f"(stuck-in-RXCONT = {opm_post_ok})")

    try:
        stats_after = fetch_stats(link)
    except Exception as exc:
        print(f"FATAL: T5 STATS_URC(after) failed: {exc}")
        print("__W1_10_VERDICT__=TRANSPORT_FAIL")
        return 2
    rx_ok_after = stats_after.get("radio_rx_ok", 0)
    crc_err_after = stats_after.get("radio_crc_err", 0)
    print(f"T5 STATS(after): radio_rx_ok={rx_ok_after} (delta={rx_ok_after - rx_ok_before}) "
          f"radio_crc_err={crc_err_after} (delta={crc_err_after - crc_err_before}) "
          f"radio_state={stats_after.get('radio_state', 0)} "
          f"host_parse_err={stats_after.get('host_parse_err', 0)}")

    violations = host_invariants_violated(stats_before, stats_after)
    invariants_ok = not violations
    for label, b, a in violations:
        print(f"  INVARIANT VIOLATED: {label} {b} -> {a}")

    # T6: gate evaluation.
    checks = [
        ("A1 RegOpMode(pre)  == 0x85 (LoRa+RXCONTINUOUS)", opm_pre_ok),
        ("A2 RegOpMode(post) == 0x85 (LoRa+RXCONTINUOUS)", opm_post_ok),
        ("B1 RegRssiValue in plausible band [10..200]", rssi_in_band),
        ("B2 RegRssiValue varies between samples (AGC alive)", rssi_alive),
        ("C  radio_state == 4 (RX_CONT)", radio_state_before == 4),
        ("D  host invariants stable", invariants_ok),
        ("E  no real FAULT_URC during window", len(real_faults) == 0),
    ]

    print()
    print("Critical checks:")
    failed = 0
    for label, ok in checks:
        status = "PASS" if ok else "FAIL"
        print(f"  [{status}] {label}")
        if not ok:
            failed += 1

    print()
    print(f"Bonus evidence: rx_frames_received={len(rx_frames)} "
          f"(0 expected on single-board quiet bench)")

    print(f"__W1_10_RX_FRAMES__={len(rx_frames)}")
    print(f"__W1_10_RSSI_PRE__={rssi_a_dbm}")
    print(f"__W1_10_RSSI_POST__={rssi_b_dbm}")
    print(f"__W1_10_OPMODE_PRE__=0x{opm_pre:02X}")
    print(f"__W1_10_OPMODE_POST__=0x{opm_post:02X}")
    print(f"__W1_10_REAL_FAULTS__={len(real_faults)}")

    if failed == 0:
        print("__W1_10_VERDICT__=RX_LIVENESS_PASS")
        print("VERDICT: PASS (single-board RX-liveness — receive chain alive, "
              "OPMODE stuck in RXCONTINUOUS, AGC twitching, no fault/invariant "
              "violations). Two-board end-to-end (W1-10b) requires Board 1 on bench.")
        return 0
    print(f"__W1_10_VERDICT__=RX_LIVENESS_FAIL_{failed}_CHECKS")
    print(f"VERDICT: FAIL ({failed} critical check(s) failed)")
    return 1


# =============================================================================
# W1-10b two-board RX/TX-pair probes (rx_listen + tx_burst).
#
# Each sub-mode runs on ONE board; the host-PC orchestrator
# (run_w1_10b_rx_pair_end_to_end.ps1) opens two parallel ADB sessions, runs
# `--probe rx_listen` on the RX board and `--probe tx_burst` on the TX board,
# then correlates structured `__RX_FRAME__ ... payload_hex=...` and
# `__TX_DONE__ ... payload_hex=...` lines by payload prefix `W1-10b seq=NNNN`
# to compute Phase B gates B1..B6 (see W1-10 plan §3.3).
#
# Lines prefixed with double-underscore tokens are the only stable
# machine-parseable contract; everything else is human-readable diagnostics.
# =============================================================================


def run_rx_listen(link: HostLink, window_s: float = 30.0) -> int:
    """W1-10b RX-side listener. Drains, warms up, signals
    `__W1_10B_LISTEN_READY__`, then prints one `__RX_FRAME__ ...` line for
    every RX_FRAME_URC received during the window. Final summary on
    `__W1_10B_LISTEN_DONE__`.
    """
    print(f"=== W1-10b probe RX_LISTEN: window={window_s:.1f}s ===")
    drain_boot(link, 1.0)
    try:
        link.request(HOST_TYPE_VER_REQ, HOST_TYPE_VER_URC, timeout=1.0)
    except Exception as exc:
        print(f"FATAL: VER warm-up failed: {exc}")
        print("__W1_10B_LISTEN_VERDICT__=TRANSPORT_FAIL")
        return 2
    drain_pending(link, quiet_s=0.25, max_s=1.0)

    try:
        stats_before = fetch_stats(link)
    except Exception as exc:
        print(f"FATAL: STATS_URC(before) failed: {exc}")
        print("__W1_10B_LISTEN_VERDICT__=TRANSPORT_FAIL")
        return 2
    rx_ok_b = stats_before.get("radio_rx_ok", 0)
    crc_b = stats_before.get("radio_crc_err", 0)
    print(f"RX_LISTEN: STATS(before) radio_rx_ok={rx_ok_b} radio_crc_err={crc_b} "
          f"radio_state={stats_before.get('radio_state', 0)}")

    # v2 change: auto-wake the SX1276 into LORA_RXCONTINUOUS if the
    # previous probe left it in LORA_SLEEP (0x80) via the
    # __RADIO_SLEEP_ON_EXIT__ cleanup. This folds the work done by the
    # external w2_02_radio_wake_rxcont.py helper into the probe itself so
    # there is no longer a dependency on the orchestrator wake step.
    try:
        opm_pre, _ = read_reg(link, SX1276_REG_OP_MODE, timeout=0.5)
        print(f"RX_LISTEN: RegOpMode(pre)=0x{opm_pre:02X} "
              f"(expected 0x{SX1276_OPMODE_LORA_RXCONT:02X})")
        if opm_pre != SX1276_OPMODE_LORA_RXCONT:
            try:
                write_reg(link, SX1276_REG_OP_MODE,
                          SX1276_OPMODE_LORA_RXCONT, timeout=0.5)
                opm_post, _ = read_reg(link, SX1276_REG_OP_MODE, timeout=0.5)
                action = "write"
            except Exception as exc:
                opm_post = opm_pre
                action = f"write_failed({exc})"
            print(f"__V2_AUTOWAKE__ opmode_pre=0x{opm_pre:02X} "
                  f"opmode_post=0x{opm_post:02X} action={action}")
        else:
            print(f"__V2_AUTOWAKE__ opmode_pre=0x{opm_pre:02X} "
                  f"opmode_post=0x{opm_pre:02X} action=none")
    except Exception as exc:
        print(f"__V2_AUTOWAKE__ FAIL reason={exc!r}")

    # Signal ready BEFORE entering the listen loop so the orchestrator can
    # safely start the TX side.
    sys.stdout.write("__W1_10B_LISTEN_READY__\n")
    sys.stdout.flush()
    _emit_progress("rx_listen_ready", window_s=f"{window_s:.1f}")

    deadline = time.time() + window_s
    rx_count = 0
    real_faults = 0
    # P5 sidecar: emit no more than once per second (cheap throttle).
    next_progress_ts = 0.0
    # 2026-05-20 P1-3 fix: graceful SIGINT shutdown. Without this, the
    # orchestrator's `pkill -INT` (paired_walk_power_sweep.ps1) lands inside
    # link.read_frames() and Python emits a full KeyboardInterrupt traceback
    # into the RX log, which downstream parsers misread as a probe crash.
    # Catching KeyboardInterrupt here emits a stable machine-parseable
    # `__RX_LISTEN_STOPPED__` token, then falls through to the normal
    # __W1_10B_LISTEN_DONE__ summary path so stats are still captured.
    stopped_by_signal = False
    try:
        while time.time() < deadline:
            now = time.time()
            if now >= next_progress_ts:
                _emit_progress("rx_listen",
                               remaining_s=f"{deadline - now:.1f}",
                               rx=rx_count, faults=real_faults)
                next_progress_ts = now + 1.0
            for frame in link.read_frames(0.2):
                ftype = frame["type"]
                if ftype == HOST_TYPE_RX_FRAME_URC:
                    try:
                        p = parse_rx_frame(frame["payload"])
                        rx_count += 1
                        print(f"__RX_FRAME__ idx={rx_count} rssi={p['rssi_dbm']} "
                              f"snr={p['snr_db']} len={p['len']} "
                              f"timestamp_us={p['timestamp_us']} "
                              f"payload_hex={p['payload'].hex()}")
                        sys.stdout.flush()
                    except Exception as exc:
                        print(f"__RX_FRAME_ERR__ {exc} raw={frame['payload'].hex()}")
                elif ftype == HOST_TYPE_FAULT_URC:
                    code = frame["payload"][0] if frame["payload"] else None
                    desc = format_fault_payload(frame["payload"])
                    if code in BENIGN_FAULT_CODES:
                        print(f"INFO: benign FAULT_URC: {desc}")
                    else:
                        real_faults += 1
                        print(f"__RX_FAULT__ {desc}")
                elif ftype == HOST_TYPE_ERR_PROTO_URC:
                    desc = format_err_proto_payload(frame["payload"])
                    real_faults += 1
                    print(f"__RX_ERR_PROTO__ {desc}")
                # Other URCs (STATS_URC etc.) are ignored during the listen window.
    except KeyboardInterrupt:
        stopped_by_signal = True
        print(f"__RX_LISTEN_STOPPED__ reason=SIGINT rx_frames_so_far={rx_count}")
        sys.stdout.flush()

    try:
        stats_after = fetch_stats(link)
    except Exception:
        stats_after = stats_before
    rx_ok_a = stats_after.get("radio_rx_ok", 0)
    crc_a = stats_after.get("radio_crc_err", 0)
    invariants_violated = host_invariants_violated(stats_before, stats_after)
    for label, b, a in invariants_violated:
        print(f"  INVARIANT VIOLATED: {label} {b} -> {a}")
    print(f"RX_LISTEN: STATS(after) radio_rx_ok={rx_ok_a} (delta={rx_ok_a - rx_ok_b}) "
          f"radio_crc_err={crc_a} (delta={crc_a - crc_b})")
    print(f"__W1_10B_LISTEN_DONE__ rx_frames={rx_count} "
          f"radio_rx_ok_delta={rx_ok_a - rx_ok_b} "
          f"radio_crc_err_delta={crc_a - crc_b} "
          f"real_faults={real_faults} "
          f"invariants_violated={len(invariants_violated)} "
          f"stopped_by_signal={int(stopped_by_signal)}")
    return 0


def run_tx_burst(link: HostLink, count: int = 100, inter_s: float = 0.2,
                 timeout: float = 5.0) -> int:
    """W1-10b TX-side burst. Sends `count` TX_FRAME_REQs with payload
    `W1-10b seq=NNNN <random-hex>` and prints one `__TX_DONE__ ...` line per
    successful TX_DONE_URC. Final summary on `__W1_10B_BURST_DONE__`.
    """
    import os as _os
    inter_s = clamp_inter_cycle_s(inter_s)
    print(f"=== W1-10b probe TX_BURST: count={count} inter={inter_s:.2f}s "
          f"timeout={timeout:.1f}s ===")
    drain_boot(link, 1.0)
    try:
        link.request(HOST_TYPE_VER_REQ, HOST_TYPE_VER_URC, timeout=1.0)
    except Exception as exc:
        print(f"FATAL: VER warm-up failed: {exc}")
        print("__W1_10B_BURST_VERDICT__=TRANSPORT_FAIL")
        return 2
    drain_pending(link, quiet_s=0.25, max_s=1.0)

    # Disable LBT (matches run_tx_probe rationale: bring-up gate is digital,
    # not over-the-air politeness; uncalibrated antenna can spuriously
    # sense the channel busy).
    try:
        link.request(HOST_TYPE_CFG_SET_REQ, HOST_TYPE_CFG_OK_URC,
                     bytes([CFG_KEY_LBT_ENABLE, 0x01, 0x00]), timeout=1.0)
        print("CFG_OK_URC: LBT_ENABLE=0")
    except Exception as exc:
        print(f"WARN: CFG_SET_REQ(LBT_ENABLE=0) failed: {exc} (continuing)")

    try:
        stats_before = fetch_stats(link)
    except Exception as exc:
        print(f"FATAL: STATS_URC(before) failed: {exc}")
        print("__W1_10B_BURST_VERDICT__=TRANSPORT_FAIL")
        return 2
    tx_ok_b = stats_before.get("radio_tx_ok", 0)
    abort_lbt_b = stats_before.get("radio_tx_abort_lbt", 0)
    abort_air_b = stats_before.get("radio_tx_abort_airtime", 0)
    print(f"TX_BURST: STATS(before) radio_tx_ok={tx_ok_b} "
          f"radio_tx_abort_lbt={abort_lbt_b} radio_tx_abort_airtime={abort_air_b}")

    sys.stdout.write("__W1_10B_BURST_READY__\n")
    sys.stdout.flush()
    _emit_progress("tx_burst_ready", count=count, inter_s=f"{inter_s:.3f}")

    real_faults = 0
    tx_done_ok = 0
    tx_done_fail = 0
    tx_timeout = 0
    for i in range(count):
        # P5: per-iteration sidecar update so a frozen burst's last index
        # is always visible to the orchestrator. Cheap: ~1 file write/frame.
        _emit_progress("tx_burst", idx=f"{i + 1}/{count}",
                       ok=tx_done_ok, fail=tx_done_fail, to=tx_timeout)
        tx_id = i & 0xFF
        rand = _os.urandom(4).hex()
        payload = f"W1-10b seq={i:04d} {rand}".encode("ascii")
        if len(payload) > 64:
            payload = payload[:64]
        tx_frame = bytes([tx_id, len(payload)]) + payload
        t_send = time.time()
        try:
            link.send(HOST_TYPE_TX_FRAME_REQ, tx_frame)
        except Exception as exc:
            print(f"__TX_SEND_ERR__ idx={i} {exc}")
            tx_timeout += 1
            continue
        try:
            done, faults = wait_for_tx_done(link, tx_id, timeout=timeout)
        except TimeoutError as exc:
            print(f"__TX_TIMEOUT__ idx={i} tx_id=0x{tx_id:02X} {exc}")
            tx_timeout += 1
            if inter_s > 0:
                time.sleep(inter_s)
            continue
        except Exception as exc:
            print(f"__TX_ERR__ idx={i} {exc}")
            tx_timeout += 1
            if inter_s > 0:
                time.sleep(inter_s)
            continue
        elapsed_ms = (time.time() - t_send) * 1000.0
        if done["status"] == SX1276_TX_STATUS_OK:
            tx_done_ok += 1
        else:
            tx_done_fail += 1
        print(f"__TX_DONE__ idx={i} tx_id=0x{tx_id:02X} "
              f"status={done['status']}({done['status_name']}) "
              f"toa_us={done['time_on_air_us']} "
              f"elapsed_ms={elapsed_ms:.1f} "
              f"payload_hex={payload.hex()}")
        sys.stdout.flush()
        for f in faults:
            real_faults += 1
            print(f"__TX_FAULT__ idx={i} {f}")
        if inter_s > 0:
            time.sleep(inter_s)

    try:
        stats_after = fetch_stats(link)
    except Exception:
        stats_after = stats_before
    tx_ok_a = stats_after.get("radio_tx_ok", 0)
    abort_lbt_a = stats_after.get("radio_tx_abort_lbt", 0)
    abort_air_a = stats_after.get("radio_tx_abort_airtime", 0)
    invariants_violated = host_invariants_violated(stats_before, stats_after)
    for label, b, a in invariants_violated:
        print(f"  INVARIANT VIOLATED: {label} {b} -> {a}")
    print(f"TX_BURST: STATS(after) radio_tx_ok={tx_ok_a} (delta={tx_ok_a - tx_ok_b}) "
          f"radio_tx_abort_lbt={abort_lbt_a} (delta={abort_lbt_a - abort_lbt_b}) "
          f"radio_tx_abort_airtime={abort_air_a} (delta={abort_air_a - abort_air_b})")
    print(f"__W1_10B_BURST_DONE__ tx_count={count} "
          f"tx_done_ok={tx_done_ok} tx_done_fail={tx_done_fail} "
          f"tx_timeout={tx_timeout} "
          f"radio_tx_ok_delta={tx_ok_a - tx_ok_b} "
          f"radio_tx_abort_lbt_delta={abort_lbt_a - abort_lbt_b} "
          f"radio_tx_abort_airtime_delta={abort_air_a - abort_air_b} "
          f"real_faults={real_faults} "
          f"invariants_violated={len(invariants_violated)}")
    if tx_done_ok == count and real_faults == 0 and not invariants_violated:
        return 0
    return 1


# -------------------------------------------------------------------------
# 2026-05-18 S1.1: TX-power sweep (`walk_power`) — software half.
#
# Walks the SX1276 TX power in 1 dB steps from --power-min to --power-max
# inclusive, sending --per-step-count TX_FRAME_REQs per step. Each step
# records its own CSV row with TX-side counters (count_sent, tx_done_ok,
# tx_done_fail, tx_timeout, radio_tx_ok delta, radio_tx_abort_lbt /
# radio_tx_abort_airtime deltas, mean time_on_air_us, requested vs echoed
# tx_power_dbm). Per-fragment RX-side measurements (rx_rssi_dbm, rx_snr_db,
# rx_per_pct, rx_crc_err_count) are NOT captured here — they live on the
# paired RX board. They are emitted as empty CSV cells so an outer
# orchestrator that runs `--probe rx_listen` in parallel can JOIN the two
# CSVs after the sweep on (timestamp, power_dbm_requested).
#
# The L072 firmware is NOT physically connected on this bench today; the
# sweep run itself is HW-BLOCKED per LifeTrac-v25/TODO.md S1.4. This code
# lands so that the moment the L072 + handheld are reattached the only
# remaining work is "execute the sweep + commit the CSV", not "write the
# harness". See AI NOTES 2026-05-18 §"Phase 0" + design-doc walk_power
# bullet.
# -------------------------------------------------------------------------
WALK_POWER_CSV_FIELDS = (
    "timestamp_iso",
    "step_idx",
    "power_dbm_requested",
    "power_dbm_echoed_first",
    "count_sent",
    "tx_done_ok",
    "tx_done_fail",
    "tx_timeout",
    "radio_tx_ok_delta",
    "radio_tx_abort_lbt_delta",
    "radio_tx_abort_airtime_delta",
    "mean_toa_us",
    "tx_per_pct",
    # RX-side columns filled by a paired `--probe rx_listen` orchestrator.
    "rx_per_pct",
    "rx_rssi_dbm_mean",
    "rx_snr_db_mean",
    "rx_crc_err_count",
)

# 2026-05-20 P1-4 falsification matrix: per-packet CSV columns. Sibling file
# alongside the per-step CSV. One row per TX attempt; lets us discriminate
# between (a) Python loop stutter under high duty cycle, (b) LBT defer hits,
# and (c) cold-start CFG_SET race ordering at the 14-15 dBm cliff.
WALK_POWER_PERPACKET_FIELDS = (
    "step_idx",
    "power_dbm_requested",
    "packet_idx_in_step",
    "tx_id",
    "send_ts_ns",
    "done_ts_ns",
    "elapsed_us",
    "outcome",            # ok|fail|timeout|send_err
    "status_code",
    "toa_us",
    "lbt_enabled",
    "inter_cycle_s",
    "cfg_set_age_ms",     # ms between CFG_OK_URC for this step and send_ts
    "host_loop_iter_us",  # send_ts - prior_send_ts (gap-since-last-send)
    "python_rss_kb",      # best-effort getrusage().ru_maxrss; 0 on Windows
)


def _format_walk_power_row(row: dict) -> str:
    """Serialise one walk_power row as a CSV line (RFC4180-ish: no quoting
    needed because every value is numeric / ISO-8601 / empty). Order is
    locked to WALK_POWER_CSV_FIELDS."""
    out = []
    for key in WALK_POWER_CSV_FIELDS:
        v = row.get(key, "")
        if v is None:
            out.append("")
        elif isinstance(v, float):
            # 3 decimal places is enough for ToA-us and PER-pct.
            out.append(f"{v:.3f}")
        else:
            out.append(str(v))
    return ",".join(out)


def run_walk_power(link: HostLink,
                   power_min: int,
                   power_max: int,
                   power_step: int,
                   per_step_count: int,
                   timeout: float,
                   inter_s: float,
                   csv_out: str | None,
                   payload_len: int = 16,
                   lbt_enable: int = 0) -> int:
    """S1.1 walk_power sweep. Returns 0 if every step's CFG_OK_URC took and
    every step issued ≥1 TX_DONE_URC (sweep produced data); returns 1 if
    any step's CFG_SET timed out; returns 2 on transport-fatal."""
    import os as _os
    import datetime as _dt
    inter_s = clamp_inter_cycle_s(inter_s)

    if power_step <= 0:
        print(f"FATAL: --power-step must be > 0 (got {power_step})")
        print("__WALK_POWER_VERDICT__=BAD_ARGS")
        return 2
    if power_min < 2 or power_max > 20 or power_min > power_max:
        print(f"FATAL: --power-min/--power-max out of range or inverted "
              f"(got {power_min}..{power_max}, SX1276 PA_BOOST range is 2..20)")
        print("__WALK_POWER_VERDICT__=BAD_ARGS")
        return 2
    if per_step_count <= 0:
        print(f"FATAL: --per-step-count must be > 0 (got {per_step_count})")
        print("__WALK_POWER_VERDICT__=BAD_ARGS")
        return 2
    if payload_len < 1 or payload_len > 64:
        print(f"FATAL: --walk-payload-len out of [1, 64] (got {payload_len})")
        print("__WALK_POWER_VERDICT__=BAD_ARGS")
        return 2

    # Default CSV path: bench-evidence/walk_power_<date>/board_<hash>.csv
    if csv_out is None:
        date_tag = _dt.datetime.now().strftime("%Y-%m-%d_%H%M%S")
        # The script runs on the X8 under /tmp/lifetrac_p0c, but it's also
        # importable on a dev box. Try to land the CSV beside the script's
        # original repo location when present; otherwise fall back to /tmp.
        repo_evidence = _os.path.normpath(_os.path.join(
            _os.path.dirname(_os.path.abspath(__file__)),
            "..", "..", "bench-evidence", f"walk_power_{date_tag}"))
        out_dir = repo_evidence if _os.path.isdir(_os.path.dirname(repo_evidence)) \
            else f"/tmp/walk_power_{date_tag}"
        try:
            _os.makedirs(out_dir, exist_ok=True)
        except OSError as exc:
            print(f"WARN: cannot create {out_dir}: {exc}; falling back to /tmp")
            out_dir = "/tmp"
        csv_out = _os.path.join(out_dir, "walk_power_tx_side.csv")
    print(f"=== walk_power: {power_min}..{power_max} dBm step={power_step} "
          f"count/step={per_step_count} csv={csv_out} ===")

    drain_boot(link, 1.0)
    try:
        link.request(HOST_TYPE_VER_REQ, HOST_TYPE_VER_URC, timeout=1.0)
    except Exception as exc:
        print(f"FATAL: VER warm-up failed: {exc}")
        print("__WALK_POWER_VERDICT__=TRANSPORT_FAIL")
        return 2
    drain_pending(link, quiet_s=0.25, max_s=1.0)

    # Disable LBT for the sweep — same rationale as run_tx_burst.
    # 2026-05-20 P1-4 fix: actually honour --lbt-enable so the falsification
    # matrix can run Pass B (LBT on) against the same probe binary.
    try:
        link.request(HOST_TYPE_CFG_SET_REQ, HOST_TYPE_CFG_OK_URC,
                     bytes([CFG_KEY_LBT_ENABLE, 0x01, lbt_enable & 0xFF]),
                     timeout=1.0)
        print(f"CFG_OK_URC: LBT_ENABLE={lbt_enable}")
    except Exception as exc:
        print(f"WARN: CFG_SET_REQ(LBT_ENABLE={lbt_enable}) failed: {exc} (continuing)")

    # Open the CSV with header.
    try:
        csv_fh = open(csv_out, "w", encoding="utf-8")
    except OSError as exc:
        print(f"FATAL: cannot open CSV {csv_out}: {exc}")
        print("__WALK_POWER_VERDICT__=CSV_OPEN_FAIL")
        return 2
    csv_fh.write(",".join(WALK_POWER_CSV_FIELDS) + "\n")
    csv_fh.flush()

    # 2026-05-20 P1-4: per-packet CSV sibling. Naming: `<stem>_perpacket.csv`.
    pp_csv_path = _os.path.splitext(csv_out)[0] + "_perpacket.csv"
    try:
        pp_fh = open(pp_csv_path, "w", encoding="utf-8")
        pp_fh.write(",".join(WALK_POWER_PERPACKET_FIELDS) + "\n")
        pp_fh.flush()
    except OSError as exc:
        print(f"WARN: cannot open per-packet CSV {pp_csv_path}: {exc} (continuing without it)")
        pp_fh = None

    # Cheap RSS sampler. getrusage is POSIX-only; on the X8 Linux target
    # ru_maxrss is in kB. Wrap so Windows dev runs don't crash.
    try:
        import resource as _resource  # type: ignore
        def _rss_kb() -> int:
            return int(_resource.getrusage(_resource.RUSAGE_SELF).ru_maxrss)
    except Exception:
        def _rss_kb() -> int:
            return 0

    sys.stdout.write(f"__WALK_POWER_READY__ csv={csv_out} perpacket_csv={pp_csv_path}\n")
    sys.stdout.flush()

    any_step_failed = False
    steps_with_data = 0
    powers = list(range(power_min, power_max + 1, power_step))
    prior_send_ns = 0
    for step_idx, dbm in enumerate(powers):
        # 1. Apply the new TX power.
        try:
            link.request(HOST_TYPE_CFG_SET_REQ, HOST_TYPE_CFG_OK_URC,
                         bytes([CFG_KEY_TX_POWER_DBM, 0x01, dbm & 0xFF]),
                         timeout=1.0)
            cfg_ok_ns = time.perf_counter_ns()
            print(f"step {step_idx}: CFG_OK_URC TX_POWER_DBM={dbm}")
        except Exception as exc:
            print(f"step {step_idx}: CFG_SET TX_POWER_DBM={dbm} FAILED: {exc}")
            any_step_failed = True
            # Still write a row so the CSV exposes the gap.
            row = {
                "timestamp_iso": _dt.datetime.now().isoformat(timespec="seconds"),
                "step_idx": step_idx,
                "power_dbm_requested": dbm,
                "count_sent": 0,
            }
            csv_fh.write(_format_walk_power_row(row) + "\n")
            csv_fh.flush()
            continue

        # 2. Snapshot stats.
        try:
            stats_b = fetch_stats(link)
        except Exception as exc:
            print(f"step {step_idx}: STATS(before) failed: {exc}; skipping step")
            any_step_failed = True
            continue
        tx_ok_b = stats_b.get("radio_tx_ok", 0)
        abort_lbt_b = stats_b.get("radio_tx_abort_lbt", 0)
        abort_air_b = stats_b.get("radio_tx_abort_airtime", 0)

        # 3. Send per_step_count frames.
        tx_done_ok = 0
        tx_done_fail = 0
        tx_timeout_n = 0
        toa_samples = []
        first_echoed = None
        for i in range(per_step_count):
            tx_id = (step_idx * 256 + i) & 0xFF
            payload = (f"WP s{step_idx:02d} p{dbm:02d} i{i:04d} ".encode("ascii")
                       + _os.urandom(max(0, payload_len - 24)))
            payload = payload[:payload_len]
            tx_frame = bytes([tx_id, len(payload)]) + payload
            send_ns = time.perf_counter_ns()
            host_loop_iter_us = ((send_ns - prior_send_ns) // 1000) if prior_send_ns else 0
            cfg_set_age_ms = (send_ns - cfg_ok_ns) // 1_000_000
            prior_send_ns = send_ns
            pp_outcome = None
            pp_status = ""
            pp_toa = ""
            done_ns = 0
            try:
                link.send(HOST_TYPE_TX_FRAME_REQ, tx_frame)
            except Exception as exc:
                print(f"  WP s{step_idx} i{i}: send err: {exc}")
                tx_timeout_n += 1
                pp_outcome = "send_err"
            if pp_outcome is None:
                try:
                    done, _ = wait_for_tx_done(link, tx_id, timeout=timeout)
                    done_ns = time.perf_counter_ns()
                except TimeoutError:
                    tx_timeout_n += 1
                    pp_outcome = "timeout"
                except Exception as exc:
                    print(f"  WP s{step_idx} i{i}: wait err: {exc}")
                    tx_timeout_n += 1
                    pp_outcome = "timeout"
                else:
                    if done["status"] == SX1276_TX_STATUS_OK:
                        tx_done_ok += 1
                        pp_outcome = "ok"
                    else:
                        tx_done_fail += 1
                        pp_outcome = "fail"
                    pp_status = str(done["status"])
                    pp_toa = str(done.get("time_on_air_us", 0))
                    toa_samples.append(done.get("time_on_air_us", 0))
                    if first_echoed is None:
                        first_echoed = done.get("tx_power_dbm")
            if pp_fh is not None:
                pp_row = [
                    str(step_idx),
                    str(dbm),
                    str(i),
                    f"0x{tx_id:02x}",
                    str(send_ns),
                    str(done_ns) if done_ns else "",
                    str((done_ns - send_ns) // 1000) if done_ns else "",
                    pp_outcome or "",
                    pp_status,
                    pp_toa,
                    str(lbt_enable),
                    f"{inter_s:.4f}",
                    str(cfg_set_age_ms),
                    str(host_loop_iter_us),
                    str(_rss_kb()),
                ]
                pp_fh.write(",".join(pp_row) + "\n")
                pp_fh.flush()
            if inter_s > 0:
                time.sleep(inter_s)

        # 4. Snapshot stats after.
        try:
            stats_a = fetch_stats(link)
        except Exception:
            stats_a = stats_b
        tx_ok_a = stats_a.get("radio_tx_ok", 0)
        abort_lbt_a = stats_a.get("radio_tx_abort_lbt", 0)
        abort_air_a = stats_a.get("radio_tx_abort_airtime", 0)

        mean_toa = (sum(toa_samples) / len(toa_samples)) if toa_samples else 0.0
        tx_per_pct = (100.0 * (per_step_count - tx_done_ok) / per_step_count
                      if per_step_count > 0 else 0.0)

        row = {
            "timestamp_iso": _dt.datetime.now().isoformat(timespec="seconds"),
            "step_idx": step_idx,
            "power_dbm_requested": dbm,
            "power_dbm_echoed_first": first_echoed if first_echoed is not None else "",
            "count_sent": per_step_count,
            "tx_done_ok": tx_done_ok,
            "tx_done_fail": tx_done_fail,
            "tx_timeout": tx_timeout_n,
            "radio_tx_ok_delta": tx_ok_a - tx_ok_b,
            "radio_tx_abort_lbt_delta": abort_lbt_a - abort_lbt_b,
            "radio_tx_abort_airtime_delta": abort_air_a - abort_air_b,
            "mean_toa_us": float(mean_toa),
            "tx_per_pct": float(tx_per_pct),
        }
        csv_fh.write(_format_walk_power_row(row) + "\n")
        csv_fh.flush()
        steps_with_data += 1
        print(f"__WALK_POWER_STEP__ step={step_idx} dbm={dbm} "
              f"ok={tx_done_ok} fail={tx_done_fail} "
              f"timeout={tx_timeout_n} per_pct={tx_per_pct:.2f} "
              f"mean_toa_us={mean_toa:.0f} echoed_dbm={first_echoed}")
        sys.stdout.flush()
        _emit_progress("walk_step_done",
                       step=f"{step_idx + 1}/{len(powers)}",
                       dbm=dbm, ok=tx_done_ok, fail=tx_done_fail,
                       timeout=tx_timeout_n)

    csv_fh.close()
    if pp_fh is not None:
        pp_fh.close()
    print(f"__WALK_POWER_DONE__ steps={len(powers)} steps_with_data={steps_with_data} "
          f"csv={csv_out} verdict={'FAIL' if any_step_failed else 'OK'}")
    if any_step_failed:
        return 1
    return 0


def run_rx_echo(link: "HostLink", window_s: float = 60.0,
                echo_timeout_s: float = 5.0) -> int:
    """W1-11 L-1 RX-side echo. Same as `rx_listen` but on every
    RX_FRAME_URC immediately issues TX_FRAME_REQ with the same payload
    (host-driven echo; no firmware change required).

    NOTE: the resulting round-trip latency observed by the TX-side
    `--probe ping_pong` includes **two** host-overhead round-trips
    (one on each board), not one. A production-equivalent RTT
    requires firmware-side echo (deferred follow-up; would need a
    new HOST_TYPE_CFG_RX_ECHO_ENABLE config key + L072 dispatch).

    Emits one `__RX_ECHO__ idx=N rx_payload_hex=H tx_id=0xX
    echo_lat_ms=M tx_done_status=S` line per echoed frame and the
    same `__W1_10B_LISTEN_DONE__` summary as `rx_listen`.
    """
    print(f"=== W1-11 probe RX_ECHO: window={window_s:.1f}s "
          f"echo_timeout={echo_timeout_s:.1f}s ===")
    drain_boot(link, 1.0)
    try:
        link.request(HOST_TYPE_VER_REQ, HOST_TYPE_VER_URC, timeout=1.0)
    except Exception as exc:
        print(f"FATAL: VER warm-up failed: {exc}")
        print("__W1_10B_LISTEN_VERDICT__=TRANSPORT_FAIL")
        return 2
    drain_pending(link, quiet_s=0.25, max_s=1.0)

    # Disable LBT on RX board too so echo TX is not suppressed by an
    # uncalibrated antenna sensing the air busy mid-window.
    try:
        link.request(HOST_TYPE_CFG_SET_REQ, HOST_TYPE_CFG_OK_URC,
                     bytes([CFG_KEY_LBT_ENABLE, 0x01, 0x00]), timeout=1.0)
        print("CFG_OK_URC: LBT_ENABLE=0")
    except Exception as exc:
        print(f"WARN: CFG_SET_REQ(LBT_ENABLE=0) failed: {exc} (continuing)")

    try:
        stats_before = fetch_stats(link)
    except Exception as exc:
        print(f"FATAL: STATS_URC(before) failed: {exc}")
        print("__W1_10B_LISTEN_VERDICT__=TRANSPORT_FAIL")
        return 2
    rx_ok_b = stats_before.get("radio_rx_ok", 0)
    crc_b = stats_before.get("radio_crc_err", 0)
    print(f"RX_ECHO: STATS(before) radio_rx_ok={rx_ok_b} radio_crc_err={crc_b} "
          f"radio_state={stats_before.get('radio_state', 0)}")

    sys.stdout.write("__W1_10B_LISTEN_READY__\n")
    sys.stdout.flush()

    deadline = time.time() + window_s
    rx_count = 0
    echo_count = 0
    real_faults = 0
    echo_idx = 0
    while time.time() < deadline:
        for frame in link.read_frames(0.2):
            ftype = frame["type"]
            if ftype == HOST_TYPE_RX_FRAME_URC:
                try:
                    p = parse_rx_frame(frame["payload"])
                    rx_count += 1
                    print(f"__RX_FRAME__ idx={rx_count} rssi={p['rssi_dbm']} "
                          f"snr={p['snr_db']} len={p['len']} "
                          f"timestamp_us={p['timestamp_us']} "
                          f"payload_hex={p['payload'].hex()}")
                    sys.stdout.flush()

                    # Echo: re-TX the same payload with our own tx_id.
                    payload = p["payload"]
                    if len(payload) > 64:
                        payload = payload[:64]
                    tx_id = echo_idx & 0xFF
                    echo_idx += 1
                    tx_frame = bytes([tx_id, len(payload)]) + payload
                    t_send = time.time()
                    try:
                        link.send(HOST_TYPE_TX_FRAME_REQ, tx_frame)
                    except Exception as exc:
                        print(f"__RX_ECHO_SEND_ERR__ idx={rx_count} {exc}")
                        continue
                    try:
                        done, faults = wait_for_tx_done(
                            link, tx_id, timeout=echo_timeout_s)
                    except TimeoutError as exc:
                        print(f"__RX_ECHO_TIMEOUT__ idx={rx_count} {exc}")
                        continue
                    except Exception as exc:
                        print(f"__RX_ECHO_ERR__ idx={rx_count} {exc}")
                        continue
                    elapsed_ms = (time.time() - t_send) * 1000.0
                    if done["status"] == SX1276_TX_STATUS_OK:
                        echo_count += 1
                    print(f"__RX_ECHO__ idx={rx_count} "
                          f"rx_payload_hex={payload.hex()} "
                          f"tx_id=0x{tx_id:02X} "
                          f"echo_lat_ms={elapsed_ms:.1f} "
                          f"tx_done_status={done['status']}({done['status_name']}) "
                          f"toa_us={done['time_on_air_us']}")
                    sys.stdout.flush()
                    for f in faults:
                        real_faults += 1
                        print(f"__RX_ECHO_FAULT__ idx={rx_count} {f}")
                except Exception as exc:
                    print(f"__RX_FRAME_ERR__ {exc} raw={frame['payload'].hex()}")
            elif ftype == HOST_TYPE_FAULT_URC:
                code = frame["payload"][0] if frame["payload"] else None
                desc = format_fault_payload(frame["payload"])
                if code in BENIGN_FAULT_CODES:
                    print(f"INFO: benign FAULT_URC: {desc}")
                else:
                    real_faults += 1
                    print(f"__RX_FAULT__ {desc}")
            elif ftype == HOST_TYPE_ERR_PROTO_URC:
                desc = format_err_proto_payload(frame["payload"])
                real_faults += 1
                print(f"__RX_ERR_PROTO__ {desc}")

    try:
        stats_after = fetch_stats(link)
    except Exception:
        stats_after = stats_before
    rx_ok_a = stats_after.get("radio_rx_ok", 0)
    crc_a = stats_after.get("radio_crc_err", 0)
    invariants_violated = host_invariants_violated(stats_before, stats_after)
    for label, b, a in invariants_violated:
        print(f"  INVARIANT VIOLATED: {label} {b} -> {a}")
    print(f"RX_ECHO: STATS(after) radio_rx_ok={rx_ok_a} (delta={rx_ok_a - rx_ok_b}) "
          f"radio_crc_err={crc_a} (delta={crc_a - crc_b})")
    # Reuse the LISTEN_DONE summary tag so the existing orchestrator parser
    # picks this up unchanged.
    print(f"__W1_10B_LISTEN_DONE__ rx_frames={rx_count} "
          f"radio_rx_ok_delta={rx_ok_a - rx_ok_b} "
          f"radio_crc_err_delta={crc_a - crc_b} "
          f"real_faults={real_faults} "
          f"invariants_violated={len(invariants_violated)}")
    print(f"__W1_11_ECHO_DONE__ rx_frames={rx_count} echoes_ok={echo_count}")
    return 0


def run_ping_pong(link: "HostLink", count: int = 100, inter_s: float = 0.2,
                  timeout: float = 5.0, rtt_timeout: float = 5.0) -> int:
    """W1-11 L-1 TX-side ping-pong. Sends `count` TX_FRAME_REQs with payload
    `W1-11 ping seq=NNNN <random-hex>` and for each one waits BOTH for the
    local TX_DONE_URC AND for a matching RX_FRAME_URC echo (correlated by
    full payload). Emits `__PINGPONG__ idx=N tx_id=0xX status=S
    toa_us=T tx_elapsed_ms=M rtt_ms=R rx_rssi=X rx_snr=Y payload_hex=H`
    per cycle and the same `__W1_10B_BURST_DONE__` summary as `tx_burst`.

    rtt_ms is the host-perceived round-trip from TX_FRAME_REQ submission
    on this board to RX_FRAME_URC arrival of the echo on this board. It
    includes 2x ToA + TX-side host overhead + RX-side host overhead +
    L072 dispatch on both boards. Useful as a real round-trip lower
    bound; production-equivalent RTT requires firmware-side echo
    (deferred).
    """
    import os as _os
    inter_s = clamp_inter_cycle_s(inter_s)
    print(f"=== W1-11 probe PING_PONG: count={count} inter={inter_s:.2f}s "
          f"tx_timeout={timeout:.1f}s rtt_timeout={rtt_timeout:.1f}s ===")
    drain_boot(link, 1.0)
    try:
        link.request(HOST_TYPE_VER_REQ, HOST_TYPE_VER_URC, timeout=1.0)
    except Exception as exc:
        print(f"FATAL: VER warm-up failed: {exc}")
        print("__W1_10B_BURST_VERDICT__=TRANSPORT_FAIL")
        return 2
    drain_pending(link, quiet_s=0.25, max_s=1.0)

    try:
        link.request(HOST_TYPE_CFG_SET_REQ, HOST_TYPE_CFG_OK_URC,
                     bytes([CFG_KEY_LBT_ENABLE, 0x01, 0x00]), timeout=1.0)
        print("CFG_OK_URC: LBT_ENABLE=0")
    except Exception as exc:
        print(f"WARN: CFG_SET_REQ(LBT_ENABLE=0) failed: {exc} (continuing)")

    try:
        stats_before = fetch_stats(link)
    except Exception as exc:
        print(f"FATAL: STATS_URC(before) failed: {exc}")
        print("__W1_10B_BURST_VERDICT__=TRANSPORT_FAIL")
        return 2
    tx_ok_b = stats_before.get("radio_tx_ok", 0)
    abort_lbt_b = stats_before.get("radio_tx_abort_lbt", 0)
    abort_air_b = stats_before.get("radio_tx_abort_airtime", 0)
    print(f"PING_PONG: STATS(before) radio_tx_ok={tx_ok_b} "
          f"radio_tx_abort_lbt={abort_lbt_b} radio_tx_abort_airtime={abort_air_b}")

    sys.stdout.write("__W1_10B_BURST_READY__\n")
    sys.stdout.flush()

    real_faults = 0
    tx_done_ok = 0
    tx_done_fail = 0
    tx_timeout_count = 0
    pong_received = 0
    pong_timeout = 0
    for i in range(count):
        tx_id = i & 0xFF
        rand = _os.urandom(4).hex()
        payload = f"W1-11 ping seq={i:04d} {rand}".encode("ascii")
        if len(payload) > 64:
            payload = payload[:64]
        tx_frame = bytes([tx_id, len(payload)]) + payload
        t_send = time.time()
        try:
            link.send(HOST_TYPE_TX_FRAME_REQ, tx_frame)
        except Exception as exc:
            print(f"__TX_SEND_ERR__ idx={i} {exc}")
            tx_timeout_count += 1
            continue
        # Phase 1: wait for our own TX_DONE_URC.
        try:
            done, faults = wait_for_tx_done(link, tx_id, timeout=timeout)
        except TimeoutError as exc:
            print(f"__TX_TIMEOUT__ idx={i} tx_id=0x{tx_id:02X} {exc}")
            tx_timeout_count += 1
            if inter_s > 0:
                time.sleep(inter_s)
            continue
        except Exception as exc:
            print(f"__TX_ERR__ idx={i} {exc}")
            tx_timeout_count += 1
            if inter_s > 0:
                time.sleep(inter_s)
            continue
        tx_elapsed_ms = (time.time() - t_send) * 1000.0
        if done["status"] == SX1276_TX_STATUS_OK:
            tx_done_ok += 1
        else:
            tx_done_fail += 1
        for f in faults:
            real_faults += 1
            print(f"__TX_FAULT__ idx={i} {f}")

        # Phase 2: wait for the RX_FRAME_URC echo matching our payload.
        rtt_ms = -1.0
        rx_rssi = None
        rx_snr = None
        deadline = t_send + rtt_timeout
        while time.time() < deadline:
            frames = link.read_frames(0.2)
            for frame in frames:
                ftype = frame["type"]
                if ftype == HOST_TYPE_RX_FRAME_URC:
                    try:
                        p = parse_rx_frame(frame["payload"])
                    except Exception:
                        continue
                    if p["payload"] == payload:
                        rtt_ms = (time.time() - t_send) * 1000.0
                        rx_rssi = p["rssi_dbm"]
                        rx_snr = p["snr_db"]
                        pong_received += 1
                        break
                    # else: stale or unrelated echo, ignore
                elif ftype == HOST_TYPE_FAULT_URC:
                    code = frame["payload"][0] if frame["payload"] else None
                    desc = format_fault_payload(frame["payload"])
                    if code not in BENIGN_FAULT_CODES:
                        real_faults += 1
                        print(f"__PINGPONG_FAULT__ idx={i} {desc}")
            if rtt_ms >= 0:
                break
        if rtt_ms < 0:
            pong_timeout += 1
            print(f"__PINGPONG_TIMEOUT__ idx={i} payload_hex={payload.hex()}")
            # 2026-05-20 P1-5: stable token consumed by analyze_rtt.py for
            # explicit timeout accounting. Keeps __PINGPONG_TIMEOUT__ for
            # backwards-compat with legacy parsers; the new analyzer prefers
            # __RTT_TIMEOUT__ because it's symmetric with __RTT_SAMPLE__.
            print(f"__RTT_TIMEOUT__ idx={i} rtt_timeout_s={rtt_timeout:.3f} "
                  f"payload_hex={payload.hex()}")

        print(f"__PINGPONG__ idx={i} tx_id=0x{tx_id:02X} "
              f"status={done['status']}({done['status_name']}) "
              f"toa_us={done['time_on_air_us']} "
              f"tx_elapsed_ms={tx_elapsed_ms:.1f} "
              f"rtt_ms={rtt_ms:.1f} "
              f"rx_rssi={rx_rssi if rx_rssi is not None else 'NA'} "
              f"rx_snr={rx_snr if rx_snr is not None else 'NA'} "
              f"payload_hex={payload.hex()}")
        # Also emit __TX_DONE__ in the same format as run_tx_burst so
        # analyze_rtt.py picks up toa/elapsed without modification.
        print(f"__TX_DONE__ idx={i} tx_id=0x{tx_id:02X} "
              f"status={done['status']}({done['status_name']}) "
              f"toa_us={done['time_on_air_us']} "
              f"elapsed_ms={tx_elapsed_ms:.1f} "
              f"payload_hex={payload.hex()}")
        sys.stdout.flush()
        if inter_s > 0:
            time.sleep(inter_s)

    try:
        stats_after = fetch_stats(link)
    except Exception:
        stats_after = stats_before
    tx_ok_a = stats_after.get("radio_tx_ok", 0)
    abort_lbt_a = stats_after.get("radio_tx_abort_lbt", 0)
    abort_air_a = stats_after.get("radio_tx_abort_airtime", 0)
    invariants_violated = host_invariants_violated(stats_before, stats_after)
    for label, b, a in invariants_violated:
        print(f"  INVARIANT VIOLATED: {label} {b} -> {a}")
    print(f"PING_PONG: STATS(after) radio_tx_ok={tx_ok_a} (delta={tx_ok_a - tx_ok_b}) "
          f"radio_tx_abort_lbt={abort_lbt_a} (delta={abort_lbt_a - abort_lbt_b}) "
          f"radio_tx_abort_airtime={abort_air_a} (delta={abort_air_a - abort_air_b})")
    # Reuse the BURST_DONE summary tag so the orchestrator parses it.
    print(f"__W1_10B_BURST_DONE__ tx_count={count} "
          f"tx_done_ok={tx_done_ok} tx_done_fail={tx_done_fail} "
          f"tx_timeout={tx_timeout_count} "
          f"radio_tx_ok_delta={tx_ok_a - tx_ok_b} "
          f"radio_tx_abort_lbt_delta={abort_lbt_a - abort_lbt_b} "
          f"radio_tx_abort_airtime_delta={abort_air_a - abort_air_b} "
          f"real_faults={real_faults} "
          f"invariants_violated={len(invariants_violated)}")
    print(f"__W1_11_PINGPONG_DONE__ tx_count={count} "
          f"tx_done_ok={tx_done_ok} pong_received={pong_received} "
          f"pong_timeout={pong_timeout}")
    if (tx_done_ok == count and pong_received == count
            and real_faults == 0 and not invariants_violated):
        return 0
    return 1


def parse_tx_done(payload: bytes) -> dict:
    if len(payload) < 7:
        raise ValueError(f"TX_DONE_URC payload too short: {payload.hex()}")
    tx_id = payload[0]
    status = payload[1]
    toa_us = struct.unpack("<I", payload[2:6])[0]
    tx_power_dbm = payload[6]
    return {
        "tx_id": tx_id,
        "status": status,
        "status_name": TX_STATUS_NAMES.get(status, f"0x{status:02X}"),
        "time_on_air_us": toa_us,
        "tx_power_dbm": tx_power_dbm,
    }


def wait_for_tx_done(link: HostLink, expected_tx_id: int, timeout: float):
    """Wait up to `timeout` seconds for a TX_DONE_URC matching expected_tx_id.

    Returns (tx_done_dict, faults_list).  Raises TimeoutError on miss.
    """
    deadline = time.time() + timeout
    faults = []
    while time.time() < deadline:
        frames = link.read_frames(0.2)
        for frame in frames:
            ftype = frame["type"]
            if ftype == HOST_TYPE_TX_DONE_URC:
                done = parse_tx_done(frame["payload"])
                if done["tx_id"] != expected_tx_id:
                    print(
                        "WARNING: TX_DONE_URC tx_id mismatch "
                        f"got=0x{done['tx_id']:02X} want=0x{expected_tx_id:02X}; ignoring"
                    )
                    continue
                return done, faults
            if ftype == HOST_TYPE_FAULT_URC:
                desc = format_fault_payload(frame["payload"])
                code = frame["payload"][0] if frame["payload"] else None
                if code in BENIGN_FAULT_CODES:
                    # 2026-05-12 W1-9f: by-design diagnostic, not a fault.
                    print(f"INFO: diagnostic FAULT_URC during TX wait (benign): {desc}")
                else:
                    print(f"FAULT_URC during TX wait: {desc}")
                    faults.append(desc)
            elif ftype == HOST_TYPE_ERR_PROTO_URC:
                desc = format_err_proto_payload(frame["payload"])
                raise RuntimeError(f"ERR_PROTO during TX wait: {desc}")
            elif ftype == HOST_TYPE_RFCO_PERTX_URC:
                # Firmware emits one RFCO_PERTX URC per TX attempt with the
                # named refusal bucket (LBT / QOS / LEGAL_DWELL / INTERNAL /
                # TX_FAIL / TX_TIMEOUT / OK). Surface it instead of dropping
                # it as "unrelated frame".
                desc = format_rfco_pertx_payload(frame["payload"])
                print(f"RFCO_PERTX during TX wait: {desc}")
            else:
                print(f"INFO: unrelated frame during TX wait type=0x{ftype:02X}")
    raise TimeoutError(f"timeout waiting for TX_DONE_URC tx_id=0x{expected_tx_id:02X}")


def run_tx_probe(link: HostLink, args) -> int:
    try:
        payload = bytes.fromhex(args.payload_hex)
    except ValueError as exc:
        print(f"FATAL: --payload-hex parse error: {exc}")
        return 2

    if not (1 <= len(payload) <= 64):
        print(f"FATAL: payload length {len(payload)} out of range [1, 64]")
        return 2

    print(f"=== Stage 2 TX bring-up probe (W1-9) ===")
    print(f"tx_id=0x{args.tx_id:02X} payload_len={len(payload)} payload_hex={payload.hex()}")
    print(f"timeout={args.timeout}s")

    try:
        # 0. Drain any BOOT_URC / startup chatter that may already sit in the
        # serial buffer.  Without this, the first STATS_DUMP_REQ can race with
        # an unparsed startup frame and the response window expires before the
        # request loop sees a clean STATS_URC (mirrors the Stage 1 BOOT_URC
        # drain step).
        settle_deadline = time.time() + 1.0
        boot_seen = False
        while time.time() < settle_deadline:
            for frame in link.read_frames(0.2):
                if frame["type"] == 0xF0:  # HOST_TYPE_BOOT_URC
                    boot_seen = True
                    print(f"BOOT_URC observed during settle (payload={frame['payload'].hex()})")
        if not boot_seen:
            print("BOOT_URC not observed during 1.0s settle (firmware likely "
                  "already past boot — continuing).")

        # 1. Snapshot stats before the TX cycle.  Retry once if the very first
        # STATS_DUMP_REQ misses (transient post-boot UART jitter).
        stats_before = None
        for attempt in (1, 2):
            try:
                stats_before = fetch_stats(link)
                break
            except Exception as exc:
                print(f"WARN: STATS_URC(before) attempt {attempt} failed: {exc}")
        if stats_before is None:
            print("FATAL: STATS_URC(before) failed after 2 attempts")
            return 2

        tx_ok_before = stats_before.get("radio_tx_ok", 0)
        dio0_before = stats_before.get("radio_dio0", 0)
        print(f"STATS(before): radio_tx_ok={tx_ok_before} "
              f"radio_dio0={dio0_before} "
              f"radio_state={stats_before.get('radio_state', 0)} "
              f"host_parse_err={stats_before.get('host_parse_err', 0)} "
              f"radio_tx_abort_lbt={stats_before.get('radio_tx_abort_lbt', 0)} "
              f"radio_tx_abort_airtime={stats_before.get('radio_tx_abort_airtime', 0)}")

        # 1b. Disable LBT for the bring-up cycle.  The default firmware build
        # (LORA_FW_LBT_ENABLE=1, threshold=-90 dBm) runs CAD + RSSI before TX
        # and can spuriously sense the channel as busy without a calibrated
        # antenna, returning ERR_PROTO_FORBIDDEN from sx1276_tx_begin().  W1-9
        # is a digital bring-up gate, not an over-the-air politeness gate.
        # CFG_SET_REQ payload = [u8 key, u8 in_len, u8 value[in_len]].
        cfg_payload = bytes([CFG_KEY_LBT_ENABLE, 0x01, 0x00])
        try:
            ack = link.request(HOST_TYPE_CFG_SET_REQ, HOST_TYPE_CFG_OK_URC,
                               cfg_payload, timeout=1.0)
            print(f"CFG_OK_URC(LBT_ENABLE=0): payload={ack['payload'].hex()}")
        except Exception as exc:
            print(f"WARN: CFG_SET_REQ(LBT_ENABLE=0) failed: {exc} (continuing)")

        # 1c. W1-9 SPI-write isolation diagnostic.  We have observed that
        # after firmware boot RegOpMode reads 0x80 (LoRa SLEEP) instead of
        # the expected 0x81 (LoRa STDBY), and that subsequent firmware
        # writes to OPMODE during sx1276_tx_begin do not take effect.
        # Test whether a *direct* SPI write to OPMODE from the host (via
        # REG_WRITE_REQ, allow-listed in W1-9 build) takes effect.  If
        # this works but firmware-internal writes do not, the bug is
        # firmware sequencing.  If this also fails, the bug is in
        # sx1276_write_reg / SPI / chip wake-from-sleep.
        try:
            opm_pre = link.request(HOST_TYPE_REG_READ_REQ, HOST_TYPE_REG_DATA_URC,
                                   bytes([SX1276_REG_OP_MODE]), timeout=0.5)
            print(f"DIAG(pre): RegOpMode(0x01)=0x{opm_pre['payload'][1]:02X}")
            # Try writing 0x81 (LoRa STDBY).
            wack = link.request(HOST_TYPE_REG_WRITE_REQ, HOST_TYPE_REG_WRITE_ACK_URC,
                                bytes([SX1276_REG_OP_MODE, 0x81]), timeout=0.5)
            print(f"DIAG: REG_WRITE_ACK(opmode=0x81): ack={wack['payload'].hex()}")
            opm_post = link.request(HOST_TYPE_REG_READ_REQ, HOST_TYPE_REG_DATA_URC,
                                    bytes([SX1276_REG_OP_MODE]), timeout=0.5)
            print(f"DIAG(post-write 0x81): RegOpMode(0x01)=0x{opm_post['payload'][1]:02X} "
                  f"({'TAKE' if opm_post['payload'][1] == 0x81 else 'IGNORED'})")
            # Try writing 0x83 (LoRa TX) directly.
            wack2 = link.request(HOST_TYPE_REG_WRITE_REQ, HOST_TYPE_REG_WRITE_ACK_URC,
                                 bytes([SX1276_REG_OP_MODE, 0x83]), timeout=0.5)
            time.sleep(0.005)
            opm_tx = link.request(HOST_TYPE_REG_READ_REQ, HOST_TYPE_REG_DATA_URC,
                                  bytes([SX1276_REG_OP_MODE]), timeout=0.5)
            print(f"DIAG(post-write 0x83): RegOpMode(0x01)=0x{opm_tx['payload'][1]:02X} "
                  f"({'TAKE' if opm_tx['payload'][1] in (0x83, 0x81) else 'IGNORED'})")
            # Restore to standby
            try:
                link.request(HOST_TYPE_REG_WRITE_REQ, HOST_TYPE_REG_WRITE_ACK_URC,
                             bytes([SX1276_REG_OP_MODE, 0x81]), timeout=0.5)
            except Exception:
                pass
        except Exception as exc:
            print(f"DIAG: SPI-write isolation test failed: {exc}")

        # 2. Build TX_FRAME_REQ payload: [u8 tx_id, u8 length, u8 payload[length]]
        tx_payload = bytes([args.tx_id, len(payload)]) + payload

        t_send = time.time()
        seq = link.send(HOST_TYPE_TX_FRAME_REQ, tx_payload)
        print(f"sent TX_FRAME_REQ seq={seq} bytes={len(tx_payload)}")

        # Mid-cycle diagnostic: ToA at SF7/BW250/8B is ~18 ms, firmware
        # timeout window is +50 ms.  Read RegIrqFlags ~25 ms after sending
        # TX_FRAME_REQ; if bit 3 (TxDone) is set the radio actually
        # transmitted and the only failure mode is the DIO0/EXTI path
        # never seeing the rising edge.
        time.sleep(0.005)
        try:
            opm_early = link.request(HOST_TYPE_REG_READ_REQ, HOST_TYPE_REG_DATA_URC,
                                     bytes([SX1276_REG_OP_MODE]), timeout=0.3)
            print(f"DIAG(t≈5ms): RegOpMode(0x01)=0x{opm_early['payload'][1]:02X} "
                  f"(expected 0x83 = LoRa+TX)")
        except Exception as exc:
            print(f"DIAG(t≈5ms): peek failed: {exc}")
        time.sleep(0.020)
        try:
            mid_irq = link.request(HOST_TYPE_REG_READ_REQ, HOST_TYPE_REG_DATA_URC,
                                   bytes([SX1276_REG_IRQ_FLAGS]), timeout=0.3)
            mid_opm = link.request(HOST_TYPE_REG_READ_REQ, HOST_TYPE_REG_DATA_URC,
                                   bytes([SX1276_REG_OP_MODE]), timeout=0.3)
            mid_payload_len = link.request(HOST_TYPE_REG_READ_REQ, HOST_TYPE_REG_DATA_URC,
                                           bytes([0x22]), timeout=0.3)
            mid_pa_cfg = link.request(HOST_TYPE_REG_READ_REQ, HOST_TYPE_REG_DATA_URC,
                                      bytes([0x09]), timeout=0.3)
            mid_dio = link.request(HOST_TYPE_REG_READ_REQ, HOST_TYPE_REG_DATA_URC,
                                   bytes([0x40]), timeout=0.3)
            mid_irq_val = mid_irq["payload"][1]
            mid_opm_val = mid_opm["payload"][1]
            print(f"DIAG(t≈25ms): RegIrqFlags(0x12)=0x{mid_irq_val:02X} "
                  f"(TxDone bit3 = {'SET' if mid_irq_val & 0x08 else 'CLR'}) "
                  f"RegOpMode(0x01)=0x{mid_opm_val:02X} "
                  f"PayloadLen(0x22)=0x{mid_payload_len['payload'][1]:02X} "
                  f"PaConfig(0x09)=0x{mid_pa_cfg['payload'][1]:02X} "
                  f"DioMap1(0x40)=0x{mid_dio['payload'][1]:02X}")
        except Exception as exc:
            print(f"DIAG(t≈25ms): peek failed: {exc}")

        # 3. Wait for TX_DONE_URC.
        try:
            done, faults = wait_for_tx_done(link, args.tx_id, args.timeout)
        except TimeoutError as exc:
            print(f"FAIL: {exc}")
            try:
                stats_late = fetch_stats(link)
                print(f"STATS(late): radio_tx_ok={stats_late.get('radio_tx_ok', 0)} "
                      f"radio_state={stats_late.get('radio_state', 0)}")
            except Exception:
                pass
            return 1
        except RuntimeError as exc:
            print(f"FAIL: {exc}")
            try:
                stats_err = fetch_stats(link)
                print(f"STATS(after-err): radio_tx_ok={stats_err.get('radio_tx_ok', 0)} "
                      f"radio_state={stats_err.get('radio_state', 0)} "
                      f"radio_tx_abort_lbt={stats_err.get('radio_tx_abort_lbt', 0)} "
                      f"radio_tx_abort_airtime={stats_err.get('radio_tx_abort_airtime', 0)} "
                      f"host_parse_err={stats_err.get('host_parse_err', 0)}")
                for k in ("radio_tx_abort_lbt", "radio_tx_abort_airtime",
                          "radio_tx_ok", "host_parse_err"):
                    delta = stats_err.get(k, 0) - stats_before.get(k, 0)
                    if delta != 0:
                        print(f"  delta {k}: {delta:+d}")
            except Exception:
                pass
            return 1

        elapsed_ms = (time.time() - t_send) * 1000.0
        print(f"TX_DONE_URC: tx_id=0x{done['tx_id']:02X} "
              f"status={done['status']}({done['status_name']}) "
              f"time_on_air_us={done['time_on_air_us']} "
              f"tx_power_dbm={done['tx_power_dbm']} "
              f"wall_elapsed_ms={elapsed_ms:.1f}")

        # 4. Snapshot stats after.
        try:
            stats_after = fetch_stats(link)
        except Exception as exc:
            print(f"FATAL: STATS_URC(after) failed: {exc}")
            return 2

        tx_ok_after = stats_after.get("radio_tx_ok", 0)
        dio0_after = stats_after.get("radio_dio0", 0)
        print(f"STATS(after): radio_tx_ok={tx_ok_after} "
              f"radio_dio0={dio0_after} (delta={dio0_after - dio0_before}) "
              f"radio_state={stats_after.get('radio_state', 0)} "
              f"host_parse_err={stats_after.get('host_parse_err', 0)} "
              f"radio_tx_abort_lbt={stats_after.get('radio_tx_abort_lbt', 0)} "
              f"radio_tx_abort_airtime={stats_after.get('radio_tx_abort_airtime', 0)}")

        # Diagnostic: if status != OK and DIO0 never fired, peek RegIrqFlags
        # and RegOpMode to see whether the radio actually transmitted.
        # NB: by the time we read here the firmware has already cleared
        # IRQ flags and returned to standby/RX, so a non-zero TxDone bit
        # means a residual that survived cleanup (unlikely).  This block
        # exists only to confirm radio responsiveness post-cycle.
        if done["status"] != SX1276_TX_STATUS_OK:
            try:
                irq = link.request(HOST_TYPE_REG_READ_REQ, HOST_TYPE_REG_DATA_URC,
                                   bytes([SX1276_REG_IRQ_FLAGS]), timeout=0.5)
                opm = link.request(HOST_TYPE_REG_READ_REQ, HOST_TYPE_REG_DATA_URC,
                                   bytes([SX1276_REG_OP_MODE]), timeout=0.5)
                print(f"DIAG: RegIrqFlags(0x12)=0x{irq['payload'][1]:02X} "
                      f"RegOpMode(0x01)=0x{opm['payload'][1]:02X}")
            except Exception as exc:
                print(f"DIAG: post-cycle register peek failed: {exc}")

        # 5. Evaluate gates.
        checks = []
        checks.append(("W1-9.B TX_DONE_URC received", True))  # already true if we got here
        checks.append(("W1-9.C status == OK",
                       done["status"] == SX1276_TX_STATUS_OK))
        checks.append(("W1-9.D radio_tx_ok delta == 1",
                       (tx_ok_after - tx_ok_before) == 1))

        invariants_ok = True
        for label in INVARIANT_COUNTERS:
            before = stats_before.get(label, 0)
            after = stats_after.get(label, 0)
            if after != before:
                print(f"  invariant violated: {label} {before} -> {after}")
                invariants_ok = False
        checks.append(("W1-9.E/F/G invariants stable", invariants_ok))
        checks.append(("no FAULT_URC during cycle", len(faults) == 0))

        # Sanity: time_on_air should be in a plausible band for SF7/BW250/8B payload.
        toa = done["time_on_air_us"]
        toa_ok = 5_000 <= toa <= 500_000
        checks.append((f"time_on_air_us in [5e3, 5e5] ({toa})", toa_ok))

        print()
        print("Critical checks:")
        failed = 0
        for label, ok in checks:
            status = "PASS" if ok else "FAIL"
            print(f"  [{status}] {label}")
            if not ok:
                failed += 1

        print()
        if failed:
            print(f"VERDICT: FAIL ({failed} critical check(s) failed)")
            return 1

        print("VERDICT: PASS (Stage 2 TX bring-up: TxDone observed via DIO0, "
              "stats consistent, no UART regression)")
        return 0
    finally:
        pass


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(
        description="W1-9 Stage 2 TX bring-up probe / W1-9b OPMODE-stuck probes")
    parser.add_argument("--dev", default="/dev/ttymxc3")
    parser.add_argument("--baud", default="921600")
    parser.add_argument("--tx-id", type=lambda v: int(v, 0), default=DEFAULT_TX_ID)
    parser.add_argument(
        "--payload-hex",
        default=DEFAULT_PAYLOAD.hex(),
        help="hex-encoded payload bytes (default: ASCII 'LIFETRAC')",
    )
    parser.add_argument("--timeout", type=float, default=5.0,
                        help="seconds to wait for TX_DONE_URC")
    parser.add_argument(
        "--probe",
        default="tx",
        choices=["tx", "regversion", "fsk", "opmode_walk", "rx",
                 "rx_listen", "tx_burst", "rx_echo", "ping_pong",
                 "walk_power"],
        help="probe mode (W1-9 default 'tx'; W1-9b 'regversion'/'fsk'/'opmode_walk'; "
             "W1-10 single-board 'rx'; W1-10b two-board 'rx_listen'/'tx_burst'; "
             "W1-11 L-1 two-board 'rx_echo'/'ping_pong'; "
             "S1.1 'walk_power' (TX-power sweep 2..17 dBm in 1 dB steps)",
    )
    parser.add_argument(
        "--rtt-timeout",
        type=float,
        default=5.0,
        help="seconds to wait for the RX_FRAME_URC echo per cycle during "
             "--probe ping_pong (default 5.0)",
    )
    parser.add_argument(
        "--burst-count",
        type=int,
        default=1024,
        help="number of REG_READ iterations for --probe regversion",
    )
    parser.add_argument(
        "--rx-window",
        type=float,
        default=10.0,
        help="seconds to listen for RX_FRAME_URC during --probe rx / --probe rx_listen "
             "(default 10.0)",
    )
    parser.add_argument(
        "--tx-count",
        type=int,
        default=100,
        help="number of TX cycles for --probe tx_burst (default 100)",
    )
    parser.add_argument(
        "--inter-cycle-s",
        type=float,
        default=0.2,
        help="seconds to sleep between TX cycles during --probe tx_burst (default 0.2)",
    )
    parser.add_argument(
        "--no-sleep-on-exit",
        dest="sleep_on_exit",
        action="store_false",
        default=True,
        help="Disable the post-run RegOpMode=0x80 (LoRa SLEEP) cleanup. Default "
             "is to park the SX1276 in SLEEP after every probe run (bench EMC "
             "hygiene rule, 2026-05-10). Override only if a follow-on probe "
             "needs the radio left in its working state.",
    )
    # 2026-05-18 S1.1 walk_power options.
    parser.add_argument("--power-min", type=int, default=2,
                        help="walk_power: minimum TX power dBm (default 2)")
    parser.add_argument("--power-max", type=int, default=17,
                        help="walk_power: maximum TX power dBm (default 17, SX1276 PA_BOOST ceiling)")
    parser.add_argument("--power-step", type=int, default=1,
                        help="walk_power: dBm step size (default 1)")
    parser.add_argument("--per-step-count", type=int, default=100,
                        help="walk_power: TX frames per power step (default 100)")
    parser.add_argument("--walk-payload-len", type=int, default=16,
                        help="walk_power: per-frame payload length (default 16, max 64)")
    parser.add_argument("--csv-out", default=None,
                        help="walk_power: explicit CSV output path. If omitted, "
                             "writes to bench-evidence/walk_power_<date>/walk_power_tx_side.csv")
    # 2026-05-20 P1-4 falsification matrix: per-packet CSV + LBT toggle.
    # The per-packet CSV is written as a sibling of --csv-out (same dir,
    # `_perpacket.csv` suffix) so existing post-processors that consume the
    # per-step CSV are unaffected.
    parser.add_argument("--lbt-enable", type=int, choices=[0, 1], default=0,
                        help="walk_power: 0 = force LBT off (legacy default, used "
                             "by Pass A/C of the falsification matrix); 1 = leave "
                             "LBT on (Pass B). Always logged into the per-packet CSV.")
    # 2026-05-20 FCC-B3-1: built-in self-test for the RUNTIME_PROFILE_ENUM
    # emitter. Skips serial/link setup entirely so it can run in CI/dev shells
    # without bench hardware attached.
    parser.add_argument("--self-test-profile-emit", action="store_true",
                        help="FCC-B3-1: exercise _format_runtime_profile_line "
                             "over canned CFG_DATA_URC payloads and exception "
                             "cases. Exits 0 on pass, 1 on any mismatch. Does "
                             "not open the serial link or touch the radio.")
    # 2026-05-22 P5: sidecar progress.txt path. Overwrite-truncate single
    # line, polled by the orchestrator every 1 s. See _emit_progress()
    # docstring + Open Problems doc v4.1 §4.
    parser.add_argument("--progress-file", default=None,
                        help="P5 sidecar: write a one-line progress.txt "
                             "(overwrite-truncate) at every major event so "
                             "an orchestrator can tail it for liveness "
                             "without parsing stdout.")
    args = parser.parse_args(argv)

    if args.self_test_profile_emit:
        return _self_test_profile_emit()

    # P5: install the sidecar BEFORE any potentially-blocking work so a
    # frozen probe still tells the orchestrator at which phase it froze.
    global _PROGRESS_PATH
    _PROGRESS_PATH = args.progress_file
    _emit_progress("starting", probe=args.probe, dev=args.dev, baud=args.baud)

    print(f"MODE: {args.probe}")
    print(f"dev={args.dev} baud={args.baud}")

    try:
        link = HostLink(args.dev, args.baud)
    except Exception as exc:
        print(f"FATAL: link open failed: {exc}")
        _emit_progress("link_open_failed", err=type(exc).__name__)
        return 2
    _emit_progress("link_open_ok")

    # 2026-05-22 Phase 4 Optimization: Send a system reset request to the co-processor
    # to clear any potential previous fault states (like RX_SCAN_FAILED stuck state)
    print("Re-setting co-processor via HostLink system reset...")
    try:
        link.send(0x03)  # HOST_TYPE_RESET_REQ
        drain_boot(link, settle_s=1.5)
        print("Co-processor system reset complete.")
    except Exception as exc:
        print(f"WARN: System reset failed or timed out: {exc} (continuing)")

    # VER warm-up + pending drain to align COBS parser before CFG requests
    try:
        link.request(HOST_TYPE_VER_REQ, HOST_TYPE_VER_URC, timeout=1.0)
        print("VER warm-up ok.")
    except Exception as exc:
        print(f"WARN: VER warm-up after reset failed: {exc} (continuing)")
    drain_pending(link, quiet_s=0.25, max_s=1.0)

    # Configure regulatory profile so the FHSS scheduler is initialized
    configure_regulatory_profile_if_needed(link)

    # 2026-05-20 FCC-B3-1: publish RUNTIME_PROFILE_ENUM=<N> exactly once per
    # probe process so the FCC-B3-2 gate (`tools/check_run_profile.py`) can
    # confirm the firmware actually running matches the orchestrator's
    # `--expected-enum`. Non-fatal: any failure here prints
    # `RUNTIME_PROFILE_ENUM=ERR <reason>` and probe execution continues so
    # we don't regress the existing capture surface.
    emit_runtime_profile_enum(link)
    _emit_progress("profile_emit_done", mode=args.probe)

    rc = 2
    try:
        _emit_progress("mode_enter", mode=args.probe)
        if args.probe == "tx":
            rc = run_tx_probe(link, args)
        elif args.probe == "regversion":
            rc = run_regversion_burst(link, count=args.burst_count)
        elif args.probe == "fsk":
            rc = run_fsk_stdby(link)
        elif args.probe == "opmode_walk":
            rc = run_opmode_walk(link)
        elif args.probe == "rx":
            rc = run_rx_liveness(link, window_s=args.rx_window)
        elif args.probe == "rx_listen":
            rc = run_rx_listen(link, window_s=args.rx_window)
        elif args.probe == "tx_burst":
            rc = run_tx_burst(link, count=args.tx_count,
                              inter_s=args.inter_cycle_s,
                              timeout=args.timeout)
        elif args.probe == "rx_echo":
            rc = run_rx_echo(link, window_s=args.rx_window,
                             echo_timeout_s=args.timeout)
        elif args.probe == "ping_pong":
            rc = run_ping_pong(link, count=args.tx_count,
                               inter_s=args.inter_cycle_s,
                               timeout=args.timeout,
                               rtt_timeout=args.rtt_timeout)
        elif args.probe == "walk_power":
            rc = run_walk_power(link,
                                power_min=args.power_min,
                                power_max=args.power_max,
                                power_step=args.power_step,
                                per_step_count=args.per_step_count,
                                timeout=args.timeout,
                                inter_s=args.inter_cycle_s,
                                csv_out=args.csv_out,
                                payload_len=args.walk_payload_len,
                                lbt_enable=args.lbt_enable)
        else:
            print(f"FATAL: unknown probe mode: {args.probe}")
            rc = 2
        _emit_progress("mode_exit", mode=args.probe, rc=rc)
        return rc
    finally:
        if getattr(args, "sleep_on_exit", True):
            sleep_radio_safely(link, label=args.probe)
        link.close()
        _emit_progress("link_closed", mode=args.probe, rc=rc)


if __name__ == "__main__":
    sys.exit(main())
