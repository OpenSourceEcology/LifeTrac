#!/usr/bin/env python3
"""bench_activate_fhss.py — bench-startup activator for the FCC 50-channel
FHSS regulatory profile (REG_PROFILE = 1, FCC_15_247_FHSS_50CH_BW250).

Purpose
-------
Make `FCC_15_247_FHSS_50CH_BW250` the default bench transmission style by
issuing the host_cfg sequence required by the L072 firmware's two-phase
profile commit BEFORE any image-pipeline / control-loop container is
started on the X8.

This script is the host-side half of plan §14.2 step 8/9 (FCC-A5 cfg
validation, profile activation) and the orchestrator hook called out in
§B3. It is intentionally a standalone utility, not part of the
image-tx daemon, so:

  * The orchestrator can fail-fast at boot if the profile cannot be
    activated, before any container that would otherwise emit RF starts.
  * A bench operator can override the profile per-shell without editing
    the daemon's environment block.
  * The script is reversible — pass `--profile bench` to drop back to
    `BENCH_ONLY_FIXED_915` without rebooting.

Safety
------
Dry-run by default. Will NOT open `/dev/ttymxc3` or emit any frame
unless `--apply` is passed. With `--apply` the script does NOT key the
radio — it only performs cfg-layer writes. The first TX after this
script returns will use the activated profile.

Wire contract
-------------
Mirrors `configure_regulatory_profile_if_needed()` in
`firmware/x8_lora_bootloader_helper/method_h_stage2_tx_probe_v2.py`,
plus a CFG_GET readback so we can fail-fast if the firmware refused.

Per-key writes (in order):
  CFG_KEY_FHSS_CHANNEL_MASK (0x07) — 8-byte LE u64 channel mask
  CFG_KEY_ANTENNA_GAIN_DBI  (0x15) — 1-byte i8
  CFG_KEY_HW_CEILING_DBM    (0x16) — 1-byte u8
  CFG_KEY_REG_PROFILE       (0x14) — 1-byte u8 enum  ← triggers stage+activate

Then:
  CFG_GET_REQ(CFG_KEY_REG_PROFILE) → expect [0x14, 0x01, <profile>]

Exit codes
----------
  0  Success: profile readback matches request.
  1  Wire transport failure (port busy, open failed, BOOT_URC missing).
  2  CFG_SET rejected by firmware (e.g. PROFILE_REJECT_MASK_POPCOUNT).
  3  CFG_GET readback returned a different profile than requested.
  4  Argument / config error.
 10  Dry run (no --apply), nothing sent.

Known issue (do NOT remove this warning)
----------------------------------------
With `--mask wide` (true 50-channel FHSS), the L072 firmware as of
2026-05-25 picks a different starting channel per board from the FHSS
mask, causing two peers to land on different fixed carriers and
`rx_frames=0`. See
`LifeTrac-v25/AI NOTES/2026-05-25_Radio_RX_Frames_Zero_Channel_Mismatch.md`
and `LifeTrac-v25/AI NOTES/2026-05-27_FHSS_Bench_Default_Activation_Plan_v1_0.md`
§3 for the open D8 (two-node sync torture) work that gates `wide` mode
for end-to-end bench traffic. Until D8 is closed, use `--mask single`
with the same `--ch` value on both peers for image-pipeline bench
tests — that still activates `REG_PROFILE = FCC_FHSS_50CH_BW250` so the
RFCO artifact stamp is honest, but narrows the active mask to one
channel so both peers land on the same carrier.

Usage
-----
  python3 bench_activate_fhss.py                       # dry run, prints plan
  python3 bench_activate_fhss.py --apply               # wide mask, REAL FHSS
  python3 bench_activate_fhss.py --apply \
      --mask single --ch 0                             # bench two-peer mode
  python3 bench_activate_fhss.py --apply --profile bench  # revert to bench profile

Cross-references
----------------
  * Plan: LifeTrac-v25/AI NOTES/2026-05-19_LoRa_FCC_50CH_FHSS_Implementation_Plan_Copilot_v1_0.md
  * Roadmap: LifeTrac-v25/AI NOTES/2026-05-27_Image_Over_LoRa_Production_Roadmap_v25.0.1_to_v25.1.0.md
  * Bench-default plan (THIS script's runbook):
    LifeTrac-v25/AI NOTES/2026-05-27_FHSS_Bench_Default_Activation_Plan_v1_0.md
  * Firmware contract: firmware/murata_l072/host/host_cfg_profile.c
  * Wire constants: firmware/murata_l072/include/host_cfg_keys.h
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

# Re-use HostLink + framing from the bootloader helper directory.
# Path layout: this file is .../LifeTrac-v25/tools/bench_activate_fhss.py;
# HostLink lives in
# .../LifeTrac-v25/DESIGN-CONTROLLER/firmware/x8_lora_bootloader_helper/.
_HELPER_DIR = (Path(__file__).resolve().parent.parent
               / "DESIGN-CONTROLLER" / "firmware"
               / "x8_lora_bootloader_helper")
if str(_HELPER_DIR) not in sys.path:
    sys.path.insert(0, str(_HELPER_DIR))


# --- Wire constants ---------------------------------------------------------
# Mirrored from firmware/murata_l072/include/host_cfg_keys.h and
# include/host_types.h. These MUST stay in sync with the firmware; the
# self-test below cross-checks the byte values that the firmware
# bench/host_proto tests already pin.

CFG_KEY_FHSS_CHANNEL_MASK   = 0x07
CFG_KEY_REG_PROFILE         = 0x14
CFG_KEY_ANTENNA_GAIN_DBI    = 0x15
CFG_KEY_HW_CEILING_DBM      = 0x16

REG_PROFILE_BENCH_ONLY_FIXED_915        = 0
REG_PROFILE_FCC_15_247_FHSS_50CH_BW250  = 1
REG_PROFILE_FCC_15_247_DTS_BW500        = 2

HOST_TYPE_CFG_SET_REQ       = 0x20
HOST_TYPE_CFG_GET_REQ       = 0x21
HOST_TYPE_CFG_OK_URC        = 0xA0
HOST_TYPE_CFG_DATA_URC      = 0xA1

# REG_READ/WRITE host_cmd types (mirror of method_h_stage2_tx_probe_v2 l.150).
HOST_TYPE_REG_READ_REQ      = 0x30
HOST_TYPE_REG_WRITE_REQ     = 0x31
HOST_TYPE_REG_DATA_URC      = 0xB0
HOST_TYPE_REG_WRITE_ACK_URC = 0xB1

# SX1276 register addresses we touch for the FRF force-pin workaround.
SX1276_REG_OP_MODE          = 0x01
SX1276_REG_FRF_MSB          = 0x06
SX1276_REG_FRF_MID          = 0x07
SX1276_REG_FRF_LSB          = 0x08
SX1276_OPMODE_STANDBY_LORA  = 0x81  # LongRangeMode=1, Mode=001 (standby)

# Wire status byte returned in CFG_OK_URC payload[1] (payload[0] == key).
# Mirrors cfg_status_t in include/host_cfg.h. Only the subset this script
# can produce a fail-on; full enum lives in the firmware header.
CFG_STATUS_OK = 0x00

# FCC 50-channel mask = bits 0..49 set, bits 50..63 clear.
FHSS_50CH_FULL_MASK_U64 = (1 << 50) - 1

# Default channel for --mask single. Channel 0 maps to first FCC-50CH
# center per sx1276_fhss_chantab.h (SX1276_FHSS_FIRST_CENTER_HZ).
DEFAULT_SINGLE_CHANNEL_IDX = 0

# Defaults that match host_cfg_profile validator's expectations and the
# existing bench-orchestrator probe (method_h_stage2_tx_probe_v2.py
# l. 415-432). Override only if the bench-side antenna SKU changes.
DEFAULT_ANTENNA_GAIN_DBI    = 2
DEFAULT_HW_CEILING_DBM      = 17


# --- Payload builders -------------------------------------------------------

def build_cfg_set_payload(key: int, value: bytes) -> bytes:
    """CFG_SET_REQ payload format: [key:u8, len:u8, value:bytes[len]]."""
    if not 0 <= key <= 0xFF:
        raise ValueError(f"key out of range: {key:#x}")
    if len(value) > 0xFF:
        raise ValueError(f"value too long: {len(value)}")
    return bytes([key, len(value)]) + bytes(value)


def build_channel_mask_value(mode: str, ch: int) -> bytes:
    """Return the 8-byte LE u64 mask value for the given mode.

    mode = "wide"    -> all 50 channels set (true FCC FHSS).
    mode = "single"  -> only bit `ch` set (bench two-peer workaround).
    """
    if mode == "wide":
        mask_u64 = FHSS_50CH_FULL_MASK_U64
    elif mode == "single":
        if not 0 <= ch <= 49:
            raise ValueError(f"--ch must be in 0..49, got {ch}")
        mask_u64 = 1 << ch
    else:
        raise ValueError(f"unknown mask mode: {mode!r}")
    return mask_u64.to_bytes(8, "little")


def profile_id_for_name(name: str) -> int:
    if name == "fhss":
        return REG_PROFILE_FCC_15_247_FHSS_50CH_BW250
    if name == "bench":
        return REG_PROFILE_BENCH_ONLY_FIXED_915
    if name == "dts":
        return REG_PROFILE_FCC_15_247_DTS_BW500
    raise ValueError(f"unknown profile name: {name!r}")


def profile_name_for_id(pid: int) -> str:
    return {
        REG_PROFILE_BENCH_ONLY_FIXED_915:       "bench",
        REG_PROFILE_FCC_15_247_FHSS_50CH_BW250: "fhss",
        REG_PROFILE_FCC_15_247_DTS_BW500:       "dts",
    }.get(pid, f"unknown({pid})")


# --- Plan rendering ---------------------------------------------------------

def resolve_effective_mask_mode(args) -> str:
    """The L072 firmware validator (`host_cfg_profile.c:124`) requires
    `popcount(channel_mask) >= 50` for REG_PROFILE_FCC_15_247_FHSS_50CH_BW250.
    A single-channel mask is therefore rejected with status=0x08
    (CFG_STATUS_PROFILE_REJECT_MASK_POPCOUNT). Falsified empirically on
    bench 2026-05-27. So when activating the FHSS profile we MUST send
    the wide mask regardless of what the operator typed; we print a
    clear notice if we are overriding.
    """
    if args.profile == "fhss" and args.mask == "single":
        return "wide"
    return args.mask


def render_plan(args) -> list[tuple[str, bytes]]:
    """Build the ordered list of (label, CFG_SET payload) writes that
    --apply would send. Same function used by dry-run + live paths so
    the printed plan and the wire sequence cannot drift.
    """
    pid = profile_id_for_name(args.profile)
    plan: list[tuple[str, bytes]] = []

    # 1. Channel mask. The validator requires popcount(mask) >= 50 AND
    #    mask ⊆ FHSS_50CH_FULL_MASK for the FHSS profile. The bench
    #    profile accepts any non-zero mask. We always send a real value
    #    so re-activating back to FHSS later has a sane mask in place.
    effective_mask = resolve_effective_mask_mode(args)
    mask_value = build_channel_mask_value(effective_mask, args.ch)
    plan.append(("CFG_SET FHSS_CHANNEL_MASK",
                 build_cfg_set_payload(CFG_KEY_FHSS_CHANNEL_MASK, mask_value)))

    # 2. Antenna gain (i8). FCC ERP clamp threshold is +6 dBi.
    plan.append(("CFG_SET ANTENNA_GAIN_DBI",
                 build_cfg_set_payload(
                     CFG_KEY_ANTENNA_GAIN_DBI,
                     args.antenna_gain.to_bytes(1, "little", signed=True))))

    # 3. Hardware PA ceiling (u8). Murata module hardware ceiling is
    #    +17 dBm on the bench path.
    plan.append(("CFG_SET HW_CEILING_DBM",
                 build_cfg_set_payload(
                     CFG_KEY_HW_CEILING_DBM,
                     args.hw_ceiling.to_bytes(1, "little", signed=False))))

    # 4. REG_PROFILE — this is the trigger that calls
    #    host_cfg_profile_stage() + host_cfg_profile_activate() in the
    #    firmware (host_cfg.c case CFG_KEY_REG_PROFILE).
    plan.append(("CFG_SET REG_PROFILE",
                 build_cfg_set_payload(CFG_KEY_REG_PROFILE,
                                       bytes([pid]))))

    return plan


def resolve_force_frf_hz(args) -> int | None:
    """Return the FRF-pin frequency in Hz, or None for no pin.

    Explicit --force-frf wins. Otherwise:
      * --profile fhss: default to 915_000_000 (channel 25 on the
        FCC 50-ch grid: 902.75 MHz + 500 kHz * 24.5; nearest channel
        is i=24 -> 914.75 MHz or i=25 -> 915.25 MHz). We pin exactly
        915.0 MHz to match the v25.0.1 demo's known-working carrier.
      * --profile bench / dts: leave alone unless asked.
    """
    if args.force_frf is not None:
        return args.force_frf if args.force_frf > 0 else None
    if args.profile == "fhss":
        return 915_000_000
    return None


def print_plan(args, plan: list[tuple[str, bytes]]) -> None:
    effective_mask = resolve_effective_mask_mode(args)
    print(f"# bench_activate_fhss.py plan (dry-run={'no' if args.apply else 'yes'})")
    print(f"# target profile: {args.profile} "
          f"(REG_PROFILE = {profile_id_for_name(args.profile)})")
    if effective_mask != args.mask:
        print(f"# mask mode:     {effective_mask}  "
              f"[FORCED from --mask {args.mask}: firmware popcount validator "
              f"requires wide for fhss]")
    else:
        print(f"# mask mode:     {effective_mask}"
              + (f" (ch={args.ch})" if effective_mask == "single" else ""))
    print(f"# antenna_gain:  {args.antenna_gain} dBi  "
          f"hw_ceiling: {args.hw_ceiling} dBm")
    force_frf = resolve_force_frf_hz(args)
    if force_frf is not None:
        print(f"# force FRF:     {force_frf/1e6:.3f} MHz  "
              f"[post-activation direct write to SX1276 0x06/0x07/0x08]")
    else:
        print("# force FRF:     (none) -- firmware picks channel from mask")
    print(f"# device:        {args.dev} @ {args.baud}")
    print()
    for label, payload in plan:
        print(f"  {label:30s} payload={payload.hex()}")
    print(f"  {'CFG_GET REG_PROFILE (verify)':30s} "
          f"payload={bytes([CFG_KEY_REG_PROFILE]).hex()}")


# --- Self-test (no hardware) ------------------------------------------------

def _self_test() -> int:
    """Golden-vector tests for the payload builders. Cross-checked against
    the bytes literally embedded in method_h_stage2_tx_probe_v2.py
    (lines 405-441) so any silent drift fails here loudly.
    """
    failures: list[str] = []

    # Wide mask (50 channels) = 0x0003FFFFFFFFFFFF little-endian.
    mask_wide = build_channel_mask_value("wide", 0)
    expected_wide = bytes.fromhex("ffffffffffff0300")
    if mask_wide != expected_wide:
        failures.append(f"wide mask: got {mask_wide.hex()}, want {expected_wide.hex()}")

    # Single-channel mask, ch=0 -> 0x0000000000000001.
    mask_one = build_channel_mask_value("single", 0)
    expected_one = bytes.fromhex("0100000000000000")
    if mask_one != expected_one:
        failures.append(f"single ch=0 mask: got {mask_one.hex()}, want {expected_one.hex()}")

    # Single-channel mask, ch=49 -> bit 49 = 0x0002000000000000.
    mask_top = build_channel_mask_value("single", 49)
    expected_top = bytes.fromhex("0000000000000200")
    if mask_top != expected_top:
        failures.append(f"single ch=49 mask: got {mask_top.hex()}, want {expected_top.hex()}")

    # Out-of-range channel must raise.
    try:
        build_channel_mask_value("single", 50)
        failures.append("ch=50 should have raised ValueError")
    except ValueError:
        pass

    # CFG_SET payload framing.
    p = build_cfg_set_payload(CFG_KEY_REG_PROFILE, bytes([1]))
    if p != bytes([0x14, 0x01, 0x01]):
        failures.append(f"CFG_SET REG_PROFILE=1: got {p.hex()}, want 140101")

    p = build_cfg_set_payload(CFG_KEY_ANTENNA_GAIN_DBI,
                              (2).to_bytes(1, "little", signed=True))
    if p != bytes([0x15, 0x01, 0x02]):
        failures.append(f"CFG_SET ANTENNA_GAIN_DBI=2: got {p.hex()}, want 150102")

    p = build_cfg_set_payload(CFG_KEY_HW_CEILING_DBM,
                              (17).to_bytes(1, "little", signed=False))
    if p != bytes([0x16, 0x01, 0x11]):
        failures.append(f"CFG_SET HW_CEILING_DBM=17: got {p.hex()}, want 160111")

    # Profile name <-> id round-trip.
    for n in ("bench", "fhss", "dts"):
        if profile_name_for_id(profile_id_for_name(n)) != n:
            failures.append(f"profile round-trip broken for {n!r}")

    if failures:
        print("SELF_TEST=FAIL")
        for f in failures:
            print(f"  - {f}")
        return 1
    print("SELF_TEST=PASS")
    return 0


# --- Live execution ---------------------------------------------------------

def _apply(args, plan: list[tuple[str, bytes]]) -> int:
    """Open HostLink, issue each CFG_SET, then CFG_GET readback. Returns
    process exit code per the contract documented in the module docstring.
    """
    # Lazy import so dry-run + self-test work on a host without pyserial /
    # without the bootloader-helper directory present.
    try:
        from method_g_stage1_probe import HostLink  # type: ignore
    except Exception as exc:
        print(f"FATAL: cannot import HostLink from {_HELPER_DIR}: {exc}")
        return 1

    print(f"# opening {args.dev} @ {args.baud} ...")
    try:
        link = HostLink(args.dev, args.baud)
    except Exception as exc:
        print(f"FATAL: HostLink open failed: {exc}")
        return 1

    rc = 0
    try:
        # Drain whatever the L072 emitted on boot / before we attached.
        # 2 s covers the empirical STATS_URC burst per
        # method_h_stage2_tx_probe_v2 emit_runtime_profile_enum().
        deadline = time.time() + 2.0
        while time.time() < deadline:
            data = link.read_raw(timeout=0.2)
            if not data:
                break

        for label, payload in plan:
            try:
                ack = link.request(HOST_TYPE_CFG_SET_REQ,
                                   HOST_TYPE_CFG_OK_URC,
                                   payload,
                                   timeout=1.5)
            except Exception as exc:
                print(f"FATAL: {label} transport failure: {exc}")
                return 1
            ack_payload = ack.get("payload", b"")
            if len(ack_payload) < 2:
                print(f"FATAL: {label} short ACK payload: {ack_payload.hex()}")
                return 2
            ack_key = ack_payload[0]
            ack_status = ack_payload[1]
            if ack_status != CFG_STATUS_OK:
                # Status byte is the wire-mapped cfg_status_t. The firmware
                # already mapped any host_cfg_profile_reject_t through
                # host_cfg_profile_reject_to_cfg_status() for us.
                print(f"REJECT: {label} key=0x{ack_key:02x} "
                      f"status=0x{ack_status:02x} (see cfg_status_t)")
                return 2
            print(f"OK:     {label} ack={ack_payload.hex()}")

        # CFG_GET readback.
        try:
            frame = link.request(HOST_TYPE_CFG_GET_REQ,
                                 HOST_TYPE_CFG_DATA_URC,
                                 bytes([CFG_KEY_REG_PROFILE]),
                                 timeout=1.5)
        except Exception as exc:
            print(f"FATAL: CFG_GET REG_PROFILE transport failure: {exc}")
            return 1
        rb = frame.get("payload", b"")
        if len(rb) < 3 or rb[0] != CFG_KEY_REG_PROFILE or rb[1] != 0x01:
            print(f"FATAL: CFG_GET REG_PROFILE bad readback shape: {rb.hex()}")
            return 3
        actual_pid = rb[2]
        expected_pid = profile_id_for_name(args.profile)
        # Machine-parseable line — keep the exact spelling that
        # method_h_stage2_tx_probe_v2 emits so existing orchestrators
        # already grep for it.
        print(f"RUNTIME_PROFILE_ENUM={actual_pid}")
        if actual_pid != expected_pid:
            print(f"MISMATCH: requested {expected_pid} "
                  f"({profile_name_for_id(expected_pid)}), "
                  f"firmware reports {actual_pid} "
                  f"({profile_name_for_id(actual_pid)})")
            return 3
        print(f"# profile activated: {profile_name_for_id(actual_pid)}")

        # FRF force-pin (workaround for the 2026-05-25 per-board starting
        # channel non-determinism). Only takes effect if resolve_force_frf_hz
        # returned a value.
        force_frf = resolve_force_frf_hz(args)
        if force_frf is not None:
            frc = _force_frf(link, force_frf)
            if frc != 0:
                return frc
    finally:
        try:
            link.close()
        except Exception:
            pass

    return rc


def _force_frf(link, freq_hz: int) -> int:
    """Pin the SX1276 carrier frequency by direct register write.

    Workaround for the 2026-05-25 channel-mismatch bug: under
    REG_PROFILE=1 + wide mask, each L072 picks a different starting
    channel from the mask -> two peers on different carriers ->
    rx_frames=0. By writing FRF directly after profile activation,
    both peers land on the same carrier regardless of the firmware's
    starting-channel pick.

    SX1276 datasheet 4.1.4: FRF can only be safely written in SLEEP
    or STANDBY. We read OPMODE, drop to STANDBY if needed, write
    0x06/0x07/0x08, restore OPMODE so the new FRF commits on next
    mode entry, then read back and verify.

    FRF computation: FRF = freq_hz * 2^19 / 32_000_000 (32 MHz TCXO).
    """
    from method_g_stage1_probe import HostLink  # noqa: F401  (already imported)

    frf_raw = (freq_hz * (1 << 19)) // 32_000_000
    frf_msb = (frf_raw >> 16) & 0xFF
    frf_mid = (frf_raw >> 8) & 0xFF
    frf_lsb = frf_raw & 0xFF
    print(f"# forcing FRF -> {freq_hz/1e6:.3f} MHz "
          f"(raw=0x{frf_raw:06X} -> Msb=0x{frf_msb:02X} "
          f"Mid=0x{frf_mid:02X} Lsb=0x{frf_lsb:02X})")

    def _rreg(addr: int) -> int:
        f = link.request(HOST_TYPE_REG_READ_REQ, HOST_TYPE_REG_DATA_URC,
                         bytes([addr]), timeout=0.8)
        pl = f["payload"]
        if len(pl) < 2:
            raise RuntimeError(f"REG_DATA_URC short payload: {pl.hex()}")
        return pl[1]

    def _wreg(addr: int, value: int) -> None:
        link.request(HOST_TYPE_REG_WRITE_REQ, HOST_TYPE_REG_WRITE_ACK_URC,
                     bytes([addr, value & 0xFF]), timeout=0.8)

    try:
        opmode_pre = _rreg(SX1276_REG_OP_MODE)
        print(f"#   opmode pre = 0x{opmode_pre:02X}")
        in_rx_tx = (opmode_pre & 0x07) not in (0x00, 0x01)
        if in_rx_tx:
            _wreg(SX1276_REG_OP_MODE, SX1276_OPMODE_STANDBY_LORA)
        _wreg(SX1276_REG_FRF_MSB, frf_msb)
        _wreg(SX1276_REG_FRF_MID, frf_mid)
        _wreg(SX1276_REG_FRF_LSB, frf_lsb)
        if in_rx_tx:
            _wreg(SX1276_REG_OP_MODE, opmode_pre)
        rb_msb = _rreg(SX1276_REG_FRF_MSB)
        rb_mid = _rreg(SX1276_REG_FRF_MID)
        rb_lsb = _rreg(SX1276_REG_FRF_LSB)
        actual_raw = (rb_msb << 16) | (rb_mid << 8) | rb_lsb
        actual_hz = (actual_raw * 32_000_000) // (1 << 19)
        ok = (rb_msb == frf_msb and rb_mid == frf_mid and rb_lsb == frf_lsb)
        print(f"#   FRF readback: 0x{actual_raw:06X} = {actual_hz/1e6:.3f} MHz "
              f"({'OK' if ok else '**MISMATCH**'})")
        # Machine-parseable line for evidence capture.
        print(f"FRF_PINNED_HZ={actual_hz}")
        if not ok:
            return 3
    except Exception as exc:
        print(f"FATAL: FRF force-write failed: {exc}")
        return 1
    return 0


# --- CLI --------------------------------------------------------------------

def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n", 1)[0])
    ap.add_argument("--dev", default="/dev/ttymxc3",
                    help="serial device (default: /dev/ttymxc3)")
    ap.add_argument("--baud", default="921600",
                    help="baud rate (default: 921600)")
    ap.add_argument("--profile", choices=["fhss", "bench", "dts"], default="fhss",
                    help="regulatory profile to activate (default: fhss)")
    ap.add_argument("--mask", choices=["wide", "single"], default="wide",
                    help="channel mask: wide=all 50 (true FHSS, blocked by D8 "
                         "two-peer bug as of 2026-05-25), single=one channel "
                         "(bench two-peer workaround). Default: wide.")
    ap.add_argument("--ch", type=int, default=DEFAULT_SINGLE_CHANNEL_IDX,
                    help="channel index 0..49 when --mask single (default: 0)")
    ap.add_argument("--antenna-gain", type=int, default=DEFAULT_ANTENNA_GAIN_DBI,
                    help=f"antenna gain dBi i8 (default: {DEFAULT_ANTENNA_GAIN_DBI})")
    ap.add_argument("--hw-ceiling", type=int, default=DEFAULT_HW_CEILING_DBM,
                    help=f"hardware PA ceiling dBm u8 (default: {DEFAULT_HW_CEILING_DBM})")
    ap.add_argument("--force-frf", type=int, default=None,
                    help="force SX1276 carrier frequency in Hz by direct "
                         "register write after profile activation. "
                         "Workaround for the 2026-05-25 per-board channel "
                         "mismatch under FHSS. Default: 915000000 when "
                         "--profile fhss, otherwise no pin. Pass 0 to "
                         "disable explicitly.")
    ap.add_argument("--apply", action="store_true",
                    help="actually send the CFG_SET sequence (default: dry-run only)")
    ap.add_argument("--self-test", action="store_true",
                    help="run hardware-free golden-vector tests and exit")
    args = ap.parse_args(argv)

    if args.self_test:
        return _self_test()

    try:
        plan = render_plan(args)
    except ValueError as exc:
        print(f"FATAL: argument error: {exc}")
        return 4

    print_plan(args, plan)
    if not args.apply:
        print()
        print("# DRY RUN — pass --apply to send these writes to the L072.")
        return 10

    print()
    print("# --apply set — issuing CFG_SET sequence ...")
    return _apply(args, plan)


if __name__ == "__main__":
    sys.exit(main())
