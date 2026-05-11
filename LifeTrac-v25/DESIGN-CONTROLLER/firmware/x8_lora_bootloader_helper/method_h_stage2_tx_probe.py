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

import argparse
import struct
import sys
import time

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


HOST_TYPE_TX_FRAME_REQ = 0x10
HOST_TYPE_TX_DONE_URC = 0x90
HOST_TYPE_CFG_SET_REQ = 0x20
HOST_TYPE_CFG_OK_URC = 0xA0
HOST_TYPE_REG_READ_REQ = 0x30
HOST_TYPE_REG_WRITE_REQ = 0x31
HOST_TYPE_REG_DATA_URC = 0xB0
HOST_TYPE_REG_WRITE_ACK_URC = 0xB1

CFG_KEY_LBT_ENABLE = 0x03

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

    try:
        opm, _ = read_reg(link, SX1276_REG_OP_MODE, timeout=0.5)
        print(f"RX_LISTEN: RegOpMode(pre)=0x{opm:02X} "
              f"(expected 0x{SX1276_OPMODE_LORA_RXCONT:02X})")
    except Exception:
        pass

    # Signal ready BEFORE entering the listen loop so the orchestrator can
    # safely start the TX side.
    sys.stdout.write("__W1_10B_LISTEN_READY__\n")
    sys.stdout.flush()

    deadline = time.time() + window_s
    rx_count = 0
    real_faults = 0
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
          f"invariants_violated={len(invariants_violated)}")
    return 0


def run_tx_burst(link: HostLink, count: int = 100, inter_s: float = 0.2,
                 timeout: float = 5.0) -> int:
    """W1-10b TX-side burst. Sends `count` TX_FRAME_REQs with payload
    `W1-10b seq=NNNN <random-hex>` and prints one `__TX_DONE__ ...` line per
    successful TX_DONE_URC. Final summary on `__W1_10B_BURST_DONE__`.
    """
    import os as _os
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

    real_faults = 0
    tx_done_ok = 0
    tx_done_fail = 0
    tx_timeout = 0
    for i in range(count):
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
                 "rx_listen", "tx_burst"],
        help="probe mode (W1-9 default 'tx'; W1-9b 'regversion'/'fsk'/'opmode_walk'; "
             "W1-10 single-board 'rx'; W1-10b two-board 'rx_listen'/'tx_burst')",
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
    args = parser.parse_args(argv)

    print(f"MODE: {args.probe}")
    print(f"dev={args.dev} baud={args.baud}")

    try:
        link = HostLink(args.dev, args.baud)
    except Exception as exc:
        print(f"FATAL: link open failed: {exc}")
        return 2

    try:
        if args.probe == "tx":
            return run_tx_probe(link, args)
        if args.probe == "regversion":
            return run_regversion_burst(link, count=args.burst_count)
        if args.probe == "fsk":
            return run_fsk_stdby(link)
        if args.probe == "opmode_walk":
            return run_opmode_walk(link)
        if args.probe == "rx":
            return run_rx_liveness(link, window_s=args.rx_window)
        if args.probe == "rx_listen":
            return run_rx_listen(link, window_s=args.rx_window)
        if args.probe == "tx_burst":
            return run_tx_burst(link, count=args.tx_count,
                                inter_s=args.inter_cycle_s,
                                timeout=args.timeout)
        print(f"FATAL: unknown probe mode: {args.probe}")
        return 2
    finally:
        link.close()


if __name__ == "__main__":
    sys.exit(main())
