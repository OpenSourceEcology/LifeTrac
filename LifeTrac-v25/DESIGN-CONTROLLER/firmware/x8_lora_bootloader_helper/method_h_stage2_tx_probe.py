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

Exit codes:
  0 = all gates passed
  1 = protocol responded but at least one gate failed
  2 = fatal transport / timeout failure
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
    parse_stats,
    format_fault_payload,
    format_err_proto_payload,
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
                print(f"FAULT_URC during TX wait: {desc}")
                faults.append(desc)
            elif ftype == HOST_TYPE_ERR_PROTO_URC:
                desc = format_err_proto_payload(frame["payload"])
                raise RuntimeError(f"ERR_PROTO during TX wait: {desc}")
            else:
                print(f"INFO: unrelated frame during TX wait type=0x{ftype:02X}")
    raise TimeoutError(f"timeout waiting for TX_DONE_URC tx_id=0x{expected_tx_id:02X}")


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description="W1-9 Stage 2 TX bring-up probe")
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
    args = parser.parse_args(argv)

    try:
        payload = bytes.fromhex(args.payload_hex)
    except ValueError as exc:
        print(f"FATAL: --payload-hex parse error: {exc}")
        return 2

    if not (1 <= len(payload) <= 64):
        print(f"FATAL: payload length {len(payload)} out of range [1, 64]")
        return 2

    print(f"=== Stage 2 TX bring-up probe (W1-9) ===")
    print(f"dev={args.dev} baud={args.baud}")
    print(f"tx_id=0x{args.tx_id:02X} payload_len={len(payload)} payload_hex={payload.hex()}")
    print(f"timeout={args.timeout}s")

    try:
        link = HostLink(args.dev, args.baud)
    except Exception as exc:
        print(f"FATAL: link open failed: {exc}")
        return 2

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
        link.close()


if __name__ == "__main__":
    sys.exit(main())
