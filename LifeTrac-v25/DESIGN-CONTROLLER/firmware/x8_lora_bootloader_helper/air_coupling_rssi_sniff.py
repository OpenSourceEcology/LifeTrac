#!/usr/bin/env python3
"""air_coupling_rssi_sniff.py

Falsification probe for 2026-05-25 "TX 4/4 ok but rx_frames=0 with perfect
modem parity" mystery.

Strategy:
- Put the RX peer's SX1276 into RXCONTINUOUS at the same FRF/BW/SF the daemons
  use (REG_PROFILE override honored via env, like the daemons).
- Sample RegRssiValue (0x1B), RegPktRssi (0x1A), RegIrqFlags (0x12), RegModemStat
  (0x18), and RegOpMode (0x01) periodically over a window.
- Stream one JSON line per sample to stdout prefixed `RSSI_SAMPLE `.
- Emit a final summary line `RSSI_SUMMARY ` with min/median/max RSSI and the
  count of IRQ events that fired during the window.

If the RX RSSI elevates noticeably during a concurrent TX burst on the peer,
the air link is fine and the bug is somewhere in RX-frame demod / IRQ handling
on the L072 firmware. If RSSI stays flat at the noise floor (~ -120 dBm) the
entire window, the bug is RF/antenna and no amount of host-side change will
help.

Usage in docker (mirror the daemon container shape):
    python3 -u /work/air_coupling_rssi_sniff.py \\
        --duration-s 20 --interval-s 0.25 --role rx
"""
from __future__ import annotations

import argparse
import json
import sys
import time

sys.path.insert(0, "/work")

from method_h_stage2_tx_probe_v2 import (  # type: ignore
    HostLink,
    HOST_TYPE_VER_REQ, HOST_TYPE_VER_URC,
    SX1276_REG_OP_MODE,
    SX1276_REG_IRQ_FLAGS,
    SX1276_REG_PKT_RSSI,
    SX1276_REG_RSSI_VALUE,
    SX1276_OPMODE_LORA_RXCONT,
    read_reg, write_reg,
    drain_boot, drain_pending,
    configure_regulatory_profile_if_needed,
)

REG_MODEM_STAT = 0x18

# SX1276 datasheet 5.5.5: RegRssiValue is the raw RSSI of the channel.
# RSSI_dBm = -157 + raw  for HF band (>= 779 MHz). 915 MHz is HF.
RSSI_OFFSET = -157


def raw_to_dbm(raw: int) -> int:
    return RSSI_OFFSET + (raw & 0xFF)


def median(xs):
    if not xs:
        return None
    s = sorted(xs)
    n = len(s)
    if n % 2:
        return s[n // 2]
    return (s[n // 2 - 1] + s[n // 2]) / 2.0


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--uart", default="/dev/ttymxc3")
    ap.add_argument("--baud", default="921600")
    ap.add_argument("--role", choices=["rx"], default="rx",
                    help="only 'rx' is supported in this probe; TX side uses "
                         "the existing method_h_stage2_tx_probe_v2 tx_burst.")
    ap.add_argument("--duration-s", type=float, default=20.0)
    ap.add_argument("--interval-s", type=float, default=0.25)
    args = ap.parse_args()

    out_meta = {"role": args.role, "uart": args.uart, "baud": args.baud,
                "duration_s": args.duration_s, "interval_s": args.interval_s,
                "ts_start": time.time()}
    print("RSSI_META " + json.dumps(out_meta), flush=True)

    link = HostLink(args.uart, args.baud)
    try:
        drain_boot(link, settle_s=0.25)
        try:
            link.request(HOST_TYPE_VER_REQ, HOST_TYPE_VER_URC, timeout=1.0)
        except Exception as exc:
            print("RSSI_FAIL " + json.dumps({"stage": "ver", "err": str(exc)}),
                  flush=True)
            return 2
        drain_pending(link, quiet_s=0.25, max_s=1.0)

        try:
            configure_regulatory_profile_if_needed(link)
        except Exception as exc:
            print("RSSI_FAIL " + json.dumps({"stage": "regprofile", "err": str(exc)}),
                  flush=True)
            return 3

        # Enter RXCONTINUOUS.
        try:
            write_reg(link, SX1276_REG_OP_MODE, SX1276_OPMODE_LORA_RXCONT,
                      timeout=0.5)
        except Exception as exc:
            print("RSSI_FAIL " + json.dumps({"stage": "rxcont", "err": str(exc)}),
                  flush=True)
            return 4

        # Settle a moment so AGC/AFC can grab the channel.
        time.sleep(0.2)

        rssi_dbm_samples = []
        pkt_rssi_dbm_seen = []
        irq_flags_or = 0
        irq_events = 0
        opmodes_seen = set()
        n_samples = 0
        t_end = time.time() + args.duration_s

        # Print ready marker for orchestrator sync.
        print("__RSSI_SNIFF_READY__", flush=True)

        while time.time() < t_end:
            t0 = time.time()
            try:
                rssi_raw, _ = read_reg(link, SX1276_REG_RSSI_VALUE, timeout=0.3)
                pkt_raw, _ = read_reg(link, SX1276_REG_PKT_RSSI, timeout=0.3)
                irq_raw, _ = read_reg(link, SX1276_REG_IRQ_FLAGS, timeout=0.3)
                stat_raw, _ = read_reg(link, REG_MODEM_STAT, timeout=0.3)
                opm_raw, _ = read_reg(link, SX1276_REG_OP_MODE, timeout=0.3)
            except Exception as exc:
                print("RSSI_READ_ERR " + json.dumps({"ts": t0, "err": str(exc)}),
                      flush=True)
                # don't break — try next round
                time.sleep(args.interval_s)
                continue

            rssi_dbm = raw_to_dbm(int(rssi_raw))
            pkt_dbm = raw_to_dbm(int(pkt_raw))
            rssi_dbm_samples.append(rssi_dbm)
            if int(pkt_raw) != 0:
                pkt_rssi_dbm_seen.append(pkt_dbm)
            irq_flags_or |= int(irq_raw)
            if int(irq_raw) != 0:
                irq_events += 1
                # Clear flags by writing 0xFF (SX1276 W1C semantics).
                try:
                    write_reg(link, SX1276_REG_IRQ_FLAGS, 0xFF, timeout=0.3)
                except Exception:
                    pass
            opmodes_seen.add(int(opm_raw))
            n_samples += 1

            sample = {
                "ts": t0,
                "rssi_raw": int(rssi_raw),
                "rssi_dbm": rssi_dbm,
                "pkt_rssi_raw": int(pkt_raw),
                "pkt_rssi_dbm": pkt_dbm,
                "irq_flags": int(irq_raw),
                "modem_stat": int(stat_raw),
                "opmode": int(opm_raw),
            }
            print("RSSI_SAMPLE " + json.dumps(sample), flush=True)

            elapsed = time.time() - t0
            sleep_left = args.interval_s - elapsed
            if sleep_left > 0:
                time.sleep(sleep_left)

        summary = {
            "n_samples": n_samples,
            "rssi_min_dbm": min(rssi_dbm_samples) if rssi_dbm_samples else None,
            "rssi_max_dbm": max(rssi_dbm_samples) if rssi_dbm_samples else None,
            "rssi_median_dbm": median(rssi_dbm_samples),
            "rssi_p90_dbm": (sorted(rssi_dbm_samples)[int(0.9 * len(rssi_dbm_samples))]
                             if rssi_dbm_samples else None),
            "pkt_rssi_seen_count": len(pkt_rssi_dbm_seen),
            "pkt_rssi_max_dbm": max(pkt_rssi_dbm_seen) if pkt_rssi_dbm_seen else None,
            "irq_flags_or": irq_flags_or,
            "irq_flags_or_hex": "0x%02X" % irq_flags_or,
            "irq_events": irq_events,
            "opmodes_seen_hex": sorted(["0x%02X" % v for v in opmodes_seen]),
            "ts_end": time.time(),
        }
        print("RSSI_SUMMARY " + json.dumps(summary), flush=True)
        return 0
    finally:
        try:
            link.close()
        except Exception:
            pass


if __name__ == "__main__":
    sys.exit(main())
