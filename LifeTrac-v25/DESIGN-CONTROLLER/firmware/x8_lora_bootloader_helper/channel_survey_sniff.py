#!/usr/bin/env python3
"""channel_survey_sniff.py — map the 902–928 MHz band for the RS-11.6 emitter.

Runs on a board inside the container (same shape as air_coupling_rssi_sniff):
parks the L072 in RXCONT at profile 2's modem settings, then steps the carrier
(FRF) across the band in fixed increments, dwelling on each channel long
enough to catch several ticks of the known ~7.08 s emitter, sampling raw
RegRssiValue throughout.

Why this works: the RS-11.6 leg-2 sniffs showed the emitter lands inside our
500 kHz window on essentially every tick, so a dwell of >= 4 ticks per
channel with ~15 Hz sampling either catches it (hot samples far above the
-114 dBm floor) or clears that channel with high confidence.

Output: one `SURVEY_CH {json}` line per channel, then `SURVEY_SUMMARY`.

Usage (in docker, mirroring the daemon container shape):
    python3 -u /work/channel_survey_sniff.py \
        --start-hz 902000000 --stop-hz 928000000 --step-hz 500000 \
        --dwell-s 30 --interval-s 0.05
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
    SX1276_REG_RSSI_VALUE,
    SX1276_OPMODE_LORA_RXCONT,
    read_reg, write_reg,
    drain_boot, drain_pending,
    configure_regulatory_profile_if_needed,
)

SX1276_REG_FRF_MSB = 0x06
SX1276_REG_FRF_MID = 0x07
SX1276_REG_FRF_LSB = 0x08
OPMODE_LORA_STANDBY = 0x81  # matches the daemons' logged 0x81 -> 0x85 dance

HOT_DBM = -75.0  # far above the ~-114 dBm floor; matches the hunt verdict cut


def raw_to_dbm(raw: int) -> int:
    # SX1276 datasheet 5.5.5, HF port: RSSI[dBm] = -157 + raw.
    return -157 + int(raw)


def set_frf(link: HostLink, freq_hz: int) -> None:
    """Retune the carrier. FRF may only change in SLEEP/STANDBY (ds 4.1.4)."""
    frf_raw = int(round(freq_hz * (1 << 19) / 32_000_000))
    write_reg(link, SX1276_REG_OP_MODE, OPMODE_LORA_STANDBY, timeout=0.5)
    write_reg(link, SX1276_REG_FRF_MSB, (frf_raw >> 16) & 0xFF, timeout=0.5)
    write_reg(link, SX1276_REG_FRF_MID, (frf_raw >> 8) & 0xFF, timeout=0.5)
    write_reg(link, SX1276_REG_FRF_LSB, frf_raw & 0xFF, timeout=0.5)
    write_reg(link, SX1276_REG_OP_MODE, SX1276_OPMODE_LORA_RXCONT, timeout=0.5)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--uart", default="/dev/ttymxc3")
    ap.add_argument("--baud", default="921600")
    ap.add_argument("--start-hz", type=int, default=902_000_000)
    ap.add_argument("--stop-hz", type=int, default=928_000_000)
    ap.add_argument("--step-hz", type=int, default=500_000)
    ap.add_argument("--dwell-s", type=float, default=30.0)
    ap.add_argument("--interval-s", type=float, default=0.05)
    args = ap.parse_args()

    meta = {"start_hz": args.start_hz, "stop_hz": args.stop_hz,
            "step_hz": args.step_hz, "dwell_s": args.dwell_s,
            "ts_start": time.time()}
    print("SURVEY_META " + json.dumps(meta), flush=True)

    link = HostLink(args.uart, args.baud)
    drain_boot(link, settle_s=0.25)
    try:
        link.request(HOST_TYPE_VER_REQ, HOST_TYPE_VER_URC, timeout=1.0)
    except Exception as exc:
        print("SURVEY_FAIL " + json.dumps({"stage": "ver", "err": str(exc)}),
              flush=True)
        return 2
    drain_pending(link, quiet_s=0.25, max_s=1.0)

    # Profile 2 gives the 500 kHz RX bandwidth the leg-2 measurements used,
    # so per-channel hot counts stay directly comparable.
    try:
        configure_regulatory_profile_if_needed(link, profile_id=2)
    except Exception as exc:
        print("SURVEY_FAIL " + json.dumps({"stage": "regprofile",
                                           "err": str(exc)}), flush=True)
        return 3

    hot_channels = []
    freq = args.start_hz
    while freq <= args.stop_hz:
        try:
            set_frf(link, freq)
        except Exception as exc:
            print("SURVEY_CH " + json.dumps(
                {"freq_hz": freq, "err": f"retune: {exc}"}), flush=True)
            freq += args.step_hz
            continue
        time.sleep(0.15)  # AGC settle

        n = 0
        hot = 0
        max_dbm = -200
        hot_ts = []
        t_end = time.time() + args.dwell_s
        while time.time() < t_end:
            t0 = time.time()
            try:
                raw, _ = read_reg(link, SX1276_REG_RSSI_VALUE, timeout=0.3)
            except Exception:
                time.sleep(args.interval_s)
                continue
            dbm = raw_to_dbm(raw)
            n += 1
            if dbm > max_dbm:
                max_dbm = dbm
            if dbm > HOT_DBM:
                hot += 1
                hot_ts.append(round(t0, 3))
            time.sleep(args.interval_s)

        rec = {"freq_hz": freq, "n": n, "hot": hot, "max_dbm": max_dbm,
               "hot_ts": hot_ts[:40]}
        print("SURVEY_CH " + json.dumps(rec), flush=True)
        if hot:
            hot_channels.append((freq, hot, max_dbm))
        freq += args.step_hz

    print("SURVEY_SUMMARY " + json.dumps(
        {"hot_channels": [{"freq_hz": f, "hot": h, "max_dbm": m}
                          for f, h, m in hot_channels]}), flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
