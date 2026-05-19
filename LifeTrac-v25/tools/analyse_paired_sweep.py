#!/usr/bin/env python3
"""Analyse paired walk_power sweep: join TX-side CSV with RX-side log.

RX log lines look like:
  __RX_FRAME__ idx=N rssi=DDD snr=DD len=LL timestamp_us=TS payload_hex=HEX

The payload contains the ASCII tag "WP s<step> p<dbm> i<idx>" injected by
run_walk_power. We parse those tags to bucket received frames by step.

Outputs a per-step JOIN CSV and a verdict line.
"""
import csv
import re
import sys
from pathlib import Path
from collections import defaultdict

def mean(xs):
    xs = list(xs)
    return sum(xs) / len(xs) if xs else 0.0

if len(sys.argv) != 4:
    print("usage: analyse_paired_sweep.py <tx_csv> <rx_log> <out_join_csv>")
    sys.exit(2)

tx_csv = Path(sys.argv[1])
rx_log = Path(sys.argv[2])
out_csv = Path(sys.argv[3])

# Parse RX log
rx_re = re.compile(
    r"__RX_FRAME__ idx=(\d+) rssi=(-?\d+) snr=(-?\d+) len=(\d+) "
    r"timestamp_us=(\d+) payload_hex=([0-9a-fA-F]+)"
)
tag_re = re.compile(r"WP s(\d+) p(\d+) i(\d+)")

by_step: "dict[int, list[dict]]" = defaultdict(list)
unparsed_payloads = 0
total_rx = 0
for line in rx_log.read_text(errors="replace").splitlines():
    m = rx_re.search(line)
    if not m:
        continue
    total_rx += 1
    idx, rssi, snr, plen, ts_us, payload_hex = m.groups()
    try:
        payload = bytes.fromhex(payload_hex)
    except ValueError:
        unparsed_payloads += 1
        continue
    # The TX payload tag is ASCII; the raw bytes may be off-by-one (leading 0x57=W, etc.)
    # Try decoding as latin-1 and searching for "WP s..."
    txt = payload.decode("latin-1", errors="replace")
    tag = tag_re.search(txt)
    if not tag:
        unparsed_payloads += 1
        continue
    step = int(tag.group(1))
    by_step[step].append(
        {"rssi": int(rssi), "snr": int(snr), "len": int(plen),
         "tx_idx": int(tag.group(3))}
    )

# Parse TX CSV
tx_rows = list(csv.DictReader(tx_csv.open()))

# Join + write
fields = [
    "step_idx", "power_dbm_requested", "tx_attempted", "tx_done_ok",
    "tx_aborted_airtime", "rx_received", "per_pct",
    "rssi_min", "rssi_mean", "rssi_max",
    "snr_min", "snr_mean", "snr_max",
]
with out_csv.open("w", newline="") as f:
    w = csv.DictWriter(f, fieldnames=fields)
    w.writeheader()
    print(f"{'step':>4}  {'dBm':>4}  {'tx_ok':>5}  {'rx':>4}  {'PER%':>6}  "
          f"{'rssi_mean':>10}  {'snr_mean':>9}")
    summary_per = []
    summary_rssi = []
    summary_snr = []
    for row in tx_rows:
        step = int(row["step_idx"])
        rxs = by_step.get(step, [])
        tx_ok = int(row["tx_done_ok"])
        rx_n = len(rxs)
        per = 100.0 * (1 - rx_n / tx_ok) if tx_ok else 0.0
        rssi_vals = [r["rssi"] for r in rxs]
        snr_vals = [r["snr"] for r in rxs]
        out = {
            "step_idx": step,
            "power_dbm_requested": int(row["power_dbm_requested"]),
            "tx_attempted": int(row["count_sent"]),
            "tx_done_ok": tx_ok,
            "tx_aborted_airtime": int(row["radio_tx_abort_airtime_delta"] or 0),
            "rx_received": rx_n,
            "per_pct": f"{per:.2f}",
            "rssi_min": min(rssi_vals) if rssi_vals else "",
            "rssi_mean": f"{mean(rssi_vals):.1f}" if rssi_vals else "",
            "rssi_max": max(rssi_vals) if rssi_vals else "",
            "snr_min": min(snr_vals) if snr_vals else "",
            "snr_mean": f"{mean(snr_vals):.1f}" if snr_vals else "",
            "snr_max": max(snr_vals) if snr_vals else "",
        }
        w.writerow(out)
        print(f"{step:>4}  {out['power_dbm_requested']:>4}  {tx_ok:>5}  "
              f"{rx_n:>4}  {per:>6.2f}  "
              f"{out['rssi_mean']:>10}  {out['snr_mean']:>9}")
        summary_per.append(per)
        if rssi_vals: summary_rssi.append(mean(rssi_vals))
        if snr_vals: summary_snr.append(mean(snr_vals))

print()
print(f"total RX parsed         : {total_rx}")
print(f"payloads with bad tag   : {unparsed_payloads}")
print(f"steps with rx>0         : {sum(1 for s,r in by_step.items() if r)}")
print(f"mean PER over all steps : {mean(summary_per):.2f}%")
print(f"RSSI delta (min..max)   : {min(summary_rssi):.1f} .. {max(summary_rssi):.1f} dBm")
print(f"SNR  delta (min..max)   : {min(summary_snr):.1f} .. {max(summary_snr):.1f} dB")
verdict = "OK" if mean(summary_per) < 10.0 and len(by_step) == len(tx_rows) else "WARN"
print(f"__PAIRED_SWEEP_VERDICT__={verdict}")
