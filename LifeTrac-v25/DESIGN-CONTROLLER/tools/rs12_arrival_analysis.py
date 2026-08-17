"""rs12_arrival_analysis.py — timing shape of the silent L072 URC drop.

Consumes `frag_arrival:` lines (LIFETRAC_LOG_FRAG_ARRIVALS=1) from an
archive's rx_daemon.log. Each line carries TWO clocks:

    fw_us     — the L072's microsecond timestamp, stamped AT DEMODULATION
    wall (log prefix) — the host's clock, stamped AT URC DELIVERY

For a train missing exactly its penultimate fragment (idx N-2 absent,
N-3 and N-1 present) three hypotheses separate:

  A. PURE EMISSION LOSS: fw gap (N-3 -> N-1) ~ 2x nominal slot — the lost
     fragment was demodulated on schedule (rx_ok counted it), its URC
     simply never left the L072, and neighbours' delivery latency is
     normal.
  B. TX COMPRESSION: fw gap well under 2x nominal — the transmitter
     actually moved the last fragments together; reopens the TX theory.
  C. MAIN-LOOP STALL: neighbours' URC delivery latency (wall - fw_us)
     jumps around the drop — the L072 stopped emitting for a while and
     the pending URC was overwritten during the stall.

Usage:
    py -3 rs12_arrival_analysis.py <archive_dir>
"""

from __future__ import annotations

import argparse
import pathlib
import re
import statistics

LINE = re.compile(
    r"(\d{2}):(\d{2}):(\d{2}),(\d{3}) INFO image_rx_daemon: frag_arrival: "
    r"seq=(\d+) idx=(\d+) total=(\d+) fw_us=(\d+) len=(\d+)")


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("archive", type=pathlib.Path)
    args = ap.parse_args()

    rows = []
    for m in LINE.finditer((args.archive / "rx_daemon.log").read_text(
            encoding="utf-8", errors="replace")):
        h, mi, s, ms, seq, idx, total, fw, ln = map(int, m.groups())
        wall_ms = ((h * 3600 + mi * 60 + s) * 1000) + ms
        rows.append({"wall": wall_ms, "seq": seq, "idx": idx,
                     "total": total, "fw": fw})
    if not rows:
        print("no frag_arrival lines — was -LogFragArrivals 1 set?")
        return 1
    print(f"fragments logged: {len(rows)}")

    # split into trains on seq change (seq wraps at 256; consecutive
    # fragments of one train share it)
    trains = []
    curr = [rows[0]]
    for r in rows[1:]:
        if r["seq"] == curr[-1]["seq"] and r["idx"] > curr[-1]["idx"]:
            curr.append(r)
        else:
            trains.append(curr)
            curr = [r]
    trains.append(curr)
    print(f"trains reconstructed: {len(trains)}")

    # nominal slot from contiguous pairs (fw clock)
    slots = []
    lat = []  # delivery latency proxy per fragment: wall - fw/1000
    for t in trains:
        for a, b in zip(t, t[1:]):
            if b["idx"] == a["idx"] + 1:
                d = (b["fw"] - a["fw"]) & 0xFFFFFFFF
                if 0 < d < 1_000_000:
                    slots.append(d)
    nominal = statistics.median(slots) / 1000.0
    print(f"nominal inter-fragment slot (fw clock): {nominal:.1f} ms "
          f"(n={len(slots)})")

    # trains missing exactly the penultimate with both neighbours present
    drop_gaps = []       # fw gap idx N-3 -> N-1, in slots
    drop_lat_jump = []   # latency(N-1) - latency(N-3), ms
    clean_lat_jump = []  # same metric across N-3 -> N-1 in clean trains
    for t in trains:
        total = t[0]["total"]
        pen = total - 2
        have = {r["idx"]: r for r in t}
        if pen - 1 in have and pen + 1 in have:
            a, b = have[pen - 1], have[pen + 1]
            gap_ms = ((b["fw"] - a["fw"]) & 0xFFFFFFFF) / 1000.0
            lat_a = a["wall"] - a["fw"] / 1000.0
            lat_b = b["wall"] - b["fw"] / 1000.0
            if pen not in have:
                drop_gaps.append(gap_ms / nominal)
                drop_lat_jump.append(lat_b - lat_a)
            else:
                clean_lat_jump.append(lat_b - lat_a)

    print(f"\npenultimate-drop trains (neighbours present): {len(drop_gaps)}")
    if drop_gaps:
        print(f"  fw gap (N-3 -> N-1) in units of the nominal slot:")
        print(f"    med={statistics.median(drop_gaps):.3f}  "
              f"min={min(drop_gaps):.3f}  max={max(drop_gaps):.3f}")
        print(f"    (2.0 = the lost fragment's slot passed exactly on "
              f"schedule)")
        print(f"  URC delivery-latency jump across the drop (ms):")
        print(f"    drop trains : med={statistics.median(drop_lat_jump):+.1f} "
              f" min={min(drop_lat_jump):+.1f}  max={max(drop_lat_jump):+.1f}")
        if clean_lat_jump:
            print(f"    clean trains: med={statistics.median(clean_lat_jump):+.1f}"
                  f"  (n={len(clean_lat_jump)}) — the control")
        near2 = sum(1 for g in drop_gaps if abs(g - 2.0) < 0.15)
        print(f"\n  VERDICT INPUTS: {near2}/{len(drop_gaps)} drops have the "
              f"missing slot passing on schedule (gap within 2.0+/-0.15);")
        print(f"  a latency jump in drop trains >> clean trains indicates a "
              f"stall; neither indicates pure emission loss.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
