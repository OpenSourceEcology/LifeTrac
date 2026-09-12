#!/usr/bin/env python3
"""rs12_deaf_join.py — are lost fragments coincident with base command TX?

RS-12 flash session (2026-09-12) showed the two instrumented firmware
URC-path sites (`rx_urc_lost`, `rx_pretx_drained`) do not carry the loss.
The pre-registered fallback is RF-level: the base radio is transmitting
(or re-arming after a transmit) when the fragment arrives, so it is
never demodulated and no counter moves. This script tests that on an
archive: for every fragment the reassembler never saw, estimate when it
should have arrived (neighbour arrival +/- the leg's pacing) and measure
the time to the nearest base `command TX ... OK (on air)` line. The same
statistic over the fragments that DID arrive is the baseline. Both use
the rx daemon's wall clock, so no cross-clock alignment is needed.

Usage: rs12_deaf_join.py <archive_dir> [--window-ms 150]
"""
import argparse
import bisect
import pathlib
import re
import statistics
import sys
from datetime import datetime

TS = r"(\d{4}-\d\d-\d\d \d\d:\d\d:\d\d,\d{3})"
RX_FRAG = re.compile(TS + r" INFO image_rx_daemon: frag_arrival: seq=(\d+) idx=(\d+) total=(\d+) fw_us=(\d+)")
TX_DONE = re.compile(TS + r" INFO image_tx_daemon: txdone_arrival: seq=(\d+) idx=(\d+) status=(\d+)")
RX_CMD = re.compile(TS + r" INFO image_rx_daemon: command TX opcode=(0x[0-9a-fA-F]+) copy=(\d+)/(\d+) OK \(on air\)")


def t_ms(s: str) -> float:
    return datetime.strptime(s, "%Y-%m-%d %H:%M:%S,%f").timestamp() * 1000.0


def nearest(sorted_vals, x):
    i = bisect.bisect_left(sorted_vals, x)
    best = None
    for j in (i - 1, i):
        if 0 <= j < len(sorted_vals):
            d = sorted_vals[j] - x
            if best is None or abs(d) < abs(best):
                best = d
    return best


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("archive", type=pathlib.Path)
    ap.add_argument("--window-ms", type=float, default=150.0)
    a = ap.parse_args()
    text = (a.archive / "rx_daemon.log").read_text(encoding="utf-8", errors="replace")

    frags = {}  # (seq, epoch) -> {idx: t_ms}; the wire seq wraps, so a
    totals = {}  # train is re-keyed when its seq reappears after > 3 s
    last_seen = {}
    epoch = {}
    for m in RX_FRAG.finditer(text):
        t, seq, idx, total = t_ms(m.group(1)), int(m.group(2)), int(m.group(3)), int(m.group(4))
        if seq in last_seen and (t - last_seen[seq]) > 3000.0:
            epoch[seq] = epoch.get(seq, 0) + 1
        last_seen[seq] = t
        key = (seq, epoch.get(seq, 0))
        frags.setdefault(key, {})
        frags[key].setdefault(idx, t)  # first arrival wins (duplicates ignored)
        totals[key] = total
    cmds = sorted(t_ms(m.group(1)) for m in RX_CMD.finditer(text))
    txdone = {}  # (seq, idx) -> list of (t_ms, status) from the tractor tx log, if archived
    txp = a.archive / "tx_daemon.log"
    if txp.exists():
        for m in TX_DONE.finditer(txp.read_text(encoding="utf-8", errors="replace")):
            txdone.setdefault((int(m.group(2)), int(m.group(3))), []).append((t_ms(m.group(1)), int(m.group(4))))
    if not frags or not cmds:
        print(f"frags={sum(len(v) for v in frags.values())} cmds={len(cmds)} — nothing to join")
        return 2

    # pacing: consecutive received pairs, excluding the pair into the final fragment
    gaps = []
    for seq, d in frags.items():
        T = totals[seq]
        for i in range(T - 2):
            if i in d and (i + 1) in d:
                gaps.append(d[i + 1] - d[i])
    pacing = statistics.median(gaps) if gaps else 117.0

    lost = []      # (seq, idx, T, t_expected)
    received = []  # (seq, idx, T, t)
    for seq, d in frags.items():
        T = totals[seq]
        if len(d) < 1:
            continue  # nothing arrived for this key (cannot happen)
        for i in range(T):
            if i in d:
                received.append((seq, i, T, d[i]))
                continue
            if (i - 1) in d:
                te = d[i - 1] + pacing
            elif (i + 1) in d:
                te = d[i + 1] - pacing
            else:
                continue
            lost.append((seq, i, T, te))

    W = a.window_ms

    def stats(items):
        ds = [nearest(cmds, t) for (_, _, _, t) in items]
        ds = [d for d in ds if d is not None]
        n = len(ds)
        within = sum(1 for d in ds if abs(d) <= W)
        return n, within, ds

    n_l, w_l, ds_l = stats(lost)
    n_r, w_r, ds_r = stats(received)
    pen_lost = [x for x in lost if x[1] == x[2] - 2]
    n_p, w_p, ds_p = stats(pen_lost)

    print(f"archive={a.archive.name}  cmds={len(cmds)}  trains={len(frags)}  pacing_ms={pacing:.1f}  window=+/-{W:.0f} ms")
    print(f"received fragments : {n_r:5d}  within window of a base TX: {w_r:4d} ({100.0 * w_r / max(n_r, 1):.1f} %)")
    print(f"lost fragments     : {n_l:5d}  within window of a base TX: {w_l:4d} ({100.0 * w_l / max(n_l, 1):.1f} %)")
    print(f"lost penultimates  : {n_p:5d}  within window of a base TX: {w_p:4d} ({100.0 * w_p / max(n_p, 1):.1f} %)")
    if n_l and n_r:
        ratio = (w_l / n_l) / max(w_r / n_r, 1e-9)
        print(f"enrichment lost/received = {ratio:.2f}x")

    def hist(ds, label):
        edges = list(range(-500, 501, 50))
        counts = [0] * (len(edges) - 1)
        beyond = 0
        for d in ds:
            if d < edges[0] or d >= edges[-1]:
                beyond += 1
                continue
            k = int((d - edges[0]) // 50)
            counts[k] += 1
        print(f"  {label} dt = t(base TX) - t(fragment) histogram, 50 ms bins from -500 ms (beyond +/-500: {beyond}):")
        print("   " + " ".join(f"{e:+4d}" for e in edges[:-1]))
        print("   " + " ".join(f"{c:4d}" for c in counts))

    if txdone:
        confirmed = sum(1 for (seq, i, T, te) in lost if any(st == 0 for (_, st) in txdone.get((seq[0], i), [])))
        unseen = sum(1 for (seq, i, T, te) in lost if (seq[0], i) not in txdone)
        print(f"tractor tx log     : of {len(lost)} lost fragments, {confirmed} have a TX_DONE status=0 at the tractor, "
              f"{unseen} have no TX_DONE line at all (tx log keyed by wire seq; wrap collisions possible)")
    if pen_lost:
        print("  lost penultimates (seq, idx/T, dt_ms to nearest base TX):")
        for (seq, i, T, te) in pen_lost:
            d = nearest(cmds, te)
            print(f"    seq={seq} idx={i}/{T} dt={d:+.0f}" if d is not None else f"    seq={seq} idx={i}/{T} dt=n/a")
    hist(ds_l, "LOST")
    hist(ds_p, "LOST PENULTIMATE")
    # received baseline as fractions in the same bins
    if ds_r:
        edges = list(range(-500, 501, 50))
        counts = [0] * (len(edges) - 1)
        for d in ds_r:
            if edges[0] <= d < edges[-1]:
                counts[int((d - edges[0]) // 50)] += 1
        print("  RECEIVED baseline, same bins (% of received):")
        print("   " + " ".join(f"{100.0 * c / len(ds_r):4.1f}" for c in counts))
    return 0


if __name__ == "__main__":
    sys.exit(main())
