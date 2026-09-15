#!/usr/bin/env python3
"""frag_gap_report.py -- lock-loss episodes from a harness archive.

The instrument that caught every RS-12.14/12.15 break: the base's fragment
arrival timeline (rx_daemon.log `frag_arrival` lines, harness flag
-LogFragArrivals 1). A follower that has lost its FHSS lock goes silent for
20-50 s; healthy links never gap more than ~1 s between fragments.

    py -3 frag_gap_report.py <archive_dir> [--gap 3.0]

Prints fragments seen, span, max gap, the count and total of gaps over the
threshold, the episode lengths, plus the command-plane tally (base sends on
air from rx_daemon.log, commands the tractor decoded from tx_daemon.log).
Timestamps are parsed per file; the base and tractor clocks are NOT
comparable, so offsets are reported per node only.
"""
from __future__ import annotations

import argparse
import os
import re
from datetime import datetime

_TS = re.compile(r"﻿?(\S+ \S+)")


def _ts(line: str) -> float | None:
    m = _TS.match(line)
    if not m:
        return None
    try:
        return datetime.strptime(m.group(1).lstrip("﻿"),
                                 "%Y-%m-%d %H:%M:%S,%f").timestamp()
    except ValueError:
        return None


def _lines(path: str):
    if not os.path.exists(path):
        return []
    with open(path, encoding="utf-8", errors="replace") as fh:
        return fh.read().splitlines()


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("archive")
    ap.add_argument("--gap", type=float, default=3.0,
                    help="episode threshold in seconds (default 3.0)")
    a = ap.parse_args()
    rx = _lines(os.path.join(a.archive, "rx_daemon.log"))
    tx = _lines(os.path.join(a.archive, "tx_daemon.log"))
    frag = [t for t in (_ts(l) for l in rx if "frag_arrival" in l) if t]
    sends = [t for t in (_ts(l) for l in rx
                         if "command TX opcode" in l and "OK (on air)" in l) if t]
    cmds = [t for t in (_ts(l) for l in tx if "LoRa cmd:" in l) if t]
    if len(frag) < 2:
        print("frag_arrival lines: %d (need -LogFragArrivals 1)" % len(frag))
        return 1
    gaps = [(frag[i] - frag[0], frag[i + 1] - frag[i])
            for i in range(len(frag) - 1)]
    big = [g for g in gaps if g[1] > a.gap]
    print("n_frag=%d span=%.0fs max_gap=%.1fs gaps>%.0fs=%d total_silent=%.1fs"
          % (len(frag), frag[-1] - frag[0], max(g[1] for g in gaps), a.gap,
             len(big), sum(g[1] for g in big)))
    print("episodes (start s from first frag, +length): "
          + (", ".join("%.1f(+%.1fs)" % g for g in big) or "NONE"))
    print("base sends on air=%d | tractor decoded cmds=%d" % (len(sends), len(cmds)))
    if cmds:
        print("tractor cmd offsets (s from its first): "
              + ", ".join("%.1f" % (c - cmds[0]) for c in cmds[:40]))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
