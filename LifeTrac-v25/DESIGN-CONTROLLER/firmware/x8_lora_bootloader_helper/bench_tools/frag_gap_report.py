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

2026-10-24 (the FHSS round's leg 6): the gaps run from the first fragment
to the last, so they leave out the silence before and after them (leg 6:
gaps>3s=4 over a 115 s span, but 138.1 s before its first fragment and
about 56 s after its last). Two more lines print them: the head, from the
base's profile activation (rs12_leg_report.py's acquisition anchor) to the
first fragment, and the tail, from the last fragment to the base daemon's
stop (or its log's last line); with the params.txt clock offsets (harness
from 2026-10-24) the two clocks are comparable, and the head also counts
from the tractor's first txdone and the tail to its last. A leg with fewer
than two fragments still prints them, and still exits 1 without the
command-plane tally: bracket_deltas.py reads that tally as its log
cross-check, so such a leg stays UNVERIFIED there, as before. They need
tools/rs12_leg_report.py: the copy stage_boards.sh puts on the boards
prints as before.
"""
from __future__ import annotations

import argparse
import os
import re
import sys
from datetime import datetime, timezone
from pathlib import Path

_HERE = Path(__file__).resolve()
if len(_HERE.parents) > 3:                     # bench_tools -> helper -> firmware -> DC
    _TOOLS = str(_HERE.parents[3] / "tools")
    if _TOOLS not in sys.path:
        sys.path.insert(0, _TOOLS)
try:
    import rs12_leg_report as rep  # noqa: E402  (the head/tail lines, 2026-10-24)
except ImportError:
    # stage_boards.sh also stages this file to the boards (/work), where there is no
    # DC/tools: the gap report runs as before, without the head and tail lines
    rep = None

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


def _s(x: float) -> str:
    """seconds to 0.1 s, without a "-0.0" (a last fragment logged a few ms
    after the tractor's last txdone)"""
    return f"{round(x, 1) + 0.0:.1f}"


def _clock(t: float) -> str:
    return datetime.fromtimestamp(t, timezone.utc).strftime("%H:%M:%S.%f")[:-3]


def head_tail(rx: list[str], tx: list[str], params: dict) -> list[str]:
    """The ``head:`` and ``tail:`` lines (2026-10-24): the silence before the
    base's first fragment and after its last, which the gaps leave out.
    Board times are read as UTC (rs12_leg_report._log_time); the tractor's
    are put on the base clock with the params.txt clock offsets (board minus
    PC) when it has them. Nothing without rs12_leg_report (a board-staged copy)."""
    if rep is None:
        return []
    frags = [t for t in (rep._log_time(ln) for ln in rx if "frag_arrival" in ln)
             if t is not None]
    # the activation on any profile (rs12_leg_report's anchor: the first
    # timestamped line after the first REG_PROFILE=N OK line)
    i = next((n for n, ln in enumerate(rx) if rep.ACTIVATION_RE.search(ln)), None)
    act = None if i is None else next(
        (t for t in map(rep._log_time, rx[i + 1:]) if t is not None), None)
    offsets = rep.clock_offsets(params)
    sent = [t for t, _, _, _ in rep.fragment_events("\n".join(tx), rx=False)
            if t is not None]

    def to_base(t):
        """tractor board time -> base board time (both minus their PC offset)"""
        return t - offsets[0] + offsets[1]
    stop = next((rep._log_time(ln) for ln in reversed(rx) if "signal received" in ln), None)
    last_line = next((t for t in map(rep._log_time, reversed(rx)) if t is not None), None)
    first, last = (frags[0], frags[-1]) if frags else (None, None)

    if act is None:
        head = "head: n/a (no REG_PROFILE activation line in rx_daemon.log)"
    elif first is None:
        head = (f"head: no fragment in the {(last_line or act) - act:.1f} s the base log "
                f"covers after its profile activation ({_clock(act)} base clock)")
    else:
        head = (f"head: the first fragment came {first - act:.1f} s after the base's profile "
                f"activation ({_clock(act)} base clock, the first line after REG_PROFILE OK)")
        if offsets is not None and sent:
            head += (f"; {first - to_base(sent[0]):.1f} s after the tractor's first txdone "
                     f"(params.txt clock offsets)")
    if last is None:
        return [head]
    parts = []
    if offsets is not None and sent:
        parts.append(f"{_s(to_base(sent[-1]) - last)} s before the tractor's last txdone "
                     f"(params.txt clock offsets)")
    if stop is not None:
        parts.append(f"{stop - last:.1f} s before the base daemon's stop (its 'signal "
                     f"received' line)")
    elif last_line is not None:
        parts.append(f"{last_line - last:.1f} s before the base log's last line (no stop "
                     f"line: an older harness read the log while the daemon ran)")
    tail = "tail: the last fragment came " + "; ".join(parts) if parts else "tail: n/a"
    return [head, tail]


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("archive")
    ap.add_argument("--gap", type=float, default=3.0,
                    help="episode threshold in seconds (default 3.0)")
    a = ap.parse_args()
    rx = _lines(os.path.join(a.archive, "rx_daemon.log"))
    tx = _lines(os.path.join(a.archive, "tx_daemon.log"))
    params = (rep.params_dict("\n".join(_lines(os.path.join(a.archive, "params.txt"))))
              if rep else {})
    frag = [t for t in (_ts(l) for l in rx if "frag_arrival" in l) if t]
    sends = [t for t in (_ts(l) for l in rx
                         if "command TX opcode" in l and "OK (on air)" in l) if t]
    cmds = [t for t in (_ts(l) for l in tx if "LoRa cmd:" in l) if t]
    if len(frag) < 2:
        print("frag_arrival lines: %d (need -LogFragArrivals 1)" % len(frag))
        # 2026-10-24: a base that caught one fragment or none (an acquisition
        # failure) still gets its head and tail, but not the command-plane
        # tally: bracket_deltas.py reads that tally as its log cross-check, and
        # without it such a leg stays UNVERIFIED there (exit 1, STOP RULE
        # UNDETERMINED), as before; with it a base that heard nothing on an
        # FHSS leg scored as a clean image-only pass (0/0, review 2026-10-24)
        if rep and any("txdone_arrival" in l for l in tx):
            for line in head_tail(rx, tx, params):
                print(line)
        return 1
    gaps = [(frag[i] - frag[0], frag[i + 1] - frag[i])
            for i in range(len(frag) - 1)]
    big = [g for g in gaps if g[1] > a.gap]
    print("n_frag=%d span=%.0fs max_gap=%.1fs gaps>%.0fs=%d total_silent=%.1fs"
          % (len(frag), frag[-1] - frag[0], max(g[1] for g in gaps), a.gap,
             len(big), sum(g[1] for g in big)))
    print("episodes (start s from first frag, +length): "
          + (", ".join("%.1f(+%.1fs)" % g for g in big) or "NONE"))
    for line in head_tail(rx, tx, params):
        print(line)
    print("base sends on air=%d | tractor decoded cmds=%d" % (len(sends), len(cmds)))
    if cmds:
        print("tractor cmd offsets (s from its first): "
              + ", ".join("%.1f" % (c - cmds[0]) for c in cmds[:40]))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
