"""rs12_ab_summary.py — aggregate the RS-12 depth A/B into a verdict.

Consumes the legs.json + per-leg stats snapshots written by
run_rs12_depth_ab.ps1, runs each leg through the same extraction as
rs12_leg_report, and reports per-arm means with spread — plus an
exact permutation test, which is the honest significance test at n=3
(20 distinct labelings of 6 legs into two arms of 3; no distributional
assumptions, and it cannot manufacture significance the data lacks:
the smallest achievable two-sided p is 2/20 = 0.10).

Usage:
    py -3 rs12_ab_summary.py <bench-evidence/RS_12_depth_ab_YYYY-MM-DD>
"""

from __future__ import annotations

import argparse
import collections
import itertools
import json
import pathlib
import re
import statistics


def parse_stats(p: pathlib.Path) -> dict[str, int]:
    out: dict[str, int] = {}
    if not p.is_file():
        return out
    for m in re.finditer(r"(radio_\w+)=(\d+)",
                         p.read_text(encoding="utf-8", errors="replace")):
        out.setdefault(m.group(1), int(m.group(2)))
    return out


def leg_metrics(archive: pathlib.Path, pre: pathlib.Path,
                post: pathlib.Path) -> dict | None:
    rxp = archive / "rx_daemon.log"
    txp = archive / "tx_daemon.log"
    if not rxp.is_file() or not txp.is_file():
        return None
    rx = rxp.read_text(encoding="utf-8", errors="replace")
    tx = txp.read_text(encoding="utf-8", errors="replace")
    sent = int(re.findall(r"frags_ok=(\d+)", tx)[-1])
    rcvd = int(re.findall(r"rx_frames=(\d+)", rx)[-1])

    idx: collections.Counter = collections.Counter()
    for m in re.finditer(r"lost_frag_idx: n=\d+.*?\| top ([0-9: ]+)", rx):
        for pair in m.group(1).split():
            i, c = pair.split(":")
            idx[int(i)] += int(c)
    tot = sum(idx.values())
    pen_share = 100.0 * idx.get(11, 0) / tot if tot else float("nan")

    a, b = parse_stats(pre), parse_stats(post)
    fw_drop = (b.get("radio_rx_ok", 0) - a.get("radio_rx_ok", 0) - rcvd
               if a and b else None)

    return {
        "offered": sent,
        "loss_pct": 100.0 * (sent - rcvd) / sent,
        "pen_share": pen_share,
        "fw_drop": fw_drop,
        "goodput": float(re.findall(r"goodput=([\d.]+)", tx)[-1]),
    }


def perm_test(a: list[float], b: list[float]) -> float:
    """Exact two-sided permutation p on the difference of means."""
    pool = a + b
    obs = abs(statistics.mean(a) - statistics.mean(b))
    n = len(a)
    hits = tot = 0
    for combo in itertools.combinations(range(len(pool)), n):
        g1 = [pool[i] for i in combo]
        g2 = [pool[i] for i in range(len(pool)) if i not in combo]
        tot += 1
        if abs(statistics.mean(g1) - statistics.mean(g2)) >= obs - 1e-12:
            hits += 1
    return hits / tot


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("abdir", type=pathlib.Path)
    args = ap.parse_args()

    legs = json.loads((args.abdir / "legs.json").read_text(encoding="utf-8-sig"))
    if isinstance(legs, dict):
        legs = [legs]

    # Arms: legs.json entries carry either "arm" (string label) or the
    # legacy "depth" int. Exactly two distinct arms are compared.
    arms: dict[str, list[dict]] = {}
    print(f"{'leg':<16}{'depth':>6}{'loss%':>8}{'pen%':>7}"
          f"{'fwdrop':>8}{'offered':>9}{'goodput':>9}")
    for leg in legs:
        arch = pathlib.Path(leg["archive"])
        m = leg_metrics(arch,
                        args.abdir / f"stats_{leg['tag']}_pre.txt",
                        args.abdir / f"stats_{leg['tag']}_post.txt")
        if not m:
            print(f"{leg['tag']:<16}{leg['depth']:>6}   (archive unreadable)")
            continue
        arm = str(leg.get("arm", leg.get("depth")))
        arms.setdefault(arm, []).append(m)
        print(f"{leg['tag']:<16}{arm:>6}{m['loss_pct']:>8.1f}"
              f"{m['pen_share']:>7.0f}"
              f"{(m['fw_drop'] if m['fw_drop'] is not None else -1):>8}"
              f"{m['offered']:>9}{m['goodput']:>9.0f}")

    print()
    for metric, label, lower_better in (("loss_pct", "loss %", True),
                                        ("pen_share", "penultimate %", True),
                                        ("fw_drop", "firmware drops", True),
                                        ("goodput", "goodput B/s", False)):
        names = sorted(arms.keys())
        if len(names) != 2:
            continue
        na, nb = names
        d2 = [x[metric] for x in arms[na] if x[metric] is not None]
        d1 = [x[metric] for x in arms[nb] if x[metric] is not None]
        if len(d2) < 2 or len(d1) < 2:
            continue
        m2, m1 = statistics.mean(d2), statistics.mean(d1)
        s2 = statistics.stdev(d2) if len(d2) > 1 else 0.0
        s1 = statistics.stdev(d1) if len(d1) > 1 else 0.0
        p = perm_test(d2, d1)
        better = (nb if ((m1 < m2) == lower_better) else na)
        print(f"{label:<16} {na}={m2:7.1f} +/-{s2:5.1f}   "
              f"{nb}={m1:7.1f} +/-{s1:5.1f}   "
              f"delta={m1 - m2:+7.1f}  perm p={p:.2f}  favours {better}")

    print("\nNote: at n=3/arm the exact permutation floor is p=0.10, so p=0.10"
          "\nmeans 'the cleanest separation this design can produce', not"
          "\n'weakly significant'. Read alongside the effect size.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
