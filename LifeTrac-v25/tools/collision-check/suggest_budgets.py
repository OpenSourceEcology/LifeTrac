#!/usr/bin/env python3
"""Suggest per-pair overlap budgets for collision_rules.json from a results file.

Usage: suggest_budgets.py out/collision_results.json [--margin 0.15] [--floor 50]

For every pair the largest overlap measured over the reachable poses is taken, a margin is
added and the value is rounded up to two significant digits. Pairs whose overlap is zero
everywhere get no entry (the default budget applies). Re-run this after a joint changes and
paste the output into collision_rules.json; the goal over time is for every entry to
disappear as the joints get real clearance.
"""
import argparse
import json
import math
import sys


def round_up_2sig(v: float) -> float:
    if v <= 0:
        return 0.0
    exp = math.floor(math.log10(v)) - 1
    step = 10 ** exp
    return math.ceil(v / step) * step


def suggest(results: dict, margin: float, floor: float) -> dict:
    worst: dict = {}
    for pose in results.get("poses", []):
        if not pose.get("reachable") or pose.get("informational"):
            continue
        for pair, vol in pose.get("overlaps_mm3", {}).items():
            if vol is None:
                continue
            worst[pair] = max(worst.get(pair, 0.0), float(vol))
    out = {}
    for pair, vol in sorted(worst.items()):
        if vol <= 0.0:
            continue
        budget = max(floor, round_up_2sig(vol * (1.0 + margin)))
        if budget > floor:
            out[pair] = budget
    return out


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("results", help="JSON written by collision_check.py --json")
    ap.add_argument("--margin", type=float, default=0.15, help="fraction added on top of the measured maximum")
    ap.add_argument("--floor", type=float, default=50.0, help="default budget; pairs below it get no entry")
    args = ap.parse_args(argv)
    with open(args.results) as f:
        results = json.load(f)
    budgets = suggest(results, args.margin, args.floor)
    print(json.dumps({"allowed_overlap_mm3": budgets}, indent=2))
    return 0


if __name__ == "__main__":
    sys.exit(main())
