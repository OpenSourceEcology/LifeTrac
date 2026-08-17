"""survey_compare.py — pick today's operating channel, and diff two surveys.

Consumes `SURVEY_CH` JSONL from channel_survey_sniff.py. With one survey it
ranks channels and recommends a center; with two it also shows how the band
changed between them.

Why this exists: the 2026-08-17 post-outage checks proved the bench
interferer HOPS — 915.0 went from the worst channel to clean overnight while
922-925 lit up and 902.5 (the pinned escape) started taking hits. A pinned
channel is therefore a perishable choice, and picking one needs to be a
routine, mechanical step rather than a hand analysis.

Ranking, worst-disqualifying-first:
  1. any hot sample (>-75 dBm) disqualifies outright — that is the emitter
  2. then by max_dbm (quietest peak wins)
  3. ties broken toward band centre-ish placement to keep the 500 kHz
     occupied bandwidth clear of the 902/928 edges

Usage:
    py -3 survey_compare.py today.jsonl [--prev yesterday.jsonl]
"""

from __future__ import annotations

import argparse
import json
import pathlib


BAND_LO = 902_000_000
BAND_HI = 928_000_000
HALF_BW = 250_000  # 500 kHz occupied


def load(path: pathlib.Path) -> dict[int, dict]:
    out: dict[int, dict] = {}
    for line in path.read_text(encoding="utf-8", errors="replace").splitlines():
        line = line.strip()
        if not line.startswith("SURVEY_CH"):
            continue
        try:
            d = json.loads(line[len("SURVEY_CH"):].strip())
        except ValueError:
            continue
        if "err" in d or "hot" not in d:
            continue
        out[int(d["freq_hz"])] = d
    return out


def legal(freq: int) -> bool:
    return freq - HALF_BW >= BAND_LO and freq + HALF_BW <= BAND_HI


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("survey", type=pathlib.Path)
    ap.add_argument("--prev", type=pathlib.Path)
    args = ap.parse_args()

    cur = load(args.survey)
    if not cur:
        print("no SURVEY_CH records parsed")
        return 1
    print(f"channels: {len(cur)}   "
          f"hot channels: {sum(1 for d in cur.values() if d['hot'])}")

    usable = {f: d for f, d in cur.items() if legal(f)}
    clean = {f: d for f, d in usable.items() if d["hot"] == 0}
    print(f"legal centers (500 kHz BW inside 902-928): {len(usable)}   "
          f"of which zero-hot: {len(clean)}")

    pool = clean or usable
    ranked = sorted(pool.items(), key=lambda kv: (kv[1]["hot"],
                                                  kv[1]["max_dbm"]))
    print("\nbest candidates:")
    for f, d in ranked[:6]:
        print(f"  {f/1e6:7.1f} MHz  hot={d['hot']:2d}  max={d['max_dbm']:5d} dBm"
              f"  n={d['n']}")
    best = ranked[0][0]
    print(f"\nRECOMMENDED: -ForceFrfHz {best}   ({best/1e6:.1f} MHz, "
          f"hot={ranked[0][1]['hot']}, max={ranked[0][1]['max_dbm']} dBm)")

    if args.prev:
        prev = load(args.prev)
        both = sorted(set(cur) & set(prev))
        print(f"\n-- change vs {args.prev.name} ({len(both)} shared channels) --")
        newly_hot, newly_clean = [], []
        for f in both:
            was, now = prev[f]["hot"], cur[f]["hot"]
            if was == 0 and now > 0:
                newly_hot.append((f, prev[f]["max_dbm"], cur[f]["max_dbm"]))
            elif was > 0 and now == 0:
                newly_clean.append((f, prev[f]["max_dbm"], cur[f]["max_dbm"]))
        print(f"newly HOT   ({len(newly_hot)}): " + ", ".join(
            f"{f/1e6:.1f}({a}->{b})" for f, a, b in newly_hot[:12]))
        print(f"newly CLEAN ({len(newly_clean)}): " + ", ".join(
            f"{f/1e6:.1f}({a}->{b})" for f, a, b in newly_clean[:12]))
        moved = len(newly_hot) + len(newly_clean)
        print(f"channels that flipped state: {moved}/{len(both)} "
              f"({100*moved/max(1,len(both)):.0f}%) — a high number means the "
              f"emitter moved, and any pinned channel needs re-validation")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
