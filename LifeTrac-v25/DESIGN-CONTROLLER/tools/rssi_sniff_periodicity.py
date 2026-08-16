"""rssi_sniff_periodicity.py — hunt the 7.085 s line in raw RSSI samples.

Input: the JSONL stream from air_coupling_rssi_sniff.py (RSSI_SAMPLE lines
with epoch `ts` and `rssi_dbm`), captured with all bench radios silent.

Two independent detectors, because the emitter's duty cycle is unknown:

1. EVENT detector — samples exceeding the floor by a margin become events;
   their times get the same Rayleigh fold-scan used on the crc_dump captures
   (imported from crc_dump_temporal, so the machinery is identical).
2. PHASE-FOLD detector — fold EVERY sample's RSSI on the 7.0853 s grid and
   compare phase-bin means. Sensitive to weak periodic elevation even when
   no single sample crosses the event threshold. Significance via circular
   shift of the RSSI series against its own timestamps (preserves both the
   sampling jitter and the RSSI autocorrelation, destroys phase).

The known fingerprint being hunted: period 7.0853 s, hits ~26 dB above the
healthy level in traffic runs. The sniffer's RegRssiValue is wideband channel
RSSI, so a hit during a sample should stand far above a quiet floor.
"""

from __future__ import annotations

import argparse
import json
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
from crc_dump_temporal import rayleigh, refine_period, scan_periods  # noqa: E402

TARGET_PERIOD = 7.0853


def load(path: pathlib.Path) -> tuple[np.ndarray, np.ndarray]:
    ts, rssi = [], []
    for line in path.read_text(encoding="utf-8", errors="replace").splitlines():
        line = line.strip()
        if not line.startswith("RSSI_SAMPLE"):
            continue
        try:
            d = json.loads(line[len("RSSI_SAMPLE"):].strip())
            ts.append(float(d["ts"]))
            rssi.append(float(d["rssi_dbm"]))
        except (ValueError, KeyError):
            continue
    t = np.array(ts)
    r = np.array(rssi)
    if len(t):
        t = t - t[0]
    return t, r


def hunt_verdict(t: np.ndarray, r: np.ndarray) -> int:
    """One-line PRESENT/ABSENT verdict for the suspect-elimination loop.

    Detection = hot samples (>-75 dBm, far above any plausible floor and
    ~consistent with the emitter's known -43..-63 dBm range at bench
    geometry) whose de-duplicated excursion times fit a 6.5-7.5 s grid with
    R >= 0.8, needing >= 3 excursions. At the measured catch rate the line
    produces ~1 excursion per ~2 cycles minimum, so 120 s (~17 cycles) gives
    plenty of margin; an ABSENT verdict on <60 s of data is refused.
    """
    dur = float(t[-1]) if len(t) else 0.0
    hot = t[r > -75.0]
    if len(hot) >= 2:
        keep = np.concatenate([[True], np.diff(hot) > 0.5])
        hot = hot[keep]
    # Review catch (PR #99): with very few excursions, maximising R over a
    # free period grid reaches R>=0.8 by chance and would falsely report
    # PRESENT after the emitter is actually removed — the costly hunt error.
    # Gates: (a) at least 4 excursions, (b) residual RMS <= 0.12 cycles (the
    # real line measures 0.03-0.10), so the fit must be a grid, not merely a
    # concentrated fold. 3 or fewer hot excursions is reported as ambiguous
    # rather than PRESENT.
    if len(hot) >= 4:
        fit = refine_period(hot, 6.5, 7.5, n_grid=40000)
        if fit["R"] >= 0.8 and fit["resid_rms"] <= 0.12:
            print(f"LINE PRESENT: {len(hot)} excursions, "
                  f"period {fit['period']:.3f}s, R={fit['R']:.2f}, "
                  f"resid={fit['resid_rms']:.3f} cyc, "
                  f"hottest {r.max():.0f} dBm")
            return 10
        print(f"HOT ENERGY PRESENT but not on the grid: {len(hot)} "
              f"excursions, best 6.5-7.5s fit R={fit['R']:.2f} "
              f"resid={fit['resid_rms']:.3f}, "
              f"hottest {r.max():.0f} dBm — investigate")
        return 11
    if len(hot) == 3:
        print(f"AMBIGUOUS: only 3 hot excursions in {dur:.0f}s — too few to "
              f"confirm the grid (hottest {r.max():.0f} dBm). Extend the "
              f"sniff (-DurationS 240) and re-run.")
        return 11
    if dur < 60.0:
        print(f"INCONCLUSIVE: only {dur:.0f}s of data (<60s) — "
              f"cannot call ABSENT")
        return 12
    print(f"LINE ABSENT: 0-{len(hot)} hot excursion(s) in {dur:.0f}s "
          f"(~{dur / 7.08:.0f} emitter cycles), floor median "
          f"{np.median(r):.0f} dBm, max {r.max():.0f} dBm")
    return 0


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("jsonl", type=pathlib.Path)
    ap.add_argument("--margin-db", type=float, default=6.0,
                    help="event threshold above median floor")
    ap.add_argument("--hunt", action="store_true",
                    help="one-line PRESENT/ABSENT verdict for the "
                         "suspect-elimination loop (exit 10=present, "
                         "11=hot-but-aperiodic, 12=inconclusive, 0=absent)")
    args = ap.parse_args()

    t, r = load(args.jsonl)
    if len(t) < 100:
        print(f"only {len(t)} samples parsed — not enough to analyse")
        return 1
    if args.hunt:
        return hunt_verdict(t, r)

    dur = float(t[-1])
    med = float(np.median(r))
    print(f"samples={len(t)}  span={dur:.1f}s  "
          f"effective rate={len(t) / dur:.1f} Hz")
    print(f"floor: median={med:.1f} dBm  p95={np.percentile(r, 95):.1f}  "
          f"p99={np.percentile(r, 99):.1f}  max={r.max():.1f}  "
          f"min={r.min():.1f}")

    # -- 1. event detector
    thr = med + args.margin_db
    ev_mask = r > thr
    ev_t = t[ev_mask]
    print(f"\n-- events: rssi > {thr:.1f} dBm (median + {args.margin_db:.0f}) "
          f"-> n={len(ev_t)} --")
    if len(ev_t) >= 6:
        # collapse consecutive samples of one excursion into one event
        keep = np.concatenate([[True], np.diff(ev_t) > 0.5])
        ev_t = ev_t[keep]
        print(f"   {len(ev_t)} distinct excursions")
        rr, pp = rayleigh(ev_t, TARGET_PERIOD)
        print(f"   Rayleigh @ {TARGET_PERIOD}s: R={rr:.3f} p={pp:.3f}")
        _, _, best = scan_periods(ev_t, dur, 0.5, dur / 3.0)
        print(f"   best-period scan: {best['period']:.4f}s R={best['R']:.3f} "
              f"p_corrected={best['p_corrected']:.3f}")
    else:
        print("   too few excursions for a period fit — "
              "no strong periodic emitter visible at this margin")

    # -- 2. phase-fold detector at the known period
    nbins = 24
    phase = np.mod(t, TARGET_PERIOD) / TARGET_PERIOD
    bins = np.minimum((phase * nbins).astype(int), nbins - 1)
    bin_mean = np.array([r[bins == b].mean() if (bins == b).any() else np.nan
                         for b in range(nbins)])
    spread = float(np.nanmax(bin_mean) - np.nanmin(bin_mean))
    print(f"\n-- phase fold @ {TARGET_PERIOD}s, {nbins} bins --")
    print(f"   bin means (dBm): "
          f"{', '.join(f'{v:.1f}' for v in bin_mean)}")
    print(f"   max-min spread = {spread:.2f} dB")

    # null: circularly shift RSSI against its own timestamps
    rng = np.random.default_rng(20260816)
    null_spreads = []
    for _ in range(400):
        k = rng.integers(1, len(r) - 1)
        rs = np.roll(r, int(k))
        bm = np.array([rs[bins == b].mean() if (bins == b).any() else np.nan
                       for b in range(nbins)])
        null_spreads.append(np.nanmax(bm) - np.nanmin(bm))
    null95 = float(np.percentile(null_spreads, 95))
    frac = float((np.array(null_spreads) >= spread).mean())
    print(f"   circular-shift null: spread 95th pct={null95:.2f} dB, "
          f"fraction >= observed = {frac:.3f}")
    if frac < 0.05:
        print("   verdict: PERIODIC RSSI elevation at the target period")
    else:
        print("   verdict: no periodic elevation detectable at "
              f"{TARGET_PERIOD}s in this capture")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
