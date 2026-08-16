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
from crc_dump_temporal import rayleigh, scan_periods  # noqa: E402

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


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("jsonl", type=pathlib.Path)
    ap.add_argument("--margin-db", type=float, default=6.0,
                    help="event threshold above median floor")
    args = ap.parse_args()

    t, r = load(args.jsonl)
    if len(t) < 100:
        print(f"only {len(t)} samples parsed — not enough to analyse")
        return 1

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
