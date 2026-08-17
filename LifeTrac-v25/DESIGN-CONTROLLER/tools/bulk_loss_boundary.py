"""bulk_loss_boundary.py — does the BULK corruption cluster at train
boundaries?

The 7.08 s external emitter owns at most ~1/3 of the bench loss floor
(corrected parity replay). The rest is a clustered process that RS-11.4
localised, via the `lost_frag_idx` attribution instrument, to per-train-
boundary events. This tool tests that mechanism with a DIFFERENT instrument
that has never been pointed at it: the crc_dump captures themselves.

Method. Both event streams live in rx_daemon.log on one clock:

    `published frame_id=...`  -> a train COMPLETED reassembly (a boundary
                                 proxy: the next train's start follows the
                                 ~213.9 ms designed gap)
    `crc_dump: ...`           -> a corrupt reception, with RSSI/SNR

For every crc_dump in the settled part of a run, compute the signed offset
to the NEAREST publish event, after subtracting the interferer population
(the same stronger-AND-dirtier cut the temporal analysis uses). If the bulk
mechanism is a boundary event, offsets should pile up near zero; if
corruption is uniform in train time, offsets spread flat.

Null: each archive's publish stream has an irregular cadence, so uniformity
is judged against synthetic captures dropped uniformly over the same span
and measured against the same publish stream (10k draws).

Caveat, stated up front: publish events mark REASSEMBLY COMPLETION on the
base, not radio train boundaries. A train that loses a fragment may publish
late or never, which SMEARS true boundary clustering. A positive result is
therefore meaningful; a negative one is weaker evidence.
"""

from __future__ import annotations

import argparse
import pathlib
import re
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
from crc_dump_temporal import parse_archive, _ts_seconds  # noqa: E402

PUB_RE = re.compile(r"published frame_id=\d+")


def publish_times(path: pathlib.Path) -> np.ndarray:
    out = []
    t0 = None
    for raw in (path / "rx_daemon.log").read_text(
            encoding="utf-8", errors="replace").splitlines():
        wall = _ts_seconds(raw)
        if wall is None:
            continue
        if t0 is None:
            t0 = wall
        if PUB_RE.search(raw):
            rel = wall - t0
            if rel < -43200:
                rel += 86400
            out.append(rel)
    return np.array(out)


def nearest_offset(events: np.ndarray, refs: np.ndarray) -> np.ndarray:
    """Signed offset of each event to its nearest reference."""
    idx = np.searchsorted(refs, events)
    idx_lo = np.clip(idx - 1, 0, len(refs) - 1)
    idx_hi = np.clip(idx, 0, len(refs) - 1)
    d_lo = events - refs[idx_lo]
    d_hi = events - refs[idx_hi]
    return np.where(np.abs(d_lo) <= np.abs(d_hi), d_lo, d_hi)


def analyse(path: pathlib.Path, out=sys.stdout) -> dict | None:
    arch = parse_archive(path)
    pubs = publish_times(path)
    if len(pubs) < 20 or len(arch.captures) < 10:
        return None

    # settle point: same convention as the temporal analysis
    if arch.windows:
        wm = np.array([w.rssi_med for w in arch.windows], dtype=float)
        ref_rssi = float(np.median(wm[len(wm) // 2:]))
        t_settle = 60.0
    else:
        ref_rssi = float(np.median([c.rssi for c in arch.captures]))
        t_settle = 60.0

    bulk = [c for c in arch.captures
            if c.t >= t_settle and not (c.snr < -5.0 and c.rssi > ref_rssi)]
    if len(bulk) < 8:
        return None
    ev = np.array([c.t for c in bulk])
    span = (float(ev.min()), float(ev.max()))

    off = nearest_offset(ev, pubs)
    frac_100 = float((np.abs(off) <= 0.100).mean())
    med_abs = float(np.median(np.abs(off)))
    # Review catch (PR #99): the documented boundary proxy is the NEXT TRAIN
    # START, which follows the publish by the ~213.9 ms designed gap -- a
    # boundary-concentrated population would center at +214 ms and
    # structurally miss a publish-centered window. Test that window too.
    off_b = nearest_offset(ev, pubs + 0.2139)
    frac_100_b = float((np.abs(off_b) <= 0.100).mean())

    # uniform null over the same span, against the same publish stream
    rng = np.random.default_rng(20260816)
    n_iter = 10000
    null_frac = np.empty(n_iter)
    null_frac_b = np.empty(n_iter)
    for k in range(n_iter):
        synth = rng.uniform(span[0], span[1], size=len(ev))
        null_frac[k] = (np.abs(nearest_offset(synth, pubs)) <= 0.100).mean()
        null_frac_b[k] = (np.abs(nearest_offset(synth, pubs + 0.2139)) <= 0.100).mean()
    p = float((null_frac >= frac_100).mean())
    p_b = float((null_frac_b >= frac_100_b).mean())

    name = path.name
    print(f"{name[14:29]}  bulk n={len(bulk):3d}  pubs={len(pubs):3d}  "
          f"|off| med={med_abs * 1000:6.0f} ms  "
          f"within+/-100ms={frac_100 * 100:5.1f}% (null={null_frac.mean() * 100:4.1f}% p={p:.4f})  "
          f"@boundary(+214ms)={frac_100_b * 100:5.1f}% "
          f"(null={null_frac_b.mean() * 100:4.1f}% p={p_b:.4f})", file=out)
    return {"n": len(bulk), "frac": frac_100,
            "null": float(null_frac.mean()), "p": p,
            "offsets_ms": (off * 1000).tolist()}


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("root", type=pathlib.Path,
                    help="bench-evidence directory")
    args = ap.parse_args()

    print("bulk (non-interferer, settled) crc_dump offsets to nearest "
          "publish event:")
    print("-" * 78)
    results = []
    pooled = []
    for d in sorted(args.root.glob("radio_monitor_*")):
        log = d / "rx_daemon.log"
        if not log.is_file():
            continue
        text = log.read_text(encoding="utf-8", errors="replace")
        if "crc_dump" not in text:
            continue
        r = analyse(d)
        if r:
            results.append(r)
            pooled.extend(r["offsets_ms"])

    if not results:
        print("no analysable archives")
        return 1

    pooled = np.array(pooled)
    print("-" * 78)
    print(f"POOLED n={len(pooled)}  |offset| percentiles (ms): "
          f"p25={np.percentile(np.abs(pooled), 25):.0f}  "
          f"p50={np.percentile(np.abs(pooled), 50):.0f}  "
          f"p75={np.percentile(np.abs(pooled), 75):.0f}")
    hist, edges = np.histogram(pooled, bins=np.arange(-1000, 1001, 100))
    print("pooled offset histogram (-1000..1000 ms, 100 ms bins):")
    print("  " + " ".join(f"{int(v):3d}" for v in hist))
    signif = [r for r in results if r["p"] < 0.05]
    print(f"\narchives with boundary concentration at p<0.05: "
          f"{len(signif)}/{len(results)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
