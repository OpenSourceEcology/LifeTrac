#!/usr/bin/env python3
"""Interleaved A/B of two encoder versions in ONE process (fair under a noisy CPU).

  python ab.py <old_pkg_dir> <new_pkg_dir> <sources.npz> <out.json> [--runs a,b] [--frames N]

Each *_pkg_dir is a directory holding encode_vector.py, vector_extract.py, vs1_codec.py and
an __init__.py; it is imported as a package (its name = the directory's basename). Every
frame is encoded by both versions in alternating order; payloads must be byte-identical.
Times: wall (perf_counter, the encoder's own last_stats['ms']) and CPU (thread_time).
"""
import hashlib
import importlib
import json
import os
import sys
import time

import numpy as np

import bench


def load(pkg_dir):
    parent, name = os.path.split(os.path.abspath(pkg_dir))
    if parent not in sys.path:
        sys.path.insert(0, parent)
    return importlib.import_module(name + ".encode_vector")


def pct(v, p):
    return float(np.percentile(np.asarray(v, np.float64), p)) if len(v) else 0.0


def main():
    a = sys.argv[1:]
    old_dir, new_dir, src_path, out_path = a[:4]
    runs, n = list(bench.RUNS), 150
    rest = a[4:]
    for i in range(0, len(rest), 2):
        if rest[i] == "--runs":
            runs = rest[i + 1].split(",")
        elif rest[i] == "--frames":
            n = int(rest[i + 1])
    evs = {"old": load(old_dir), "new": load(new_dir)}
    srcs = dict(np.load(src_path))
    out = {"runs": {}, "cv2": bench.cv2.__version__, "numpy": np.__version__}
    agg = {k: {"wall": [], "cpu": [], "stages": {}} for k in evs}
    mismatches = 0
    for name in runs:
        scen, budget, quality, masked = bench.RUNS[name]
        clocks = {k: [0.0] for k in evs}
        encs = {k: ev.VectorEncoder(mask=bench.hood_mask() if masked else None,
                                    clock=(lambda c=clocks[k]: c[0])) for k, ev in evs.items()}
        rec = {k: {"wall": [], "cpu": [], "stages": {}, "sha": hashlib.sha256()} for k in evs}
        first_diff = None
        for fi, (rgb, dt) in enumerate(bench.frames_of(name, srcs, n)):
            order = ("old", "new") if fi % 2 == 0 else ("new", "old")
            pay = {}
            for k in order:
                c0 = time.thread_time()
                pay[k] = encs[k].frame(rgb, budget, quality=quality, seq=fi)
                c1 = time.thread_time()
                clocks[k][0] += dt
                ms = encs[k].last_stats["ms"]
                rec[k]["wall"].append(ms["total"])
                rec[k]["cpu"].append((c1 - c0) * 1000.0)
                for st, v in ms.items():
                    if st != "total":
                        rec[k]["stages"].setdefault(st, []).append(v)
                rec[k]["sha"].update(len(pay[k]).to_bytes(2, "big") + pay[k])
            if pay["old"] != pay["new"] and first_diff is None:
                first_diff = fi
        same = first_diff is None
        mismatches += 0 if same else 1
        r_out = {"identical": same, "first_diff": first_diff}
        line = f"{name:11s} {'IDENTICAL' if same else 'DIFFER@%d' % first_diff:10s}"
        for k in evs:
            r = rec[k]
            r_out[k] = {"sha256": r["sha"].hexdigest(), "wall_p50": pct(r["wall"], 50), "wall_p95": pct(r["wall"], 95),
                        "cpu_p50": pct(r["cpu"], 50), "cpu_p95": pct(r["cpu"], 95),
                        "stage_p50": {s: round(pct(v, 50), 2) for s, v in r["stages"].items()}}
            agg[k]["wall"] += r["wall"]
            agg[k]["cpu"] += r["cpu"]
            for s, v in r["stages"].items():
                agg[k]["stages"].setdefault(s, []).extend(v)
            line += f" | {k} wall {r_out[k]['wall_p50']:6.1f}/{r_out[k]['wall_p95']:6.1f} cpu {r_out[k]['cpu_p50']:6.1f}/{r_out[k]['cpu_p95']:6.1f}"
        line += f" | sha {r_out['new']['sha256'][:12]}"
        print(line, flush=True)
        out["runs"][name] = r_out
    for k in evs:
        g = agg[k]
        out[k] = {"wall_p50": pct(g["wall"], 50), "wall_p95": pct(g["wall"], 95),
                  "cpu_p50": pct(g["cpu"], 50), "cpu_p95": pct(g["cpu"], 95),
                  "stage_p50": {s: round(pct(v, 50), 2) for s, v in g["stages"].items()}}
        print(f"ALL {k}: wall p50 {out[k]['wall_p50']:.1f} p95 {out[k]['wall_p95']:.1f} | cpu p50 {out[k]['cpu_p50']:.1f} "
              f"p95 {out[k]['cpu_p95']:.1f} | stages p50 {out[k]['stage_p50']}", flush=True)
    for m in ("wall_p50", "wall_p95", "cpu_p50", "cpu_p95"):
        print(f"  {m}: {100.0 * (1.0 - out['new'][m] / out['old'][m]):.1f} % off")
    print("BYTE-IDENTICAL on every run" if mismatches == 0 else f"MISMATCH on {mismatches} run(s)", flush=True)
    with open(out_path, "w") as fh:
        json.dump(out, fh)


if __name__ == "__main__":
    main()
