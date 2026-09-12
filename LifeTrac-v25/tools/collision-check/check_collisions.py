#!/usr/bin/env python3
"""Pairwise interference check between exported rigid groups of the LifeTrac v25 assembly.

Prototype for issue #119. For one animation time t it loads every ``<group>_t<t>.stl``
found in the output directory (static groups fall back to their t=0 export), runs an FCL
broad/narrow-phase check through ``trimesh.collision.CollisionManager`` and prints, per
pair, the number of triangle contacts and the maximum FCL contact depth, plus the nearest
approach for pairs that are clear.

Notes
-----
* FCL's contact depth is a per-triangle-pair number. It is a reliable zero/non-zero
  filter (flush faces and polygon slivers stay below a fraction of a millimetre) but it is
  not the size of the overlap. Use export_pair.sh + analyze_pairs.py for real volumes.
* OpenSCAD 2021.01 writes STL with six significant digits, which breaks watertightness of
  small features, so the Manifold boolean volume is only attempted when both meshes are
  watertight. A nightly OpenSCAD (full-precision export, Manifold backend) fixes this.
"""
import glob
import itertools
import os
import sys
import time

import numpy as np
import trimesh

OUT = os.environ.get("COLLISION_OUT", os.path.dirname(os.path.abspath(__file__)) + "/out")
STATIC = {"frame", "wheels", "platform"}
DEPTH_TOL_MM = 0.5      # ignore face-on-face contacts and polygonal slivers below this
VOLUME_TOL_MM3 = 50.0   # ignore sliver overlaps below this
# Pairs that are allowed to touch or overlap by design (pins in lugs, clevises on mounts).
# Fill this in once the joints are modelled with real clearance holes.
WHITELIST = set()


def load_group(group, t):
    path = f"{OUT}/{group}_t{t}.stl"
    if not os.path.exists(path) and group in STATIC:
        path = f"{OUT}/{group}_t0.stl"
    if not os.path.exists(path):
        return None
    m = trimesh.load(path, force="mesh", process=True)
    if m.is_empty:
        return None
    m.merge_vertices()
    return m


def main(t):
    groups = sorted({os.path.basename(p).split("_t")[0] for p in glob.glob(f"{OUT}/*_t*.stl")
                     if not os.path.basename(p).startswith("pair_")})
    meshes = {}
    for g in groups:
        m = load_group(g, t)
        if m is not None:
            meshes[g] = m
            print(f"loaded {g:<11} faces={len(m.faces):>8} watertight={m.is_watertight} "
                  f"bounds={np.round(m.bounds, 1).tolist()}")
    if len(meshes) < 2:
        print("need at least two groups")
        return 2

    t0 = time.time()
    cm = trimesh.collision.CollisionManager()
    for g, m in meshes.items():
        cm.add_object(g, m)
    _hit, _names, data = cm.in_collision_internal(return_names=True, return_data=True)
    t_fcl = time.time() - t0

    per_pair = {}
    for c in data:
        key = tuple(sorted(c.names))
        d = per_pair.setdefault(key, {"contacts": 0, "max_depth": 0.0})
        d["contacts"] += 1
        d["max_depth"] = max(d["max_depth"], float(c.depth))

    failures = 0
    print(f"\nFCL check over {len(meshes)} groups took {t_fcl:.2f}s; colliding pairs: {len(per_pair)}")
    print(f"{'pair':<28}{'contacts':>9}{'max depth mm':>14}{'overlap mm^3':>14}  verdict")
    for a, b in itertools.combinations(sorted(meshes), 2):
        d = per_pair.get((a, b))
        vol = "n/a"
        verdict = "clear"
        if d:
            ma, mb = meshes[a], meshes[b]
            real = d["max_depth"] > DEPTH_TOL_MM
            if ma.is_watertight and mb.is_watertight:
                try:
                    inter = trimesh.boolean.intersection([ma, mb], engine="manifold", check_volume=False)
                    vol = f"{abs(inter.volume):.1f}"
                    real = abs(inter.volume) > VOLUME_TOL_MM3
                except Exception as e:  # noqa: BLE001 - report and fall back to depth
                    vol = f"err:{type(e).__name__}"
            if (a, b) in WHITELIST:
                verdict = "allowed (whitelist)"
            elif real:
                verdict = "INTERFERENCE"
                failures += 1
            else:
                verdict = "touch/sliver (ignored)"
            print(f"{a + '/' + b:<28}{d['contacts']:>9}{d['max_depth']:>14.3f}{vol:>14}  {verdict}")
        else:
            print(f"{a + '/' + b:<28}{0:>9}{0.0:>14.3f}{vol:>14}  {verdict}")

    print("\nnearest approach for clear pairs (mm):")
    for a, b in itertools.combinations(sorted(meshes), 2):
        if (a, b) in per_pair:
            continue
        single = trimesh.collision.CollisionManager()
        single.add_object(a, meshes[a])
        print(f"  {a}/{b}: {single.min_distance_single(meshes[b]):.1f}")
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1] if len(sys.argv) > 1 else "0"))
