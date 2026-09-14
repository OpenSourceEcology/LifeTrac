#!/usr/bin/env python3
"""Summarize OpenSCAD-rendered intersection() meshes produced by export_pair.sh.

Prints, per pair, the overlap volume, how many separate overlap bodies there are and the
volume and centroid of the three largest, which is usually enough to see which joint or
part the overlap belongs to.
"""
import glob
import os
import sys

import numpy as np
import trimesh

OUT = os.environ.get("COLLISION_OUT", os.path.dirname(os.path.abspath(__file__)) + "/out")


def main(t):
    files = sorted(glob.glob(f"{OUT}/pair_*_t{t}.stl"))
    if not files:
        print(f"no pair_*_t{t}.stl files in {OUT}")
        return 2
    print(f"{'pair':<22}{'volume mm^3':>13}{'bodies':>8}  largest bodies (volume mm^3 @ centroid x,y,z mm)")
    for f in files:
        name = os.path.basename(f)[len("pair_"):-len(f"_t{t}.stl")]
        m = trimesh.load(f, force="mesh", process=True)
        if m.is_empty:
            print(f"{name:<22}{0:>13}{0:>8}  (empty)")
            continue
        m.merge_vertices()
        bodies = m.split(only_watertight=False)
        vols = sorted(((abs(b.volume), b) for b in bodies), key=lambda x: -x[0])
        desc = "; ".join(f"{v:.0f} @ {np.round(b.centroid).astype(int).tolist()}" for v, b in vols[:3])
        print(f"{name:<22}{abs(m.volume):>13.0f}{len(bodies):>8}  {desc}")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1] if len(sys.argv) > 1 else "0"))
