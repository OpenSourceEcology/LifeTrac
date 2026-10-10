"""Pass 1: per-frame features of every scenario (rule-independent up to the
gain normalisation), so the candidate rules can be screened offline."""
from __future__ import annotations

import json
import sys
from multiprocessing import Pool

import cv2


def run(name):
    cv2.setNumThreads(1)
    from rl_common import ProbeEncoder
    from rl_scenarios import SCENARIOS
    n, gen, cut, kind = SCENARIOS[name]
    enc = ProbeEncoder(lambda f: False, clock=lambda: 0.0)       # never relabel: features only
    for i in range(n):
        enc.frame(gen(i), 203, seq=i)
    feats = []
    for f in enc.feat_log:
        if f is not None:
            f = dict(f)
            f["mc_shift"] = list(f["mc_shift"])
        feats.append(f)
    return name, {"cut": cut, "kind": kind, "feats": feats}


if __name__ == "__main__":
    from rl_scenarios import SCENARIOS
    names = sys.argv[1:] or list(SCENARIOS)
    with Pool(8) as pool:
        out = dict(pool.map(run, names))
    json.dump(out, open("features.json", "w"), indent=0)
    print("scenarios", len(out))
