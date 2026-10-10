"""Pass 2: every scenario × rule end to end — the real encoder (only its
relabel decision swapped) into the real store through test_vector_sync.Link,
500 ms per frame on a fake clock (no wall-clock safety refresh), optional
deterministic loss. Writes e2e_<tag>.json."""
from __future__ import annotations

import json
import sys
from multiprocessing import Pool

import cv2
import numpy as np


def run(job):
    name, rule_name, loss = job
    cv2.setNumThreads(1)
    from rl_common import ProbeEncoder, RULES, tvs, stable_seed
    from rl_scenarios import SCENARIOS
    n, gen, cut, kind = SCENARIOS[name]
    t = [1000.0]
    clock = lambda: t[0]                                   # noqa: E731
    link = tvs.Link(clock=clock)
    link.enc = ProbeEncoder(RULES[rule_name], clock=clock)
    rng = np.random.default_rng(stable_seed(name) ^ 0x5A5A)
    lose = rng.random(n) < loss
    lose[0] = False                                        # the first epoch start arrives
    starts, reasons, bytes_, resid = [], [], [], []
    resync_frames = 0
    digest_false = 0
    for i in range(n):
        link.step(gen(i), lose=bool(lose[i]))
        st = link.enc.last_stats
        bytes_.append(st["frame_bytes"])
        resid.append(st["residual"])
        if st["epoch_committed"] and i > 0:
            starts.append(i)
            reasons.append(st["epoch_trigger"])
        ss = link.store.stats
        resync_frames += bool(ss["resync"])
        if i > 0 and link.snap()["digest_ok"] is False:
            digest_false += 1
        t[0] += 0.5
    ss = link.store.stats
    lag = None
    if cut is not None:
        after = [s for s in starts if s >= cut]
        lag = (after[0] - cut) if after else None
    spurious = [s for s in starts if cut is None or not (cut <= s <= cut + 2)]
    return {"scenario": name, "rule": rule_name, "loss": loss, "kind": kind, "cut": cut,
            "starts": starts, "reasons": reasons, "spurious": len(spurious), "lag": lag,
            "mean_bytes": float(np.mean(bytes_)), "mean_resid": float(np.mean(resid)),
            "post_cut_resid": (float(np.mean(resid[cut:cut + 5])) if cut is not None else None),
            "orphans": ss["orphans"], "digest_checks": ss["digest_checks"], "digest_mismatch": ss["digest_mismatch"],
            "resync_events": ss["resync_events"], "resync_frames": resync_frames, "ttl_dropped": ss["ttl_dropped"],
            "digest_false_frames": digest_false, "final_digest_ok": link.snap()["digest_ok"],
            "lost": int(lose.sum())}


if __name__ == "__main__":
    from rl_scenarios import SCENARIOS
    tag = sys.argv[1]
    loss = float(sys.argv[2])
    rules = sys.argv[3].split(",")
    names = sys.argv[4].split(",") if len(sys.argv) > 4 else list(SCENARIOS)
    jobs = [(nm, r, loss) for r in rules for nm in names]
    with Pool(8) as pool:
        out = pool.map(run, jobs, chunksize=1)
    json.dump(out, open(f"e2e_{tag}.json", "w"), indent=0)
    print("runs", len(out))
