"""Pass 3: the loss trade-off. Long no-change runs (160 frames = 80 s at
2 fps, so the 60 s safety refresh is inside) under deterministic loss; per
frame the agreement between the encoder's mirror and the store's live set
(Jaccard of (id, define-hash, state-hash)) — what the base can draw right."""
from __future__ import annotations

import json
import sys
from multiprocessing import Pool

import cv2
import numpy as np

LONG = ["syn_static_n24", "syn_pan_4px_n24", "ph29_static_n12", "ph29_pan_8", "ph29_pan_16", "ph31_pan_16",
        "sky29_static_n12", "sky28_static_n12", "sky28_pan_16", "ph29_fast", "ph31_fast", "ph29_tilt_up"]
N = 160
import os  # noqa: E402
UNSTICK = int(os.environ.get("RL_UNSTICK", "0"))
REFRESH = float(os.environ.get("RL_REFRESH", "0"))


def run(job):
    name, rule_name, loss, rep = job
    cv2.setNumThreads(1)
    from rl_common import ProbeEncoder, RULES, tvs, stable_seed
    from rl_scenarios import SCENARIOS
    _, gen, _, _ = SCENARIOS[name]
    t = [1000.0]
    clock = lambda: t[0]                                   # noqa: E731
    if REFRESH:
        from rl_common import ev
        ev.SAFETY_REFRESH_S = REFRESH                       # experiment only
    link = tvs.Link(clock=clock)
    link.enc = ProbeEncoder(RULES[rule_name], mc=False, clock=clock)
    if UNSTICK:
        # Experiment only (not the store under test): leave resync after
        # UNSTICK consecutive matching DIGESTs, the mirror of the entry rule.
        st = link.store
        orig = st._check_digest
        st._ok_run = 0

        def check(rec, st=st, orig=orig):
            orig(rec)
            st._ok_run = st._ok_run + 1 if st._digest_ok else 0
            if st._resync and st._ok_run >= UNSTICK:
                st._resync = False
        st._check_digest = check
    rng = np.random.default_rng((stable_seed(name) ^ 0x1234) + 7919 * rep)
    lose = rng.random(N) < loss
    lose[0] = False
    jac, resync, ok = [], 0, 0
    starts = 0
    for i in range(N):
        link.step(gen(i), lose=bool(lose[i]))
        if link.enc.last_stats["epoch_committed"] and i > 0:
            starts += 1
        enc = {(sid, s.dhash, s.state_hash()) for sid, s in link.enc._shapes.items()}
        st = link.store
        base = {(sh.id, sh.dhash, st._state_hash(sh)) for sh in st._live()}
        u = enc | base
        j = len(enc & base) / len(u) if u else 1.0
        jac.append(j)
        ok += (j == 1.0)
        resync += bool(st.stats["resync"])
        t[0] += 0.5
    ss = link.store.stats
    return {"scenario": name, "rule": rule_name, "loss": loss, "rep": rep, "starts": starts,
            "jaccard": float(np.mean(jac)), "in_step_frames": ok, "resync_frames": resync,
            "orphans": ss["orphans"], "digest_mismatch": ss["digest_mismatch"], "resync_events": ss["resync_events"],
            "ttl_dropped": ss["ttl_dropped"], "lost": int(lose.sum())}


if __name__ == "__main__":
    tag = sys.argv[1]
    rules = sys.argv[2].split(",")
    losses = [float(x) for x in sys.argv[3].split(",")]
    reps = int(sys.argv[4]) if len(sys.argv) > 4 else 1
    jobs = [(nm, r, l, k) for l in losses for r in rules for nm in LONG for k in range(reps)]
    with Pool(8) as pool:
        out = pool.map(run, jobs, chunksize=1)
    json.dump(out, open(f"loss_{tag}.json", "w"), indent=0)
    from collections import defaultdict
    agg = defaultdict(list)
    for r in out:
        agg[(r["loss"], r["rule"])].append(r)
    print("%-6s %-15s %7s %9s %12s %13s %8s %9s %8s %5s" % ("loss", "rule", "starts", "jaccard", "in-step fr",
                                                            "resync fr", "orph", "dig mm", "rs ev", "ttl"))
    for (l, ru), rs in sorted(agg.items()):
        nf = N * len(rs)
        print("%-6.2f %-15s %7d %9.3f %12s %13s %8d %9d %8d %5d" % (
            l, ru, sum(r["starts"] for r in rs), np.mean([r["jaccard"] for r in rs]),
            "%d/%d" % (sum(r["in_step_frames"] for r in rs), nf), "%d/%d" % (sum(r["resync_frames"] for r in rs), nf),
            sum(r["orphans"] for r in rs), sum(r["digest_mismatch"] for r in rs),
            sum(r["resync_events"] for r in rs), sum(r["ttl_dropped"] for r in rs)))
