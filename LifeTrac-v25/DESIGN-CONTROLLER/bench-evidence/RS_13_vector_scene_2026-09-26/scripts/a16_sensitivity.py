"""A16 sensitivity: the same single-drop sweep on scenes with fewer
CONFIRM-only (static) shapes, shipped encoder only, to see how the A16 rate and
the 'shapes short' magnitude scale toward the bench's 1-2 shapes.

Usage: PYTHONIOENCODING=utf-8 py -3 a16_sensitivity.py
"""
from __future__ import annotations

import json
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import a16_sil as S  # noqa: E402
from a16_common import encode_stream  # noqa: E402

SCENES = {
    "static12+trees5 (main)": dict(n_star=8, n_static=12, n_static_trees=5),
    "static6": dict(n_star=8, n_static=6, n_static_trees=0),
    "static2": dict(n_star=8, n_static=2, n_static_trees=0),
}


def main():
    out = os.path.join(os.path.dirname(os.path.abspath(__file__)), "a16_sens_out")
    os.makedirs(out, exist_ok=True)
    frames, step = 300, 3
    drops = list(range(30, frames - S.WINDOW - 1, step))
    res = {}
    for sname, scene in SCENES.items():
        for bname, budget in (("bw500", 243), ("bw250", 203)):
            t1 = time.time()
            stream = encode_stream(frames, budget, scene_kw=scene, safety_s=1e9)
            main_rows, forks = S.forked_replay(stream, drops)
            eps = [S.episode(stream, d, forks[d], main_rows[d - 1]["ttl"]) for d in drops]
            s = S.summarise(f"{sname}/{bname}/norefresh", stream, budget, eps, main_rows)
            a16 = [e for e in eps if e["a16_like"]]
            short = []
            for e in a16:
                rows = forks[e["drop"]][1:]
                short.append(max(len(r["missing"]) for r in rows))
            keep = {k: s[k] for k in ("case", "frame_bytes", "records_p50", "n_live_p50", "carousel", "drops",
                                      "disturbed", "recovery_frames", "a16_like", "a16_orph_per_missing_frame",
                                      "a16_missing_id_frames_why")}
            keep["a16_max_shapes_short_p50"] = S.pct(short, 0.5)
            keep["a16_max_shapes_short_max"] = max(short) if short else None
            keep["seconds"] = round(time.time() - t1, 1)
            res[keep["case"]] = keep
            print(json.dumps(keep, default=str), flush=True)
    with open(os.path.join(out, "sensitivity.json"), "w", encoding="utf-8") as fh:
        json.dump(res, fh, default=str, indent=1)


if __name__ == "__main__":
    main()
