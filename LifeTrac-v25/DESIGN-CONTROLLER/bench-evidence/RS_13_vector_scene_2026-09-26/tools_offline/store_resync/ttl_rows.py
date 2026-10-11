"""Rows where ttl_dropped increments, with the dropped ids (which dict, whether
the id was carriable and had a copy), captured inside _tick_ttl.
usage: ttl_rows.py <DC> <capture> <profile>"""
import os
import sys

dc = os.path.abspath(sys.argv[1])
sys.path.insert(0, os.path.join(dc, "tools"))
sys.path.insert(0, os.path.join(dc, "base_station"))
import vector_dry_run as v  # noqa: E402
import image_pipeline.vector_scene_store as S  # noqa: E402

log = []
orig = S.VectorSceneStore._tick_ttl


def traced(self):
    s0, c0 = dict(self._shapes), dict(self._cached)
    info = {i: (sh.frames_since_verify, i in self._verified_now) for i, sh in list(s0.items())}
    cinfo = {i: (sh.frames_since_verify, i in self._verified_now, self._carriable(i), i in s0)
             for i, sh in list(c0.items())}
    n0 = self._st["ttl_dropped"]
    orig(self)
    if self._st["ttl_dropped"] > n0:
        gone_s = [(i, info[i]) for i in s0 if i not in self._shapes]
        gone_c = [(i, cinfo[i]) for i in c0 if i not in self._cached]
        log.append((self._st["ttl_dropped"] - n0, self._epoch_clear, gone_s, gone_c))


S.VectorSceneStore._tick_ttl = traced
dr = v.DryRun(sys.argv[3])
for ts, _t, payload in v.iter_capture(sys.argv[2]):
    n = len(log)
    row = dr.feed(payload, ts)
    for d, clear, gs, gc in log[n:]:
        print(f"row {row.idx} ep={row.vs_epoch} K={row.frame_kind} switch={row.epoch_switched} +{d} clear={clear}")
        print(f"   from _shapes (id,(fsv_before,verified_now)): {gs}")
        print(f"   from _cached (id,(fsv_before,verified_now,carriable,has_copy)): {gc}")
