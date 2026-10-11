"""Replay RS-13.1 captures through tools/vector_dry_run.DryRun and report the
store line plus resync episodes (start row, end row, how it ended, duration).

usage: py -3 episodes.py <DC dir> <label> <capture.jsonl> <profile> [...pairs]
"""
import os
import sys

dc = os.path.abspath(sys.argv[1])
label = sys.argv[2]
sys.path.insert(0, os.path.join(dc, "tools"))
sys.path.insert(0, os.path.join(dc, "base_station"))
import vector_dry_run as v  # noqa: E402

pairs = sys.argv[3:]
for cap, prof in zip(pairs[0::2], pairs[1::2]):
    dr = v.DryRun(prof)
    episodes = []
    cur = None
    prev = dr.store.stats
    rows = []
    for ts, _t, payload in v.iter_capture(cap):
        row = dr.feed(payload, ts)
        st = dr.store.stats
        rows.append(row)
        r = st["resync"]
        if r and cur is None:
            cur = {"start": row.idx, "t0": row.t_rel, "orph0": prev["orphans"], "ttl0": prev["ttl_dropped"]}
        elif not r and cur is not None:
            how = "key" if row.frame_kind == 1 else ("epoch-switch" if row.epoch_switched else "digest-run")
            if "resync_digest_ends" in st and st["resync_digest_ends"] > prev.get("resync_digest_ends", 0):
                how = "digest-run"
            cur.update(end=row.idx, t1=row.t_rel, how=how, orph=st["orphans"] - cur["orph0"],
                       ttl=st["ttl_dropped"] - cur["ttl0"])
            episodes.append(cur)
            cur = None
        prev = st
    if cur is not None:
        cur.update(end=None, t1=rows[-1].t_rel, how="open at end", orph=dr.store.stats["orphans"] - cur["orph0"],
                   ttl=dr.store.stats["ttl_dropped"] - cur["ttl0"])
        episodes.append(cur)
    s = dr.summary()
    lines = v.format_summary(s).splitlines()
    print(f"=== {label}: {os.path.basename(cap)} ({prof})")
    for ln in lines:
        if ln.startswith(("store:", "scene:", "epoch starts:")):
            print("  " + ln)
    bad = sum(1 for r in rows if r.digest == "BAD")
    print(f"  digest BAD rows {bad}; resync episodes {len(episodes)}; frames in resync "
          f"{sum(((e['end'] if e['end'] is not None else len(rows)) - e['start']) for e in episodes)}")
    for e in episodes:
        end = e["end"] if e["end"] is not None else "end"
        n = (e["end"] if e["end"] is not None else len(rows)) - e["start"]
        print(f"    rows {e['start']}->{end}: {n} frames, {e['t1'] - e['t0']:.1f} s, ended by {e['how']}, "
              f"orphans +{e['orph']}, ttl_dropped +{e['ttl']}")
