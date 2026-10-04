"""Per-row digest / resync pattern in row windows.
usage: digest_pattern.py <DC> <capture> <profile> a:b [c:d ...]"""
import os
import sys

dc = os.path.abspath(sys.argv[1])
sys.path.insert(0, os.path.join(dc, "tools"))
sys.path.insert(0, os.path.join(dc, "base_station"))
import vector_dry_run as v  # noqa: E402

dr = v.DryRun(sys.argv[3])
rows = []
prev = dr.store.stats
info = []
for ts, _t, payload in v.iter_capture(sys.argv[2]):
    r = dr.feed(payload, ts)
    st = dr.store.stats
    info.append((r, st["orphans"] - prev["orphans"], st["ttl_dropped"] - prev["ttl_dropped"], st["shapes"],
                 st["cached_shapes"]))
    prev = st
for win in sys.argv[4:]:
    a, b = (int(x) for x in win.split(":"))
    line = []
    for r, orph, ttl, ns, nc in info[a:b + 1]:
        seq = r.seq
        line.append(f"#{r.idx} s{seq} K{r.frame_kind} {r.digest}{' R' if r.resync else ''}"
                    f"{' o+%d' % orph if orph else ''}{' ttl+%d' % ttl if ttl else ''}")
    print(f"--- rows {a}..{b}")
    for ln in line:
        print("   ", ln)
