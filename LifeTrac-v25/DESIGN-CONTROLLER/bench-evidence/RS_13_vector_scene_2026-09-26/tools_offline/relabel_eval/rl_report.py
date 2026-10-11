"""Summarise e2e_<tag>.json per rule: churn on no-change scenarios, cut
detection, bytes, residual and the store's verdicts."""
import json
import sys
from collections import defaultdict

tag = sys.argv[1] if len(sys.argv) > 1 else "loss0"
R = json.load(open(f"e2e_{tag}.json"))
by = defaultdict(list)
for r in R:
    by[r["rule"]].append(r)

print(f"== {tag}: {len(R)} runs")
hdr = ("rule", "nochg starts", "nochg frames", "partial starts", "cuts lag<=1", "cut miss", "cut spur",
       "bytes", "resid nochg", "resid cut", "post-cut", "orph", "dig mm/chk", "resync ev/fr", "ttl")
print("%-15s %12s %12s %14s %11s %8s %8s %6s %11s %9s %8s %5s %11s %12s %4s" % hdr)
rows = []
for rule, rs in by.items():
    nc = [r for r in rs if r["kind"] == "nochange"]
    pa = [r for r in rs if r["kind"] == "partial"]
    cu = [r for r in rs if r["kind"] == "cut"]
    nstarts = sum(len(r["starts"]) for r in nc)
    nframes = sum(40 if "static" not in r["scenario"] or not r["scenario"].startswith("syn_static") else 50
                  for r in nc)
    pstarts = sum(len(r["starts"]) for r in pa)
    ok = sum(1 for r in cu if r["lag"] is not None and r["lag"] <= 1)
    miss = [r["scenario"] for r in cu if r["lag"] is None or r["lag"] > 1]
    spur = sum(r["spurious"] for r in cu)
    allr = rs
    mb = sum(r["mean_bytes"] for r in allr) / len(allr)
    rn = sum(r["mean_resid"] for r in nc) / len(nc)
    rc = sum(r["mean_resid"] for r in cu) / len(cu)
    pc = sum(r["post_cut_resid"] for r in cu) / len(cu)
    orph = sum(r["orphans"] for r in allr)
    dmm = sum(r["digest_mismatch"] for r in allr)
    dch = sum(r["digest_checks"] for r in allr)
    rse = sum(r["resync_events"] for r in allr)
    rsf = sum(r["resync_frames"] for r in allr)
    ttl = sum(r["ttl_dropped"] for r in allr)
    print("%-15s %12d %12s %14d %11s %8d %8d %6.1f %11.1f %9.1f %8.1f %5d %11s %12s %4d"
          % (rule, nstarts, f"{nstarts}/{nframes}", pstarts, f"{ok}/{len(cu)}", len(miss), spur, mb, rn, rc, pc,
             orph, f"{dmm}/{dch}", f"{rse}/{rsf}", ttl))
    rows.append((rule, miss))
print()
for rule, miss in rows:
    if miss:
        print(f"{rule}: missed {miss}")
