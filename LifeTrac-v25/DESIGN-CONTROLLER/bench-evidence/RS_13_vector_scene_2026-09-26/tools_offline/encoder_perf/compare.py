"""compare.py ref.json new.json: byte-identity per run (and the first differing frame) + timing."""
import json
import sys

a = json.load(open(sys.argv[1]))
b = json.load(open(sys.argv[2]))
all_ok = True
for name, ra in a["runs"].items():
    rb = b["runs"].get(name)
    if rb is None:
        continue
    same = ra["sha256"] == rb["sha256"]
    first = None
    if not same:
        all_ok = False
        for i, (pa, pb) in enumerate(zip(ra["payloads"], rb["payloads"])):
            if pa != pb:
                first = i
                break
    print(f"{name:11s} {'IDENTICAL' if same else 'DIFFER at frame %s' % first:22s} frames={len(rb['payloads'])} "
          f"p50 {ra['p50']:7.2f} -> {rb['p50']:7.2f}  p95 {ra['p95']:7.2f} -> {rb['p95']:7.2f}")
    if not same:
        print("   old stages", ra["stage_p50"])
        print("   new stages", rb["stage_p50"])
print("ALL old p50=%.2f p95=%.2f %s" % (a["all"]["p50"], a["all"]["p95"], a["all"]["stage_p50"]))
print("ALL new p50=%.2f p95=%.2f %s" % (b["all"]["p50"], b["all"]["p95"], b["all"]["stage_p50"]))
print("BYTE-IDENTICAL" if all_ok else "MISMATCH")
