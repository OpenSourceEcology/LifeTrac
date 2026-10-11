import json, struct, sys
LEGS = sys.argv[1]
def rows(path):
    out = []
    with open(path) as f:
        for line in f:
            d = json.loads(line)
            if 'hex' not in d: continue
            b = bytes.fromhex(d['hex'])
            fk, seq, gw, gh, tpx, codec = struct.unpack_from('BBBBBB', b, 0)
            out.append((d['ts'], fk, codec, len(b), seq))
    return out
for tag in sys.argv[2:]:
    p = f"{LEGS}/{tag}_base.jsonl"
    r = rows(p)
    print("==", tag, len(r), "frames", "codecs", {c: sum(1 for x in r if x[2]==c) for c in set(x[2] for x in r)})
    for i in range(1, len(r)):
        if r[i][2] != r[i-1][2]:
            for j in range(max(1,i-2), min(len(r), i+3)):
                gap = r[j][0] - r[j-1][0]
                print(f"  row {j:4d} gap {gap:5.2f} s codec {r[j][2]} kind {r[j][1]} len {r[j][3]} seq {r[j][4]}")
            print("  --")
    g = [r[i][0]-r[i-1][0] for i in range(1, len(r))]
    gs = sorted(g)
    print("  gap p50 %.2f p95 %.2f max %.2f; gaps>1.0s: %d, >1.5s: %d, >2s: %d" % (gs[len(gs)//2], gs[int(len(gs)*0.95)], gs[-1], sum(x>1.0 for x in g), sum(x>1.5 for x in g), sum(x>2.0 for x in g)))
