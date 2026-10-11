"""Off-air A16 probe: replay a VECTOR capture through the base's own DryRun/store,
dropping ONE extra frame at a time, and measure how long the store stays out of step.
Read-only on the trees (imports tools/vector_dry_run.py from the code tree)."""
import sys, time, statistics
CODE = r"C:\Users\dorkm\Documents\GitHub\LifeTrac\LifeTrac-v25\DESIGN-CONTROLLER"
sys.path.insert(0, CODE + r"\tools"); sys.path.insert(0, CODE + r"\base_station")
import vector_dry_run as v

def load(path):
    return [(ts, p) for ts, _t, p in v.iter_capture(path)]

def replay(frames, prof, drop=None):
    dr = v.DryRun(prof)
    rows = []
    for i, (ts, p) in enumerate(frames):
        if i == drop:
            rows.append(None); continue
        rows.append(dr.feed(p, ts))
    return rows, dr

def outcome(rows, d):
    """After drop d: first BAD DIGEST, then the first ok DIGEST after it; was a key row in between?"""
    first_bad = None
    for i in range(d + 1, len(rows)):
        r = rows[i]
        if r is None or not r.vector: continue
        if first_bad is None:
            if r.digest == "BAD": first_bad = i
            elif r.digest == "ok": return ("harmless", 0, False, i)
            continue
        if r.digest == "ok":
            keys = [j for j in range(first_bad, i + 1) if rows[j] is not None and rows[j].frame_kind == 1]
            return ("repaired", i - d, (keys[0] - d) if keys else False, i)
    return ("unrepaired_at_end", len(rows) - d, False, None)

def main(path, prof, step, min_tail):
    frames = load(path)
    base, dr0 = replay(frames, prof)
    seqs = [r.seq for r in base]
    gaps = {i for i in range(1, len(seqs)) if (seqs[i] - seqs[i-1]) % 256 != 1}
    t0 = time.time()
    res = []
    for d in range(30, len(frames) - min_tail, step):
        # only drop where the baseline is clean around d (no real gap within -5..+min_tail, digest ok just before)
        if any(g in gaps for g in range(d - 5, d + min_tail)): continue
        prev = [r for r in base[max(0, d-6):d] if r.digest in ("ok", "BAD")]
        if not prev or prev[-1].digest != "ok" or base[d].resync: continue
        rows, dr = replay(frames, prof, d)
        kind, n, key, at = outcome(rows, d)
        res.append((d, kind, n, key, base[d].frame_kind == 1, base[d].wire))
    dt = time.time() - t0
    fps = (len(frames) - 1) / (frames[-1][0] - frames[0][0])
    print(f"{path.split(chr(92))[-1]}: {len(frames)} frames, {fps:.2f} fps, prof {prof}, real gaps at {sorted(gaps)[:20]}; {len(res)} single drops tried in {dt:.0f} s")
    harmless = [r for r in res if r[1] == "harmless"]
    rep = [r for r in res if r[1] == "repaired"]
    rep_dig = [r for r in rep if not r[3]]; rep_key = [r for r in rep if r[3]]
    un = [r for r in res if r[1] == "unrepaired_at_end"]
    print(f"  harmless (next DIGEST ok): {len(harmless)}; repaired by DIGEST run: {len(rep_dig)}; needed an epoch start: {len(rep_key)}; unrepaired at capture end: {len(un)}")
    if rep_dig: print(f"  frames to repair (DIGEST): median {statistics.median([r[2] for r in rep_dig])}, max {max(r[2] for r in rep_dig)}")
    if rep_key: print(f"  needed an epoch start: frames from drop to that key: median {statistics.median([r[3] for r in rep_key])}, max {max(r[3] for r in rep_key)}; (drop, frames to key, frames to first ok DIGEST): {[(r[0], r[3], r[2]) for r in rep_key][:40]}")
    long_ = [r for r in rep_key if r[3] > 20 * fps] + [r for r in rep_dig if r[2] > 20 * fps]
    print(f"  out of step > 20 s (to the key, or to the ok DIGEST): {len(long_)} of {len(res)}")

if __name__ == "__main__":
    main(sys.argv[1], sys.argv[2], int(sys.argv[3]), int(sys.argv[4]))
