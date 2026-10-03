"""Replay a leg's base capture: post-lock seq gaps, resync episodes vs epoch starts (P1 loss rule), orphan bursts."""
import sys, collections
sys.path.insert(0, "tools"); sys.path.insert(0, "base_station")
import vector_dry_run as v
E = "bench-evidence/RS_13_vector_scene_2026-09-26/legs"
leg, prof = sys.argv[1], sys.argv[2]
dr = v.DryRun(prof, min_frames=250)
prev = False; episodes = []; start = None; keys = []; seqs = []; codecs = []; orph = collections.Counter(); po = 0; bad = []
for ts, topic, payload in v.iter_capture(f"{E}/leg{leg}_base.jsonl"):
    row = dr.feed(payload, ts); st = dr.store.stats; seqs.append(row.seq); codecs.append(row.codec)
    if row.vector and row.frame_kind == 1: keys.append(row.idx)
    if row.digest == "BAD": bad.append(row.idx)
    if st["orphans"] > po: orph[row.idx] = st["orphans"] - po
    po = st["orphans"]
    if st["resync"] and not prev: start = row.idx
    if prev and not st["resync"]: episodes.append((start, row.idx, row.frame_kind == 1)); start = None
    prev = st["resync"]
if prev: episodes.append((start, None, False))
# seq is a u8 per codec, and each codec keeps its own counter (a mono_g4 <-> VECTOR
# switch restarts it), so only consecutive frames of the SAME codec are compared.
gaps = [(i, (b - a) % 256 - 1) for i, ((a, ca), (b, cb)) in enumerate(zip(zip(seqs, codecs), zip(seqs[1:], codecs[1:])), start=1)
        if ca == cb and (b - a) % 256 > 1]
switches = sum(1 for ca, cb in zip(codecs, codecs[1:]) if ca != cb)
lost = sum(g for _, g in gaps)
print(f"leg {leg}: {len(seqs)} frames received; seq gaps within same-codec runs (row, lost): {gaps} -> {lost} lost of {len(seqs) + lost} = {100.0 * lost / (len(seqs) + lost):.1f} % after the first received frame ({switches} codec switch(es) not counted)")
print(f"  resync episodes (start, end, ended by key): {episodes}")
print(f"  all ended by an epoch start: {all(e[2] for e in episodes)}; final resync {dr.store.stats['resync']}, digest_ok {dr.store.snapshot(int(dr.last_ts * 1000) + 100)['digest_ok']}")
print(f"  key rows ({len(keys)}): {keys[:50]}")
print(f"  BAD rows ({len(bad)}): {bad[:80]}")
print(f"  orphan bursts top: {orph.most_common(8)}; orphans {dr.store.stats['orphans']}, ttl_dropped {dr.store.stats['ttl_dropped']}")
