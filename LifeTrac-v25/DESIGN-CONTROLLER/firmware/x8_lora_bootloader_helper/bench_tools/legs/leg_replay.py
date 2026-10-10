"""leg_replay.py -- replay a leg's base capture: post-lock seq gaps, resync episodes
vs epoch starts (the P1 loss rule), orphan bursts, last DIGEST checked.

Usage (PC, any working directory):
    py -3 leg_replay.py <leg> <image_bw500|image_bw250> [--evidence-dir DIR | --capture FILE]

    <leg>            the leg tag; reads <DIR>/leg<leg>_base.jsonl
    --evidence-dir   the legs/ folder holding the capture (default: $EVIDENCE_DIR,
                     which lib/bench_env.sh exports)
    --capture        a capture file directly (overrides --evidence-dir)

Reads only the capture; touches no board and no radio. leg_post.sh prints its
first line as the "seq-gap loss (per codec run)" row of leg<leg>_p2p4.txt.
Origin: bench-evidence/RS_13_vector_scene_2026-09-26/scripts/leg_replay.py
(historical copy, unchanged; it hardcoded the RS-13.1 legs folder and had to run
from DESIGN-CONTROLLER).
"""
import argparse
import collections
import os
import sys
from pathlib import Path

DC = Path(__file__).resolve().parents[4]          # legs -> bench_tools -> helper -> firmware -> DESIGN-CONTROLLER
sys.path.insert(0, str(DC / "tools"))             # vector_dry_run puts base_station/ on sys.path itself
import vector_dry_run as v  # noqa: E402

ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
ap.add_argument("leg")
ap.add_argument("profile", choices=["image_bw500", "image_bw250"])
ap.add_argument("--evidence-dir", default=os.environ.get("EVIDENCE_DIR"))
ap.add_argument("--capture")
args = ap.parse_args()
if args.capture:
    cap = Path(args.capture)
elif args.evidence_dir:
    cap = Path(args.evidence_dir) / f"leg{args.leg}_base.jsonl"
else:
    ap.error("give --evidence-dir or --capture (or set EVIDENCE_DIR)")
if not cap.is_file():
    ap.error(f"no capture at {cap}")

leg, prof = args.leg, args.profile
dr = v.DryRun(prof, min_frames=250)
prev = False; episodes = []; start = None; keys = []; seqs = []; codecs = []; orph = collections.Counter(); po = 0; bad = []; lastd = None
for ts, topic, payload in v.iter_capture(str(cap)):
    row = dr.feed(payload, ts); st = dr.store.stats; seqs.append(row.seq); codecs.append(row.codec)
    if row.vector and row.frame_kind == 1: keys.append(row.idx)
    if row.digest == "BAD": bad.append(row.idx)
    if row.digest in ("ok", "BAD"): lastd = (row.idx, row.digest)
    if st["orphans"] > po: orph[row.idx] = st["orphans"] - po
    po = st["orphans"]
    if st["resync"] and not prev: start = row.idx
    if prev and not st["resync"]: episodes.append((start, row.idx, row.frame_kind == 1)); start = None
    prev = st["resync"]
if prev: episodes.append((start, None, False))
if not seqs:
    print(f"leg {leg}: 0 frames in {cap}")
    raise SystemExit(1)
# seq is a u8 per codec, and each codec keeps its own counter (a mono_g4 <-> VECTOR
# switch restarts it), so only consecutive frames of the SAME codec are compared.
gaps = [(i, (b - a) % 256 - 1) for i, ((a, ca), (b, cb)) in enumerate(zip(zip(seqs, codecs), zip(seqs[1:], codecs[1:])), start=1)
        if ca == cb and (b - a) % 256 > 1]
switches = sum(1 for ca, cb in zip(codecs, codecs[1:]) if ca != cb)
lost = sum(g for _, g in gaps)
print(f"leg {leg}: {len(seqs)} frames received; seq gaps within same-codec runs (row, lost): {gaps} -> {lost} lost of {len(seqs) + lost} = {100.0 * lost / (len(seqs) + lost):.1f} % after the first received frame ({switches} codec switch(es) not counted)")
print(f"  resync episodes (start, end, ended by key): {episodes}")
print(f"  all ended by an epoch start: {all(e[2] for e in episodes)}; final resync {dr.store.stats['resync']}, digest_ok {dr.store.snapshot(int(dr.last_ts * 1000) + 100)['digest_ok']}")
print(f"  last DIGEST checked (row, result): {lastd}; last epoch start row {keys[-1] if keys else None}, {len(seqs) - 1 - keys[-1] if keys else None} frame(s) before the end (no DIGEST inside TTL_FRAMES+3 of a range-1/2 start, encode_vector.py)")
print(f"  key rows ({len(keys)}): {keys[:50]}")
print(f"  BAD rows ({len(bad)}): {bad[:80]}")
print(f"  orphan bursts top: {orph.most_common(8)}; orphans {dr.store.stats['orphans']}, ttl_dropped {dr.store.stats['ttl_dropped']}")
