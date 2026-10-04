"""Per-row DIGEST trace of a leg capture: encoder n_live vs base live count, live-id set diff."""
import sys
sys.path.insert(0, "tools"); sys.path.insert(0, "base_station")
import vector_dry_run as v
E = "bench-evidence/RS_13_vector_scene_2026-09-26/legs"
leg, prof, lo, hi = sys.argv[1], sys.argv[2], int(sys.argv[3]), int(sys.argv[4])
dr = v.DryRun(prof, min_frames=250)
cur = {}
orig = dr.store._check_digest
def hook(rec):
    cur["d"] = (rec.n_live, len(dr.store._live()))
    return orig(rec)
dr.store._check_digest = hook
po = 0; prev_ids = None
for ts, topic, payload in v.iter_capture(f"{E}/leg{leg}_base.jsonl"):
    cur.clear()
    row = dr.feed(payload, ts); st = dr.store.stats
    ids = sorted(sh.id for sh in dr.store._live())
    if lo <= row.idx <= hi:
        add = sorted(set(ids) - set(prev_ids or [])); rem = sorted(set(prev_ids or []) - set(ids))
        print(f"#{row.idx:3d} seq={row.seq:3d} kind={row.frame_kind} {row.wire:3d}B rec={row.records:2d} dig={row.digest:3s} "
              f"enc/base n_live={cur.get('d')} orph+{st['orphans']-po} ttl={st['ttl_dropped']} rs={int(st['resync'])} +{add} -{rem}")
    po = st["orphans"]; prev_ids = ids
