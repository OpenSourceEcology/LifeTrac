import sys
DC = r"C:/Users/dorkm/Documents/GitHub/LifeTrac-rs13-evidence/LifeTrac-v25/DESIGN-CONTROLLER"
sys.path.insert(0, DC + "/base_station"); sys.path.insert(0, DC + "/tools")
import vector_dry_run as v
from image_pipeline.frame_format import parse_tile_delta_frame
for leg in sys.argv[1:]:
    p = DC + f"/bench-evidence/RS_13_vector_scene_2026-09-26/legs/leg{leg}_base.jsonl"
    rows = []
    for ts, topic, pl in v.iter_capture(p):
        try:
            f = parse_tile_delta_frame(pl)
            rows.append((ts, f.codec, f.base_seq, f.frame_kind, len(pl)))
        except Exception as e:
            rows.append((ts, None, None, None, len(pl)))
    print(leg, len(rows))
    prev = None
    for i, r in enumerate(rows):
        if prev is not None:
            d = (r[2] - prev[2]) % 256 if r[2] is not None and prev[2] is not None else None
            if d != 1 or r[1] != prev[1]:
                print("  row", i, "prev", prev[1:4], "cur", r[1:4], "d", d, "dt", round(r[0]-prev[0],2))
        prev = r
    print("  first", rows[0][1:4], "last", rows[-1][1:4])
