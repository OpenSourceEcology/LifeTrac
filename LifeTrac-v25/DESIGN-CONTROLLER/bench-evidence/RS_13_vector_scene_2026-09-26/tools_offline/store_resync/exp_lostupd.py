"""Experiment: a lost UPD frame -> resync -> carousel repair -> resync exit."""
import os
import sys

DC = sys.argv[1]
sys.path.insert(0, os.path.join(DC, "base_station"))
sys.path.insert(0, os.path.join(DC, "base_station", "tests"))
import cv2  # noqa: E402
import test_vector_sync as T  # noqa: E402
from image_pipeline.vector_scene import codec as vs  # noqa: E402

which = sys.argv[2] if len(sys.argv) > 2 else "small"
cv2.setRNGSeed(0)
link = T.Link()
if which == "small":
    base = [(20, 44, 4, T.GREEN), (50, 40, 4, T.GREEN)]
    rects = [(10, 20, 40, 30, T.RED)]
    moved = [(20, 44, 4, T.GREEN), (53, 40, 4, T.GREEN)]
    img0, img1 = T.scene(blobs=base, rects=rects), T.scene(blobs=moved, rects=rects)
elif which == "mid":
    base = [(20, 44, 4, T.GREEN), (50, 40, 4, T.GREEN), (80, 48, 4, T.GREEN), (35, 52, 3, T.GREEN)]
    rects = [(10, 20, 40, 30, T.RED), (60, 18, 90, 28, (200, 200, 60))]
    moved = list(base)
    moved[1] = (53, 40, 4, T.GREEN)
    img0, img1 = T.scene(blobs=base, rects=rects), T.scene(blobs=moved, rects=rects)
else:
    rects = T.SyncTests.grid31()
    moved = rects[:]
    moved[0] = (rects[0][0] + 3, rects[0][1], rects[0][2] + 3, rects[0][3], T.RED)
    img0, img1 = T.scene(y0=8.0, rects=rects), T.scene(y0=8.0, rects=moved)
for _ in range(3):
    link.step(img0)
lost = link.step(img1, lose=True)
print("lost", [type(r).__name__ for r in lost.records])
for i in range(40):
    f = link.step(img1)
    st = link.store.stats
    enc, store = link.ttl_clocks()
    print(i + 1, "K" if f.header.key else "-", "dig", link.snap()["digest_ok"], "resync", st["resync"],
          "orph", st["orphans"], "ttl", st["ttl_dropped"], "/", link.enc.last_stats["ttl_dropped"],
          "ends", st.get("resync_digest_ends"), "clk_eq", enc == store, "maxclk", max(store.values() or [0]),
          "recs", ",".join(type(r).__name__[:4] for r in f.records))
