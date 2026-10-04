"""Experiment: lose a safety refresh's key frame and its repeat; does the
store's ttl_dropped stay equal to the encoder's through the pending hand-over?"""
import os
import sys

DC = sys.argv[1]
sys.path.insert(0, os.path.join(DC, "base_station"))
sys.path.insert(0, os.path.join(DC, "base_station", "tests"))
import cv2  # noqa: E402
import test_vector_sync as T  # noqa: E402
from image_pipeline.vector_scene import codec as vs  # noqa: E402

cv2.setRNGSeed(0)
t = [1000.0]
link = T.Link(clock=lambda: t[0])
blobs = [(20, 44, 4, T.GREEN), (50, 40, 4, T.GREEN), (80, 48, 4, T.GREEN)]
rects = [(10, 20, 40, 30, T.RED), (60, 18, 90, 28, (200, 200, 60))]
img = T.scene(blobs=blobs, rects=rects)
for _ in range(4):
    link.step(img)
    t[0] += 0.5
t[0] += 61.0
nlost = int(sys.argv[2]) if len(sys.argv) > 2 else 2
for i in range(nlost):
    f = link.step(img, lose=True)
    t[0] += 0.5
    print("lost", f.header, [type(r).__name__ for r in f.records])
for i in range(40):
    f = link.step(img)
    t[0] += 0.5
    st = link.store.stats
    snap = link.snap()
    print(i, "K" if f.header.key else "-", "ep", f.header.epoch, "store ttl", st["ttl_dropped"], "enc ttl",
          link.enc.last_stats["ttl_dropped"], "handover", snap["handover"], "shapes", st["shapes"], "cached",
          st["cached_shapes"], "dig", snap["digest_ok"], "resync", st["resync"],
          "recs", ",".join(type(r).__name__[:4] for r in f.records))
