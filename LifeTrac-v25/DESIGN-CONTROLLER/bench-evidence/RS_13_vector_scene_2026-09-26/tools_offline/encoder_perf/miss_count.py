"""Which records miss the encoder's record_bits / define_hash memos, per frame?"""
import collections
import sys

import numpy as np

import bench
sys.path.insert(0, ".")
import abnew.encode_vector as ev  # noqa: E402

miss = collections.Counter()
orig_bits, orig_dh = ev.vs.record_bits, ev.vs.define_hash


def bits(rec):
    miss["bits:" + type(rec).__name__] += 1
    return orig_bits(rec)


def dh(rec):
    miss["dhash:" + type(rec).__name__] += 1
    return orig_dh(rec)


ev.vs.record_bits, ev.vs.define_hash = bits, dh
srcs = dict(np.load("sources.npz"))
for run in sys.argv[1:] or ["pan243"]:
    miss.clear()
    t = [0.0]
    enc = ev.VectorEncoder(clock=lambda: t[0])
    n = 0
    for rgb, dt in bench.frames_of(run, srcs, 150):
        enc.frame(rgb, bench.RUNS[run][1], quality=bench.RUNS[run][2], seq=n)
        t[0] += dt
        n += 1
    print(run, {k: round(v / n, 1) for k, v in sorted(miss.items())})
