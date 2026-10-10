"""Which summation order does numpy use for a length-3 last-axis float32 reduction?"""
import numpy as np

rng = np.random.default_rng(7)
for shape in ((6000, 8, 3), (64, 96, 3), (6000, 3), (3,)):
    x = (rng.random(shape, dtype=np.float32) * 300).astype(np.float32) ** 2
    s = x.sum(axis=-1)
    a = (x[..., 0] + x[..., 1]) + x[..., 2]
    b = x[..., 0] + (x[..., 1] + x[..., 2])
    print(shape, "(s0+s1)+s2:", bool(np.array_equal(s, a)), " s0+(s1+s2):", bool(np.array_equal(s, b)),
          " mismatches a/b:", int(np.sum(s != a)), int(np.sum(s != b)))
