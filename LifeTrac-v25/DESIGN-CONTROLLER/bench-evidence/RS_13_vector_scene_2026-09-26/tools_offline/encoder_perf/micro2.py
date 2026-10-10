"""Where does np.linalg.lstsq spend its time on a (n, 3) float32 problem?"""
import timeit

import numpy as np
from numpy.linalg import _umath_linalg

rng = np.random.default_rng(0)


def t(label, fn, number=500):
    fn()
    dt = min(timeit.repeat(fn, number=number, repeat=3)) / number * 1e3
    print(f"{label:48s} {dt:8.4f} ms")


for n in (6, 40, 300, 2000):
    a = np.ones((n, 3), np.float32)
    a[:, 1] = rng.integers(0, 20, n) - 10.0
    a[:, 2] = rng.integers(0, 20, n) - 10.0
    b = (rng.random(n, dtype=np.float32) * 255).astype(np.float32)
    rcond = np.finfo(np.float64).eps * max(n, 3)
    t(f"lstsq n={n}", lambda: np.linalg.lstsq(a, b, rcond=None))

    def direct():
        x, resids, rank, s = _umath_linalg.lstsq(a, b[:, None], rcond, signature="ddd->ddid")
        return x[:, 0].astype(np.float32)

    t(f"  gufunc direct n={n}", direct)
    ref = np.linalg.lstsq(a, b, rcond=None)[0]
    print("    identical:", bool(np.array_equal(ref, direct())), ref.dtype)
