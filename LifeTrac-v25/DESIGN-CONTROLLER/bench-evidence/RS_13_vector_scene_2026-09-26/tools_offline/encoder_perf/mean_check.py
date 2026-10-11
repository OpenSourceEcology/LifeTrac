"""Does an axis-0 float32 mean over integer columns equal the 1-D float32 mean bit for bit,
including the cases where a direct float32 division and float64-then-float32 disagree?"""
import numpy as np

rng = np.random.default_rng(3)
found = []
for _ in range(400000):
    n = int(rng.integers(6, 6144))
    s = int(rng.integers(0, min(n * 255, 2 ** 24 - 1)))
    f32 = np.float32(np.float32(s) / np.float32(n))          # a direct float32 division
    f64 = np.float32(np.float64(s) / np.float64(n))          # float64, then rounded to float32
    if f32 != f64:
        found.append((s, n))
    if len(found) >= 200:
        break
print("double-rounding cases found:", len(found))
bad = 0
for s, n in found[:200]:
    # an integer column of length n summing to s
    q, r = divmod(s, n)
    col = np.full(n, q, np.float32)
    col[:r] += 1
    one_d = col.mean()                                        # the old per-column mean
    two_d = np.stack([col, col, col, col, col], axis=1)[:, :5].mean(axis=0)[3]
    f32 = np.float32(np.float32(s) / np.float32(n))
    f64 = np.float32(np.float64(s) / np.float64(n))
    if one_d != two_d:
        bad += 1
    if len(found) and (s, n) == found[0]:
        print("first case", s, n, "1-D mean ==", "f64" if one_d == f64 else "f32",
              "| axis-0 mean ==", "f64" if two_d == f64 else "f32")
print("numpy", np.__version__, "1-D vs axis-0 mean mismatches:", bad)
