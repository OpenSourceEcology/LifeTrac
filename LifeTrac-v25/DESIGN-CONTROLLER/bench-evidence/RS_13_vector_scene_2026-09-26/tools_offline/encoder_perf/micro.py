"""Micro-timings of candidate formulations (run on the X8 and the PC)."""
import timeit

import numpy as np

rng = np.random.default_rng(0)


def t(label, fn, number=200):
    fn()
    dt = min(timeit.repeat(fn, number=number, repeat=3)) / number * 1e3
    print(f"{label:48s} {dt:8.3f} ms")


data = (rng.random((5000, 3), dtype=np.float32) * 100).astype(np.float32)
c = (rng.random((8, 3), dtype=np.float32) * 100).astype(np.float32)
t("kmeans init dist: reduce (n,8,3)", lambda: ((data[:, None, :] - c[None, :, :]) ** 2).sum(axis=2))


def explicit():
    d0 = data[:, 0, None] - c[None, :, 0]
    d1 = data[:, 1, None] - c[None, :, 1]
    d2 = data[:, 2, None] - c[None, :, 2]
    return d0 * d0 + d1 * d1 + d2 * d2


t("kmeans init dist: explicit", explicit)
a = ((data[:, None, :] - c[None, :, :]) ** 2).sum(axis=2)
print("   identical:", bool(np.array_equal(a, explicit())))

rasters = [rng.random((64, 96)) < 0.05 for _ in range(35)]
for r in rasters:
    r[10:20, 10:30] = True


def loop_top():
    top = np.zeros((64, 96), np.int32)
    sec = np.zeros((64, 96), np.int32)
    for i, r in enumerate(rasters):
        sec[r] = top[r]
        top[r] = i + 1
    return top, sec


def max_top():
    n = len(rasters)
    st = np.stack(rasters).reshape(n, -1)
    lab = st * np.arange(1, n + 1, dtype=np.uint8)[:, None]
    last = lab.max(axis=0)
    lab[lab == last] = 0
    sec = lab.max(axis=0)
    return last.reshape(64, 96), sec.reshape(64, 96)


def argmax_top():
    n = len(rasters)
    st = np.stack(rasters).reshape(n, -1)
    last = (n - 1) - np.argmax(st[::-1], axis=0)
    top = np.where(st.any(axis=0), last + 1, 0)
    st[last, np.arange(st.shape[1])] = False
    below = (n - 1) - np.argmax(st[::-1], axis=0)
    sec = np.where(st.any(axis=0), below + 1, 0)
    return top.reshape(64, 96), sec.reshape(64, 96)


t("paint top+second: loop", loop_top)
t("paint top+second: argmax", argmax_top)
t("paint top+second: max", max_top)
a, b = loop_top()
c1, d1 = max_top()
e1, f1 = argmax_top()
print("   identical:", bool(np.array_equal(a, c1) and np.array_equal(b, d1) and np.array_equal(a, e1) and np.array_equal(b, f1)))


def loop_top_only():
    top = np.zeros((64, 96), np.int32)
    for i, r in enumerate(rasters):
        top[r] = i + 1
    return top


def max_top_only():
    n = len(rasters)
    st = np.stack(rasters).reshape(n, -1)
    return (st * np.arange(1, n + 1, dtype=np.uint8)[:, None]).max(axis=0).reshape(64, 96)


t("paint top: loop", loop_top_only)
t("paint top: max", max_top_only)
print("   identical:", bool(np.array_equal(loop_top_only(), max_top_only())))
x = np.zeros(3, np.float32)
t("np.clip small", lambda: np.clip(x * x, 0.0, 255.0), 5000)
img = rng.integers(0, 256, (64, 96, 3)).astype(np.uint8)
m = rng.random((64, 96)) < 0.3
t("bool index 64x96x3 float32", lambda: img[m], 5000)
t("np.linalg.lstsq (300,3)", lambda: np.linalg.lstsq(np.ones((300, 3), np.float32) + rng.random((300, 3), dtype=np.float32), np.ones(300, np.float32), rcond=None), 500)
