"""Are batched numpy formulations bit-identical to the per-item calls the encoder makes?"""
import sys
import numpy as np

sys.path.insert(0, sys.argv[1] if len(sys.argv) > 1 else "old")
import vector_extract as vx  # noqa: E402

rng = np.random.default_rng(1)
N = 20000
bad = {}


def check(name, a, b):
    a = np.atleast_1d(np.asarray(a))
    b = np.atleast_1d(np.asarray(b))
    ok = a.dtype == b.dtype and a.shape == b.shape and np.array_equal(a.view(np.uint8), b.view(np.uint8))
    if not ok:
        bad[name] = bad.get(name, 0) + 1


# means as the encoder makes them: float32 means of uint8 pixels, and those / gain, clipped
means = (rng.integers(0, 256, size=(N, 3)).astype(np.float32) + rng.random((N, 3), dtype=np.float32)).astype(np.float32)
means = np.clip(means, 0, 255)
labs_b = vx.rgb_to_lab(means)
de_b = vx.delta_e76(vx.PALETTE_LAB[None, :, :], labs_b[:, None, :])
for i in range(N):
    lab = vx.rgb_to_lab(means[i])
    check("lab", labs_b[i], lab)
    check("de_pal", de_b[i], vx.delta_e76(vx.PALETTE_LAB, lab[None, :]))
# pairwise lab distances (region x shape)
A = labs_b[:300]
B = labs_b[300:600]
D = vx.delta_e76(A[:, None, :], B[None, :, :])
for i in range(0, 300, 7):
    for j in range(300):
        check("de_pair", D[i, j], np.float32(vx.delta_e76(A[i], B[j])))
# kmeans centre distances
C = rng.normal(50, 30, size=(400, 8, 3)).astype(np.float32)
for c in C:
    D2 = np.linalg.norm(c[:, None, :] - c[None, :, :], axis=-1)
    for i in range(8):
        for j in range(i):
            check("centre_norm", float(D2[i, j]), float(np.linalg.norm(c[i] - c[j])))
# per-row sum of squares over (n,3) vs full-frame (h,w,3)
F = rng.random((64, 96, 3), dtype=np.float32) * 255
M = rng.random((64, 96, 3), dtype=np.float32) * 255
E = ((F - M) ** 2).sum(axis=2)
m = rng.random((64, 96)) < 0.3
check("err_rows", E[m], ((F[m] - M[m]) ** 2).sum(axis=1))
# (m,3) row ΔE against single (3,) calls
P = labs_b[:5000]
Q = labs_b[5000:10000]
DR = vx.delta_e76(P, Q)
for i in range(5000):
    check("de_rows", DR[i], vx.delta_e76(P[i], Q[i]))
# log2 of float32 ratios, batched vs per row (GAIN)
A3 = np.clip(means[:5000], 8, 245)
B3 = np.clip(means[5000:10000], 8, 245)
LB = np.log2(A3 / B3)
for i in range(5000):
    check("log2_rows", LB[i], np.log2(A3[i] / B3[i]))
# percentile pair vs two calls (trimmed mean)
for n in (20, 37, 500, 3000):
    for _ in range(20):
        px = rng.integers(0, 256, size=(n, 3)).astype(np.float32)
        lo, hi = np.percentile(px, (10, 90), axis=0)
        check("pct_lo", lo, np.percentile(px, 10, axis=0))
        check("pct_hi", hi, np.percentile(px, 90, axis=0))
# float32 np.round(means / 17) batch vs per row (RGB444 code)
QB = np.clip(np.round(means / 17.0), 0, 15).astype(int)
for i in range(0, N, 3):
    check("rgb444", QB[i], np.clip(np.round(np.asarray(means[i], dtype=np.float32) / 17.0), 0, 15).astype(int))
# findContours on a 1-px-margin crop with offset == on the whole image (both chain modes)
import cv2  # noqa: E402
ncomp = 0
for t in range(300):
    shp = (64, 96) if t % 2 else (128, 192)
    noise = rng.random(shp).astype(np.float32)
    img = (cv2.GaussianBlur(noise, (0, 0), 1.0 + (t % 4)) > 0.5).astype(np.uint8)
    for conn in (4, 8):
        n, cc, stats, _ = cv2.connectedComponentsWithStats(img, connectivity=conn)
        for i in range(1, n):
            ncomp += 1
            for method in (cv2.CHAIN_APPROX_SIMPLE, cv2.CHAIN_APPROX_NONE):
                full, _ = cv2.findContours((cc == i).astype(np.uint8), cv2.RETR_EXTERNAL, method)
                h, w = cc.shape
                x0, y0 = int(stats[i, 0]), int(stats[i, 1])
                x1, y1 = x0 + int(stats[i, 2]), y0 + int(stats[i, 3])
                x0, y0, x1, y1 = max(x0 - 1, 0), max(y0 - 1, 0), min(x1 + 1, w), min(y1 + 1, h)
                crop, _ = cv2.findContours((cc[y0:y1, x0:x1] == i).astype(np.uint8), cv2.RETR_EXTERNAL, method,
                                           offset=(x0, y0))
                same = len(full) == len(crop) and all(np.array_equal(a, b) for a, b in zip(full, crop))
                if not same:
                    bad["contour_crop"] = bad.get("contour_crop", 0) + 1
print("cv2", cv2.__version__, "components checked", ncomp)
print("numpy", np.__version__, "N", N, "mismatches:", bad if bad else "none")
