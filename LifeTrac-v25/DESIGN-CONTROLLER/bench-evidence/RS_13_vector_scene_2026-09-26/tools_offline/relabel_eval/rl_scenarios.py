"""Deterministic scenario sequences for the A8 relabel evaluation.

Each scenario: (n_frames, gen(i) -> RGB image, cut frame index or None, kind).
kind 'nochange' scenarios must not churn; 'cut' scenarios must start an
epoch within 1-2 frames of the cut index (the first frame of the new view).
"""
from __future__ import annotations

import numpy as np

from rl_common import tvs, view, CW, CH

SKY, GROUND, GREEN, RED = tvs.SKY, tvs.GROUND, tvs.GREEN, tvs.RED
YELLOW = (200, 200, 40)
BLUE = (40, 60, 180)
PURPLE = (130, 50, 150)
GREY = (150, 150, 150)
DKGREEN = (30, 80, 40)
BLOBS = tvs.SyncTests.BLOBS
RECTS = tvs.SyncTests.RECTS


def _wide_objects(seed: int, width: int = 288, n_rect: int = 14, n_blob: int = 12, y_lo: int = 30):
    rng = np.random.default_rng(seed)
    pal = [RED, GREEN, YELLOW, BLUE, PURPLE, GREY, DKGREEN]
    rects, blobs = [], []
    for _ in range(n_rect):
        x0 = int(rng.integers(0, width - 12))
        y0 = int(rng.integers(y_lo, 56))
        rects.append((x0, y0, x0 + int(rng.integers(5, 16)), min(63, y0 + int(rng.integers(4, 10))),
                      pal[int(rng.integers(0, len(pal)))]))
    for _ in range(n_blob):
        blobs.append((int(rng.integers(0, width)), int(rng.integers(y_lo + 4, 60)), int(rng.integers(3, 6)),
                      pal[int(rng.integers(0, len(pal)))]))
    return rects, blobs


def _clip_rects(rects, off):
    out = []
    for x0, y0, x1, y1, c in rects:
        a, b = x0 - off, x1 - off
        if b < 0 or a > 95:
            continue
        out.append((max(0, a), y0, min(95, b), y1, c))
    return out


def _syn_window(objs, off, y0=112.0, noise=12, seed=0, ground=None):
    rects, blobs = objs
    img = tvs.scene(y0=y0, blobs=[(cx - off, cy, r, c) for cx, cy, r, c in blobs],
                    rects=_clip_rects(rects, off), noise=0)
    if ground is not None:
        line = y0 / 4.0
        ys = np.arange(64)[:, None] + 0.5
        below = np.broadcast_to(ys >= line, (64, 96))
        g = np.all(img == np.array(GROUND, np.uint8), axis=2) & below
        img[g] = ground
    if noise:
        import cv2
        big = cv2.resize(img, (CW, CH), interpolation=cv2.INTER_NEAREST)
        rng = np.random.default_rng(seed)
        img = np.clip(big.astype(np.int16) + rng.integers(-noise, noise + 1, big.shape), 0, 255).astype(np.uint8)
    return img


def _pingpong(i, step, lo, hi):
    span = hi - lo
    if span <= 0:
        return lo
    t = (i * step) % (2 * span)
    return lo + (t if t <= span else 2 * span - t)


def build() -> dict:
    S: dict = {}
    # ---------------------------------------------------------- synthetic (scene())
    for n in (12, 24, 28):
        S[f"syn_static_n{n}"] = (50, lambda i, n=n: tvs.scene(blobs=BLOBS, rects=RECTS, noise=n, seed=i),
                                 None, "nochange")
    wide = _wide_objects(7)
    for k in (1, 2, 4, 8):
        S[f"syn_pan_{k}px"] = (40, lambda i, k=k: _syn_window(wide, _pingpong(i, k, 0, 192), seed=i), None,
                               "nochange")
    S["syn_pan_4px_n24"] = (40, lambda i: _syn_window(wide, _pingpong(i, 4, 0, 192), noise=24, seed=i), None,
                            "nochange")
    other = _wide_objects(99)
    FIELD = (70, 110, 70)                 # a different field: the ground mass changes too (full relabel)
    S["syn_cut"] = (40, lambda i: (tvs.scene(blobs=BLOBS, rects=RECTS, noise=12, seed=i) if i < 20 else
                                   _syn_window(other, 40, noise=12, seed=i, ground=FIELD)), 20, "cut")
    S["syn_cut_n24"] = (40, lambda i: (tvs.scene(blobs=BLOBS, rects=RECTS, noise=24, seed=i) if i < 20 else
                                       _syn_window(other, 40, noise=24, seed=i, ground=FIELD)), 20, "cut")
    S["syn_pan_cut"] = (40, lambda i: (_syn_window(wide, _pingpong(i, 2, 0, 192), seed=i) if i < 20 else
                                       _syn_window(other, _pingpong(i, 2, 0, 192), seed=i, ground=FIELD)), 20, "cut")
    recol = [(cx, cy, r, YELLOW) for cx, cy, r, _ in BLOBS]
    S["syn_recolour_cut"] = (40, lambda i: (tvs.scene(blobs=BLOBS, rects=RECTS, noise=12, seed=i) if i < 20 else
                                            tvs.scene(blobs=recol, rects=[(0, 28, 95, 63, (210, 210, 215)),
                                                                          (60, 20, 90, 30, BLUE)],
                                                      noise=12, seed=i)), 20, "cut")
    S["syn_gain_like_recolour"] = (40, lambda i: (tvs.scene(blobs=BLOBS, rects=RECTS, noise=12, seed=i) if i < 20 else
                                                  tvs.scene(blobs=recol, rects=[(0, 28, 95, 63, (150, 140, 60)),
                                                                                (60, 20, 90, 30, BLUE)],
                                                            noise=12, seed=i)), 20, "partial")
    # Partial changes (the ground and sky stay): not a camera change; the
    # temporal pass DELs and defines. Informational: no rule is required to
    # start an epoch, and today's rule does not.
    S["syn_objects_change"] = (40, lambda i: (tvs.scene(blobs=BLOBS, rects=RECTS, noise=12, seed=i) if i < 20 else
                                              _syn_window(other, 40, noise=12, seed=i)), 20, "partial")
    S["syn_objects_recolour"] = (40, lambda i: tvs.scene(blobs=BLOBS if i < 20 else recol,
                                                         rects=RECTS if i < 20 else [(60, 20, 90, 30, BLUE)],
                                                         noise=12, seed=i), 20, "partial")
    # Sky-heavy synthetic view: the horizon low (canvas row 200 → working row 50,
    # 78 % sky owned by L0), a ground strip with a few masses; a cut changes the
    # strip's colour and its masses (labelled ≈ 22 % of the valid area).
    skyA = ([(10, 52, 25, 60, RED), (60, 54, 80, 62, GREY)], [(45, 57, 4, GREEN)])
    skyB = ([(30, 53, 50, 62, YELLOW), (70, 52, 90, 58, BLUE)], [(15, 58, 4, PURPLE)])
    S["syn_sky_static_n24"] = (40, lambda i: _syn_window(skyA, 0, y0=200.0, noise=24, seed=i), None, "nochange")
    S["syn_sky_cut"] = (40, lambda i: (_syn_window(skyA, 0, y0=200.0, seed=i) if i < 20 else
                                       _syn_window(skyB, 0, y0=200.0, seed=i, ground=(70, 110, 70))), 20, "cut")
    S["syn_sky_cut_n24"] = (40, lambda i: (_syn_window(skyA, 0, y0=200.0, noise=24, seed=i) if i < 20 else
                                           _syn_window(skyB, 0, y0=200.0, noise=24, seed=i, ground=(70, 110, 70))),
                            20, "cut")
    # ---------------------------------------------------------- photos (canvas noise)
    full = dict(cx=1920, cy=1200, sw=3600)
    for p in (28, 29, 30, 31):
        for n in ((12, 24, 28) if p in (29, 31) else (12,)):
            S[f"ph{p}_static_n{n}"] = (40, lambda i, p=p, n=n: view(p, noise=n, seed=i, **full), None, "nochange")
    for p in (29, 31):
        for d in (8, 16, 32):                      # canvas px per frame at sw 1200 (3.125 src px per canvas px)
            S[f"ph{p}_pan_{d}"] = (40, lambda i, p=p, d=d: view(p, _pingpong(i, d * 3.125, 600, 3240), 1300, 1200,
                                                                12, i), None, "nochange")
        S[f"ph{p}_zoom_in3"] = (40, lambda i, p=p: view(p, 1920, 1300, 3600 * 0.97 ** i, 12, i), None, "nochange")
        S[f"ph{p}_zoom_out3"] = (40, lambda i, p=p: view(p, 1920, 1300, 1060 / 0.97 ** i, 12, i), None, "nochange")
        S[f"ph{p}_fast"] = (40, lambda i, p=p: view(p, _pingpong(i, 24 * 3.125, 800, 3040),
                                                    _pingpong(i, 20, 1000, 1500),
                                                    _pingpong(i, 0.03 * 2400, 1200, 2400), 24, i), None, "nochange")
    for a, b in ((29, 31), (31, 29), (28, 29), (28, 30), (30, 31), (29, 28)):
        S[f"ph_cut_{a}_{b}"] = (40, lambda i, a=a, b=b: view(a if i < 20 else b, noise=12, seed=i, **full), 20,
                                "cut")
        S[f"ph_cut_{a}_{b}_n24"] = (40, lambda i, a=a, b=b: view(a if i < 20 else b, noise=24, seed=i, **full),
                                    20, "cut")
    S["ph_pancut_29_31"] = (40, lambda i: view(29 if i < 20 else 31, _pingpong(i, 50, 600, 3240), 1300, 1200, 12, i),
                            20, "cut")
    S["ph_fastcut_31_29"] = (40, lambda i: view(31 if i < 20 else 29, _pingpong(i, 75, 800, 3040),
                                                _pingpong(i, 20, 1000, 1500), _pingpong(i, 72, 1200, 2400), 24, i),
                             20, "cut")
    # Sky-heavy tilt-up views: L0 owns the sky, the regions cover ~15-45 % of
    # the valid area (29 @ cy 600: 27 %; 31 @ cy 700 sw 2400: 14 %).
    sky29 = dict(cx=1920, cy=600, sw=1800)
    sky31 = dict(cx=1920, cy=700, sw=2400)
    sky28 = dict(cx=1920, cy=700, sw=2400)
    for nm, kw in (("29", sky29), ("31", sky31), ("28", sky28)):
        for n in (12, 24):
            S[f"sky{nm}_static_n{n}"] = (40, lambda i, kw=kw, n=n, nm=nm: view(int(nm), kw['cx'], kw['cy'],
                                                                                kw['sw'], n, i),
                                         None, "nochange")
        S[f"sky{nm}_pan_16"] = (40, lambda i, kw=kw, nm=nm: view(int(nm), _pingpong(i, 16 * kw['sw'] / 384, 1200, 2640),
                                                                 kw['cy'], kw['sw'], 12, i), None, "nochange")
    for (an, akw), (bn, bkw) in (((29, sky29), (31, sky31)), ((31, sky31), (29, sky29)), ((28, sky28), (29, sky29)),
                                 ((29, sky29), (28, sky28))):
        S[f"sky_cut_{an}_{bn}"] = (40, lambda i, an=an, akw=akw, bn=bn, bkw=bkw:
                                   (view(an, akw['cx'], akw['cy'], akw['sw'], 12, i) if i < 20 else
                                    view(bn, bkw['cx'], bkw['cy'], bkw['sw'], 12, i)), 20, "cut")
    # Same photo, sky-heavy, cut sideways to a different part of the treeline.
    S["sky_cut_29_side"] = (40, lambda i: view(29, 1200 if i < 20 else 3000, 600, 1800, 12, i), 20, "cut")
    # A tilt-up from the level view to the sky-heavy one (no cut: a camera move).
    S["ph29_tilt_up"] = (40, lambda i: view(29, 1920, max(600, 1300 - 25 * i), 1800, 12, i), None, "nochange")
    return S


SCENARIOS = build()
