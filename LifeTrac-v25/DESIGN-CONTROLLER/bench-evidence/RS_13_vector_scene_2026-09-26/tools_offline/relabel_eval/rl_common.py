"""Shared pieces of the A8 relabel-trigger evaluation: paths, photo views,
the feature probe and the probe encoder (the real VectorEncoder with only
the relabel decision swapped)."""
from __future__ import annotations

import os
import sys
import zlib

DC = os.environ.get("RL_DC", r"C:/Users/dorkm/Documents/GitHub/LifeTrac/.claude/worktrees/wf_adf2ad4c-341-2/LifeTrac-v25/DESIGN-CONTROLLER")
BS = os.path.join(DC, "base_station")
X8 = os.path.join(DC, "firmware", "tractor_x8")
for p in (BS, X8, os.path.join(BS, "tests")):
    if p not in sys.path:
        sys.path.insert(0, p)

import numpy as np  # noqa: E402
import cv2  # noqa: E402

import test_vector_sync as tvs  # noqa: E402
from x8_image_pipeline import encode_vector as ev  # noqa: E402
from x8_image_pipeline import vector_extract as vx  # noqa: E402

CW, CH = 384, 256
PHOTO_DIR = r"C:/Windows/Web/Wallpaper/ThemeC"
_PHOTOS: dict = {}


def photo(n: int) -> np.ndarray:
    if n not in _PHOTOS:
        im = cv2.imread(os.path.join(PHOTO_DIR, f"img{n}.jpg"))
        _PHOTOS[n] = cv2.cvtColor(im, cv2.COLOR_BGR2RGB)
    return _PHOTOS[n]


def add_noise(img: np.ndarray, noise: int, seed: int) -> np.ndarray:
    if not noise:
        return img
    rng = np.random.default_rng(seed)
    return np.clip(img.astype(np.int16) + rng.integers(-noise, noise + 1, img.shape), 0, 255).astype(np.uint8)


def view(n: int, cx: float, cy: float, sw: float, noise: int = 0, seed: int = 0) -> np.ndarray:
    """A 3:2 crop of photo n, sw source px wide centred on (cx, cy) (clamped
    inside the photo), INTER_AREA to the 384×256 canvas, noise at canvas size."""
    im = photo(n)
    H, W = im.shape[:2]
    sw = min(sw, W, H * 1.5)
    sh = sw / 1.5
    x0 = int(round(min(max(cx - sw / 2, 0), W - sw)))
    y0 = int(round(min(max(cy - sh / 2, 0), H - sh)))
    crop = im[y0:y0 + int(round(sh)), x0:x0 + int(round(sw))]
    can = cv2.resize(crop, (CW, CH), interpolation=cv2.INTER_AREA)
    return add_noise(can, noise, seed)


def shift_mask(m: np.ndarray, sx: int, sy: int) -> np.ndarray:
    return ev._shift_mask(m, sx, sy)


MC_RANGE = 8                     # the temporal matcher's local-motion range (working px)


def features(prev: np.ndarray, prev_lab: np.ndarray, cur: np.ndarray, regions: list, valid: np.ndarray,
             hz, mc: bool = True) -> dict:
    """Areas the candidate rules need, at 96×64 working px."""
    h, w = cur.shape
    np_, nc = int(prev.max()) + 2, int(cur.max()) + 2
    pair = np.bincount((prev.ravel() + 1) * nc + (cur.ravel() + 1), minlength=np_ * nc).reshape(np_, nc)
    area_c = pair.sum(axis=0)
    area_p = pair.sum(axis=1)
    labelled = float(area_c[1:].sum())
    prev_labelled = float(area_p[1:].sum())
    yy, xx = np.mgrid[0:h, 0:w]
    pid = prev.ravel() + 1
    cnt = np.bincount(pid, minlength=np_).astype(np.float64)
    pcx = np.bincount(pid, weights=xx.ravel().astype(np.float64), minlength=np_) / np.maximum(cnt, 1)
    pcy = np.bincount(pid, weights=yy.ravel().astype(np.float64), minlength=np_) / np.maximum(cnt, 1)
    relab = relab_mc = 0.0
    matched_prev = set()
    matched_prev_mc = set()
    n_regions = 0
    for r in regions:
        c = r.index + 1
        if c >= nc or area_c[c] == 0:
            continue
        n_regions += 1
        if np_ < 2:
            relab += float(area_c[c])
            relab_mc += float(area_c[c])
            continue
        p = int(np.argmax(pair[1:, c])) + 1
        inter = pair[p, c]
        union = area_c[c] + area_p[p] - inter
        if union > 0 and inter / union >= ev.MATCH_IOU \
                and float(vx.delta_e76(r.lab, prev_lab[p - 1])) <= ev.VERIFY_DE:
            matched_prev.add(p)
            matched_prev_mc.add(p)
            continue
        relab += float(area_c[c])
        cm = None
        ok = False
        for q in range(1, np_):
            if area_p[q] == 0 or float(vx.delta_e76(r.lab, prev_lab[q - 1])) > ev.VERIFY_DE:
                continue
            ddx = int(round(r.centroid[0] - pcx[q]))
            ddy = int(round(r.centroid[1] - pcy[q]))
            if max(abs(ddx), abs(ddy)) > MC_RANGE:
                continue
            if cm is None:
                cm = cur == r.index
            sh = shift_mask(prev == q - 1, ddx, ddy)
            it = int((sh & cm).sum())
            un = int(sh.sum()) + int(area_c[c]) - it
            if un > 0 and it / un >= ev.MATCH_IOU:
                ok = True
                matched_prev_mc.add(q)
                break
        if not ok:
            relab_mc += float(area_c[c])
    # Pixel colour consistency: a labelled pixel is changed when it was not
    # labelled before or its region's colour moved by more than VERIFY_DE.
    curm = cur >= 0
    prevm = prev >= 0
    lab_c = np.zeros((h, w, 3), np.float32)
    lab_p = np.zeros((h, w, 3), np.float32)
    if regions:
        cl = np.stack([r.lab for r in regions]).astype(np.float32)
        lab_c[curm] = cl[cur[curm]]
    if len(prev_lab):
        lab_p[prevm] = prev_lab[prev[prevm]]
    de_px = np.sqrt(((lab_c - lab_p) ** 2).sum(axis=2))
    chg = curm & (~prevm | (de_px > ev.VERIFY_DE))
    pix_chg = float(chg.sum())
    both = curm & prevm
    pix_chg_both = float((both & (de_px > ev.VERIFY_DE)).sum())
    # The same after the best global shift within the UPD range (screening only).
    best = pix_chg
    best_s = (0, 0)
    rng_mc = range(-MC_RANGE, MC_RANGE + 1) if mc else range(0)
    for dy in rng_mc:
        for dx in range(-MC_RANGE, MC_RANGE + 1):
            if dx == 0 and dy == 0:
                continue
            ys0, ys1 = max(0, dy), min(h, h + dy)
            xs0, xs1 = max(0, dx), min(w, w + dx)
            pm = np.zeros((h, w), bool)
            pl = np.zeros((h, w, 3), np.float32)
            pm[ys0:ys1, xs0:xs1] = prevm[ys0 - dy:ys1 - dy, xs0 - dx:xs1 - dx]
            pl[ys0:ys1, xs0:xs1] = lab_p[ys0 - dy:ys1 - dy, xs0 - dx:xs1 - dx]
            d = np.sqrt(((lab_c - pl) ** 2).sum(axis=2))
            n = float((curm & (~pm | (d > ev.VERIFY_DE))).sum())
            if n < best:
                best, best_s = n, (dx, dy)
    # Region coverage: a region is relabelled when under half of its pixels
    # lay on a previous region of its colour (split / merge tolerant).
    relab_cov = 0.0
    for r in regions:
        m = cur == r.index
        a = int(m.sum())
        if a == 0:
            continue
        pm = m & prevm
        ok = 0
        if pm.any():
            d = np.sqrt(((lab_p[pm] - r.lab[None, :]) ** 2).sum(axis=1))
            ok = int((d <= ev.VERIFY_DE).sum())
        if ok * 2 < a:
            relab_cov += a
    gone = float(sum(area_p[q] for q in range(1, np_) if q not in matched_prev))
    gone_mc = float(sum(area_p[q] for q in range(1, np_) if q not in matched_prev_mc))
    valid_a = float(valid.sum())
    l1v = valid.copy()
    if hz.found and hz.sky is not None:
        ys = np.arange(h, dtype=np.float32)[:, None] + 0.5
        above = ys < hz.line_work(w)[None, :]
        l1v &= ~(hz.sky & above)
    return {"relab": relab, "relab_mc": relab_mc, "labelled": labelled, "prev_labelled": prev_labelled,
            "valid": valid_a, "l1valid": float(l1v.sum()), "gone": gone, "gone_mc": gone_mc,
            "n_regions": n_regions, "hz_found": bool(hz.found), "pix_chg": pix_chg,
            "pix_chg_both": pix_chg_both, "both": float(both.sum()), "pix_mc": best, "mc_shift": best_s,
            "relab_cov": relab_cov}


def _safe(a, b):
    return a / b if b > 0 else 0.0


# ------------------------------------------------------------ candidate rules
# Each takes the feature dict and says whether the relabel trigger fires.

def r_lab(t=0.40):
    return lambda f: _safe(f["relab"], f["labelled"]) > t


def r_val(t=0.40):
    return lambda f: _safe(f["relab"], f["valid"]) > t


def r_l1v(t=0.40):
    return lambda f: _safe(f["relab"], f["l1valid"]) > t


def r_lab_minval(x, t=0.40):
    return lambda f: _safe(f["relab"], f["labelled"]) > t and _safe(f["relab"], f["valid"]) >= x


def r_mc_lab(t=0.40):
    return lambda f: _safe(f["relab_mc"], f["labelled"]) > t


def r_mc_lab_minval(x, t=0.40):
    return lambda f: _safe(f["relab_mc"], f["labelled"]) > t and _safe(f["relab_mc"], f["valid"]) >= x


def r_sym(t=0.40):
    """Relabelled + vanished area over the union of both captures' labelled area."""
    return lambda f: _safe(f["relab"] + f["gone"], f["labelled"] + f["prev_labelled"]) > t


def r_mc_sym(t=0.40):
    return lambda f: _safe(f["relab_mc"] + f["gone_mc"], f["labelled"] + f["prev_labelled"]) > t


def r_cov(t=0.40):
    return lambda f: _safe(f["relab_cov"], f["labelled"]) > t


def r_pix(t=0.40):
    return lambda f: _safe(f["pix_chg"], f["labelled"]) > t


def r_pixmc(t=0.40):
    return lambda f: _safe(f["pix_mc"], f["labelled"]) > t


RULES = {
    "lab40 (today)": r_lab(0.40),
    "val40": r_val(0.40),
    "l1v40": r_l1v(0.40),
    "lab50": r_lab(0.50),
    "lab60": r_lab(0.60),
    "lab40&val>=10": r_lab_minval(0.10),
    "lab40&val>=15": r_lab_minval(0.15),
    "lab40&val>=20": r_lab_minval(0.20),
    "mc_lab40": r_mc_lab(0.40),
    "mc_lab40&val>=10": r_mc_lab_minval(0.10),
    "mc_lab50": r_mc_lab(0.50),
    "sym40": r_sym(0.40),
    "mc_sym40": r_mc_sym(0.40),
    "cov40": r_cov(0.40),
    "cov50": r_cov(0.50),
    "pix40": r_pix(0.40),
    "pix50": r_pix(0.50),
    "pix60": r_pix(0.60),
    "pixmc50": r_pixmc(0.50),
}


class ProbeEncoder(ev.VectorEncoder):
    """The real encoder; only the relabel decision is the rule under test.
    Logs the features of every frame (whether or not the rule was asked)."""

    def __init__(self, rule, mc: bool = True, **kw):
        super().__init__(**kw)
        self._mc = mc
        self._rule = rule
        self._feat = None
        self.feat_log: list = []

    def _epoch_trigger(self, epoch_start, hz, region_map, regions, now):
        prev = self._prev_region_map
        self._feat = (features(prev, self._prev_region_lab, region_map, regions, self._valid, hz, self._mc)
                      if prev is not None else None)
        self.feat_log.append(self._feat)
        return super()._epoch_trigger(epoch_start, hz, region_map, regions, now)

    def _relabelled_fraction(self, prev, prev_lab, cur, regions):     # noqa: D401 (instance override)
        return 1.0 if (self._feat is not None and self._rule(self._feat)) else 0.0


def stable_seed(name: str) -> int:
    return zlib.crc32(name.encode()) & 0x7FFFFFFF
