"""Layer extraction for the VS1 vector-scene encoder (``VECTOR_SCENE.md`` §2).

Turns one working-resolution RGB image into the *measured* ingredients that
the temporal stage in ``encode_vector.py`` turns into records: the L0
horizon with its sky and ground bands (§2.4), the L1 colour regions with
their polygons, fills and gradients (§2.5), and the L2 tree/shrub test
(§2.6). Everything is numpy + OpenCV and knows nothing about ids, epochs or
budgets. The base never imports this module (§7.1: the tractor extracts,
the base only decodes).

Geometry conventions. The working image is the canvas divided by
``WORK_SCALE`` (96×64 for the 384×256 canvas), so one working pixel is one
cell of the 4 px grid and two working pixels are one cell of the 8 px
grid; every record coordinate is derived from working pixels with those two
factors. The horizon is fitted in canvas pixels because the §3.3 HZN codes
are in canvas units (2 px, 0.5°, 4 px).

Colour conventions. Means are measured (principle 4): a fill is a static
palette slot only when the slot is within ΔE76 ≤ 4 of the measured mean,
else the mean quantised to RGB444. The renderer's gradient rules of §3.3
(``dl3``: ±ΔL at the bounding-box extremes along ``dir3``; ``dl4``: +ΔL at the
top of a band and −ΔL at its bottom) are inverted here by least squares.
"""
from __future__ import annotations

import math
import os
import sys
from dataclasses import dataclass, field
from typing import Optional

import numpy as np

try:                                             # pragma: no cover
    import cv2                                   # type: ignore
except ImportError:                              # pragma: no cover
    cv2 = None                                   # type: ignore

try:
    from image_pipeline.vector_scene import codec as vs
except ImportError:                              # camera_service has no base_station on sys.path (§7.1)
    _BS_DIR = os.path.normpath(os.path.join(
        os.path.dirname(os.path.abspath(__file__)), "..", "..", "..", "base_station"))
    if _BS_DIR not in sys.path:
        sys.path.insert(0, _BS_DIR)
    from image_pipeline.vector_scene import codec as vs  # noqa: E402

CANVAS_W, CANVAS_H = 384, 256
WORK_W, WORK_H = 96, 64
WORK_SCALE = CANVAS_W // WORK_W          # canvas px per working px (4 px grid)
CELL8 = 2                                # working px per 8 px cell

# --- L0 (§2.4) ---------------------------------------------------------------
HZN_COLS = 24                            # per-column scores on 24 column bands
HZN_TOL_WORK = 2.0                       # "within 2 cells" at 96×64 → 2 working px
HZN_MIN_SUPPORT = 0.40                   # ≥ 40 % of the non-excluded columns
HZN_MIN_COLUMNS = 3
HZN_QUAD_MIN_COLUMNS = 8                 # fit the sag only with enough spread
SKY_BR_MIN = 12.0                        # B − R above this is chroma sky evidence
SKY_NEUTRAL_SPREAD = 28.0                # max − min below this and ...
SKY_NEUTRAL_LUMA = 150.0                 # ... bright: neutral overcast sky
SKY_TEXTURE_MAX = 10.0                   # local luma std above this is not sky
VEG_EXG_MIN = 20.0                       # 2G − R − B above this is vegetation, never sky
SKY_PURITY_ABOVE = 0.5                   # sky share above the split ...
SKY_PURITY_BELOW = 0.3                   # ... and below it, for a usable column

# --- L1 / L2 (§2.5, §2.6) ---------------------------------------------------
MIN_REGION_AREA = 6                      # cells at 96×64 outside the corridor (§2.3)
MAX_REGIONS = 40
POLY_EPSILON_WORK = 2.0                  # approxPolyDP ε = 1 cell of the 8 px grid
POLY_MAX_VERTICES = 10
PALETTE_DE_MAX = 4.0                     # ΔE76 for a palette slot (§3.3)
TREE_MAX_HALF_WORK = 8                   # rx/ry codes reach (7+1)·4 px = 8 working px
SHRUB_MAX_AREA = 60                      # cells, below the horizon (§2.6)
TREE_MIN_SOLIDITY = 0.8
TREE_MIN_ELLIPSE_IOU = 0.8
VEG_TREE_MARGIN = 8.0                    # mean G exceeds R and B by this: vegetation


# ---------------------------------------------------------------- colour

def rgb_to_lab(rgb: np.ndarray) -> np.ndarray:
    """sRGB 0..255 (…×3) → CIELAB (D65), float32."""
    c = np.asarray(rgb, dtype=np.float32) / 255.0
    lin = np.where(c <= 0.04045, c / 12.92, ((c + 0.055) / 1.055) ** 2.4)
    r, g, b = lin[..., 0], lin[..., 1], lin[..., 2]
    x = (0.4124 * r + 0.3576 * g + 0.1805 * b) / 0.95047
    y = 0.2126 * r + 0.7152 * g + 0.0722 * b
    z = (0.0193 * r + 0.1192 * g + 0.9505 * b) / 1.08883
    xyz = np.stack([x, y, z], axis=-1)
    f = np.where(xyz > 0.008856, np.cbrt(xyz), 7.787 * xyz + 16.0 / 116.0)
    lum = 116.0 * f[..., 1] - 16.0
    a = 500.0 * (f[..., 0] - f[..., 1])
    bb = 200.0 * (f[..., 1] - f[..., 2])
    return np.stack([lum, a, bb], axis=-1).astype(np.float32)


def delta_e76(lab1: np.ndarray, lab2: np.ndarray) -> np.ndarray:
    return np.linalg.norm(np.asarray(lab1, np.float32) - np.asarray(lab2, np.float32), axis=-1)


def luma(rgb: np.ndarray) -> np.ndarray:
    rgb = np.asarray(rgb, dtype=np.float32)
    return 0.299 * rgb[..., 0] + 0.587 * rgb[..., 1] + 0.114 * rgb[..., 2]


def rgb444_code(rgb8) -> int:
    q = np.clip(np.round(np.asarray(rgb8, dtype=np.float32) / 17.0), 0, 15).astype(int)
    return (int(q[0]) << 8) | (int(q[1]) << 4) | int(q[2])


def rgb444_to_rgb8(code: int) -> np.ndarray:
    return np.array([(code >> 8) & 15, (code >> 4) & 15, code & 15], dtype=np.float32) * 17.0


PALETTE_RGB8 = np.stack([rgb444_to_rgb8(c) for c in vs.STATIC_PALETTE])
PALETTE_LAB = rgb_to_lab(PALETTE_RGB8)


def choose_colour(mean_rgb8) -> tuple:
    """(palette slot or None, rgb444 code, shown 8-bit colour) for a measured mean.
    A slot is used only within ΔE76 ≤ 4 of the mean (§3.3)."""
    mean = np.asarray(mean_rgb8, dtype=np.float32)
    de = delta_e76(PALETTE_LAB, rgb_to_lab(mean)[None, :])
    slot = int(np.argmin(de))
    if de[slot] <= PALETTE_DE_MAX:
        return slot, vs.STATIC_PALETTE[slot], PALETTE_RGB8[slot].copy()
    code = rgb444_code(mean)
    return None, code, rgb444_to_rgb8(code)


def fill_base_rgb444(f) -> int:
    """The base colour of a FILL/vfill as RGB444 (palette slots expanded)."""
    return f.rgb444 if f.rgb444 is not None else vs.STATIC_PALETTE[f.palette]


def fill_shown_rgb8(f) -> np.ndarray:
    return rgb444_to_rgb8(fill_base_rgb444(f))


def make_fill(mean_rgb8, grad=None):
    slot, code, _ = choose_colour(mean_rgb8)
    if slot is not None:
        return vs.Fill(palette=slot, grad=grad)
    return vs.Fill(rgb444=code, grad=grad)


def make_vfill(mean_rgb8, dl: int):
    slot, code, _ = choose_colour(mean_rgb8)
    dl = max(-8, min(7, int(dl)))
    if slot is not None:
        return vs.VFill(palette=slot, dl=dl)
    return vs.VFill(rgb444=code, dl=dl)


def trimmed_mean_rgb(pixels: np.ndarray) -> np.ndarray:
    """Per-channel mean of the 10–90 % band; the plain mean for tiny samples."""
    if len(pixels) == 0:
        return np.zeros(3, np.float32)
    if len(pixels) < 20:
        return pixels.mean(axis=0).astype(np.float32)
    lo = np.percentile(pixels, 10, axis=0)
    hi = np.percentile(pixels, 90, axis=0)
    return np.clip(pixels, lo, hi).mean(axis=0).astype(np.float32)


def band_trend(lum: np.ndarray, t: np.ndarray) -> float:
    """ΔL (8-bit luma) for a vfill: the band is base + ΔL at t = 0 (its top)
    and base − ΔL at t = 1 (its bottom), so ΔL = −slope / 2 of L(t)."""
    if len(lum) < 8:
        return 0.0
    tm = t - t.mean()
    var = float((tm * tm).sum())
    if var <= 1e-6:
        return 0.0
    slope = float((tm * (lum - lum.mean())).sum() / var)
    return -slope / 2.0


# ---------------------------------------------------------------- L0

@dataclass
class Horizon:
    """One L0 measurement. Canvas units: ``y0`` at x = 192, ``slope`` = dy/dx,
    ``sag`` at the frame edge (|x − 192| = 192). ``top``/``bottom`` are the
    band means (8-bit RGB) and ``top_dl``/``bottom_dl`` the ΔL of §3.3
    (sky/ground when ``found``, upper/lower half otherwise)."""
    found: bool
    y0: float = 0.0
    slope: float = 0.0
    sag: float = 0.0
    support: float = 0.0
    columns: int = 0
    top: np.ndarray = field(default_factory=lambda: np.zeros(3, np.float32))
    bottom: np.ndarray = field(default_factory=lambda: np.zeros(3, np.float32))
    top_dl: float = 0.0
    bottom_dl: float = 0.0
    sky: Optional[np.ndarray] = None             # bool H×W sky-like pixels

    @property
    def angle_deg(self) -> float:
        return math.degrees(math.atan(self.slope))

    def line_canvas(self, x_canvas: np.ndarray) -> np.ndarray:
        u = (np.asarray(x_canvas, np.float32) - CANVAS_W / 2) / (CANVAS_W / 2)
        return self.y0 + self.slope * (CANVAS_W / 2) * u + self.sag * u * u

    def line_work(self, width: int = WORK_W) -> np.ndarray:
        """Horizon y in working px for each working column (pixel centres)."""
        xc = (np.arange(width, dtype=np.float32) + 0.5) * WORK_SCALE
        return self.line_canvas(xc) / WORK_SCALE


def horizon_from_codes(y: int, ang: int, curv: int) -> "Horizon":
    """The horizon the base draws for HZN ABS codes (§3.3)."""
    return Horizon(True, y0=2.0 * y - 128.0, slope=math.tan(math.radians(0.5 * ang)),
                   sag=4.0 * curv)


def sky_indicator(rgb: np.ndarray, valid: np.ndarray) -> np.ndarray:
    """Per-pixel sky evidence on chroma and texture, never luma alone (§2.4)."""
    f = rgb.astype(np.float32)
    r, g, b = f[..., 0], f[..., 1], f[..., 2]
    lum = 0.299 * r + 0.587 * g + 0.114 * b
    spread = f.max(axis=2) - f.min(axis=2)
    chroma = (b - r > SKY_BR_MIN) | ((spread < SKY_NEUTRAL_SPREAD) & (lum > SKY_NEUTRAL_LUMA))
    veg = (2 * g - r - b) > VEG_EXG_MIN
    # Texture over a horizontal 5×1 window: a vertical window would see the
    # sky/ground edge itself and strip the last sky row (a half-cell bias).
    mean = cv2.boxFilter(lum, -1, (5, 1), normalize=True, borderType=cv2.BORDER_REFLECT)
    mean_sq = cv2.boxFilter(lum * lum, -1, (5, 1), normalize=True, borderType=cv2.BORDER_REFLECT)
    std = np.sqrt(np.maximum(mean_sq - mean * mean, 0.0))
    return chroma & ~veg & (std < SKY_TEXTURE_MAX) & valid


def _fit_horizon_points(u: np.ndarray, y: np.ndarray, quadratic: bool) -> tuple:
    if quadratic:
        a = np.stack([np.ones_like(u), u, u * u], axis=1)
    else:
        a = np.stack([np.ones_like(u), u], axis=1)
    coef, *_ = np.linalg.lstsq(a, y, rcond=None)
    return tuple(float(c) for c in coef) + ((0.0,) if not quadratic else ())


def extract_horizon(rgb: np.ndarray, valid: np.ndarray) -> Horizon:
    """L0 for one working image: per-column sky→ground split on chroma and
    texture, a two-pass trimmed least-squares line (quadratic before
    calibration, the sag in canvas px at the frame edge), acceptance on
    support and purity, and the band colours. ``found = False`` is the
    NO_HORIZON state: the expected result with the camera tilted down."""
    h, w = valid.shape
    sky = sky_indicator(rgb, valid)
    band = max(1, w // HZN_COLS)
    ncol = w // band
    s = sky[:, :ncol * band].reshape(h, ncol, band).sum(axis=2).astype(np.int32)
    nv = valid[:, :ncol * band].reshape(h, ncol, band).sum(axis=2).astype(np.int32)
    # score(r) = sky rows above r + non-sky rows from r down, for every split r = 0..h
    above = np.vstack([np.zeros((1, ncol), np.int32), np.cumsum(s, axis=0)])
    nonsky_above = np.vstack([np.zeros((1, ncol), np.int32), np.cumsum(nv - s, axis=0)])
    score = above + (nonsky_above[-1][None, :] - nonsky_above)
    split = np.argmax(score, axis=0)
    cols = np.arange(ncol)
    valid_px = nv.sum(axis=0)
    excluded = valid_px < (h * band) // 2               # mostly masked columns (the hood)
    sky_above = above[split, cols]
    nv_above = np.vstack([np.zeros((1, ncol), np.int32), np.cumsum(nv, axis=0)])[split, cols]
    sky_below = above[-1] - sky_above
    nv_below = valid_px - nv_above
    usable = (~excluded) & (split >= 1) & (split <= h - 1) & (nv_above > 0) & (nv_below > 0)
    with np.errstate(divide="ignore", invalid="ignore"):
        purity_above = np.where(nv_above > 0, sky_above / np.maximum(nv_above, 1), 0.0)
        purity_below = np.where(nv_below > 0, sky_below / np.maximum(nv_below, 1), 1.0)
    usable &= (purity_above >= SKY_PURITY_ABOVE) & (purity_below <= SKY_PURITY_BELOW)
    hz = Horizon(False, sky=sky)
    n_considered = int((~excluded).sum())
    if usable.sum() >= HZN_MIN_COLUMNS and n_considered > 0:
        xc = (cols[usable] * band + band / 2.0) * WORK_SCALE
        u = ((xc - CANVAS_W / 2) / (CANVAS_W / 2)).astype(np.float64)
        y = (split[usable] * WORK_SCALE).astype(np.float64)   # boundary between rows
        fit = _robust_horizon_fit(u, y)
        if fit is not None:
            a0, b0, c0, inliers = fit
            support = inliers / n_considered
            if support >= HZN_MIN_SUPPORT:
                hz = Horizon(True, y0=a0, slope=b0 / (CANVAS_W / 2), sag=c0,
                             support=support, columns=inliers, sky=sky)
    _measure_bands(hz, rgb, valid)
    return hz


def _robust_horizon_fit(u: np.ndarray, y: np.ndarray) -> Optional[tuple]:
    """Line through the largest consensus of columns, then a two-pass trimmed
    least squares (quadratic with enough columns). A shed or tree line that
    rises above the horizon in a minority of columns must not tilt the
    line: those columns are the skyline (§2.4 item 7), not the horizon.
    Returns (y0, b, sag, inlier count) with b in canvas px per unit u."""
    tol = HZN_TOL_WORK * WORK_SCALE
    n = len(u)
    i, j = np.triu_indices(n, 1)
    du = u[j] - u[i]
    ok = np.abs(du) > 1e-6
    i, j = i[ok], j[ok]
    if len(i) == 0:
        return None
    slope = (y[j] - y[i]) / (u[j] - u[i])
    icpt = y[i] - slope * u[i]
    res = np.abs(y[None, :] - (icpt[:, None] + slope[:, None] * u[None, :]))
    # The consensus is counted at one 4 px cell: at the 2-cell acceptance
    # tolerance a diagonal through two groups of columns at different heights
    # collects half of each and beats the true line.
    tight = tol / 2.0
    inl = res <= tight
    score = inl.sum(axis=1) - res.clip(max=tight).sum(axis=1) / (tight * n)   # count, then closeness
    keep = inl[int(np.argmax(score))]
    if keep.sum() < HZN_MIN_COLUMNS:
        return None
    a0, b0, c0 = _fit_horizon_points(u[keep], y[keep], False)
    # A pre-calibration horizon sags up to ~27 px at the frame edge (§2.4), so
    # the quadratic pass admits columns within 2·tol of the consensus line.
    near = np.abs(y - (a0 + b0 * u)) <= 2 * tol
    if near.sum() >= HZN_MIN_COLUMNS:
        a0, b0, c0 = _fit_horizon_points(u[near], y[near], near.sum() >= HZN_QUAD_MIN_COLUMNS)
    final = np.abs(y - (a0 + b0 * u + c0 * u * u)) <= tol
    if final.sum() >= HZN_MIN_COLUMNS:
        a0, b0, c0 = _fit_horizon_points(u[final], y[final], final.sum() >= HZN_QUAD_MIN_COLUMNS)
    inliers = int((np.abs(y - (a0 + b0 * u + c0 * u * u)) <= tol).sum())
    return a0, b0, c0, inliers


def _measure_bands(hz: Horizon, rgb: np.ndarray, valid: np.ndarray) -> None:
    h, w = valid.shape
    f = rgb.astype(np.float32)
    ys = np.arange(h, dtype=np.float32)[:, None] + 0.5
    if hz.found:
        line = np.clip(hz.line_work(w), 0.5, h - 0.5)[None, :]
    else:
        line = np.full((1, w), h / 2.0, np.float32)
    top_t = ys / line                                   # 0 at the top, 1 at the split
    bot_t = (ys - line) / np.maximum(h - line, 1e-3)
    top = (ys < line) & valid
    bot = (ys >= line) & valid
    if hz.found and hz.sky is not None and (top & hz.sky).sum() >= 8:
        top_pix = top & hz.sky                          # measure the sky, not the treeline
    else:
        top_pix = top
    for sel, t, name in ((top_pix, top_t, "top"), (bot, bot_t, "bottom")):
        px = f[sel]
        if len(px) == 0:
            px = f[valid] if valid.any() else f.reshape(-1, 3)
            t_sel = np.full(len(px), 0.5, np.float32)
        else:
            t_sel = np.broadcast_to(t, (h, w))[sel]
        mean = trimmed_mean_rgb(px)
        dl = band_trend(luma(px), t_sel.astype(np.float32))
        setattr(hz, name, mean)
        setattr(hz, name + "_dl", dl)


def render_l0(hz: Horizon, top_vf, bot_vf, shape=(WORK_H, WORK_W)) -> np.ndarray:
    """The L0 layer as the base draws it (flat vfills with the vertical trend)."""
    h, w = shape
    ys = np.arange(h, dtype=np.float32)[:, None] + 0.5
    if hz.found:
        line = np.clip(hz.line_work(w), 0.0, float(h))[None, :]
    else:
        line = np.full((1, w), h / 2.0, np.float32)
    out = np.empty((h, w, 3), np.float32)
    top_t = np.clip(ys / np.maximum(line, 1e-3), 0.0, 1.0)
    bot_t = np.clip((ys - line) / np.maximum(h - line, 1e-3), 0.0, 1.0)
    top_c = fill_shown_rgb8(top_vf)[None, None, :] + (8.0 * top_vf.dl) * (1.0 - 2.0 * top_t)[..., None]
    bot_c = fill_shown_rgb8(bot_vf)[None, None, :] + (8.0 * bot_vf.dl) * (1.0 - 2.0 * bot_t)[..., None]
    above = (ys < line)[..., None]
    out[:] = np.where(above, top_c, bot_c)
    return np.clip(out, 0.0, 255.0)


# ---------------------------------------------------------------- L1 / L2

@dataclass
class Region:
    """One connected colour region of the working image."""
    index: int
    area: int
    mean: np.ndarray                              # 8-bit RGB mean
    centroid: tuple                               # (x, y) working px
    bbox: tuple                                   # (x0, y0, x1, y1) inclusive, working px
    fill: object = None                           # vs.Fill (mean colour + gradient)
    poly: Optional[tuple] = None                  # 8 px grid vertices, or None
    tree: Optional[tuple] = None                  # (cx, cy, rx, ry) codes, or None
    above_horizon: bool = False
    lab: np.ndarray = field(default_factory=lambda: np.zeros(3, np.float32))
    raster: Optional[np.ndarray] = None           # what the define would paint (bool H×W)


class Segmenter:
    """k-means in Lab with the previous centres as the initial labels, so the
    cluster indices stay stable between captures (§2.5 item 1)."""

    def __init__(self, k: int = 8, iterations: int = 3):
        self.k = k
        self.iterations = iterations
        self._centres: Optional[np.ndarray] = None

    def reset(self) -> None:
        self._centres = None

    def labels(self, lab: np.ndarray, valid: np.ndarray) -> np.ndarray:
        """Cluster label per pixel (int32), −1 where not valid."""
        h, w = valid.shape
        out = np.full((h, w), -1, np.int32)
        data = np.ascontiguousarray(lab[valid], dtype=np.float32)
        n = len(data)
        if n == 0:
            return out
        k = min(self.k, n)
        criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_MAX_ITER, self.iterations, 1.0)
        if self._centres is not None and len(self._centres) == k:
            d = ((data[:, None, :] - self._centres[None, :, :]) ** 2).sum(axis=2)
            init = np.argmin(d, axis=1).astype(np.int32).reshape(-1, 1)
            _, lbl, centres = cv2.kmeans(data, k, init, criteria, 1, cv2.KMEANS_USE_INITIAL_LABELS)
        else:
            _, lbl, centres = cv2.kmeans(data, k, None, criteria, 1, cv2.KMEANS_PP_CENTERS)
        lbl = lbl.reshape(-1)
        # Merge clusters whose centres are indistinguishable (ΔE76 ≤ 4): a flat
        # colour split in two would otherwise become two identical shapes.
        remap = np.arange(k)
        for i in range(k):
            for j in range(i):
                if remap[j] == j and float(np.linalg.norm(centres[i] - centres[j])) <= PALETTE_DE_MAX:
                    remap[i] = j
                    break
        lbl = remap[lbl]
        self._centres = centres.astype(np.float32)
        out[valid] = lbl
        return out


def mode_filter(labels: np.ndarray, k: int) -> np.ndarray:
    """3×3 mode filter on a label map (§2.5 item 3); −1 pixels vote for nothing."""
    votes = []
    for j in range(k):
        ind = (labels == j).astype(np.float32)
        votes.append(cv2.boxFilter(ind, -1, (3, 3), normalize=False, borderType=cv2.BORDER_CONSTANT))
    if not votes:
        return labels
    stack = np.stack(votes, axis=0)
    best = np.argmax(stack, axis=0).astype(np.int32)
    return np.where(labels >= 0, best, -1)


def _vw_simplify(pts: np.ndarray, max_n: int) -> np.ndarray:
    """Visvalingam–Whyatt: drop the vertex of least triangle area until max_n."""
    pts = [tuple(p) for p in pts]
    while len(pts) > max_n:
        n = len(pts)
        best, best_area = 0, None
        for i in range(n):
            (x0, y0), (x1, y1), (x2, y2) = pts[i - 1], pts[i], pts[(i + 1) % n]
            area = abs((x1 - x0) * (y2 - y0) - (x2 - x0) * (y1 - y0))
            if best_area is None or area < best_area:
                best, best_area = i, area
        del pts[best]
    return np.array(pts, dtype=np.float32)


def _ring_area(pts) -> float:
    a = 0.0
    n = len(pts)
    for i in range(n):
        x0, y0 = pts[i]
        x1, y1 = pts[(i + 1) % n]
        a += x0 * y1 - x1 * y0
    return abs(a) / 2.0


def _vertex_touches_mask(mask: np.ndarray, cx: int, cy: int) -> bool:
    """A vertex on the 8 px grid sits at the corner of working pixel (2cx, 2cy);
    it is inside the mask if any of the four working pixels around it is."""
    h, w = mask.shape
    x0, x1 = max(0, 2 * cx - 1), min(w - 1, 2 * cx)
    y0, y1 = max(0, 2 * cy - 1), min(h - 1, 2 * cy)
    return bool(mask[y0:y1 + 1, x0:x1 + 1].any())


def polygon_of(component: np.ndarray, mask: Optional[np.ndarray]) -> Optional[tuple]:
    """Outer contour → Douglas–Peucker (ε = 1 cell) → VW to ≤ 10 vertices →
    8 px grid cells. Vertices that would fall inside the self-mask are moved
    to a clean neighbouring cell or dropped (no vertex may lie in the mask)."""
    contours, _ = cv2.findContours(component.astype(np.uint8), cv2.RETR_EXTERNAL,
                                   cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None
    contour = max(contours, key=cv2.contourArea)
    if len(contour) < 3:
        return None
    approx = cv2.approxPolyDP(contour, POLY_EPSILON_WORK, True).reshape(-1, 2).astype(np.float32)
    if len(approx) < 3:
        approx = contour.reshape(-1, 2).astype(np.float32)
    if len(approx) > POLY_MAX_VERTICES:
        approx = _vw_simplify(approx, POLY_MAX_VERTICES)
    # Contour points are boundary-pixel centres; +0.5 puts the ring on the
    # pixel edge so a region spanning px 10..19 becomes cells 5..10.
    cells = np.round((approx + 0.5) / CELL8).astype(int)
    cells[:, 0] = np.clip(cells[:, 0], 0, WORK_W // CELL8)      # the far edge (cell 48 / 32)
    cells[:, 1] = np.clip(cells[:, 1], 0, WORK_H // CELL8)      # is legal after v0 (§3.3)
    ring: list = []
    for cx, cy in cells:
        cx, cy = int(cx), int(cy)
        if mask is not None and _vertex_touches_mask(mask, cx, cy):
            moved = None
            for ddx, ddy in ((1, 0), (-1, 0), (0, 1), (0, -1), (1, 1), (-1, -1), (1, -1), (-1, 1)):
                nx, ny = cx + ddx, cy + ddy
                if 0 <= nx <= WORK_W // CELL8 and 0 <= ny <= WORK_H // CELL8 \
                        and not _vertex_touches_mask(mask, nx, ny):
                    moved = (nx, ny)
                    break
            if moved is None:
                continue
            cx, cy = moved
        if not ring or ring[-1] != (cx, cy):
            ring.append((cx, cy))
    if len(ring) > 1 and ring[0] == ring[-1]:
        ring.pop()
    if len(ring) > POLY_MAX_VERTICES:
        ring = [tuple(int(v) for v in p) for p in _vw_simplify(np.array(ring, np.float32), POLY_MAX_VERTICES)]
    if len(ring) < 3 or _ring_area(ring) < 0.5:
        return None
    # v0 must be inside the 8 px grid (x 0..47, y 0..31); rotate the ring so it is.
    start = next((i for i, (x, y) in enumerate(ring)
                  if x < WORK_W // CELL8 and y < WORK_H // CELL8), None)
    if start is None:
        return None
    return tuple(ring[start:] + ring[:start])


def gradient_of(coef: tuple, bbox: tuple) -> Optional[tuple]:
    """(dir3, dl3) of §3.3 from a fitted luma plane Y = a + b·x + c·y (working
    px): ΔL is the luma change from the centroid to the bounding-box extreme
    along the gradient. None when the quantised ΔL is 0."""
    _, b, c = coef
    mag = math.hypot(b, c)
    if mag < 1e-6:
        return None
    theta = math.degrees(math.atan2(c, b)) % 360.0
    half_w = (bbox[2] - bbox[0] + 1) / 2.0
    half_h = (bbox[3] - bbox[1] + 1) / 2.0
    rad = math.radians(theta)
    extent = half_w * abs(math.cos(rad)) + half_h * abs(math.sin(rad))
    dl = mag * extent
    if theta >= 180.0:                            # dir3 spans 0..157.5°; the sign carries the rest
        theta -= 180.0
        dl = -dl
    code = int(round(dl / 16.0))
    if code == 0:
        return None
    return int(round(theta / 22.5)) % 8, max(-4, min(3, code))


def ellipse_raster(cx: float, cy: float, rx: float, ry: float, shape=(WORK_H, WORK_W)) -> np.ndarray:
    out = np.zeros(shape, np.uint8)
    cv2.ellipse(out, (int(round(cx)), int(round(cy))), (max(1, int(round(rx))), max(1, int(round(ry)))),
                0, 0, 360, 1, -1)
    return out.astype(bool)


def tree_codes_of(component: np.ndarray, region: "Region", horizon_line: Optional[np.ndarray]) -> Optional[tuple]:
    """§2.6 tree/shrub test: a vegetation region that is compact (solidity ≥ 0.8,
    ellipse IoU ≥ 0.8), small enough for the rx/ry codes, and either reaches
    the horizon (tree) or is under 60 cells below it (shrub); a region that
    touches the bottom edge is the ground itself and stays L1."""
    r, g, b = (float(v) for v in region.mean)
    if not (g > r + VEG_TREE_MARGIN and g > b + VEG_TREE_MARGIN):
        return None
    x0, y0, x1, y1 = region.bbox
    half_w = (x1 - x0 + 1) / 2.0
    half_h = (y1 - y0 + 1) / 2.0
    if half_w > TREE_MAX_HALF_WORK or half_h > TREE_MAX_HALF_WORK or y1 >= WORK_H - 1:
        return None
    if horizon_line is not None:
        cx = int(round((x0 + x1) / 2.0))
        reaches = y0 <= horizon_line[min(max(cx, 0), len(horizon_line) - 1)]
    else:
        reaches = False
    if not reaches and region.area >= SHRUB_MAX_AREA:
        return None
    contours, _ = cv2.findContours(component.astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None
    contour = max(contours, key=cv2.contourArea)
    hull_area = cv2.contourArea(cv2.convexHull(contour))
    # Contour polygons run through boundary-pixel centres, so the hull area
    # undercounts by about half a pixel per boundary pixel: add it back.
    hull_area += 0.5 * len(contour)
    if hull_area <= 0 or region.area / hull_area < TREE_MIN_SOLIDITY:
        return None
    # Axis-aligned ellipse from the second moments (area-consistent, unlike
    # the bounding box on an eroded blob), tested against the analytic ellipse.
    m = cv2.moments(component.astype(np.uint8), binaryImage=True)
    if m["m00"] <= 0:
        return None
    cx, cy = m["m10"] / m["m00"], m["m01"] / m["m00"]
    ax = max(2.0 * math.sqrt(max(m["mu20"] / m["m00"], 0.0)), 0.5)
    ay = max(2.0 * math.sqrt(max(m["mu02"] / m["m00"], 0.0)), 0.5)
    if ax > TREE_MAX_HALF_WORK or ay > TREE_MAX_HALF_WORK:
        return None
    yy, xx = np.mgrid[0:component.shape[0], 0:component.shape[1]]
    ell = (((xx + 0.5 - (cx + 0.5)) / ax) ** 2 + ((yy + 0.5 - (cy + 0.5)) / ay) ** 2) <= 1.0
    inter = float((ell & component).sum())
    union = float((ell | component).sum())
    if union <= 0 or inter / union < TREE_MIN_ELLIPSE_IOU:
        return None
    cx8 = int(round((cx + 0.5) / CELL8))
    cy8 = int(round((cy + 0.5) / CELL8))
    rx = max(0, min(7, int(round(ax)) - 1))
    ry = max(0, min(7, int(round(ay)) - 1))
    if not (0 <= cx8 <= 47 and 0 <= cy8 <= 31):
        return None
    return cx8, cy8, rx, ry


def extract_regions(rgb: np.ndarray, lab: np.ndarray, valid: np.ndarray, mask: Optional[np.ndarray],
                    segmenter: Segmenter, horizon: Horizon) -> tuple:
    """L1/L2 candidates: (regions, region_map). ``region_map`` holds each pixel's
    region index or −1 (sky pixels above the horizon belong to L0, masked
    pixels to nobody). Regions are area-descending and capped at MAX_REGIONS."""
    h, w = valid.shape
    l1_valid = valid.copy()
    if horizon.found and horizon.sky is not None:
        # Sky above the horizon is L0; a sky-like (bright, neutral) wall below
        # it is a mass and stays in L1.
        ys = np.arange(h, dtype=np.float32)[:, None] + 0.5
        above = ys < horizon.line_work(w)[None, :]
        l1_valid &= ~(horizon.sky & above)
    labels = segmenter.labels(lab, l1_valid)
    labels = mode_filter(labels, segmenter.k)
    region_map = np.full((h, w), -1, np.int32)
    comps: list = []                                    # (area, label, cc_index, cc_labels, stats)
    for j in range(segmenter.k):
        ind = (labels == j).astype(np.uint8)
        if not ind.any():
            continue
        n, cc, stats, _ = cv2.connectedComponentsWithStats(ind, connectivity=4)
        for i in range(1, n):
            area = int(stats[i, cv2.CC_STAT_AREA])
            if area >= MIN_REGION_AREA:
                comps.append((area, j, i, cc, stats[i]))
    comps.sort(key=lambda c: -c[0])
    comps = comps[:MAX_REGIONS]
    regions: list = []
    hline = horizon.line_work(w) if horizon.found else None
    f = rgb.astype(np.float32)
    lum = luma(f)
    yy, xx = np.mgrid[0:h, 0:w]
    xx = xx.astype(np.float32)
    yy = yy.astype(np.float32)
    for idx, (area, _, i, cc, st) in enumerate(comps):
        comp = cc == i
        region_map[comp] = idx
        px = f[comp]
        mean = px.mean(axis=0)
        x0, y0 = int(st[cv2.CC_STAT_LEFT]), int(st[cv2.CC_STAT_TOP])
        x1, y1 = x0 + int(st[cv2.CC_STAT_WIDTH]) - 1, y0 + int(st[cv2.CC_STAT_HEIGHT]) - 1
        cx, cy = float(xx[comp].mean()), float(yy[comp].mean())
        # Luma plane by least squares over the region (§2.5 item 6).
        a = np.stack([np.ones(area, np.float32), xx[comp] - cx, yy[comp] - cy], axis=1)
        coef, *_ = np.linalg.lstsq(a, lum[comp], rcond=None)
        region = Region(idx, area, mean.astype(np.float32), (cx, cy), (x0, y0, x1, y1))
        region.lab = rgb_to_lab(mean)
        region.fill = make_fill(mean, gradient_of(tuple(float(c) for c in coef), region.bbox))
        region.above_horizon = bool(hline is not None and cy < hline[min(max(int(cx), 0), w - 1)])
        region.tree = tree_codes_of(comp, region, hline)
        if region.tree is None:
            region.poly = polygon_of(comp, mask)
        if region.tree is not None:
            cx8, cy8, rx, ry = region.tree
            region.raster = raster_of(vs.Tree(32, cx8, cy8, rx, ry, region.fill), 0, 0, (h, w))
        elif region.poly is not None:
            region.raster = raster_of(vs.Poly(1, 0, region.poly, region.fill), 0, 0, (h, w))
        if region.raster is not None:
            inter = int((region.raster & comp).sum())
            union = int((region.raster | comp).sum())
            if union == 0 or inter / union < 0.5:
                # A wire, post or rut is thinner than a cell: its polygon would
                # not cover it. It stays a region for L3 to describe as an edge.
                region.raster = region.poly = region.tree = None
        regions.append(region)
    return regions, region_map


# ---------------------------------------------------------------- L3 (§2.7)

EDGE_W, EDGE_H = 192, 128                # Canny resolution; 2 px per 4 px-grid cell
EDGE_TOP_K = 30
EDGE_MIN_PIXELS = 12                     # at 192×128 (6 cells)
EDGE_MIN_LENGTH_CELLS = 6.0              # polyline length on the 4 px grid
EDGE_EPSILON_PX = 1.5
EDGE_MAX_POINTS = 5                      # k2: at most 4 segments
EDGE_CANNY_FLOOR = (40.0, 80.0)          # L1 Sobel floors so a flat frame yields no edges
EDGE_CANNY_CAP = (64.0, 96.0)            # ... and caps so a blurred 40-luma line still seeds
OVERHEAD_MIN_CELLS = 24.0
OVERHEAD_MAX_SAG_CELLS = 2.0
CLS_RUT, CLS_STRUCTURE, CLS_OVERHEAD = 0, 1, 4


@dataclass
class EdgeCand:
    points: tuple                         # 2..5 (x, y) cells of the 4 px grid
    cls: int
    score: float                          # length (cells) × contrast
    raster: np.ndarray                    # bool H×W polyline at working resolution


def _open_polyline_vw(pts: np.ndarray, max_n: int) -> np.ndarray:
    pts = [tuple(p) for p in pts]
    while len(pts) > max_n:
        best, best_area = 1, None
        for i in range(1, len(pts) - 1):              # the endpoints stay
            (x0, y0), (x1, y1), (x2, y2) = pts[i - 1], pts[i], pts[i + 1]
            area = abs((x1 - x0) * (y2 - y0) - (x2 - x0) * (y1 - y0))
            if best_area is None or area < best_area:
                best, best_area = i, area
        del pts[best]
    return np.array(pts, dtype=np.float32)


def _trim_corner(pts: np.ndarray, max_len: float = 8.0) -> np.ndarray:
    while len(pts) > 2:
        a, b = pts[1] - pts[0], pts[2] - pts[1]
        la, lb = float(np.hypot(*a)), float(np.hypot(*b))
        if la >= max_len or la == 0 or lb == 0:
            break
        if float(a[0] * b[0] + a[1] * b[1]) / (la * lb) > 0.7071:   # turns by < 45°: real curve
            break
        pts = pts[1:]
    return pts


def polyline_of_component(component: np.ndarray) -> Optional[np.ndarray]:
    """Open polyline (≤ 5 points, 192×128 px) through a thin edge component:
    the outer contour of a 1 px curve runs out and back, so the arc between
    its two farthest points is one traverse of the curve."""
    contours, _ = cv2.findContours(component.astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    if not contours:
        return None
    c = max(contours, key=len).reshape(-1, 2).astype(np.float32)
    if len(c) < 2:
        return None
    step = max(1, len(c) // 120)                      # the farthest pair on ≤ ~120 samples
    sub = c[::step]
    d = ((sub[:, None, :] - sub[None, :, :]) ** 2).sum(axis=2)
    i, j = (int(v) * step for v in np.unravel_index(int(np.argmax(d)), d.shape))
    if i > j:
        i, j = j, i
    arc = c[i:j + 1]
    if len(arc) < 2:
        return None
    approx = cv2.approxPolyDP(arc.reshape(-1, 1, 2), EDGE_EPSILON_PX, False).reshape(-1, 2)
    # The arc starts and ends at corners of the (dilated) component: a short
    # end segment that turns sharply against its neighbour is that corner,
    # not the curve.
    approx = _trim_corner(approx)
    approx = _trim_corner(approx[::-1])[::-1]
    if len(approx) < 2:
        return None
    if len(approx) > EDGE_MAX_POINTS:
        approx = _open_polyline_vw(approx, EDGE_MAX_POINTS)
    # The arc runs along one side of the band; pull each vertex to the band's
    # local centre so the polyline sits on the edge, not beside it.
    h, w = component.shape
    snapped = []
    for x, y in approx:
        x0, x1 = max(0, int(x) - 3), min(w, int(x) + 4)
        y0, y1 = max(0, int(y) - 3), min(h, int(y) + 4)
        ys, xs = np.nonzero(component[y0:y1, x0:x1])
        snapped.append((x0 + xs.mean(), y0 + ys.mean()) if len(xs) else (x, y))
    return np.array(snapped, np.float32)


def extract_edges(rgb192: np.ndarray, mask: Optional[np.ndarray], regions: list, shape: tuple,
                  horizon: Horizon) -> list:
    """L3 candidates: Canny at 192×128 with thresholds from the 90th/97th
    percentiles of the Sobel magnitude (§2.7), minus the outlines L1 already
    draws (the polygon and ellipse rasters of ``regions``) and edges inside
    the self-mask; the top-K components by length × contrast become
    ≤ 4-segment polylines on the 4 px grid with a class from the horizon side."""
    gray = cv2.GaussianBlur(cv2.cvtColor(rgb192, cv2.COLOR_RGB2GRAY), (3, 3), 0)
    gx = cv2.Sobel(gray, cv2.CV_32F, 1, 0, ksize=3)
    gy = cv2.Sobel(gray, cv2.CV_32F, 0, 1, ksize=3)
    mag = np.abs(gx) + np.abs(gy)                      # Canny's default L1 norm
    # Percentiles over every pixel, clamped: on a natural image almost all
    # pixels carry sensor noise and the percentiles track it; on a frame that
    # is mostly flat they would land on the strongest edges and a visible rut
    # could never seed the hysteresis, so a 40-luma step always can.
    lo = min(max(float(np.percentile(mag, 90)), EDGE_CANNY_FLOOR[0]), EDGE_CANNY_CAP[0])
    hi = min(max(float(np.percentile(mag, 97)), EDGE_CANNY_FLOOR[1], lo + 1.0), EDGE_CANNY_CAP[1])
    edges = cv2.Canny(gray, lo, hi)
    h, w = shape
    drawn = np.zeros((h, w), np.int32)                 # label map of what L1 paints
    for r in sorted((r for r in regions if r.raster is not None), key=lambda r: -r.area):
        drawn[r.raster] = r.index + 1
    boundary = np.zeros((h, w), np.uint8)
    boundary[1:, :] |= (drawn[1:, :] != drawn[:-1, :]).astype(np.uint8)
    boundary[:, 1:] |= (drawn[:, 1:] != drawn[:, :-1]).astype(np.uint8)
    if horizon.found:                                  # the sky/ground step is L0's
        line = np.round(horizon.line_work(w)).astype(np.int32)
        pts = np.stack([np.arange(w, dtype=np.int32), np.clip(line, 0, h - 1)], axis=1)
        cv2.polylines(boundary, [pts.reshape(-1, 1, 2)], False, 1, 1)
    suppress = cv2.dilate(boundary, np.ones((3, 3), np.uint8))
    if mask is not None:
        suppress |= cv2.dilate(mask.astype(np.uint8), np.ones((3, 3), np.uint8))
    suppress = cv2.resize(suppress, (edges.shape[1], edges.shape[0]), interpolation=cv2.INTER_NEAREST)
    edges[suppress > 0] = 0
    # A 1-cell feature gives Canny a double, gappy edge at 2 px per cell: one
    # dilation joins it into a single component before the top-K cut.
    edges = cv2.dilate(edges, np.ones((3, 3), np.uint8))
    n, cc, stats, _ = cv2.connectedComponentsWithStats((edges > 0).astype(np.uint8), connectivity=8)
    if n <= 1:
        return []
    ys, xs = np.nonzero(edges)
    lbl = cc[ys, xs]
    contrast = np.bincount(lbl, weights=mag[ys, xs], minlength=n) / np.maximum(np.bincount(lbl, minlength=n), 1)
    comps = [(int(stats[i, cv2.CC_STAT_AREA]) * float(contrast[i]) / 255.0, i)
             for i in range(1, n) if stats[i, cv2.CC_STAT_AREA] >= EDGE_MIN_PIXELS]
    comps.sort(reverse=True)
    hline = horizon.line_work(w) if horizon.found else None
    out: list = []
    for _, i in comps[:EDGE_TOP_K]:
        poly = polyline_of_component(cc == i)
        if poly is None:
            continue
        cells = np.round(poly / (EDGE_W // w)).astype(int)
        pts: list = []
        for x, y in cells:
            p = (int(min(max(x, 0), w - 1)), int(min(max(y, 0), h - 1)))
            if not pts or pts[-1] != p:
                pts.append(p)
        if len(pts) < 2:
            continue
        seg = np.diff(np.array(pts, np.float32), axis=0)
        length = float(np.hypot(seg[:, 0], seg[:, 1]).sum())
        if length < EDGE_MIN_LENGTH_CELLS:
            continue
        mx = int(np.mean([p[0] for p in pts]))
        my = float(np.mean([p[1] for p in pts]))
        above = hline is not None and my < hline[mx]
        cls = CLS_STRUCTURE if above else CLS_RUT
        if above and length >= OVERHEAD_MIN_CELLS:
            p0, p1 = np.array(pts[0], np.float32), np.array(pts[-1], np.float32)
            chord = p1 - p0
            norm = float(np.hypot(*chord))
            if norm > 0:
                dev = max((abs(float(chord[0] * (p[1] - p0[1]) - chord[1] * (p[0] - p0[0]))) / norm
                           for p in pts[1:-1]), default=0.0)
                if dev <= OVERHEAD_MAX_SAG_CELLS:
                    cls = CLS_OVERHEAD
        raster = np.zeros((h, w), np.uint8)
        cv2.polylines(raster, [np.array(pts, np.int32).reshape(-1, 1, 2)], False, 1, 1)
        out.append(EdgeCand(tuple(pts), cls, length * float(contrast[i]) / 255.0, raster.astype(bool)))
    return out


def raster_of(record, dx: int = 0, dy: int = 0, shape=(WORK_H, WORK_W)) -> np.ndarray:
    """What the base paints for a define at a cumulative UPD offset (4 px units
    = working px), as a bool mask at working resolution."""
    out = np.zeros(shape, np.uint8)
    if isinstance(record, vs.Poly):
        scale = CELL8 if record.grid == 0 else 1
        pts = np.array([(x * scale + dx, y * scale + dy) for x, y in record.vertices], np.int32)
        cv2.fillPoly(out, [pts.reshape(-1, 1, 2)], 1)
    elif isinstance(record, vs.Tree):
        cv2.ellipse(out, (record.cx * CELL8 + dx, record.cy * CELL8 + dy),
                    (record.rx + 1, record.ry + 1), 0, 0, 360, 1, -1)
    elif isinstance(record, vs.Edge):
        pts = np.array([(x + dx, y + dy) for x, y in record.points], np.int32)
        cv2.polylines(out, [pts.reshape(-1, 1, 2)], False, 1, 1)
    return out.astype(bool)


def resize_working(rgb: np.ndarray, size=(WORK_W, WORK_H)) -> np.ndarray:
    """INTER_AREA resize to the working resolution (a no-op when it already is)."""
    if rgb.shape[1] == size[0] and rgb.shape[0] == size[1]:
        return rgb
    return cv2.resize(rgb, size, interpolation=cv2.INTER_AREA)
