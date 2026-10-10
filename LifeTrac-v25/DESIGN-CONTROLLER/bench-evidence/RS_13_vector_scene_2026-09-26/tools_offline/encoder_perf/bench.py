#!/usr/bin/env python3
"""VS1 encoder benchmark + byte-identity harness (imp/encoder-perf).

Usage:
  python bench.py prep <out_dir>                      # downscale the photos to sources.npz (PC only)
  python bench.py run <impl_dir> <sources.npz> <out.json> [--runs a,b,...] [--frames N] [--profile out.prof]

<impl_dir> holds encode_vector.py, vector_extract.py, vs1_codec.py (imported as bare
modules). Frames are synthesised deterministically from the sources (slow pan/zoom,
still, scene cuts, exposure steps) with seeded +/-12 noise at canvas size, and fed to a
VectorEncoder with a simulated clock, so the payloads are a pure function of the code.
"""
import hashlib
import json
import sys
import time

import numpy as np
import cv2

CANVAS_W, CANVAS_H = 384, 256
SRC_W = 1152                     # sources are downscaled to 1152 wide (3x the canvas)

PHOTOS = {
    "img28": "C:/Windows/Web/Wallpaper/ThemeC/img28.jpg",
    "img29": "C:/Windows/Web/Wallpaper/ThemeC/img29.jpg",
    "img30": "C:/Windows/Web/Wallpaper/ThemeC/img30.jpg",
    "img31": "C:/Windows/Web/Wallpaper/ThemeC/img31.jpg",
    "img102": "C:/Windows/Web/Screen/img102.jpg",
}


def prep(out_dir):
    arrs = {}
    for name, path in PHOTOS.items():
        bgr = cv2.imread(path)
        h, w = bgr.shape[:2]
        sh = int(round(h * SRC_W / w))
        small = cv2.resize(bgr, (SRC_W, sh), interpolation=cv2.INTER_AREA)
        arrs[name] = np.ascontiguousarray(small[:, :, ::-1])      # RGB
        print(name, arrs[name].shape)
    np.savez(out_dir + "/sources.npz", **arrs)


def view(src, cx, cy, width):
    """384x256 view of the source centred at (cx, cy) source px, `width` source px wide."""
    s = width / CANVAS_W
    x0 = cx - width / 2.0
    y0 = cy - (width * CANVAS_H / CANVAS_W) / 2.0
    m = np.array([[s, 0.0, x0], [0.0, s, y0]], np.float64)
    return cv2.warpAffine(src, m, (CANVAS_W, CANVAS_H), flags=cv2.INTER_LINEAR | cv2.WARP_INVERSE_MAP,
                          borderMode=cv2.BORDER_REFLECT)


def lerp(a, b, t):
    return a + (b - a) * t


def segment(src, n, c0, c1, w0, w1):
    """n frames moving the view centre c0 -> c1 and width w0 -> w1."""
    out = []
    for i in range(n):
        t = i / max(n - 1, 1)
        out.append((src, lerp(c0[0], c1[0], t), lerp(c0[1], c1[1], t), lerp(w0, w1, t)))
    return out


def scenario_plan(name, srcs, n):
    if name == "pan":            # slow pan right with a slight zoom in (~1.3 canvas px / frame)
        s = srcs["img28"]
        return segment(s, n, (420, 360), (720, 340), 640, 560), 1.0
    if name == "still":          # static scene, sensor noise only
        s = srcs["img29"]
        return segment(s, n, (576, 360), (576, 360), 700, 700), 1.0
    if name == "cut":            # pan, cut, zoom, cut, still
        a, b, c = n // 3, n // 3, n - 2 * (n // 3)
        return (segment(srcs["img30"], a, (400, 380), (560, 380), 600, 600)
                + segment(srcs["img102"], b, (576, 360), (600, 350), 900, 700)
                + segment(srcs["img31"], c, (600, 360), (600, 360), 640, 640)), 0.5
    if name == "zoom":           # zoom in on img31 with exposure steps (GAIN path)
        s = srcs["img31"]
        return segment(s, n, (560, 380), (590, 360), 900, 520), 0.5
    raise ValueError(name)


def exposure(name, i, n):
    if name == "zoom":
        if i >= 2 * n // 3:
            return 0.85
        if i >= n // 3:
            return 1.12
    return 1.0


def hood_mask():
    m = np.zeros((64, 96), bool)
    for y in range(54, 64):                       # a trapezoid hood at the bottom
        inset = max(0, 63 - y) * 2
        m[y, 20 + inset:76 - inset] = True
    return m


RUNS = {
    # name: (scenario, budget, quality, mask)
    "pan243": ("pan", 243, 80, False),
    "pan203": ("pan", 203, 80, False),
    "still243": ("still", 243, 80, False),
    "still203": ("still", 203, 80, False),
    "cut243": ("cut", 243, 80, False),
    "cut203": ("cut", 203, 80, False),
    "zoom243": ("zoom", 243, 80, False),
    "zoom203": ("zoom", 203, 80, False),
    "panhood203": ("pan", 203, 80, True),
    "cutV1": ("cut", 203, 45, False),
    "cutV2": ("cut", 203, 25, False),
}


def frames_of(run, srcs, n):
    scen = RUNS[run][0]
    plan, dt = scenario_plan(scen, srcs, n)
    rng = np.random.default_rng(sum(map(ord, run)))
    for i, (src, cx, cy, w) in enumerate(plan):
        img = view(src, cx, cy, w).astype(np.int16)
        g = exposure(scen, i, n)
        if g != 1.0:
            img = (img.astype(np.float32) * g).astype(np.int16)
        img += rng.integers(-12, 13, size=img.shape, dtype=np.int16)
        yield np.clip(img, 0, 255).astype(np.uint8), dt


def pct(v, p):
    return float(np.percentile(np.asarray(v, np.float64), p)) if len(v) else 0.0


WT = "C:/Users/dorkm/Documents/GitHub/LifeTrac/.claude/worktrees/wf_adf2ad4c-341-1/LifeTrac-v25/DESIGN-CONTROLLER/firmware/tractor_x8/x8_image_pipeline"


def run(impl_dir, src_path, out_path, runs, n, prof_path=None):
    impl_dir = WT if impl_dir == "wt" else impl_dir
    sys.path.insert(0, impl_dir)
    import encode_vector as ev                     # noqa: E402
    srcs = dict(np.load(src_path))
    result = {"impl": impl_dir, "cv2": cv2.__version__, "numpy": np.__version__, "runs": {}}
    prof = None
    if prof_path:
        import cProfile
        prof = cProfile.Profile()
    for name in runs:
        scen, budget, quality, masked = RUNS[name]
        t_sim = [0.0]
        enc = ev.VectorEncoder(mask=hood_mask() if masked else None, clock=lambda: t_sim[0])
        payloads, ms = [], []
        h = hashlib.sha256()
        for rgb, dt in frames_of(name, srcs, n):
            if prof is not None:
                prof.enable()
            p = enc.frame(rgb, budget, quality=quality, seq=len(payloads))
            if prof is not None:
                prof.disable()
            t_sim[0] += dt
            payloads.append(p.hex())
            h.update(len(p).to_bytes(2, "big") + p)
            ms.append(dict(enc.last_stats["ms"]))
        tot = [m["total"] for m in ms]
        stages = {k: round(pct([m[k] for m in ms], 50), 2) for k in ms[0] if k != "total"}
        result["runs"][name] = {"sha256": h.hexdigest(), "payloads": payloads, "ms": ms,
                                "p50": pct(tot, 50), "p95": pct(tot, 95), "stage_p50": stages,
                                "epochs": enc.last_stats.get("epochs")}
        print(f"{name:11s} sha={h.hexdigest()[:16]} p50={pct(tot, 50):7.2f} p95={pct(tot, 95):7.2f} "
              f"stages={stages} epochs={enc.last_stats.get('epochs')}", flush=True)
    alltot = [m["total"] for r in result["runs"].values() for m in r["ms"]]
    allst = {}
    for r in result["runs"].values():
        for m in r["ms"]:
            for k, v in m.items():
                allst.setdefault(k, []).append(v)
    result["all"] = {"p50": pct(alltot, 50), "p95": pct(alltot, 95),
                     "stage_p50": {k: round(pct(v, 50), 2) for k, v in allst.items()}}
    print("ALL p50=%.2f p95=%.2f stages=%s" % (result["all"]["p50"], result["all"]["p95"],
                                               result["all"]["stage_p50"]), flush=True)
    with open(out_path, "w") as fh:
        json.dump(result, fh)
    if prof is not None:
        prof.dump_stats(prof_path)


if __name__ == "__main__":
    a = sys.argv[1:]
    if a[0] == "prep":
        prep(a[1])
    elif a[0] == "run":
        runs = list(RUNS)
        n = 150
        prof = None
        rest = a[4:]
        i = 0
        while i < len(rest):
            if rest[i] == "--runs":
                runs = rest[i + 1].split(",")
            elif rest[i] == "--frames":
                n = int(rest[i + 1])
            elif rest[i] == "--profile":
                prof = rest[i + 1]
            i += 2
        run(a[1], a[2], a[3], runs, n, prof)
