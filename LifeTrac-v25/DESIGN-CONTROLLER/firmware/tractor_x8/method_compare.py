#!/usr/bin/env python3
"""Compare image-pipeline plan letters A / B / C side-by-side.

Builds a synthetic 384×256 RGB canvas pair where motion is concentrated in
the bottom-right quadrant. Runs ``camera_service._build_frame`` under a
tight 250 B byte budget for each method and prints the kept-tile bitmap.

Method A is expected to ship the top-left strip (row-major starvation);
Method B is expected to ship the motion-hot tiles; Method C is expected to
match B on the first frame and then steadily extend coverage on subsequent
no-motion frames.

Usage (from repo root):

    python LifeTrac-v25/DESIGN-CONTROLLER/firmware/tractor_x8/method_compare.py

Requires Pillow + NumPy (same deps as the real encoder). Falls back with a
clear message if either is missing.
"""
from __future__ import annotations

import importlib.util
import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, _HERE)

try:
    import numpy as np  # type: ignore
except ImportError:                                            # pragma: no cover
    print("method_compare: NumPy is required; install it to run this demo.")
    sys.exit(2)
try:
    import PIL  # noqa: F401
except ImportError:                                            # pragma: no cover
    print("method_compare: Pillow is required; install it to run this demo.")
    sys.exit(2)


def _load_camera_service(image_method: str) -> "any":
    """Re-import camera_service with a fresh ``LIFETRAC_IMAGE_METHOD`` env.

    Module-level constants capture the env at import time, so we have to
    drop the cached module before re-importing.
    """
    os.environ["LIFETRAC_IMAGE_METHOD"] = image_method
    # Tight budget that reproduces the top-left-strip symptom under Method A.
    os.environ.setdefault("LIFETRAC_FRAGMENT_BUDGET", "")  # don't auto-apply
    for name in list(sys.modules):
        if name == "camera_service" or name.startswith("camera_service."):
            del sys.modules[name]
    spec = importlib.util.spec_from_file_location(
        "camera_service", os.path.join(_HERE, "camera_service.py"))
    assert spec is not None and spec.loader is not None
    cs = importlib.util.module_from_spec(spec)
    sys.modules["camera_service"] = cs
    spec.loader.exec_module(cs)
    return cs


class _ReplayCam:
    """Hands out preconfigured RGB canvases one at a time."""

    def __init__(self, frames: "list[bytes]") -> None:
        self._frames = list(frames)
        self._i = 0

    def grab_rgb(self) -> bytes:
        f = self._frames[min(self._i, len(self._frames) - 1)]
        self._i += 1
        return f


def _make_canvases(cs) -> "list[bytes]":
    """Return [frame0, frame1, frame2, frame3]:

    * frame0/frame1 differ in a bottom-right 4×3 tile block (real motion).
    * frame1/frame2 differ only by a thin band of low-amplitude noise
      across the whole canvas (sensor noise).
    * frame2/frame3 are identical (still scene; tests Method C sweep).
    """
    w, h = cs.CANVAS_W, cs.CANVAS_H
    base = np.zeros((h, w, 3), dtype=np.uint8)
    base[:, :, 0] = 96     # mid-grey background
    base[:, :, 1] = 96
    base[:, :, 2] = 96

    # frame0 = base
    f0 = base.copy()

    # frame1: large bright square in the bottom-right 4×3 tile block.
    f1 = base.copy()
    f1[5 * 32:8 * 32, 8 * 32:12 * 32] = [255, 64, 64]

    # frame2: f1 + uniform low-amplitude noise (≤ ±3 levels) — sensor noise.
    rng = np.random.default_rng(1234)
    noise = rng.integers(-3, 4, size=f1.shape, dtype=np.int16)
    f2 = np.clip(f1.astype(np.int16) + noise, 0, 255).astype(np.uint8)

    # frame3: identical to f2 (still scene — exercises Method C sweep).
    f3 = f2.copy()

    return [f0.tobytes(), f1.tobytes(), f2.tobytes(), f3.tobytes()]


def _parse_kept(payload: bytes, cs) -> "list[int]":
    """Return the kept-tile indices from a TileDeltaFrame payload."""
    bitmap_len = (cs.GRID_W * cs.GRID_H + 7) // 8
    bitmap = payload[5:5 + bitmap_len]
    return [i for i in range(cs.GRID_W * cs.GRID_H)
            if bitmap[i // 8] & (1 << (i % 8))]


def _render_grid(kept: "list[int]", cs) -> str:
    grid = []
    for ty in range(cs.GRID_H):
        row = []
        for tx in range(cs.GRID_W):
            i = ty * cs.GRID_W + tx
            row.append("#" if i in kept else ".")
        grid.append(" ".join(row))
    return "\n  ".join(grid)


def _run_one_method(method: str, budget: int) -> None:
    print(f"\n=== Method {method}  (budget = {budget} B) ===")
    cs = _load_camera_service(method)
    frames = _make_canvases(cs)
    accum = cs.FrameAccum()
    cam = _ReplayCam(frames)

    labels = [
        ("F0  initial keyframe (all tiles)",                          False),
        ("F1  motion in bottom-right 4×3 block",                       False),
        ("F2  noise-only (sensor noise across the whole canvas)",       False),
        ("F3  still scene (identical to F2)",                          False),
        ("F4  forced 2nd keyframe — Method C should rotate coverage",  True),
        ("F5  forced 3rd keyframe — coverage should keep advancing",   True),
    ]
    for label, force_key in labels:
        payload = cs._build_frame(cam, accum, force_keyframe=force_key,
                                  byte_budget=budget)
        kept = _parse_kept(payload, cs)
        is_key = payload[0] == 1
        n_changed = sum(1 for byte in payload[5:5 + 12] for bit in range(8)
                        if byte & (1 << bit))
        print(f"\n  {label}")
        print(f"    is_key={is_key}  kept={len(kept):>3}/96  "
              f"changed_bitmap_bits={n_changed}  payload={len(payload):>4} B")
        print(f"    grid (#=kept, .=dropped):")
        print(f"  {_render_grid(kept, cs)}")


def main() -> int:
    budget = int(os.environ.get("LIFETRAC_FRAGMENT_BUDGET_DEMO", "250"))
    print(f"method_compare — synthetic canvas, budget = {budget} B")
    print("Expected:")
    print("  A: F1 wastes the budget on top-left noise; motion tiles starve.")
    print("  B: F1 ships motion tiles (bottom-right 4×3 block) instead.")
    print("  C: same as B on F1; F2/F3 extend coverage via the sweep.")
    for m in ("A", "B", "C"):
        _run_one_method(m, budget)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
