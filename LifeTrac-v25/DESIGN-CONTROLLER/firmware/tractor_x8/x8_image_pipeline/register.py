"""Phase-correlation pre-diff image registration.

Per IMAGE_PIPELINE.md week 2: without registering successive frames against
each other before computing the changed-tile bitmap, byte savings collapse
the moment the tractor moves (every tile is "different" because the whole
frame shifted by 4 pixels). We estimate a global (dx, dy) translation by
phase correlation in the frequency domain and shift the previous frame by
that vector before per-tile diffing in tile_diff.py.

NumPy + scipy.fft is the reference implementation; on a real Portenta X8
the same operation is NEON-accelerated through OpenCV (`cv2.phaseCorrelate`)
at ~5 % CPU. We don't import OpenCV at the top of the module because the
test environment may not have it; ``register_phase_correlation()`` will use
OpenCV iff available and fall back to a pure-NumPy implementation otherwise.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

try:                                             # pragma: no cover
    import numpy as _np                          # type: ignore
    _HAVE_NUMPY = True
except ImportError:                              # pragma: no cover
    _np = None                                    # type: ignore
    _HAVE_NUMPY = False

try:                                             # pragma: no cover
    import cv2 as _cv2                           # type: ignore
    _HAVE_CV2 = True
except ImportError:                              # pragma: no cover
    _cv2 = None                                   # type: ignore
    _HAVE_CV2 = False


@dataclass(frozen=True)
class Translation:
    dx: float
    dy: float
    confidence: float                # 0..1; 1 = peak picks itself out of noise


def register_phase_correlation(prev_y: bytes, curr_y: bytes,
                               width: int, height: int) -> Translation:
    """Estimate the global translation that maps `prev_y` onto `curr_y`.

    Inputs are single-channel luma byte strings (Y plane of YCbCr) of size
    ``width * height``. Returns a :class:`Translation`. When neither NumPy
    nor OpenCV are available the function returns a zero translation with
    confidence 0.0 — the caller (tile_diff) should treat that as "no
    registration data, diff the raw frames" rather than crashing.
    """
    if not _HAVE_NUMPY:
        return Translation(0.0, 0.0, 0.0)
    arr_prev = _np.frombuffer(prev_y, dtype=_np.uint8).reshape((height, width)).astype(_np.float32)
    arr_curr = _np.frombuffer(curr_y, dtype=_np.uint8).reshape((height, width)).astype(_np.float32)
    if _HAVE_CV2:                                 # pragma: no cover
        (dx, dy), confidence = _cv2.phaseCorrelate(arr_prev, arr_curr)
        return Translation(float(dx), float(dy), float(confidence))
    # Pure-NumPy phase correlation.
    fa = _np.fft.fft2(arr_prev)
    fb = _np.fft.fft2(arr_curr)
    cross = fa * _np.conj(fb)
    denom = _np.abs(cross)
    denom[denom == 0] = 1.0
    cps = cross / denom
    corr = _np.fft.ifft2(cps).real
    peak_y, peak_x = _np.unravel_index(_np.argmax(corr), corr.shape)
    if peak_y > height // 2:
        peak_y -= height
    if peak_x > width // 2:
        peak_x -= width
    confidence = float(corr.max() / (corr.std() * corr.size + 1e-9))
    return Translation(float(peak_x), float(peak_y), min(1.0, confidence))


def shift_canvas(plane: bytes, width: int, height: int, dx: int, dy: int) -> bytes:
    """Shift `plane` (single-channel) by integer (dx, dy), zero-filling the
    edges. Used by tile_diff before computing per-tile change bits."""
    if not _HAVE_NUMPY:
        return plane
    arr = _np.frombuffer(plane, dtype=_np.uint8).reshape((height, width))
    out = _np.zeros_like(arr)
    src_x_lo = max(0, -dx); src_x_hi = min(width, width - dx)
    src_y_lo = max(0, -dy); src_y_hi = min(height, height - dy)
    dst_x_lo = max(0, dx);  dst_x_hi = dst_x_lo + (src_x_hi - src_x_lo)
    dst_y_lo = max(0, dy);  dst_y_hi = dst_y_lo + (src_y_hi - src_y_lo)
    if src_x_hi > src_x_lo and src_y_hi > src_y_lo:
        out[dst_y_lo:dst_y_hi, dst_x_lo:dst_x_hi] = \
            arr[src_y_lo:src_y_hi, src_x_lo:src_x_hi]
    return out.tobytes()
