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
    confidence: float                # 0..1; 1 = pure translation, ~0 = unrelated frames


def _hann_window(width: int, height: int) -> "_np.ndarray":
    """The window ``cv2.createHanningWindow`` builds: the square root of
    the separable Hann product (OpenCV takes the sqrt as its last step), so
    the NumPy and cv2 paths taper the frames identically and report the
    same confidence.

    Without a window the frame border is a second, stationary "image" whose
    spectrum competes with the true peak as soon as the scene really moves
    (successive camera frames are not circular shifts of each other).
    """
    hann = _np.outer(_np.hanning(height), _np.hanning(width))
    return _np.sqrt(hann).astype(_np.float32)


def _parabolic_peak_offset(left: float, centre: float, right: float) -> float:
    """Sub-pixel offset (-0.5..0.5) of the vertex of the parabola through
    three equally spaced samples whose middle one is the integer maximum.
    A flat neighbourhood (or one that is not a maximum) refines to 0."""
    denom = left - 2.0 * centre + right
    if denom >= 0.0:
        return 0.0
    return 0.5 * (left - right) / denom


def register_phase_correlation(prev_y: bytes, curr_y: bytes,
                               width: int, height: int) -> Translation:
    """Estimate the global translation that maps `prev_y` onto `curr_y`.

    Inputs are single-channel luma byte strings (Y plane of YCbCr) of size
    ``width * height``. Returns a :class:`Translation` with the sign
    convention of ``cv2.phaseCorrelate``: ``(+dx, +dy)`` when ``curr_y`` is
    ``prev_y`` moved right by ``dx`` and down by ``dy`` pixels, so that
    ``shift_canvas(prev_y, ..., round(dx), round(dy))`` lines up with
    ``curr_y``. ``confidence`` is the share of the correlation energy in
    the 5x5 neighbourhood of the peak (what OpenCV reports as ``response``)
    clipped to 0..1: ~1.0 for a pure translation, near 0 for unrelated
    frames. Both frames are Hann-windowed first so the frame border does
    not register as a stationary image. When neither NumPy nor OpenCV are
    available the function returns a zero translation with confidence 0.0
    — the caller (tile_diff) should treat that as "no registration data,
    diff the raw frames" rather than crashing.
    """
    if not _HAVE_NUMPY:
        return Translation(0.0, 0.0, 0.0)
    arr_prev = _np.frombuffer(prev_y, dtype=_np.uint8).reshape((height, width)).astype(_np.float32)
    arr_curr = _np.frombuffer(curr_y, dtype=_np.uint8).reshape((height, width)).astype(_np.float32)
    if _HAVE_CV2:                                 # pragma: no cover
        window = _cv2.createHanningWindow((width, height), _cv2.CV_32F)
        (dx, dy), confidence = _cv2.phaseCorrelate(arr_prev, arr_curr, window)
        return Translation(float(dx), float(dy), min(1.0, max(0.0, float(confidence))))
    # Pure-NumPy phase correlation with cv2's conventions: curr * conj(prev)
    # puts the peak at (+dy, +dx) modulo the frame size, and the normalised
    # inverse FFT makes the peak height 1.0 for a pure circular shift.
    window = _hann_window(width, height)
    fa = _np.fft.fft2(arr_prev * window)
    fb = _np.fft.fft2(arr_curr * window)
    cross = fb * _np.conj(fa)
    denom = _np.abs(cross)
    denom[denom == 0] = 1.0
    cps = cross / denom
    corr = _np.fft.ifft2(cps).real
    peak_y, peak_x = _np.unravel_index(_np.argmax(corr), corr.shape)
    # Parabolic sub-pixel refinement through the peak and its (circular)
    # neighbours on each axis.
    sub_x = _parabolic_peak_offset(corr[peak_y, (peak_x - 1) % width],
                                   corr[peak_y, peak_x],
                                   corr[peak_y, (peak_x + 1) % width])
    sub_y = _parabolic_peak_offset(corr[(peak_y - 1) % height, peak_x],
                                   corr[peak_y, peak_x],
                                   corr[(peak_y + 1) % height, peak_x])
    # Same 5x5 window sum cv2's weightedCentroid reports as `response`.
    ys = (peak_y + _np.arange(-2, 3)) % height
    xs = (peak_x + _np.arange(-2, 3)) % width
    confidence = float(corr[_np.ix_(ys, xs)].sum())
    if peak_y > height // 2:
        peak_y -= height
    if peak_x > width // 2:
        peak_x -= width
    return Translation(float(peak_x + sub_x), float(peak_y + sub_y),
                       min(1.0, max(0.0, confidence)))


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
