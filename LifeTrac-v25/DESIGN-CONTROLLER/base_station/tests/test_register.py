"""Unit tests for the X8 phase-correlation registration (``register.py``).

Pins the sign convention of ``register_phase_correlation`` to the one
``cv2.phaseCorrelate`` uses — ``(+dx, +dy)`` when the current frame is the
previous frame moved right/down — on both the OpenCV path (skipped when
cv2 is not installed) and the pure-NumPy fallback, plus the Hann window,
the parabolic sub-pixel refinement and the 0..1 confidence. Before this
the fallback returned the negated shift, so ``shift_canvas`` would have
moved the previous frame the wrong way before tile diffing, and its
"confidence" was ~0.01 for a perfect match.
"""

from __future__ import annotations

import os
import sys
import unittest
from unittest import mock


_THIS_DIR = os.path.dirname(os.path.abspath(__file__))
_X8_DIR = os.path.normpath(os.path.join(
    _THIS_DIR, "..", "..", "firmware", "tractor_x8"))
if _X8_DIR not in sys.path:
    sys.path.insert(0, _X8_DIR)

from x8_image_pipeline import register  # noqa: E402

try:                                             # pragma: no cover
    import numpy as np
except ImportError:                              # pragma: no cover
    np = None


W, H = 96, 64                       # small enough to keep the FFTs cheap
CANVAS_W, CANVAS_H = 384, 256       # the X8's 12x8 grid of 32 px tiles

# Measured for a windowed integer roll of the ``_texture`` fixture. The
# window is the one ``cv2.createHanningWindow`` builds (the square root of
# the separable Hann product), so the NumPy path (numpy 2.4/2.5) and the cv2
# path (OpenCV 5.0) agree to three decimals: confidence 0.84..0.91 across
# ten seeds at 96x64 (0.87 and 0.92 for the seeds pinned below) and 0.99 at
# 384x256. The shortfall from 1.0 is the fixed window's mismatch with the
# moved content, of order |dx|/W + |dy|/H (~0.1 here). 0.80 leaves margin
# while staying far above an unrelated pair, which measured <= 0.25 (and
# sometimes < 0, hence the clip).
MIN_SHIFT_CONFIDENCE = 0.80
MAX_UNRELATED_CONFIDENCE = 0.3
SHIFT_TOL_PX = 0.1                  # integer truth; measured error < 0.01 px
                                    # (NumPy parabola), < 0.07 px (cv2's 5x5
                                    # weighted centroid)
# A 3-point parabola through a sinc-shaped peak is biased at quarter-pixel
# offsets: measured 0.10..0.12 px in y for the (2.5, -1.25) case below (x,
# at a half-pixel offset, lands within 0.04 px). An un-refined integer
# answer would be 0.25 px off in y, so 0.2 discriminates.
FRACTIONAL_TOL_PX = 0.2


def _texture(width: int, height: int, seed: int) -> "np.ndarray":
    """Deterministic natural-ish luma plane: white noise, lightly blurred."""
    rng = np.random.default_rng(seed)
    img = rng.integers(0, 256, size=(height, width)).astype(np.float64)
    for _ in range(2):
        img = (img + np.roll(img, 1, 0) + np.roll(img, -1, 0)
               + np.roll(img, 1, 1) + np.roll(img, -1, 1)) / 5.0
    img -= img.min()
    return np.round(img * (255.0 / img.max())).astype(np.uint8)


def _roll(arr: "np.ndarray", dx: int, dy: int) -> "np.ndarray":
    """Circular shift by (+dx right, +dy down): a pure translation."""
    return np.roll(arr, (dy, dx), axis=(0, 1))


def _fourier_shift(arr: "np.ndarray", dx: float, dy: float) -> "np.ndarray":
    """Sub-pixel circular shift via a spectral phase ramp, back to uint8."""
    ky = np.fft.fftfreq(arr.shape[0])[:, None]
    kx = np.fft.fftfreq(arr.shape[1])[None, :]
    spec = np.fft.fft2(arr.astype(np.float64))
    spec *= np.exp(-2j * np.pi * (kx * dx + ky * dy))
    return np.clip(np.round(np.fft.ifft2(spec).real), 0, 255).astype(np.uint8)


def _register(prev: "np.ndarray", curr: "np.ndarray") -> register.Translation:
    h, w = prev.shape
    return register.register_phase_correlation(
        prev.tobytes(), curr.tobytes(), w, h)


@unittest.skipUnless(register._HAVE_NUMPY, "numpy not installed")
class NumpyFallbackTests(unittest.TestCase):
    """The pure-NumPy path, forced even when cv2 happens to be installed."""

    def setUp(self) -> None:
        patcher = mock.patch.object(register, "_HAVE_CV2", False)
        patcher.start()
        self.addCleanup(patcher.stop)

    def test_known_positive_shift_sign_and_magnitude(self) -> None:
        for (w, h) in ((W, H), (CANVAS_W, CANVAS_H)):
            with self.subTest(size=(w, h)):
                prev = _texture(w, h, seed=1)
                t = _register(prev, _roll(prev, 5, 3))
                self.assertAlmostEqual(t.dx, 5.0, delta=SHIFT_TOL_PX)
                self.assertAlmostEqual(t.dy, 3.0, delta=SHIFT_TOL_PX)
                self.assertGreater(t.confidence, MIN_SHIFT_CONFIDENCE)
                self.assertLessEqual(t.confidence, 1.0)

    def test_no_shift_is_zero_with_unit_confidence(self) -> None:
        prev = _texture(W, H, seed=2)
        t = _register(prev, prev)
        self.assertAlmostEqual(t.dx, 0.0, places=3)
        self.assertAlmostEqual(t.dy, 0.0, places=3)
        self.assertAlmostEqual(t.confidence, 1.0, places=3)

    def test_negative_shift_wraps_to_signed(self) -> None:
        prev = _texture(W, H, seed=3)
        t = _register(prev, _roll(prev, -4, -2))
        self.assertAlmostEqual(t.dx, -4.0, delta=SHIFT_TOL_PX)
        self.assertAlmostEqual(t.dy, -2.0, delta=SHIFT_TOL_PX)
        self.assertGreater(t.confidence, MIN_SHIFT_CONFIDENCE)

    def test_subpixel_refinement_tracks_fractional_shift(self) -> None:
        prev = _texture(W, H, seed=4)
        t = _register(prev, _fourier_shift(prev, 2.5, -1.25))
        self.assertAlmostEqual(t.dx, 2.5, delta=FRACTIONAL_TOL_PX)
        self.assertAlmostEqual(t.dy, -1.25, delta=FRACTIONAL_TOL_PX)

    def test_unrelated_frames_have_low_clipped_confidence(self) -> None:
        # Seeds chosen so the raw 5x5 window sum is negative (measured
        # -0.03): the clip must bring it to exactly 0.0.
        t = _register(_texture(W, H, seed=2), _texture(W, H, seed=102))
        self.assertGreaterEqual(t.confidence, 0.0)
        self.assertLess(t.confidence, MAX_UNRELATED_CONFIDENCE)

    def test_agrees_with_shift_canvas_direction(self) -> None:
        # The consumer contract: shift_canvas(prev, dx, dy) lines up with
        # curr, so tile_diff compares like with like.
        prev = _texture(W, H, seed=6)
        curr = _roll(prev, 5, 3)
        t = _register(prev, curr)
        dx, dy = round(t.dx), round(t.dy)
        moved = np.frombuffer(
            register.shift_canvas(prev.tobytes(), W, H, dx, dy),
            dtype=np.uint8).reshape(H, W)
        # Away from the zero-filled edge the two planes are byte-identical.
        self.assertTrue(np.array_equal(moved[dy:, dx:], curr[dy:, dx:]))
        self.assertFalse(np.array_equal(moved[dy:, dx:], prev[dy:, dx:]))


@unittest.skipUnless(register._HAVE_CV2, "cv2 not installed")
class Cv2PathTests(unittest.TestCase):
    """The OpenCV path: same sign, magnitude and confidence as the fallback."""

    def test_known_positive_shift_sign_and_magnitude(self) -> None:
        prev = _texture(W, H, seed=1)
        t = _register(prev, _roll(prev, 5, 3))
        self.assertAlmostEqual(t.dx, 5.0, delta=SHIFT_TOL_PX)
        self.assertAlmostEqual(t.dy, 3.0, delta=SHIFT_TOL_PX)
        self.assertGreater(t.confidence, MIN_SHIFT_CONFIDENCE)
        self.assertLessEqual(t.confidence, 1.0)

    def test_no_shift_is_zero_with_unit_confidence(self) -> None:
        prev = _texture(W, H, seed=2)
        t = _register(prev, prev)
        self.assertAlmostEqual(t.dx, 0.0, places=3)
        self.assertAlmostEqual(t.dy, 0.0, places=3)
        self.assertAlmostEqual(t.confidence, 1.0, places=3)

    def test_matches_numpy_fallback(self) -> None:
        prev = _texture(W, H, seed=7)
        curr = _roll(prev, -4, -2)
        with_cv2 = _register(prev, curr)
        with mock.patch.object(register, "_HAVE_CV2", False):
            fallback = _register(prev, curr)
        self.assertAlmostEqual(with_cv2.dx, fallback.dx, delta=SHIFT_TOL_PX)
        self.assertAlmostEqual(with_cv2.dy, fallback.dy, delta=SHIFT_TOL_PX)
        self.assertAlmostEqual(with_cv2.confidence, fallback.confidence,
                               delta=0.05)


class NoNumpyTests(unittest.TestCase):

    def test_without_numpy_returns_zero_translation(self) -> None:
        blank = bytes(W * H)
        with mock.patch.object(register, "_HAVE_NUMPY", False):
            t = register.register_phase_correlation(blank, blank, W, H)
        self.assertEqual(t, register.Translation(0.0, 0.0, 0.0))


if __name__ == "__main__":
    unittest.main()
