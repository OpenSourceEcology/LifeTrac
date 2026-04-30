"""Round 49 / BC-26 (K-A3): SIL coverage for the ramp interpolation shape.

The build-config leaf ``hydraulic.ramp_shape`` selects the interpolation
curve used during release ramps and BC-22 reversal-decay ramps.

* ``"linear"`` (default) preserves the pre-Round-49 truncation-toward-zero
  math byte-for-byte: ``v = start * (duration_ms - elapsed) / duration_ms``.
* ``"scurve"`` substitutes a half-cosine smoothstep:
  ``shape(t) = 0.5 * (1 + cos(\u03c0 t))`` with ``v = start * shape``. The
  derivative is zero at both ``t = 0`` and ``t = 1``, so the velocity
  curve is gentle at engagement and at the stop \u2014 cutting P95 jerk
  roughly in half for the same total stop distance.

Test classes:

* ``RS_A_LinearIdentityTests`` \u2014 the default ``"linear"`` shape produces
  the exact same per-tick values as the pre-Round-49 truncation formula
  at a representative spread of ``elapsed`` / ``duration_ms`` /
  ``start`` combinations.
* ``RS_B_ScurveEndpointsTests`` \u2014 at ``elapsed = 0`` the scurve sample
  equals ``start``; at ``elapsed >= duration_ms`` it is ``0``; the value
  is monotonic non-increasing in ``elapsed`` for ``start > 0``.
* ``RS_C_ScurveQuarterPointsTests`` \u2014 the smoothstep stays HIGHER than
  linear before the midpoint and LOWER after; pinned by closed-form
  values at ``t = 0.25 / 0.5 / 0.75``.
* ``RS_D_ScurveSignPreservationTests`` \u2014 negative ``start`` values
  produce negative outputs along the ramp; the magnitude profile
  matches the positive case.
* ``RS_E_ArbiterEndToEndTests`` \u2014 a ``FourAxisArbiter`` constructed
  with ``ramp_shape="scurve"`` differs from the linear default at
  quarter-points of a representative release ramp, while still hitting
  zero at the deadline tick.
* ``RS_F_DefaultIdentityTests`` \u2014 ``FourAxisArbiter()`` with no
  ``ramp_shape`` argument produces byte-for-byte the same release-ramp
  trajectory as the explicit ``"linear"`` arbiter.
* ``RS_G_FirmwareTripwireTests`` \u2014 firmware contains the ``BC-26`` /
  ``K-A3`` markers, the ``ramp_interpolate`` helper symbol, the
  ``LIFETRAC_HYDRAULIC_RAMP_SHAPE_SCURVE`` consumption, and the call
  site replacing the inline interpolation. Build config exposes the
  schema enum and the CAPABILITY_INVENTORY row.

Pure stdlib.
"""

from __future__ import annotations

import math
import pathlib
import unittest

from test_axis_ramp_sil import (
    AXIS_DEADBAND,
    RAMP_TICK_MS,
    AxisRamp,
    FourAxisArbiter,
    _ramp_interpolate,
    ramp_duration_ms,
    step_axis_ramp,
)


_FIRMWARE_PATH = (pathlib.Path(__file__).resolve().parents[3]
                  / "DESIGN-CONTROLLER" / "firmware"
                  / "tractor_h7" / "tractor_h7.ino")
_INVENTORY_PATH = (pathlib.Path(__file__).resolve().parents[3]
                   / "DESIGN-CONTROLLER" / "CAPABILITY_INVENTORY.md")
_SCHEMA_PATH = (pathlib.Path(__file__).resolve().parents[1]
                / "config" / "build_config.schema.json")


def _linear_truncated(start: int, elapsed: int, duration_ms: int) -> int:
    """Pre-Round-49 canonical formula, kept here as the oracle."""
    if duration_ms <= 0 or elapsed >= duration_ms:
        return 0
    return int(start * (duration_ms - elapsed) / duration_ms)


# ---------------------------------------------------------------------------
# RS-A \u2014 linear default is byte-for-byte identical to the legacy formula
# ---------------------------------------------------------------------------


class RS_A_LinearIdentityTests(unittest.TestCase):

    def test_linear_matches_legacy_truncation(self):
        for start in [10, 32, 64, 96, 127, -32, -96, -127]:
            for duration_ms in [250, 500, 1000, 2000]:
                for elapsed in [0, 1, 50, duration_ms // 4,
                                duration_ms // 2,
                                duration_ms * 3 // 4,
                                duration_ms - 1]:
                    with self.subTest(start=start, dur=duration_ms,
                                      elapsed=elapsed):
                        legacy = _linear_truncated(start, elapsed,
                                                   duration_ms)
                        actual = _ramp_interpolate(start, elapsed,
                                                   duration_ms,
                                                   shape="linear")
                        self.assertEqual(actual, legacy)

    def test_default_shape_arg_is_linear(self):
        for start in [50, -100]:
            v_default = _ramp_interpolate(start, 250, 1000)
            v_explicit = _ramp_interpolate(start, 250, 1000,
                                           shape="linear")
            self.assertEqual(v_default, v_explicit)


# ---------------------------------------------------------------------------
# RS-B \u2014 scurve endpoints + monotonicity
# ---------------------------------------------------------------------------


class RS_B_ScurveEndpointsTests(unittest.TestCase):

    def test_scurve_at_t0_equals_start(self):
        for start in [10, 64, 127, -50, -127]:
            with self.subTest(start=start):
                self.assertEqual(_ramp_interpolate(start, 0, 1000,
                                                   shape="scurve"),
                                 start)

    def test_scurve_at_deadline_is_zero(self):
        for start in [10, 64, 127, -50, -127]:
            for elapsed in [1000, 1001, 5000]:
                with self.subTest(start=start, elapsed=elapsed):
                    self.assertEqual(_ramp_interpolate(start, elapsed,
                                                       1000,
                                                       shape="scurve"),
                                     0)

    def test_scurve_zero_duration_returns_zero(self):
        self.assertEqual(_ramp_interpolate(127, 0, 0, shape="scurve"), 0)
        self.assertEqual(_ramp_interpolate(127, 0, 0, shape="linear"), 0)

    def test_scurve_monotonic_non_increasing_for_positive_start(self):
        prev = 128
        for elapsed in range(0, 1001, 25):
            v = _ramp_interpolate(127, elapsed, 1000, shape="scurve")
            self.assertLessEqual(v, prev,
                                 f"non-monotonic at elapsed={elapsed}: "
                                 f"prev={prev}, v={v}")
            prev = v


# ---------------------------------------------------------------------------
# RS-C \u2014 scurve quarter-points distinguish from linear
# ---------------------------------------------------------------------------


class RS_C_ScurveQuarterPointsTests(unittest.TestCase):

    def test_quarter_point_higher_than_linear(self):
        # At t=0.25, linear = 0.75 * start; smoothstep = 0.5*(1+cos(\u03c0/4))
        # = 0.5*(1+0.7071) \u2248 0.8536 \u2192 strictly larger.
        v_lin = _ramp_interpolate(127, 250, 1000, shape="linear")
        v_sc = _ramp_interpolate(127, 250, 1000, shape="scurve")
        # Closed-form expectation
        expected = round(127 * 0.5 * (1 + math.cos(math.pi * 0.25)))
        self.assertEqual(v_sc, expected)
        self.assertGreater(v_sc, v_lin)

    def test_midpoint_equals_half_for_both(self):
        # Both shapes pass through start/2 at t=0.5.
        for shape in ("linear", "scurve"):
            v = _ramp_interpolate(127, 500, 1000, shape=shape)
            with self.subTest(shape=shape):
                self.assertGreaterEqual(v, 62)
                self.assertLessEqual(v, 64)

    def test_three_quarter_point_lower_than_linear(self):
        # At t=0.75, linear = 0.25*start; smoothstep = 0.5*(1+cos(3\u03c0/4))
        # = 0.5*(1-0.7071) \u2248 0.1464 \u2192 strictly smaller.
        v_lin = _ramp_interpolate(127, 750, 1000, shape="linear")
        v_sc = _ramp_interpolate(127, 750, 1000, shape="scurve")
        expected = round(127 * 0.5 * (1 + math.cos(math.pi * 0.75)))
        self.assertEqual(v_sc, expected)
        self.assertLess(v_sc, v_lin)


# ---------------------------------------------------------------------------
# RS-D \u2014 sign preservation
# ---------------------------------------------------------------------------


class RS_D_ScurveSignPreservationTests(unittest.TestCase):

    def test_negative_start_stays_negative(self):
        for elapsed in [0, 100, 250, 500, 750, 999]:
            with self.subTest(elapsed=elapsed):
                v = _ramp_interpolate(-127, elapsed, 1000, shape="scurve")
                self.assertLessEqual(v, 0)

    def test_magnitude_profile_matches_positive(self):
        for elapsed in [0, 100, 250, 500, 750, 999]:
            pos = _ramp_interpolate(127, elapsed, 1000, shape="scurve")
            neg = _ramp_interpolate(-127, elapsed, 1000, shape="scurve")
            with self.subTest(elapsed=elapsed):
                # Allow a 1-LSB asymmetry from sign-aware half-up rounding.
                self.assertLessEqual(abs(pos + neg), 1)


# ---------------------------------------------------------------------------
# RS-E \u2014 arbiter end-to-end: scurve differs from linear, both stop on time
# ---------------------------------------------------------------------------


class RS_E_ArbiterEndToEndTests(unittest.TestCase):

    def _drive_then_release(self, ramp_shape: str) -> list[int]:
        """Drive forward at lhy=127 for one tick, release, capture every
        tick of the release ramp until the axis reaches 0."""
        arb = FourAxisArbiter(ramp_shape=ramp_shape)
        t = 0
        # Drive (engages instantaneously)
        arb.tick(0, 127, 0, 0, t)
        t += RAMP_TICK_MS
        # Release \u2014 K-A4 forces both tracks to share the duration of the
        # larger starting magnitude (127 \u2192 ramp_duration_ms = 2000 ms).
        samples = []
        for _ in range(0, 2200, RAMP_TICK_MS):
            out = arb.tick(0, 0, 0, 0, t)
            samples.append(out["left_track"])
            t += RAMP_TICK_MS
        return samples

    def test_scurve_differs_from_linear_midramp(self):
        lin_samples = self._drive_then_release("linear")
        sc_samples = self._drive_then_release("scurve")
        # Both must reach 0 by the end (post-deadline).
        self.assertEqual(lin_samples[-1], 0)
        self.assertEqual(sc_samples[-1], 0)
        # And both must START from 127 on the first release tick.
        self.assertEqual(lin_samples[0], 127)
        self.assertEqual(sc_samples[0], 127)
        # At a quarter into the 2 s ramp (~500 ms = 5 ticks at 100 ms),
        # scurve should be strictly higher than linear.
        idx = 5
        self.assertGreater(sc_samples[idx], lin_samples[idx],
                           f"scurve[{idx}]={sc_samples[idx]} should exceed "
                           f"linear[{idx}]={lin_samples[idx]}")

    def test_both_shapes_reach_zero_by_deadline(self):
        for shape in ("linear", "scurve"):
            samples = self._drive_then_release(shape)
            with self.subTest(shape=shape):
                # The 2 s ramp \u00f7 100 ms tick = 20 ticks; sample 20 (= idx
                # 19 zero-based) is past the deadline and must be 0.
                self.assertEqual(samples[-1], 0)


# ---------------------------------------------------------------------------
# RS-F \u2014 default identity at the arbiter level
# ---------------------------------------------------------------------------


class RS_F_DefaultIdentityTests(unittest.TestCase):

    def test_default_arbiter_uses_linear(self):
        arb = FourAxisArbiter()
        self.assertEqual(arb.ramp_shape, "linear")

    def test_default_arbiter_release_matches_explicit_linear(self):
        # Drive a release ramp through both arbiters and confirm every
        # post-release tick is byte-for-byte identical.
        a_default = FourAxisArbiter()
        a_explicit = FourAxisArbiter(ramp_shape="linear")
        t = 0
        a_default.tick(0, 100, 0, 0, t)
        a_explicit.tick(0, 100, 0, 0, t)
        t += RAMP_TICK_MS
        for _ in range(20):
            out_d = a_default.tick(0, 0, 0, 0, t)
            out_e = a_explicit.tick(0, 0, 0, 0, t)
            self.assertEqual(out_d["left_track"], out_e["left_track"])
            self.assertEqual(out_d["right_track"], out_e["right_track"])
            t += RAMP_TICK_MS


# ---------------------------------------------------------------------------
# RS-G \u2014 source / config tripwires
# ---------------------------------------------------------------------------


class RS_G_FirmwareTripwireTests(unittest.TestCase):

    @classmethod
    def setUpClass(cls) -> None:
        cls.fw = _FIRMWARE_PATH.read_text(encoding="utf-8")
        cls.inv = _INVENTORY_PATH.read_text(encoding="utf-8")
        cls.schema = _SCHEMA_PATH.read_text(encoding="utf-8")

    def test_bc26_marker_present_in_firmware(self):
        self.assertIn("BC-26", self.fw)

    def test_ramp_interpolate_helper_present(self):
        self.assertIn("ramp_interpolate", self.fw)

    def test_scurve_macro_consumed(self):
        self.assertIn("LIFETRAC_HYDRAULIC_RAMP_SHAPE_SCURVE", self.fw)

    def test_inline_linear_interpolation_removed(self):
        # The pre-Round-49 inline ``int32_t v = start * (duration - elapsed) /
        # duration`` block in step_axis_ramp must be gone in favour of the
        # helper. We pin this by checking the helper is the call site.
        self.assertIn("r.effective = ramp_interpolate(", self.fw)

    def test_schema_has_ramp_shape_enum(self):
        self.assertIn("ramp_shape", self.schema)
        self.assertIn("\"linear\"", self.schema)
        self.assertIn("\"scurve\"", self.schema)

    def test_inventory_documents_new_leaf(self):
        self.assertIn("hydraulic.ramp_shape", self.inv)


if __name__ == "__main__":
    unittest.main()
