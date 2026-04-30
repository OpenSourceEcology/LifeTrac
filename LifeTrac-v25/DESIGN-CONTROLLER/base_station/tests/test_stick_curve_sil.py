"""Round 48 / BC-25 (K-A2): SIL coverage for the per-stick response curve.

The build-config leaf ``ui.stick_curve_exponent`` selects the exponent
used in ``effective = sign(v) * 127 * (|v|/127) ** n`` applied to every
raw stick axis BEFORE deadband filtering and BEFORE differential mixing.
The canonical default is ``n = 1.0`` (linear / identity) so existing
operator-feel is unchanged unless a build opts in. Higher exponents
(1.5 / 2.0) compress the low-stick band toward zero, giving finer
creep-speed precision at the cost of top-end resolution.

Test classes:

* ``SC_A_CurveMathTests`` \u2014 the pure ``_apply_stick_curve`` formula:
  identity at n=1.0, square law at n=2.0, monotonic for n=1.5,
  preserves sign, fixed points at 0 and \u00b1127, clamping behaviour.
* ``SC_B_DefaultIdentityTests`` \u2014 ``FourAxisArbiter`` with the default
  exponent of 1.0 produces identical outputs to passing through
  ``_apply_stick_curve(v, 1.0)`` directly (no surprises for legacy
  fixtures that don't override the exponent).
* ``SC_C_LowStickPrecisionTests`` \u2014 at n=2.0 a 50%-stick input produces
  a meaningfully smaller post-curve magnitude than at n=1.0; pinned by
  numeric example.
* ``SC_D_FullStickAuthorityTests`` \u2014 pegged sticks (\u00b1127) reach the
  same value at every supported exponent (top end preserved).
* ``SC_E_CurveAppliesPreMixingTests`` \u2014 with n=2.0, the differential
  ``(left_track - right_track) / 2`` reflects the CURVED lhx, not the
  raw lhx.
* ``SC_F_FirmwareTripwireTests`` \u2014 firmware source carries the BC-25 +
  K-A2 markers, the ``apply_stick_curve`` symbol, the LUT initialiser,
  and the four pre-mix call sites in ``apply_control()``. Build config
  exposes the new schema leaf and CAPABILITY_INVENTORY row.

Pure stdlib.
"""

from __future__ import annotations

import pathlib
import unittest

from test_axis_ramp_sil import (
    RAMP_TICK_MS,
    FourAxisArbiter,
    _apply_stick_curve,
)


_FIRMWARE_PATH = (pathlib.Path(__file__).resolve().parents[3]
                  / "DESIGN-CONTROLLER" / "firmware"
                  / "tractor_h7" / "tractor_h7.ino")
_INVENTORY_PATH = (pathlib.Path(__file__).resolve().parents[3]
                   / "DESIGN-CONTROLLER" / "CAPABILITY_INVENTORY.md")
_SCHEMA_PATH = (pathlib.Path(__file__).resolve().parents[1]
                / "config" / "build_config.schema.json")


# ---------------------------------------------------------------------------
# SC-A \u2014 pure curve math
# ---------------------------------------------------------------------------


class SC_A_CurveMathTests(unittest.TestCase):

    def test_identity_at_exponent_1(self):
        for v in [-127, -64, -13, -1, 0, 1, 13, 64, 127]:
            with self.subTest(v=v):
                self.assertEqual(_apply_stick_curve(v, 1.0), v)

    def test_zero_is_fixed_point(self):
        for n in [1.0, 1.5, 2.0]:
            with self.subTest(n=n):
                self.assertEqual(_apply_stick_curve(0, n), 0)

    def test_full_stick_preserved(self):
        for n in [1.0, 1.5, 2.0]:
            with self.subTest(n=n):
                self.assertEqual(_apply_stick_curve(127, n), 127)
                self.assertEqual(_apply_stick_curve(-127, n), -127)

    def test_square_law_at_exponent_2(self):
        # 64/127 \u2248 0.504; 0.504^2 \u2248 0.254; * 127 \u2248 32.3 \u2192 32 rounded.
        self.assertEqual(_apply_stick_curve(64, 2.0), 32)
        self.assertEqual(_apply_stick_curve(-64, 2.0), -32)

    def test_one_point_five_intermediate(self):
        # 64/127 \u2248 0.504; 0.504^1.5 \u2248 0.358; * 127 \u2248 45.5 \u2192 45 or 46.
        result = _apply_stick_curve(64, 1.5)
        self.assertGreaterEqual(result, 44)
        self.assertLessEqual(result, 46)
        # And strictly between linear (64) and square (32).
        self.assertGreater(result, _apply_stick_curve(64, 2.0))
        self.assertLess(result, _apply_stick_curve(64, 1.0))

    def test_monotonic_in_input(self):
        for n in [1.0, 1.5, 2.0]:
            prev = -1
            for v in range(0, 128):
                cur = _apply_stick_curve(v, n)
                self.assertGreaterEqual(cur, prev,
                                        f"non-monotonic at v={v} n={n}")
                prev = cur

    def test_sign_preserved(self):
        # Use values large enough that the curved magnitude doesn't round
        # down to zero at n=2.0 (where 1\u00b2/127 \u2248 0.008 \u2192 0).
        for n in [1.5, 2.0]:
            for v in [13, 50, 100, 127]:
                with self.subTest(n=n, v=v):
                    pos = _apply_stick_curve(v, n)
                    neg = _apply_stick_curve(-v, n)
                    self.assertGreater(pos, 0)
                    self.assertLess(neg, 0)
                    self.assertEqual(pos, -neg)

    def test_clamps_out_of_range(self):
        # Defensive: out-of-range inputs are clamped rather than indexing
        # past the LUT.
        self.assertEqual(_apply_stick_curve(200, 2.0), 127)
        self.assertEqual(_apply_stick_curve(-200, 2.0), -127)


# ---------------------------------------------------------------------------
# SC-B \u2014 default exponent of 1.0 is byte-identical
# ---------------------------------------------------------------------------


class SC_B_DefaultIdentityTests(unittest.TestCase):

    def test_default_arbiter_uses_linear(self):
        arb = FourAxisArbiter()
        self.assertEqual(arb.stick_curve_exponent, 1.0)

    def test_default_arbiter_outputs_match_pre_bc25_pipeline(self):
        # With n=1.0 the curve is identity, so every existing test that
        # didn't pass an exponent still works. Spot-check a few inputs.
        arb = FourAxisArbiter()
        t = 0
        for _ in range(5):
            out = arb.tick(40, 80, 30, 60, t)
            t += RAMP_TICK_MS
        # Mixed: left = 80+40 = 120, right = 80-40 = 40, arms = 60, bucket = 30.
        self.assertEqual(out["left_track"], 120)
        self.assertEqual(out["right_track"], 40)
        self.assertEqual(out["arms"], 60)
        self.assertEqual(out["bucket"], 30)


# ---------------------------------------------------------------------------
# SC-C \u2014 low-stick precision (the K-A2 motivating use case)
# ---------------------------------------------------------------------------


class SC_C_LowStickPrecisionTests(unittest.TestCase):

    def test_50pct_stick_compressed_at_n2(self):
        # At 50% stick the linear curve gives ~64; the n=2 curve gives ~32
        # \u2014 a ~50% reduction in commanded magnitude. This is the entire
        # point of the feature: better creep precision.
        v = 64
        linear = _apply_stick_curve(v, 1.0)
        squared = _apply_stick_curve(v, 2.0)
        self.assertEqual(linear, 64)
        self.assertEqual(squared, 32)
        self.assertLess(squared, linear // 2 + 5,
                        "n=2 must compress 50% stick by ~half")

    def test_arbiter_low_stick_compression(self):
        # Drive forward at lhy=64 with n=2 \u2192 both tracks should be 32, not 64.
        arb = FourAxisArbiter(stick_curve_exponent=2.0)
        t = 0
        for _ in range(5):
            out = arb.tick(0, 64, 0, 0, t)
            t += RAMP_TICK_MS
        self.assertEqual(out["left_track"], 32)
        self.assertEqual(out["right_track"], 32)


# ---------------------------------------------------------------------------
# SC-D \u2014 full stick authority preserved
# ---------------------------------------------------------------------------


class SC_D_FullStickAuthorityTests(unittest.TestCase):

    def test_full_forward_reaches_127_under_every_curve(self):
        for n in [1.0, 1.5, 2.0]:
            with self.subTest(n=n):
                arb = FourAxisArbiter(stick_curve_exponent=n)
                t = 0
                for _ in range(5):
                    out = arb.tick(0, 127, 0, 0, t)
                    t += RAMP_TICK_MS
                self.assertEqual(out["left_track"], 127)
                self.assertEqual(out["right_track"], 127)


# ---------------------------------------------------------------------------
# SC-E \u2014 curve applies pre-mixing
# ---------------------------------------------------------------------------


class SC_E_CurveAppliesPreMixingTests(unittest.TestCase):

    def test_steering_uses_curved_lhx(self):
        # n=2: lhx=64 \u2192 curved 32; lhy=0 \u2192 curved 0. Tracks: (32, -32).
        # If the curve were applied AFTER mixing, the result would be
        # different: raw mix = (64, -64), curved mix = (16, -16). The
        # pre-mixing variant gives \u00b132; post-mixing would give \u00b116.
        arb = FourAxisArbiter(stick_curve_exponent=2.0)
        t = 0
        for _ in range(5):
            out = arb.tick(64, 0, 0, 0, t)
            t += RAMP_TICK_MS
        self.assertEqual(out["left_track"], 32,
                         "curve must apply pre-mixing (left should be 32, "
                         "not 16)")
        self.assertEqual(out["right_track"], -32)


# ---------------------------------------------------------------------------
# SC-F \u2014 source / config tripwires
# ---------------------------------------------------------------------------


class SC_F_FirmwareTripwireTests(unittest.TestCase):

    @classmethod
    def setUpClass(cls) -> None:
        cls.fw = _FIRMWARE_PATH.read_text(encoding="utf-8")
        cls.inv = _INVENTORY_PATH.read_text(encoding="utf-8")
        cls.schema = _SCHEMA_PATH.read_text(encoding="utf-8")

    def test_bc25_marker_present_in_firmware(self):
        self.assertIn("BC-25", self.fw)

    def test_apply_stick_curve_helper_present(self):
        self.assertIn("apply_stick_curve", self.fw)

    def test_init_lut_called_from_setup(self):
        self.assertIn("init_stick_curve_lut", self.fw)

    def test_curve_applied_to_all_four_stick_axes(self):
        # apply_control() must call the helper on every raw stick axis.
        for axis in ("axis_lh_x", "axis_lh_y", "axis_rh_x", "axis_rh_y"):
            with self.subTest(axis=axis):
                self.assertIn(f"apply_stick_curve(cf.{axis})", self.fw,
                              f"BC-25 must apply curve to cf.{axis}")

    def test_schema_has_new_leaf(self):
        self.assertIn("stick_curve_exponent", self.schema)
        self.assertIn("\"enum\": [1.0, 1.5, 2.0]", self.schema)

    def test_inventory_documents_new_leaf(self):
        self.assertIn("ui.stick_curve_exponent", self.inv)


if __name__ == "__main__":
    unittest.main()
