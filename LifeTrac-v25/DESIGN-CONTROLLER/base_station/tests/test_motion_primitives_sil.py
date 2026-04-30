"""Round 47: pin MP-01..MP-12 motion primitives to FourAxisArbiter
behavior.

The 12 primitives in
[MOTION_PRIMITIVES.md](../../../DESIGN-KINEMATICS/MOTION_PRIMITIVES.md)
are the canonical operator-intelligible motion vocabulary. This SIL
fixture exercises each one through the same arbiter the firmware
mirrors, asserting the post-mix logical-axis pattern matches the
documented signature and that the appropriate landed BC fixes are
exercised.

Test classes track the doc table 1:1:

* MP01_DriveForwardTests
* MP02_DriveReverseTests
* MP03_SkidTurnTests             (uses BC-21 mix-then-ramp + BC-23 saturation)
* MP04_PivotTurnTests            (one track at zero)
* MP05_SpinInPlaceTests          (uses BC-24 flow boost)
* MP06_ArmsRaiseTests
* MP07_ArmsLowerTests
* MP08_BucketCurlTests
* MP09_BucketDumpTests
* MP10_DriveAndArmsTests         (flow contention single-EFC)
* MP11_TurnAndBucketTests
* MP12_FloatTests                (build-gated; documented behaviour only)

Each test is short and behavioural \u2014 the goal is to provide a single
file that fails loudly if any of MP-01..MP-11's user-visible signature
silently regresses. MP-12 is documented as build-gated and currently has
no SIL surface (skipped with reason).
"""

from __future__ import annotations

import unittest

from test_axis_ramp_sil import (
    AXIS_DEADBAND,
    RAMP_TICK_MS,
    FourAxisArbiter,
)


def _settle(arb: FourAxisArbiter, lhx: int, lhy: int,
            rhx: int = 0, rhy: int = 0, ticks: int = 5) -> dict:
    t = 0
    out: dict = {}
    for _ in range(ticks):
        out = arb.tick(lhx, lhy, rhx, rhy, t)
        t += RAMP_TICK_MS
    return out


# ---------------------------------------------------------------------------
# MP-01 / MP-02 \u2014 drive forward / reverse
# ---------------------------------------------------------------------------


class MP01_DriveForwardTests(unittest.TestCase):
    def test_full_forward_both_tracks_positive_equal(self):
        out = _settle(FourAxisArbiter(), lhx=0, lhy=120)
        self.assertEqual(out["left_track"], 120)
        self.assertEqual(out["right_track"], 120)
        self.assertEqual(out["arms"], 0)
        self.assertEqual(out["bucket"], 0)


class MP02_DriveReverseTests(unittest.TestCase):
    def test_full_reverse_both_tracks_negative_equal(self):
        out = _settle(FourAxisArbiter(), lhx=0, lhy=-120)
        self.assertEqual(out["left_track"], -120)
        self.assertEqual(out["right_track"], -120)


# ---------------------------------------------------------------------------
# MP-03 \u2014 skid turn (both tracks forward, asymmetric)
# ---------------------------------------------------------------------------


class MP03_SkidTurnTests(unittest.TestCase):
    def test_gentle_right_turn_both_tracks_positive_left_faster(self):
        # lhy=80, lhx=30 \u2192 intents (110, 50). No saturation.
        out = _settle(FourAxisArbiter(), lhx=30, lhy=80)
        self.assertEqual(out["left_track"], 110)
        self.assertEqual(out["right_track"], 50)
        # Both same sign \u2192 not a spin-turn.
        self.assertGreater(out["left_track"], 0)
        self.assertGreater(out["right_track"], 0)


# ---------------------------------------------------------------------------
# MP-04 \u2014 pivot turn (one track stopped)
# ---------------------------------------------------------------------------


class MP04_PivotTurnTests(unittest.TestCase):
    def test_pivot_right_left_track_active_right_zero(self):
        # lhy = 60, lhx = 60 \u2192 intents (120, 0). Right track exactly 0.
        out = _settle(FourAxisArbiter(), lhx=60, lhy=60)
        self.assertEqual(out["left_track"], 120)
        self.assertEqual(out["right_track"], 0)

    def test_pivot_left_right_track_active_left_zero(self):
        out = _settle(FourAxisArbiter(), lhx=-60, lhy=60)
        self.assertEqual(out["left_track"], 0)
        self.assertEqual(out["right_track"], 120)


# ---------------------------------------------------------------------------
# MP-05 \u2014 spin in place (counter-rotation; exercises BC-24)
# ---------------------------------------------------------------------------


class MP05_SpinInPlaceTests(unittest.TestCase):
    def test_pure_spin_tracks_have_opposite_signs(self):
        out = _settle(FourAxisArbiter(), lhx=120, lhy=0)
        self.assertEqual(out["left_track"], 120)
        self.assertEqual(out["right_track"], -120)

    def test_pure_spin_triggers_bc24_full_flow(self):
        out = _settle(FourAxisArbiter(), lhx=120, lhy=0)
        # Sum-of-magnitudes clamps to 127 \u2192 full flow set-point.
        self.assertEqual(out["flow_sp"], 10000)


# ---------------------------------------------------------------------------
# MP-06 / MP-07 \u2014 arms raise / lower
# ---------------------------------------------------------------------------


class MP06_ArmsRaiseTests(unittest.TestCase):
    def test_arms_raise_only(self):
        out = _settle(FourAxisArbiter(), lhx=0, lhy=0, rhx=0, rhy=120)
        self.assertEqual(out["left_track"], 0)
        self.assertEqual(out["right_track"], 0)
        self.assertEqual(out["arms"], 120)
        self.assertEqual(out["bucket"], 0)


class MP07_ArmsLowerTests(unittest.TestCase):
    def test_arms_lower_only(self):
        out = _settle(FourAxisArbiter(), lhx=0, lhy=0, rhx=0, rhy=-120)
        self.assertEqual(out["arms"], -120)
        self.assertEqual(out["left_track"], 0)
        self.assertEqual(out["right_track"], 0)


# ---------------------------------------------------------------------------
# MP-08 / MP-09 \u2014 bucket curl / dump
# ---------------------------------------------------------------------------


class MP08_BucketCurlTests(unittest.TestCase):
    def test_bucket_curl_only(self):
        out = _settle(FourAxisArbiter(), lhx=0, lhy=0, rhx=120, rhy=0)
        self.assertEqual(out["bucket"], 120)
        self.assertEqual(out["arms"], 0)


class MP09_BucketDumpTests(unittest.TestCase):
    def test_bucket_dump_only(self):
        out = _settle(FourAxisArbiter(), lhx=0, lhy=0, rhx=-120, rhy=0)
        self.assertEqual(out["bucket"], -120)


# ---------------------------------------------------------------------------
# MP-10 \u2014 combined drive + arms (flow contention on single-EFC)
# ---------------------------------------------------------------------------


class MP10_DriveAndArmsTests(unittest.TestCase):
    def test_drive_and_arms_use_max_magnitude_for_flow(self):
        # lhy=60 (track mag 60), arms=120 \u2192 flow_sp uses arms.
        arb = FourAxisArbiter()
        out = _settle(arb, lhx=0, lhy=60, rhx=0, rhy=120)
        self.assertEqual(out["left_track"], 60)
        self.assertEqual(out["arms"], 120)
        # Flow set-point recovered: should match arms magnitude (120),
        # not track magnitude (60). Both same sign \u2192 max formula, not
        # spin boost.
        self.assertGreater(out["flow_sp"], 8500,
                           f"flow must scale with arms=120, "
                           f"got flow_sp={out['flow_sp']}")


# ---------------------------------------------------------------------------
# MP-11 \u2014 combined turn + bucket
# ---------------------------------------------------------------------------


class MP11_TurnAndBucketTests(unittest.TestCase):
    def test_turn_with_bucket_curl(self):
        # Pivot turn (one track 0, other 120) + bucket curl 80.
        # Same-sign tracks (one zero), bucket independent.
        out = _settle(FourAxisArbiter(), lhx=60, lhy=60, rhx=80, rhy=0)
        self.assertEqual(out["left_track"], 120)
        self.assertEqual(out["right_track"], 0)
        self.assertEqual(out["bucket"], 80)
        # No spin-turn (right=0 \u2192 not strictly opposite signs above DB).
        # flow_sp uses max(120, 0, 0, 80) = 120.

    def test_spin_with_bucket_does_combine(self):
        # Pure spin + bucket. Spin already triggers BC-24 boost.
        out = _settle(FourAxisArbiter(), lhx=80, lhy=0, rhx=80, rhy=0)
        # Tracks: (80, -80) \u2192 spin. Bucket: 80.
        self.assertEqual(out["left_track"], 80)
        self.assertEqual(out["right_track"], -80)
        self.assertEqual(out["bucket"], 80)
        # spin-turn track_mag = min(127, 160) = 127 dominates bucket.
        self.assertEqual(out["flow_sp"], 10000)


# ---------------------------------------------------------------------------
# MP-12 \u2014 float (build-gated, no SIL surface today)
# ---------------------------------------------------------------------------


class MP12_FloatTests(unittest.TestCase):
    @unittest.skip("MP-12 float is build-gated on hydraulic.spool_type "
                   "(BC-19); no SIL surface exists yet because the "
                   "FourAxisArbiter fixture does not model spool type.")
    def test_float_documented_only(self):  # pragma: no cover
        pass


# ---------------------------------------------------------------------------
# Cross-cutting: deadband filtering applies uniformly across MP-01..MP-09
# ---------------------------------------------------------------------------


class MP_CrossCutting_DeadbandTests(unittest.TestCase):
    def test_sub_deadband_inputs_produce_zero_outputs_on_all_axes(self):
        # Inputs small enough that the post-mix logical axes are also
        # sub-deadband: with lhx=lhy=5, left_track = 10 < AXIS_DEADBAND
        # = 13, right_track = 0. Arms / bucket use raw rh sticks so use
        # 5 there too.
        v = 5
        self.assertLess(v + v, AXIS_DEADBAND,
                        "test setup: post-mix value must be sub-deadband")
        out = _settle(FourAxisArbiter(), lhx=v, lhy=v, rhx=v, rhy=v)
        self.assertEqual(out["left_track"], 0)
        self.assertEqual(out["right_track"], 0)
        self.assertEqual(out["arms"], 0)
        self.assertEqual(out["bucket"], 0)
        self.assertEqual(out["flow_sp"], 0)


if __name__ == "__main__":
    unittest.main()
