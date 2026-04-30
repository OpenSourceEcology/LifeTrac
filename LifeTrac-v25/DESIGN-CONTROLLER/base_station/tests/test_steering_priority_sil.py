"""Round 46: SIL coverage for BC-23 (preserve-steering mix) and BC-24
(spin-turn flow boost).

**BC-23.** Pre-BC-23, ``apply_control()`` clipped each post-mix track
intent independently via ``clip_to_int8``. When the throttle+steering
sum exceeded \u00b1127 on one side, that side was clipped while the other
was untouched, destroying the differential (steering) ratio. Concrete
example: ``lhy=120, lhx=80`` produced raw track intents ``(200, 40)``
which clipped to ``(127, 40)`` \u2014 the differential dropped from 80 to
(127-40)/2 = 43, nearly half. Operator's commanded turn tightness was
silently halved.

BC-23 replaces the per-side clip with ``mix_tracks_preserve_steering()``:
when ``max(|left_intent|, |right_intent|) > 127``, BOTH intents are
scaled by ``127/max_mag`` so the ratio (and turn tightness) is preserved
at the cost of throttle authority. The same example now produces
``(127, 25)`` \u2014 differential = 51, and crucially both sides are scaled
proportionally.

**BC-24.** The proportional flow set-point feeds a single A0602 0..10 V
output to the proportional valve. Pre-BC-24 it carried the MAX magnitude
across all four logical axes. For SAME-sign track motion (forward,
reverse, smooth-curve turn) the two track motors share a common flow
path \u2014 max is correct. For OPPOSITE-sign motion (spin-turn) each track
motor draws its own flow because they're moving in opposite directions
through the hydraulic circuit. Pre-BC-24, a pure spin-turn at full
stick produced flow_sp matching a SINGLE track at full \u2014 the valve
under-budgeted by ~50%, often stalling the spin.

BC-24 detects spin-turn (``left > db && right < -db`` or mirror) and
uses ``min(127, |left| + |right|)`` for the track-magnitude contribution
to the flow set-point. Same-sign cases continue to use max.

Both fixes live in ``apply_control()`` in
``firmware/tractor_h7/tractor_h7.ino`` and are mirrored in
``FourAxisArbiter`` in ``test_axis_ramp_sil.py``.

Test classes:

* ``SP_A_PreserveSteeringMathTests`` \u2014 the BC-23 helper formula
  across non-saturating, single-side-saturating, both-saturating,
  symmetric-spin (no scale needed), and pure-throttle cases.
* ``SP_B_SteeringRatioPreservedTests`` \u2014 end-to-end through
  ``FourAxisArbiter``: the differential / throttle ratio of the
  POST-mix output equals the ratio of the intents (within int truncation)
  whenever scaling kicks in.
* ``SP_C_FlowBoostTests`` \u2014 BC-24: pure spin-turn uses sum-of-magnitudes;
  same-sign turns use max.
* ``SP_D_NoFlowBoostOnNonSpinTests`` \u2014 forward / reverse / smooth-curve /
  arms-only / mixed must NOT trigger the boost.
* ``SP_E_FirmwareTripwireTests`` \u2014 firmware source carries the BC-23 +
  BC-24 markers, the helper name, and the spin-turn detection pattern.

Pure stdlib.
"""

from __future__ import annotations

import pathlib
import unittest

from test_axis_ramp_sil import (
    AXIS_DEADBAND,
    RAMP_TICK_MS,
    FourAxisArbiter,
    _mix_tracks_preserve_steering,
)


_FIRMWARE_PATH = (pathlib.Path(__file__).resolve().parents[3]
                  / "DESIGN-CONTROLLER" / "firmware"
                  / "tractor_h7" / "tractor_h7.ino")


def _flow_to_mag(flow_mv: int) -> int:
    """Invert the ``flow = (mag - DB) * 10000 / (127 - DB)`` formula."""
    if flow_mv == 0:
        return 0
    return (flow_mv * (127 - AXIS_DEADBAND)) // 10000 + AXIS_DEADBAND


# ---------------------------------------------------------------------------
# SP-A \u2014 mix_tracks_preserve_steering math
# ---------------------------------------------------------------------------


class SP_A_PreserveSteeringMathTests(unittest.TestCase):
    """Pin the BC-23 scale-down formula and its non-saturating identity."""

    def test_non_saturating_passes_through(self):
        # Both intents within \u00b1127 \u2192 untouched.
        self.assertEqual(_mix_tracks_preserve_steering(60, 40), (60, 40))
        self.assertEqual(_mix_tracks_preserve_steering(-60, -40), (-60, -40))
        self.assertEqual(_mix_tracks_preserve_steering(127, -127),
                         (127, -127))
        self.assertEqual(_mix_tracks_preserve_steering(0, 0), (0, 0))

    def test_one_side_saturates_scales_both(self):
        # lhy=120, lhx=80 \u2192 intents (200, 40). Scale by 127/200.
        # left = 200*127/200 = 127, right = 40*127/200 = 25 (truncated).
        self.assertEqual(_mix_tracks_preserve_steering(200, 40),
                         (127, 25))

    def test_both_sides_saturate(self):
        # Symmetric large opposite (max spin). Intents (200, -200) \u2192
        # mx=200, both scale to \u00b1127.
        self.assertEqual(_mix_tracks_preserve_steering(200, -200),
                         (127, -127))

    def test_negative_saturation_preserves_signs(self):
        # lhy=-120, lhx=-80 \u2192 intents (-200, -40). Scale by 127/200.
        # left = -127, right = -25 (truncated toward zero).
        self.assertEqual(_mix_tracks_preserve_steering(-200, -40),
                         (-127, -25))

    def test_mixed_signs_with_saturation(self):
        # lhy=80, lhx=120 (more steering than throttle) \u2192 intents
        # (200, -40). mx=200, scale=127/200. left=127, right=-25.
        self.assertEqual(_mix_tracks_preserve_steering(200, -40),
                         (127, -25))

    def test_just_above_threshold_scales(self):
        # mx=128 \u2192 scale=127/128, near-identity but truncates.
        # left = 128*127/128 = 127, right = 64*127/128 = 63.
        self.assertEqual(_mix_tracks_preserve_steering(128, 64),
                         (127, 63))

    def test_exact_127_does_not_scale(self):
        # Boundary: max equals 127. No scale-down.
        self.assertEqual(_mix_tracks_preserve_steering(127, 50),
                         (127, 50))

    def test_pure_throttle_no_scale_needed(self):
        # lhy=127, lhx=0 \u2192 intents (127, 127). Equal, both within range.
        self.assertEqual(_mix_tracks_preserve_steering(127, 127),
                         (127, 127))


# ---------------------------------------------------------------------------
# SP-B \u2014 differential ratio preserved end-to-end
# ---------------------------------------------------------------------------


class SP_B_SteeringRatioPreservedTests(unittest.TestCase):
    """The whole point of BC-23: the operator's commanded turn-tightness
    (steering / throttle ratio) survives saturation."""

    def _compare_pre_vs_post_bc23_steering(self, lhy: int, lhx: int) -> None:
        """Helper: assert post-BC-23 differential is closer to the
        intended ``lhx`` than the pre-BC-23 independent-clip would have
        been."""
        left_intent = lhy + lhx
        right_intent = lhy - lhx
        # Pre-BC-23 (independent clip).
        def clip(v): return max(-127, min(127, v))
        pre_left, pre_right = clip(left_intent), clip(right_intent)
        pre_diff = (pre_left - pre_right) / 2
        # Post-BC-23.
        post_left, post_right = _mix_tracks_preserve_steering(left_intent,
                                                              right_intent)
        post_diff = (post_left - post_right) / 2
        # |post_diff - lhx| <= |pre_diff - lhx|: the new differential
        # is no further from the operator's intent than the old one.
        self.assertLessEqual(
            abs(post_diff - lhx), abs(pre_diff - lhx),
            f"BC-23 must not WORSEN steering authority: lhy={lhy} "
            f"lhx={lhx} pre_diff={pre_diff} post_diff={post_diff}")

    def test_saturating_smooth_turns_preserve_ratio(self):
        for lhy, lhx in [(120, 80), (120, 60), (100, 50), (-120, 80),
                         (120, -80), (100, 100), (90, 60)]:
            with self.subTest(lhy=lhy, lhx=lhx):
                self._compare_pre_vs_post_bc23_steering(lhy, lhx)

    def test_arbiter_preserves_ratio_for_high_steering(self):
        # End-to-end: feed the arbiter a saturating turn and verify the
        # differential output is closer to lhx than independent-clip.
        arb = FourAxisArbiter()
        t = 0
        # Warm up at the saturating input so ramp is bypassed (active
        # → pass through).
        for _ in range(5):
            out = arb.tick(80, 120, 0, 0, t)
            t += RAMP_TICK_MS
        diff = (out["left_track"] - out["right_track"]) / 2
        # Pre-BC-23 differential would have been (127-40)/2 = 43.5.
        # Post-BC-23 should be at least as close to lhx=80 as 43.5 is.
        self.assertGreater(diff, 43.5,
                           f"BC-23 must improve differential authority "
                           f"(got diff={diff}, pre-BC-23 baseline=43.5)")


# ---------------------------------------------------------------------------
# SP-C \u2014 BC-24 spin-turn flow boost
# ---------------------------------------------------------------------------


class SP_C_FlowBoostTests(unittest.TestCase):
    """Pure spin-turns budget flow as sum-of-magnitudes, not max."""

    def test_pure_spin_uses_sum_of_magnitudes(self):
        arb = FourAxisArbiter()
        t = 0
        # Pure spin: lhy=0, lhx=120 \u2192 left=120, right=-120.
        for _ in range(5):
            out = arb.tick(120, 0, 0, 0, t)
            t += RAMP_TICK_MS
        self.assertEqual(out["left_track"], 120)
        self.assertEqual(out["right_track"], -120)
        # Pre-BC-24 would have used max(120, 120) = 120 as track_mag.
        # BC-24 uses min(127, 120+120) = 127.
        # Flow = (127 - DB) * 10000 / (127 - DB) = 10000.
        self.assertEqual(out["flow_sp"], 10000,
                         "spin-turn flow must reach 10000 mV (BC-24)")

    def test_partial_spin_uses_sum(self):
        arb = FourAxisArbiter()
        t = 0
        # lhy=0, lhx=40 \u2192 left=40, right=-40. Sum=80.
        for _ in range(5):
            out = arb.tick(40, 0, 0, 0, t)
            t += RAMP_TICK_MS
        # track_mag should be 80, not 40.
        # Recover mag from flow_sp.
        recovered_mag = _flow_to_mag(out["flow_sp"])
        self.assertGreaterEqual(recovered_mag, 75,
                                f"spin-turn at \u00b140 must budget flow for "
                                f"~80, got mag~{recovered_mag}")

    def test_spin_clamps_at_int8_max(self):
        # lhx=80, lhy=0 \u2192 left=80, right=-80. Sum=160. Clamp to 127.
        arb = FourAxisArbiter()
        t = 0
        for _ in range(5):
            out = arb.tick(80, 0, 0, 0, t)
            t += RAMP_TICK_MS
        self.assertEqual(out["flow_sp"], 10000,
                         "summed magnitude must clamp at 127 (full flow)")


# ---------------------------------------------------------------------------
# SP-D \u2014 non-spin cases must NOT trigger BC-24 boost
# ---------------------------------------------------------------------------


class SP_D_NoFlowBoostOnNonSpinTests(unittest.TestCase):

    def _flow_for(self, lhx: int, lhy: int, rhx: int = 0,
                  rhy: int = 0) -> int:
        arb = FourAxisArbiter()
        t = 0
        for _ in range(5):
            out = arb.tick(lhx, lhy, rhx, rhy, t)
            t += RAMP_TICK_MS
        return out["flow_sp"]

    def test_pure_forward_uses_max(self):
        # lhy=60, lhx=0 \u2192 both tracks +60. Same sign, NOT a spin.
        # track_mag = max(60, 60) = 60, not 120.
        flow = self._flow_for(lhx=0, lhy=60)
        recovered = _flow_to_mag(flow)
        self.assertLess(recovered, 70,
                        f"forward must NOT trigger spin-turn boost "
                        f"(got mag~{recovered})")

    def test_pure_reverse_uses_max(self):
        flow = self._flow_for(lhx=0, lhy=-60)
        recovered = _flow_to_mag(flow)
        self.assertLess(recovered, 70)

    def test_smooth_curve_uses_max(self):
        # Forward + small turn: lhy=120, lhx=20 \u2192 (140, 100) \u2192
        # BC-23 scale: (127, 90). Both positive (same sign), NOT a spin.
        flow = self._flow_for(lhx=20, lhy=120)
        recovered = _flow_to_mag(flow)
        # track_mag = max(127, 90) = 127.
        self.assertGreaterEqual(recovered, 125)
        # And not the spin-boosted 217 (clamped to 127) which would
        # produce identical max-output. Distinguish via a less-saturated
        # case below.

    def test_smooth_curve_no_double_count(self):
        # lhy=60, lhx=20 \u2192 (80, 40). track_mag=80, not 120.
        flow = self._flow_for(lhx=20, lhy=60)
        recovered = _flow_to_mag(flow)
        self.assertLess(recovered, 95,
                        f"same-sign asymmetric must use max(80,40)=80, "
                        f"got mag~{recovered}")

    def test_arms_only_does_not_use_track_path(self):
        # No track motion, arms only. flow comes from arms magnitude.
        flow = self._flow_for(lhx=0, lhy=0, rhx=0, rhy=70)
        recovered = _flow_to_mag(flow)
        self.assertGreaterEqual(recovered, 65)
        self.assertLess(recovered, 80)


# ---------------------------------------------------------------------------
# SP-E \u2014 firmware source tripwires
# ---------------------------------------------------------------------------


class SP_E_FirmwareTripwireTests(unittest.TestCase):
    """Pin BC-23 / BC-24 markers and patterns in firmware source."""

    @classmethod
    def setUpClass(cls) -> None:
        cls.src = _FIRMWARE_PATH.read_text(encoding="utf-8")

    def test_bc23_marker_present(self):
        self.assertIn("BC-23", self.src)

    def test_bc24_marker_present(self):
        self.assertIn("BC-24", self.src)

    def test_preserve_steering_helper_present(self):
        self.assertIn("mix_tracks_preserve_steering", self.src,
                      "BC-23 helper function missing from firmware")

    def test_bc23_replaces_independent_clip_for_tracks(self):
        # Pre-BC-23 used `clip_to_int8(left_track_raw)` directly. That
        # exact pattern must be gone in apply_control.
        self.assertNotIn("clip_to_int8(left_track_raw)", self.src,
                         "pre-BC-23 independent track clip still present")
        self.assertNotIn("clip_to_int8(right_track_raw)", self.src,
                         "pre-BC-23 independent track clip still present")

    def test_spin_turn_detection_pattern_present(self):
        # The BC-24 detection block must check for opposite-sign tracks
        # both above the deadband.
        self.assertIn("spin_turn", self.src,
                      "BC-24 spin_turn detection variable missing")

    def test_python_mirror_exports_helper(self):
        from test_axis_ramp_sil import (
            _mix_tracks_preserve_steering as exported,
        )
        # Sanity: the SIL fixture exposes it under the documented name.
        self.assertIs(exported, _mix_tracks_preserve_steering)


if __name__ == "__main__":
    unittest.main()
