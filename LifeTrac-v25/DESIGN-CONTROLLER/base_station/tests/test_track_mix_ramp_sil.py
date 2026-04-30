"""Round 44 (BC-21 / K-A4): SIL for mix-then-ramp + coordinated bilateral
track stop.

Pins the new behaviours introduced when ramping moved from raw stick
axes onto post-mix logical axes (left_track / right_track / arms /
bucket). The shared ``FourAxisArbiter`` in ``test_axis_ramp_sil.py``
already mirrors the post-BC-21 ``apply_control()`` pipeline; this file
adds the BC-21-specific assertions that don't fit the W4-05 / W4-06
shape.

Cases pinned here (RT = "Round 44 Track-mix-Ramp"):

* **RT-A — Mixing math.** ``left_track = clip(lhy + lhx, ±127)``,
  ``right_track = clip(lhy - lhx, ±127)``. Pure stick-input cases
  (forward only, spin-turn, smooth-curve turn, saturation).
* **RT-B — Spin-turn coil activation.** Pre-BC-21 a pure spin-turn
  (lhy=0, lhx=full) produced ZERO drive coils because only ``lhy``
  drove the coil bits. Post-BC-21 each track side energises in the
  correct direction.
* **RT-C — Smooth-curve turn does not step the right track.** This
  is the central correctness claim of BC-21. Ramping ``lhx`` on a
  forward-pinned ``lhy`` (the canonical curve manoeuvre) must NOT
  cause a step on the right-track effective value as ``lhx`` ramps
  in linearly from 0 — because the ramp now operates on the
  resulting logical track signals.
* **RT-D — K-A4 coordinated bilateral track stop.** When both tracks
  release together (lhy=120 → 0 with lhx=0 throughout), both ramps
  use a SHARED duration computed from the larger starting magnitude.
  Asserts both tracks reach 0 in the same tick.
* **RT-E — K-A4 with mismatched starting magnitudes.** Smooth-curve
  release where left_track started higher than right_track. Both
  must hit zero at the same wallclock time using the LARGER magnitude's
  ladder duration; the smaller-magnitude track follows a steeper
  effective slope.
* **RT-F — Single track release does NOT trigger K-A4.** When only
  one track is active at release time (e.g. lhx=full spin-turn
  released with lhy=0), only that track's logical axis ramps; the
  silent track stays at effective=0 and no forced duration is
  applied.
* **RT-G — Source tripwire.** Pin that the firmware comments call out
  BC-21 / K-A4 in apply_control so future renames don't silently
  drift.

Pure stdlib.
"""

from __future__ import annotations

import pathlib
import unittest

from test_axis_ramp_sil import (
    AXIS_DEADBAND,
    RAMP_TICK_MS,
    VB_DRIVE_LF,
    VB_DRIVE_LR,
    VB_DRIVE_RF,
    VB_DRIVE_RR,
    FourAxisArbiter,
    _clip_to_int8,
    _mix_tracks_preserve_steering,
    ramp_duration_ms,
)


def mix_tracks(lhy: int, lhx: int) -> tuple[int, int]:
    """Mirror the BC-21 mixing in apply_control(), with BC-23
    preserve-steering proportional scale-down on saturation."""
    return _mix_tracks_preserve_steering(lhy + lhx, lhy - lhx)


# ---------------------------------------------------------------------------
# RT-A — mixing math
# ---------------------------------------------------------------------------


class RT_A_MixingMathTests(unittest.TestCase):
    """Pin the ``left_track = lhy + lhx`` / ``right_track = lhy - lhx``
    formula plus int8 saturation clipping."""

    def test_forward_only(self):
        # Pure forward: both tracks equal lhy.
        self.assertEqual(mix_tracks(100, 0), (100, 100))

    def test_reverse_only(self):
        self.assertEqual(mix_tracks(-100, 0), (-100, -100))

    def test_spin_in_place_right(self):
        # lhx=+full, lhy=0 → left forward, right reverse.
        self.assertEqual(mix_tracks(0, 100), (100, -100))

    def test_spin_in_place_left(self):
        self.assertEqual(mix_tracks(0, -100), (-100, 100))

    def test_smooth_curve_forward_right(self):
        # Forward + slight right turn: left faster than right. Under
        # BC-23 preserve-steering, intents (130, 70) get scaled by
        # 127/130 since the left intent saturates: left=127, right=68.
        self.assertEqual(mix_tracks(100, 30), (127, 68))

    def test_saturation_positive(self):
        # lhy=100 + lhx=100 = 200 (left intent), right intent = 0.
        # BC-23 scales both by 127/200: left=127, right=0.
        self.assertEqual(mix_tracks(100, 100), (127, 0))

    def test_saturation_negative(self):
        self.assertEqual(mix_tracks(-100, -100), (-127, 0))

    def test_zero(self):
        self.assertEqual(mix_tracks(0, 0), (0, 0))


# ---------------------------------------------------------------------------
# RT-B — spin-turn coil activation (the second BC-21 bug fix)
# ---------------------------------------------------------------------------


class RT_B_SpinTurnCoilTests(unittest.TestCase):
    """Pre-BC-21 only ``lhy`` drove drive coils, so pure spin-turns
    produced no drive-coil activation. Post-BC-21 each side energises
    in the correct direction."""

    def test_spin_right_energises_lf_and_rr(self):
        # lhx=+full, lhy=0 → left fwd (LF), right rev (RR).
        arb = FourAxisArbiter()
        out = arb.tick(120, 0, 0, 0, 0)
        self.assertTrue(out["coils"] & (1 << VB_DRIVE_LF),
                        "spin-right must energise left-forward coil")
        self.assertTrue(out["coils"] & (1 << VB_DRIVE_RR),
                        "spin-right must energise right-reverse coil")
        # And NOT the opposite-direction coils on either side.
        self.assertFalse(out["coils"] & (1 << VB_DRIVE_LR))
        self.assertFalse(out["coils"] & (1 << VB_DRIVE_RF))

    def test_spin_left_energises_lr_and_rf(self):
        arb = FourAxisArbiter()
        out = arb.tick(-120, 0, 0, 0, 0)
        self.assertTrue(out["coils"] & (1 << VB_DRIVE_LR))
        self.assertTrue(out["coils"] & (1 << VB_DRIVE_RF))
        self.assertFalse(out["coils"] & (1 << VB_DRIVE_LF))
        self.assertFalse(out["coils"] & (1 << VB_DRIVE_RR))

    def test_pure_forward_energises_both_forward_coils(self):
        arb = FourAxisArbiter()
        out = arb.tick(0, 120, 0, 0, 0)
        self.assertTrue(out["coils"] & (1 << VB_DRIVE_LF))
        self.assertTrue(out["coils"] & (1 << VB_DRIVE_RF))
        self.assertFalse(out["coils"] & (1 << VB_DRIVE_LR))
        self.assertFalse(out["coils"] & (1 << VB_DRIVE_RR))


# ---------------------------------------------------------------------------
# RT-C — smooth-curve turn does not step the right track
# ---------------------------------------------------------------------------


class RT_C_SmoothCurveTurnTests(unittest.TestCase):
    """The central BC-21 correctness claim: ramping a stick axis
    smoothly into a previously-idle channel must not step the resulting
    logical track signal. Pre-BC-21 the right track stepped because the
    raw stick ramp short-circuited (`is_now=true`) on the previously-
    inactive lhx axis."""

    def test_smooth_lhx_ramp_in_does_not_step_right_track(self):
        # Hold lhy=120 (forward) steady. Ramp lhx from 0 → 60 over
        # 1000 ms (20 ticks). The operator's intent is "gentle right
        # turn while moving forward", so right_track should slew from
        # 120 down toward 60 SMOOTHLY (or at least monotonically).
        # Post-BC-21 right_track tracks (lhy - lhx) directly because it
        # is now an active channel from tick 1 (no ramp engagement).
        arb = FourAxisArbiter()
        # Warm up forward.
        t = 0
        while t < 200:
            arb.tick(0, 120, 0, 0, t)
            t += RAMP_TICK_MS
        # right_track should be at 120 (lhy=120, lhx=0 → 120-0).
        self.assertEqual(arb.right_track.effective, 120)

        # Now ramp lhx in over 20 ticks.
        right_samples = []
        for step in range(0, 21):
            lhx = (60 * step) // 20
            out = arb.tick(lhx, 120, 0, 0, t)
            right_samples.append(out["right_track"])
            t += RAMP_TICK_MS

        # right_track should DECREASE monotonically from 120 toward its
        # final value. Under BC-23 preserve-steering, intents (180, 60)
        # at the end scale by 127/180: right = 60*127/180 = 42.
        for prev, cur in zip(right_samples, right_samples[1:]):
            self.assertLessEqual(cur, prev,
                                 f"right_track must be monotonically "
                                 f"non-increasing during smooth-curve "
                                 f"turn (samples={right_samples})")
        # Final value should be ~42 (BC-23 scale).
        self.assertAlmostEqual(right_samples[-1], 42, delta=1)
        # Left track should INCREASE toward 127 (saturating).
        # We don't sample it explicitly here — RT-A pinned the math.

    def test_pre_bc21_step_would_have_been_visible(self):
        """Sanity: a single-tick lhx jump from 0 to 60 (the worst case
        the BC-21 fix prevents on the wire) must produce right_track
        EXACTLY one step from 120 → 42 (BC-23 scaled) because right_track
        was already active and ramp short-circuits via is_now=true. This
        pins that the ramp engine still passes through active logical axis
        commands without artificial smoothing — BC-21 isn't adding
        ramping to active channels, only ensuring the ramping happens
        on the right channel."""
        arb = FourAxisArbiter()
        t = 0
        while t < 200:
            arb.tick(0, 120, 0, 0, t)
            t += RAMP_TICK_MS

        out_before = arb.tick(0, 120, 0, 0, t)
        self.assertEqual(out_before["right_track"], 120)
        t += RAMP_TICK_MS
        out_after = arb.tick(60, 120, 0, 0, t)
        # Post-mix intents: (180, 60). BC-23 scale by 127/180:
        # left=127, right = 60 * 127 / 180 = 42 (truncated). Active →
        # pass through.
        self.assertEqual(out_after["right_track"], 42)


# ---------------------------------------------------------------------------
# RT-D — K-A4 coordinated bilateral track stop
# ---------------------------------------------------------------------------


class RT_D_CoordinatedBilateralStopTests(unittest.TestCase):
    """K-A4: when both tracks transition active → released in the same
    tick, share a single ramp duration so the tractor doesn't pivot
    during release."""

    def test_symmetric_release_uses_max_magnitude_ladder(self):
        arb = FourAxisArbiter()
        t = 0
        # Warm up: pure forward at lhy=120 → both tracks at 120.
        while t < 200:
            arb.tick(0, 120, 0, 0, t)
            t += RAMP_TICK_MS
        self.assertEqual(arb.left_track.effective, 120)
        self.assertEqual(arb.right_track.effective, 120)

        # Release.
        arb.tick(0, 0, 0, 0, t)
        # Both tracks should be ramping with the same duration.
        self.assertTrue(arb.left_track.ramping)
        self.assertTrue(arb.right_track.ramping)
        expected_dur = ramp_duration_ms(120, is_arm=False)
        self.assertEqual(arb.left_track.duration_ms, expected_dur)
        self.assertEqual(arb.right_track.duration_ms, expected_dur)

    def test_asymmetric_release_uses_larger_magnitude(self):
        # Smooth-curve turn: lhy=120, lhx=30. BC-23 intents (150, 90)
        # scale by 127/150 → (127, 76). On release both transition
        # together; K-A4 should pin both to the LARGER magnitude's
        # ladder duration (left=127 → 2000 ms).
        arb = FourAxisArbiter()
        t = 0
        while t < 200:
            arb.tick(30, 120, 0, 0, t)
            t += RAMP_TICK_MS
        self.assertEqual(arb.left_track.effective, 127)
        self.assertEqual(arb.right_track.effective, 76)

        arb.tick(0, 0, 0, 0, t)
        self.assertTrue(arb.left_track.ramping)
        self.assertTrue(arb.right_track.ramping)
        # Larger magnitude is 127 → 2000 ms ladder. Both must use 2000.
        self.assertEqual(arb.left_track.duration_ms, 2000)
        self.assertEqual(arb.right_track.duration_ms, 2000)

    def test_both_tracks_reach_zero_in_same_tick(self):
        # Asymmetric release: left=127, right=90. Both pinned to 2000 ms.
        # Both effective values should reach 0 in the same tick.
        arb = FourAxisArbiter()
        t = 0
        while t < 200:
            arb.tick(30, 120, 0, 0, t)
            t += RAMP_TICK_MS
        release_t = t
        # Step until both tracks are zero.
        left_zero_t = None
        right_zero_t = None
        while t <= release_t + 2200:
            out = arb.tick(0, 0, 0, 0, t)
            if left_zero_t is None and out["left_track"] == 0:
                left_zero_t = t
            if right_zero_t is None and out["right_track"] == 0:
                right_zero_t = t
            t += RAMP_TICK_MS
        self.assertIsNotNone(left_zero_t)
        self.assertIsNotNone(right_zero_t)
        self.assertEqual(left_zero_t, right_zero_t,
                         f"K-A4: both tracks must hit 0 in the same tick "
                         f"(left={left_zero_t}, right={right_zero_t})")

    def test_pre_ka4_simulation_would_pivot(self):
        """Sanity: WITHOUT K-A4 (using each track's own magnitude
        ladder) the asymmetric case has different durations. We
        compute what those would have been to confirm K-A4 actually
        changed something."""
        unforced_left = ramp_duration_ms(127, is_arm=False)   # 2000
        unforced_right = ramp_duration_ms(90, is_arm=False)   # 1000
        self.assertNotEqual(unforced_left, unforced_right,
                            "this test only meaningful when the asymmetric "
                            "case would have used different durations")


# ---------------------------------------------------------------------------
# RT-E — single-track release (e.g. spin-turn) does not trigger K-A4
# ---------------------------------------------------------------------------


class RT_F_SingleTrackReleaseTests(unittest.TestCase):
    """When only one track is active at release time, only that track
    ramps. The silent track stays at effective=0 and no forced duration
    is applied."""

    def test_spin_turn_release_only_ramps_the_active_pair(self):
        # Spin-turn lhx=+120 → left=120, right=-120. Both active.
        # Release: K-A4 SHOULD trigger here too because both are active.
        arb = FourAxisArbiter()
        t = 0
        while t < 200:
            arb.tick(120, 0, 0, 0, t)
            t += RAMP_TICK_MS
        self.assertEqual(arb.left_track.effective, 120)
        self.assertEqual(arb.right_track.effective, -120)

        arb.tick(0, 0, 0, 0, t)
        # Both pinned to max(|120|, |-120|) = 120 → 2000 ms.
        self.assertTrue(arb.left_track.ramping)
        self.assertTrue(arb.right_track.ramping)
        self.assertEqual(arb.left_track.duration_ms, 2000)
        self.assertEqual(arb.right_track.duration_ms, 2000)

    def test_silent_track_untouched_when_only_other_is_active(self):
        # Drive only via combined lhy + lhx that gives left=120, right=0.
        # Specifically: lhy=60, lhx=60 → left=120, right=0.
        arb = FourAxisArbiter()
        t = 0
        while t < 200:
            arb.tick(60, 60, 0, 0, t)
            t += RAMP_TICK_MS
        self.assertEqual(arb.left_track.effective, 120)
        self.assertEqual(arb.right_track.effective, 0)
        self.assertFalse(arb.right_track.ramping)

        # Release. Only left was active → left ramps; right was already 0
        # so it has no transition to make and no ramp begins.
        arb.tick(0, 0, 0, 0, t)
        self.assertTrue(arb.left_track.ramping)
        self.assertFalse(arb.right_track.ramping)
        # Left uses its OWN magnitude (120), not a forced shared value
        # different from the natural ladder. Both are 2000 here so this
        # cell is not differentiating; the test pins that the call does
        # not crash and that right_track stays uninvolved.
        self.assertEqual(arb.left_track.duration_ms, 2000)


# ---------------------------------------------------------------------------
# RT-G — firmware source tripwire
# ---------------------------------------------------------------------------


_FIRMWARE_PATH = (pathlib.Path(__file__).resolve().parents[3]
                  / "DESIGN-CONTROLLER" / "firmware" / "tractor_h7"
                  / "tractor_h7.ino")


class RT_G_SourceTripwireTests(unittest.TestCase):
    """Pin the BC-21 / K-A4 markers in apply_control() so a future
    rename or revert can't silently drift away from this SIL."""

    @classmethod
    def setUpClass(cls):
        cls.src = _FIRMWARE_PATH.read_text(encoding="utf-8")

    def test_bc21_marker_present(self):
        self.assertIn("BC-21", self.src,
                      "BC-21 marker missing from tractor_h7.ino — "
                      "did the mix-then-ramp logic get reverted?")

    def test_ka4_marker_present(self):
        self.assertIn("K-A4", self.src,
                      "K-A4 marker missing from tractor_h7.ino — "
                      "coordinated bilateral track stop is gone")

    def test_logical_axis_ramp_globals_present(self):
        for name in ("g_ramp_left_track", "g_ramp_right_track",
                     "g_ramp_arms", "g_ramp_bucket"):
            with self.subTest(name=name):
                self.assertIn(name, self.src)

    def test_old_stick_axis_globals_removed(self):
        for name in ("g_ramp_lhx", "g_ramp_lhy", "g_ramp_rhx", "g_ramp_rhy"):
            with self.subTest(name=name):
                self.assertNotIn(name, self.src,
                                 f"stale stick-axis ramp global {name!r} "
                                 "still present after BC-21 — should be "
                                 "renamed to logical-axis equivalent")

    def test_clip_to_int8_helper_present(self):
        self.assertIn("clip_to_int8", self.src,
                      "post-mix saturating cast helper missing")

    def test_per_side_drive_coil_mapping(self):
        # Each VB_DRIVE_* coil should now appear independently (not OR'd
        # in pairs) in the coil-mapping block. Filter to lines that look
        # like coil assignments (`coils |= ...`) and assert no such line
        # OR's both LF and RF together — pre-BC-21 they were paired.
        coil_lines = [ln for ln in self.src.splitlines()
                      if "coils |=" in ln
                      and ("VB_DRIVE_LF" in ln or "VB_DRIVE_RF" in ln)]
        self.assertTrue(coil_lines,
                        "expected at least one drive-coil assignment line")
        for ln in coil_lines:
            with self.subTest(line=ln.strip()):
                self.assertFalse(
                    "VB_DRIVE_LF" in ln and "VB_DRIVE_RF" in ln,
                    "VB_DRIVE_LF and VB_DRIVE_RF must be set on "
                    "independent assignment lines (one per logical "
                    "track) post-BC-21",
                )


if __name__ == "__main__":
    unittest.main()
