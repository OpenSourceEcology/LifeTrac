"""Round 45: SIL coverage for BC-22 reversal brake.

The M7 ``step_axis_ramp()`` in ``firmware/tractor_h7/tractor_h7.ino`` now
detects a same-tick sign-flip on the raw axis input while the axis is
still energised (``was_active && is_now && opposite signs``), converts
the operator's reversal command into a decay-to-zero ramp using the
existing release ladder, and on completion holds the axis at zero for
``REVERSAL_BRAKE_MS`` (100 ms = 2 arbiter ticks) before allowing the new
direction to pass through. This prevents the spool slam / cavitation
pair documented in
``DESIGN-KINEMATICS/REVERSAL_HANDLING.md``.

This file is a pure-stdlib SIL fixture that uses the Python mirror of
``step_axis_ramp`` from ``test_axis_ramp_sil.py``. Test classes:

* ``BR_A_DecayPhaseTests`` \u2014 sign-flip triggers a release-ladder ramp
  to zero (not an instant snap to the new direction).
* ``BR_B_SettleWindowTests`` \u2014 after decay reaches zero, the axis is
  pinned at zero for the full ``REVERSAL_BRAKE_MS`` window regardless
  of subsequent operator input.
* ``BR_C_PostBrakeResumptionTests`` \u2014 once the brake elapses the
  axis accepts whatever operator input is current that tick (including
  zero).
* ``BR_D_NonReversalNoBrakeTests`` \u2014 same-sign jumps and
  release-from-active (no sign flip) do NOT trigger BC-22.
* ``BR_E_ArmReversalTests`` \u2014 arm axes use the arm ladder for the
  decay phase; settle window is the same shared constant.
* ``BR_F_SourceTripwireTests`` \u2014 firmware source carries the
  ``BC-22`` marker, ``REVERSAL_BRAKE_MS`` constant, and
  ``brake_until_ms`` field name; Python mirror exports the constant.

Pure stdlib.
"""

from __future__ import annotations

import pathlib
import unittest

from test_axis_ramp_sil import (
    AXIS_DEADBAND,
    RAMP_TICK_MS,
    REVERSAL_BRAKE_MS,
    AxisRamp,
    ramp_duration_ms,
    step_axis_ramp,
)


def _drive_until_steady(r: AxisRamp, raw: int, is_arm: bool,
                        now_ms: int = 0, ticks: int = 5) -> int:
    """Hold ``raw`` for ``ticks`` arbiter cycles and return the final
    wallclock time. After this ``r.effective`` equals ``raw``."""
    for _ in range(ticks):
        step_axis_ramp(r, raw, is_arm, False, now_ms)
        now_ms += RAMP_TICK_MS
    return now_ms


class BR_A_DecayPhaseTests(unittest.TestCase):
    """BC-22 detects a sign-flip and converts it into a decay ramp."""

    def test_full_reversal_does_not_snap_to_new_direction(self):
        # Pre-BC-22 a stick flip from +127 to -127 set effective = -127
        # in the same tick. Post-BC-22 the first tick after the flip
        # carries a ramp-decayed value with the OLD sign (positive).
        r = AxisRamp()
        now = _drive_until_steady(r, 127, is_arm=False)
        # Operator slams stick to -127.
        out = step_axis_ramp(r, -127, is_arm=False,
                             other_active=False, now_ms=now)
        self.assertGreater(out, 0,
                           "first reversal tick must still carry the "
                           "old sign (decay phase, not snap)")
        self.assertTrue(r.ramping)
        self.assertTrue(r.reversal_pending)
        self.assertEqual(r.start, 127,
                         "decay starts from the pre-reversal effective")

    def test_decay_uses_release_ladder_duration(self):
        # Reversal from +127 should pick the same 2000ms track ladder
        # the release ramp would have used.
        r = AxisRamp()
        now = _drive_until_steady(r, 127, is_arm=False)
        step_axis_ramp(r, -127, is_arm=False,
                       other_active=False, now_ms=now)
        self.assertEqual(r.duration_ms,
                         ramp_duration_ms(127, is_arm=False))

    def test_decay_phase_is_monotonically_non_increasing(self):
        # While decaying, |effective| must never go up. Sample every
        # arbiter tick across the decay window.
        r = AxisRamp()
        now = _drive_until_steady(r, 127, is_arm=False)
        decay_ms = ramp_duration_ms(127, is_arm=False)
        prev = r.effective
        t = now
        # Hold the reversed stick the whole time.
        while t <= now + decay_ms:
            out = step_axis_ramp(r, -127, is_arm=False,
                                 other_active=False, now_ms=t)
            with self.subTest(t_ms=t - now):
                self.assertLessEqual(out, prev,
                                     "decay magnitude must not grow")
                self.assertGreaterEqual(out, 0,
                                        "decay must not undershoot zero "
                                        "(BC-22 enters settle phase first)")
            prev = out
            t += RAMP_TICK_MS

    def test_decay_completion_arms_brake_window(self):
        r = AxisRamp()
        now = _drive_until_steady(r, 127, is_arm=False)
        decay_ms = ramp_duration_ms(127, is_arm=False)
        # Trigger the reversal at t=now (this starts the decay ramp).
        step_axis_ramp(r, -127, is_arm=False,
                       other_active=False, now_ms=now)
        # Advance to the deadline tick where elapsed >= duration_ms.
        deadline = now + decay_ms
        step_axis_ramp(r, -127, is_arm=False,
                       other_active=False, now_ms=deadline)
        self.assertEqual(r.effective, 0)
        self.assertFalse(r.ramping)
        self.assertNotEqual(r.brake_until_ms, 0,
                            "brake window must be armed at decay end")
        self.assertEqual(r.brake_until_ms, deadline + REVERSAL_BRAKE_MS)
        self.assertFalse(r.reversal_pending,
                         "pending flag clears once brake window is armed")


class BR_B_SettleWindowTests(unittest.TestCase):
    """Settle phase pins effective to zero for REVERSAL_BRAKE_MS."""

    def _enter_settle(self) -> tuple[AxisRamp, int]:
        r = AxisRamp()
        now = _drive_until_steady(r, 127, is_arm=False)
        decay_ms = ramp_duration_ms(127, is_arm=False)
        # Trigger reversal at t=now (decay starts) then jump to deadline.
        step_axis_ramp(r, -127, is_arm=False, other_active=False, now_ms=now)
        deadline = now + decay_ms
        step_axis_ramp(r, -127, is_arm=False,
                       other_active=False, now_ms=deadline)
        # Brake is now armed; deadline is the wallclock time at arm.
        return r, deadline

    def test_settle_holds_zero_for_full_window(self):
        r, t0 = self._enter_settle()
        # Sample every arbiter tick across the settle window with
        # the reversed stick still held at -127.
        t = t0 + RAMP_TICK_MS
        while t < t0 + REVERSAL_BRAKE_MS:
            out = step_axis_ramp(r, -127, is_arm=False,
                                 other_active=False, now_ms=t)
            with self.subTest(t_ms=t - t0):
                self.assertEqual(out, 0,
                                 "settle phase must hold effective at zero")
            t += RAMP_TICK_MS

    def test_settle_ignores_input_changes(self):
        # Even if the operator wiggles the stick during settle, the axis
        # must stay at zero.
        r, t0 = self._enter_settle()
        for inp in (-127, 0, 127, -64, 64):
            with self.subTest(input=inp):
                out = step_axis_ramp(r, inp, is_arm=False,
                                     other_active=False,
                                     now_ms=t0 + RAMP_TICK_MS)
                self.assertEqual(out, 0)
                self.assertEqual(r.effective, 0)

    def test_settle_window_constant_is_two_ticks(self):
        # Sanity: the design says 2 arbiter ticks. Pin it.
        self.assertEqual(REVERSAL_BRAKE_MS, 2 * RAMP_TICK_MS)


class BR_C_PostBrakeResumptionTests(unittest.TestCase):
    """After the brake window elapses, normal pass-through resumes."""

    def test_brake_clears_and_new_direction_passes_through(self):
        r = AxisRamp()
        now = _drive_until_steady(r, 127, is_arm=False)
        decay_ms = ramp_duration_ms(127, is_arm=False)
        step_axis_ramp(r, -127, is_arm=False, other_active=False, now_ms=now)
        deadline = now + decay_ms
        step_axis_ramp(r, -127, is_arm=False,
                       other_active=False, now_ms=deadline)
        # Jump past the brake window.
        t_after = deadline + REVERSAL_BRAKE_MS + RAMP_TICK_MS
        out = step_axis_ramp(r, -127, is_arm=False,
                             other_active=False, now_ms=t_after)
        self.assertEqual(out, -127,
                         "post-brake the new direction passes through "
                         "(snap-on-activation contract preserved)")
        self.assertEqual(r.brake_until_ms, 0,
                         "brake state cleared after window expiry")
        self.assertFalse(r.ramping)

    def test_post_brake_with_zero_input_stays_at_zero(self):
        # If the operator releases the stick during settle, post-brake
        # the axis stays at zero (no ghost ramp).
        r = AxisRamp()
        now = _drive_until_steady(r, 127, is_arm=False)
        decay_ms = ramp_duration_ms(127, is_arm=False)
        step_axis_ramp(r, -127, is_arm=False, other_active=False, now_ms=now)
        deadline = now + decay_ms
        step_axis_ramp(r, -127, is_arm=False, other_active=False,
                       now_ms=deadline)
        # Operator lets go.
        t_after = deadline + REVERSAL_BRAKE_MS + RAMP_TICK_MS
        out = step_axis_ramp(r, 0, is_arm=False,
                             other_active=False, now_ms=t_after)
        self.assertEqual(out, 0)
        self.assertFalse(r.ramping,
                         "no ramp should be re-armed when input is zero "
                         "post-brake (was_active synthetic-cleared)")


class BR_D_NonReversalNoBrakeTests(unittest.TestCase):
    """Cases that look superficially similar but must NOT brake."""

    def test_same_sign_magnitude_jump_does_not_brake(self):
        # Stick goes +127 → +60. Same sign. Pass-through, no ramp,
        # no brake.
        r = AxisRamp()
        now = _drive_until_steady(r, 127, is_arm=False)
        out = step_axis_ramp(r, 60, is_arm=False,
                             other_active=False, now_ms=now)
        self.assertEqual(out, 60)
        self.assertFalse(r.ramping)
        self.assertFalse(r.reversal_pending)
        self.assertEqual(r.brake_until_ms, 0)

    def test_release_to_zero_does_not_brake(self):
        # Plain release. Triggers the standard release ramp, not BC-22.
        r = AxisRamp()
        now = _drive_until_steady(r, 127, is_arm=False)
        step_axis_ramp(r, 0, is_arm=False, other_active=False, now_ms=now)
        self.assertTrue(r.ramping)
        self.assertFalse(r.reversal_pending,
                         "plain release must not arm BC-22 reversal flag")
        # And once the ramp completes the brake window stays clear.
        decay_ms = ramp_duration_ms(127, is_arm=False)
        step_axis_ramp(r, 0, is_arm=False, other_active=False,
                       now_ms=now + decay_ms)
        self.assertEqual(r.brake_until_ms, 0,
                         "release-ramp completion must NOT arm a brake")

    def test_below_deadband_reversal_passes_through(self):
        # Tiny stick movements (within ±deadband on both sides) should
        # not trigger BC-22 — was_active is false to start with.
        r = AxisRamp()
        # effective starts at 0. Stick goes to +5 (below deadband) → 0
        # then to -5 (still below deadband). axis_active is false on
        # both, so step_axis_ramp falls through to ``r.effective = 0``.
        now = 0
        out = step_axis_ramp(r, 5, is_arm=False,
                             other_active=False, now_ms=now)
        self.assertEqual(out, 0)
        out = step_axis_ramp(r, -5, is_arm=False,
                             other_active=False, now_ms=now + RAMP_TICK_MS)
        self.assertEqual(out, 0)
        self.assertEqual(r.brake_until_ms, 0)

    def test_mixed_mode_skip_release_does_not_set_reversal_flag(self):
        # If a sibling axis is active when this axis releases, we take
        # the mixed-mode skip path, NOT the reversal path.
        r = AxisRamp()
        now = _drive_until_steady(r, 127, is_arm=False)
        out = step_axis_ramp(r, 0, is_arm=False,
                             other_active=True, now_ms=now)
        self.assertEqual(out, 0)
        self.assertFalse(r.ramping)
        self.assertFalse(r.reversal_pending)
        self.assertEqual(r.brake_until_ms, 0)


class BR_E_ArmReversalTests(unittest.TestCase):
    """Arm axes use the arm ladder for the decay phase."""

    def test_arm_decay_uses_arm_ladder(self):
        r = AxisRamp()
        now = _drive_until_steady(r, 127, is_arm=True)
        step_axis_ramp(r, -127, is_arm=True,
                       other_active=False, now_ms=now)
        self.assertEqual(r.duration_ms,
                         ramp_duration_ms(127, is_arm=True))
        self.assertTrue(r.reversal_pending)

    def test_arm_settle_window_matches_track_settle(self):
        # The settle window is a single shared constant — arm reversals
        # use the same window as track reversals.
        r = AxisRamp()
        now = _drive_until_steady(r, 127, is_arm=True)
        decay_ms = ramp_duration_ms(127, is_arm=True)
        step_axis_ramp(r, -127, is_arm=True,
                       other_active=False, now_ms=now)
        deadline = now + decay_ms
        step_axis_ramp(r, -127, is_arm=True,
                       other_active=False, now_ms=deadline)
        self.assertEqual(r.brake_until_ms,
                         deadline + REVERSAL_BRAKE_MS)


# ---------------------------------------------------------------------------
# BR-F: source tripwires.
# ---------------------------------------------------------------------------

_FIRMWARE_PATH = (pathlib.Path(__file__).resolve().parents[3]
                  / "DESIGN-CONTROLLER" / "firmware"
                  / "tractor_h7" / "tractor_h7.ino")


class BR_F_SourceTripwireTests(unittest.TestCase):
    """Pin firmware markers so a refactor cannot silently drop BC-22."""

    @classmethod
    def setUpClass(cls) -> None:
        cls.src = _FIRMWARE_PATH.read_text(encoding="utf-8")

    def test_bc22_marker_present(self):
        self.assertIn("BC-22", self.src,
                      "firmware must carry a BC-22 marker comment")

    def test_reversal_brake_constant_present(self):
        self.assertIn("REVERSAL_BRAKE_MS", self.src,
                      "REVERSAL_BRAKE_MS constant must be defined")
        # Pin the value to avoid accidental drift from the design doc.
        self.assertIn("REVERSAL_BRAKE_MS = 100", self.src)

    def test_axisramp_carries_brake_state_fields(self):
        # Both BC-22 fields must be declared in struct AxisRamp.
        self.assertIn("brake_until_ms", self.src)
        self.assertIn("reversal_pending", self.src)

    def test_step_axis_ramp_clears_brake_when_elapsed(self):
        # Tripwire on the brake-clear assignment.
        self.assertIn("r.brake_until_ms = 0", self.src,
                      "brake-clear path must be reachable in step_axis_ramp")

    def test_step_axis_ramp_arms_brake_after_decay(self):
        # Tripwire that the brake-arm path exists post-decay.
        self.assertIn("r.brake_until_ms", self.src)
        self.assertIn("REVERSAL_BRAKE_MS", self.src)
        # Pin the actual arming line shape so a refactor that drops
        # the +REVERSAL_BRAKE_MS calc fails.
        self.assertIn("now + REVERSAL_BRAKE_MS", self.src)

    def test_python_mirror_exports_constant(self):
        # Sanity: the SIL fixture exposes the same constant name so
        # downstream tests don't have to grep firmware.
        from test_axis_ramp_sil import REVERSAL_BRAKE_MS as exported
        self.assertEqual(exported, 100)
        self.assertEqual(exported, REVERSAL_BRAKE_MS)


if __name__ == "__main__":
    unittest.main()
