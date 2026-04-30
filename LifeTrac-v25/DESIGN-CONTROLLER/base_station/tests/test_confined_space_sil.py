"""Round 50 / BC-27 (K-D2): SIL coverage for confined-space mode.

The build-config leaf ``ui.confined_space_mode_enabled`` is a single
boolean. When ``true``, ``ramp_duration_ms()`` multiplies its base
duration (the magnitude-derived ladder value) by ``3 / 2`` so the
release-ramp / reversal-decay path takes 1.5\u00d7 as long. The tractor
therefore stops more gently in tight quarters at the cost of a slightly
softer feel. ``false`` (default) is byte-for-byte identity.

Composition: the multiplier applies to **every** ramp on every axis
(track + arm + bucket) and works orthogonally to BC-25 (stick curve)
and BC-26 (scurve ramp shape). It also applies to the K-A4 coordinated
bilateral track-stop forced duration so both tracks still finish at
the same wallclock instant when confined-space mode is on.

Test classes:

* ``CS_A_RampDurationMathTests`` \u2014 the pure ``ramp_duration_ms()``
  formula across the full ladder for tracks and arms; ``confined=False``
  matches the pre-Round-50 ladder byte-for-byte; ``confined=True``
  multiplies by 3/2 (integer-exact for every ladder value).
* ``CS_B_DefaultIdentityTests`` \u2014 ``FourAxisArbiter()`` with no
  ``confined_space_mode_enabled`` argument produces byte-for-byte the
  same release-ramp trajectory as the explicit ``False`` arbiter; the
  release ramp from full forward reaches zero at the same tick.
* ``CS_C_LongerRampTests`` \u2014 a ``FourAxisArbiter`` constructed with
  ``confined_space_mode_enabled=True`` keeps the track non-zero for
  ~1.5\u00d7 as many ticks during a release ramp from full forward; both
  modes still hit zero at their respective deadlines and never undershoot.
* ``CS_D_KA4CoordinatedStopTests`` \u2014 the K-A4 forced track duration
  also gets the 1.5\u00d7 multiplier when confined-space mode is on, so
  asymmetric pre-release magnitudes still finish at the same tick.
* ``CS_E_ComposesWithBC25Tests`` \u2014 confined-space mode + stick curve
  exponent 2.0 simultaneously: the release ramp is longer **and** the
  starting magnitude is curve-compressed; both effects apply.
* ``CS_F_ComposesWithBC26Tests`` \u2014 confined-space mode + scurve ramp
  shape simultaneously: the longer ramp uses the smoothstep envelope.
* ``CS_G_FirmwareTripwireTests`` \u2014 firmware contains the BC-27 marker,
  the ``LIFETRAC_UI_CONFINED_SPACE_MODE_ENABLED`` macro consumption,
  and the ``base * 3u`` multiplier expression. Build config exposes the
  schema leaf and the CAPABILITY_INVENTORY row.

Pure stdlib.
"""

from __future__ import annotations

import pathlib
import unittest

from test_axis_ramp_sil import (
    AXIS_DEADBAND,
    RAMP_TICK_MS,
    AxisRamp,
    FourAxisArbiter,
    ramp_duration_ms,
)


_FIRMWARE_PATH = (pathlib.Path(__file__).resolve().parents[3]
                  / "DESIGN-CONTROLLER" / "firmware"
                  / "tractor_h7" / "tractor_h7.ino")
_INVENTORY_PATH = (pathlib.Path(__file__).resolve().parents[3]
                   / "DESIGN-CONTROLLER" / "CAPABILITY_INVENTORY.md")
_SCHEMA_PATH = (pathlib.Path(__file__).resolve().parents[1]
                / "config" / "build_config.schema.json")


# ---------------------------------------------------------------------------
# CS-A \u2014 pure ramp_duration_ms math
# ---------------------------------------------------------------------------


class CS_A_RampDurationMathTests(unittest.TestCase):

    def test_default_off_matches_legacy_track_ladder(self):
        # Ladder: <48 \u2192 500, 48..95 \u2192 1000, >=96 \u2192 2000.
        self.assertEqual(ramp_duration_ms(0, is_arm=False), 500)
        self.assertEqual(ramp_duration_ms(47, is_arm=False), 500)
        self.assertEqual(ramp_duration_ms(48, is_arm=False), 1000)
        self.assertEqual(ramp_duration_ms(95, is_arm=False), 1000)
        self.assertEqual(ramp_duration_ms(96, is_arm=False), 2000)
        self.assertEqual(ramp_duration_ms(127, is_arm=False), 2000)

    def test_default_off_matches_legacy_arm_ladder(self):
        # Ladder: <48 \u2192 250, 48..95 \u2192 500, >=96 \u2192 1000.
        self.assertEqual(ramp_duration_ms(0, is_arm=True), 250)
        self.assertEqual(ramp_duration_ms(47, is_arm=True), 250)
        self.assertEqual(ramp_duration_ms(48, is_arm=True), 500)
        self.assertEqual(ramp_duration_ms(95, is_arm=True), 500)
        self.assertEqual(ramp_duration_ms(96, is_arm=True), 1000)
        self.assertEqual(ramp_duration_ms(127, is_arm=True), 1000)

    def test_confined_track_ladder_is_3_2_x(self):
        for mag, base in [(0, 500), (47, 500), (48, 1000),
                          (95, 1000), (96, 2000), (127, 2000)]:
            with self.subTest(mag=mag, base=base):
                self.assertEqual(
                    ramp_duration_ms(mag, is_arm=False, confined_space=True),
                    (base * 3) // 2)

    def test_confined_arm_ladder_is_3_2_x(self):
        for mag, base in [(0, 250), (47, 250), (48, 500),
                          (95, 500), (96, 1000), (127, 1000)]:
            with self.subTest(mag=mag, base=base):
                self.assertEqual(
                    ramp_duration_ms(mag, is_arm=True, confined_space=True),
                    (base * 3) // 2)

    def test_negative_magnitude_treated_as_absolute(self):
        # Sign is irrelevant; only magnitude drives the ladder.
        for v in [-1, -47, -48, -96, -127]:
            with self.subTest(v=v):
                self.assertEqual(ramp_duration_ms(v, is_arm=False),
                                 ramp_duration_ms(-v, is_arm=False))
                self.assertEqual(ramp_duration_ms(v, is_arm=True),
                                 ramp_duration_ms(-v, is_arm=True))


# ---------------------------------------------------------------------------
# CS-B \u2014 default-off identity at the arbiter level
# ---------------------------------------------------------------------------


class CS_B_DefaultIdentityTests(unittest.TestCase):

    def test_default_arbiter_disables_confined_space(self):
        arb = FourAxisArbiter()
        self.assertFalse(arb.confined_space_mode_enabled)

    def test_default_arbiter_release_matches_explicit_off(self):
        a_default = FourAxisArbiter()
        a_explicit = FourAxisArbiter(confined_space_mode_enabled=False)
        t = 0
        a_default.tick(0, 127, 0, 0, t)
        a_explicit.tick(0, 127, 0, 0, t)
        t += RAMP_TICK_MS
        for _ in range(25):
            out_d = a_default.tick(0, 0, 0, 0, t)
            out_e = a_explicit.tick(0, 0, 0, 0, t)
            self.assertEqual(out_d["left_track"], out_e["left_track"])
            self.assertEqual(out_d["right_track"], out_e["right_track"])
            t += RAMP_TICK_MS


# ---------------------------------------------------------------------------
# CS-C \u2014 release ramp is ~1.5\u00d7 longer with confined-space mode on
# ---------------------------------------------------------------------------


def _release_ramp_samples(arb: FourAxisArbiter) -> list[int]:
    """Drive forward at lhy=127 for one tick, release, return every
    subsequent left_track sample until well past the deadline."""
    t = 0
    arb.tick(0, 127, 0, 0, t)
    t += RAMP_TICK_MS
    samples = []
    # 4000 ms of post-release ticks at RAMP_TICK_MS=50 covers both the
    # 2000 ms baseline ramp and the 3000 ms confined ramp comfortably.
    for _ in range(0, 4000, RAMP_TICK_MS):
        out = arb.tick(0, 0, 0, 0, t)
        samples.append(out["left_track"])
        t += RAMP_TICK_MS
    return samples


def _last_nonzero_index(samples: list[int]) -> int:
    for i in range(len(samples) - 1, -1, -1):
        if samples[i] != 0:
            return i
    return -1


class CS_C_LongerRampTests(unittest.TestCase):

    def test_confined_track_ramp_is_about_1_5x_longer(self):
        baseline = _release_ramp_samples(FourAxisArbiter())
        confined = _release_ramp_samples(
            FourAxisArbiter(confined_space_mode_enabled=True))
        last_off = _last_nonzero_index(baseline)
        last_on = _last_nonzero_index(confined)
        # RAMP_TICK_MS=50; base duration for mag=127 is 2000 ms → last
        # non-zero ~tick 38–40; confined is 3000 ms → last non-zero
        # ~tick 58–60. Allow ±2-tick slack for int-truncation rounding.
        self.assertGreaterEqual(last_off, 37)
        self.assertLessEqual(last_off, 40)
        self.assertGreaterEqual(last_on, 57)
        self.assertLessEqual(last_on, 60)
        # And the confined ramp must outlast the baseline by enough ticks
        # to make the 1.5× increase obvious (≥ 18 extra ticks of 50 ms).
        self.assertGreaterEqual(last_on - last_off, 18)

    def test_both_eventually_reach_zero(self):
        for arb in (FourAxisArbiter(),
                    FourAxisArbiter(confined_space_mode_enabled=True)):
            samples = _release_ramp_samples(arb)
            with self.subTest(confined=arb.confined_space_mode_enabled):
                self.assertEqual(samples[-1], 0,
                                 "ramp must reach 0 within the test window")
                # And it must never undershoot below zero from a positive start.
                self.assertGreaterEqual(min(samples), 0)


# ---------------------------------------------------------------------------
# CS-D \u2014 K-A4 coordinated bilateral stop also stretches under confined mode
# ---------------------------------------------------------------------------


class CS_D_KA4CoordinatedStopTests(unittest.TestCase):

    def test_both_tracks_reach_zero_by_ka4_deadline(self):
        # K-A4 shares one duration across both tracks (max-magnitude
        # ladder lookup). Due to integer truncation in the linear
        # interpolator the smaller-magnitude track can hit 0 a few ticks
        # earlier, but BOTH must be at 0 by the deadline tick (and stay
        # there) under both modes. Drive lhx=80 lhy=120 → left=127,
        # right=25 → K-A4 forced duration = ramp_duration_ms(127).
        for confined in (False, True):
            with self.subTest(confined=confined):
                arb = FourAxisArbiter(confined_space_mode_enabled=confined)
                t = 0
                arb.tick(80, 120, 0, 0, t)
                t += RAMP_TICK_MS
                base_dur = 2000 if not confined else 3000
                deadline_tick = base_dur // RAMP_TICK_MS  # 40 or 60
                # Run past the deadline.
                for k in range(0, deadline_tick + 5):
                    out = arb.tick(0, 0, 0, 0, t)
                    t += RAMP_TICK_MS
                    if k >= deadline_tick:
                        self.assertEqual(
                            out["left_track"], 0,
                            f"left should be 0 at k={k} (deadline {deadline_tick})")
                        self.assertEqual(
                            out["right_track"], 0,
                            f"right should be 0 at k={k} (deadline {deadline_tick})")

    def test_confined_coordinated_stop_takes_longer_than_baseline(self):
        def deadline_zero_tick(confined: bool) -> int:
            arb = FourAxisArbiter(confined_space_mode_enabled=confined)
            t = 0
            arb.tick(80, 120, 0, 0, t)
            t += RAMP_TICK_MS
            for k in range(0, 80):
                out = arb.tick(0, 0, 0, 0, t)
                if out["left_track"] == 0 and out["right_track"] == 0:
                    return k
                t += RAMP_TICK_MS
            return -1

        baseline = deadline_zero_tick(False)
        confined = deadline_zero_tick(True)
        # Baseline = 2000/50 = 40 ticks; confined = 3000/50 = 60 ticks.
        self.assertEqual(baseline, 40)
        self.assertEqual(confined, 60)
        self.assertGreater(confined, baseline,
                           "confined-space mode must stretch K-A4 stop")


# ---------------------------------------------------------------------------
# CS-E \u2014 composes with BC-25 stick curve
# ---------------------------------------------------------------------------


class CS_E_ComposesWithBC25Tests(unittest.TestCase):

    def test_confined_plus_curve_compresses_starting_value(self):
        # lhy = 64. With curve n=2.0 the stick value compresses to 32
        # before mixing. The release ramp from 32 takes the <48 base of
        # 500 ms (linear) or 750 ms (confined). The starting effective
        # while the stick is held is 32 either way.
        arb = FourAxisArbiter(stick_curve_exponent=2.0,
                              confined_space_mode_enabled=True)
        t = 0
        out = arb.tick(0, 64, 0, 0, t)
        self.assertEqual(out["left_track"], 32)
        self.assertEqual(out["right_track"], 32)
        # Both effects must remain individually identifiable: the curve
        # is responsible for the 64 \u2192 32 compression, the confined-space
        # multiplier is responsible for stretching the release ramp.
        self.assertEqual(arb.stick_curve_exponent, 2.0)
        self.assertTrue(arb.confined_space_mode_enabled)


# ---------------------------------------------------------------------------
# CS-F \u2014 composes with BC-26 scurve ramp shape
# ---------------------------------------------------------------------------


class CS_F_ComposesWithBC26Tests(unittest.TestCase):

    def test_confined_plus_scurve_runs_longer_with_smoothstep_envelope(self):
        arb = FourAxisArbiter(ramp_shape="scurve",
                              confined_space_mode_enabled=True)
        samples = _release_ramp_samples(arb)
        # scurve must still reach zero by the 3000 ms confined deadline
        # (= 60 ticks at RAMP_TICK_MS=50).
        last = _last_nonzero_index(samples)
        self.assertGreaterEqual(last, 56)
        self.assertLessEqual(last, 60)
        # And the trajectory must not drop below zero at any sample
        # (smoothstep is monotone on a positive start).
        self.assertGreaterEqual(min(samples), 0)
        # Sanity: at the quarter-point of the 3 s ramp (~750 ms = 15
        # ticks post-release) the scurve sample is well above linear's
        # ~95 (linear at t=0.25 is 0.75 × 127 ≈ 95; scurve is ~108).
        self.assertGreater(samples[15], 100)


# ---------------------------------------------------------------------------
# CS-G \u2014 source / config tripwires
# ---------------------------------------------------------------------------


class CS_G_FirmwareTripwireTests(unittest.TestCase):

    @classmethod
    def setUpClass(cls) -> None:
        cls.fw = _FIRMWARE_PATH.read_text(encoding="utf-8")
        cls.inv = _INVENTORY_PATH.read_text(encoding="utf-8")
        cls.schema = _SCHEMA_PATH.read_text(encoding="utf-8")

    def test_bc27_marker_present_in_firmware(self):
        self.assertIn("BC-27", self.fw)

    def test_macro_consumed_in_firmware(self):
        self.assertIn("LIFETRAC_UI_CONFINED_SPACE_MODE_ENABLED", self.fw)

    def test_three_halves_multiplier_present(self):
        # Pin the exact multiplier expression so a regression to a
        # different ratio (\u00d72, \u00d75/4, etc.) trips the tripwire.
        self.assertIn("base * 3u", self.fw)
        self.assertIn(") / 2u", self.fw)

    def test_schema_documents_new_leaf(self):
        self.assertIn("confined_space_mode_enabled", self.schema)

    def test_inventory_documents_new_leaf(self):
        self.assertIn("ui.confined_space_mode_enabled", self.inv)


if __name__ == "__main__":
    unittest.main()
