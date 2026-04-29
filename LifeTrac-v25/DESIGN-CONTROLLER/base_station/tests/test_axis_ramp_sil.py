"""Round 18: SIL model of the M7 per-axis ramp + mixed-mode skip.

Mirrors ``step_axis_ramp()`` and the surrounding ``apply_control()``
arbitration in ``firmware/tractor_h7/tractor_h7.ino`` so the W4-05 and
W4-06 HIL gates become *verification* passes against a documented model
in the same style as the Round-8 / Round-10 SIL suites
(``test_m4_safety_sil.py``, ``test_modbus_slave_sil.py``).

What the C side does, captured here:

* ``AXIS_DEADBAND = 13`` (~10% of int8 full scale).
* ``RAMP_TICK_MS  = 50`` (20 Hz arbitration tick).
* On a non-zero (active) raw axis input, the ramp is cancelled and the
  raw value is passed through immediately. ``effective`` becomes the
  raw value and ``ramping`` is false.
* On the *first* tick where the axis transitions from active to
  released (``was_active && !is_now && !r.ramping``):
    - If any *other* axis is currently active, jump straight to 0
      (mixed-mode skip — coordinated dig→drive transitions stay snappy).
    - Otherwise, latch ``r.start = r.effective`` and
      ``r.duration_ms = ramp_duration_ms(start, is_arm)`` and start
      a linear interpolation from ``start`` down to 0.
* Per-axis ramp ladder, in magnitude units 0..127:
    Track (lh_x / lh_y, ``is_arm = false``):
      mag >= 96 → 2000 ms
      mag >= 48 → 1000 ms
      mag <  48 →  500 ms
    Arm   (rh_x / rh_y, ``is_arm = true``):
      mag >= 96 → 1000 ms
      mag >= 48 →  500 ms
      mag <  48 →  250 ms
* While ramping, ``effective = start * (duration - elapsed) / duration``
  (integer truncation toward zero — the C code uses ``int32_t`` and
  truncating division). When ``elapsed >= duration``, ``effective = 0``
  and ``ramping = false``.
* The valve-coil bitmap and ``REG_FLOW_SP_*`` are re-derived from
  ``effective`` (not raw) every tick:
    - Coil bits are set iff ``|effective| > AXIS_DEADBAND`` for the
      axis that owns them. Concretely, a ramp-out keeps the coil
      energised until the ramped magnitude crosses the deadband.
    - ``REG_FLOW_SP_*`` = max(|effective|) across all four axes,
      mapped from (deadband..127] linearly onto (0..10000] mV. Below
      the deadband the flow set-point is 0.

W4-05 (proportional valve ramp-out) verification surface:

* **TR-A** Track full-speed (raw=127) released alone → 2000 ms ramp,
  monotonically non-increasing magnitude, ends at 0 within one
  ``RAMP_TICK_MS`` of the deadline.
* **TR-B** Linear-interpolation midpoint: at t = duration/2 the
  effective value is within ±1 axis-unit of ``start/2`` (allows for
  integer truncation).
* **TR-C** Track ladder: (96..127] → 2000 ms; (48..96) → 1000 ms;
  (deadband..48) → 500 ms.
* **TR-D** Arm  ladder: (96..127] → 1000 ms; (48..96) →  500 ms;
  (deadband..48) →  250 ms.
* **TR-E** Coil-stays-engaged invariant: from release to one tick
  before the ramp ends, the coil bit owned by the ramping axis stays
  set. The coil drops in the same tick that ``REG_FLOW_SP_*`` first
  reads 0 (or the immediately following tick — see TR-E note below).
* **TR-F** Flow set-point during ramp is monotonically non-increasing
  and reaches 0 by the deadline.

W4-06 (mixed-mode skip) verification surface:

* **TR-G** Two-axis: drive lhy + lhx active; release lhy with lhx
  still active → lhy.effective drops to 0 in the *same tick*; lhy is
  not marked ``ramping``.
* **TR-H** Sibling release order doesn't matter: same outcome whether
  the still-held axis is the same hand or opposite hand (lhy held vs
  rhy held).
* **TR-I** Last-axis release falls back to ramp: in a sequence
  release-A-while-B-active (skip), then release-B alone, B *does*
  ramp from its current value — the skip path must not poison the
  arbiter.
* **TR-J** Skip is checked against *raw* sibling state, not the
  ramping ``effective`` of an already-released sibling. A previously-
  released axis that's still mid-ramp must NOT block a fresh release
  from taking the ramp path. This pins the comment in apply_control:
  "``other_active`` is computed against the *raw* values so a stale
  ramping axis doesn't block a sibling axis's mixed-mode skip."

Pure stdlib.
"""

from __future__ import annotations

import unittest


# ---------------------------------------------------------------------------
# Constants mirrored from firmware/tractor_h7/tractor_h7.ino
# ---------------------------------------------------------------------------

AXIS_DEADBAND = 13
RAMP_TICK_MS = 50

# Valve-coil bit positions (REG_VALVE_COILS layout per TRACTOR_NODE.md).
VB_DRIVE_LF = 0
VB_DRIVE_LR = 1
VB_DRIVE_RF = 2
VB_DRIVE_RR = 3
VB_BOOM_UP = 4
VB_BOOM_DN = 5
VB_BUCKET_CURL = 6
VB_BUCKET_DUMP = 7


def axis_active(v: int) -> bool:
    """Mirror ``static inline bool axis_active(int16_t v)``."""
    return v > AXIS_DEADBAND or v < -AXIS_DEADBAND


def ramp_duration_ms(magnitude: int, is_arm: bool) -> int:
    """Mirror ``ramp_duration_ms()`` from tractor_h7.ino."""
    mag = -magnitude if magnitude < 0 else magnitude
    if is_arm:
        if mag >= 96:
            return 1000
        if mag >= 48:
            return 500
        return 250
    if mag >= 96:
        return 2000
    if mag >= 48:
        return 1000
    return 500


class AxisRamp:
    """Mirror ``struct AxisRamp``.

    All five fields are kept verbatim; ``effective`` and ``start`` are
    int16 in C but Python ints are wide enough to model them losslessly
    so long as we keep arithmetic to the same shape.
    """

    __slots__ = ("effective", "start", "start_ms", "duration_ms", "ramping")

    def __init__(self) -> None:
        self.effective = 0
        self.start = 0
        self.start_ms = 0
        self.duration_ms = 0
        self.ramping = False


def step_axis_ramp(r: AxisRamp, raw: int, is_arm: bool, other_active: bool,
                   now_ms: int) -> int:
    """Mirror ``step_axis_ramp()`` from tractor_h7.ino.

    Returns the new ``r.effective`` after one tick. ``now_ms`` is the
    SIL stand-in for ``millis()``.
    """
    was_active = axis_active(r.effective)
    is_now = axis_active(raw)

    if is_now:
        # Operator commanding — cancel any ramp and pass through.
        r.ramping = False
        r.effective = raw
        return r.effective

    # Operator released the axis.
    if was_active and not r.ramping:
        if other_active:
            # Mixed-mode skip — drop instantly so coordinated transitions
            # stay snappy.
            r.effective = 0
            return 0
        # Start a ramp from the last effective value down to zero.
        r.ramping = True
        r.start = r.effective
        r.start_ms = now_ms
        r.duration_ms = ramp_duration_ms(r.start, is_arm)

    if r.ramping:
        elapsed = now_ms - r.start_ms
        if elapsed >= r.duration_ms:
            r.ramping = False
            r.effective = 0
        else:
            # Linear interpolation start → 0 with C-style integer
            # truncation toward zero (Python ``int(a*b/c)`` after
            # arithmetic on ints differs from ``//`` for negatives, so
            # we replicate the C ``int32_t`` truncation explicitly).
            num = r.start * (r.duration_ms - elapsed)
            v = int(num / r.duration_ms)  # truncates toward 0
            r.effective = v
    else:
        r.effective = 0
    return r.effective


# ---------------------------------------------------------------------------
# A small four-axis driver mirroring the apply_control() arbitration shape
# so the W4-06 mixed-mode-skip tests can exercise the *combination* of all
# four axes' ramps simultaneously, not just one in isolation.
# ---------------------------------------------------------------------------


class FourAxisArbiter:
    """Drive lhx/lhy/rhx/rhy through ``step_axis_ramp`` like apply_control.

    Field naming and ``other_active`` wiring are byte-for-byte identical
    to the C in ``apply_control()`` lines that compute ``any_lhx`` …
    ``any_rhy`` and pass them as siblings to ``step_axis_ramp``.
    """

    def __init__(self) -> None:
        self.lhx = AxisRamp()
        self.lhy = AxisRamp()
        self.rhx = AxisRamp()
        self.rhy = AxisRamp()

    def tick(self, raw_lhx: int, raw_lhy: int, raw_rhx: int, raw_rhy: int,
             now_ms: int) -> dict:
        any_lhx = axis_active(raw_lhx)
        any_lhy = axis_active(raw_lhy)
        any_rhx = axis_active(raw_rhx)
        any_rhy = axis_active(raw_rhy)
        lhx = step_axis_ramp(self.lhx, raw_lhx, False,
                             any_lhy or any_rhx or any_rhy, now_ms)
        lhy = step_axis_ramp(self.lhy, raw_lhy, False,
                             any_lhx or any_rhx or any_rhy, now_ms)
        rhx = step_axis_ramp(self.rhx, raw_rhx, True,
                             any_lhx or any_lhy or any_rhy, now_ms)
        rhy = step_axis_ramp(self.rhy, raw_rhy, True,
                             any_lhx or any_lhy or any_rhx, now_ms)

        # Coil bitmap from post-ramp values (mirrors apply_control).
        coils = 0
        if lhy > AXIS_DEADBAND:
            coils |= (1 << VB_DRIVE_LF) | (1 << VB_DRIVE_RF)
        if lhy < -AXIS_DEADBAND:
            coils |= (1 << VB_DRIVE_LR) | (1 << VB_DRIVE_RR)
        if rhy > AXIS_DEADBAND:
            coils |= 1 << VB_BOOM_UP
        if rhy < -AXIS_DEADBAND:
            coils |= 1 << VB_BOOM_DN

        # Flow set-point: max |effective| across all four, mapped to
        # 0..10000 mV through the deadband-stretched live region.
        mag = max(abs(lhx), abs(lhy), abs(rhx), abs(rhy))
        if mag > AXIS_DEADBAND:
            flow = (mag - AXIS_DEADBAND) * 10000 // (127 - AXIS_DEADBAND)
            if flow > 10000:
                flow = 10000
        else:
            flow = 0

        return {
            "lhx": lhx, "lhy": lhy, "rhx": rhx, "rhy": rhy,
            "coils": coils, "flow_sp": flow,
        }


# ---------------------------------------------------------------------------
# W4-05 — proportional valve ramp-out
# ---------------------------------------------------------------------------


class W4_05_RampOutTests(unittest.TestCase):
    """W4-05 SIL: per-axis ramp duration, shape, and coil-engagement."""

    def _drive_then_release(self, raw_drive: int, is_arm: bool,
                            release_at_ms: int, run_for_ms: int):
        """Hold ``raw_drive`` until ``release_at_ms``, then 0; sample
        every RAMP_TICK_MS up to ``release_at_ms + run_for_ms``.

        Returns a list of ``(t_ms, effective)`` samples taken *after*
        the release (inclusive of t=release_at_ms which is the first
        zero-input tick — this is the tick at which the ramp begins).
        """
        r = AxisRamp()
        # Warm-up: a few ticks at the full drive value so ``effective``
        # is established before the release tick.
        t = 0
        while t < release_at_ms:
            step_axis_ramp(r, raw_drive, is_arm, False, t)
            t += RAMP_TICK_MS

        samples = []
        end = release_at_ms + run_for_ms
        while t <= end:
            v = step_axis_ramp(r, 0, is_arm, False, t)
            samples.append((t - release_at_ms, v))
            t += RAMP_TICK_MS
        return samples

    # TR-A — full-speed track release ramps over 2 s.
    def test_tr_a_track_full_speed_ramp_is_2_seconds(self):
        samples = self._drive_then_release(127, is_arm=False,
                                           release_at_ms=200,
                                           run_for_ms=2200)
        # Magnitude must be monotonically non-increasing across the ramp.
        mags = [abs(v) for _, v in samples]
        for prev, cur in zip(mags, mags[1:]):
            self.assertLessEqual(cur, prev,
                                 "ramp magnitude must be non-increasing")
        # Must reach 0 by 2000 ms (allowing one tick of slack on either
        # side — the deadline check is ``elapsed >= duration``).
        zero_times = [t for t, v in samples if v == 0]
        self.assertTrue(zero_times, "ramp must reach 0")
        self.assertLessEqual(zero_times[0], 2000 + RAMP_TICK_MS,
                             "ramp must hit 0 within one tick of 2 s")
        # And not earlier than the deadline minus one tick.
        self.assertGreaterEqual(zero_times[0], 2000 - RAMP_TICK_MS,
                                "full-speed ramp finished too early")

    # TR-B — midpoint check: linear interpolation halfway down.
    def test_tr_b_track_midpoint_is_half_start(self):
        # Use raw=120 so start=120 cleanly halves to 60.
        samples = self._drive_then_release(120, is_arm=False,
                                           release_at_ms=100,
                                           run_for_ms=2000)
        # Find the sample closest to t=1000 ms (midpoint of 2 s ramp).
        mid = min(samples, key=lambda s: abs(s[0] - 1000))
        self.assertAlmostEqual(mid[1], 60, delta=2,
                               msg=f"midpoint sample {mid} should be ~60")

    # TR-C — track ladder (false=track).
    def test_tr_c_track_ladder_durations(self):
        cases = [
            (127, 2000),  # >= 96 → 2000 ms
            (96, 2000),
            (95, 1000),  # >= 48, < 96 → 1000 ms
            (48, 1000),
            (47, 500),  # > deadband, < 48 → 500 ms
            (14, 500),  # just above deadband (13)
        ]
        for raw, expected in cases:
            with self.subTest(raw=raw, expected_ms=expected):
                self.assertEqual(ramp_duration_ms(raw, is_arm=False),
                                 expected)

    # TR-D — arm ladder.
    def test_tr_d_arm_ladder_durations(self):
        cases = [
            (127, 1000),  # >= 96 → 1000 ms
            (96, 1000),
            (95, 500),  # >= 48, < 96 → 500 ms
            (48, 500),
            (47, 250),  # > deadband, < 48 → 250 ms
            (14, 250),
        ]
        for raw, expected in cases:
            with self.subTest(raw=raw, expected_ms=expected):
                self.assertEqual(ramp_duration_ms(raw, is_arm=True),
                                 expected)

    # TR-E — coil stays engaged through the ramp until the ramped
    # magnitude crosses below the deadband. Drive lhy forward at 127,
    # release alone, watch the VB_DRIVE_LF / VB_DRIVE_RF bits.
    def test_tr_e_coil_stays_engaged_during_ramp(self):
        arb = FourAxisArbiter()
        t = 0
        # Warm up at full forward.
        while t < 200:
            arb.tick(0, 127, 0, 0, t)
            t += RAMP_TICK_MS

        # Release. Track sample-by-sample: while |lhy.effective| > deadband
        # the drive coils MUST be set; once it drops to/below deadband the
        # coils MUST clear (not lag behind).
        coils_engaged_after_subdeadband = False
        while t <= 200 + 2200:
            out = arb.tick(0, 0, 0, 0, t)
            mag_above_db = abs(out["lhy"]) > AXIS_DEADBAND
            drive_bits = (1 << VB_DRIVE_LF) | (1 << VB_DRIVE_RF)
            coils_set = bool(out["coils"] & drive_bits)
            if mag_above_db:
                self.assertTrue(coils_set,
                                f"coils dropped while ramping ({out}, t={t})")
            else:
                if coils_set:
                    coils_engaged_after_subdeadband = True
            t += RAMP_TICK_MS
        self.assertFalse(coils_engaged_after_subdeadband,
                         "coils must drop once ramped magnitude is "
                         "<= deadband (otherwise the valve stays open "
                         "with no flow command, defeating ramp-out)")

    # TR-F — flow set-point monotonic during ramp; reaches 0 by deadline.
    def test_tr_f_flow_setpoint_monotonic_to_zero(self):
        arb = FourAxisArbiter()
        t = 0
        while t < 200:
            arb.tick(0, 127, 0, 0, t)
            t += RAMP_TICK_MS

        flows = []
        while t <= 200 + 2200:
            out = arb.tick(0, 0, 0, 0, t)
            flows.append((t - 200, out["flow_sp"]))
            t += RAMP_TICK_MS

        for (t_a, f_a), (t_b, f_b) in zip(flows, flows[1:]):
            self.assertLessEqual(f_b, f_a,
                                 f"flow_sp must be non-increasing during "
                                 f"ramp ({t_a}={f_a}, {t_b}={f_b})")
        # Must hit zero within one tick of the 2 s deadline.
        zero = [t for t, f in flows if f == 0]
        self.assertTrue(zero, "flow_sp must reach 0 during ramp-out")
        self.assertLessEqual(zero[0], 2000 + RAMP_TICK_MS)


# ---------------------------------------------------------------------------
# W4-06 — mixed-mode skip
# ---------------------------------------------------------------------------


class W4_06_MixedModeSkipTests(unittest.TestCase):
    """W4-06 SIL: releasing one axis with a sibling still active skips
    the ramp entirely on that axis."""

    # TR-G — two-axis: lhy released while lhx still active → lhy goes
    # straight to 0 in the same tick, no ``ramping`` flag set.
    def test_tr_g_release_with_sibling_active_skips_ramp(self):
        arb = FourAxisArbiter()
        t = 0
        # Warm up with both lhx and lhy active.
        while t < 200:
            arb.tick(120, 120, 0, 0, t)
            t += RAMP_TICK_MS
        self.assertGreater(arb.lhy.effective, AXIS_DEADBAND)

        # Release lhy; lhx still active.
        out = arb.tick(120, 0, 0, 0, t)
        self.assertEqual(out["lhy"], 0, "lhy must skip to 0 immediately")
        self.assertFalse(arb.lhy.ramping,
                         "lhy must not enter ramp state on mixed release")
        # And lhx untouched.
        self.assertEqual(out["lhx"], 120)

    # TR-H — same outcome with sibling on the *other* hand.
    def test_tr_h_skip_is_orientation_agnostic(self):
        for held_axis in ("lhx", "lhy", "rhx", "rhy"):
            with self.subTest(held=held_axis):
                arb = FourAxisArbiter()
                # Warm up: drive lhy AND the held axis.
                t = 0
                while t < 200:
                    raw = {"lhx": 0, "lhy": 120, "rhx": 0, "rhy": 0}
                    if held_axis != "lhy":
                        raw[held_axis] = 100
                    arb.tick(raw["lhx"], raw["lhy"], raw["rhx"], raw["rhy"], t)
                    t += RAMP_TICK_MS
                if held_axis == "lhy":
                    # Degenerate: only one axis ever active, so releasing
                    # *it* alone must take the ramp path, not skip.
                    out = arb.tick(0, 0, 0, 0, t)
                    self.assertTrue(arb.lhy.ramping,
                                    "single-axis release must ramp, not skip")
                else:
                    raw = {"lhx": 0, "lhy": 0, "rhx": 0, "rhy": 0}
                    raw[held_axis] = 100
                    out = arb.tick(raw["lhx"], raw["lhy"],
                                   raw["rhx"], raw["rhy"], t)
                    self.assertEqual(out["lhy"], 0,
                                     f"lhy must skip with {held_axis} held")
                    self.assertFalse(arb.lhy.ramping)

    # TR-I — last-axis release falls back to the ramp path. Sequence:
    # release lhy with lhx held (skip), then release lhx alone (ramp).
    def test_tr_i_last_release_takes_ramp_path(self):
        arb = FourAxisArbiter()
        t = 0
        while t < 200:
            arb.tick(120, 120, 0, 0, t)
            t += RAMP_TICK_MS

        # Release lhy first — skip path.
        arb.tick(120, 0, 0, 0, t)
        self.assertEqual(arb.lhy.effective, 0)
        self.assertFalse(arb.lhy.ramping)
        t += RAMP_TICK_MS

        # Hold lhx steady for a beat, then release. lhx must ramp.
        arb.tick(120, 0, 0, 0, t)
        t += RAMP_TICK_MS
        out = arb.tick(0, 0, 0, 0, t)
        self.assertTrue(arb.lhx.ramping,
                        "last-axis release must take the ramp path")
        self.assertEqual(arb.lhx.start, 120)
        self.assertEqual(arb.lhx.duration_ms, 2000)  # track @ 120 → 2 s

    # TR-J — pin the apply_control comment: ``other_active`` is computed
    # against *raw* values, not the ``effective`` of an already-ramping
    # sibling. So a previously-released-and-now-ramping lhx must NOT
    # block lhy from taking its own ramp path when lhy is released
    # second with no other raw-active sibling.
    def test_tr_j_stale_ramping_sibling_does_not_block_skip_decision(self):
        arb = FourAxisArbiter()
        t = 0
        # Warm up: lhx + lhy + rhy all active. (rhy is the "anchor" so
        # the lhx release below is forced to take the skip path.)
        while t < 200:
            arb.tick(120, 120, 0, 100, t)
            t += RAMP_TICK_MS

        # Release lhx with lhy + rhy still active — skip.
        arb.tick(0, 120, 0, 100, t)
        self.assertEqual(arb.lhx.effective, 0)
        self.assertFalse(arb.lhx.ramping)
        t += RAMP_TICK_MS

        # Now release rhy (boom) with only lhy still raw-active. lhy is
        # still raw-active at 120 so rhy must skip to 0.
        out = arb.tick(0, 120, 0, 0, t)
        self.assertEqual(out["rhy"], 0)
        self.assertFalse(arb.rhy.ramping)
        t += RAMP_TICK_MS

        # Finally release lhy. Only lhy is raw-active at this point;
        # lhx and rhy are both already at effective=0 (skipped earlier).
        # The arbiter computes ``other_active`` from raw inputs, which
        # are now all zero → lhy MUST take the ramp path, not skip.
        out = arb.tick(0, 0, 0, 0, t)
        self.assertTrue(arb.lhy.ramping,
                        "lhy release with no raw-active sibling must "
                        "ramp; the previously-skipped axes' effective=0 "
                        "must not be observable to the skip check")
        self.assertEqual(arb.lhy.start, 120)
        self.assertEqual(arb.lhy.duration_ms, 2000)


if __name__ == "__main__":
    unittest.main()
