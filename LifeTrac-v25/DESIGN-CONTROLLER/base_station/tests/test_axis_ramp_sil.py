"""Round 18: SIL model of the M7 per-axis ramp + mixed-mode skip.

**Round 44 (BC-21 / K-A4) update.** Ramping now happens on **logical**
motion axes (``left_track``, ``right_track``, ``arms``, ``bucket``)
*after* differential mixing of the raw stick channels, not on the raw
``lhx``/``lhy``/``rhx``/``rhy`` channels. The ramp engine
(``step_axis_ramp``) and ladder are unchanged; only what we feed into
them changes. The W4-05 / W4-06 tests below have been re-expressed in
terms of the new logical axes — the *behavioural* invariants (ramp
shape, mixed-mode skip, coil-engagement during ramp) are identical;
only the axis names and coil-mapping shift.

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
    - If any *other* logical axis is currently active, jump straight
      to 0 (mixed-mode skip — coordinated dig→drive transitions stay
      snappy).
    - Otherwise, latch ``r.start = r.effective`` and
      ``r.duration_ms = ramp_duration_ms(start, is_arm)`` and start
      a linear interpolation from ``start`` down to 0.
* **K-A4 coordinated bilateral track stop:** if BOTH track logical
  axes transition active → released in the same tick (and neither is
  already ramping), both ramps share a single ``forced_duration_ms``
  computed from the larger starting magnitude. The ``forced`` value is
  passed through ``step_axis_ramp``'s optional ``forced_duration_ms``
  parameter and overrides the magnitude-derived ladder.
* Per-axis ramp ladder, in magnitude units 0..127:
    Track (left_track / right_track, ``is_arm = false``):
      mag >= 96 → 2000 ms
      mag >= 48 → 1000 ms
      mag <  48 →  500 ms
    Arm   (arms / bucket, ``is_arm = true``):
      mag >= 96 → 1000 ms
      mag >= 48 →  500 ms
      mag <  48 →  250 ms
* While ramping, ``effective = start * (duration - elapsed) / duration``
  (integer truncation toward zero — the C code uses ``int32_t`` and
  truncating division). When ``elapsed >= duration``, ``effective = 0``
  and ``ramping = false``.
* The valve-coil bitmap and ``REG_FLOW_SP_*`` are re-derived from
  ``effective`` (not raw) every tick:
    - **BC-21 coil mapping (logical-axis driven):** ``left_track > db``
      → ``VB_DRIVE_LF``; ``left_track < -db`` → ``VB_DRIVE_LR``;
      ``right_track > db`` → ``VB_DRIVE_RF``; ``right_track < -db``
      → ``VB_DRIVE_RR``. Pre-BC-21 only ``lhy`` drove drive coils, so a
      pure spin-turn (lhy=0, lhx=full) produced ZERO drive coil
      activation. Per-side coils fix that.
    - ``REG_FLOW_SP_*`` = max(|effective|) across all four logical
      axes, mapped from (deadband..127] linearly onto (0..10000] mV.

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
* **TR-E** Coil-stays-engaged invariant on the LEFT-TRACK logical axis:
  from release to one tick before the ramp ends, the ``VB_DRIVE_LF``
  coil bit owned by the ramping axis stays set. The coil drops in the
  same tick that the ramped magnitude crosses the deadband.
* **TR-F** Flow set-point during ramp is monotonically non-increasing
  and reaches 0 by the deadline.

W4-06 (mixed-mode skip) verification surface, expressed on logical
axes:

* **TR-G** Two logical axes active (e.g. left_track + arms via stick
  inputs lhy=120, rhy=120). Releasing arms while left_track is still
  active → arms.effective drops to 0 in the *same tick*; arms is not
  marked ``ramping``.
* **TR-H** Sibling release order doesn't matter: same outcome
  regardless of which other logical axis is the holder.
* **TR-I** Last-axis release falls back to ramp: in a sequence
  release-A-while-B-active (skip), then release-B alone, B *does*
  ramp from its current value — the skip path must not poison the
  arbiter.
* **TR-J** Skip is checked against *current-tick logical inputs*, not
  the ``effective`` of an already-ramping sibling. A previously-
  released axis that's still mid-ramp must NOT block a fresh release
  from taking the ramp path.

Pure stdlib.
"""

from __future__ import annotations

import unittest


# ---------------------------------------------------------------------------
# Constants mirrored from firmware/tractor_h7/tractor_h7.ino
# ---------------------------------------------------------------------------

AXIS_DEADBAND = 13
RAMP_TICK_MS = 50
# BC-22 (Round 45) reversal-brake settle-window length, in milliseconds.
REVERSAL_BRAKE_MS = 100

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


def ramp_duration_ms(magnitude: int, is_arm: bool,
                     confined_space: bool = False) -> int:
    """Mirror ``ramp_duration_ms()`` from tractor_h7.ino.

    BC-27 / K-D2 (Round 50): when ``confined_space`` is true the base
    duration is multiplied by 3/2 so the tractor stops more gently in
    tight quarters. Default ``False`` is byte-for-byte identity vs
    pre-Round-50.
    """
    mag = -magnitude if magnitude < 0 else magnitude
    if is_arm:
        if mag >= 96:
            base = 1000
        elif mag >= 48:
            base = 500
        else:
            base = 250
    else:
        if mag >= 96:
            base = 2000
        elif mag >= 48:
            base = 1000
        else:
            base = 500
    if confined_space:
        base = (base * 3) // 2
    return base


class AxisRamp:
    """Mirror ``struct AxisRamp``.

    All seven fields are kept verbatim; ``effective`` and ``start`` are
    int16 in C but Python ints are wide enough to model them losslessly
    so long as we keep arithmetic to the same shape. ``brake_until_ms``
    and ``reversal_pending`` are BC-22 reversal-brake state; both default
    to inert (0 / False).
    """

    __slots__ = ("effective", "start", "start_ms", "duration_ms",
                 "ramping", "reversal_pending", "brake_until_ms")

    def __init__(self) -> None:
        self.effective = 0
        self.start = 0
        self.start_ms = 0
        self.duration_ms = 0
        self.ramping = False
        self.reversal_pending = False
        self.brake_until_ms = 0


def step_axis_ramp(r: AxisRamp, raw: int, is_arm: bool, other_active: bool,
                   now_ms: int, forced_duration_ms: int = 0,
                   ramp_shape: str = "linear",
                   confined_space: bool = False) -> int:
    """Mirror ``step_axis_ramp()`` from tractor_h7.ino.

    Returns the new ``r.effective`` after one tick. ``now_ms`` is the
    SIL stand-in for ``millis()``. ``forced_duration_ms`` (BC-21 / K-A4):
    when non-zero AND a fresh ramp is starting this tick, override the
    magnitude-derived ladder duration with the supplied value.

    BC-22 reversal brake: a same-tick sign-flip on the raw input while the
    axis is still energised triggers a decay-to-zero ramp; on completion the
    axis is held at zero for ``REVERSAL_BRAKE_MS`` before the new direction
    is allowed to pass through.

    BC-26 / K-A3: ``ramp_shape`` selects the interpolation curve used
    during release / reversal-decay. ``"linear"`` (default) preserves the
    pre-Round-49 shape; ``"scurve"`` substitutes a half-cosine smoothstep.
    """
    was_active = axis_active(r.effective)
    is_now = axis_active(raw)

    # BC-22 settle phase — hold zero, ignore raw, until brake expires.
    if r.brake_until_ms != 0:
        if now_ms < r.brake_until_ms:
            r.effective = 0
            r.ramping = False
            return 0
        # Brake elapsed — clear and proceed normally.
        r.brake_until_ms = 0
        was_active = False

    # BC-22 reversal-decay shield: if a reversal ramp is mid-flight, the
    # operator's held opposite-sign input must not cancel it via the
    # snap-on-activation path — the decay+settle sequence must run to
    # completion for hydraulic safety.
    if not (r.ramping and r.reversal_pending):
        reversal = (was_active and is_now and (
            (r.effective > 0 and raw < 0) or (r.effective < 0 and raw > 0)))
        if reversal and not r.ramping:
            r.ramping = True
            r.start = r.effective
            r.start_ms = now_ms
            r.duration_ms = (forced_duration_ms
                             if forced_duration_ms != 0
                             else ramp_duration_ms(r.start, is_arm,
                                                   confined_space))
            r.reversal_pending = True
            # Fall through to interpolation.
        elif is_now:
            # Normal pass-through (no reversal).
            r.ramping = False
            r.reversal_pending = False
            r.effective = raw
            return r.effective
        elif was_active and not r.ramping:
            # Operator released the axis.
            if other_active:
                # Mixed-mode skip.
                r.effective = 0
                r.reversal_pending = False
                return 0
            # Start a release ramp.
            r.ramping = True
            r.start = r.effective
            r.start_ms = now_ms
            r.duration_ms = (forced_duration_ms
                             if forced_duration_ms != 0
                             else ramp_duration_ms(r.start, is_arm,
                                                   confined_space))
            r.reversal_pending = False

    if r.ramping:
        elapsed = now_ms - r.start_ms
        if elapsed >= r.duration_ms:
            r.ramping = False
            r.effective = 0
            if r.reversal_pending:
                # Decay complete — enter the settle window.
                r.brake_until_ms = now_ms + REVERSAL_BRAKE_MS
                r.reversal_pending = False
        else:
            r.effective = _ramp_interpolate(r.start, elapsed,
                                            r.duration_ms, ramp_shape)
    else:
        r.effective = 0
    return r.effective


# ---------------------------------------------------------------------------
# A four-axis arbiter mirroring apply_control() post-BC-21: stick inputs
# (lhx/lhy/rhx/rhy) are mixed into logical axes (left_track, right_track,
# arms, bucket) BEFORE ramping. ``other_active`` and the K-A4 coordinated
# bilateral track stop are applied to the logical axes.
# ---------------------------------------------------------------------------


def _clip_to_int8(v: int) -> int:
    """Mirror the C ``clip_to_int8(int16_t)`` saturating cast."""
    if v > 127:
        return 127
    if v < -127:
        return -127
    return v


def _ramp_interpolate(start: int, elapsed: int, duration_ms: int,
                      shape: str = "linear") -> int:
    """BC-26 / K-A3 mirror of ``ramp_interpolate()`` in
    ``firmware/tractor_h7/tractor_h7.ino``.

    Returns the int16 axis value at ``elapsed`` ms into a
    ``start → 0`` ramp of length ``duration_ms``. ``"linear"`` preserves
    the pre-Round-49 truncation-toward-zero math byte-for-byte;
    ``"scurve"`` uses ``shape(t) = 0.5 * (1 + cos(πt))`` rounded to
    nearest int with sign-aware half-up.
    """
    if duration_ms <= 0:
        return 0
    if elapsed >= duration_ms:
        return 0
    if elapsed < 0:
        elapsed = 0
    if shape == "scurve":
        import math
        t = elapsed / duration_ms
        s = 0.5 * (1.0 + math.cos(math.pi * t))
        v = start * s
        if v >  127.0:
            v =  127.0
        if v < -127.0:
            v = -127.0
        return int(v + 0.5) if v >= 0 else int(v - 0.5)
    # linear (canonical, pre-Round-49)
    num = start * (duration_ms - elapsed)
    return int(num / duration_ms)  # truncates toward 0


def _apply_stick_curve(v: int, exponent: float = 1.0) -> int:
    """BC-25 / K-A2 mirror of ``apply_stick_curve()`` in
    ``firmware/tractor_h7/tractor_h7.ino``.

    ``effective = sign(v) * 127 * (|v|/127) ** exponent`` rounded to
    nearest int8. Default exponent of 1.0 is the canonical (linear)
    pass-through so existing tests are unaffected unless they opt in.
    """
    if exponent == 1.0 or v == 0:
        return v
    sign = 1 if v > 0 else -1
    mag = -v if v < 0 else v
    if mag > 127:
        mag = 127
    u = mag / 127.0
    curved = 127.0 * (u ** exponent)
    q = int(curved + 0.5)
    if q < 0:
        q = 0
    if q > 127:
        q = 127
    return sign * q


def _mix_tracks_preserve_steering(left_intent: int,
                                  right_intent: int) -> tuple[int, int]:
    """BC-23 mirror of ``mix_tracks_preserve_steering()`` from
    ``firmware/tractor_h7/tractor_h7.ino``.

    Replaces the previous independent ``clip_to_int8`` per side. When
    either intent magnitude exceeds 127, both are scaled by ``127/max_mag``
    so the differential (steering) ratio is preserved at the cost of
    throttle authority.
    """
    lm = -left_intent if left_intent < 0 else left_intent
    rm = -right_intent if right_intent < 0 else right_intent
    mx = lm if lm > rm else rm
    if mx <= 127:
        return left_intent, right_intent
    # C-style int truncation toward zero.
    left = int(left_intent * 127 / mx)
    right = int(right_intent * 127 / mx)
    return left, right


class FourAxisArbiter:
    """Drive lhx/lhy/rhx/rhy through the BC-21 mix-then-ramp pipeline.

    Field naming and ``other_active`` wiring are byte-for-byte identical
    to the post-BC-21 C in ``apply_control()``: the four logical-axis
    ramps (``left_track``, ``right_track``, ``arms``, ``bucket``) are
    fed clipped post-mix values and the mixed-mode-skip rule operates
    on the logical-axis activity.
    """

    def __init__(self, stick_curve_exponent: float = 1.0,
                 ramp_shape: str = "linear",
                 confined_space_mode_enabled: bool = False) -> None:
        self.left_track = AxisRamp()
        self.right_track = AxisRamp()
        self.arms = AxisRamp()
        self.bucket = AxisRamp()
        # BC-25 / K-A2: per-stick response curve exponent. Mirrors
        # ``BUILD.ui.stick_curve_exponent``. Default 1.0 = linear identity.
        self.stick_curve_exponent = stick_curve_exponent
        # BC-26 / K-A3: ramp interpolation shape. Mirrors
        # ``BUILD.hydraulic.ramp_shape``. Default ``"linear"`` = identity.
        self.ramp_shape = ramp_shape
        # BC-27 / K-D2: confined-space mode. Mirrors
        # ``BUILD.ui.confined_space_mode_enabled``. Default False = identity.
        self.confined_space_mode_enabled = confined_space_mode_enabled

    def tick(self, raw_lhx: int, raw_lhy: int, raw_rhx: int, raw_rhy: int,
             now_ms: int) -> dict:
        # BC-25 / K-A2: apply per-stick response curve before mixing.
        n = self.stick_curve_exponent
        lhx = _apply_stick_curve(raw_lhx, n)
        lhy = _apply_stick_curve(raw_lhy, n)
        rhx = _apply_stick_curve(raw_rhx, n)
        rhy = _apply_stick_curve(raw_rhy, n)
        # BC-21 mixing: tracks use Y±X, arms/bucket pass-through.
        # BC-23: preserve-steering proportional scale-down on saturation.
        left_intent = lhy + lhx
        right_intent = lhy - lhx
        left_in, right_in = _mix_tracks_preserve_steering(left_intent,
                                                          right_intent)
        arms_in = rhy
        bucket_in = rhx

        any_left = axis_active(left_in)
        any_right = axis_active(right_in)
        any_arms = axis_active(arms_in)
        any_bucket = axis_active(bucket_in)

        # K-A4 coordinated bilateral track stop.
        forced_track_dur = 0
        left_was_active = axis_active(self.left_track.effective)
        right_was_active = axis_active(self.right_track.effective)
        if (not any_left and not any_right
                and not self.left_track.ramping
                and not self.right_track.ramping
                and (left_was_active or right_was_active)):
            lm = abs(self.left_track.effective)
            rm = abs(self.right_track.effective)
            forced_track_dur = ramp_duration_ms(
                max(lm, rm), is_arm=False,
                confined_space=self.confined_space_mode_enabled)

        left = step_axis_ramp(self.left_track, left_in, False,
                              any_right or any_arms or any_bucket,
                              now_ms, forced_track_dur,
                              ramp_shape=self.ramp_shape,
                              confined_space=self.confined_space_mode_enabled)
        right = step_axis_ramp(self.right_track, right_in, False,
                               any_left or any_arms or any_bucket,
                               now_ms, forced_track_dur,
                               ramp_shape=self.ramp_shape,
                               confined_space=self.confined_space_mode_enabled)
        arms = step_axis_ramp(self.arms, arms_in, True,
                              any_left or any_right or any_bucket, now_ms,
                              ramp_shape=self.ramp_shape,
                              confined_space=self.confined_space_mode_enabled)
        bucket = step_axis_ramp(self.bucket, bucket_in, True,
                                any_left or any_right or any_arms, now_ms,
                                ramp_shape=self.ramp_shape,
                                confined_space=self.confined_space_mode_enabled)

        # Coil bitmap from post-ramp logical axes (BC-21 per-side mapping).
        coils = 0
        if left > AXIS_DEADBAND:
            coils |= 1 << VB_DRIVE_LF
        if left < -AXIS_DEADBAND:
            coils |= 1 << VB_DRIVE_LR
        if right > AXIS_DEADBAND:
            coils |= 1 << VB_DRIVE_RF
        if right < -AXIS_DEADBAND:
            coils |= 1 << VB_DRIVE_RR
        if arms > AXIS_DEADBAND:
            coils |= 1 << VB_BOOM_UP
        if arms < -AXIS_DEADBAND:
            coils |= 1 << VB_BOOM_DN

        # Flow set-point: largest active magnitude across the four logical
        # axes mapped to 0..10000 mV through the deadband-stretched live
        # region.
        # BC-24 spin-turn flow boost: opposite-sign tracks contribute the
        # SUM of magnitudes (clamped to 127), not the max, since each track
        # motor draws its own flow during a spin.
        lt_mag = abs(left)
        rt_mag = abs(right)
        spin_turn = ((left > AXIS_DEADBAND and right < -AXIS_DEADBAND) or
                     (left < -AXIS_DEADBAND and right > AXIS_DEADBAND))
        if spin_turn:
            track_mag = min(127, lt_mag + rt_mag)
        else:
            track_mag = max(lt_mag, rt_mag)
        mag = max(track_mag, abs(arms), abs(bucket))
        if mag > AXIS_DEADBAND:
            flow = (mag - AXIS_DEADBAND) * 10000 // (127 - AXIS_DEADBAND)
            if flow > 10000:
                flow = 10000
        else:
            flow = 0

        return {
            "left_track": left, "right_track": right,
            "arms": arms, "bucket": bucket,
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
    # magnitude crosses below the deadband. Drive forward at lhy=127
    # (which mixes to left_track=127, right_track=127), release alone,
    # watch the VB_DRIVE_LF bit (BC-21 per-side mapping).
    def test_tr_e_coil_stays_engaged_during_ramp(self):
        arb = FourAxisArbiter()
        t = 0
        # Warm up at full forward.
        while t < 200:
            arb.tick(0, 127, 0, 0, t)
            t += RAMP_TICK_MS

        # Release. Track sample-by-sample: while |left_track.effective|
        # > deadband the VB_DRIVE_LF coil MUST be set; once it drops
        # to/below deadband the coil MUST clear (not lag behind).
        coils_engaged_after_subdeadband = False
        while t <= 200 + 2200:
            out = arb.tick(0, 0, 0, 0, t)
            mag_above_db = abs(out["left_track"]) > AXIS_DEADBAND
            drive_bit = 1 << VB_DRIVE_LF
            coil_set = bool(out["coils"] & drive_bit)
            if mag_above_db:
                self.assertTrue(coil_set,
                                f"VB_DRIVE_LF dropped while ramping ({out}, t={t})")
            else:
                if coil_set:
                    coils_engaged_after_subdeadband = True
            t += RAMP_TICK_MS
        self.assertFalse(coils_engaged_after_subdeadband,
                         "coil must drop once ramped magnitude is "
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
    """W4-06 SIL: releasing one logical axis with a sibling still
    active skips the ramp entirely on that axis."""

    # TR-G — two logical axes active (left_track + arms via lhy=120,
    # rhy=120). Release arms while left_track still active → arms goes
    # straight to 0 in the same tick, no ``ramping`` flag set.
    def test_tr_g_release_with_sibling_active_skips_ramp(self):
        arb = FourAxisArbiter()
        t = 0
        # Warm up with both left_track (via lhy) and arms (via rhy) active.
        while t < 200:
            arb.tick(0, 120, 0, 120, t)
            t += RAMP_TICK_MS
        self.assertGreater(arb.arms.effective, AXIS_DEADBAND)

        # Release arms (rhy=0); left_track still active.
        out = arb.tick(0, 120, 0, 0, t)
        self.assertEqual(out["arms"], 0, "arms must skip to 0 immediately")
        self.assertFalse(arb.arms.ramping,
                         "arms must not enter ramp state on mixed release")
        # And left_track untouched.
        self.assertEqual(out["left_track"], 120)

    # TR-H — same outcome regardless of which logical axis is the holder.
    def test_tr_h_skip_is_orientation_agnostic(self):
        # Each entry maps a "holder" logical axis to the stick input that
        # activates it (and only it). We then drive arms (rhy) plus the
        # holder, release arms, and assert arms skips while the holder
        # remains active.
        holders = {
            "left_track":  (0,    120, 0, 0),   # lhy=120 → both tracks fwd; we only need left
            "right_track": (0,    120, 0, 0),   # mirror of left (mixing makes both = 120)
            "bucket":      (0,      0, 100, 0), # rhx=100 → bucket only
        }
        for name, (lhx, lhy, rhx, _rhy) in holders.items():
            with self.subTest(holder=name):
                arb = FourAxisArbiter()
                t = 0
                # Warm up with the holder + arms (rhy=120) active.
                while t < 200:
                    arb.tick(lhx, lhy, rhx, 120, t)
                    t += RAMP_TICK_MS
                self.assertGreater(arb.arms.effective, AXIS_DEADBAND)
                # Release arms only.
                out = arb.tick(lhx, lhy, rhx, 0, t)
                self.assertEqual(out["arms"], 0,
                                 f"arms must skip with {name} held")
                self.assertFalse(arb.arms.ramping)

    # TR-I — last-axis release falls back to the ramp path. Sequence:
    # release arms with left_track held (skip), then release left_track
    # alone (ramp).
    def test_tr_i_last_release_takes_ramp_path(self):
        arb = FourAxisArbiter()
        t = 0
        # Warm up: lhy=120 (→ left_track=120, right_track=120) + rhy=120 (arms).
        # K-A4 will couple the two track ramps later; for this test we want
        # to release ARMS first so only the tracks remain active and they
        # release together (still a single fall-through).
        while t < 200:
            arb.tick(0, 120, 0, 120, t)
            t += RAMP_TICK_MS

        # Release arms first — arms skips (left_track + right_track held).
        arb.tick(0, 120, 0, 0, t)
        self.assertEqual(arb.arms.effective, 0)
        self.assertFalse(arb.arms.ramping)
        t += RAMP_TICK_MS

        # Hold tracks steady for a beat, then release. Both tracks must
        # ramp (and K-A4 will give them a shared duration).
        arb.tick(0, 120, 0, 0, t)
        t += RAMP_TICK_MS
        arb.tick(0, 0, 0, 0, t)
        self.assertTrue(arb.left_track.ramping,
                        "last-axis release must take the ramp path")
        self.assertEqual(arb.left_track.start, 120)
        self.assertEqual(arb.left_track.duration_ms, 2000)  # track @ 120 → 2 s

    # TR-J — pin the apply_control comment: ``other_active`` is computed
    # against *current-tick logical-axis inputs*, not the ``effective``
    # of an already-ramping sibling. So a previously-released-and-now-
    # ramping logical axis must NOT block another logical axis from
    # taking its own ramp path when released second with no other
    # currently-active sibling.
    def test_tr_j_stale_ramping_sibling_does_not_block_skip_decision(self):
        arb = FourAxisArbiter()
        t = 0
        # Warm up: bucket (rhx=100) + arms (rhy=120) + left_track via
        # lhy=120 — three logical axes simultaneously active.
        while t < 200:
            arb.tick(0, 120, 100, 120, t)
            t += RAMP_TICK_MS

        # Release bucket (rhx=0) with arms + left_track still active — skip.
        arb.tick(0, 120, 0, 120, t)
        self.assertEqual(arb.bucket.effective, 0)
        self.assertFalse(arb.bucket.ramping)
        t += RAMP_TICK_MS

        # Now release arms (rhy=0) with only left_track still active — skip.
        out = arb.tick(0, 120, 0, 0, t)
        self.assertEqual(out["arms"], 0)
        self.assertFalse(arb.arms.ramping)
        t += RAMP_TICK_MS

        # Finally release tracks. All other logical axes are at
        # effective=0 (skipped earlier). The arbiter computes
        # ``other_active`` from current-tick logical inputs which are
        # all zero → tracks MUST take the ramp path, not skip.
        arb.tick(0, 0, 0, 0, t)
        self.assertTrue(arb.left_track.ramping,
                        "track release with no active sibling must "
                        "ramp; the previously-skipped axes' effective=0 "
                        "must not be observable to the skip check")
        self.assertEqual(arb.left_track.start, 120)
        self.assertEqual(arb.left_track.duration_ms, 2000)


if __name__ == "__main__":
    unittest.main()
