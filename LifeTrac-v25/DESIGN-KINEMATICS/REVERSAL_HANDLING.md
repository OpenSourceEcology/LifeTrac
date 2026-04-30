# REVERSAL_HANDLING

**Status.** BC-22 LANDED Round 45 (decay-then-settle on same-tick sign-flip).
``valve_settling_ms`` schema-leaf integration deferred — the as-shipped
settle window uses a hard-coded ``REVERSAL_BRAKE_MS = 100`` (2 arbiter ticks).

## Scope

Detection and sequencing of **cross-zero direction reversals** on logical
axes. When a stick movement causes a logical axis to flip sign while still
carrying significant magnitude, the naive ramp passes the new value through
immediately and the hydraulic motor must reverse against its own inertia.

## Problem

Concrete scenario: operator slams `left_x` from `+1.0` (full right turn) to
`-1.0` (full left turn) in one tick. With `lhy = 0`:

```
leftTrack_prev  = lhy + lhx_prev = 0 + 1.0 = +1.0
leftTrack_new   = lhy + lhx_new  = 0 - 1.0 = -1.0
```

Without reversal handling, the left-track command jumps from `+1.0` to `-1.0`
instantly. Consequences:

- **Mechanical:** drivetrain, wheel motor, and chain take a step impulse;
  shock-loads bearings and welds.
- **Hydraulic:** motor must reverse against its rotational inertia + fluid
  inertia in the line; can cavitate the suction side and spike the pressure
  side over relief setting.

## Brake-then-reverse sequence (BC-22 — landed Round 45)

When ``sign(raw) ≠ sign(r.effective)`` AND both are above ``AXIS_DEADBAND``
at the top of ``step_axis_ramp()``:

1. **Decay phase.** A fresh ramp from ``r.effective`` toward ``0`` is started
   using the standard release ladder (``ramp_duration_ms(|r.effective|, is_arm)``).
   ``r.reversal_pending`` is set to mark this as a BC-22 decay (not a
   plain release).
2. **Decay-shield invariant.** While ``r.ramping && r.reversal_pending``,
   the operator's still-held opposite-sign input is NOT allowed to cancel
   the ramp via the snap-on-activation path. The decay+settle sequence runs
   to completion deterministically — this is the central hydraulic-safety
   reason for BC-22.
3. **Settle phase.** When the decay ramp completes (``elapsed >= duration_ms``
   with ``r.reversal_pending == true``), ``r.brake_until_ms`` is set to
   ``millis() + REVERSAL_BRAKE_MS`` and ``r.reversal_pending`` is cleared.
   While ``millis() < r.brake_until_ms``, ``step_axis_ramp`` returns ``0``
   regardless of raw input.
4. **Resume.** Once the brake window elapses (``millis() >= r.brake_until_ms``),
   ``brake_until_ms`` is cleared and ``was_active`` is synthetically set to
   ``false`` for the current tick so the operator's then-current input is
   evaluated against a zero baseline (the new direction passes through if
   active, the axis stays at zero if released).

Implementation: [tractor_h7.ino](../DESIGN-CONTROLLER/firmware/tractor_h7/tractor_h7.ino)
``step_axis_ramp()``. Two new fields on ``struct AxisRamp``: ``bool
reversal_pending`` and ``uint32_t brake_until_ms``. One new constant:
``REVERSAL_BRAKE_MS = 100`` (= 2 × ``RAMP_TICK_MS``).

Verification: [test_reversal_brake_sil.py](../DESIGN-CONTROLLER/base_station/tests/test_reversal_brake_sil.py)
BR-A (decay phase shape + ladder selection + brake arming), BR-B (settle
window holds zero + ignores input changes + 2-tick width invariant), BR-C
(post-brake resumption with active or zero input), BR-D (non-reversal
cases pass through unmodified — same-sign jump, plain release, sub-deadband
flicker, mixed-mode skip), BR-E (arm reversals use arm decay ladder + same
shared settle window), BR-F (firmware source tripwires for ``BC-22`` marker,
``REVERSAL_BRAKE_MS = 100``, ``brake_until_ms`` field, brake-arm and
brake-clear paths).

## Schema impact

New leaf:

```toml
[hydraulic]
reversal_brake_enabled = true     # bool, live, default true
```

Disable for racing / high-performance builds where the operator wants raw
stick→track response.

## Detection precedence

Reversal detection runs **after** mixing (BC-21) and **before** ramping. The
reversal sequence **replaces** the standard ramp for the affected axis until
the sequence completes; new operator inputs during the sequence update the
target value but do not interrupt the brake/settle phases (the settle phase
in particular must complete for hydraulic safety reasons).

## E-stop interaction

E-stop bypasses reversal handling entirely (E-stop already commands all axes
to 0; if axis_eff was non-zero at E-stop time the hydraulic safety properties
of the E-stop sequence already cover the worst case).

## Cross-references

- Hydraulic settling-time origin (BC-19): [BUILD_CONFIG.md](../DESIGN-CONTROLLER/BUILD_CONFIG.md)
- Soft-stop hydraulic safety properties: [SOFT_STOP_STRATEGY.md](../DESIGN-HYDRAULIC/SOFT_STOP_STRATEGY.md)
- Differential mixing (provides the logical axis reversal-handling operates on): [DIFFERENTIAL_MIXING.md](DIFFERENTIAL_MIXING.md)

## Pending (post-BC-22)

- [ ] Wire ``REVERSAL_BRAKE_MS`` to BC-19's ``[hydraulic].valve_settling_ms``
      so the constant becomes config-driven
- [ ] Add a ``[hydraulic].reversal_brake_enabled`` boolean for racing /
      high-performance builds that want raw stick→track response
- [ ] Quantify expected jerk reduction on bench (target: P95 jerk < 1.0 g/s
      during reversal) once HIL hardware is available
- [ ] Add waveform diagrams for the decay→settle→resume sequence
