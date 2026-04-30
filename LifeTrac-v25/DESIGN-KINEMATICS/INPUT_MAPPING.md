# INPUT_MAPPING

**Status.** LANDED Round 47 (alongside BC-21 / BC-22 / BC-23 mixing,
reversal, and saturation refinements that this doc cross-references).
Schema-driven stick-curve / source-arbitration leaves remain pending.

## Scope

Canonical mapping from each **input source** to logical kinematic axes,
parameterised by build variant.

Input sources:

- **DroidPad joysticks** (BLE or MQTT): 4 raw axes `lhx`, `lhy`, `rhx`, `rhy`
  ∈ `[-127, +127]`
- **Web UI joysticks** (Raspberry Pi): same 4 axes, scaled to `[-1.0, +1.0]`
- **ROS2 Twist** (BeagleBone): `linear.x`, `angular.z` plus auxiliary topics
  for arms / bucket
- **Hardware joystick on Opta** (optional manual override): same 4 axes
- **E-stop button** (hardware): all axes → 0, latches until manual reset

Logical axes (output):

- `leftTrack ∈ [-1, +1]`
- `rightTrack ∈ [-1, +1]`
- `arms ∈ [-1, +1]`
- `bucket ∈ [-1, +1]`

## Default mapping (all builds)

```
leftTrack  = clip(lhy + lhx, ±1)
rightTrack = clip(lhy - lhx, ±1)
arms       = rhy
bucket     = rhx
```

This is the formula from `computeTrackSpeeds()` in
[lifetrac_v25_controller.ino](../DESIGN-CONTROLLER/RESEARCH-CONTROLLER/arduino_opta_controller/lifetrac_v25_controller.ino).

Saturation handling (BC-23) and reversal handling (BC-22) refine this base
formula; see [DIFFERENTIAL_MIXING.md](DIFFERENTIAL_MIXING.md) and
[REVERSAL_HANDLING.md](REVERSAL_HANDLING.md).

## Build-variant overrides

Will be populated as variants are formalised. Examples:

- **Mecanum-wheel research build** (hypothetical): adds strafe primitive,
  needs 4-axis mixer
- **Single-stick experimental** (hypothetical): both tracks + arms on one stick
  via mode toggle

## Deadband and shaping

- **Deadband.** Per [BUILD_CONFIG.md](../DESIGN-CONTROLLER/BUILD_CONFIG.md),
  `ui.joystick_deadband_pct` (default 10%) — values within ±deadband map to 0.
- **Stick curve.** As of Round 48 (BC-25 / K-A2), `ui.stick_curve_exponent`
  (default 1.0, options 1.5 / 2.0) applies `sign(x) × |x|^n` post-deadband
  and pre-mixing on every raw stick axis. Improves low-speed precision
  without sacrificing top-end. Default 1.0 is byte-for-byte identity, so
  legacy operator-feel is preserved unless a build opts in.

## Pending (post-Round 48)

- [x] Wire ``[ui].stick_curve_exponent`` schema leaf and add SIL coverage
      (K-A2). Landed Round 48 — see ``test_stick_curve_sil.py``.
- [ ] Pin per-source unit conventions (raw int vs normalised float) once
      the Web UI / ROS2 bridge formalise their wire formats
- [ ] Document priority arbitration when multiple sources active
      simultaneously (current behaviour: last-writer-wins on the M7
      ramp set-point)
- [ ] Document E-stop precedence (immediate, bypasses ramping — see
      SOFT_STOP_STRATEGY.md). Today E-stop forces ``g_ramp_*.setpoint = 0``
      AND ``effective = 0`` simultaneously, bypassing the decay ladder.
- [ ] ROS2 Twist → 4-axis decomposition formula (likely
      ``leftTrack = linear.x + angular.z * track_width / 2`` etc.)
