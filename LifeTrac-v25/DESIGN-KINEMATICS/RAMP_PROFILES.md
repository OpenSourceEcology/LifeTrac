# RAMP_PROFILES

**Status.** BC-21 + K-A4 LANDED Round 44 (logical-axis ramping + coordinated bilateral track stop). K-A1 / K-A2 / K-A3 still pending.

## Scope

Time-domain shaping of logical-axis values to limit jerk and acceleration.
**Hydraulic safety properties** of soft-stop (cavitation, pressure
equalisation, valve settling) are owned by
[SOFT_STOP_STRATEGY.md](../DESIGN-HYDRAULIC/SOFT_STOP_STRATEGY.md); this doc
owns the **mathematical shape** and **per-axis scheduling**.

## Current ramp ladder (linear, symmetric, per-axis)

From [DECELERATION_FEATURE.md](../DESIGN-CONTROLLER/RESEARCH-CONTROLLER/arduino_opta_controller/DECELERATION_FEATURE.md)
and `step_axis_ramp()` in
[tractor_h7.ino](../DESIGN-CONTROLLER/firmware/tractor_h7/tractor_h7.ino):

| Magnitude band | Track ramp duration | Arm/bucket ramp duration |
|---|---|---|
| ≥ 75% | 2000 ms | 1000 ms |
| ≥ 37% | 1000 ms | 500 ms |
| < 37% | 500 ms | 250 ms |

- **Linear interpolation** between current effective value and target.
- **Mixed-mode skip:** if any other axis transitions to active in this tick,
  snap to zero (no ramp). To be re-evaluated post-BC-21.
- **E-stop bypass:** ramping is bypassed entirely on E-stop.

## Pending shape changes

### K-A3 — S-curve / smoothstep [Round 45 candidate]

Replace linear interp with half-cosine smoothstep:

```
progress         = elapsed_ms / duration_ms              # ∈ [0, 1]
progress_smooth  = 0.5 * (1.0 - cos(π * progress))       # smoothstep
effective        = start + (target - start) * progress_smooth
```

Cuts P95 jerk approximately in half for the same total stop distance. New
schema leaf `hydraulic.ramp_shape` ∈ `{linear, scurve}`, default `linear`
initially, flip default to `scurve` after operator validation.

### K-A1 — Asymmetric accel vs decel [Round 45 candidate]

Today's ramp applies to **release** transitions only (target moving toward 0).
Press transitions (target moving away from 0) snap immediately. Operator-feel
research suggests this asymmetry is desirable but should be tunable:

- New leaf `hydraulic.track_accel_seconds` (float, default = 0.5 ×
  `track_ramp_seconds`). Applied on press transitions to give a brief
  ramp-up that prevents joint shock without making the machine feel sluggish.

### K-A4 — Coordinated bilateral stop [LANDED Round 44]

Implemented in [tractor_h7.ino](../DESIGN-CONTROLLER/firmware/tractor_h7/tractor_h7.ino)
`apply_control()`. When both track logical axes transition `active → released`
in the same tick (and neither is already ramping), a single shared
`forced_track_dur = ramp_duration_ms(max(|left.effective|, |right.effective|), is_arm=false)`
is computed and passed to both `step_axis_ramp` calls via the new
`forced_duration_ms` parameter (default 0 = use magnitude-derived ladder).

Verification: [test_track_mix_ramp_sil.py](../DESIGN-CONTROLLER/base_station/tests/test_track_mix_ramp_sil.py)
RT-D (symmetric, asymmetric, same-tick zero-crossing) and RT-F (single-track
release does not falsely trigger K-A4).

## Cross-references

- Mixing happens **before** ramping (post BC-21): [DIFFERENTIAL_MIXING.md](DIFFERENTIAL_MIXING.md)
- Reversal-handling overrides ramp on sign-flip: [REVERSAL_HANDLING.md](REVERSAL_HANDLING.md)
- Hydraulic safety properties: [SOFT_STOP_STRATEGY.md](../DESIGN-HYDRAULIC/SOFT_STOP_STRATEGY.md)
- IMU-adaptive ramp tuning (BC-20A/B/C): [SOFT_STOP_STRATEGY.md](../DESIGN-HYDRAULIC/SOFT_STOP_STRATEGY.md)

## TODO when promoted

- [ ] Plot waveform comparisons (linear vs scurve, symmetric vs asymmetric)
- [ ] Quantify P95 jerk reduction empirically (ties to BC-20A telemetry)
- [ ] Re-evaluate mixed-mode skip rule after BC-21 (mixing-first means the rule
      now operates on logical axes, which may make it unnecessary)
