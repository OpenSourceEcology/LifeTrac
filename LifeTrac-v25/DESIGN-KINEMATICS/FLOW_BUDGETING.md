# FLOW_BUDGETING

**Status.** BC-24 LANDED Round 46 (spin-turn flow boost on single-EFC builds).
``hydraulic.spin_turn_boost_enabled`` schema-leaf integration deferred —
the as-shipped boost is unconditionally enabled.

## Scope

How the system computes `REG_FLOW_SP_*` (electronic-flow-control setpoint)
from the active logical axes, accounting for build variant
(`hydraulic.flow_valve_count`, pump architecture).

## Current formula (single-EFC builds)

```
flow_sp = max(|leftTrack|, |rightTrack|, |arms|, |bucket|)   # written to both REG_FLOW_SP_1 and REG_FLOW_SP_2
```

This formula is correct for **strictly cooperating** axes (e.g. drive forward
+ raise arms — both want flow in the same proportion). It **under-budgets**
flow for **counter-rotating** track scenarios (spin-turn) where both tracks
move at full magnitude in opposite directions, but the formula sees only one
magnitude.

## Spin-turn under-budgeting (BC-24 — landed Round 46)

Implemented in [tractor_h7.ino](../DESIGN-CONTROLLER/firmware/tractor_h7/tractor_h7.ino)
``apply_control()``. Spin-turn detection compares post-ramp ``left_track``
and ``right_track``: when both are above ``AXIS_DEADBAND`` in opposite
directions, the track contribution to ``mag`` becomes ``min(127,
|left_track| + |right_track|)`` instead of ``max(|left_track|,
|right_track|)``. Same-sign cases (forward, reverse, smooth-curve turn)
continue to use max. Arms and bucket continue to use max in both cases
since they're independent valve banks.

For the spin-turn example: pure spin at ±120 yields
``min(127, 240) = 127`` → full flow set-point, vs the pre-BC-24 max of
120. For the mixed-turn example (``left_track = +60, right_track = -36``
after BC-23 scaling): ``min(127, 96) = 96``, vs the pre-BC-24 max of 60.

Verification: [test_steering_priority_sil.py](../DESIGN-CONTROLLER/base_station/tests/test_steering_priority_sil.py)
SP-C (pure spin reaches 10000 mV; partial spin uses sum; sum clamps to
int8 max) and SP-D (forward / reverse / smooth-curve / arms-only do NOT
trigger the boost).

## Dual-EFC builds (`hydraulic.flow_valve_count = 2`)

Per [CHANGELOG_FLOW_VALVES.md](../DESIGN-HYDRAULIC/CHANGELOG_FLOW_VALVES.md),
dual-EFC builds give Valve 1 to (left tracks + arms) and Valve 2 to
(right tracks + bucket). Each EFC commands its own side's full magnitude:

```
flow_sp_1 = max(|leftTrack|, |arms|)
flow_sp_2 = max(|rightTrack|, |bucket|)
```

This is **structurally** correct for spin-turn — each side sees its own full
magnitude — and BC-24's spin-boost augmentation is a no-op on dual-EFC builds.

## Tandem-pump architecture (deferred — see HYDRAULIC_BOM.md §9)

Future: dual independent pumps (e.g. Honor 2DG1BU0606R, ~6+6 GPM SAE-A).
Eliminates flow contention entirely; each pump dedicated to one track side.
Would graduate `flow_valve_count` to a 3-state enum or add a separate
`pump_architecture` leaf.

## Cross-references

- Build-variant hydraulic configuration: [BUILD_CONFIG.md](../DESIGN-CONTROLLER/BUILD_CONFIG.md)
- Flow-valve hardware variants: [FLOW_VALVE_CONFIGURATION.md](../DESIGN-HYDRAULIC/FLOW_VALVE_CONFIGURATION.md)
- Tandem-pump deferred design: [HYDRAULIC_BOM.md](../DESIGN-HYDRAULIC/HYDRAULIC_BOM.md) §9
- Spool-type interaction (BC-19): [BUILD_CONFIG.md](../DESIGN-CONTROLLER/BUILD_CONFIG.md)

## Pending (post-BC-24)

- [ ] Wire ``[hydraulic].spin_turn_boost_enabled`` schema leaf so racing /
      high-performance builds can opt out
- [ ] Quantify pump-demand math empirically (bench-up with flowmeters)
- [ ] Pin behaviour when arms+bucket simultaneously active during spin-turn
- [ ] Consider weighted formulas (arms have different displacement than tracks)
