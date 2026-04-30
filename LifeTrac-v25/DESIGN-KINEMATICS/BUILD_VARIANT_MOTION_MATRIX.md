# BUILD_VARIANT_MOTION_MATRIX

**Status.** STUB — fill in as build variants are formally captured.

## Scope

Cross-tabulation of **build variants** (rows) against **motion primitives**
(columns) showing which combinations are **supported**, **degraded**, or
**forbidden**.

## Reference build variants

These mirror the four reference builds documented in
[SOFT_STOP_STRATEGY.md](../DESIGN-HYDRAULIC/SOFT_STOP_STRATEGY.md) §3:

| ID | Name | spool_type | load_holding | flow_valve_count | imu_enabled |
|---|---|---|---|---|---|
| BV-1 | Canonical (post-BC-18) | tandem | spool_inherent | 1 | true |
| BV-2 | Pre-BC-18 interim | float | po_check | 1 | false |
| BV-3 | High-performance dual-EFC | tandem | spool_inherent | 2 | true |
| BV-4 | Float-mode forklift | float | counterbalance | 1 | true |

## Motion primitive support matrix

Columns reference primitive IDs from [MOTION_PRIMITIVES.md](MOTION_PRIMITIVES.md).

| Build | MP-01 fwd | MP-02 rev | MP-03 skid | MP-04 pivot | MP-05 spin | MP-06 raise | MP-07 lower | MP-08 curl | MP-09 dump | MP-10 drv+arm | MP-11 turn+buck | MP-12 float |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| BV-1 | ✓ | ✓ | ✓ | ✓ | ⚠ flow-limited (BC-24) | ✓ | ✓ | ✓ | ✓ | ⚠ shared flow | ⚠ shared flow | ✗ no float spool |
| BV-2 | ✓ | ✓ | ✓ | ✓ | ⚠ flow-limited | ✓ | ✓ | ✓ | ✓ | ⚠ shared flow | ⚠ shared flow | ✓ |
| BV-3 | ✓ | ✓ | ✓ | ✓ | ✓ | ✓ | ✓ | ✓ | ✓ | ✓ | ✓ | ✗ no float spool |
| BV-4 | ✓ | ✓ | ✓ | ✓ | ⚠ flow-limited | ✓ | ✓ | ✓ | ✓ | ⚠ shared flow | ⚠ shared flow | ✓ (arms only) |

Legend:

- **✓ supported** — primitive works as documented in [MOTION_PRIMITIVES.md](MOTION_PRIMITIVES.md)
- **⚠ degraded** — primitive functions but with a documented limitation (e.g.
  reduced speed under flow contention)
- **✗ forbidden** — primitive is structurally impossible on this build; the
  validator (BC-19) should reject any config trying to invoke it

## Validator hooks

- BC-19 hydraulic compatibility validator (in [build_config.py](../DESIGN-CONTROLLER/base_station/build_config.py))
  catches `spool_type` × `load_holding` mismatches at config-load time.
- Future: per-primitive build-gate map (e.g. MP-12 float requires
  `spool_type = float`) should be enforced at runtime when the firmware
  attempts to enter float mode on an incompatible build.

## TODO when promoted

- [ ] Fill in quantitative speed/flow numbers per cell (post-BC-12 HIL bench-up)
- [ ] Add columns for IMU-adaptive ramp tuning availability (BC-20A/B/C)
- [ ] Add columns for K-D1 operator-profile presets
- [ ] Cross-reference each ⚠ cell to the ROADMAP entry that addresses it
- [ ] Capture variant additions in DECISIONS.md when new BV-N is introduced
