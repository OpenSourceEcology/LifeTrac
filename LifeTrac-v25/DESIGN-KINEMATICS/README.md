# DESIGN-KINEMATICS

**Scope.** How operator inputs (joysticks, buttons, web UI, ROS2 commands) translate
into physical motion of the LifeTrac, given a specific hydraulic build variant
(spool type, valve count, pump architecture). This folder owns the **mapping,
mixing, shaping, and sequencing** of motion commands. It does not own:

- The **physical plumbing** (lives in [DESIGN-HYDRAULIC/](../DESIGN-HYDRAULIC/))
- The **firmware pipeline / transport / safety latches** (lives in [DESIGN-CONTROLLER/](../DESIGN-CONTROLLER/))
- The **mechanical geometry** (lives in [DESIGN-STRUCTURAL/](../DESIGN-STRUCTURAL/))

Kinematics sits at the **intersection** of those three. When a change crosses
folders (typical for a BC-round affecting motion), the design doc lives here and
the plumbing/firmware/structure folders link **into** here.

## Why this folder exists

Prior to Round 44 these topics were smeared across three homes:

| Topic | Was in | Now canonical here |
|---|---|---|
| Differential mixing (`leftTrack = Y ± X`) | controller research code | [DIFFERENTIAL_MIXING.md](DIFFERENTIAL_MIXING.md) |
| Per-axis ramp ladder | [DECELERATION_FEATURE.md](../DESIGN-CONTROLLER/RESEARCH-CONTROLLER/arduino_opta_controller/DECELERATION_FEATURE.md) | [RAMP_PROFILES.md](RAMP_PROFILES.md) |
| EFC ramp + valve settling sequence | [SOFT_STOP_STRATEGY.md](../DESIGN-HYDRAULIC/SOFT_STOP_STRATEGY.md) | Cross-references; hydraulic safety stays in DESIGN-HYDRAULIC |
| Cross-zero direction reversal | nowhere | [REVERSAL_HANDLING.md](REVERSAL_HANDLING.md) |
| Single-EFC vs dual-EFC vs tandem-pump motion implications | [CHANGELOG_FLOW_VALVES.md](../DESIGN-HYDRAULIC/CHANGELOG_FLOW_VALVES.md) | [FLOW_BUDGETING.md](FLOW_BUDGETING.md) |
| Build-variant × motion-primitive expectations | nowhere | [BUILD_VARIANT_MOTION_MATRIX.md](BUILD_VARIANT_MOTION_MATRIX.md) |

## Folder index

| File | Purpose |
|---|---|
| [README.md](README.md) | This file — index, glossary, scope |
| [MOTION_PRIMITIVES.md](MOTION_PRIMITIVES.md) | The 8–12 atomic motions the machine can perform |
| [INPUT_MAPPING.md](INPUT_MAPPING.md) | Stick / button / web / ROS2 → logical-axis mapping per build |
| [DIFFERENTIAL_MIXING.md](DIFFERENTIAL_MIXING.md) | Y±X mixing, saturation, steering priority (BC-23) |
| [RAMP_PROFILES.md](RAMP_PROFILES.md) | Ladder, S-curve, asymmetric accel/decel (BC-21, A1, A3) |
| [REVERSAL_HANDLING.md](REVERSAL_HANDLING.md) | Cross-zero brake-then-reverse (BC-22) |
| [FLOW_BUDGETING.md](FLOW_BUDGETING.md) | Single vs dual EFC, spin-turn boost (BC-24) |
| [BUILD_VARIANT_MOTION_MATRIX.md](BUILD_VARIANT_MOTION_MATRIX.md) | Reference builds × primitives → expected behaviour |
| [ROADMAP.md](ROADMAP.md) | BC-21..24 plan + idea backlog |

## Glossary

- **Logical axis.** A motion intent expressed in machine frame, e.g.
  `leftTrack ∈ [-1, +1]`, `arms ∈ [-1, +1]`. Distinct from **stick axis**
  (`lhx`, `lhy`, `rhx`, `rhy`) which is the raw input.
- **Mixing.** Transforming stick axes → logical axes (e.g. `leftTrack = lhy + lhx`).
- **Ramping / shaping.** Time-domain transformation of a logical axis to limit
  jerk and acceleration. Owned here; safety properties (cavitation, pressure
  equalisation) owned by [SOFT_STOP_STRATEGY.md](../DESIGN-HYDRAULIC/SOFT_STOP_STRATEGY.md).
- **Sequencing.** Ordering of valve commands across time, e.g. brake → settle →
  reverse. Cross-references hydraulic settling-time constants.
- **Build variant.** A configured combination of spool type, valve count, pump
  architecture, IMU presence, etc. Captured by [BuildConfig](../DESIGN-CONTROLLER/BUILD_CONFIG.md).
- **Motion primitive.** An atomic operator-intelligible motion (drive forward,
  spin in place, lift arms, etc.). Enumerated in [MOTION_PRIMITIVES.md](MOTION_PRIMITIVES.md).

## Cross-references out

- Hydraulic safety properties of soft-stop: [DESIGN-HYDRAULIC/SOFT_STOP_STRATEGY.md](../DESIGN-HYDRAULIC/SOFT_STOP_STRATEGY.md)
- Build-variant configuration: [DESIGN-CONTROLLER/BUILD_CONFIG.md](../DESIGN-CONTROLLER/BUILD_CONFIG.md)
- Firmware ramping implementation: [DESIGN-CONTROLLER/firmware/tractor_h7/tractor_h7.ino](../DESIGN-CONTROLLER/firmware/tractor_h7/tractor_h7.ino)
- Flow-valve hardware variants: [DESIGN-HYDRAULIC/FLOW_VALVE_CONFIGURATION.md](../DESIGN-HYDRAULIC/FLOW_VALVE_CONFIGURATION.md)
