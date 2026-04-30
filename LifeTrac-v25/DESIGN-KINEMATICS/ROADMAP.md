# ROADMAP — Kinematics

This roadmap captures the kinematics work surfaced during Round 43 ramping/turning
research. Each entry is sized as a single BC-round (schema leaves + dataclass +
codegen + firmware + SIL test + doc).

## Conventions

- **BC-** rounds are formal build-config-affecting rounds (require schema bump,
  codegen, MASTER_TEST_PROGRAM row, DECISIONS.md ADR).
- **K-** ideas are non-config kinematics tweaks that may or may not graduate
  to a BC round.
- Confidence: H = high, M = medium, L = low/exploratory.
- Risk: how much could go wrong if shipped.

## Pending — adopt from existing notes

### BC-21 — Mix-then-ramp [✓ LANDED Round 44]

**Status.** Shipped in Round 44. Logical-axis ramping is now canonical;
per-side coil mapping shipped as an incidental fix (pre-BC-21 pure spin-turns
produced zero drive coil activation).

**Files touched.**
- [tractor_h7.ino](../DESIGN-CONTROLLER/firmware/tractor_h7/tractor_h7.ino):
  4 ramp calls now operate on `g_ramp_left_track` / `g_ramp_right_track` /
  `g_ramp_arms` / `g_ramp_bucket` (logical axes). New `clip_to_int8()`
  saturating cast helper. Per-side coil mapping replaces old `lhy`-only mapping.
- [test_axis_ramp_sil.py](../DESIGN-CONTROLLER/base_station/tests/test_axis_ramp_sil.py):
  `FourAxisArbiter` rewritten to mirror new pipeline; W4-05 / W4-06 tests
  re-expressed on logical axes (10 tests, behavioural invariants preserved).
- [test_track_mix_ramp_sil.py](../DESIGN-CONTROLLER/base_station/tests/test_track_mix_ramp_sil.py)
  (new): 25 tests across RT-A..RT-G covering mixing math, spin-turn coils,
  smooth-curve no-step, and source-tripwire markers.
- [DIFFERENTIAL_MIXING.md](DIFFERENTIAL_MIXING.md), [RAMP_PROFILES.md](RAMP_PROFILES.md):
  promoted from stub to landed status.

**Schema impact.** None — re-uses existing `track_axis_count`.

**Confidence.** H. Risk: M (changes joystick feel; needs operator sign-off).

**Reviews calling for this fix.**
- [2026-04-26_Controller_Hardware_Testing_Readiness_Review.md](../AI%20NOTES/2026-04-26_Controller_Hardware_Testing_Readiness_Review.md)
- [CODE_REVIEW_FIXES.md](../DESIGN-CONTROLLER/firmware/CODE_REVIEW_FIXES.md)
- [2026-04-28_Controller_Code_Pipeline_Review_GitHub_Copilot_v1_0.md](../AI%20NOTES/2026-04-28_Controller_Code_Pipeline_Review_GitHub_Copilot_v1_0.md)

---

### BC-22 — Cross-zero direction reversal soft-stop [MEDIUM]

**Status.** ✓ LANDED Round 45. See [REVERSAL_HANDLING.md](REVERSAL_HANDLING.md)
and [test_reversal_brake_sil.py](../DESIGN-CONTROLLER/base_station/tests/test_reversal_brake_sil.py)
BR-A..BR-F (21 tests). Schema leaf ``hydraulic.reversal_brake_enabled``
deferred — the as-shipped behaviour is unconditionally enabled with
``REVERSAL_BRAKE_MS = 100`` (= 2 × RAMP_TICK_MS).

**Problem.** When operator slams `left_x` from `+full` (right turn) to `-full`
(left turn), the **left** track command flips sign instantly. Hard mechanical
jolt + cavitation risk on the hydraulic motor.

**Fix.** When new and previous logical-axis values have opposite signs AND
previous magnitude > deadband, force a brake-then-reverse sequence:
1. Decay current direction to zero (existing stop ramp).
2. Hold zero for `valve_settling_ms` (re-uses BC-19 constant).
3. Start fresh ramp toward new direction.

**Schema impact.**
- New leaf `hydraulic.reversal_brake_enabled` (bool, default `true`, `live`).
- BUILD_CONFIG.md hydraulic section + CAPABILITY_INVENTORY row.

**Confidence.** M. Risk: M (could feel patronising to experienced operators;
schema-toggleable so easy to disable).

**Depends on.** BC-21 (so reversal is detected on logical axes, not stick axes).

---

### BC-23 — Saturation-aware steering priority [LOW]

**Status.** ✓ LANDED Round 46. See [DIFFERENTIAL_MIXING.md](DIFFERENTIAL_MIXING.md)
§ "Saturation policy: preserve-steering" and
[test_steering_priority_sil.py](../DESIGN-CONTROLLER/base_station/tests/test_steering_priority_sil.py)
SP-A / SP-B / SP-E.

**Problem.** When `|lhy| + |lhx| > 127`, today's `clip(±127)` consumes throttle
budget before steering, so a forward-pinned stick + steering input loses the
turn.

**Fix.** Proportional scale: `if |lhy|+|lhx| > 127: scale = 127/(|lhy|+|lhx|)`.

**Schema impact.**
- New leaf `hydraulic.steering_priority` enum
  `equal | throttle_first | steering_first`, default `equal`.

**Confidence.** H (math is well-known from skid-steer industry).
Risk: L (operator-feel change, schema-toggleable).

**Depends on.** BC-21 (mixing must live in M7 first).

---

### BC-24 — Spin-turn flow boost [LOW]

**Status.** ✓ LANDED Round 46. See [FLOW_BUDGETING.md](FLOW_BUDGETING.md)
§ "Spin-turn under-budgeting (BC-24 — landed Round 46)" and
[test_steering_priority_sil.py](../DESIGN-CONTROLLER/base_station/tests/test_steering_priority_sil.py)
SP-C / SP-D / SP-E.

**Problem.** Pure spin-turn (`lhx = ±127, lhy = 0`) commands both tracks at
full magnitude → ~2× flow demand vs straight drive. But `REG_FLOW_SP` uses
`max(|axis|) = 127` → setpoint says "100%", actual demand is 200%, tracks crawl.

**Fix.** Flow-setpoint formula augmentation:
```
spin_boost = if sign(leftTrack) != sign(rightTrack):
                min(|leftTrack| + |rightTrack|, 127) - max(|leftTrack|, |rightTrack|)
             else: 0
flow_sp = clamp(max(|track|, |arms|, |bucket|) + spin_boost, 0, 127)
```
For dual-EFC builds (`hydraulic.flow_valve_count = 2`), each EFC commands its
own side's full magnitude → boost is structural and this fix is a no-op.

**Schema impact.** None at minimum (formula change). Optional new leaf
`hydraulic.spin_turn_boost_enabled` (bool, default `true`).

**Confidence.** M. Risk: L for single-EFC, no-op for dual-EFC.

**Depends on.** BC-21.

---

### BC-25 \u2014 Per-stick response curve exponent [LOW]

**Status.** \u2713 LANDED Round 48. Implements cheap-win K-A2.
See [INPUT_MAPPING.md](INPUT_MAPPING.md) \u00a7 "Stick curve" and
[test_stick_curve_sil.py](../DESIGN-CONTROLLER/base_station/tests/test_stick_curve_sil.py)
SC-A / SC-B / SC-C / SC-D / SC-E / SC-F.

**Problem.** Linear stick-to-flow mapping gives the operator the same
sensitivity at low-stick (creep, fine bucket trim) as at full deflection.
Skid-steer industry practice is to apply a soft expo curve so the bottom
half of stick travel commands proportionally less flow, while pegged
sticks still hit 100%.

**Fix.** New schema leaf `ui.stick_curve_exponent` \u2208 `{1.0, 1.5, 2.0}`,
default `1.0` (identity \u2014 no behavioural change vs pre-Round-48). Firmware
precomputes a 128-entry uint8_t LUT at boot from
`LIFETRAC_UI_STICK_CURVE_EXPONENT` and applies
`effective = sign(x) \u00d7 LUT[|x|]` to every raw stick axis (`lhx`, `lhy`,
`rhx`, `rhy`) **post-deadband, pre-mixing**. Pegged sticks (\u00b1127) reach
\u00b1127 at every supported exponent; the 50% point compresses to ~32 at n=2.0.

**Schema impact.** New required leaf under `[ui]`. `reload_class =
restart_required` (LUT is built once in `setup()`).

**Confidence.** H (industry-standard math; identity default is a no-op).
Risk: L (operator-feel only; opt-in per build).

**Depends on.** BC-21 (mixing must consume the curved values, not raw).

---

### BC-26 — S-curve ramp shape (half-cosine smoothstep) [LOW]

**Status.** ✓ LANDED Round 49. Implements cheap-win K-A3.
See [test_ramp_shape_sil.py](../DESIGN-CONTROLLER/base_station/tests/test_ramp_shape_sil.py)
RS-A / RS-B / RS-C / RS-D / RS-E / RS-F / RS-G.

**Problem.** The pre-Round-49 release-ramp / BC-22 reversal-decay path
uses straight linear interpolation `start × (duration - elapsed) /
duration`. The velocity profile is therefore a step at engagement
(immediate) and another step at the deadline (immediate stop), giving
the operator (and the implements) a measurable jerk at both endpoints.

**Fix.** New schema leaf `hydraulic.ramp_shape` ∈ `{linear, scurve}`,
default `linear` (byte-for-byte identity — no behavioural change vs
pre-Round-49). Firmware factors the interpolation into a
`ramp_interpolate(start, elapsed, duration_ms)` helper. When the
build defines `LIFETRAC_HYDRAULIC_RAMP_SHAPE_SCURVE` the helper
substitutes a half-cosine smoothstep `shape(t) = 0.5 × (1 + cos(π t))`
whose derivative is zero at both `t = 0` and `t = 1`, cutting P95
jerk roughly in half for the same total stop distance. Pinned by
closed-form quarter-point values: `t = 0.25 → 0.854`, `t = 0.5 → 0.5`,
`t = 0.75 → 0.146`.

**Schema impact.** New required leaf under `[hydraulic]`. `reload_class
= restart_required` (compile-time branch on the macro).

**Confidence.** H (single helper; closed-form pinned in SIL).
Risk: L (operator-feel only; opt-in per build).

**Depends on.** BC-21 (release ramps live on the logical-axis ramps).

---

### BC-27 — Confined-space mode (1.5× ramp duration) [LOW]

**Status.** ✓ LANDED Round 50. Implements cheap-win K-D2 (minimal version).
See [test_confined_space_sil.py](../DESIGN-CONTROLLER/base_station/tests/test_confined_space_sil.py)
CS-A / CS-B / CS-C / CS-D / CS-E / CS-F / CS-G.

**Problem.** Operating LifeTrac in tight quarters (inside a barn, near
people, against a fence) calls for a "creep mode" where stops are
extra-gentle without re-tuning the operator's ramp/curve preferences.
A per-build static knob is too coarse — the operator wants to opt in
for the duration of the task, then revert.

**Fix.** New schema leaf `ui.confined_space_mode_enabled` (bool,
default `false` = byte-for-byte identity). When `true`,
`ramp_duration_ms()` multiplies its base ladder result by `3 / 2`
(integer-exact for every ladder value: 250→375, 500→750, 1000→1500,
2000→3000). The multiplier applies uniformly to release ramps,
BC-22 reversal-decay ramps, and the K-A4 forced-coordinated track
duration so the bilateral-stop guarantee still holds. Composes
orthogonally with BC-25 stick curve and BC-26 scurve ramp shape.
Minimal version — no curve / flow cap change (deferred to a future
full K-D2 if real-world testing motivates it).

**Schema impact.** New required leaf under `[ui]`. `reload_class =
restart_required` (compile-time macro on the codegen header).

**Confidence.** H (single multiplier; closed-form pinned in SIL;
default path is identity).
Risk: L (operator opt-in per build; no firmware-shape change).

**Depends on.** BC-21 (logical-axis ramps), BC-26 (composes with).

---

### BC-28 — Operator profile preset (config-layer override bundle) [LOW]

**Status.** ✓ LANDED Round 51. Implements cheap-win K-D1 (minimal version).
See [test_operator_profile_sil.py](../DESIGN-CONTROLLER/base_station/tests/test_operator_profile_sil.py)
OP-A / OP-B / OP-C / OP-D / OP-E.

**Problem.** Operators have started accumulating individual feel
leaves — BC-25 stick curve, BC-26 ramp shape, BC-27 confined-space
mode — and tuning all three together for a coherent vibe (e.g. "feels
right in the barn" or "feels crisp on flat dirt") is fiddly. Worse,
an operator who flips one leaf and forgets about the others can land
in a confusing intermediate state.

**Fix.** New schema leaf `ui.operator_profile` enum
`{normal, gentle, sport}`, default `normal` = byte-for-byte identity
(individual leaves stand as authored). `gentle` overrides
`ui.confined_space_mode_enabled = true` AND
`hydraulic.ramp_shape = "scurve"` for tight-quarters work. `sport`
overrides `ui.confined_space_mode_enabled = false`,
`hydraulic.ramp_shape = "linear"` AND `ui.stick_curve_exponent = 1.0`
for crisp control. Overrides apply at config-load time inside
`build_config.load()` via the `_apply_operator_profile_overrides()`
helper, which mutates the parsed TOML dict in place after schema
validation but before the hydraulic-compatibility cross-check. Both
downstream Python consumers and the codegen-emitted firmware header
automatically pick up the post-override state because both flow
through the same loader. Bundles only touch operator-feel leaves
with `reload_class = restart_required`; safety / hydraulic-topology /
network leaves are NEVER overridden by a profile.

**Schema impact.** New required leaf under `[ui]`. `reload_class =
restart_required` (compile-time macro on the codegen header AND
bundle changes need a fresh header anyway).

**Confidence.** H (single override helper; closed-form bundle table
pinned in SIL; default `normal` is identity). Risk: L (operator
opt-in per build; no firmware-shape change; pure config layer).

**Depends on.** BC-25 (stick curve), BC-26 (ramp shape), BC-27
(confined-space mode) — BC-28 bundles all three.

---

### BC-29 — Configurable axis deadband [LOW]

**Status.** ✓ LANDED Round 52. Implements cheap-win K-D3.
See [test_axis_deadband_sil.py](../DESIGN-CONTROLLER/base_station/tests/test_axis_deadband_sil.py)
DB-A / DB-B / DB-C / DB-D / DB-E.

**Problem.** The tractor M7 firmware hard-coded
`AXIS_DEADBAND = 13` (≈10% of int8 full scale) for all four use
sites — `axis_active()`, per-coil activation, BC-24 spin-turn
detection, and the flow-set-point computation. Operators with
jittery hands or cold-day stiff sticks have no way to widen it; fine
arms work has no way to shrink it.

**Fix.** New schema leaf `ui.axis_deadband` (integer, range
`0..32`, default `13` = byte-for-byte identity vs the pre-Round-52
behaviour). Firmware sources the constant from the codegen-emitted
header: `static const int8_t AXIS_DEADBAND = (int8_t)
LIFETRAC_UI_AXIS_DEADBAND;`. All four use sites continue to read the
same compile-time constant, so the spin-turn detect / coil-activation
/ flow-setpoint logic stays in sync.

**Schema impact.** New required leaf under `[ui]`. `reload_class =
restart_required` (compile-time `static const` initialiser).

**Confidence.** H (single literal substitution; default = identity;
codegen path already exercised by BC-25/26/27/28).
Risk: L (per-build opt-in; bounded range; firmware tripwire pins
the macro source and the four use sites).

**Depends on.** BC-03 (codegen header path).

---

### Cheap wins (SIL-only, ship anytime)

| ID | Idea | Confidence | Notes |
|---|---|---|---|
| K-A1 | Asymmetric accel vs decel ramps (fast start, slow stop) | H | New leaf `hydraulic.track_accel_seconds`, default = 0.5 × `track_ramp_seconds`. |
| K-A2 | Stick-curve exponent `effective = sign(x) × |x|^n` | ✓ LANDED Round 48 | Shipped as BC-25; new leaf `ui.stick_curve_exponent` ∈ `{1.0, 1.5, 2.0}`, default 1.0 = identity. |
| K-A3 | S-curve ramp shape (half-cosine smoothstep) | ✓ LANDED Round 49 | Shipped as BC-26; new leaf `hydraulic.ramp_shape` ∈ `{linear, scurve}`, default `linear` = identity. Cuts P95 jerk ~½ for same stop distance. |
| K-A4 | Coordinated bilateral stop (both tracks reach 0 at same wallclock time) | ✓ LANDED Round 44 | Shipped alongside BC-21; uses new `forced_duration_ms` parameter on `step_axis_ramp`. |

### IMU-coupled (gated on BC-20A passive jerk telemetry shipping)

| ID | Idea | Confidence | Notes |
|---|---|---|---|
| K-B1 | Slope-compensated ramps (downhill = shorter, uphill = shorter) | M | Pitch from BNO086 quaternion; scale ramp `× cos(pitch)` clamped to ±20%. |
| K-B2 | Audit-log soft-stop violations (jerk > 2 × P95_target) | H | Useful tuning data; informs BC-20B advisory. |
| K-B3 | Pitch-rate veto on BC-20C ramp shortening | H | Refuse to shorten if recent stops show |pitch_rate| > threshold (bucket bounce). |

### Pump/load-aware (gated on BC-12 HIL bench-up + pressure sensors present)

| ID | Idea | Confidence | Notes |
|---|---|---|---|
| K-C1 | Pressure-aware ramp lengthening | M | If P95 pressure > 80% relief, use `ramp × 1.3`. Matches mass-energy at stop to load. |
| K-C2 | Hydraulic temperature gating | M | New leaf `hydraulic.cold_start_settling_ms_extra`; needs new tank thermistor. |
| K-C3 | Cooldown ramp on E-stop recovery | H | First post-E-stop transition uses `× 1.5` start ramp; absorbs operator over-correction. |

### Operator-feel modes (pure schema, no firmware change)

| ID | Idea | Confidence | Notes |
|---|---|---|---|
| K-D1 | Operator profile preset `ui.operator_profile` ∈ `{gentle, normal, sport}` | ✓ LANDED Round 51 (minimal) | Shipped as BC-28; new enum leaf `ui.operator_profile`, default `normal` = identity. Bundles override the BC-25/BC-26/BC-27 leaves at config-load time. |
| K-D2 | Confined-space mode (hot toggle: `× 1.5` ramp, cube curve, 60% flow cap) | ✓ LANDED Round 50 (minimal) | Shipped as BC-27; new bool leaf `ui.confined_space_mode_enabled`, default `false` = identity. Minimal version: 1.5× ramp duration only (curve/flow cap deferred). Composes with BC-25/BC-26. |
| K-D3 | Configurable axis deadband (was hard-coded `13` ≈ 10% of int8) | ✓ LANDED Round 52 | Shipped as BC-29; new int leaf `ui.axis_deadband` ∈ `[0, 32]`, default `13` = identity. Firmware `AXIS_DEADBAND` constant sourced from codegen macro. |

### Diagnostics & observability (pure software, ship anytime)

| ID | Idea | Confidence | Notes |
|---|---|---|---|
| K-E1 | Per-transition CSV log on M7 SD card | H | Circular buffer; useful for post-hoc tuning without telemetry burden. |
| K-E2 | Web UI ramp-state heatmap (effective bar lags raw bar) | ✓ LANDED Round 53 | Pure-Python data model in `base_station/ramp_heatmap.py` (5-state classifier: idle / matched / reversal / decay / mushy). `DEFAULT_DEADBAND` mirrors BC-29 `ui.axis_deadband` default. Painter wiring deferred. |

## Recommended sequencing

```
Round 44: ✓ BC-21 mix-then-ramp + ✓ K-A4 coordinated bilateral stop  [LANDED]
Round 45: ✓ BC-22 reversal brake  [LANDED] — K-A1 / K-A2 / K-A3 deferred
Round 46: ✓ BC-23 preserve-steering + ✓ BC-24 spin-turn flow boost  [LANDED]
Round 47: ✓ INPUT_MAPPING + MOTION_PRIMITIVES doc-debt promotion + MP-01..MP-12 SIL traceability  [LANDED]
Round 48: ✓ BC-25 / K-A2 stick-curve exponent  [LANDED]
Round 49: ✓ BC-26 / K-A3 S-curve ramp shape  [LANDED]
Round 50: ✓ BC-27 / K-D2 confined-space mode (minimal)  [LANDED]
Round 51: ✓ BC-28 / K-D1 operator profile preset (minimal)  [LANDED]
Round 52: ✓ BC-29 / K-D3 configurable axis deadband  [LANDED]
Round 53: ✓ K-E2 web UI ramp-state heatmap (data model)  [LANDED]
Round 47: K-D1 / K-D2 operator profiles + K-E1 / K-E2 diagnostics
─── Gate: BC-20A passive jerk telemetry shipped ───
Round 48+: K-B1 slope compensation, K-B2 audit log, K-B3 pitch-rate veto
─── Gate: BC-12 HIL hardware bench-up complete ───
Round 49+: K-C1 / K-C2 / K-C3 pump/temp aware
```

## Cross-references

- Source synthesis: see `AI NOTES/` Round 43 ramping/turning research session
  (transcript captured in chat session log).
- Hydraulic safety properties: [SOFT_STOP_STRATEGY.md](../DESIGN-HYDRAULIC/SOFT_STOP_STRATEGY.md).
- BuildConfig conventions: [BUILD_CONFIG.md](../DESIGN-CONTROLLER/BUILD_CONFIG.md).
