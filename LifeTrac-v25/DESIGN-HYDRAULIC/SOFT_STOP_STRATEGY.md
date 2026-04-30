# Soft-Stop Strategy — EFC Ramp + Delayed Valve Centring

**Status:** Adopted 2026-04-29. This document specifies the canonical actuator soft-stop / soft-start strategy for LifeTrac v25 and the spool-selection consequences. It supersedes the load-holding hardware approach (pilot-operated check valves) that was sketched in the first-pass [HYDRAULIC_BOM.md](HYDRAULIC_BOM.md).

> **See also (kinematics):** ramp shape, per-axis scheduling, and stick-input
> mapping live in [DESIGN-KINEMATICS/](../DESIGN-KINEMATICS/README.md). This
> document owns the **hydraulic safety properties** of the soft-stop sequence
> (cavitation prevention, pressure equalisation, valve settling). For the
> mathematical ramp profiles see
> [DESIGN-KINEMATICS/RAMP_PROFILES.md](../DESIGN-KINEMATICS/RAMP_PROFILES.md);
> for cross-zero direction reversal sequencing see
> [DESIGN-KINEMATICS/REVERSAL_HANDLING.md](../DESIGN-KINEMATICS/REVERSAL_HANDLING.md).

## Why this exists

Earlier rounds settled on Parker D1VW directional valves with **spool 4 — float centre** (P blocked, A↔B↔T connected). Float centre allows track motors to coast freely on de-energise but lets implement cylinders drift under gravity. The mitigation was load-holding pilot-operated check valves on the rod sides of arm and bucket cylinders.

This document records a better answer: use the Brand Hydraulics EFC and Burkert 8605 — **already in the BOM for proportional flow control** — as an electronic soft-stop / soft-start sequencer. The EFC ramps flow to zero *before* the directional valve spring-centres, so the valve never closes on inertial flow. With that sequence in place, **tandem-centre spools (D1VW spool 8: P→T open through, A & B blocked when centred) become safe everywhere** and the design picks up:

- Natural cylinder load-holding without external PO check valves.
- Near-zero pump idle pressure (P→T open through stack) instead of full-flow EFC bypass heat.
- Mechanical hill-hold on track motors when fully stopped.
- Smoother operator feel — controlled deceleration instead of immediate coast.

The trade-off is exactly one thing: **no free coast on track release**. The tracks decelerate over ~0.5–2 s along the existing IP-303 ramp ladder instead of free-wheeling. For loader work, hill manoeuvring, and precise positioning this is a feature, not a regression. For high-speed traversal where free-wheeling matters, that profile is selectable (see *Spool variants* below).

## Sequence (canonical timing)

```
START — operator pushes joystick from neutral
  t = 0    ms : Opta receives axis command. EFC at idle (4 mA, 0 GPM).
                Opta energises directional valve solenoid (P→A or P→B).
  t ≈ 30   ms : D1VW spool fully shifted. Motor sees pressure but ~zero flow.
                No torque step, no jolt at the actuator.
  t = 30+  ms : Opta begins EFC current ramp 4 → 20 mA proportional to
                joystick magnitude. Brand EFC compensator opens; flow ramps
                from 0 GPM to setpoint over the firmware's command profile.
  t = ~250 ms : Steady-state flow reached at the operator's commanded level.

STOP — operator releases joystick to neutral
  t = 0    ms : Axis ramp begins (existing IP-303 logic). Ramped axis value
                drives REG_FLOW_SP_* downward. EFC current decays.
  t = 0–T  ms : EFC flow ramps from setpoint to ~0 GPM along ladder
                (T = 250..2000 ms depending on axis class & magnitude — see
                [tractor_h7.ino IP-303 block](../DESIGN-CONTROLLER/firmware/tractor_h7/tractor_h7.ino)).
  t = T    ms : Axis ramp reaches zero. Opta sees stable zero-flow command.
                **NEW:** Opta starts a settling timer (DEFAULT 100 ms).
  t = T+50 ms : EFC fully closed at hardware level (compensator dead-band).
  t = T+100ms : Settling timer expires. **Opta now de-energises directional
                valve solenoids.** Spool spring-centres over ~30 ms with
                near-zero flow in the line — no shock, no cavitation.
  t = T+130ms : Valve fully centred. Tandem centre passes residual P→T;
                cylinder lines blocked → load held mechanically.

E-STOP — emergency or unplanned shutdown (no soft-stop possible)
  t = 0    ms : Opta E-stop latch fires OR Opta loses power.
                EFC current drops to 0 (controller is current-driven, fail-safe).
                All directional valve solenoids de-energise.
  t = ~30  ms : All D1VW spools spring-centre simultaneously.
                Tandem centre blocks A & B → cylinders hold, track motor
                inertial flow is bottled. The pump-side relief valve
                (3000 PSI cracking, BOM §1) dumps the spike to tank.
                Cracking response < 5 ms — well inside any plausible spike.
```

The **STOP** path is the new behaviour this document standardises. The **START** path is already correct in the current firmware. The **E-STOP** path is unchanged but listed here so the relief valve's safety-critical role is explicit.

## Tuning parameters

| Parameter | Default | Range | Source / file |
|---|---|---|---|
| Track ramp duration ladder | 2000 / 1000 / 500 ms (≥75 % / ≥37 % / <37 % magnitude) | 250–4000 ms | `LIFETRAC_HYDRAULIC_TRACK_RAMP_SECONDS` in [common/lifetrac_build_config.h](../DESIGN-CONTROLLER/firmware/common/lifetrac_build_config.h); `ramp_duration_ms()` in [tractor_h7.ino IP-303 block](../DESIGN-CONTROLLER/firmware/tractor_h7/tractor_h7.ino) |
| Arm ramp duration ladder | 1000 / 500 / 250 ms | 100–2000 ms | `LIFETRAC_HYDRAULIC_ARM_RAMP_SECONDS`; same source |
| Axis deadband | ±13 / 127 (~10 %) | 5–25 | `AXIS_DEADBAND` in tractor_h7.ino |
| Ramp tick | 50 ms (20 Hz) | fixed by arbiter | `RAMP_TICK_MS` in tractor_h7.ino |
| **NEW — valve-centre settling delay** | **100 ms after ramp reaches zero** | **50–250 ms** | **TODO_FW** — to be added in `tractor_opta.ino` valve-drive path. See *Implementation work* below. |
| EFC dashpot orifice | smallest (fastest response) | factory configurable | Brand Hydraulics EFC physical adjustment at install time |
| Pump-side relief cracking | 3000 PSI, < 5 ms response | fixed | [HYDRAULIC_BOM.md §1](HYDRAULIC_BOM.md) |

Ramp shape is currently **linear**. A future round may switch to S-curve / quadratic for smoother feel; the IP-303 framework supports that with one function swap.

## Spool variants (canonical lock-in)

| Spool | Centre | Used for | Part number | Behaviour |
|---|---|---|---|---|
| **8 — Tandem** | P→T open through, A & B blocked | **All four functions** (track ×2, arms, bucket) | **D1VW00*8*CNKW** | Cylinder hold without PO check; low idle heat; controlled stop via EFC ramp |
| 4 — Float | P blocked, A↔B↔T | Optional service-mode profile (high-speed traversal) | D1VW004CNKW | Free coast on release; **not** the canonical choice — listed here only as a documented alternate spool if a future v25 variant needs free-wheeling tracks |

Going forward the canonical D1VW SKU is **D1VW00*8*CNKW × 4** (≈$528 each at Hyspeco). The **load-holding section of the BOM is removed**: no PO check valves, no pilot-line plumbing.

## Hydraulic variants & per-build configuration

LifeTrac is an open-source platform; other builders will not all run the canonical hydraulic stack. The firmware sequencer must therefore be **driven by [BuildConfig](../DESIGN-CONTROLLER/base_station/config/build.default.toml)**, not by hard-coded constants. The pattern mirrors how `safety.estop_topology` already lets a build pick `psr_monitored_dual` / `psr_monitored_single` / `hardwired_only` — the firmware branches on the enum, the SIL matrix exercises every value, and the C header carries the active choice as both a string and a `_<VARIANT>` boolean.

### Configuration knobs (proposed schema additions, BC-19)

To support arbitrary hydraulic builds we need three new leaves under `[hydraulic]`. Each carries a `reload_class` per the existing convention in [build_config.schema.json](../DESIGN-CONTROLLER/base_station/config/build_config.schema.json).

| Leaf | Type / range | Default (canonical v25) | reload_class | Drives |
|---|---|---|---|---|
| `hydraulic.spool_type` | enum: `tandem` / `float` / `closed` / `open` | `tandem` | `restart_required` | Whether the firmware sequencer needs to enforce the EFC-must-reach-zero-first rule before de-energising solenoids. `tandem` and `closed` need it; `float` and `open` do not. Also gates whether the BOM expects PO check valves on cylinders. |
| `hydraulic.load_holding` | enum: `spool_inherent` / `po_check` / `counterbalance` / `none` | `spool_inherent` | `restart_required` | What's responsible for holding the arms / bucket against gravity. Affects only the BOM and the bench-test program — firmware behaviour is identical in all four cases. Setting `none` plus a non-`tandem`/`closed` spool fails `lifetrac-config self-test`. |
| `hydraulic.valve_settling_ms` | uint, range 0–250, default 100 | `100` | `live` | Settling delay between EFC reaching zero and valve solenoid de-energising. **Set to `0` for `spool_type = float` or `open`** — no delay needed because the centre vents both cylinder lines. The build-time validator forces `0` whenever spool is float/open and warns if non-zero. |

The existing `hydraulic.track_ramp_seconds` / `hydraulic.arm_ramp_seconds` leaves stay as-is and apply to every spool type; ramping the EFC is good practice regardless of centre type because it shapes the operator-felt acceleration profile.

### Three reference builds

| Build profile | `spool_type` | `load_holding` | `valve_settling_ms` | Notes |
|---|---|---|---|---|
| **OSE-legacy / pre-BC-18 interim** | `float` | `po_check` | `0` | The pattern used by every prior LifeTrac generation (v16 through v20). Two PO checks on arm + bucket rod-side lines. EFC bypass at idle wastes a few hundred watts of pump heat — sized into the cooler. Compatible with current firmware as-shipped. |
| **v25 canonical (post-BC-18)** | `tandem` | `spool_inherent` | `100` | This document. Lowest part count, lowest idle heat, cylinder hold for free. Requires BC-18 firmware. |
| **High-performance / racing tractor** | `open` | `counterbalance` | `0` | Tracks free-coast on release for sharper feel; counterbalance valves on cylinders for proper load-lowering control. Pump unloads through P→T centre (no relief-valve heat). Atypical for agricultural use; documented as a third reference point because the schema must support it. |

A fourth build using `closed` centre + accumulator is plausible but not currently planned; the schema permits it because there is no reason to exclude it, but no LifeTrac reference design uses it.

### Firmware branch points

Only **two** code paths actually look at `hydraulic.spool_type`:

1. **`tractor_opta.ino` valve-drive loop** — if `LIFETRAC_HYDRAULIC_SPOOL_TYPE_TANDEM` or `_CLOSED`, gate solenoid de-energise behind the settling timer; if `_FLOAT` or `_OPEN`, drop the solenoid the same tick the EFC ramp output reaches zero (current behaviour).
2. **`lifetrac-config self-test`** — refuse to bundle a config where spool is `tandem`/`closed` and `load_holding == none` (the rare combination of "natural hold from spool, nothing else needed" is exactly `spool_inherent`, never `none`); refuse where spool is `float`/`open` and `load_holding == spool_inherent` (those spools have no inherent hold).

Everything else — the IP-303 ramp ladder, the EFC current trajectory, the E-stop cut path, the relief-valve sizing — is identical across all four spool types. **The cost of supporting multiple hydraulic builds is ~30 lines of firmware + 3 schema leaves + one SIL test matrix expansion**, not a separate firmware build.

### Documentation surface for non-canonical builds

A builder who wants a non-canonical hydraulic stack should:

1. Copy `build.default.toml` → `build.<my-unit>.toml`, edit the three `[hydraulic]` leaves, run `lifetrac-config validate` then `bundle` then `verify` (existing flow).
2. Update their own copy of [HYDRAULIC_BOM.md](HYDRAULIC_BOM.md) §3 spool SKU and (re-introduce or remove) §4 load-holding hardware to match. The BOM stays a per-build artifact even when the firmware is shared.
3. If their spool is `float` or `open` they can ignore [SOFT_STOP_STRATEGY.md](SOFT_STOP_STRATEGY.md) entirely — the sequencer is a no-op for those centres.
4. Run the full SIL matrix (`pytest base_station/tests/`) — the variant-matrix tests under [test_build_config_variant_matrix_sil.py](../DESIGN-CONTROLLER/base_station/tests/test_build_config_variant_matrix_sil.py) will exercise their config alongside the canonical default.

This is the same configurability contract that already covers cameras (`count`, `front_present`, `coral_tpu`), sensors (`imu_model`, `gps_model`), comm (`lora_region`), and aux (`coupler_type`). Hydraulic spool/hold/timing fits the same pattern.

## IMU-adaptive ramp tuning

The canonical ramp constants (`track_ramp_seconds = 2.0`, `arm_ramp_seconds = 1.0`, `valve_settling_ms = 100`) are **starting points**, not optimal values. Optimal values vary with payload mass, ground slope, fluid temperature, pump wear, and operator preference. The BNO086 IMU on the canonical [HARDWARE_BOM.md](../DESIGN-CONTROLLER/HARDWARE_BOM.md) Tier-1 list gives us the data needed to detect when a ramp is too aggressive (jerk spike at start/stop) or too conservative (sluggish operator-felt response without any jerk to justify it). **Yes, we should use the IMU to tune the soft-stop code** — but only with the right safety architecture, because adaptive control of a safety-relevant parameter needs gates that one-shot static tuning does not.

### What "jerk" means here

Jerk is the time derivative of acceleration: `j = d(a)/dt`. Operator-felt smoothness correlates strongly with jerk magnitude, not acceleration magnitude — a sustained 0.5 g acceleration is comfortable, a 0.1 s spike to 0.5 g is harsh. The ISO 2631 comfort limits for whole-body vibration translate roughly to **P95 jerk < 1 g/s = 9.8 m/s³** for an agricultural cab; sport / racing vehicles tolerate higher values. The BNO086 publishes linear acceleration at 100 Hz; computing jerk is one finite difference per axis per sample, well within the M7 budget.

A tractor with the canonical ramp values typically shows:

- **Start transitions:** clean ramp = jerk peak ≤ 0.5 g/s; harsh ramp = > 1.5 g/s.
- **Stop transitions:** clean ramp = jerk peak ≤ 0.8 g/s; harsh stop = > 2.0 g/s. Stops are the primary diagnostic target because that's where the EFC-then-valve-settle sequence does its work.
- **E-stop:** jerk peak whatever the inertia + relief valve allow; **not a target for tuning** — E-stop intentionally bypasses the ramp.

### Three-stage adoption (BC-20)

Tuning a safety-relevant parameter from sensor feedback needs more care than tuning a comfort parameter. Three sub-rounds give us the value of jerk telemetry without the risk of an autonomous writeback running away:

#### BC-20A — Passive jerk telemetry (safe to ship anytime)

M7 computes per-axis jerk from BNO086 linear-acceleration samples and publishes per-transition peak (signed, g/s) over the existing telemetry channel. Web UI diagnostics page surfaces a chart per axis. **No control-loop change.** The point is to make the data visible: an operator can correlate a "feels rough" complaint with an objective number, and the data accumulates per-build over weeks of normal use.

SIL test: synthetic accel time series → expected jerk peak at the right sample. Implementation cost is small; observability value is large.

#### BC-20B — Advisory recommendation engine (no autonomous writes)

Base station accumulates per-build jerk histograms over operator-confirmed clean transitions (transitions where the operator did not E-stop, did not reverse direction mid-ramp, and stayed within the same axis-magnitude band). After enough samples accumulate (target: ~500 transitions ≈ a few hours of operation), the recommendation engine evaluates `track_ramp_seconds`, `arm_ramp_seconds`, and `valve_settling_ms` against the jerk distribution and recommends edits:

- "Track P95 jerk = 1.8 g/s, target ≤ 1.0 g/s. Recommend `track_ramp_seconds 2.0 → 2.4`."
- "Arm P95 jerk = 0.3 g/s with no operator complaints. Recommend no change (room to make ramps faster if operator requests)."
- "Stop transitions show jerk peak at t = settling_start + 50 ms — recommend `valve_settling_ms 100 → 150`."

The operator approves recommendations via the existing `lifetrac-config` flow: `validate / bundle / verify / push`. **No autonomous writes; the human is in the loop.** Recommendations that would push values outside the schema range (e.g. `track_ramp_seconds > 5.0`) are refused at recommendation time, not at validation time, so the operator never sees a "validate fails on the recommendation we just gave you" experience.

SIL test: mock telemetry stream with known statistics → expected recommendation; out-of-range recommendations refused; histogram bucketing across magnitude bands works.

#### BC-20C — Closed-loop online tuning (gated, opt-in, default off)

New schema leaf `hydraulic.adaptive_ramp_tuning` (bool, default `false`, `restart_required`). When enabled, M7 firmware adjusts ramp seconds in **±5 % steps** per accepted clean transition, bounded by the schema range, audit-logged. An operator-facing **panic revert** button restores the original baseline at any time.

Disabled by default for v25 canonical because:

1. Closed-loop control on a safety parameter requires real bench validation that the loop is stable under all payloads / slopes / temperatures (BC-12 HIL hardware dependency).
2. The advisory mode (BC-20B) gets 90 % of the value — most operators find one good ramp setting and never need re-tuning unless something changes.
3. The autonomous-write surface is exactly the kind of thing IEC-62443 + ISO-25119 reviewers want to see disabled by default with explicit operator opt-in.

SIL test: simulated jerk feedback drives ramp adjustments through the ±5 % stepper; out-of-range writes refused; revert-on-panic restores the audit-logged baseline; rate-limit prevents > N adjustments per hour.

### Why not skip straight to BC-20C

Two reasons:

1. **The SIL alone cannot validate the closed loop.** Stability of an adaptive controller on a real machine depends on plant dynamics that the SIL doesn't capture — pump wear over 200 h, oil viscosity at -10 °C vs +40 °C, payload variation across implements. Shipping BC-20C without HIL data would be guessing.
2. **BC-20A passive telemetry has independent value.** Even if BC-20B and BC-20C never ship, the ability to see a jerk number on the diagnostics page changes how operators report problems ("feels rough" → "P95 jerk is 1.8 g/s on the left track"). Shipping it standalone is a clear win.

### Cross-references

- **Decision record:** [DESIGN-CONTROLLER/DECISIONS.md](../DESIGN-CONTROLLER/DECISIONS.md) §D-HYD3.
- **TODO entry:** [TODO.md](../TODO.md) "Open initiative — Hydraulic soft-stop sequencer & build variants (BC-18 / BC-19 / BC-20)".
- **Master test program:** [MASTER_TEST_PROGRAM.md](../MASTER_TEST_PROGRAM.md) §5 BC-20 row (planned).
- **IMU hardware:** [DESIGN-CONTROLLER/HARDWARE_BOM.md](../DESIGN-CONTROLLER/HARDWARE_BOM.md) Tier-1 BNO086 row.
- **M7 telemetry channel:** [firmware/tractor_h7/tractor_h7.ino](../DESIGN-CONTROLLER/firmware/tractor_h7/tractor_h7.ino) — same path the existing IP-303 ramp infrastructure already uses to push state to the base.

## Critical safety properties

1. **EFC must reach zero before valve centres.** Enforced by the firmware sequencer: settling timer starts only after the axis ramp output is at zero for a full tick. If the EFC dashpot is set too slow, the operator may notice mushy stops; the worst-case mechanical outcome is a ~30 ms slug of low-flow fluid hitting a closing port, which the relief valve protects against. **Build-time check:** `lifetrac-config self-test` should warn if `LIFETRAC_HYDRAULIC_*_RAMP_SECONDS` × 1000 < 100 ms (settling delay is meaningful only when the ramp is non-trivial).
2. **E-stop bypasses the sequence by design.** When the latch fires, both EFC and valves drop simultaneously. The pump-side relief valve handles the inertial spike. **This is why the relief valve cannot be removed from the BOM** even though tandem centre means the pump never sees deadhead pressure during normal operation. The relief valve's job is the E-stop kinetic-energy dump, not steady-state pressure limiting.
3. **EFC controller failure is fail-safe.** The Burkert 8605 is current-driven (4–20 mA). Loss of signal = 0 mA = EFC closed = zero flow. Combined with tandem-centre valves at de-energise, the failure mode is "everything stops" — the correct safety bias for an agricultural machine.
4. **EFC dead-band at minimum flow** (~5–10 % of body size) is absorbed by the 100 ms settling delay. Below ~0.5 GPM the EFC compensator can hunt; the settling delay lets the spool seat fully closed before the directional valve commits.

## Implementation work (firmware side)

The existing IP-303 ramp infrastructure in [`firmware/tractor_h7/tractor_h7.ino`](../DESIGN-CONTROLLER/firmware/tractor_h7/tractor_h7.ino) already covers the EFC current ramp (axis → `REG_FLOW_SP_*` → Burkert AO). The new piece is the **valve-centre settling delay** — currently the directional valve solenoid is de-energised the moment the axis goes inactive, which is what tandem centre cannot tolerate.

Concrete additions needed (single round of work, fully SIL-testable):

1. **`tractor_opta.ino`** — in the valve-drive loop, gate solenoid de-energise behind a settling timer that starts when the corresponding `REG_FLOW_SP_*` register reaches zero. Default 100 ms; configurable via a new `LIFETRAC_HYDRAULIC_VALVE_SETTLING_MS` macro in [`common/lifetrac_build_config.h`](../DESIGN-CONTROLLER/firmware/common/lifetrac_build_config.h).
2. **BuildConfig schema** — add `hydraulic.valve_settling_ms` leaf (uint, default 100, range 50–250) to [`tools/lifetrac_config.py`](../DESIGN-CONTROLLER/tools/lifetrac_config.py) so per-unit overrides are deterministic. Treat as `live` change-class — adjustable without restart. Codegen extends the C header.
3. **SIL gate** — new test file `base_station/tests/test_valve_settling_sil.py` exercising: (a) axis goes from active to zero, settling timer starts; (b) EFC reaches zero before settling timer expires; (c) valve solenoid de-energises only after timer expires; (d) E-stop cancels settling timer immediately.
4. **HIL gate (deferred)** — once BC-12 hardware lands, verify on real Burkert + Brand EFC + D1VW that the timing budget holds. Until then the firmware change is gated by SIL only.
5. **MASTER_TEST_PROGRAM.md** — record the new SIL gate.

This work has not been scheduled into a BC-* round yet. Tag it as **BC-18** when prioritised (after BC-15 / BC-16 / BC-17). It is a prerequisite for using D1VW00*8*CNKW in field hardware — the BOM lock-in to spool 8 only takes effect once BC-18 ships.

**Until BC-18 ships,** the existing firmware behaviour (immediate valve de-energise on axis-zero) is incompatible with tandem-centre spools. If a tractor is built before BC-18 lands, populate it with **D1VW00*4*CNKW (float, current pre-pivot canonical) + 2 × PO checks** as the interim hardware bill, and migrate to tandem at the next service interval after BC-18.

## Cross-references

- **BOM impact:** [HYDRAULIC_BOM.md](HYDRAULIC_BOM.md) §3 (directional valves SKU change) and §4 (load-holding section removed). Updated this round.
- **Schematic impact:** [HYDRAULIC_DIAGRAM.md](HYDRAULIC_DIAGRAM.md) — diagram unchanged at the topology level (still 4 valves, manifold, EFC, pump). Spool symbols would update if the diagram showed centre detail; current ASCII art does not.
- **Flow-control config:** [FLOW_VALVE_CONFIGURATION.md](FLOW_VALVE_CONFIGURATION.md) — single vs dual EFC mode is unchanged. The EFC-ramp role is additive; both single and dual modes use it.
- **Firmware reference:** [`DESIGN-CONTROLLER/firmware/tractor_h7/tractor_h7.ino`](../DESIGN-CONTROLLER/firmware/tractor_h7/tractor_h7.ino) — IP-303 axis ramp.
- **Firmware reference:** [`DESIGN-CONTROLLER/firmware/tractor_opta/tractor_opta.ino`](../DESIGN-CONTROLLER/firmware/tractor_opta/tractor_opta.ino) — valve-drive loop (where the settling timer will be added).
- **Build config:** [`DESIGN-CONTROLLER/firmware/common/lifetrac_build_config.h`](../DESIGN-CONTROLLER/firmware/common/lifetrac_build_config.h) — ramp duration macros (existing).
- **Hopsan model:** [`hopsan/README.md`](hopsan/README.md) — future BC-16 `.hmf` templates must model the valve as `Hydraulic43Valve` with **closed-centre A/B** (P→T orifice present, A→T and B→T orifice areas zero) to match tandem behaviour. Update this when BC-16 lands.
- **Architectural decision log:** [`DESIGN-CONTROLLER/DECISIONS.md`](../DESIGN-CONTROLLER/DECISIONS.md) — append a one-line ADR entry pointing back here so the controller-side decision archive captures the firmware/hydraulic boundary.

## Provenance

- Design conversation: chat session 2026-04-29, exchanges 1-7 covering Parker D1VW004CNKW spool decoding, Vanguard 18 HP horizontal-engine pivot, tandem pump architectural exploration, and the EFC-ramp / valve-delay synthesis.
- Industry references: Bobcat T7X (electric skid steer), Cat 299D3 XE (high-end electric drive). Both use proportional pump/motor displacement control combined with valve sequencing for soft start/stop. The LifeTrac approach uses fixed-displacement pump + EFC instead of variable pump, but the soft-stop sequencing principle is the same.
- IP-303 axis-ramp foundation: [`tractor_h7.ino` IP-303 block](../DESIGN-CONTROLLER/firmware/tractor_h7/tractor_h7.ino) — re-derivation from `RESEARCH-CONTROLLER/arduino_opta_controller/lifetrac_v25_controller.ino`.
