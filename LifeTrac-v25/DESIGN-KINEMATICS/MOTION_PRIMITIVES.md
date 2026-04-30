# MOTION_PRIMITIVES

**Status.** LANDED Round 47. The 12 primitives MP-01..MP-12 below are
the canonical naming surface used in test docstrings and roadmap items.

Landed BC rounds touching these primitives:

- **MP-03 / MP-04 / MP-05** (skid / pivot / spin turns): BC-21 mix-then-ramp
  (Round 44), BC-23 preserve-steering on saturation (Round 46), and BC-24
  spin-turn flow boost (Round 46) all refine these.
- **MP-01 / MP-02** (drive forward / reverse) and any direction reversal
  between them: BC-22 reversal brake (Round 45).
- **MP-06..MP-09** (arms raise / lower, bucket curl / dump): mechanical
  pipeline shared with tracks; ramp profiles in RAMP_PROFILES.md.
- **MP-10 / MP-11** (combined drive + arms / turn + bucket): flow
  contention modelled in FLOW_BUDGETING.md (max-magnitude formula on
  single-EFC builds; structural separation on dual-EFC).
- **MP-12** (float): spool-type-gated, BC-19; not exercised by current
  SIL fixtures.

## Scope

Enumerate the **atomic operator-intelligible motions** the LifeTrac can perform.
A motion primitive is the smallest unit a human operator names ("spin in place",
"curl bucket", "creep forward"). Higher-level intents (e.g. "scoop a load") are
**sequences** of primitives and live in operator training docs, not here.

For each primitive, this doc will eventually capture:

- Required logical-axis activation (e.g. spin-turn = `leftTrack = +X, rightTrack = -X, arms=0, bucket=0`)
- Required hydraulic capability (e.g. spin-turn requires both track sides to have
  independent flow; constrained on single-EFC builds — see [FLOW_BUDGETING.md](FLOW_BUDGETING.md))
- Expected jerk / acceleration profile under default ramps
- Build-variant constraints (which builds support, which degrade gracefully, which forbid)

## Provisional primitive list

The following 12 primitives are the working set. Numbers are stable handles for
cross-referencing in test programs and SIL fixtures.

| # | Primitive | Logical-axis pattern | Notes |
|---|---|---|---|
| MP-01 | Drive forward | `leftTrack = rightTrack = +Y` | Baseline; flow demand = max(|track|) |
| MP-02 | Drive reverse | `leftTrack = rightTrack = -Y` | Mirror of MP-01 |
| MP-03 | Skid-turn (gentle, both tracks forward) | `leftTrack = +Y+X, rightTrack = +Y-X`, both > 0 | Standard while-driving turn |
| MP-04 | Pivot turn (one track stopped) | one track at 0, other at ±Y | Edge of MP-03 |
| MP-05 | Spin in place (counter-rotation) | `leftTrack = +X, rightTrack = -X` | High flow demand; see BC-24 |
| MP-06 | Arms raise | `arms = +1` | Bucket geometry constrains ground clearance |
| MP-07 | Arms lower | `arms = -1` | |
| MP-08 | Bucket curl | `bucket = +1` | |
| MP-09 | Bucket dump | `bucket = -1` | |
| MP-10 | Combined drive + arms (raise while moving) | `Y ≠ 0, arms ≠ 0` | Flow contention on single-EFC |
| MP-11 | Combined turn + bucket (curl while turning) | `X ≠ 0, bucket ≠ 0` | Flow contention on single-EFC |
| MP-12 | Float (arms or bucket free-floating) | depends on `hydraulic.spool_type = float` | Build-gated; see BC-19 |

## Build-variant compatibility table

Will live in [BUILD_VARIANT_MOTION_MATRIX.md](BUILD_VARIANT_MOTION_MATRIX.md) once
that doc is filled in.

## Pending (post-Round 47)

- [ ] Lock primitive numbering by adding MP-01..MP-12 cross-references
      to MASTER_TEST_PROGRAM.md test rows where they apply
- [ ] Document quantitative flow demand per primitive at full-stick
      (ties into FLOW_BUDGETING.md once bench-up data exists)
- [ ] Add diagram (top-down arrows showing track motion) once mechanical
      package is bench-photographed
- [ ] Promote BUILD_VARIANT_MOTION_MATRIX.md once at least 2 build
      variants are formally captured
