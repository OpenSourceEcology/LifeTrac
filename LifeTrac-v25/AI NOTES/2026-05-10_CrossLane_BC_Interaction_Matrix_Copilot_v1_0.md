# 2026-05-10 Cross-Lane B+C Interaction Matrix (Copilot v1.0)

## Why this run

This run continued the next-step plan after lane-C one-hot testing:
- test reduced cross-lane B+C pairings centered on the `PC9` interaction hypothesis,
- keep the same pre-reset forcing order,
- repeat divergent signatures to check reproducibility.

## Evidence folder

- `DESIGN-CONTROLLER/bench-evidence/T6_crosslane_BC_2026-05-10_190847/`

## Config set executed

Primary matrix (`56..63`):
- `56_cross_pc10_pb11pc9_low_pre_reset.cfg`
- `57_cross_pc10_pb11_high_pc9_low_pre_reset.cfg`
- `58_cross_pc10_pb11_low_pc9_high_pre_reset.cfg`
- `59_cross_pc10_pb11pc9_high_pre_reset.cfg`
- `60_cross_pe11_pa9pa10_low_pre_reset.cfg`
- `61_cross_pe11_pa9_high_pa10_low_pre_reset.cfg`
- `62_cross_pe11_pa9_low_pa10_high_pre_reset.cfg`
- `63_cross_pe11_pa9pa10_high_pre_reset.cfg`

Selective repeats (`r2`):
- `56_cross_pc10_pb11pc9_low_pre_reset.r2.*`
- `59_cross_pc10_pb11pc9_high_pre_reset.r2.*`
- `60_cross_pe11_pa9pa10_low_pre_reset.r2.*`

## Validation of forcing

OpenOCD readback lines confirm each intended cross-lane state was asserted correctly in every case. Example classes:
- `56`: `C10=1`, `B11=0`, `C9=0`
- `59`: `C10=1`, `B11=1`, `C9=1`
- `60`: `E11=1`, `A9=0`, `A10=0`
- `63`: `E11=1`, `A9=1`, `A10=1`

## Probe outcome summary

Across all matrix and repeat cases:
- `VER_REQ` still times out waiting for `VER_URC`.
- `AT+VER?` remains no-response.

ATI behavior:
- First pass showed mixed signatures:
  - `59` and `60` had ATI silence (`no bytes observed`).
  - others mostly showed zero-heavy ATI payloads (~141-144 bytes).
- Repeat runs did not hold the silent signature:
  - `59.r2` and `60.r2` both returned ATI payloads again.
  - `56.r2` stayed in ATI-present class.

## Interpretation

What improved:
- Cross-lane state space around `PC9`/`PB11` and `PE11`/`PA9`/`PA10` was tested with controlled reset ordering and forcing verification.

What remains true:
- No deterministic state recovered valid ingress (`VER_URC` absent in all cases).
- The ATI-silent condition is currently non-deterministic under this matrix and reverts on immediate repeat.

## Updated hypothesis

Most likely now:
- The ingress blocker is still board-state coupled, but not explained by a single static cross-lane tuple in this reduced matrix.
- There is likely an additional timing-dependent or ownership-dependent control interaction outside this tested subset.

## Recommended next move

1. Add a timed hold-window sweep on the best-perturbing tuples (asserted for different intervals before and after NRST release).
2. Add one owner-policy perturbation dimension (where feasible) while holding one cross-lane tuple fixed.
3. Continue ABX00043 schematic extraction to map exact owner and gate nets, then shrink experiments to only those evidence-backed control lines.
