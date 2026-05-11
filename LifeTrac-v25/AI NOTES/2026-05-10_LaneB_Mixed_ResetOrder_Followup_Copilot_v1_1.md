# 2026-05-10 Lane-B Mixed-State + Reset-Order Follow-up (Copilot v1.1)

## Why this run

This follow-up executed the previously proposed next step:
- lane-B mixed-state bit-flip matrix, and
- reset-order control (state asserted before NRST release)

to refine the board-level ingress-gating hypothesis after the all-high/all-low sweep.

## Evidence folder

- `DESIGN-CONTROLLER/bench-evidence/T6_laneB_mixed_resetorder_2026-05-10_185139/`

## Config set executed

Primary matrix (8 cases):
- `40_mix_laneB_pa9_high_pre_reset.cfg`
- `41_mix_laneB_pa10_high_pre_reset.cfg`
- `42_mix_laneB_pb10_high_pre_reset.cfg`
- `43_mix_laneB_pb11_high_pre_reset.cfg`
- `44_mix_laneB_pc9_high_pre_reset.cfg`
- `45_mix_laneB_pa9_low_others_high_pre_reset.cfg`
- `46_mix_laneB_pb10_low_others_high_pre_reset.cfg`
- `47_mix_laneB_pc9_low_others_high_pre_reset.cfg`

Targeted confirmation (2 cases):
- `48_mix_laneB_pb11_pc9_high_pre_reset.cfg`
- `49_mix_laneB_pb11_pc9_low_pre_reset.cfg`

Stability repeat:
- `43_mix_laneB_pb11_high_pre_reset.r2.*`
- `44_mix_laneB_pc9_high_pre_reset.r2.*`

## Validation of forcing

OpenOCD IDR readbacks match intended low/high states for every case, so mixed-state forcing and reset-order application were successful.

## Probe outcome summary

Across all cases:
- `VER_REQ` still times out waiting for `VER_URC`.
- `AT+VER?` remains no-response.

ATI behavior:
- Most cases: zero-heavy ATI payload present (~141-144 bytes).
- One early pass showed no ATI bytes for PB11-only-high and PC9-only-high, but these did not reproduce on immediate repeat (`r2` logs show ATI bytes again).

Additional observation:
- Several runs show `pre-drain: 1 byte (0x00)` before active queries, indicating idle delimiter-like residue but not valid framed ingress.

## Interpretation

What this run strengthened:
- Lane-B mixed-state perturbations and reset-order timing do influence observed serial behavior.
- The path is not simply fixed-open/fixed-closed; behavior remains stateful and noisy.

What this run did not prove:
- No deterministic single-pin gate in the tested lane-B subset.
- No recovered valid ingress path (`VER_URC` still absent in all states).

## Updated hypothesis quality

Likely:
- Multi-signal interaction and/or additional untested control net outside lane-B.

Still possible:
- Timing-window sensitivity in firmware/user-mode bring-up sequence beyond this reset-order method.

Less likely now:
- A single, static one-bit lane-B selector being the sole root cause.

## Recommended immediate next experiment

Move to lane-C mixed-state matrix with the same reset-order method, but bias toward reduced factorial search:
1. one-hot high / one-hot low around candidate pairings,
2. include `PC9` shared interaction points explicitly, and
3. repeat only cases that show materially different ATI/pre-drain signatures.

In parallel, continue schematic extraction to replace heuristic lane groups with exact ABX00043 net names and ownership.
