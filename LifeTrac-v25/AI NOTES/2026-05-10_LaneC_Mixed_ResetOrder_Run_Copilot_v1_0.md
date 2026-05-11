# 2026-05-10 Lane-C Mixed-State + Reset-Order Run (Copilot v1.0)

## Why this run

This run executed the prepared lane-C one-hot reset-order matrix to check whether lane-C controls provide a more deterministic ingress discriminator than lane-B.

## Evidence folder

- `DESIGN-CONTROLLER/bench-evidence/T6_laneC_mixed_resetorder_2026-05-10_190458/`

## Config set executed

- `50_mix_laneC_pc10_high_pre_reset.cfg`
- `51_mix_laneC_pc11_high_pre_reset.cfg`
- `52_mix_laneC_pc12_high_pre_reset.cfg`
- `53_mix_laneC_pc9_high_pre_reset.cfg`
- `54_mix_laneC_pe11_high_pre_reset.cfg`
- `55_mix_laneC_pe12_high_pre_reset.cfg`

## Validation of forcing

OpenOCD IDR readbacks match intended one-hot state in every case:
- `50`: `PC10=1`, others in lane-C set `0`
- `51`: `PC11=1`, others `0`
- `52`: `PC12=1`, others `0`
- `53`: `PC9=1`, others `0`
- `54`: `PE11=1`, others `0`
- `55`: `PE12=1`, others `0`

So the lane-C reset-order forcing operation was applied correctly at the H747 side.

## Probe outcome summary

Across all six cases:
- `BOOT_URC`: not observed in initial window.
- Binary protocol: `VER_REQ` times out waiting for `VER_URC`.
- ASCII `AT+VER?`: no response.

ATI response split:
- `50/51/52`: non-empty zero-heavy ATI payloads (~141-143 bytes).
- `53/54/55`: ATI silence (`no bytes observed`).

This gives a stronger behavioral split than previous lane-B mixed-state runs, but still without protocol recovery.

## Interpretation

What this run established:
- Lane-C state materially changes observed user-mode serial behavior under controlled reset order.
- The `PC10/PC11/PC12` branch and the `PC9/PE11/PE12` branch show distinct ATI signatures.

What is still unresolved:
- No state recovered valid host command ingress (`VER_URC` absent in all cases).
- No response to `AT+VER?` in any lane-C one-hot state.

## Updated hypothesis

Most likely now:
- Ingress gating is a multi-net condition that includes lane-C and likely a cross-lane dependency (especially around `PC9`), rather than a single static selector bit.

## Recommended next step

Run a small cross-lane interaction matrix (lane-B + lane-C) centered on the newly separated signature families:
1. Hold `PC10/PC11/PC12` one-hot states while toggling `PB11` and `PC9` pairings.
2. Hold `PE11/PE12` one-hot states while toggling `PA9/PA10` pairings.
3. Repeat only divergent cases (ATI-present vs ATI-silent), and keep the same pre-reset forcing order.
