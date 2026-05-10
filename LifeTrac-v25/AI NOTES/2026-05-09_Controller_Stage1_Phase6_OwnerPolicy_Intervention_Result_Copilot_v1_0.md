# Controller Stage 1 Phase 6 Owner/Policy Intervention Result (Copilot v1.0)

Date: 2026-05-09
Board: 2E2C1209DABC240B

## Scope

Advance from static lane-level output forcing to owner/policy interventions:

1. freeze runtime owner state (halt after takeover)
2. inject lane AF policy in halted state (lane B and lane C)
3. inject mixed lane policy with AF + pull-up/input semantics (lane A)

## Added Configs

- `DESIGN-CONTROLLER/firmware/x8_lora_bootloader_helper/40_phase6_owner_freeze_runtime.cfg`
- `DESIGN-CONTROLLER/firmware/x8_lora_bootloader_helper/41_phase6_owner_freeze_laneB_af.cfg`
- `DESIGN-CONTROLLER/firmware/x8_lora_bootloader_helper/42_phase6_owner_freeze_laneA_policy.cfg`
- `DESIGN-CONTROLLER/firmware/x8_lora_bootloader_helper/43_phase6_owner_freeze_laneC_af.cfg`

## Evidence Folder

- `DESIGN-CONTROLLER/bench-evidence/T6_phase6_owner_policy_2026-05-09_190059/`

## Integrity Checks

OpenOCD logs confirm expected Phase C actions executed:

- owner freeze case logged `halt/freeze owner state`
- lane-B case logged AF writes:
  - `PA9/PA10 -> AF7`
  - `PB10/PB11 -> AF7`
  - `PC9 -> AF4`
- lane-A case logged:
  - `PA2/PA3 -> AF7`, `PA15 -> AF6`
  - `PB14/PB15 -> input + pull-up`
- lane-C case logged:
  - `PC10/PC11/PC12 -> AF6`, `PC9 -> AF4`
  - `PE11/PE12 -> AF5`

This verifies the intended register-policy interventions were actually applied.

## Stage 1 Outcome

All four probe logs (`40..43`) showed the same failure signature:

- `STATS_URC(init)` reports `host_rx_bytes=0`
- active query path continues to return `STATS_URC` frames
- no `VER_URC` (`0x81`) response to request `0x01`
- final state: `FATAL timeout waiting for response type 0x81 to req 0x01`

## Conclusion

Phase 6 result is negative across all owner/policy interventions tested.

Interpretation:

- static halted-state ownership freeze is insufficient
- static halted-state AF/pull image injection is insufficient

Likely blocker now sits in live runtime peripheral/transaction path (active owner loop and/or handshake gating) that is not captured by post-halt register snapshots alone.

## Recommended Next Discriminator (Phase 7)

Move to live runtime intervention rather than halted post-state injection:

1. instrument runtime to expose selected UART-route owner state at request boundary
2. force runtime branch/policy selection in firmware (not via post-halt GPIO write)
3. re-run Stage 1 and look for first non-zero `host_rx_bytes` delta as success criterion
