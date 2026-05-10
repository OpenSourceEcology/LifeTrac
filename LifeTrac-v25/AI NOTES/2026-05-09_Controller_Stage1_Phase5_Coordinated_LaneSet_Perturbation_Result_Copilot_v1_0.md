# Controller Stage 1 Phase 5 Coordinated Lane-Set Perturbation Result (Copilot v1.0)

Date: 2026-05-09
Board: 2E2C1209DABC240B

## Scope

Use Phase 4 route-detail AF findings to test lane-level coordinated perturbations rather than single-pin forcing.

Lane-set objective:

1. lane A around `PA2/PA3` with companion controls
2. lane B around `PA9/PA10` with companion controls
3. lane C around `PC10/PC11/PC12` with companion controls

Each lane set was executed in two states: all-low and all-high.

## Added Configs

- `DESIGN-CONTROLLER/firmware/x8_lora_bootloader_helper/34_perturb_laneA_low.cfg`
- `DESIGN-CONTROLLER/firmware/x8_lora_bootloader_helper/35_perturb_laneA_high.cfg`
- `DESIGN-CONTROLLER/firmware/x8_lora_bootloader_helper/36_perturb_laneB_low.cfg`
- `DESIGN-CONTROLLER/firmware/x8_lora_bootloader_helper/37_perturb_laneB_high.cfg`
- `DESIGN-CONTROLLER/firmware/x8_lora_bootloader_helper/38_perturb_laneC_low.cfg`
- `DESIGN-CONTROLLER/firmware/x8_lora_bootloader_helper/39_perturb_laneC_high.cfg`

## Lane Composition

- Lane A set: `PA2`, `PA3`, `PA15`, `PB14`, `PB15`
- Lane B set: `PA9`, `PA10`, `PB10`, `PB11`, `PC9`
- Lane C set: `PC10`, `PC11`, `PC12`, `PC9`, `PE11`, `PE12`

## Evidence Folder

- `DESIGN-CONTROLLER/bench-evidence/T6_phase5_lane_sets_2026-05-09_185820/`

## Validation of Perturbation Integrity

All six OpenOCD runs produced pin-level IDR readback logs matching requested state:

- low variants: all listed pins read back `val=0`
- high variants: all listed pins read back `val=1`

This confirms each perturbation was applied electrically at the tested H7 GPIO points.

## Stage 1 Outcome (All Six Cases)

Across `34..39` probe logs:

- `host_rx_bytes` remained `0`
- active query path returned recurring `STATS_URC` content
- `VER_URC` (`0x81` to request `0x01`) never arrived
- each run ended with fatal probe timeout

Representative signature:

- `type=0xC1(STATS_URC) ... host_rx_bytes=0 ...`
- `FATAL: probe failed: timeout waiting for response type 0x81 to req 0x01`

## Conclusion

Phase 5 disproves the next candidate class:

- coordinated static forcing of lane sets A/B/C is still insufficient to restore ingress.

Combined with Phases 3 and 4, this strengthens the root-cause direction toward runtime-owned/peripheral-level gating beyond static GPIO level state.

## Recommended Next Discriminator

Move from static GPIO forcing to runtime route-owner intervention, for example:

1. targeted firmware-side owner override of candidate UART route block
2. live mux-state checkpoint inside runtime just before `VER_REQ` transmit window
3. explicit handshake-policy bypass at firmware layer (not H7 pin level only)
