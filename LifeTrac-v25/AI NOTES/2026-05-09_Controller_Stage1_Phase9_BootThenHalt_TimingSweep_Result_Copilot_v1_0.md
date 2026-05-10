# Controller Stage 1 Phase 9 Boot-Then-Halt Timing Sweep Result (Copilot v1.0)

Date: 2026-05-09
Board: 2E2C1209DABC240B

## Scope

Test whether a short runtime-owner timing window exists immediately after user boot by halting H7 at multiple early delays before running the Stage 1 probe.

Hypothesis:

- if runtime ownership logic quickly closes ingress, then halting very early (0..20 ms) may preserve a probe-successful route state.

## Added Configs

- `DESIGN-CONTROLLER/firmware/x8_lora_bootloader_helper/47_phase9_boot_then_halt_000ms.cfg`
- `DESIGN-CONTROLLER/firmware/x8_lora_bootloader_helper/48_phase9_boot_then_halt_020ms.cfg`
- `DESIGN-CONTROLLER/firmware/x8_lora_bootloader_helper/49_phase9_boot_then_halt_100ms.cfg`
- `DESIGN-CONTROLLER/firmware/x8_lora_bootloader_helper/50_phase9_boot_then_halt_400ms.cfg`

## Test Sequence

Each config performs:

1. BOOT0 low + NRST pulse (user app path)
2. `resume`
3. timed delay (`0 / 20 / 100 / 400 ms`)
4. `halt`
5. shutdown OpenOCD
6. run `method_g_stage1_probe.py --dev /dev/ttymxc3 --baud 921600`

## Evidence Folder

- `DESIGN-CONTROLLER/bench-evidence/T6_phase9_halt_timing_2026-05-09_191923/`

## Integrity Checks

All four OpenOCD logs show valid attach and halt transitions; no Tcl/runtime errors observed.

## Stage 1 Outcome (All Four Cases)

For `47..50` probe logs:

- `BOOT_URC` observed
- `STATS_URC` responses present with `host_rx_bytes=0`
- `VER_URC` never observed
- final state: `FATAL timeout waiting for response type 0x81 to req 0x01`

Consistent fault telemetry observed:

- `FAULT_URC code=0x03` (radio init fail)
- `FAULT_URC code=0x08` (clock HSE failed)

No ingress-progress indicators appeared.

## Conclusion

Phase 9 is negative across all early-halt timings.

Interpretation:

- no exploitable owner-domain timing window was found in the 0..400 ms post-boot range,
- ingress remains absent before parser-level processing.

## Recommended Next Discriminator

Prioritize pre-parser transport path verification at the source domain boundary (X8 UART4 TX activity and routing owner), with success gate defined as first non-zero `host_rx_bytes`.
