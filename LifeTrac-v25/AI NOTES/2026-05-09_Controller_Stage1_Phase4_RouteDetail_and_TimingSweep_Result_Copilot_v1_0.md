# Controller Stage 1 Phase 4 Route-Detail and Timing-Sweep Result (Copilot v1.0)

Date: 2026-05-09
Board: 2E2C1209DABC240B

## Scope

Use prior Phase 3 negatives as a hint to pivot from static GPIO forcing to runtime route ownership diagnostics:

1. capture detailed ROM vs user-runtime GPIO mux state (`AFR*`, `PUPDR`, `MODER`, `IDR`)
2. test a short-delay boot-to-probe timing sweep to detect transient ingress enable windows

## Added Configs

- `DESIGN-CONTROLLER/firmware/x8_lora_bootloader_helper/27_snapshot_rom_route_detail.cfg`
- `DESIGN-CONTROLLER/firmware/x8_lora_bootloader_helper/28_snapshot_user_runtime_route_detail.cfg`
- `DESIGN-CONTROLLER/firmware/x8_lora_bootloader_helper/29_boot_user_delay_050ms.cfg`
- `DESIGN-CONTROLLER/firmware/x8_lora_bootloader_helper/30_boot_user_delay_150ms.cfg`
- `DESIGN-CONTROLLER/firmware/x8_lora_bootloader_helper/31_boot_user_delay_400ms.cfg`
- `DESIGN-CONTROLLER/firmware/x8_lora_bootloader_helper/32_boot_user_delay_1000ms.cfg`
- `DESIGN-CONTROLLER/firmware/x8_lora_bootloader_helper/33_boot_user_delay_2000ms.cfg`

## Evidence Folders

- `DESIGN-CONTROLLER/bench-evidence/T6_phase4_route_detail_fix_2026-05-09_185217/`
- `DESIGN-CONTROLLER/bench-evidence/T6_phase4_timing_sweep_2026-05-09_185326/`

## Result 1: Route-Detail Snapshot (ROM vs User Runtime)

Detailed snapshots confirmed that runtime state introduces large AF ownership changes, not just logic-level changes.

High-signal pin transitions observed include:

- `PA02/PA03`: `AF 0 -> 7` (plus mode to AF)
- `PA09/PA10`: `AF 0 -> 7`
- `PA15`: `AF 0 -> 6`
- `PB10/PB11`: `AF 0 -> 7`
- `PC10/PC11/PC12`: `AF 0 -> 6`
- `PC09`: `AF 0 -> 4`
- `PE11..PE14`: `AF 0 -> 5`
- `PF0/PF1`: `AF 0 -> 4`
- `PH13/PH14`: `AF 0 -> 9` with pull changes
- `PI0..PI3`: `AF 0 -> 5`

Interpretation:

- The path is being actively remuxed by runtime firmware across multiple UART-like and route-control-capable nets.
- This supports the prior hypothesis that ingress failure is tied to ownership/mux behavior, not a single static gate pin.

## Result 2: Timing Sweep (50/150/400/1000/2000 ms)

For each delay, sequence was:

1. user boot (`BOOT0 low + NRST pulse`)
2. resume for configured delay
3. shutdown OpenOCD
4. run `method_g_stage1_probe.py`

Observed outcomes:

- `50/150/400 ms` runs: `READY_URC` and `BOOT_URC` observed
- `1000/2000 ms` runs: probe window did not consistently show boot URC
- all five runs: `FATAL timeout waiting for response type 0x81 to req 0x01`
- all five runs: decoded `STATS_URC` still reports `host_rx_bytes=0`

Interpretation:

- No transient ingress-enable window was found in this delay range.
- Startup/output path visibility changes with timing, but ingress remains blocked.

## Consolidated Conclusion

Phase 4 strengthens the current classifier:

1. runtime mux ownership is real and broad (AF-state evidence)
2. ingress is still absent across static pin forcing and short timing-window sweeps

This narrows likely root causes to:

- wrong active ingress lane selection within runtime route map
- missing/incorrect handshake line ownership (not fixed by prior single-pin forcing)
- route dependency outside tested static values that requires coordinated mux+handshake state

## Recommended Next Step

Use the new AF map as the candidate list for lane-level coordinated perturbation:

1. lane-set test A: prioritize `PA2/PA3` + related handshake candidates
2. lane-set test B: prioritize `PA9/PA10` + related handshake candidates
3. lane-set test C: prioritize `PB10/PB11` or `PC10/PC11/PC12` route set

Run each lane set as a coherent mux/handshake state (not single-pin level-only) and re-evaluate `host_rx_bytes` + `VER_URC`.
