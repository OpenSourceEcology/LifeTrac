# Controller Stage 1 Phase 3 Perturbation Batch Result (Copilot v1.1)

Date: 2026-05-09
Board: 2E2C1209DABC240B

## Scope

Consolidated results for single-pin perturbation A/B runs executed after PE11, targeting additional level-changing runtime candidates.

## Configs Added

- `DESIGN-CONTROLLER/firmware/x8_lora_bootloader_helper/15_perturb_pb14_low.cfg`
- `DESIGN-CONTROLLER/firmware/x8_lora_bootloader_helper/16_perturb_pb14_high.cfg`
- `DESIGN-CONTROLLER/firmware/x8_lora_bootloader_helper/17_perturb_pb15_low.cfg`
- `DESIGN-CONTROLLER/firmware/x8_lora_bootloader_helper/18_perturb_pb15_high.cfg`
- `DESIGN-CONTROLLER/firmware/x8_lora_bootloader_helper/19_perturb_pb10_low.cfg`
- `DESIGN-CONTROLLER/firmware/x8_lora_bootloader_helper/20_perturb_pb10_high.cfg`
- `DESIGN-CONTROLLER/firmware/x8_lora_bootloader_helper/21_perturb_pc12_low.cfg`
- `DESIGN-CONTROLLER/firmware/x8_lora_bootloader_helper/22_perturb_pc12_high.cfg`

## Evidence Folders

- `DESIGN-CONTROLLER/bench-evidence/T6_phase3_pb14_ab_2026-05-09_184402/`
- `DESIGN-CONTROLLER/bench-evidence/T6_phase3_pb15_ab_2026-05-09_184430/`
- `DESIGN-CONTROLLER/bench-evidence/T6_phase3_pb10_ab_2026-05-09_184502/`
- `DESIGN-CONTROLLER/bench-evidence/T6_phase3_pc12_ab_2026-05-09_184532/`

## Per-Candidate Outcome

1. PB14 low/high
- OpenOCD confirmed `GPIOB_IDR.PB_14 = 0/1`.
- Stage 1 unchanged: `host_rx_bytes=0`, no `VER_URC`, timeout persists.

2. PB15 low/high
- OpenOCD confirmed `GPIOB_IDR.PB_15 = 0/1`.
- Stage 1 unchanged: `host_rx_bytes=0`, no `VER_URC`, timeout persists.

3. PB10 low/high
- OpenOCD confirmed `GPIOB_IDR.PB_10 = 0/1`.
- Stage 1 unchanged: `host_rx_bytes=0`, no `VER_URC`, timeout persists.

4. PC12 low/high
- OpenOCD confirmed `GPIOC_IDR.PC_12 = 0/1`.
- Stage 1 unchanged: `host_rx_bytes=0`, no `VER_URC`, timeout persists.

## Cross-Run Signature

Across all runs, active query windows consistently show:

- `STATS_URC(init): host_rx_bytes=0 host_rx_lpuart=0 host_rx_usart1=0 ...`
- `ATI`/`AT+VER?` returning recurring `STATS_URC` frame content only
- fatal timeout waiting for `VER_URC` (`0x81`) to `VER_REQ` (`0x01`)

This indicates user firmware TX path is alive but ingress bytes are still not reaching L072 RX ISR.

## Conclusion

`PB14`, `PB15`, `PB10`, and `PC12` are not sufficient single-pin ingress gates on this board.

## Next Narrowing Step

Proceed with remaining changed-level candidates in Phase 3:

1. `PC9` (other changed bit in GPIOC IDR)
2. selected `GPIOA` runtime-changed bits (excluding already-characterized BOOT/reset behavior)
