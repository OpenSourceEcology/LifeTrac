# Controller Stage 1 Phase 3 PC9 and Cluster Perturbation Result (Copilot v1.0)

Date: 2026-05-09
Board: 2E2C1209DABC240B

## Scope

Continue Phase 3 narrowing after negative single-pin results on `PE11`, `PB14`, `PB15`, `PB10`, `PC12`.

Two targets:

1. remaining changed-level single candidate `PC9`
2. multi-pin interaction hypothesis across the full changed cluster (`PB10/PB14/PB15/PC9/PC12/PE11`)

## Added Configs

- `DESIGN-CONTROLLER/firmware/x8_lora_bootloader_helper/23_perturb_pc9_low.cfg`
- `DESIGN-CONTROLLER/firmware/x8_lora_bootloader_helper/24_perturb_pc9_high.cfg`
- `DESIGN-CONTROLLER/firmware/x8_lora_bootloader_helper/25_perturb_cluster_low.cfg`
- `DESIGN-CONTROLLER/firmware/x8_lora_bootloader_helper/26_perturb_cluster_high.cfg`

## Evidence Folders

- `DESIGN-CONTROLLER/bench-evidence/T6_phase3_pc9_ab_2026-05-09_184622/`
- `DESIGN-CONTROLLER/bench-evidence/T6_phase3_cluster_ab_2026-05-09_184659/`
- `DESIGN-CONTROLLER/bench-evidence/T6_phase3_cluster_ab_verifyfix_2026-05-09_184754/`

## Result 1: PC9 Low/High A/B

OpenOCD confirms forcing:

- low run: `GPIOC_IDR.PC_9 = 0`
- high run: `GPIOC_IDR.PC_9 = 1`

Stage 1 probe result is unchanged in both runs:

- `STATS_URC(init): host_rx_bytes=0 host_rx_lpuart=0 host_rx_usart1=0 ...`
- no `BOOT_URC` during initial window
- `ATI` / `AT+VER?` return recurring `STATS_URC` frame payload
- timeout waiting for `VER_URC` (`0x81`) to `VER_REQ` (`0x01`)

Interpretation: `PC9` is not a sufficient single-pin ingress gate.

## Result 2: Cluster Low/High A/B

Cluster forced together:

- `PB10`, `PB14`, `PB15`, `PC9`, `PC12`, `PE11`

Initial run:

- `T6_phase3_cluster_ab_2026-05-09_184659`
- Stage 1 remained negative in both low/high cases.
- Per-pin readback was not printed in that first script revision.

Verification rerun (with explicit IDR prints in script):

- `T6_phase3_cluster_ab_verifyfix_2026-05-09_184754`
- low case readback:
  - `PB10=0 PB14=0 PB15=0 PC9=0 PC12=0 PE11=0`
- high case readback:
  - `PB10=1 PB14=1 PB15=1 PC9=1 PC12=1 PE11=1`
- Stage 1 probe in both verified cases still shows:
  - `host_rx_bytes=0`
  - `ATI`/`AT+VER?` return `STATS_URC`
  - timeout waiting for `VER_URC`

Interpretation: this candidate GPIO cluster state is not sufficient to restore ingress.

## Consolidated Interpretation

Phase 3 now has negative evidence for:

- single-pin: `PE11`, `PB14`, `PB15`, `PB10`, `PC12`, `PC9`
- cluster interaction: `{PB10, PB14, PB15, PC9, PC12, PE11}` forced all-low / all-high

Across all cases, the stable signature is unchanged: firmware TX path alive (`STATS_URC`), ingress counters remain zero.

This lowers probability that ingress failure is caused by one of these GPIO level states alone and increases probability of:

1. non-GPIO ownership/routing gate (alternate mux path or peripheral ownership interaction)
2. handshake/timing gate outside static pin level forcing
3. board-level route dependency not captured by these pins

## Recommended Next Step

Move from GPIO level perturbation to ownership/path diagnostics:

1. capture/perturb UART peripheral ownership and mux registers around active query window
2. test controlled, time-windowed handoff patterns (resume/halt offsets) to detect transient ingress-enable state
3. correlate with Max Carrier route documents to identify non-tested control nets
