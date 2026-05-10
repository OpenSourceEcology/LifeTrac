# Controller Stage 1 Phase 2 GPIO Snapshot Diff Result (Copilot v1.0)

Date: 2026-05-09
Board: 2E2C1209DABC240B
Evidence folder: `DESIGN-CONTROLLER/bench-evidence/T6_phase1_owner_ab_2026-05-09_183718/`

## Goal

Execute Phase 2 from the master plan: capture comparable GPIO state in ROM-working and user-failing conditions, then identify control-net candidates for Phase 3 perturbation.

## Added Helper Configs

- `DESIGN-CONTROLLER/firmware/x8_lora_bootloader_helper/10_snapshot_rom_state.cfg`
  - BOOT0 high + NRST pulse, dump all GPIO banks, shutdown.
- `DESIGN-CONTROLLER/firmware/x8_lora_bootloader_helper/11_snapshot_user_state.cfg`
  - BOOT0 low + NRST pulse, dump all GPIO banks, shutdown.
- `DESIGN-CONTROLLER/firmware/x8_lora_bootloader_helper/12_snapshot_user_runtime_state.cfg`
  - BOOT0 low + NRST pulse, resume ~1.2 s, halt, dump all GPIO banks, shutdown.

## Captures

- `phase2_snapshot_rom_state_allbanks.txt`
- `phase2_snapshot_user_state_allbanks.txt`
- `phase2_snapshot_user_runtime_allbanks.txt`

## Key Results

1. Halted-state comparison (ROM vs user selection only) is non-discriminating.
   - ROM snapshot and user snapshot differ only at `GPIOA IDR` (`0xc800` vs `0xc000`), which is consistent with forced `PA_11` BOOT0 level.
   - No additional route-control signal appears in this halted-only method.

2. Runtime user snapshot is strongly discriminating.
   - After allowing user firmware runtime before halt, multiple banks changed versus ROM baseline:
     - `GPIOA`: `MODER 0xab7fffff -> 0xaaaaaaaf`, `IDR 0xc800 -> 0xc704`
     - `GPIOB`: `MODER 0xfffffebf -> 0x0fabeebf`, `IDR 0x0010 -> 0xc410`
     - `GPIOC`: `MODER 0xffffffff -> 0xfeabffff`, `IDR 0x0000 -> 0x1200`
     - `GPIOE`: `MODER 0xffffffff -> 0xeabfffff`, `IDR 0x0000 -> 0x0800`
     - `GPIOF`: `MODER 0xfffffdff -> 0xfffffdfa`, `IDR 0x0010 -> 0x0010` (mode change without level change)
     - `GPIOH`: `MODER 0xffffffff -> 0xebffffff`, `IDR 0x0000 -> 0x0000`
     - `GPIOI`: `MODER 0xffffffff -> 0xffffffaa`, `IDR 0x0000 -> 0x0000`

## Interpretation

- Phase 2 now provides actionable leads: ingress failure correlates with runtime H7 pin ownership/configuration changes, not only BOOT0/reset state.
- The next discriminating step should target pins that changed level in runtime state first, then mode-only changes.

## Phase 3 Candidate Priority (initial)

1. Level-changing candidates (highest signal):
   - Bank A: bits changed in `IDR` from `0xc800` to `0xc704`
   - Bank B: bits changed in `IDR` from `0x0010` to `0xc410`
   - Bank C: bits changed in `IDR` from `0x0000` to `0x1200`
   - Bank E: bits changed in `IDR` from `0x0000` to `0x0800`

2. Mode-only candidates (second pass):
   - Bank F, H, I MODER transitions where IDR stayed constant.

## Recommended Immediate Run Order

1. Start with one-pin perturbation on the highest-confidence level-changing candidates.
2. After each perturbation, run both:
   - `diag_uart_rxtx.py --dev /dev/ttymxc3 --baud 921600 --hwflow off`
   - `method_g_stage1_probe.py --dev /dev/ttymxc3 --baud 921600`
3. Stop and classify immediately if any run yields `VER_URC` or deterministic ASCII reply.
