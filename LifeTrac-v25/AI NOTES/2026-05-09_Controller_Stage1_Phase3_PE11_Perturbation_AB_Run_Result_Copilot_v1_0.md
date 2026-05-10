# Controller Stage 1 Phase 3 PE11 Perturbation A/B Run Result (Copilot v1.0)

Date: 2026-05-09
Board: 2E2C1209DABC240B
Evidence folder: `DESIGN-CONTROLLER/bench-evidence/T6_phase3_pe11_ab_2026-05-09_184225/`

## Objective

Run first single-pin perturbation from Phase 2 runtime-diff candidates and determine whether forcing `PE_11` changes host->L072 ingress behavior.

## Method

Added and used:

- `DESIGN-CONTROLLER/firmware/x8_lora_bootloader_helper/13_perturb_pe11_low.cfg`
- `DESIGN-CONTROLLER/firmware/x8_lora_bootloader_helper/14_perturb_pe11_high.cfg`

Each config performs:

1. user boot select (`PA_11` low + `PF_4` reset pulse)
2. runtime dwell (`resume` ~1.2 s)
3. halt and force `PE_11` to target level
4. resume and exit

After each perturbation:

- `diag_uart_rxtx.py --dev /dev/ttymxc3 --baud 921600 --hwflow off`
- `method_g_stage1_probe.py --dev /dev/ttymxc3 --baud 921600`

## Results

1. Pin forcing was successful.

- low case: `GPIOE_IDR.PE_11 = 0`
- high case: `GPIOE_IDR.PE_11 = 1`

2. Stage 1 probe outcome was unchanged in both cases.

- no `BOOT_URC` in initial window
- ASCII fallback empty (`ATI` and `AT+VER?` no bytes)
- timeout waiting for `VER_URC` (`0x81`) to `VER_REQ` (`0x01`)

3. Diag behavior was effectively unchanged between low/high.

- passive window and active query windows showed the same repeating outbound binary frame pattern
- no ingress recovery signature observed

## Conclusion

`PE_11` is not a sufficient single-pin ingress gate for resolving Method G Stage 1 on this board.

## Next Candidate Direction

Continue Phase 3 perturbations with other level-changing runtime candidates from Phase 2 first:

1. `GPIOB` changed-level bits (`IDR 0x0010 -> 0xc410`)
2. `GPIOC` changed-level bits (`IDR 0x0000 -> 0x1200`)
3. `GPIOA` changed-level bits (`IDR 0xc800 -> 0xc704`, excluding already-tested BOOT path assumptions)
