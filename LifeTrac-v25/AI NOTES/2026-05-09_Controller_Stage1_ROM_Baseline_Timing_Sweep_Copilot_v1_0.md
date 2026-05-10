# Controller Stage 1 ROM Baseline Timing Sweep (Copilot v1.0)

Date: 2026-05-09
Board: 2E2C1209DABC240B

## Objective

Quantify how ROM ACK probability (`0x79` after `0x7F`) changes with settle delay inside one continuous ROM-hold window.

This isolates timing sensitivity without repeated full setup/teardown between probes.

## Method

Helper used:

- `DESIGN-CONTROLLER/firmware/x8_lora_bootloader_helper/run_rom_baseline_timing_sweep.sh`

Flow:

- start one OpenOCD ROM-hold session with `06_assert_pa11_pf4.cfg`
- sweep settle delays before sync send: `50, 100, 200, 400 ms`
- `5` sync attempts per delay at `19200 8E1`
- classify each response as `ACK`, `NACK`, `SILENT`, `ZERO`, or `OTHER`

## Evidence

- `DESIGN-CONTROLLER/bench-evidence/T6_rom_baseline_timing_2026-05-09_2115/`

Key files:

- `summary.txt`
- `delay_summary.csv`
- `results.csv`
- `openocd_06.txt`

## Results

From `summary.txt`:

- `TOTAL_PROBES=20`
- `ACK_COUNT=1`
- `NACK_COUNT=7`
- `SILENT_COUNT=12`
- `ZERO_COUNT=0`
- `OTHER_COUNT=0`
- `BEST_DELAY_LINE=50,5,1,2,0,2,0`

From `delay_summary.csv`:

- `50ms`: total `5`, `ACK=1`, `NACK=2`, `SILENT=2`
- `100ms`: total `5`, `ACK=0`, `NACK=2`, `SILENT=3`
- `200ms`: total `5`, `ACK=0`, `NACK=3`, `SILENT=2`
- `400ms`: total `5`, `ACK=0`, `NACK=0`, `SILENT=5`

## Interpretation

1. ACK probability in this run was low overall (`1/20`), but non-zero.
2. The only ACK occurred at the shortest tested settle delay (`50ms`).
3. As delay increased, responses shifted away from ACK and toward NACK/silent, with `400ms` fully silent.
4. No `0x00` class appeared in this sweep, unlike earlier noisy reruns, suggesting this path mainly exhibits NACK/silent instability in this configuration.

## Conclusion

ROM baseline entry appears most viable near short settle timings (`<=100ms`) under a single hold window. Longer waits did not improve ACK yield and may reduce it.

## Next Move

Use this as the next hardening axis:

- keep settle delay around `50ms` (or narrow sweep around it)
- increase attempts per hold window (multi-sync retry burst)
- track ACK/NACK/silent ratios per burst and only proceed to A/B compare when burst-level ACK threshold is met
