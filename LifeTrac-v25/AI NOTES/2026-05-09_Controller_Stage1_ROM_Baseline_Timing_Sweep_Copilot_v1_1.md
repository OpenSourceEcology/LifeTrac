# Controller Stage 1 ROM Baseline Timing Sweep (Copilot v1.1)

Date: 2026-05-09
Board: 2E2C1209DABC240B

## Objective

Run a higher-attempt short-delay sweep to test whether ACK yield improves when concentrating attempts in the best-performing timing band identified by v1.0.

## Method

Helper used:

- `DESIGN-CONTROLLER/firmware/x8_lora_bootloader_helper/run_rom_baseline_timing_sweep.sh`

Invocation profile:

- `DELAYS_MS=30,50,75,100`
- `ATTEMPTS_PER_DELAY=20`
- one continuous ROM-hold session via `06_assert_pa11_pf4.cfg`
- classifier buckets: `ACK`, `NACK`, `SILENT`, `ZERO`, `OTHER`

Note: helper was updated to accept comma-separated delay lists to avoid sudo/env quoting failures under the host invocation path.

## Evidence

- `DESIGN-CONTROLLER/bench-evidence/T6_rom_baseline_timing_80probe_2026-05-09_081721/`

Key files:

- `summary.txt`
- `delay_summary.csv`
- `results.csv`
- `openocd_06.txt`

## Results

From `summary.txt`:

- `TOTAL_PROBES=80`
- `ACK_COUNT=1`
- `NACK_COUNT=29`
- `SILENT_COUNT=50`
- `ZERO_COUNT=0`
- `OTHER_COUNT=0`
- `BEST_DELAY_LINE=30,20,1,9,0,10,0`

From `delay_summary.csv`:

- `30ms`: total `20`, `ACK=1`, `NACK=9`, `SILENT=10`
- `50ms`: total `20`, `ACK=0`, `NACK=7`, `SILENT=13`
- `75ms`: total `20`, `ACK=0`, `NACK=3`, `SILENT=17`
- `100ms`: total `20`, `ACK=0`, `NACK=10`, `SILENT=10`

## Comparison vs v1.0 Sweep

v1.0 evidence:

- `DESIGN-CONTROLLER/bench-evidence/T6_rom_baseline_timing_2026-05-09_2115/`

v1.0 summary:

- `TOTAL_PROBES=20`
- `ACK_COUNT=1`
- `NACK_COUNT=7`
- `SILENT_COUNT=12`
- only ACK at `50ms`

v1.1 delta:

1. Probe count increased `4x` (`20` -> `80`).
2. Absolute ACK count stayed flat (`1`), so ACK rate dropped from `5.0%` to `1.25%`.
3. Best-delay signal remained in the shortest tested region, but shifted from `50ms` (v1.0 set) to `30ms` (v1.1 set).
4. `ZERO` class remained absent in this sweep (`0/80`), while failures were dominated by `SILENT` and secondarily `NACK`.

## Interpretation

1. The short-delay band still dominates any observed ACK opportunity, but reproducibility remains low.
2. Increasing attempts in-band did not produce proportional ACK gain; this argues against settle-delay-only tuning as the primary lever.
3. Failure distribution (`SILENT` > `NACK`) suggests entry-state instability or control-net/state contention is still dominating over simple serial framing mismatches.

## Conclusion

The 80-probe rerun does not improve ROM baseline reproducibility enough to treat timing-only hardening as sufficient. A discriminator exists, but ACK capture remains low-probability and unstable.

## Next Move

Shift from delay-only tuning to a stronger hardening axis:

- per-hold multi-sync burst logic with explicit burst-level acceptance thresholds
- state conditioning before first sync (ensure a reproducible ROM-ready precondition)
- capture/control-net correlation during successful vs failed short-delay bursts
