# Controller Stage 1 Follow-Up: Phase 9 Backfill + Phase 10 Stability Quant (Copilot v1.0)

Date: 2026-05-09
Board: 2E2C1209DABC240B

## Why This Follow-Up

After the first post-fix Phase 10 success capture, the next question was whether a broad rewind of earlier phases was required.

Chosen approach:

- run one representative pre-Phase-10 backfill to check for drift
- run a bounded Phase 10 reproducibility campaign to quantify stability

## 1) Representative Backfill (Phase 9 cfg47)

Evidence:

- `DESIGN-CONTROLLER/bench-evidence/T6_phase9_backfill_cfg47_2026-05-09_210323/`

Artifacts:

- `47_phase9_boot_then_halt_000ms_openocd.txt`
- `47_phase9_boot_then_halt_000ms_stage1_probe.txt`

Observed probe signature (same as prior Phase 9):

- `BOOT_URC` present
- startup `FAULT_URC` (`0x03`, `0x08`) present
- active path returns repeated `STATS_URC` with `host_rx_bytes=0`
- probe ends with timeout waiting for `VER_URC` (`0x81` to req `0x01`)

Interpretation:

- no behavioral drift detected in this representative pre-Phase-10 slice
- broad rerun of earlier phases is not justified by this check

## 2) Phase 10 Stability Quantification (10 Runs)

Evidence:

- `DESIGN-CONTROLLER/bench-evidence/T6_phase10_atomic_stability_10x_2026-05-09_2110_clean/`

Key files:

- `results_clean.csv`
- `run_1_summary.txt` ... `run_10_summary.txt`
- `run_*_rom1_verify.txt`
- `run_1_userhalt_verify.txt` (only present on successful ROM-baseline run)

Campaign configuration:

- atomic runner `run_phase10_source_domain_ab_atomic.sh`
- `MAX_ROM_ATTEMPTS=1` (single-attempt Bernoulli style quantification)

Results from `results_clean.csv`:

- Run 1: `OK`, `ROM_ACK_ATTEMPT=1`, `USERHALT_ROM_RESP_SIZE=0`
- Runs 2..10: `FAIL_NO_ROM_BASELINE`, `ROM_ACK_ATTEMPT=0`

Aggregate:

- ROM-baseline success rate under this setup: `1/10` = 10%
- user-halt compare outcome on successful baseline run: still silent on `0x7F` (`ROM_RESP_SIZE=0`)

Interpretation:

- Phase 10 discriminator remains valid (captured successful ROM-positive / user-halt-negative run)
- main unresolved issue is reproducibility/stability of ROM baseline entry

## Conclusion

No full rewind is needed.

What is now supported by evidence:

- earlier phase conclusions remain directionally valid (checked with one representative backfill)
- Phase 10 proof exists but with low immediate repeatability under single-attempt gate conditions

## Recommended Next Step

Treat ROM baseline reproducibility as the active engineering target (timing hardening and deterministic orchestration), while keeping earlier phase interventions closed unless new contradictory evidence appears.
