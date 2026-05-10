# Controller Stage 1 ROM Burst Matrix Campaign Result (Copilot v1.1)

Date: 2026-05-09
Scope: Validate the hardened ROM baseline burst matrix on the X8 target and record reproducibility by delay bucket.

## Run summary

Completed evidence folder:
- [DESIGN-CONTROLLER/bench-evidence/T6_rom_baseline_burst_2022-05-04_084927/](DESIGN-CONTROLLER/bench-evidence/T6_rom_baseline_burst_2022-05-04_084927/)

Run settings from `summary.txt`:
- `DELAYS_MS=30,50`
- `BURSTS_PER_DELAY=5`
- `ATTEMPTS_PER_BURST=10`
- `BURST_PASS_MIN_ACK=1`

Overall result:
- `TOTAL_BURSTS=10`
- `PASS_BURSTS=3`
- `FAIL_BURSTS=7`
- `BURST_PASS_RATE=0.3000`
- `TOTAL_PROBES=100`
- `ACK_COUNT=3`
- `NACK_COUNT=47`
- `ZERO_COUNT=0`
- `SILENT_COUNT=50`
- `OTHER_COUNT=0`

Delay breakdown:
- `30 ms`: `2/5` burst passes, `2 ACK`, `23 NACK`, `25 SILENT`
- `50 ms`: `1/5` burst passes, `1 ACK`, `24 NACK`, `25 SILENT`

Best delay line:
- `30,5,2,3,0.4000,50,2,23,0,25,0`

## Interpretation

The hardened harness now scales beyond the minimal verify case and still reaches deterministic completion (`run.log` ends with `done`). The run does not show a strong monotonic pass-rate improvement with longer delay; instead, ACKs remain sparse and concentrated in the short-delay band. That is consistent with the earlier timing-sweep signal that shorter settle windows are more likely to produce ROM sync, but burst-level reproducibility is still low.

## Next practical step

If the goal is to improve ROM-entry repeatability rather than simply characterize it, the next experiment should stay near the short-delay edge and increase the number of sync attempts per burst, instead of widening the delay window further.
