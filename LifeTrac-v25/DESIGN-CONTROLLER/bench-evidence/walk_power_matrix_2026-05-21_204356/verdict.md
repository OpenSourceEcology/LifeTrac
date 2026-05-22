# Walk-power falsification matrix verdict

- run_uuid: `fbfcc984-9080-4e93-9501-3604a9b30304`
- git_sha_short: `156ee63`
- run_start_ts_local: `2026-05-21_204356`

## Per-pass summary

| Pass | LBT | inter_s | total pkts | overall PER % | LBT aborts | Cliff |
|------|-----|---------|------------|---------------|------------|-------|
| A | 0 | 0.05 | 800 | 0.0 | 0 | none |
| B | 1 | 0.05 | 800 | 0.0 | 0 | none |
| C | 0 | 0.02 | 800 | 13.375 | 0 | none |

## Decision rule application

- **Host-cadence hypothesis**: aggregate PER A=0.00% vs C=13.38% (delta=+13.38 pp, threshold >= 5.0 pp). Only differing variable is `inter_cycle_s` (A=0.05, C=0.02). **CONFIRMED**: host cadence is a real factor in the legacy 20 ms tick regime. (May still be downstream of a real RF/duty-cycle effect that only manifests when ToA + cadence saturate the host.)
- **LBT-defer hypothesis**: aggregate PER A=0.00% vs B=0.00% (delta=+0.00 pp). **FALSIFIED at aggregate level**: LBT is not a major contributor at this cadence.
- **Overall**: no cliff in any of the three passes. The original pilot cliff was not reproducible; treat the pilot finding as a transient (likely supply, antenna, or peer drift).
