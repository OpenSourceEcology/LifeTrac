# Method G Single-Board Stress Report (Imported)

- Imported: 2026-05-08 23:31:30
- Source device report: /tmp/lifetrac_p0c/stress_runs/20220503_102811/report.md
- Source run stamp: 20220503_102811
# Method G Single-Board Stress Report

- Generated: 2022-05-03 10:37:40 UTC
- Run directory: /tmp/lifetrac_p0c/stress_runs/20220503_102811
- Source TSV: /tmp/lifetrac_p0c/stress_runs/20220503_102811/results.tsv
- Summary log: /tmp/lifetrac_p0c/stress_runs/20220503_102811/summary.txt

## Results

| Metric | Value |
|---|---:|
| Total cycles | 20 |
| Pass | 0 |
| Fail | 20 |
| Pass rate | 0.0% |
| Banner hit count | 20 |
| Banner hit rate | 100.0% |
| Tick hit count | 0 |
| Tick hit rate | 0.0% |
| Auto-reboot heuristic hits | 20 |
| Avg cycle seconds | 19.2 |
| Min cycle seconds | 19 |
| Max cycle seconds | 20 |
| Avg rx bytes | 625.0 |
| Min rx bytes | 625 |
| Max rx bytes | 625 |

## Recommendation

- STAY_IN_SINGLE_BOARD_HARDENING

## Time Window

- First cycle start epoch: 1651573691
- Last cycle end epoch: 1651574078

## Notes

- Recommendation rule matches the one-board gate currently used by the stress wrapper: pass >= 95% and banner-hit >= 95%.
- Auto-reboot detection is text heuristic only.

