# 2026-05-10 Stage1 Standard Quant 20x Run (Copilot v1.0)

## Why this run

After the hold-window discriminator sweep, this run was used to quantify end-to-end launcher stability on the current bench path and verify repeatability over multiple cycles.

## Evidence folder

- `DESIGN-CONTROLLER/bench-evidence/T6_stage1_standard_quant_2026-05-10_103651_078-25816/`

## Run parameters

- Script: `DESIGN-CONTROLLER/firmware/x8_lora_bootloader_helper/run_stage1_standard_quant_end_to_end.ps1`
- Cycles: `20`
- Per-cycle timeout: `180 s`
- Board serial: `2E2C1209DABC240B`
- Window: `2026-05-10T15:36:51Z` to `2026-05-10T16:06:19Z`

## Summary metrics

From `summary.txt`:
- `LAUNCHER_FAIL_COUNT=0`
- `TIMEOUT_COUNT=0`
- `FINAL_RESULT_PASS=20`

From `results.csv`:
- `20/20` cycles are `PASS`
- `launcher_rc=0` on all cycles
- Sub-check fields are all `1` on each row (`sync_ok/getid_ok/erase_ok/write_ok/verify_ok/boot_ok`)
- Per-cycle elapsed time is stable (`88-89 s`)

## Interpretation

What this confirms:
- The Stage1 standard launcher path is currently stable over repeated cycles on this bench setup.
- Flash/write/verify/boot sub-steps are consistently successful in this run.

What this does not confirm:
- This quant run does not replace Method G ingress criteria (`VER_REQ` -> `VER_URC`) used by the active protocol bring-up gate.
- Therefore, this result improves confidence in tooling/path reliability but does not clear the W1-7 protocol blocker by itself.

## Conclusion

The 20-cycle standard quant run is a clean pass and should be treated as positive infrastructure/harness stability evidence.
Protocol ingress bring-up remains blocked until Method G gate evidence turns green.
