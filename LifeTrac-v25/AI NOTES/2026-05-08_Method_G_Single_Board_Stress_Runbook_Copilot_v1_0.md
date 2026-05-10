# Method G Single-Board Stress Runbook

**Date:** 2026-05-08
**Author:** GitHub Copilot (GPT-5.3-Codex)
**Version:** v1.0

## Purpose

Run repeated one-board Method G flash cycles and produce quantitative reliability metrics (pass rate, banner/tick liveness, basic reboot signal) before two-board RF gates.

## Script

- `LifeTrac-v25/DESIGN-CONTROLLER/firmware/x8_lora_bootloader_helper/run_single_board_stress.sh`
- `LifeTrac-v25/DESIGN-CONTROLLER/firmware/x8_lora_bootloader_helper/summarize_single_board_stress.sh`
- `LifeTrac-v25/DESIGN-CONTROLLER/firmware/x8_lora_bootloader_helper/pull_stress_report_to_ai_notes.ps1`
- `LifeTrac-v25/DESIGN-CONTROLLER/firmware/x8_lora_bootloader_helper/run_single_board_stress_end_to_end.ps1`
- `LifeTrac-v25/DESIGN-CONTROLLER/firmware/x8_lora_bootloader_helper/update_latest_stress_status.ps1`

## Inputs

1. Image path (default `/tmp/lifetrac_p0c/hello.bin`)
2. Number of cycles (default `10`)
3. Optional environment controls:
- `RUN_POST_LISTEN` default `1`
- `POST_LISTEN_SEC` default `8`
- `RUN_REVIVE` default `0`

## Example command

```bash
echo fio | sudo -S -p '' \
  RUN_POST_LISTEN=1 POST_LISTEN_SEC=10 RUN_REVIVE=0 \
  bash /tmp/lifetrac_p0c/run_single_board_stress.sh /tmp/lifetrac_p0c/mlm32l07x01.bin 20
```

## Outputs

Per run set:
- `/tmp/lifetrac_p0c/stress_runs/<timestamp>/summary.txt`
- `/tmp/lifetrac_p0c/stress_runs/<timestamp>/results.tsv`
- `/tmp/lifetrac_p0c/stress_runs/<timestamp>/cycle_###/` logs and captures

Captured artifacts per cycle (when present):
- `pipeline.log`
- `flash_run.log`
- `flash_ocd.log`
- `boot_listen.log`
- `openocd_boot.log`
- `rx.bin`
- `pipeline.stdout.log`

Generated report:
- `<run_dir>/report.md` (from `summarize_single_board_stress.sh`)

## Metrics

The stress wrapper computes:

1. flash-pipeline pass count and pass percentage,
2. runtime banner hits (`LIFETRAC L072` in `rx.bin`),
3. runtime tick hits (`tick=` in `rx.bin`),
4. basic reboot-text heuristic in console log,
5. per-cycle duration via epoch start/end.

## Recommendation rule

At end of run, script emits:
- `recommendation=READY_FOR_NEXT_STAGE` if pass >= 95% and banner hit >= 95%.
- else `recommendation=STAY_IN_SINGLE_BOARD_HARDENING`.

## Report generation

After a stress run completes, generate a markdown report from the latest run:

```bash
echo fio | sudo -S -p '' \
  bash /tmp/lifetrac_p0c/summarize_single_board_stress.sh
```

Or provide a specific run folder:

```bash
echo fio | sudo -S -p '' \
  bash /tmp/lifetrac_p0c/summarize_single_board_stress.sh \
  /tmp/lifetrac_p0c/stress_runs/<timestamp>
```

## Import report into AI NOTES (Windows host)

Use the PowerShell helper to run the remote summary and pull the generated
`report.md` into this repository's `AI NOTES` folder with a dated filename:

```powershell
powershell -ExecutionPolicy Bypass -File \
  "LifeTrac-v25/DESIGN-CONTROLLER/firmware/x8_lora_bootloader_helper/pull_stress_report_to_ai_notes.ps1"
```

Optional parameters:

```powershell
powershell -ExecutionPolicy Bypass -File \
  "LifeTrac-v25/DESIGN-CONTROLLER/firmware/x8_lora_bootloader_helper/pull_stress_report_to_ai_notes.ps1" \
  -AdbSerial 2E2C1209DABC240B \
  -RunDir /tmp/lifetrac_p0c/stress_runs/<timestamp>
```

After import, the helper also refreshes `AI NOTES/LATEST_STRESS_STATUS.md`.

## One-command end-to-end (Windows host)

This orchestrator pushes helper scripts to the X8, runs the stress batch,
then imports the generated report into `AI NOTES`:

```powershell
powershell -ExecutionPolicy Bypass -File \
  "LifeTrac-v25/DESIGN-CONTROLLER/firmware/x8_lora_bootloader_helper/run_single_board_stress_end_to_end.ps1"
```

Example with explicit options:

```powershell
powershell -ExecutionPolicy Bypass -File \
  "LifeTrac-v25/DESIGN-CONTROLLER/firmware/x8_lora_bootloader_helper/run_single_board_stress_end_to_end.ps1" \
  -AdbSerial 2E2C1209DABC240B \
  -ImageRemote /tmp/lifetrac_p0c/mlm32l07x01.bin \
  -Cycles 20 \
  -RunPostListen 1 \
  -PostListenSec 10 \
  -RunRevive 0
```

## Refresh latest status manually

If needed, refresh the latest-status index from the newest imported report:

```powershell
powershell -ExecutionPolicy Bypass -File \
  "LifeTrac-v25/DESIGN-CONTROLLER/firmware/x8_lora_bootloader_helper/update_latest_stress_status.ps1"
```

## Notes

1. This is intentionally one-board-only validation and does not close W4-00 RF exchange gates.
2. The reboot signal is heuristic text-matching only; use explicit uptime probes in future if needed.
3. Keep `RUN_REVIVE=0` unless diagnosing revive behavior.
