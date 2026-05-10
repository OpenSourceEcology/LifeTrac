# Stage1 Standard Contract — 100-Cycle Soak Launch (2026-05-10)

## Run Launched

- Quant folder:
  - `LifeTrac-v25/DESIGN-CONTROLLER/bench-evidence/T6_stage1_standard_quant_2026-05-10_100932_771-31048`
- Invocation:
  - `run_stage1_standard_quant_end_to_end.ps1 -AdbSerial 2E2C1209DABC240B -Cycles 100`

## Live State Snapshot

```text
STATUS=RUNNING
CYCLES_PLANNED=100
CYCLES_COMPLETED=8
LAST_CYCLE_RESULT=PASS
```

## Early Soak Quality

- `results.csv` cycles 1-8 all `PASS`
- All rows show `BOOT_OK=1`, `VERIFY_OK=1`, `WRITE_OK=1`
- No launcher failures observed in early rows (`launcher_rc=0`)

## Probe-Path Confirmation Under Soak

From cycle evidence (`T6_stage1_standard_2022-05-04_205601/boot_probe.log`):

- Multi-device sweep present:
  - `/dev/ttymxc3 /dev/ttymxc2 /dev/ttymxc1 /dev/ttymxc0`
- `/dev/ttymxc0` selected for AT response at 19200
- `stty: ... Inappropriate ioctl for device` observed and handled as non-fatal NOTE
- Valid AT payload captured (`AT+VER?`)

## Interpretation

The long soak is running with the intended patched behavior and early cycles are stable. The new `status.txt` mechanism provides clear liveness and completion state while the run is in progress.
