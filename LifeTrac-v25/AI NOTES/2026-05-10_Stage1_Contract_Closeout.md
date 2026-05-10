# Stage1 Standard Contract Closeout - 2026-05-10

## Decision

Stage1 standard contract validation is accepted.

## Acceptance Criteria

A run is accepted only if all checks below are true:

1. `FINAL_RESULT_PASS == CYCLES`
2. `LAUNCHER_FAIL_COUNT == 0`
3. `TIMEOUT_COUNT == 0`
4. Representative cycle logs confirm corrected boot probe decision path:
   - UART sweep includes `/dev/ttymxc3 /dev/ttymxc2 /dev/ttymxc1 /dev/ttymxc0`
   - `/dev/ttymxc0` is selected for AT response at 19200 8N1
   - `stty` ioctl complaint is handled as non-fatal NOTE

## Primary Evidence

### 100-cycle soak (final signoff)

- Folder:
  - `LifeTrac-v25/DESIGN-CONTROLLER/bench-evidence/T6_stage1_standard_quant_2026-05-10_100932_771-31048`
- Final summary:
  - `CYCLES=100`
  - `FINAL_RESULT_PASS=100`
  - `LAUNCHER_FAIL_COUNT=0`
  - `TIMEOUT_COUNT=0`
- Live status at completion:
  - `STATUS=COMPLETE`
  - `CYCLES_COMPLETED=100`

### Direct check (post-fix)

- Folder:
  - `LifeTrac-v25/DESIGN-CONTROLLER/bench-evidence/T6_stage1_standard_directcheck_2026-05-10_073200_fix2c`
- Summary:
  - `BOOT_OK=1`
  - `FINAL_RESULT=PASS`

### Probe path confirmation under quant

- Example cycle folder:
  - `LifeTrac-v25/DESIGN-CONTROLLER/bench-evidence/T6_stage1_standard_2022-05-04_205601`
- `boot_probe.log` confirms:
  - full UART device sweep
  - non-fatal `stty: Inappropriate ioctl for device` note
  - AT response selected from `/dev/ttymxc0`

## Scope of Fixes Validated

1. `boot_and_probe.sh`
   - Multi-device sweep includes `/dev/ttymxc0`
   - `wait_for_uart_dev` uses `-e` instead of `-c`
   - Known `stty` ioctl warning treated as non-fatal

2. `run_stage1_standard_quant_end_to_end.ps1`
   - Writes live `status.txt` while running
   - Marks `STATUS=COMPLETE` on successful loop completion

## Operational Outcome

The prior `FAIL_BOOT` mode caused by stale/single-device probing and strict device-type checks is resolved for this hardware path. The post-fix harness sustained 100/100 PASS with zero launcher failures and zero timeouts.
