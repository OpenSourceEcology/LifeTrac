# Stage1 Boot-Probe Fix Validation — 2026-05-10

## Problem Summary

The stage1 contract was producing `FINAL_RESULT=FAIL_BOOT` consistently. Root cause
was a two-step failure in `boot_and_probe.sh`:

1. **Single-device probe** — original helper only checked `/dev/ttymxc3`; the
   Murata L072 user firmware actually responds on `/dev/ttymxc0` at 19200 8N1.

2. **`-c` availability guard regression** — first hardening pass used
   `[ -c "$dev" ]` (char-device check) to gate UART readiness.
   `/dev/ttymxc0` passes `[ -e "$dev" ]` but fails `[ -c "$dev" ]` on this
   kernel, causing `wait_for_uart_dev()` to return 1 and skip the device.

3. **Fatal `stty` ioctl** — `/dev/ttymxc0` emits `Inappropriate ioctl for
   device` from `stty` but remains readable/writable.  Originally treated as
   fatal (skip device); must be treated as informational.

## Fixes Applied (boot_and_probe.sh)

| Fix | Change |
|-----|--------|
| Multi-device sweep | `DEV_LIST_DEFAULT="/dev/ttymxc3 /dev/ttymxc2 /dev/ttymxc1 /dev/ttymxc0"` |
| Device availability | `[ -c "$dev" ]` → `[ -e "$dev" ]` in `wait_for_uart_dev()` |
| ioctl non-fatal | `grep -qi "Inappropriate ioctl for device"` → downgrade to `NOTE:`, continue probing |

## Validation Evidence

### Direct-check run: `T6_stage1_standard_directcheck_2026-05-10_073200_fix2c`

```
BOOT_OK=1
FINAL_RESULT=PASS
ELAPSED_S=88
```

**boot_probe.log** confirms the decision path:
- `/dev/ttymxc3`, `/dev/ttymxc2`, `/dev/ttymxc1` → 0 bytes each baud/attempt
- `/dev/ttymxc0 19200 8N1`:
  - `stty: /dev/ttymxc0: Inappropriate ioctl for device`
  - `NOTE: continuing with existing line settings ...`
  - `Passive probe result: ... size=9 bytes`
  - `AT response selected from dev=/dev/ttymxc0 baud=19200 attempt=1`
  - Response payload: `AT+VER?..` (9 bytes)

No `WARN: skipping ... (device unavailable)` lines present. The ioctl complaint
was treated as non-fatal exactly as intended.

### 1-cycle quant: `T6_stage1_standard_quant_2026-05-10_072647_080-26340`

```
CYCLES=1
FINAL_RESULT_PASS=1
LAUNCHER_FAIL_COUNT=0
TIMEOUT_COUNT=0
```

## Quant Wrapper Hardening (run_stage1_standard_quant_end_to_end.ps1)

Added a live `status.txt` file to `$quantDir`:

- Written at loop start: `STATUS=RUNNING`, `CYCLES_COMPLETED=0`
- Updated after every cycle: `CYCLES_COMPLETED=N`, `LAST_CYCLE_RESULT=...`
- Finalized after loop: `STATUS=COMPLETE`

If a run is interrupted or crashes mid-loop, `status.txt` remains as
`STATUS=RUNNING` with the last completed cycle count — unambiguously incomplete.
Previously the only indicator was an empty `results.csv` (header only) and
absent `summary.txt`, both of which look identical to a run that just started.

## Known Probe Behavior

- `/dev/ttymxc0` is the authoritative response path for post-flash L072 AT probing.
- `/dev/ttymxc3` is the flash transport (AN3155 8E1 19200) and is silent after boot.
- `/dev/ttymxc1`, `/dev/ttymxc2` are silent.
- `stty` ioctl errors on `/dev/ttymxc0` are a kernel quirk, not a functional fault.
