# 2026-05-10 Boot Probe Sweep and Heartbeat Diagnostics

## Scope
Implemented and validated the three next-step diagnostics after stable 20/20 `FAIL_BOOT` outcomes with no hangs:

1. Multi-attempt AT probing with per-attempt logging.
2. Multi-baud sweep probing (19200, 9600, 38400).
3. Bench-gated early firmware boot heartbeat markers.

## Code Changes

### 1) Boot Probe Retry + Baud Sweep
- File: `DESIGN-CONTROLLER/firmware/x8_lora_bootloader_helper/boot_and_probe.sh`
- Changes:
  - Added `BAUD_LIST="19200 9600 38400"`.
  - Added `ATTEMPTS_PER_BAUD=3`.
  - Added `probe_at_once()` helper to:
    - Reconfigure UART per baud.
    - Send `AT` + `AT+VER?`.
    - Capture per-attempt response and stderr.
    - Log: `AT probe result: baud=... attempt=... size=... bytes`.
  - Tracks best response and emits final contract line:
    - `AT response size = <N> bytes`.

### 2) Bench-Gated Firmware Heartbeat Support
- File: `DESIGN-CONTROLLER/firmware/murata_l072/config.h`
- Added:
  - `LIFETRAC_BENCH_BOOT_HEARTBEAT_ENABLE` (default `0`)

- File: `DESIGN-CONTROLLER/firmware/murata_l072/main.c`
- Added (gated by `LIFETRAC_BENCH_BOOT_HEARTBEAT_ENABLE`):
  - `LT_BOOT_HEARTBEAT stage=post_uart_init`
  - `LT_BOOT_HEARTBEAT stage=radio_ready` / `stage=radio_fault`

Note: heartbeat path is implemented but not active unless firmware is rebuilt with
`LIFETRAC_BENCH_BOOT_HEARTBEAT_ENABLE=1`.

## Validation Run (Post-Change)

### Quant run folder
- `DESIGN-CONTROLLER/bench-evidence/T6_stage1_standard_quant_2026-05-10_042854_580-23616`

### Result
- `CYCLES=1`
- `FINAL_RESULT_FAIL_BOOT=1`
- `TIMEOUT_COUNT=0`
- All flash gates remained green: `SYNC_OK=1`, `GETID_OK=1`, `ERASE_OK=1`, `WRITE_OK=1`, `VERIFY_OK=1`
- `BOOT_OK=0`

### Probe evidence
- `DESIGN-CONTROLLER/bench-evidence/T6_stage1_standard_2022-05-04_151522/boot_probe.log`
- Sweep executed fully:
  - `19200 x3`, `9600 x3`, `38400 x3`
- Every attempt reported `size=0 bytes`
- Final line preserved: `AT response size = 0 bytes`

## Interpretation
The instrumentation is now robust and complete:
- No false truncation at probe stage.
- No hidden early exits in the probe path.
- No baud among tested set returned bytes.

Current bottleneck is not harness reliability; it is missing post-boot UART response from the L072 user app path.

## Recommended Immediate Bench Follow-up
1. Rebuild and flash firmware with `LIFETRAC_BENCH_BOOT_HEARTBEAT_ENABLE=1`.
2. Re-run 1-cycle quant and inspect probe log for `LT_BOOT_HEARTBEAT` text.
3. If heartbeat appears but AT stays silent, focus on AT parser/dispatch path.
4. If heartbeat does not appear, focus on early app execution/clock/UART route assumptions after BOOT0 release.
