# 2026-05-10 Heartbeat Build Validation Run

## Scope
- Build murata_l072 firmware with bench-only heartbeat enabled at build time.
- Run a fresh 1-cycle stage-1 quant loop and inspect boot probe evidence for heartbeat and AT responses.

## Build Configuration
- Firmware path: `LifeTrac-v25/DESIGN-CONTROLLER/firmware/murata_l072`
- Build-time define used:
  - `-DLIFETRAC_BENCH_BOOT_HEARTBEAT_ENABLE=1`
- Method:
  - Set `EXTRA_CFLAGS` in environment before invoking `build.ps1`.
  - This keeps `config.h` default unchanged for production.

## Build Outcome
- Initial build attempt failed due missing register/IRQ defines referenced by host UART code:
  - `USART_CR1_RXNEIE`
  - `RNG_LPUART1_IRQn`
  - `USART1_IRQn`
- Fix applied in register header:
  - File: `LifeTrac-v25/DESIGN-CONTROLLER/firmware/murata_l072/include/stm32l072_regs.h`
  - Added missing bit/IRQ constants.
- Rebuild succeeded after header fix.
- Output image size: `14628` bytes.
- Heartbeat marker presence confirmed in binary (`LT_BOOT_HEARTBEAT` found in `build/firmware.bin`).

## Quant Run Outcome
- Quant folder:
  - `LifeTrac-v25/DESIGN-CONTROLLER/bench-evidence/T6_stage1_standard_quant_2026-05-10_043221_281-28208`
- Device serial used:
  - `2E2C1209DABC240B`
- Summary:
  - `CYCLES=1`
  - `LAUNCHER_FAIL_COUNT=1`
  - `TIMEOUT_COUNT=0`
  - `FINAL_RESULT_FAIL_BOOT=1`

## Boot Probe Evidence
- Standard run folder:
  - `LifeTrac-v25/DESIGN-CONTROLLER/bench-evidence/T6_stage1_standard_2022-05-04_151849`
- `boot_probe.log` result:
  - 19200 baud attempts 1..3: size `0` bytes
  - 9600 baud attempts 1..3: size `0` bytes
  - 38400 baud attempts 1..3: size `0` bytes
  - Final line: `AT response size = 0 bytes`
- No heartbeat lines were observed in boot probe capture.

## Environment/Execution Notes
- ADB multi-device conflict occurred initially (`more than one device/emulator`).
- After transient disconnect, run succeeded when pinned to serial `2E2C1209DABC240B`.

## Interpretation
- Flash pipeline and quant harness are functioning (erase/write/verify all pass, no timeout).
- Despite heartbeat-enabled image, UART capture remains fully silent post-release.
- This narrows root cause toward runtime path before/at UART observability, pin mux/route mismatch, or transport capture boundary rather than a simple AT parser issue.

## Suggested Next Bench Steps
1. Add an even earlier, minimal startup trace path that writes directly to the selected UART TX register before normal init dependencies.
2. Validate physical/UART routing assumptions for the tested unit against the serial node used by probe scripts.
3. Run one probe with OpenOCD halt-and-inspect at post-reset to verify PC/vector path enters expected app region.
