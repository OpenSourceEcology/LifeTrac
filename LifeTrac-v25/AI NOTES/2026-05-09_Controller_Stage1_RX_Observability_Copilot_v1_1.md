# 2026-05-09 Controller Stage1 RX Observability (Copilot v1.1)

## Objective

Advance Method G Phase 1 / W1-7 by instrumenting firmware to classify the remaining blocker:
- Case A: no bytes reach Murata RX ISR from X8 during active probe traffic.
- Case B: bytes arrive but are dropped by framing/parser logic before `VER_URC`.

This v1.1 note supersedes the open action from v1.0 to add RX observability and rerun Stage 1.

## Firmware Changes Implemented

### 1) Additive host stats counters in UART transport

File: `DESIGN-CONTROLLER/firmware/murata_l072/host/host_uart.c`

Added counters:
- `host_rx_bytes` (all ISR RX bytes)
- `host_rx_lpuart_bytes`
- `host_rx_usart1_bytes`
- `host_parse_ok`
- `host_parse_err`
- `host_uart_err_lpuart`
- `host_uart_err_usart1`

Behavioral details:
- RX byte counters increment in ISR RXNE loops.
- parser error counter increments on COBS decode failure or inner-frame parse failure.
- parser OK counter increments on successful inner-frame parse.
- per-lane UART error counters increment on PE/FE/NE/ORE paths.

### 2) Expose new counters in STATS payload (additive-only)

Files:
- `DESIGN-CONTROLLER/firmware/murata_l072/include/host_types.h`
- `DESIGN-CONTROLLER/firmware/murata_l072/host/host_stats.c`

Wire schema extension (tail-only):
- offsets 68..92 used for new host RX observability fields
- `HOST_STATS_PAYLOAD_LEN` changed from 68 to 96

Compatibility note:
- layout remains additive; existing parsers that consume only the original 68 bytes remain valid.

### 3) Surface counters in AT+STAT text output

File: `DESIGN-CONTROLLER/firmware/murata_l072/host/host_cmd.c`

Added AT keys:
- `HOST_RX_BYTES`
- `HOST_RX_LPUART`
- `HOST_RX_USART1`
- `HOST_PARSE_OK`
- `HOST_PARSE_ERR`
- `HOST_UART_ERR_LPUART`
- `HOST_UART_ERR_USART1`

### 4) Add unsolicited stats URC snapshot on RX-byte changes

Files:
- `DESIGN-CONTROLLER/firmware/murata_l072/include/host_cmd.h`
- `DESIGN-CONTROLLER/firmware/murata_l072/host/host_cmd.c`
- `DESIGN-CONTROLLER/firmware/murata_l072/main.c`

New API:
- `host_cmd_emit_stats_snapshot()`

Main-loop behavior:
- when `host_uart_stats_rx_bytes()` changes, emit one `STATS_URC` snapshot (seq=0)
- rate-limited to 200 ms to avoid flooding

Reason:
- enables RX-ingress observability without requiring successful request/response completion.

### 5) Probe parser updates for new fields

File: `DESIGN-CONTROLLER/firmware/x8_lora_bootloader_helper/method_g_stage1_probe.py`

- Extended `parse_stats()` labels to include new counters.
- Added init-window print path for unsolicited `STATS_URC` frames.
- Expanded `STATS_URC` summary line to include `host_rx_bytes` and `host_parse_err`.

## Build Validation

Windows build succeeded after instrumentation:
- output image: `firmware/murata_l072/build/firmware.bin`
- final verified image size in last run metadata: 14100 bytes

## Bench Runs Executed

Two full Stage 1 end-to-end reruns were executed with the updated firmware:
- `DESIGN-CONTROLLER/bench-evidence/T6_bringup_2026-05-09_144749/`
- `DESIGN-CONTROLLER/bench-evidence/T6_bringup_2026-05-09_144935/`

Both runs show:
- flash PASS
- boot pulse PASS
- early 921600 capture includes startup TX breadcrumbs and startup URCs (`FAULT_URC`, `BOOT_URC`, `FAULT_URC`)
- active probe still fails waiting for `VER_URC` to `VER_REQ`
- ASCII fallback (`ATI`, `AT+VER?`) still no bytes

Critical observation:
- no unsolicited `STATS_URC` snapshot appears in early capture/probe window even with snapshot-on-RX-change instrumentation.

## Classification Result

Current evidence strongly favors:
- **Case A (no host->Murata ingress at UART ISR level)** during active probe traffic.

Rationale:
- Murata TX path is clearly alive (startup breadcrumbs + startup URCs captured).
- If probe bytes were reaching RX ISR, `host_rx_bytes` would change and trigger a `STATS_URC` snapshot; none observed.
- Therefore the failure is likely before parser logic (transport/routing/electrical directionality), not a pure parser bug.

## Updated Next Actions

1. Add a temporary forced periodic diagnostic URC (time-based, not RX-change-based) carrying RX counters for absolute confirmation in every run.
2. Capture X8 TX physical activity during `VER_REQ` send window (logic analyzer/scope).
3. Verify X8 UART open/flush/reopen behavior around the probe start for any line-state side effects.
4. If physical TX is present but ISR counters remain zero, trace board routing and mux directionality for X8 TX -> Murata RX lane.

## Files Modified in v1.1

- `DESIGN-CONTROLLER/firmware/murata_l072/include/host_uart.h`
- `DESIGN-CONTROLLER/firmware/murata_l072/include/host_types.h`
- `DESIGN-CONTROLLER/firmware/murata_l072/include/host_cmd.h`
- `DESIGN-CONTROLLER/firmware/murata_l072/host/host_uart.c`
- `DESIGN-CONTROLLER/firmware/murata_l072/host/host_stats.c`
- `DESIGN-CONTROLLER/firmware/murata_l072/host/host_cmd.c`
- `DESIGN-CONTROLLER/firmware/murata_l072/main.c`
- `DESIGN-CONTROLLER/firmware/x8_lora_bootloader_helper/method_g_stage1_probe.py`
- `TODO.md`
- `DESIGN-CONTROLLER/TODO.md`
