# 2026-05-09 Transport Ingress Investigation (Copilot v1.0)

## Scope
Board: 2E2C1209DABC240B
Goal: resolve persistent `VER_REQ` timeout during Method G Stage 1 (`host_rx_bytes=0`).

## What Was Changed

### Firmware
1. `LifeTrac-v25/DESIGN-CONTROLLER/firmware/murata_l072/host/host_uart.c`
- Added additional first-error traces:
  - `H:ERR_LPUART1`
  - `H:ERR_USART1`
- Kept prior RX/dispatch traces in place:
  - `H:RX_LPUART1`, `H:RX_USART1`, `H:VER_REQ_RX`, `C:VER_DISP`, `C:AT_VER_DISP`
- Improved USART1 baud generation for 921600 by enabling OVER8 and BRR encoding logic.
- Added local `USART_CR1_OVER8` bit definition for this register header set.

### Probe tooling
2. `LifeTrac-v25/DESIGN-CONTROLLER/firmware/x8_lora_bootloader_helper/method_g_stage1_probe.py`
- Added robust serial `write_all()` path for non-blocking FD writes (handles partial writes / `BlockingIOError`).
- Updated `send()` and `send_ascii()` to use `write_all()`.
- Updated UART setup to explicitly disable flow control:
  - `-ixon -ixoff -ixany -crtscts`

## Validation Runs

### Stage 1 re-runs after each change
Latest confirmed run metadata:
- `bench-evidence/T6_bringup_2026-05-09_162430/run_metadata.json`
- `local_image_sha256`: `8b4ccac8630a298f4f12deba77d5911a48c16499f522374fa103d97677d0e53c`
- `image_size_bytes`: `14820`

Latest probe outcome from device-side log (`/tmp/lifetrac_p0c/method_g_stage1.log`, timestamp 03:27:29 UTC):
- `STATS_URC(init): host_rx_bytes=0 host_rx_lpuart=0 host_rx_usart1=0`
- No `H:RX_*`, no `H:ERR_*`, no `H:VER_REQ_RX`, no `C:VER_DISP`, no `C:AT_VER_DISP`
- ASCII fallback gets only `FAULT_URC` code `0x0A` (`no_ingress_during_probe_window`) or no bytes.
- Final: `timeout waiting for response type 0x81 to req 0x01`.

### Extra baud sweep (direct probe on X8)
Executed probe with sudo at multiple baud rates (`921600, 460800, 230400, 115200, 57600, 38400, 19200`).
Result: no ingress response path recovered at alternate bauds.

## Interpretation
Parser and dispatch are not the active bottleneck.

Evidence indicates host-originated bytes are not reaching the active Murata RX path at all during these runs:
- RX counters remain exactly zero.
- UART error counters/traces also remain zero (not even framing/noise activity).
- No transport parse traces fire.

Given this, the remaining likely root-cause class is lane/routing ownership outside protocol parser logic (electrical/mux/ownership/path state), not binary framing correctness.

## Recommended Next Decisive Checks
1. Add a controlled loopback/line-monitor step at X8 side:
- Verify TX toggling activity on `/dev/ttymxc3` while probe writes (logic probe or known-good USB-UART tee).
2. Confirm RX net ownership/routing state during Stage 1 window:
- Revalidate that no concurrent process reclaims/reconfigures LoRa UART route after boot sequence.
3. Add temporary firmware diagnostic that forces a periodic marker when RX IRQ pending flag transitions (without requiring successful byte read path) to distinguish "no edge" vs "interrupt not enabled".
4. If hardware routing uncertainty remains, run a minimal echo firmware variant (single-uart, no radio stack, 921600 and 19200 variants) to isolate transport from system load.

## Current Status
- Build is healthy.
- Probe tooling is more robust than baseline.
- Original timeout issue persists with stronger evidence that ingress is not arriving to firmware RX handlers.
