# 2026-05-09 Controller Stage1 ISR RX Marker Classification (Copilot v1.3)

## Objective

Extend v1.2 classification with an ISR-near ingress marker that is independent of frame parsing:
- latch first RX-byte-seen events per UART lane inside ISR context,
- emit a one-shot FAULT marker to host link,
- rerun full Stage 1 and classify result.

## Delta From v1.2

v1.2 proved deterministic heartbeat stats with zero ingress counters during active probe.
Open residual risk: a parser/path issue could still hide ISR-level byte arrival.

v1.3 removes that risk by instrumenting first-byte detection at ISR boundary.

## Firmware / Probe Changes (v1.3)

### Firmware

- `DESIGN-CONTROLLER/firmware/murata_l072/include/host_types.h`
  - Added `HOST_FAULT_CODE_HOST_RX_SEEN = 0x09`.

- `DESIGN-CONTROLLER/firmware/murata_l072/include/host_uart.h`
  - Added `uint8_t host_uart_take_rx_seen_flags(void);`.

- `DESIGN-CONTROLLER/firmware/murata_l072/host/host_uart.c`
  - Added volatile ISR latch `s_rx_seen_flags`.
  - Added per-lane bits:
    - `HOST_RX_SEEN_FLAG_LPUART = 0x01`
    - `HOST_RX_SEEN_FLAG_USART1 = 0x02`
  - Set bits in RXNE loops for each lane before ingest.
  - Added atomic take-and-clear accessor `host_uart_take_rx_seen_flags()`.
  - Reset latch in `host_uart_stats_reset()`.

- `DESIGN-CONTROLLER/firmware/murata_l072/main.c`
  - Added one-shot emission logic:
    - poll `host_uart_take_rx_seen_flags()` each loop,
    - emit `host_cmd_emit_fault(HOST_FAULT_CODE_HOST_RX_SEEN, new_flags)` only for first-seen lane bits.

### Probe

- `DESIGN-CONTROLLER/firmware/x8_lora_bootloader_helper/method_g_stage1_probe.py`
  - Added FAULT payload decoder for code/sub fields.
  - Added human-readable lane text for code `0x09`.
  - `FAULT_URC` lines now print decoded form when available.

## Build Result

Build succeeded on Windows:
- `firmware.bin` size: 14244 bytes

## Bench Run Executed

Full Stage 1 end-to-end rerun:
- evidence: `DESIGN-CONTROLLER/bench-evidence/T6_bringup_2026-05-09_145716/`
- flash: PASS
- boot pulse: PASS
- probe: FAIL

## Key Evidence

From run logs:

- `STATS_URC(init): host_rx_bytes=0 host_rx_lpuart=0 host_rx_usart1=0 host_parse_ok=0 host_parse_err=0 uart_err_lpuart=0 uart_err_usart1=0`
- `FATAL: probe failed: timeout waiting for response type 0x81 to req 0x01`

Expected new marker if any ISR ingress occurred:
- `FAULT_URC: code=0x09 sub=... lanes=...`

Observed:
- no `HOST_RX_SEEN` marker in active probe window.

## Interpretation

v1.3 confirms and tightens the v1.2 diagnosis:

1. Firmware loop and TX path are active enough to emit `STATS_URC`.
2. RX accounting remains zero on both lanes.
3. ISR-near first-byte latch does not trigger during active probe traffic.

Best current classification:
- **host->Murata ingress failure before UART RX ISR visibility**, not parser/dispatch.

## Recommended Next Actions

1. Logic-analyzer capture on X8 TX and Murata RX pin(s) during `VER_REQ` send window.
2. Verify board-level mux/routing and electrical level compatibility for the active lane.
3. Confirm probe-side serial reopen/flush behavior is not suppressing outbound write path on X8.
4. If electrical transitions are present at Murata RX pin, add temporary GPIO edge-toggles in RXNE IRQ entry for cycle-accurate correlation.

## Files Changed In This Pass

- `DESIGN-CONTROLLER/firmware/murata_l072/include/host_types.h`
- `DESIGN-CONTROLLER/firmware/murata_l072/include/host_uart.h`
- `DESIGN-CONTROLLER/firmware/murata_l072/host/host_uart.c`
- `DESIGN-CONTROLLER/firmware/murata_l072/main.c`
- `DESIGN-CONTROLLER/firmware/x8_lora_bootloader_helper/method_g_stage1_probe.py`
- `TODO.md`
- `DESIGN-CONTROLLER/TODO.md`
