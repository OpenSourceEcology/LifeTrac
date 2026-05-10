# 2026-05-09 Controller Stage1 RX Heartbeat Classification (Copilot v1.2)

## Objective

Execute the next Method G Phase 1 / W1-7 classification step after v1.1:
- force deterministic stats diagnostics even when no RX deltas occur,
- rerun full Stage 1,
- determine whether ingress counters move under active probe traffic.

## Delta From v1.1

v1.1 added RX counters and delta-triggered stats snapshots.
Open risk remained: if no RX delta occurs, no snapshot is guaranteed.

v1.2 removes that ambiguity by adding a time-based forced heartbeat window.

## Firmware Change (v1.2)

File: `DESIGN-CONTROLLER/firmware/murata_l072/main.c`

Added unconditional early-runtime stats beacons:
- cadence: every 250 ms
- duration: first 6 s after `host_cmd_init`
- independent of `host_uart_stats_rx_bytes()` changes

Existing delta-triggered snapshot behavior remains in place.

Pseudo-behavior:
- `next_stats_heartbeat_ms = now + 100`
- `stats_heartbeat_until_ms = now + 6000`
- in loop: if `now < until` and `now >= next`, emit `host_cmd_emit_stats_snapshot()`

## Build Result

Build succeeded on Windows:
- `firmware.bin` size: 14144 bytes

## Bench Runs Executed

Full Stage 1 end-to-end rerun:
- evidence: `DESIGN-CONTROLLER/bench-evidence/T6_bringup_2026-05-09_145307/`
- flash: PASS
- boot pulse: PASS
- probe: FAIL (same request timeout)

Follow-up rerun after probe logging expansion (lane/error counters):
- evidence: `DESIGN-CONTROLLER/bench-evidence/T6_bringup_2026-05-09_145434/`
- image SHA256: `e5ba5b767db6d2d335c98c68206883559f54c2ece2546ae6bdfb95994b4cc86d`
- image size: 14144 bytes
- probe still FAIL with identical request timeout

## Key Evidence

From `method_g_stage1.log`:

- `STATS_URC(init): host_rx_bytes=0 host_parse_ok=0 host_parse_err=0`
- `BOOT_URC: not observed during initial window (continuing with active queries)`
- `reply to ATI: no bytes observed`
- `reply to AT+VER?: no bytes observed`
- `FATAL: probe failed: timeout waiting for response type 0x81 to req 0x01`

From follow-up rerun with expanded probe print:

- `STATS_URC(init): host_rx_bytes=0 host_rx_lpuart=0 host_rx_usart1=0 host_parse_ok=0 host_parse_err=0 uart_err_lpuart=0 uart_err_usart1=0`
- `FATAL: probe failed: timeout waiting for response type 0x81 to req 0x01`

Early 921600 hex capture also shows repeated `0xC1` frames (stats URCs), confirming forced beacon emission is active on wire.

## Interpretation

This run materially strengthens the prior diagnosis:

1. Firmware main loop is alive in the probe window.
2. Murata -> X8 TX path is alive enough to emit repeated stats URCs.
3. Reported RX counters remain zero (`host_rx_bytes=0`, `host_parse_ok=0`, `host_parse_err=0`).
4. Lane split and error accounting are also zero (`host_rx_lpuart=0`, `host_rx_usart1=0`, `uart_err_lpuart=0`, `uart_err_usart1=0`).

Therefore, the blocker is now best classified as:
- **host->Murata ingress path failure before UART RX ISR/accounting** (electrical routing, line direction, host TX side effects, or muxing),
- not command-dispatch logic and not parser-only reject.

## Immediate Next Actions

1. Verify physical TX transitions on X8 UART during `VER_REQ` send window (logic analyzer/scope).
2. Confirm board routing/mux direction for X8 TX -> Murata RX on the active lane.
3. If physical transitions exist, add temporary ISR-near single-byte toggle/marker for absolute byte-arrival confirmation.
4. Optionally gate the heartbeat to stop after first non-zero RX byte, preserving low-noise diagnostics while proving ingress.

## Files Changed in This Pass

- `DESIGN-CONTROLLER/firmware/murata_l072/main.c`
- `TODO.md`
- `DESIGN-CONTROLLER/TODO.md`
