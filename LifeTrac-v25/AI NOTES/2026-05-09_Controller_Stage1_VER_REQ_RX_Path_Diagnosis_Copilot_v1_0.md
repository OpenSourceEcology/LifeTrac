# 2026-05-09 Controller Stage1 VER_REQ RX Path Diagnosis (Copilot v1.0)

## Scope

Continue Method G Phase 1 / W1-7 debugging after BOOT_URC wire decode success.
Primary objective: explain why `VER_REQ` (`0x01`) from probe receives no `VER_URC` (`0x81`) even though firmware startup is proven.

## Confirmed Baseline Before This Pass

- Stage 1 harness fixes are in place and validated:
  - boot OpenOCD runs synchronously
  - 921600 capture uses timeout-only `cat` (no `count=` cap)
  - post-capture hex dump is logged
- Captured startup traffic includes valid COBS frames:
  - `FAULT_URC` (radio init fail)
  - `BOOT_URC`
  - `FAULT_URC` (HSE fail)
- Therefore custom firmware reaches `host_cmd_init()` and enters main loop.

## Code Audit Findings

### 1) Host type constants are correct

File: `firmware/murata_l072/include/host_types.h`

- `HOST_TYPE_VER_REQ = 0x01`
- `HOST_TYPE_VER_URC = 0x81`
- `HOST_TYPE_BOOT_URC = 0xF0`
- `HOST_TYPE_FAULT_URC = 0xF1`

No mismatch with probe constants.

### 2) Probe frame construction is correct

File: `firmware/x8_lora_bootloader_helper/method_g_stage1_probe.py`

- Frame header format: `<BBBHH>`
- CRC16-CCITT implementation matches firmware expectations.
- `request()` requires both matching response type and matching `seq`.
- Probe sends `VER_REQ` and waits for `VER_URC` with same seq.

No obvious protocol-construction bug identified.

### 3) Firmware response seq echo is correct

File: `firmware/murata_l072/host/host_cmd.c`

- `handle_version()` calls `host_uart_send_urc(HOST_TYPE_VER_URC, frame->seq, ...)`.

No seq mismatch bug in dispatch path.

## Firmware Change Tested In This Pass

### Change made

File: `firmware/murata_l072/host/host_uart.c`

- Kept USART1 as TX mirror only.
- Disabled USART1 RX path in init:
  - removed `RE`, `RXNEIE`, `IDLEIE` from `USART1_CR1`
- Removed `platform_irq_enable(USART1_IRQn, 1U)`.
- Kept LPUART1 RX path (`RNG_LPUART1_IRQn`) enabled as primary receiver.

### Why this was tested

Hypothesis: if both PA3/LPUART1 and PA10/USART1 are physically tied to the same host RX stream, each byte may be ingested twice, corrupting AT detection and COBS framing.

### Build status

- Windows build succeeded (`build.ps1`), `firmware.bin` produced (13392 bytes).
- No compile/lint errors reported for edited files.

## Bench Result After Firmware Change

- Flash succeeded.
- Boot succeeded.
- Early 921600 capture still shows startup breadcrumbs + binary frames including BOOT_URC and FAULT_URCs.
- Stage 1 probe still fails:
  - `BOOT_URC: not observed during initial window`
  - `reply to ATI: no bytes observed`
  - `reply to AT+VER?: no bytes observed`
  - `timeout waiting for response type 0x81 to req 0x01`

Conclusion: disabling USART1 RX did not resolve the issue.

## Additional Diagnostic Added

Created helper:

- `firmware/x8_lora_bootloader_helper/diag_uart_rxtx.py`

Purpose:

- Configure `/dev/ttymxc3` to 921600 8N1 raw.
- Send:
  - `ATI\r\n`
  - binary `VER_REQ`
  - binary `PING_REQ`
- Listen for reply bytes after each send.

Observed result on X8 target:

- No bytes received for ATI.
- No bytes received for VER_REQ.
- No bytes received for PING_REQ.

## Current Technical Interpretation

The TX direction from Murata->X8 is proven (startup bytes and URCs captured).
The failing direction is X8->Murata request path during active probing.

Most likely classes of failure now:

1. Host TX path issue after probe reopen/flush
- Probe open + `stty` + nonblocking read/write sequence may leave line state/timing different from early capture conditions.

2. Electrical/routing asymmetry
- X8 RX may be connected to Murata TX, but X8 TX may not be connected to Murata RX on this effective route.

3. Receiver framing/overrun on L072 side at steady-state
- Less likely, but still possible if RXNE/IDLE/error ordering drops bytes before enqueue.

## Next Actions (Priority Order)

1. Add minimal inbound-byte counters to firmware host UART RX ISR path
- Count bytes seen on LPUART1 and expose via STATS_URC.
- This distinguishes "no incoming bytes" from "incoming but parser reject".

2. Add temporary raw echo mode command in firmware
- Any received byte emits deterministic marker/echo.
- Quick truth test for physical RX viability.

3. Capture X8 TX waveform (scope/logic analyzer)
- Verify real transitions on the Murata RX pin during probe sends.
- Confirms whether host writes actually leave the SoC UART pin.

4. Pull latest `/tmp/lifetrac_p0c/method_g_stage1.log` into a timestamped `bench-evidence` folder
- Keep all subsequent runs under canonical evidence paths.

## Files Modified This Pass

- `firmware/murata_l072/host/host_uart.c`
- `firmware/x8_lora_bootloader_helper/diag_uart_rxtx.py`
- `TODO.md`
- `DESIGN-CONTROLLER/TODO.md`
