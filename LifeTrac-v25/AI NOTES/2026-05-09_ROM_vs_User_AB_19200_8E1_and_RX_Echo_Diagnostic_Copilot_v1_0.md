# 2026-05-09 ROM vs User A/B at 19200 8E1 + RX Echo Diagnostic (Copilot v1.0)

## Goal
Eliminate UART configuration ambiguity by running a strict A/B sync probe with identical line settings in two boot states:

- ROM bootloader state (BOOT0 high)
- User firmware state (BOOT0 low + NRST pulse)

Both halves used `19200 8E1` and the same probe byte `0x7F`.

## A/B Probe Result
### ROM half
- Probe sent: `0x7F`
- Response length: `1`
- Response bytes: `0x79`
- Interpretation: STM32 ROM bootloader ACK observed on `/dev/ttymxc3`

### User half
- Probe sent: `0x7F`
- Response length: `0`
- Response bytes: none
- Interpretation: No ROM ACK in user mode path

## Conclusion
With identical UART settings and byte timing, ROM state and user state behave differently:

- ROM mode: explicit ACK path exists
- User mode: no ACK/response

This further excludes parity/baud setting mismatch as the ingress blocker and reinforces a state-dependent physical routing/gating issue (mux/OE/wiring context), not host-side serial configuration.

## Firmware Diagnostic Added
A raw RX echo diagnostic was added to L072 firmware IRQ handlers:

- File changed: `firmware/murata_l072/host/host_uart.c`
- New macro: `HOST_UART_RX_ECHO_DIAG` (currently enabled)
- Behavior: any byte received on either LPUART1 or USART1 RX is echoed non-blocking to both TX lanes directly in IRQ context before normal parser ingest.

This enables a definitive field test:

1. Send bytes from X8 on `/dev/ttymxc3`.
2. If any byte reaches L072 RX, host should immediately see echo bytes back.
3. If no echo appears, ingress still does not electrically reach active RX path.

## Build Validation
Firmware rebuilt successfully after the diagnostic patch.

- Build script: `firmware/murata_l072/build.ps1`
- Status: success
- Output artifact size: `build/firmware.bin = 14880 bytes`

## Notes
One ROM-half run printed a log-redirection permission warning for `/tmp/ab_rom_openocd.log`, but the ROM probe still returned definitive `0x79` ACK. The user-half probe returned zero bytes under the same serial settings.
