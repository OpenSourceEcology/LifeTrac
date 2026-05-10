# 2026-05-09 Ingress Parity Contradiction Resolution (Copilot v1.0)

## Objective
Resolve the remaining contradiction:

- ROM flasher over `/dev/ttymxc3` at `19200 8E1` can communicate with STM32L072 ROM bootloader.
- Custom firmware ingress tests at `19200 8N1` showed zero RX/errors on L072.

Primary hypothesis tested: parity mismatch (`8E1` vs `8N1`) might explain why flasher works but firmware ingress test fails.

## Method
1. Reboot L072 into user firmware using OpenOCD script:
   - `/usr/arduino/extra/openocd_script-imx_gpio.cfg`
   - `/tmp/lifetrac_p0c/08_boot_user_app.cfg`
2. Run a parity-matched ingress test (`baud_ingress_test_8e1.py`) on X8:
   - Baseline at `921600 8N1`: collect STATS_URC.
   - Inject 32 bytes `0x55` at `19200 8E1`.
   - Return to `921600 8N1` and compare counters.
3. Evaluate deltas:
   - `host_rx_bytes`
   - `host_uart_err_lpuart`
   - `host_uart_err_usart1`

## Key Runtime Observations
During baseline window after reset:

- `FAULT: code=0x03` (RADIO_INIT_FAIL)
- `BOOT_URC received`
- `FAULT: code=0x08` (CLOCK_HSE_FAILED)
- Repeated STATS with:
  - `rx=0`
  - `err_lpuart=0`
  - `err_usart1=0`

After 32-byte injection at `19200 8E1`:

- Post-injection STATS remained unchanged.
- Final deltas:
  - `Delta err_lpuart=0`
  - `Delta err_usart1=0`
  - `host_rx_bytes` unchanged at 0

## Conclusion
Parity is ruled out as the cause.

Using the same parity mode as the AN3155 flasher (`19200 8E1`) still produced **zero** UART error or RX-byte increments in custom firmware.

Therefore, the unresolved issue remains physical/routing/gating on X8 TX to L072 RX in user-firmware path, not `8N1`/`8E1` mismatch.

## Updated Interpretation
- Flasher success and firmware ingress failure are not reconciled by parity.
- Contradiction narrows to path/state differences between ROM-bootloader communication context and user-firmware communication context:
  - mux/level-shifter enable state,
  - board-level routing asymmetry,
  - or an external gate dependent on reset/boot state.

## Immediate Next Steps
1. Execute a strict ROM-vs-user A/B with identical line settings and timing:
   - same `stty 19200 8E1`, same write pattern, same read window.
2. Instrument L072 firmware with immediate raw-byte echo in both UART RX IRQ handlers.
3. Verify whether PA_11/BOOT0-related control state influences any TX-direction gate that persists into user-mode transitions.
4. Continue hardware path audit for any OE/mux control not currently modeled in scripts.
