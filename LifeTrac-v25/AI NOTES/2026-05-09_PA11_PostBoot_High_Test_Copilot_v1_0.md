# 2026-05-09 PA11 Post-Boot HIGH Test (Copilot v1.0)

## Objective
Test whether H7 `PA_11` state alone controls a one-way host->L072 ingress gate in user-firmware mode.

## Setup
1. Flash diagnostic L072 image with IRQ-level raw RX echo enabled (`diag_rx_echo.bin`).
2. Boot user firmware using `08_boot_user_app.cfg` (BOOT0 low + NRST pulse).
3. In a second OpenOCD invocation (no L072 reset), force `PA_11` HIGH via direct H7 GPIO register writes.
4. Send host pattern `c3 5a a7 1e 33 cc` at `921600 8N1` and look for echoed bytes.

## Verification
OpenOCD log confirms PA11 was actually forced HIGH:

- `PA11 forced HIGH, GPIOA_IDR=0x0000c800`

## Result
Echo probe output:

- `sent_hex= c35aa71e33cc`
- `pattern_found= False`
- RX stream contained normal firmware URC traffic (~432 bytes), but no echoed injected pattern.

## Conclusion
Forcing `PA_11` HIGH after user boot does **not** restore host->L072 ingress.

This refines the hypothesis:

- `PA_11` by itself is not the sole one-way TX gate control.
- Remaining likely causes are another control net (OE/SEL), different routed RX destination than expected, or a board-level path dependency not controlled by `PA_11` alone.

## Immediate Next Step
Use schematic/net mapping to identify the full UART4_TXD path and any intermediate direction-control components/nets, then test those control states directly.
