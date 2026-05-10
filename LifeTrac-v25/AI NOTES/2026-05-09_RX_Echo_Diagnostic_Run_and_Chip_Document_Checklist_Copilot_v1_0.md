# 2026-05-09 RX Echo Diagnostic Run + Chip Document Checklist (Copilot v1.0)

## Session Objective
1. Flash a diagnostic L072 firmware build that echoes every received UART byte directly from IRQ handlers.
2. Boot into user app mode and test whether host-injected bytes are echoed back.
3. Define exactly which vendor manuals/documents should be consulted next for hardware-path resolution.

## Execution Summary
### A. Diagnostic firmware flash
- Image flashed: `/tmp/lifetrac_p0c/diag_rx_echo.bin` (14880 bytes)
- Flash pipeline: `/tmp/lifetrac_p0c/run_flash_l072.sh`
- Result: success (`flasher exit code = 0`)
- Timing: flash complete in ~11.2s

### B. User-mode echo probe
- Boot mode: user app hold (`09_boot_user_app_hold.cfg`)
- Probe script: `/tmp/echo_probe_user_mode.py`
- Serial mode: `921600 8N1`
- Injected pattern: `c3 5a a7 1e 33 cc`
- Received bytes: `475` bytes (boot/URC traffic present)
- Pattern detected in RX stream: `False`

### C. Interpretation
- The board is alive and transmitting (many bytes received from firmware), but injected host pattern is still not echoed.
- Since echo occurs at RX IRQ entry before parser logic, absence of echoed pattern indicates no host byte arrival at active L072 RX path in user mode.
- This is consistent with prior zero `host_rx_bytes` / zero UART error-delta evidence.

## Do We Need Manuals/Documents?
Yes. At this stage, targeted vendor documentation is necessary to close the remaining routing/gating uncertainty.

## Required Documents (Priority Order)
1. Max Carrier schematic and netlist-level pin mapping
- Need exact electrical path for i.MX8 UART4_TXD to Murata L072 RX pin(s)
- Need any buffer, level shifter, analog switch, mux, or OE net on this path

2. STM32H747 reference manual + Max Carrier firmware GPIO ownership mapping
- Need definitive ownership/state of PA_11 and any other LoRa-side control pins during/after OpenOCD scripts
- Need whether any control pin can disable one TX direction while leaving reverse direction alive

3. Murata CMWX1ZZABZ-078 hardware integration material
- Need module pin mapping for USART/LPUART options and any boot/alt function caveats
- Need confirmation of which UART peripheral/pins are used by ROM bootloader vs user firmware wiring on this board

4. NXP i.MX8M Mini UART4 chapter + board DTS/pinctrl docs
- Need any TX gating dependencies, pad control edge cases, or DMA behaviors that can present write-complete without physical line transition

## Recommended Next Investigation Steps
1. Correlate schematic net names to software symbols:
- `UART4_TXD` -> intermediate components -> Murata RX net
- Identify any OE/SEL/EN pin and who drives it (H7, PMIC GPIO, fixed strapping)

2. Capture hardware-level state during user mode:
- Read/trace controlling GPIO states around the suspected buffer/mux path before and after `08/09` boot scripts

3. If no controllable OE exists in docs:
- Treat as likely board routing mismatch between ROM-entry path and user-firmware expected RX pins, and test alternative RX pin/peripheral mapping hypotheses in firmware.

## Bottom Line
The diagnostic echo firmware removed parser ambiguity and still found no host-byte ingress echo in user mode. Documentation lookup is now not optional but required for the hardware routing/gating closure step.
