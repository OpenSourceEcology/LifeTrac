# L072 Hello-World Build + Flash + UART Capture Success

Date: 2026-05-08
Author: Copilot (GPT-5.3-Codex)

## Objective

Complete Method G bring-up tasks:
1. Build `hello_world.bin` for L072 from canonical Makefile.
2. Flash via `run_flash_l072.sh` on X8.
3. Capture runtime UART output to confirm user firmware executes.

## Final Result

All objectives completed successfully.

- Build: PASS
- Flash/Verify: PASS (`flasher RC=0`)
- Runtime UART capture: PASS (609 bytes, banner + tick stream)

## Key Fixes Applied

1. Canonical Makefile updated with explicit hello target:
   - `DESIGN-CONTROLLER/firmware/murata_l072/Makefile`
   - Added `hello` target and `HELLO_*` vars.

2. Duplicate build file removed:
   - Deleted `DESIGN-CONTROLLER/firmware/murata_l072/Makefile.hello`

3. `hello_world.c` aligned to validated bench profile:
   - Uses HSI16 clock setup.
   - Emits at 19200 8N1 on both:
     - USART1 (PA9/PA10, AF4)
     - LPUART1 (PA2/PA3, AF6)
   - Text includes startup banner + incrementing hex tick.

4. Register header expanded:
   - `DESIGN-CONTROLLER/firmware/murata_l072/include/stm32l072_regs.h`
   - Added `RCC_CCIPR`, `RCC_APB1ENR_LPUART1EN`, and LPUART1 register macros.

## Toolchain Used

- `arm-none-eabi-gcc`:
  - `C:/Users/dorkm/AppData/Local/Arduino15/packages/arduino/tools/arm-none-eabi-gcc/7-2017q4/bin/arm-none-eabi-gcc.exe`
- make:
  - `C:/Users/dorkm/AppData/Local/Microsoft/WinGet/Packages/BrechtSanders.WinLibs.POSIX.UCRT_Microsoft.Winget.Source_8wekyb3d8bbwe/mingw64/bin/mingw32-make.exe`

## Build Command

```bash
mingw32-make -C LifeTrac-v25/DESIGN-CONTROLLER/firmware/murata_l072 -f Makefile CROSS=<arm-none-eabi-prefix> hello
```

Observed section total: `9342` bytes.

## X8 Staging + Flash Commands

Device: `2E2C1209DABC240B`

```bash
adb -s 2E2C1209DABC240B push <repo>/DESIGN-CONTROLLER/firmware/x8_lora_bootloader_helper/mlm32l07x01.bin /tmp/lifetrac_p0c/mlm32l07x01.bin
adb -s 2E2C1209DABC240B shell "cd /tmp/lifetrac_p0c; echo fio | sudo -S -p '' bash run_flash_l072.sh /tmp/lifetrac_p0c/mlm32l07x01.bin"
```

Flash evidence:
- `loaded ... 4808 bytes`
- `write OK in 3.6 s`
- `verify OK in 3.3 s`
- `flasher exit code = 0`

## UART Capture Command (Critical)

Use BOOT0-low HOLD path (not one-shot shutdown path):

```bash
adb -s 2E2C1209DABC240B shell "cd /tmp/lifetrac_p0c; echo fio | sudo -S -p '' bash boot_and_listen_hold.sh 12"
```

Why: one-shot boot script can let BOOT0 float and re-enter ROM bootloader. Hold script keeps PA_11 low while listening.

## Captured UART Evidence

`/tmp/lifetrac_p0c/rx.bin` size: `609` bytes

Hex dump head includes:
- `=== LIFETRAC L072 hello v0.2 (USART1+LPUART1) ===`
- `If you can read this, the X8 -> L072 flash pipeline works.`
- `LIFETRAC L072 tick=0x00000000`
- `... tick=0x0000000F`

This confirms the flashed custom L072 firmware is executing and streaming over the X8-visible UART path.

## Notes

- Sudo password for X8 user `fio` is required for OpenOCD path access; automation succeeded with `echo fio | sudo -S -p ''`.
- Current staged image in helper dir is the working hello-world binary.
