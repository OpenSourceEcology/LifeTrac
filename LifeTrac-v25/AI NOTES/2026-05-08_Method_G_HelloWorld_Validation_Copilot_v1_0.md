# Method G Phase 1 — Custom Hello-World Validation (END-TO-END PROVEN)

**Date:** 2026-05-08
**Author:** GitHub Copilot (Claude Opus 4.7)
**Version:** 1.0
**Status:** ✅ COMPLETE — bare-metal STM32L072 binary built, flashed and observed running on hardware

## TL;DR

Method G is now end-to-end proven for **arbitrary custom firmware**, not just round-tripping the MKRWAN reference image.

A 677-byte stdlib-free hello-world binary (USART1 + LPUART1 dual-TX, hex tick counter on a 16 MHz HSI) was built with the Arduino-bundled `arm-none-eabi-gcc 7-2017q4`, flashed via the Method G pipeline (openocd `imx_gpio` driver halts the Portenta H7 in System Bootloader → Tcl drives `PA_11`=BOOT0 HIGH and pulses `PF_4`=NRST → AN3155 flasher over `/dev/ttymxc3` at 19200 8E1, per-page erase, 256B writes, read-back verify), and observed transmitting the boot banner and incrementing tick counter on the host UART:

```
=== LIFETRAC L072 hello v0.2 (USART1+LPUART1) ===
If you can read this, the X8 -> L072 flash pipeline works.
LIFETRAC L072 tick=0x00000000
LIFETRAC L072 tick=0x00000001
...
```

This closes the validation loop opened in [`2026-05-08_Method_G_Phase1_End_to_End_Flash_Success_Copilot_v1_0.md`](2026-05-08_Method_G_Phase1_End_to_End_Flash_Success_Copilot_v1_0.md): the X8→L072 firmware pipeline owns a custom-binary path; we are no longer limited to reflashing Murata's stock modem image.

## Artifacts produced this session

### Custom hello-world firmware
- [LifeTrac-v25/DESIGN-CONTROLLER/firmware/murata_l072_hello/main.c](../DESIGN-CONTROLLER/firmware/murata_l072_hello/main.c) — bare-metal C, no CMSIS/HAL/libc. HSI16 SYSCLK switch, GPIOA AF setup for both USART1 (PA9/PA10 AF4) and LPUART1 (PA2/PA3 AF6), polled TX. Hex tick (no division → no libgcc).
- [LifeTrac-v25/DESIGN-CONTROLLER/firmware/murata_l072_hello/stm32l072.ld](../DESIGN-CONTROLLER/firmware/murata_l072_hello/stm32l072.ld) — STM32L072CZ memory map: FLASH 192K @ 0x08000000, RAM 20K @ 0x20000000, `_estack` at 0x20005000.
- [LifeTrac-v25/DESIGN-CONTROLLER/firmware/murata_l072_hello/build.ps1](../DESIGN-CONTROLLER/firmware/murata_l072_hello/build.ps1) — PowerShell build (uses Arduino's `arm-none-eabi-gcc 7-2017q4`).

Build output: `text 661 + data 16 + bss 0 = 677 bytes`. Compile flags: `-mcpu=cortex-m0plus -mthumb -Os -ffunction-sections -fdata-sections -nostdlib -nostartfiles -fno-builtin -T stm32l072.ld -Wl,--gc-sections`.

### New X8-side helpers
- [LifeTrac-v25/DESIGN-CONTROLLER/firmware/x8_lora_bootloader_helper/run_flash_l072.sh](../DESIGN-CONTROLLER/firmware/x8_lora_bootloader_helper/run_flash_l072.sh) — now accepts the binary path as `$1` (defaults to `mlm32l07x01.bin` for backward-compat). Single canonical flash launcher for any L072 binary.
- [LifeTrac-v25/DESIGN-CONTROLLER/firmware/x8_lora_bootloader_helper/09_boot_user_app_hold.cfg](../DESIGN-CONTROLLER/firmware/x8_lora_bootloader_helper/09_boot_user_app_hold.cfg) — **critical fix**: same as `08_boot_user_app.cfg` but never calls `shutdown`. Re-asserts `PA_11`=LOW once per second for up to 600 s while the listener captures UART. Without this, the L072 reverts to ROM bootloader mode within milliseconds.
- [LifeTrac-v25/DESIGN-CONTROLLER/firmware/x8_lora_bootloader_helper/boot_and_listen_hold.sh](../DESIGN-CONTROLLER/firmware/x8_lora_bootloader_helper/boot_and_listen_hold.sh) — starts `cat /dev/ttymxc3 > rx.bin` BEFORE launching openocd, sleeps N seconds, then kills both. Replaces the silent `boot_and_listen.sh` for any post-flash bring-up.

## Hardware-confirmed observations (this session)

| Step | Result |
|------|--------|
| `build.ps1` (v0.1, USART1 only) | 532 B, decimal counter pulled `__aeabi_uidivmod` → fixed to hex shift+mask |
| `build.ps1` (v0.2, USART1+LPUART1, hex counter) | 677 B clean |
| Flash via `run_flash_l072.sh hello.bin` | sync OK, GetID 0x447, page erase 5 pages 1.7 s, write 3 blocks/677 B 0.5 s @ ~1.3 KB/s, **verify OK** RC=0 |
| `boot_and_listen.sh` (uses `08` cfg with `shutdown`) | **0 bytes captured** ❌ |
| `boot_and_listen_hold.sh` (uses `09` cfg, no `shutdown`) | **485 bytes captured ✓** — banner + 11 tick lines |

## Root-cause finding: `shutdown` releases the L072 from user-flash boot

The first three boot attempts captured 0 bytes despite a verified-correct flash image. Diagnosis:

1. The L072 vector table read back correctly (SP `0x20005000`, PC `0x0800015D` thumb).
2. After `shutdown`, openocd disconnects the SWD bridge; the H7 (still halted in System Bootloader) is no longer being actively held by openocd.
3. Empirically, this state allows `PA_11` (BOOT0) to drift / be re-driven HIGH within milliseconds — likely by an external pull-up or by H7 ROM behavior — and the L072's next NRST settles it back into the ROM bootloader instead of user flash. Even without an NRST, the L072 may have been latching its next BOOT0 sample wrongly during the very brief window between the `08` cfg's PA_11=LOW and openocd's `shutdown`.

The fix is structural: hold the H7 halted with PA_11 actively LOW for the entire observation window. The new `09_boot_user_app_hold.cfg` re-asserts the GPIOA BSRR-based clear of PA_11 every second inside a Tcl `for` loop. With this in place, the L072 stays in user-flash boot indefinitely.

**Implication for future work:** any post-Method-G operation that needs the L072 to stay in user firmware (talking AT, reading sensors, listening on UART, etc.) MUST keep the H7 halted with PA_11 LOW. A clean fix would teach the H7 default firmware to drive PA_11=LOW immediately after reset (BOOT0 should be LOW unless we explicitly want the bootloader). This is one of the upstream-contribution items in the companion plan doc.

## Reproducible command sequence (for the bench)

```powershell
# Build (Windows host, requires Arduino IDE installed)
powershell -ExecutionPolicy Bypass -File `
  "LifeTrac-v25\DESIGN-CONTROLLER\firmware\murata_l072_hello\build.ps1"

# Push (assumes /tmp/lifetrac_p0c/ already populated from prior session)
$env:Path += ";C:\Users\dorkm\AppData\Local\Android\Sdk\platform-tools"
adb -s 2E2C1209DABC240B push `
  "LifeTrac-v25\DESIGN-CONTROLLER\firmware\murata_l072_hello\hello.bin" `
  /tmp/lifetrac_p0c/hello.bin
adb -s 2E2C1209DABC240B push `
  "LifeTrac-v25\DESIGN-CONTROLLER\firmware\x8_lora_bootloader_helper\09_boot_user_app_hold.cfg" `
  /tmp/lifetrac_p0c/
adb -s 2E2C1209DABC240B push `
  "LifeTrac-v25\DESIGN-CONTROLLER\firmware\x8_lora_bootloader_helper\boot_and_listen_hold.sh" `
  /tmp/lifetrac_p0c/

# Flash (run_flash_l072.sh is already on device from prior session)
adb -s 2E2C1209DABC240B exec-out `
  "echo fio | sudo -S -p '' bash /tmp/lifetrac_p0c/run_flash_l072.sh /tmp/lifetrac_p0c/hello.bin"

# Boot + listen 8 seconds (captures banner + ~6 ticks)
adb -s 2E2C1209DABC240B exec-out `
  "echo fio | sudo -S -p '' bash /tmp/lifetrac_p0c/boot_and_listen_hold.sh 8"
```

## Known side-effect (unchanged from Phase 0)

After every openocd run, the X8 bridge stalls (`/sys/kernel/x8h7_firmware/version` returns `Connection timed out`). Subsequent flash attempts fail with `openocd did not reach READY phase` or hang the whole `adb exec-out` channel. **Workaround:** power-cycle the X8 between flash sessions. This was reproduced again at the end of this session when attempting to restore the MKRWAN reference image; the L072 currently still has `hello.bin` programmed. Restoring `mlm32l07x01.bin` is a one-command operation after a power-cycle.

## Status of stated session goals

- ✅ Build a tiny custom L072 binary that owns the entire flash (no MKRWAN dependence).
- ✅ Flash it via the existing Method G pipeline with no pipeline changes (only a parameterization of `run_flash_l072.sh`).
- ✅ Observe the binary running on hardware (UART banner + monotonic tick counter).
- ✅ Document the new `09`/`hold` boot-and-listen pattern as the canonical post-flash bring-up flow.
- ⏸ Restore MKRWAN reference image after the bridge resets (deferred to next bench session — single command).
- ➡ Companion doc: see [2026-05-08_Method_G_Upstream_Contribution_Plan_Copilot_v1_0.md](2026-05-08_Method_G_Upstream_Contribution_Plan_Copilot_v1_0.md) for the upstream-contribution roadmap so external Portenta X8 users can use this same path.
