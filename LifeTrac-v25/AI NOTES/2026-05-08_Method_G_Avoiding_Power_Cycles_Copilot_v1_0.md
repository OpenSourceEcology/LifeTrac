# Avoiding X8 Power-Cycles Between openocd Sessions — Findings & Workaround

**Date:** 2026-05-08
**Author:** GitHub Copilot (Claude Opus 4.7)
**Version:** 1.0
**Status:** ✅ Root-caused; ⏸ workaround scripts written but NOT YET BENCH-VERIFIED (current X8 unit is wedged from earlier session)

## TL;DR

A power-cycle is required after every Method G openocd session **only because the x8h7_gpio kernel driver does not handle mid-flight bridge disconnects**. When openocd halts H7, in-flight SPI requests time out and leak refcount inside the driver. Once that happens, `rmmod` cannot remove the module and there is no software-only recovery — confirmed empirically that the LmP kernel was built **without `CONFIG_MODULE_FORCE_UNLOAD`**, so even `rmmod -f` is unavailable.

**The workaround is preemptive, not reactive:** unload the x8h7 modules BEFORE running openocd (while the bridge is still healthy), then reload them after openocd cleanly resets H7. Two new scripts implement this:

- [prep_bridge.sh](../DESIGN-CONTROLLER/firmware/x8_lora_bootloader_helper/prep_bridge.sh) — run BEFORE openocd. Unexports x8h7 gpios + rmmods all x8h7_* modules.
- [revive_bridge.sh](../DESIGN-CONTROLLER/firmware/x8_lora_bootloader_helper/revive_bridge.sh) — run AFTER openocd. Issues `reset run; shutdown` so H7 cold-boots back into x8h7-firmware, then reloads modules and verifies via gpio163.

These are designed to be called from the existing `run_flash_l072.sh` and `boot_and_listen_hold.sh` flows (TODO once next bench session can verify).

The proper long-term fix is upstream PR1 in [2026-05-08_Method_G_Upstream_Contribution_Plan_Copilot_v1_0.md](2026-05-08_Method_G_Upstream_Contribution_Plan_Copilot_v1_0.md): expose BOOT0/NRST control through the x8h7-firmware bridge protocol so we **never need to halt H7 in the first place**. Once that lands, none of this is necessary.

## Diagnostic walk

### Symptom
After any openocd session that does `init; reset halt`, `/sys/kernel/x8h7_firmware/*` and `/sys/class/gpio/gpioNNN/value` (for any x8h7 gpio) hang forever with `Connection timed out`.

### What happens during an openocd session
1. `openocd_script-imx_gpio.cfg` runs `adapter driver imx_gpio`, configures SWD on pins 15/8, SRST on pin 10, then `init; reset halt`.
2. `reset halt` pulls SRST LOW then HIGH AND immediately halts H7 before the bootloader transitions to flash. PC is left at `0x1ff09abc` (System Bootloader).
3. The `x8h7-firmware` at flash `0x08000000` is therefore not running.
4. Linux-side `x8h7_*` drivers send SPI requests; H7 does not respond.

### What gets stuck

```
=== lsmod x8h7 refcounts AFTER bridge wedge ===
x8h7_ui                16384  0
x8h7_uart              16384  0
x8h7_pwm               16384  0
x8h7_rtc               16384  0
x8h7_adc               16384  0
x8h7_can               20480  0
x8h7_h7                16384  0
x8h7_gpio              20480  2     <-- internal refcount; driver kthread holds it
x8h7_drv               16384  8     <-- 8 = sum of dependents
```

The kernel threads `[irq/82-x8h7]` and `[x8h7_gpio_irq_a]` keep running. They cannot be killed (kernel threads). `rmmod x8h7_gpio` returns `Resource temporarily unavailable`. `rmmod -f` returns `not implemented` (CONFIG_MODULE_FORCE_UNLOAD=n).

### Why NRST + module reload alone doesn't recover
Even after we pull H7 NRST via `gpio10` (Arduino's `/usr/arduino/extra/reset.sh`), the H7 firmware DOES come back up — but the in-kernel x8h7_gpio driver's session state is stale. Because we cannot rmmod it (refcount stuck), we cannot reload it to re-handshake. The driver remains zombie until the kernel is rebooted (i.e. power-cycle).

### Why pre-emptive unload works (in theory; needs bench verification)
If we rmmod the x8h7 modules BEFORE openocd halts H7, there is no driver to get stuck. The drivers come down cleanly because they're talking to a healthy bridge. After openocd's `reset run; shutdown` (which we can issue so H7 cold-boots from flash back into x8h7-firmware), we re-insmod and the drivers do a fresh handshake.

## Proposed integrated workflow

```bash
# On X8 (root):
bash /tmp/lifetrac_p0c/prep_bridge.sh                       # unload x8h7 cleanly
bash /tmp/lifetrac_p0c/run_flash_l072.sh /tmp/.../hello.bin # halt H7, flash L072
bash /tmp/lifetrac_p0c/boot_and_listen_hold.sh 8            # hold L072 in user fw
bash /tmp/lifetrac_p0c/revive_bridge.sh                     # reset run + reload modules
# Bridge alive again, no power-cycle needed.
```

Wrapping this all in a single `flash_l072_safe.sh` would be cleaner; deferred until prep+revive is bench-verified.

## Bench verification status

⏸ **NOT YET TESTED**. The current X8 unit is wedged from the hello-world session that revealed this issue. Verification plan after next power-cycle:

1. Power-cycle X8 (cold boot). Confirm bridge healthy: `cat /sys/class/gpio/gpiochip160/label` returns `x8h7_gpio`, `cat /sys/kernel/x8h7_firmware/version` may or may not work.
2. Run `prep_bridge.sh` — expect all x8h7 modules cleanly removed.
3. Run `run_flash_l072.sh /tmp/.../hello.bin` — flash should succeed.
4. Run `revive_bridge.sh` — expect `SUCCESS: bridge revived without power-cycle`.
5. Run `prep_bridge.sh` + `run_flash_l072.sh /tmp/.../mlm32l07x01.bin` + `revive_bridge.sh` (back to back, no power-cycle in between) — expect both flashes to succeed and bridge alive at end.

If step 5 succeeds, **power-cycles are eliminated from the Method G workflow**.

If step 5 fails (e.g. driver state still corrupt after reload), fallback is the proper upstream fix: x8h7-firmware-side BOOT0/NRST control (PR1), which avoids halting H7 entirely.

## Why this isn't a simple `systemctl restart`

There is a `stm32h7-program.service` that runs `program-h7.sh` at boot to flash x8h7-firmware via openocd, but it does NOT reload the kernel modules — those are loaded earlier by `monitor-m4-elf-file.path` chain via insmod from `load_modules_*.sh`. There is no service whose restart would do everything we need. Hence the explicit sequence above.

## Files added this session

- [LifeTrac-v25/DESIGN-CONTROLLER/firmware/x8_lora_bootloader_helper/prep_bridge.sh](../DESIGN-CONTROLLER/firmware/x8_lora_bootloader_helper/prep_bridge.sh)
- [LifeTrac-v25/DESIGN-CONTROLLER/firmware/x8_lora_bootloader_helper/revive_bridge.sh](../DESIGN-CONTROLLER/firmware/x8_lora_bootloader_helper/revive_bridge.sh)
- [LifeTrac-v25/DESIGN-CONTROLLER/firmware/x8_lora_bootloader_helper/diag_holders.sh](../DESIGN-CONTROLLER/firmware/x8_lora_bootloader_helper/diag_holders.sh) (debugging aid)
