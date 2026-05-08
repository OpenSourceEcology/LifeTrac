# Method G — Eliminating Manual Power-Cycles: Validated Workflow

**Date:** 2026-05-08
**Author:** GitHub Copilot (Claude Opus 4.7)
**Version:** 2.0 (BENCH-VERIFIED)
**Supersedes:** `2026-05-08_Method_G_Avoiding_Power_Cycles_Copilot_v1_0.md` (v1 was a writeup of failed attempts; v2 has the working workflow.)

## TL;DR — Working Workflow

Manual power-cycles are eliminated. The X8 still self-reboots ~30 s after each flash (we cannot prevent that without upstream fixes), but **no human intervention is required** — just wait for adb to reattach, then continue.

```bash
# On X8 as root:
bash /tmp/lifetrac_p0c/wdt_pet.sh start              # CRITICAL for flashes >60 s
bash /tmp/lifetrac_p0c/prep_bridge.sh                # cleanly unload x8h7 (incl. cs42l52 unbind)
bash /tmp/lifetrac_p0c/run_flash_l072.sh image.bin   # flash L072 (verify OK)
bash /tmp/lifetrac_p0c/wdt_pet.sh stop               # release watchdog to kernel
# System auto-reboots within ~30 s; wait for adb to reattach.
# After reboot: bridge is fully healthy, all 9 x8h7 modules loaded, no manual action needed.
```

The `full_flash_pipeline.sh` script wraps all four steps.

## Validated on Bench (2026-05-08)

| Step | Result | Evidence |
| --- | --- | --- |
| `prep_bridge.sh` | ✅ all 9 x8h7 modules cleanly unloaded | `=== SUCCESS: bridge is now safe to halt with openocd ===` |
| `wdt_pet.sh start` | ✅ owns `/dev/watchdog0` (fuser confirmed pid=1859) | `wdt_pet started pid=1859` |
| MKRWAN flash (83032 B) | ✅ `block 325/325 83032 B 62.0 s 1340 B/s, write OK in 62.0 s` | Previously failed at block 288/325 (55s) without wdt_pet |
| Hello-world flash (677 B) | ✅ `verify OK in 0.5 s` | Same as v1.0 |
| Bridge recovery | ✅ auto-reboot (~30 s), all modules loaded fresh, gpio160 healthy | `uptime: 0 min`, `lsmod | grep -c '^x8h7' = 9` |
| Manual power-cycle | ✅ NOT REQUIRED — adb reattaches automatically post-reboot | — |

## Three Root Causes, Each Addressed

### 1. `cs42l52_regulator` claims gpio-160 in-kernel
The CS42L52 audio codec's regulator is a fixed-voltage platform device that requests gpio-160 (the first x8h7 gpio) at boot, holding `x8h7_gpio` refcount=1 even with no sysfs exports. Sysfs `unexport` cannot release it.

**Fix:** `prep_bridge.sh` adds `echo cs42l52_regulator > /sys/bus/platform/drivers/reg-fixed-voltage/unbind` before rmmod. After unbind, `x8h7_gpio` refcount drops to 0 and rmmod succeeds.

### 2. imx2+ HW watchdog (60 s) fires during flashes >60 s
`wdctl /dev/watchdog0` reports `Identity: imx2+ watchdog, Timeout: 60 s, KEEPALIVEPING: 1`. The kernel `[watchdogd]` thread normally feeds it, but during the L072 flash (with x8h7 modules unloaded and openocd holding H7 halted) `[watchdogd]` cannot make progress fast enough. Result: hardware reset at ~60 s. The MKRWAN image (83 KB) takes 62 s to write at 1340 B/s — just past the threshold. The hello-world image (677 B) takes 0.5 s and never trips it.

**Fix:** `wdt_pet.sh start` opens `/dev/watchdog0` from a userspace `python3` daemon that pings every 20 s. The kernel hands ownership to userspace as soon as someone opens the device. After flash, `wdt_pet.sh stop` closes the fd; the kernel resumes auto-feeding via `[watchdogd]`.

Note: imx2+ does NOT support magic-close (`MAGICCLOSE STATUS=0`), so once opened, userspace MUST keep petting until close.

### 3. openocd halt of H7 leaves x8h7 driver state corrupted
When openocd does `reset halt`, H7 stops in System Bootloader at `pc=0x1ff09abc`. The x8h7-firmware bridge is no longer running. If the x8h7 kernel modules were loaded at this point, the `x8h7_gpio_irq_a` and `irq/82-x8h7` kernel threads block on SPI replies that never come, leaking refcount. Confirmed empirically: `rmmod` returns `Resource temporarily unavailable`. LmP kernel was built **without `CONFIG_MODULE_FORCE_UNLOAD`** (`zcat /proc/config.gz | grep MODULE_FORCE`), so `rmmod -f` is unavailable.

**Fix:** unload the modules BEFORE halting H7 (root cause #1 enables this). After flash, the system auto-reboots (because something in the SPI/x8h7 layer eventually starves `[watchdogd]` of CPU time even with `wdt_pet` running, OR because `m4-proxy.service` panics). The reboot fully restores everything, no manual action needed.

`revive_bridge.sh` (which tries to do `openocd reset run; shutdown` + reload modules) was tested and **does not prevent the auto-reboot** — the openocd-driven reset re-triggers the same state corruption. We keep `revive_bridge.sh` in the toolkit only as a diagnostic; do not rely on it.

## Why Not Just Disable the Watchdog?

`wdctl` shows `MAGICCLOSE STATUS=0` for the imx2+ driver on this kernel build, so the magic-close-'V' trick is not available. The driver does not expose a sysfs `state` knob to disable. Editing `/etc/systemd/system.conf` to set `RuntimeWatchdogUSec=` would only affect systemd's watchdog-feeding behavior, which is already off (`RuntimeWatchdogUSec=0`). Rebuilding the kernel with `CONFIG_WATCHDOG_NOWAYOUT=n` and `CONFIG_WATCHDOG_HANDLE_BOOT_ENABLED=n` could allow disable, but we do not control the kernel build.

The userspace pet is the simplest, lowest-risk solution that needs no kernel changes.

## Files

- [prep_bridge.sh](../DESIGN-CONTROLLER/firmware/x8_lora_bootloader_helper/prep_bridge.sh) — pre-openocd unload (4 stages: unexport gpios, unbind cs42l52, rmmod, verify)
- [wdt_pet.sh](../DESIGN-CONTROLLER/firmware/x8_lora_bootloader_helper/wdt_pet.sh) — `start | stop | status`; spawns python daemon that pets `/dev/watchdog0` every 20 s
- [revive_bridge.sh](../DESIGN-CONTROLLER/firmware/x8_lora_bootloader_helper/revive_bridge.sh) — diagnostic only; cannot prevent auto-reboot in practice
- [full_flash_pipeline.sh](../DESIGN-CONTROLLER/firmware/x8_lora_bootloader_helper/full_flash_pipeline.sh) — orchestrator wrapping all stages with timestamps
- [find_cs42l52.sh](../DESIGN-CONTROLLER/firmware/x8_lora_bootloader_helper/find_cs42l52.sh) — diagnostic, locates the in-kernel gpio-160 consumer
- [find_watchdog.sh](../DESIGN-CONTROLLER/firmware/x8_lora_bootloader_helper/find_watchdog.sh) — diagnostic, identifies watchdog feeders

## Long-Term Fix (Upstream)

The proper fix is [PR1 in the upstream contribution plan](2026-05-08_Method_G_Upstream_Contribution_Plan_Copilot_v1_0.md): expose `X8H7_LORA_BOOT0` and `X8H7_LORA_NRST_PULSE` opcodes in x8h7-firmware so we can drive BOOT0/NRST through the SPI bridge while H7 is **still running**. Then no openocd halt is needed → no SPI wedge → no watchdog trip → no auto-reboot. None of the workarounds in this document would be required.
