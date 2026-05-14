# Software Reset Index — every non-physical reset path

**Purpose:** complete catalogue of every reset / restart / re-init that can be
triggered **without** unplugging power, removing a cable, flipping a DIP, or
pressing the carrier RESET button. Drawn from every routine, recovery tier,
and helper script currently in this folder and in
`firmware/x8_lora_bootloader_helper/`.

Use this as the **menu** for "what can I still do from the keyboard before I
have to walk to the bench?" Order roughly = least invasive → most invasive.
Cross-references show where each one is already wired into a Tier (T0..T3) or
Health Check (HC-xx).

> **Hard truth (2026-05-12 / 2026-05-13):** none of the soft resets below
> recover the i.MX IOMUXC drift that follows a camera-induced ci_hdrc-imx
> wedge. Once Stage 1 reports `cannot read IDR` and HC-03 shows `srst 0`,
> only T2 (cold power cycle) or T3a (SDP reflash) clear the latched pad
> state. Document any new exceptions here as they're discovered.
>
> **2026-05-13 update:** A standard ~10 s T2 cold cycle on Board 1 cleared the
> Linux soft hang (HC-01/HC-02 PASS) but did **NOT** clear the SWD wedge
> (HC-04 Stage 1 still 3/3 FAIL_SYNC; HC-03 now shows `srst 1` after preflight
> but DP IDR still unreadable — the pad drift has migrated from gpio10 to
> gpio8/gpio15). T2 may need a longer hold time, or T3a SDP reflash may be
> the only path. Tracked in [T2_cold_power_cycle.md](T2_cold_power_cycle.md)
> and [../routines/HC-03_h7_swd_attach_diff.md](../routines/HC-03_h7_swd_attach_diff.md).
>
> **2026-05-13 update #2:** A long-hold T2 (≥60 s, both 12 V and USB-C
> unplugged) on Board 1 also failed identically — same `cannot read IDR`,
> same Stage 1 FAIL_SYNC (RUN_ID `T6_stage1_standard_quant_2026-05-13_154202_460-46560`).
> **T2 is therefore conclusively insufficient for this wedge class on Board 1.**
> T3a SDP/uuu reflash is now the only proven recovery path.
>
> **2026-05-13 update #3 — IOMUXC HYPOTHESIS DISPROVEN:** T3a SDP/uuu reflash on
> Board 1 succeeded (12/12 stages, fresh LmP `4.0.11-934-91`, all 9 x8h7 modules,
> services active, x8h7_can/rtc/uart functional over SPI bridge — confirms H7
> firmware is alive and SPI side works), **but HC-03 SWD attach STILL fails**
> with `cannot read IDR`. A fresh boot + fresh kernel + fresh pinmux + fresh
> userland did NOT clear the wedge. **The fault is therefore not a Linux-side
> IOMUXC drift** — it is a hardware-side or H7-firmware-side fault on the
> SWD pad path (i.MX gpio8/gpio15 ↔ H7 PA13/PA14) that `uuu` does not touch.
> See [../log/2026-05-13_board1_T3a_postreflash_v1.md](../log/2026-05-13_board1_T3a_postreflash_v1.md).

---

## Layer 0 — Host PC only (no traffic to the X8)

| # | Mechanism | Command | Resets | Tier | Efficacy |
|---|---|---|---|---|---|
| 0.1 | adb daemon kick | `adb kill-server; adb start-server; adb devices -l` | Host adb server only | [T0](T0_adb_daemon_kick.md) | Fixes stale Windows transport entries; never fixes a board-side hang. |
| 0.2 | adb reconnect | `adb reconnect` (or `adb -s <serial> reconnect`) | Host↔device transport on an already-attached device | T0 | Useful when `adb devices` shows `offline` but the device is still listed. |
| 0.3 | Windows PnP cycle | `Disable-PnpDevice -InstanceId ...; Enable-PnpDevice -InstanceId ...` | Windows WinUSB binding for the X8 composite | T1 (host-side) | Helps only when the host driver bind is what broke. Rare on this stack. |
| 0.4 | Drain host COM port | open + `DtrEnable=$true` toggle on `COMnn`, then close | Per-port DTR/RTS lines (no effect on X8 if X8 ignores those) | implicit in `diagnose_x8_recovery.ps1` | Diagnostic only. |

---

## Layer 1 — X8 Linux userland (adb still alive)

| # | Mechanism | Command (run via `adb -s <serial> exec-out`) | Resets | Tier / file | Efficacy |
|---|---|---|---|---|---|
| 1.1 | `m4-proxy` restart | `echo fio \| sudo -S systemctl restart m4-proxy.service` | userspace ↔ M4 proxy only | implicit in HC-02 fail row | Cheap; first thing to try when bridge is partially alive. |
| 1.2 | `stm32h7-program` restart | `echo fio \| sudo -S systemctl restart stm32h7-program.service` | re-pushes the H7 SPI-bridge firmware via SWD; **WILL trigger watchdog auto-reboot** if the x8h7 driver stack is wedged | [T1](T1_bridge_rmmod_cascade.md) | Proven: the watchdog reboot is the actual recovery on 2026-05-08. Does NOT clear IOMUXC. |
| 1.3 | adbd restart over serial console | log in on `COMnn` as `fio/fio`, then `sudo systemctl restart adbd` | adbd only | T1 (host-side) | Only reachable when the COM getty is alive. Board 1 2026-05-12: getty also dead, so unreachable. |
| 1.4 | `prep_bridge.sh` (rmmod cascade) | push `firmware/x8_lora_bootloader_helper/prep_bridge.sh`, `echo fio \| sudo -S sh /tmp/prep_bridge.sh` | unexport gpio 160-193, unbind `cs42l52_regulator`, `rmmod x8h7_*` (9 modules) | [T1](T1_bridge_rmmod_cascade.md), references `prep_bridge.sh` | Required preflight before any `openocd` halt that would otherwise wedge the bridge. |
| 1.5 | `revive_bridge.sh` (rmmod + insmod loop) | push `firmware/x8_lora_bootloader_helper/revive_bridge.sh`, run as root | rmmod cascade + `load_modules_pre.sh` + `load_modules_post.sh` + `cs42l52_regulator` rebind | not in any Tier (documented INEFFECTIVE in `2026-05-12_X8_Board1_Recovery_Plan_Copilot_v1_0.md`) | Empirically fails to keep the bridge alive long-term. Kept for completeness; do not rely on. |
| 1.6 | Single-module reload | `sudo modprobe -r x8h7_<mod>; sudo modprobe x8h7_<mod>` | one driver only | not currently used | UNTESTED — `prep_bridge.sh` rmmod's the whole tree because individual unloads usually fail with EBUSY due to `industrialio` and `cs42l52_regulator` refs. |
| 1.7 | GPIO sysfs unexport / re-export | `echo <n> > /sys/class/gpio/unexport`, `echo <n> > /sys/class/gpio/export` | one GPIO line's sysfs handle (does NOT alter the underlying pinmux) | used inside `prep_bridge.sh`, `manual_gpio_preflight.sh` | Frees module refcounts so `rmmod` can succeed. Has no effect on i.MX IOMUXC. |
| 1.8 | `systemctl reboot` | `echo fio \| sudo -S systemctl reboot` | full Linux reboot, clean shutdown of services first | not currently in any Tier doc | Cleanest soft reboot. Should be tried before T1's watchdog-induced reboot when adbd is healthy. **GAP — promote to a documented step.** |
| 1.9 | `reboot` / `reboot -f` | `echo fio \| sudo -S reboot -f` | force-reboot bypassing systemd shutdown ordering | not currently in any Tier | Use when systemd is too sick to coordinate shutdown. **GAP.** |
| 1.10 | SysRq trigger | `echo b > /proc/sysrq-trigger` (immediate reboot), `echo c` (panic+reboot if `kernel.panic` set), `echo s` (sync), `echo u` (remount RO) | kernel-level reboot independent of userspace | not currently in any Tier | Requires `kernel.sysrq != 0`; worth probing on this LmP image. **GAP.** |
| 1.11 | `kill -9` of adbd | `pkill -9 adbd` (systemd respawns it) | one daemon | not currently in any Tier | Quicker than `systemctl restart adbd` when ADB just needs a kick and adbd is still pet-able. **GAP.** |
| 1.12 | USB gadget soft-disconnect | `echo 0 > /sys/class/udc/<udc>/soft_connect; echo 1 > /sys/class/udc/<udc>/soft_connect` | forces host re-enumeration of the X8 composite without rebooting Linux | not currently in any Tier | Useful when Linux is healthy but Windows transport is wedged. **GAP — verify UDC name on this image.** |
| 1.13 | Kill watchdog petter to force WDT reboot | `bash wdt_pet.sh stop` then stop kernel `[watchdogd]` somehow | i.MX2+ HW watchdog fires after 60 s → full reboot | helper exists (`wdt_pet.sh`) but explicitly listed under "What NOT to do" in `2026-05-12_X8_Board1_Recovery_Plan.md` | The kernel's `[watchdogd]` keeps petting it; can't be reliably triggered from the keyboard. |

---

## Layer 2 — STM32H747 (H7 + M4) on the X8 module

| # | Mechanism | Command | Resets | Tier / file | Efficacy |
|---|---|---|---|---|---|
| 2.1 | H7 NRST pulse via gpio10 | push `firmware/x8_lora_bootloader_helper/pulse_h7_nrst.sh`, `sudo sh /tmp/pulse_h7_nrst.sh` (drives `/sys/class/gpio/gpio10/value` 0 → sleep 0.2 → 1) | hard NRST of STM32H747 (both H7 and M4 cores) | referenced in HC-03 follow-up; **GAP — promote to its own Tier (e.g. T0.5)** | Works only if i.MX IOMUXC has `gpio10` muxed as GPIO. The whole point of the W2-01 wedge is that this **fails** when pad is in alt-function. |
| 2.2 | H7 NRST pulse via openocd `reset run` | `openocd -f /usr/arduino/extra/openocd_script-imx_gpio.cfg -c "init; reset run; shutdown"` | hard NRST of STM32H747 + clean release of SWD | used by `revive_bridge.sh` step [2/5] | Same IOMUXC dependency as 2.1. |
| 2.3 | H7 sysreset via openocd | `openocd -f ... -c "init; reset; resume; shutdown"` (uses ARM SYSRESETREQ when NRST mode is `srst_only` off) | core-only reset on STM32H747; peripherals depend on cfg | available via openocd cfgs `30..33_boot_user_delay_*ms.cfg` etc. | Less invasive than NRST; needs healthy SWD attach (precondition: HC-03 PASS). |
| 2.4 | `monitor-m4-elf-file.path` triggered reload | drop a new `.elf` into the watched path (`/usr/arduino/m4/`) | systemd path-unit re-loads M4 firmware via `m4-proxy` | discoverable via `find_watchdog.sh` | Nominal M4 update path; not a recovery in itself. |
| 2.5 | re-push x8h7-firmware via openocd flash | `openocd -f openocd_script-imx_gpio.cfg -c "program ..."` | overwrites H7 internal flash with the SPI-bridge firmware | not in current Tiers (would be a "T1.5") | Heavy hammer for an H7 firmware corruption case. **GAP.** |

---

## Layer 3 — Murata CMWX1ZZABZ (STM32L072 + SX1276)

| # | Mechanism | Command | Resets | Tier / file | Efficacy |
|---|---|---|---|---|---|
| 3.1 | L072 reset via gpio163 (LoRa NRST through x8h7_gpio) | `echo 1 > /sys/class/gpio/gpio163/value; echo 0 > .../value; sleep 0.05; echo 1 > .../value` | NRST of the L072 (and indirectly SX1276 if it's wired off the L072) | implicit in `revive_bridge.sh` verify step [5/5] (writes gpio163 once); **GAP — no standalone Tier doc for "kick the LoRa modem"** | Requires healthy x8h7 bridge. Cheap when bridge works. |
| 3.2 | L072 BOOT0/NRST sequence via openocd-on-H7 (BLS entry) | `openocd -f openocd_script-imx_gpio.cfg -f 08_boot_user_app.cfg` (release BOOT0 + pulse NRST), or any of the `30..33_boot_user_delay_*ms.cfg` variants | enters / exits AN3155 system bootloader on L072 | used by `boot_and_probe.sh`, `boot_then_at.sh`, the `run_stage1_standard_quant_end_to_end.ps1` pipeline | This is the **canonical** L072 reset path during firmware flashing. |
| 3.3 | LoRa MAC layer reset (AT command) | `printf 'AT+REBOOT\r\n' > /dev/ttymxc3` (or `AT+FACT?`) | resets MKRWAN stack on L072 without re-flashing | used implicitly in `boot_then_at.sh` AT probe | Only works when current L072 firmware exposes the AT shell. |

---

## Layer 4 — i.MX 8M Mini (the X8 SoC itself)

| # | Mechanism | Command | Resets | Tier / file | Efficacy |
|---|---|---|---|---|---|
| 4.1 | Linux reboot (via 1.8 / 1.9 / 1.10) | see Layer 1 | full SoC re-boot from eMMC, **but NOT a power-on-reset** — IOMUXC retention behaviour applies | T1 (indirectly) | Does **not** clear post-camera IOMUXC drift on Board 1 (proven 2026-05-13 after overnight disconnect). |
| 4.2 | `kexec` into a recovery kernel | `kexec -l <kernel> --initrd=<initrd> --command-line=...; kexec -e` | jumps to a fresh kernel without re-running U-Boot, mask-ROM, or hitting the i.MX reset controller | not currently used | UNTESTED on this LmP image; same IOMUXC-non-clearing limitation as 4.1. **GAP — research only.** |
| 4.3 | i.MX `WDOG_B` self-reset via watchdog daemon stop | requires both `wdt_pet.sh stop` AND killing kernel `[watchdogd]` | full SoC reset including IOMUXC (POR-equivalent for most blocks) | NOT REACHABLE from soft path (kernel watchdogd is unkillable from userspace) | Theoretically the only soft path that resets IOMUXC. In practice: not achievable. |
| 4.4 | SDP entry without DIP flip | requires asserting `BOOT_MODE` straps via the carrier's BMP firmware | i.MX mask-ROM SDP mode (Layer 5 below) | not implemented; would need carrier firmware change | **GAP — investigate whether the on-carrier J-Link can drive the BOOT_MODE pins.** Would convert T3a from a physical to a soft procedure. |

---

## Layer 5 — Mask-ROM (only via DIP flip = no longer "soft")

Listed for completeness; everything in this layer requires walking to the
bench. See [T3a_sdp_uuu_reflash.md](T3a_sdp_uuu_reflash.md) and
[T3b_sdp_ram_rescue.md](T3b_sdp_ram_rescue.md).

---

## Summary — what we currently have vs. what is missing

**Already scripted and tier-documented:**
- 0.1, 0.2 → T0
- 1.2, 1.4 → T1
- 2.1 (script exists; only referenced as HC-03 follow-up)
- 2.2, 3.1 (used inside `revive_bridge.sh`)
- 3.2, 3.3 (entire Stage 1 pipeline depends on these)

**Documentation gaps (worth promoting to standalone steps):**
1. **`systemctl reboot`** (1.8) and **`reboot -f`** (1.9) — should be a documented "T0.5" between T0 and T1; tried before T1's watchdog-induced reboot.
2. **SysRq triggers** (1.10) — verify `kernel.sysrq` setting; if enabled, `echo b` is the cleanest soft reboot when systemd is wedged.
3. **`pkill -9 adbd`** (1.11) — quicker variant of T0/1.3.
4. **USB gadget `soft_connect` toggle** (1.12) — clears Windows-side transport without rebooting Linux. Needs UDC name discovery on this image.
5. **`pulse_h7_nrst.sh` (2.1)** — script exists, used during HC-03; promote to its own **T0.7 — H7 NRST pulse** so future operators see it as a discrete option.
6. **L072 `gpio163` NRST** (3.1) — currently buried inside `revive_bridge.sh`; deserves a one-liner doc as **T0.8 — L072 reset**.
7. **`AT+REBOOT` over `/dev/ttymxc3`** (3.3) — non-destructive LoRa-stack reset; document as **T0.9**.
8. **Single-module `modprobe -r x8h7_<mod>`** (1.6) — UNTESTED, document only after a bench attempt.
9. **`kexec` recovery** (4.2) — research only; do not promote without a known-good rescue kernel.
10. **Carrier BMP-driven BOOT_MODE strap** (4.4) — research item; would soft-ify T3a if feasible.

**Cannot be made into soft resets (documented for "do not waste time on"):**
- 1.13 / 4.3 (kill watchdog petter) — kernel `[watchdogd]` is unkillable.
- Any path that requires clearing i.MX IOMUXC pad-retention without POR — confirmed empirically on Board 1 across 4 soft attempts + 12 h overnight disconnect.

---

## When to use which layer

```
adb shows board?  ── no ──► Layer 0 (T0). If still no, jump to Layer 4 cold cycle (T2).
        │ yes
        ▼
Linux services healthy?  ── no ──► Layer 1: try 1.8 (systemctl reboot) → 1.4 (prep_bridge) → 1.2 (T1).
        │ yes
        ▼
H7 / x8h7 bridge healthy (HC-02)? ── no ──► Layer 2: 2.2 (openocd reset run) or 2.1 (NRST pulse).
        │ yes
        ▼
LoRa modem responding (AT)? ── no ──► Layer 3: 3.3 (AT+REBOOT) → 3.1 (gpio163 NRST) → 3.2 (BLS entry).
        │ yes
        ▼
Stage 1 still failing?  ── yes ──► IOMUXC drift suspected → Layer 4 (T2 cold cycle) — no soft path will fix it.
```

---

## Cross-references

- [README.md](../README.md) — folder index.
- [T0_adb_daemon_kick.md](T0_adb_daemon_kick.md), [T1_bridge_rmmod_cascade.md](T1_bridge_rmmod_cascade.md), [T2_cold_power_cycle.md](T2_cold_power_cycle.md), [T2_5_carrier_power_sanity.md](T2_5_carrier_power_sanity.md), [T3a_sdp_uuu_reflash.md](T3a_sdp_uuu_reflash.md), [T3b_sdp_ram_rescue.md](T3b_sdp_ram_rescue.md).
- [HC-02_linux_bridge_full.md](../routines/HC-02_linux_bridge_full.md), [HC-03_h7_swd_attach_diff.md](../routines/HC-03_h7_swd_attach_diff.md).
- Helper scripts: `firmware/x8_lora_bootloader_helper/` — `prep_bridge.sh`, `revive_bridge.sh`, `pulse_h7_nrst.sh`, `wdt_pet.sh`, `manual_gpio_preflight.sh`, `boot_and_probe.sh`, `boot_then_at.sh`, `find_watchdog.sh`, `diagnose_x8_recovery.ps1`.
