# T1 — Linux soft-recovery (rmmod cascade + service restart)

**Tier:** 1 (board still adb-reachable; want to clean up bridge without unplug)

**When to try:** `x8h7_*` modules wedged but adb still alive. E.g. `cat
/sys/kernel/x8h7_firmware/version` blocks, `m4-proxy` shows as failed, or you
need to flip H7 firmware without a physical reset.

**Side effect:** Triggers the i.MX2+ HW watchdog (60 s as u-boot arms it; with
the [`firmware/x8_recovery`](../../firmware/x8_recovery/install_watchdog.sh) L2
drop-in armed, 30 s if the wedge also stops PID 1's petting, else maybe no
reboot at all. Inferred, VERIFY under PWR-1) **auto-reboot** of the X8. The reboot itself
is the recovery; modules come back fresh on the other side.

> **Corrections 2026-09-12 (flash session, see FLASH_RUNBOOK.md):** the
> watchdog is armed by u-boot on both boards and DOES honour magic close —
> write `'V'` before closing `/dev/watchdog0`; closing without it leaves the
> dog active with no petter and reboots the board 60 s later (tractor, kernel
> 5.10.93). A timeout asserts WDOG_B into the PMIC, so WRSR always reads POR.
> (2026-10-17, inferred; L2 is armed on no board yet: once PID 1 has armed
> the L2 drop-in -- at the next `daemon-reload`, daemon-reexec or boot after
> install, when `systemctl show -p RuntimeWatchdogUSec` stops reading `0` --
> it holds the single-open device at 30 s, so a second open gets `EBUSY` and
> never reaches the dog; VERIFY under PWR-1. Until then a close without `'V'`
> still reboots the board 60 s later.)
> On the base (kernel 6.1.24) re-inserting `x8h7_drv.ko` after an openocd
> session OOPSes in `spi_probe → of_irq_get`; the pipeline's
> `REVIVE_MODE=reboot` (openocd `reset run`, magic-close, `systemctl reboot`)
> replaces the module reload.

## Procedure

```powershell
$script = "LifeTrac-v25/DESIGN-CONTROLLER/firmware/x8_lora_bootloader_helper/prep_bridge.sh"
adb -s <serial> push $script /tmp/prep_bridge.sh
adb -s <serial> exec-out "echo fio | sudo -S sh /tmp/prep_bridge.sh"
# Then either wait ~30 s for auto-reboot, OR:
adb -s <serial> exec-out "echo fio | sudo -S systemctl restart stm32h7-program.service"
```

`prep_bridge.sh` (4 stages):
1. `unexport gpio8/10/15` (release SWD lines)
2. `unbind cs42l52_regulator` (audio codec holds gpio-160 from boot)
3. `rmmod x8h7_*` in dependency order
4. Verify all 9 modules unloaded

## Pass criteria

After the watchdog auto-reboot:
- All 9 `x8h7_*` modules re-loaded.
- `m4-proxy.service` and `stm32h7-program.service` `active`.
- HC-02 fully green.

## Common failure modes

| Symptom | Notes |
|---|---|
| Auto-reboot, but `cannot read IDR` still on Stage 1 | T1 cleared bridge state but did **not** clear i.MX IOMUXC pad drift. Escalate to T2. |
| `systemctl restart stm32h7-program.service` errors `insmod: File exists` | Script uses `insmod` not `modprobe`; run `prep_bridge.sh` first. |
| `revive_bridge.sh` does not prevent eventual reboot | Confirmed by 2026-05-08 bench. |

## Verdict matrix

| Date | Board | Outcome | Notes |
|---|---|---|---|
| 2026-05-08 | Board 1 | PASS | First proven cycle without manual unplug. |
| 2026-05-12 | Board 1 (`2D0A`) | PARTIAL | Bridge restored, but Stage 1 still FAIL_SYNC (IOMUXC wedge persisted). |
