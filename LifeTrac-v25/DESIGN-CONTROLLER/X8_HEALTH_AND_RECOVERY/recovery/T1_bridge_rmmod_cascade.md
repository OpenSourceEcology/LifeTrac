# T1 — Linux soft-recovery (rmmod cascade + service restart)

**Tier:** 1 (board still adb-reachable; want to clean up bridge without unplug)

**When to try:** `x8h7_*` modules wedged but adb still alive. E.g. `cat
/sys/kernel/x8h7_firmware/version` blocks, `m4-proxy` shows as failed, or you
need to flip H7 firmware without a physical reset.

**Side effect:** Triggers the i.MX2+ HW watchdog (60 s, no MAGICCLOSE) **auto-
reboot** of the X8. The reboot itself is the recovery; modules come back fresh
on the other side.

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
