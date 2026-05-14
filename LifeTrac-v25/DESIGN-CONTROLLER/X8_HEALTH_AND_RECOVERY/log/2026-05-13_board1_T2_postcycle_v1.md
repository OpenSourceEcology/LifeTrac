# 2026-05-13 — Board 1 (`2D0A`) T2 cold power cycle PASS

**Routine:** [T2_cold_power_cycle.md](../recovery/T2_cold_power_cycle.md) → [HC-01](../routines/HC-01_adb_and_usb_enumeration.md) → [HC-02](../routines/HC-02_linux_bridge_full.md)

**Trigger:** Overnight disconnect (12 h, see 2026-05-13 board1 HC-01 log) had
**failed** to clear the W2-01 camera-induced soft hang. User then performed a
deliberate cold cycle (USB-C + 12V unplug) on Board 1.

## HC-01 result

```
List of devices attached
2D0A1209DABC240B       device transport_id:2
2E2C1209DABC240B       device transport_id:1
```

Both boards `device`. PASS.

## HC-02 result (Board 1)

| # | Check | Value | Verdict |
|---|---|---|---|
| 1 | uptime | `up 1 min` | PASS (fresh boot) |
| 2 | x8h7 modules | all 9 loaded (`adc, can, drv, gpio, h7, pwm, rtc, uart, ui`) | PASS |
| 3 | `x8h7_drv` Used by | 8 | PASS |
| 4 | `m4-proxy.service` | active | PASS |
| 5 | `stm32h7-program.service` | active | PASS |
| 6 | gpio8/10/15 | not yet exported | EXPECTED (Stage 1 launcher exports them) |
| 7 | `/dev/ttymxc3` | crw-rw---- root dialout 207,19 | PASS |
| 8 | `/dev/watchdog0` | present | PASS |
| 9 | dmesg err/warn | anx7625 USB-C, brcmfmac scan timeouts, cs42l52 regmap, mipi_csi missing port | PASS (all known benign) |
| 10 | x8h7 fw version | `0.0.5-next-53df799-20250314:072359` | PASS (read returned a string this time, also fine) |
| 11 | disk `/` | 13% used (12 G free) | PASS |
| 12 | RAM | 1377 M free | PASS |

## Conclusion

T2 cold cycle restored Board 1 to a healthy operational state. The W2-01
post-camera IOMUXC wedge is **cleared**. Board 1 is now ready for HC-03
(SWD attach) and HC-04 (Stage 1 standard quant) before resuming the
W1-10b 100-cycle air-link campaign.

## Raw log

`LifeTrac-v25/AI NOTES/2026-05-13_board1_healthcheck_postT2_v1_0.log`
