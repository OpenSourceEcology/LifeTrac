# HC-03 — H7 SWD attach diff (`imx_gpio` openocd)

**Purpose:** Verify openocd on the X8 can attach to the STM32H747 SW-DP via the
official Arduino-shipped `imx_gpio` config. Run on both boards and diff —
divergence at the `saved pinmux settings` line is the canonical wedge signature.

**When to run:** Whenever Stage 1 fails with `cannot read IDR`, or whenever you
suspect IOMUXC drift after a USB host wedge.

## Procedure

```powershell
$script = "LifeTrac-v25/DESIGN-CONTROLLER/firmware/x8_lora_bootloader_helper/probe_h7_swd_official.sh"
foreach ($s in '2D0A1209DABC240B','2E2C1209DABC240B') {
  $tag = if ($s -like '2D0A*') {'board1'} else {'board2'}
  $log = "LifeTrac-v25/AI NOTES/$(Get-Date -Format yyyy-MM-dd)_W2-01_${tag}_openocd_official_attach_v1_0.log"
  adb -s $s push $script /tmp/probe_h7_swd_official.sh 2>&1 | Out-Null
  adb -s $s exec-out "sh /tmp/probe_h7_swd_official.sh" 2>&1 |
    Out-File -FilePath $log -Encoding utf8
  Get-Content $log |
    Select-String -Pattern "DPIDR|IDR|cannot read|imx_gpio|clock speed|halt|target|Error|Info :" |
    ForEach-Object { $_.Line }
}
```

Helper: [probe_h7_swd_official.sh](../../firmware/x8_lora_bootloader_helper/probe_h7_swd_official.sh)

The helper runs:
```
echo fio | sudo -S -p '' /usr/bin/openocd -d3 \
    -f /usr/arduino/extra/openocd_script-imx_gpio.cfg \
    -c "init; halt; reg pc; shutdown" 2>&1
```

## Pass criteria

| # | Check | Healthy line | Wedged line |
|---|---|---|---|
| 1 | `saved pinmux settings: ... srst N` | `srst 1` | `srst 0` ← bug |
| 2 | DPIDR read | `Info : SWD DPIDR 0x6ba02477` | `Error: Error connecting DP: cannot read IDR` |
| 3 | Examine end | `target event 21 (examine-end) for core stm32h7x.cpu0` | (never reached) |
| 4 | Halt + PC read | `pc (/32): 0x...` | (never reached) |

## Diagnostic interpretation

| Pattern | Diagnosis |
|---|---|
| Both boards show `srst 1` + DPIDR read OK | SWD layer healthy. If Stage 1 still fails, bug is higher up (Tcl cfg, flasher, etc.). |
| Subject board `srst 0`, control `srst 1` | i.MX GPIO1_IO10 pad mux drifted off GPIO function. **Soft reset will not fix.** Escalate to T2 cold power cycle. |
| Both boards `srst 0` | openocd `imx_gpio` driver bug or shared upstream regression — investigate openocd build. |
| Both boards `cannot read IDR` but `srst 1` | H7 actively driving NRST or hard-stuck in reset. Check H7 firmware (`stm32h7-program.service`). |

## Verdict matrix

| Date | Board | `srst` | DPIDR | Result |
|---|---|---|---|---|
| 2026-05-12 | Board 2 (`2E2C`) | 1 | 0x6ba02477 | PASS |
| 2026-05-12 | Board 1 (`2D0A`) | 0 | `cannot read IDR` | FAIL — IOMUXC pad drift confirmed (root cause: post-W2-01 camera-induced ci_hdrc-imx wedge cycles + emergency unplugs). |
| 2026-05-13 | Board 1 (`2D0A`) post-T2, no preflight | 0 | `cannot read IDR` | FAIL — gpio10 still in alt-function on fresh boot (Linux boot did not mux it as GPIO). |
| 2026-05-13 | Board 1 (`2D0A`) post-T2 + `manual_gpio_preflight.sh` | 1 | `cannot read IDR` | NEW SIGNATURE: gpio10 (srst) now muxed correctly, but DP still unreachable — drift has migrated/persists on gpio8/gpio15 SWD pads. `prep_bridge.sh` did not change the result. |
| 2026-05-13 | Board 1 (`2D0A`) post-long-hold-T2 (≥60 s) + preflight | 1 | `cannot read IDR` (`JUNK DP read reg 0 = ffffffff`) | FAIL — long-hold T2 also insufficient. Confirms wedge survives full power-rail discharge. T3a SDP reflash required. |
