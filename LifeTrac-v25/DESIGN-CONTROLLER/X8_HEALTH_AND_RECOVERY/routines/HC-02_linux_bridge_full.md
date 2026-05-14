# HC-02 — Linux bridge + GPIO + UART health (full)

**Purpose:** Full snapshot of X8 Linux side: uptime, kernel, dmesg errors,
`x8h7_*` modules, M4/H7 services, GPIO chips, exported SWD lines (gpio8/10/15),
`/dev/ttymxc3`, watchdog, disk, RAM, USB topology.

**When to run:** After HC-01 PASS. Establishes baseline before any flash or
SWD work.

## Procedure

```powershell
$script = "LifeTrac-v25/DESIGN-CONTROLLER/firmware/x8_lora_bootloader_helper/board1_healthcheck.sh"
foreach ($s in '2D0A1209DABC240B','2E2C1209DABC240B') {
  $tag = if ($s -like '2D0A*') {'board1'} else {'board2'}
  $log = "LifeTrac-v25/AI NOTES/$(Get-Date -Format yyyy-MM-dd)_${tag}_healthcheck_v1_0.log"
  adb -s $s push $script /tmp/board1_healthcheck.sh 2>&1 | Out-Null
  adb -s $s exec-out "sh /tmp/board1_healthcheck.sh" 2>&1 |
    Out-File -FilePath $log -Encoding utf8
  Get-Content $log
}
```

Helper: [board1_healthcheck.sh](../../firmware/x8_lora_bootloader_helper/board1_healthcheck.sh)

## Pass criteria

| # | Check | Expected (good) | Tolerated |
|---|---|---|---|
| 1 | `uptime` reports a sane value | `up Nh` or `up N days` | reboot OK if intentional |
| 2 | `lsmod \| grep ^x8h7` lists 9 modules | `x8h7_{adc,can,drv,gpio,h7,pwm,rtc,uart,ui}` | — |
| 3 | `x8h7_drv` `Used by` count | `8` | — |
| 4 | `m4-proxy.service` | `active` | — |
| 5 | `stm32h7-program.service` | `active` | — |
| 6 | `gpio{8,10,15}` direction | `out` for all three | — |
| 7 | `gpio10` value (H7 NRST) | `1` | — |
| 8 | `/dev/ttymxc3` present | crw-rw---- root dialout 207,19 | — |
| 9 | `/dev/watchdog0` present | yes | — |
| 10 | dmesg err/warn last boot | only known benign noise (PCIe init, MIPI-DSI defer, cs42l52 mismatch, brcmfmac fw miss) | flag any new entry |
| 11 | `/sys/kernel/x8h7_firmware/version` read | `<read failed/timeout>` is **expected/benign** on this OE image; if it returns a string, that's also fine | — |

## Common failure modes

| Symptom | Likely cause | Action |
|---|---|---|
| Script never returns | adbd died mid-exec (soft hang) | HC-01 → T2 cold power cycle |
| `gpio8/10/15` not exported | Stage 1 launcher preflight didn't run | Run Stage 1 (preflight is at line 229) |
| `gpio10 val=0` | NRST stuck low | T2 cold cycle (often IOMUXC drift) |
| `m4-proxy` inactive | x8h7 firmware crash | `systemctl restart m4-proxy.service`; if fails → `prep_bridge.sh` then service restart |
| New dmesg errors | Kernel-level fault | Save full `dmesg -T` and triage |

## Verdict matrix

| Date | Board | Result | Notes |
|---|---|---|---|
| 2026-05-13 | Board 2 (`2E2C`) | PASS | uptime 3d 9:49; all 9 x8h7 modules, both services active, gpio8/10/15 = 0/1/0, ttymxc3 + watchdog0 present, 14% disk, 1.58 G free RAM. |
| 2026-05-13 | Board 1 (`2D0A`) | UNREACHABLE | adb dropped before script could run. |
