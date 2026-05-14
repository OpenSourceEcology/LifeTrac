# HC-01 — adb + Windows USB enumeration

**Purpose:** Confirm the host can see each board's USB composite device, the ADB
Interface (MI_02), and the CDC serial port (MI_00 → COMx); verify `adb devices`
lists each board as `device`.

**When to run:** Always first. If this fails, all higher-layer routines are
moot.

## Procedure (Windows / PowerShell)

```powershell
# 1. adb visibility
adb devices -l

# 2. Windows PnP enumeration of both boards + Max Carrier J-Link
Get-PnpDevice -PresentOnly | Where-Object {
    $_.InstanceId -match 'VID_2341' -or
    $_.InstanceId -match 'VID_1366&PID_0105' -or
    $_.InstanceId -match 'VID_1FC9'
} | Select-Object Status, Class, FriendlyName, InstanceId | Format-List
```

## Pass criteria

| # | Check | Expected |
|---|---|---|
| 1 | `adb devices -l` lists serial as `device` | `<serial>  device transport_id:<n>` |
| 2 | Windows PnP shows `USB Composite Device` for serial | `Status=OK`, `InstanceId` ends in `\<serial>` |
| 3 | Windows PnP shows `ADB Interface` (MI_02) | `Status=OK` |
| 4 | Windows PnP shows `USB Serial Device (COMx)` (MI_00) | `Status=OK` |
| 5 | Re-running `adb devices` 30 s later still shows `device` | stable, not transient |

## Common failure modes

| Symptom | Likely cause | Next routine |
|---|---|---|
| All 4 PnP entries OK but `adb devices` empty | adbd dead in Linux (soft-hung userland) | T2 cold power cycle |
| Briefly `device` then drops | adbd starts during boot then crashes | T2 → T3a SDP reflash |
| No PnP entries at all | Cable / hub / VBUS / module not powered | Check cable + carrier 12V |
| `Status=Error` on Composite Device | Windows driver issue or VBUS sag | Reseat USB-C; try different host port |
| ADB serial appears with `unauthorized` | Trust prompt on device — not applicable to LmP X8 | n/a |

## Pointers to the actual evidence
- Helper: this routine is host-side PowerShell; no `.sh` needed.
- Logs: `LifeTrac-v25/AI NOTES/YYYY-MM-DD_<board>_healthcheck_*.log`

## Verdict matrix

| Date | Board | Result | Notes |
|---|---|---|---|
| 2026-05-13 | Board 2 (`2E2C`) | PASS | All 4 checks; stable across 4× polls. |
| 2026-05-13 | Board 1 (`2D0A`) | FAIL_TRANSIENT | PnP all OK; adb visible at session start (transport_id=14) then dropped within seconds, did not return for 60+ s. COM11 `WriteLine` → semaphore timeout. Soft-hung Linux userland. |
