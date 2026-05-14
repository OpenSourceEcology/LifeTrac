# T2.5 — Carrier-power sanity check

**Tier:** 2.5 (pre-flight before T3 SDP work)

**When to try:** Before attempting T3a SDP reflash. The X8 module's USB-C only
exposes mask-ROM SDP if the **module** is powered, but separately, the **Max
Carrier**'s onboard J-Link BMP and DIP-switch logic only work if the carrier's
own 12V (and/or its USB-C) is up. If the BMP isn't enumerating on the host,
the carrier is under-powered, regardless of whether the X8 module is.

## Procedure

```powershell
Get-PnpDevice -PresentOnly | Where-Object {
    $_.InstanceId -match 'VID_1366&PID_0105'
} | Select-Object Status, FriendlyName, InstanceId | Format-List
```

## Pass criteria

At least one (typically two) `VID_1366&PID_0105` interface(s) enumerated with
`Status=OK` (Max Carrier J-Link BMP + JLink CDC UART).

## Common failure modes

| Symptom | Cause | Fix |
|---|---|---|
| Zero `1366:0105` entries | Max Carrier 12V missing OR carrier USB-C not connected to host OR carrier dead | Plug carrier 12V; plug carrier USB-C (separate from X8 USB-C); reseat module on carrier |
| Entries present but status `Error` | Driver issue (rare) | Reseat carrier USB-C |

## Notes

- A previous Board-1 session (2026-05-12) found **zero** `1366:0105` entries
  enumerated, indicating the carrier itself was not being driven. T3a will not
  succeed in that state — fix carrier power first.

## Verdict matrix

| Date | Board | Result | Notes |
|---|---|---|---|
| 2026-05-12 | Board 1 (`2D0A`) | FAIL_NO_CARRIER | No `1366:0105` enumerated; resolved by plugging carrier USB-C separately. |
