# T0 — Host adb daemon kick

**Tier:** 0 (host-side only; no board change)

**When to try:** `adb devices` shows `offline`, `no permissions`, or stale state
that doesn't match Windows PnP. **Not** for "device fully gone from `adb
devices` while PnP shows it OK" — that's a board-side problem, not a host one.

## Procedure

```powershell
adb kill-server
adb start-server
Start-Sleep -Seconds 2
adb devices -l
```

Optional: also re-issue `adb reconnect` (works only on already-attached
transports).

## Pass criteria

Board returns to `device` state in `adb devices`.

## Verdict matrix

| Date | Board | Worked? | Notes |
|---|---|---|---|
| 2026-05-13 | Board 1 (`2D0A`) | NO | Daemon restart had no effect — issue is in adbd on the X8, not the host. |
| (historic) | various | sometimes | Fixes occasional Windows-side stale transport entries; never fixes board-side hangs. |
