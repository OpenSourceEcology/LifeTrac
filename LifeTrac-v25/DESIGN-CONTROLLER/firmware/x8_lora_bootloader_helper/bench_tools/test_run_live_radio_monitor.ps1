#requires -Version 5.1
<#
.SYNOPSIS
    Offline tests for the helper functions of run_live_radio_monitor.ps1.
.DESCRIPTION
    Loads the helper functions out of the harness through the PowerShell AST
    (the harness body never runs: no adb, no board, no radio) and checks them
    against sample adb output, including the \r\r\n line ends of adb from
    platform-tools 37 as PowerShell really receives them from `cmd /c`.

    Bench kit (2026-10-19) additions: the checked adb push (Invoke-AdbPush and
    Confirm-AdbPushedFiles against stub adb .cmd files in a temp folder, never
    the real adb), the clock-offset parser, the leg-end order (read from the
    harness text), the 10-19 archived reset logs, and the SIGHUP race of
    bench_tools/ocd_reset.sh and inject_watch.sh: Git for Windows' dash/bash
    run them with stub sudo / nohup / openocd / pkill / docker and SIGHUP the
    spawn's process group the way the adb/sudo session hangup does. Those
    shell checks are SKIPPED when Git for Windows' usr\bin is not found.

        powershell -NoProfile -File bench_tools\test_run_live_radio_monitor.ps1

    Exit 0 = all passed, 1 = a failure (listed), 2 = the harness did not parse.
#>
Set-StrictMode -Version Latest
$ErrorActionPreference = "Stop"

$harness = Join-Path $PSScriptRoot "..\run_live_radio_monitor.ps1"
$errs = $null; $tokens = $null
$ast = [System.Management.Automation.Language.Parser]::ParseFile((Resolve-Path $harness).Path, [ref]$tokens, [ref]$errs)
if ($errs.Count -gt 0) {
    $errs | ForEach-Object { Write-Host "PARSE ERROR line $($_.Extent.StartLineNumber): $($_.Message)" }
    exit 2
}
$names = @("ConvertFrom-AdbOutput", "ConvertTo-InjectSpec", "Test-InjectFaultLine",
           "Get-InjectWatchPid", "Get-InjectSummary", "Get-TractorResetVerdict",
           "Test-AdbPushOutput", "Get-PushedFileList", "Get-CleanPushSource", "Test-PushedMd5", "Get-ClockOffset",
           "Get-PcEpochS", "Read-BoardClock", "Invoke-AdbPush", "Confirm-AdbPushedFiles")
foreach ($n in $names) {
    $fn = $ast.Find({ param($x) $x -is [System.Management.Automation.Language.FunctionDefinitionAst] -and $x.Name -eq $n }, $true)
    if (-not $fn) { Write-Host "MISSING function $n in the harness"; exit 2 }
    . ([scriptblock]::Create($fn.Extent.Text))
}

$script:fails = 0
$script:passes = 0
$script:skips = 0
function Check([string]$Name, [bool]$Cond, [string]$Got = "") {
    if ($Cond) { $script:passes++; Write-Host "PASS $Name" }
    else { $script:fails++; Write-Host "FAIL $Name  (got: $Got)" -ForegroundColor Red }
}
function Skip([string]$Name, [string]$Why) {
    $script:skips++; Write-Host "SKIP $Name  ($Why)" -ForegroundColor Yellow
}
function Throws([scriptblock]$Block) {
    try { & $Block | Out-Null; return $false } catch { return $true }
}

# Real \r\r\n output through cmd /c, the way the harness captures adb.
$tmp = Join-Path ([System.IO.Path]::GetTempPath()) ("lt_harness_test_{0}.txt" -f $PID)
function CmdLines([string]$Text) {
    [System.IO.File]::WriteAllText($tmp, $Text.Replace("`n", "`r`r`n"))
    return (cmd /c "type `"$tmp`"")
}

# --- ConvertFrom-AdbOutput ---------------------------------------------------
$raw = CmdLines "CFG-OK`n"
Check "cmd /c splits \r\r\n into a line and an empty element" ((@($raw).Count -eq 2) -and (@($raw)[1] -eq "")) "$(@($raw).Count) elements"
$clean = @(ConvertFrom-AdbOutput $raw)
Check "adb lines: empty elements dropped" ($clean.Count -eq 1 -and $clean[0] -eq "CFG-OK") ($clean -join '|')
Check "adb lines: whole-line -contains works" ($clean -contains "CFG-OK")
$one = @(ConvertFrom-AdbOutput "INJ-STAGED`r`r`nsecond line `r`r`n")
Check "single string with CRs: split and trimmed" ($one.Count -eq 2 -and $one[0] -eq "INJ-STAGED" -and $one[1] -eq "second line") ($one -join '|')
Check "a raw CR-terminated string is not equal before cleaning" ("INJ-STAGED`r" -ne "INJ-STAGED")
Check "null gives nothing" (@(ConvertFrom-AdbOutput $null).Count -eq 0)
$tail = @(ConvertFrom-AdbOutput (CmdLines "l1`nl2`nl3`nl4`nl5`nl6`nl7`nl8`nl9`nl10`n"))
Check "camera tail: -Last 8 gives 8 log lines" ((@($tail | Select-Object -Last 8)).Count -eq 8 -and $tail[-1] -eq "l10")
$rawTail = CmdLines "l1`nl2`nl3`nl4`nl5`nl6`nl7`nl8`nl9`nl10`n"
Check "camera tail: the old -Last 8 on raw output held only 4 lines" ((@($rawTail | Select-Object -Last 8 | Where-Object { $_ -ne "" })).Count -eq 4)

# --- Get-InjectWatchPid --------------------------------------------------------
Check "pid from \r\r\n output" ((Get-InjectWatchPid (CmdLines "INJECT_WATCH_SPAWNED pid=4321`n")) -eq 4321)
Check "pid from a CR-terminated string" ((Get-InjectWatchPid "INJECT_WATCH_SPAWNED pid=77`r`r`n") -eq 77)
Check "no pid line -> null" ($null -eq (Get-InjectWatchPid (CmdLines "sudo: a password is required`n")))

# --- ConvertTo-InjectSpec ------------------------------------------------------
$s = ConvertTo-InjectSpec -Cmd "dual_inject.py 250" -On ""
Check "default trigger is first_frame" ($s.On -eq "first_frame" -and $s.Script -eq "dual_inject.py" -and $s.Cmd -eq "dual_inject.py 250")
$s = ConvertTo-InjectSpec -Cmd "  kf_inject.py   15  20 " -On "first_frame"
Check "spaces collapse" ($s.Cmd -eq "kf_inject.py 15 20") $s.Cmd
$s = ConvertTo-InjectSpec -Cmd "dual_inject.py 120" -On "fault:0x0D:final"
Check "fault:0x0D:final" ($s.On -eq "fault" -and $s.Code -eq 13 -and $s.Final -and $s.Sub -eq -1 -and $s.Label -eq "fault:0x0D:final") $s.Label
$s = ConvertTo-InjectSpec -Cmd "dual_inject.py 120" -On "fault:d:83"
Check "fault:d:83 (no 0x)" ($s.Code -eq 13 -and $s.Sub -eq 0x83 -and -not $s.Final -and $s.Label -eq "fault:0x0D:0x83") $s.Label
$s = ConvertTo-InjectSpec -Cmd "dual_inject.py" -On "FAULT:0x0a"
Check "fault code only, any case" ($s.Code -eq 10 -and $s.Sub -eq -1 -and -not $s.Final)
Check "InjectOn without InjectCmd throws" (Throws { ConvertTo-InjectSpec -Cmd "" -On "first_frame" })
Check "non-.py command throws" (Throws { ConvertTo-InjectSpec -Cmd "rm -rf /" -On "" })
Check "shell metacharacter in an argument throws" (Throws { ConvertTo-InjectSpec -Cmd "dual_inject.py 250;reboot" -On "" })
Check "quote in an argument throws" (Throws { ConvertTo-InjectSpec -Cmd "dual_inject.py 'x'" -On "" })
Check "path in the script name throws" (Throws { ConvertTo-InjectSpec -Cmd "../x.py" -On "" })
Check "bad trigger throws" (Throws { ConvertTo-InjectSpec -Cmd "dual_inject.py" -On "first_fram" })
Check "bad fault sub throws" (Throws { ConvertTo-InjectSpec -Cmd "dual_inject.py" -On "fault:0x0D:last" })

# --- Test-InjectFaultLine (leg X tx_daemon.log lines, 2026-10-18) -------------
$x0 = "2026-09-29 09:21:33,635 WARNING image_tx_daemon: FAULT during pipelined TX: code=0x0D sub=0x00 raw=0d0000000000000000000000000000000000000084830000"
$x2 = "2026-09-29 09:22:33,518 WARNING image_tx_daemon: FAULT during pipelined TX: code=0x0D sub=0x02 raw=0d02000000000000000000000000000000000000e46d0100"
$x3 = "2026-09-29 09:23:03,425 WARNING image_tx_daemon: FAULT during pipelined TX: code=0x0D sub=0x83 raw=0d8300000000000000000000000000000000000016e30100"
$settle = "FAULT_URC during settle (expected for W1-9 lenient init): code=0x0A sub=0x00 reason=no_ingress_during_probe_window raw=0a00000000000000000000000000000000000000c50b0000"
$final = ConvertTo-InjectSpec -Cmd "dual_inject.py 120" -On "fault:0x0D:final"
Check "final: sub 0x00 does not fire" (-not (Test-InjectFaultLine -Line $x0 -Spec $final))
Check "final: sub 0x02 does not fire" (-not (Test-InjectFaultLine -Line $x2 -Spec $final))
Check "final: sub 0x83 fires" (Test-InjectFaultLine -Line $x3 -Spec $final)
$any = ConvertTo-InjectSpec -Cmd "dual_inject.py 120" -On "fault:0x0D"
Check "code only: first 0x0D fires" (Test-InjectFaultLine -Line $x0 -Spec $any)
Check "code mismatch does not fire" (-not (Test-InjectFaultLine -Line $settle -Spec $any))
$sub2 = ConvertTo-InjectSpec -Cmd "dual_inject.py 120" -On "fault:0x0D:0x02"
Check "exact sub fires only on it" ((Test-InjectFaultLine -Line $x2 -Spec $sub2) -and -not (Test-InjectFaultLine -Line $x3 -Spec $sub2))
$a = ConvertTo-InjectSpec -Cmd "dual_inject.py" -On "fault:0x0A"
Check "settle FAULT_URC counts as a FAULT line" (Test-InjectFaultLine -Line $settle -Spec $a)
Check "a non-FAULT line never fires" (-not (Test-InjectFaultLine -Line "INFO stats code=0x0D sub=0x83" -Spec $final))
$xl = @(ConvertFrom-AdbOutput (CmdLines "$x0`n$x2`n$x3`n"))
Check "grep output through cmd /c: the final line found" (@($xl | Where-Object { Test-InjectFaultLine -Line $_ -Spec $final }).Count -eq 1)

# --- Get-InjectSummary -----------------------------------------------------------
$ok = CmdLines ("INJECT_WATCH 16:21:58 1792340518 mode=first_frame deadline_s=360 cmd=dual_inject.py 250`n" +
    "INJECT_TRIGGER 16:21:59 1792340519 first_frame: 2026-10-18 16:21:57,488 INFO image_rx_daemon: published frame_id=0 seq=1 2985 B`n" +
    "INJECT_START 16:21:59 1792340519 dual_inject.py 250`nDUAL t=0 em=1 kf=1`nDUAL_DONE em=356 kf=45 dur=250`n" +
    "INJECT_EXIT 16:26:10 1792340770 rc=0`nINJECT_STOP_DONE`n")
$r = Get-InjectSummary $ok
Check "summary: finished" ($r.Status -eq "finished" -and $r.Start -eq "16:21:59" -and $r.Exit -eq "0") "$($r.Status) $($r.Start) $($r.Exit)"
Check "summary: trigger keeps the rx_smoke line" ($r.Trigger -like "16:21:59 first_frame: 2026-10-18 16:21:57,488*") $r.Trigger
$r = Get-InjectSummary ("INJECT_WATCH 1 2 mode=first_frame`nINJECT_TRIGGER 16:14:19 3 first_frame: x`nINJECT_START 16:14:19 3 dual_inject.py 250`n" +
    "INJECT_STOP 16:15:58 4 by=harness injector_running=1 watcher_running=1`nINJECT_EXIT 16:15:59 5 rc=137`n")
Check "summary: stopped at leg end" ($r.Status -eq "stopped_at_leg_end" -and $r.Exit -eq "137") $r.Status
$r = Get-InjectSummary "INJECT_WATCH 1 2 mode=first_frame`nINJECT_STOP 16:15:58 4 by=harness injector_running=0 watcher_running=1`n"
Check "summary: trigger never fired before the end" ($r.Status -eq "not_started" -and $r.Detail -like "*had not fired*") $r.Detail
$r = Get-InjectSummary "INJECT_WATCH 1 2 mode=first_frame`nINJECT_NOT_STARTED 16:20:00 9 rx_smoke stopped before its first published frame_id`n"
Check "summary: not started with the watcher's reason" ($r.Status -eq "not_started" -and $r.Detail -eq "rx_smoke stopped before its first published frame_id") $r.Detail
$r = Get-InjectSummary "INJECT_ARMED 16:30:12 9 fault:0x0D:final`nINJECT_STOP_DONE`n"
Check "summary: fault trigger never seen" ($r.Status -eq "not_started" -and $r.Detail -eq "its trigger never fired") $r.Detail
$r = Get-InjectSummary "INJECT_WATCH 1 2 x`nINJECT_TRIGGER 1 2 now`nINJECT_START 1 2 kf_inject.py`nTraceback`nINJECT_EXIT 1 2 rc=1`n"
Check "summary: injector failed" ($r.Status -eq "failed" -and $r.Exit -eq "1") $r.Status
Check "summary: nothing came back" ((Get-InjectSummary $null).Status -eq "unknown")

# --- Get-TractorResetVerdict -------------------------------------------------
# Phase lines as 08_boot_user_app.cfg echoes them (format of the 2026-09-15
# base flash log); markers as the end-of-leg read prints them.
$phases = "Open On-Chip Debugger 0.11.0-dirty (2022-10-19-16:13)`nInfo : SWD DPIDR 0x6ba02477`n" +
    "Phase D: pulse NRST (PF_4 LOW 250 ms then HIGH)`n" +
    "  GPIOF_IDR.PF_4 (low)  = 0   (expect 0)`n  GPIOF_IDR.PF_4 (high) = 1   (expect 1)`n" +
    "DONE: BOOT0 LOW, NRST released. L072 should now run user firmware.`nshutdown command invoked`n"
$v = Get-TractorResetVerdict -Output (CmdLines ($phases + "OCD_EXIT=0`n__OCD_LOG_RC=0`n__OCD_RUNNING=0`n"))
Check "reset: landed" ($v.Verdict -eq "landed" -and $v.Reason -like "*openocd exit 0*") "$($v.Verdict): $($v.Reason)"
$v = Get-TractorResetVerdict -Output (($phases + "OCD_EXIT=0`n__OCD_LOG_RC=0`n__OCD_RUNNING=0`n").Replace("`n", "`r`r`n"))
Check "reset: landed from one CR-laden string" ($v.Verdict -eq "landed") $v.Verdict
$v = Get-TractorResetVerdict -Output (CmdLines ("Open On-Chip Debugger 0.11.0`nError: Can't find /tmp/lifetrac_p0c/08_boot_user_app.cfg`nOCD_EXIT=1`n__OCD_LOG_RC=0`n__OCD_RUNNING=0`n")) -CfgOk $false
Check "reset: failed, exit 1 with the first error and the cfg note" ($v.Verdict -eq "failed" -and $v.Reason -like "*exit 1*Can't find*" -and $v.Reason -like "*cfg was missing*") $v.Reason
$v = Get-TractorResetVerdict -Output (CmdLines "Open On-Chip Debugger 0.11.0`nInfo : imx_gpio GPIO JTAG/SWD bitbang driver`n__OCD_LOG_RC=0`n__OCD_RUNNING=1`n")
Check "reset: failed, openocd hung" ($v.Verdict -eq "failed" -and $v.Reason -like "*hung*") $v.Reason
$v = Get-TractorResetVerdict -Output (CmdLines "cat: can't open '/tmp/lifetrac_p0c/ocd_reset.log': No such file or directory`n__OCD_LOG_RC=1`n__OCD_RUNNING=0`n")
Check "reset: failed, no log" ($v.Verdict -eq "failed" -and $v.Reason -like "*never started*") $v.Reason
$v = Get-TractorResetVerdict -Output (CmdLines "error: device '2E2C1209DABC240B' not found`n")
Check "reset: unknown when the read failed" ($v.Verdict -eq "unknown") $v.Verdict
$v = Get-TractorResetVerdict -Output (CmdLines ("  GPIOF_IDR.PF_4 (low)  = 1   (expect 0)`n  GPIOF_IDR.PF_4 (high) = 1   (expect 1)`nOCD_EXIT=0`n__OCD_LOG_RC=0`n__OCD_RUNNING=0`n"))
Check "reset: failed, PF4 never read low" ($v.Verdict -eq "failed" -and $v.Reason -like "*low=1*") $v.Reason
$v = Get-TractorResetVerdict -Output (CmdLines "Info : SWD DPIDR 0x6ba02477`nOCD_EXIT=0`n__OCD_LOG_RC=0`n__OCD_RUNNING=0`n")
Check "reset: unknown, clean exit without Phase D" ($v.Verdict -eq "unknown") $v.Verdict
$v = Get-TractorResetVerdict -Output (CmdLines "Error connecting DP: cannot read IDR`n__OCD_LOG_RC=0`n__OCD_RUNNING=0`n")
Check "reset: failed, attach error and no exit line" ($v.Verdict -eq "failed" -and $v.Reason -like "*cannot read IDR*") $v.Reason

# Per-leg tag (OCD_LEG=<stamp>, the log's first line) and the spawn output.
$tag = "OCD_LEG=20261019T140501Z"
$spawnOk = CmdLines "swd_reset_spawned`n"
$v = Get-TractorResetVerdict -Output (CmdLines ("$tag`n" + $phases + "OCD_EXIT=0`n__OCD_LOG_RC=0`n__OCD_RUNNING=0`n")) -Tag $tag -SpawnOutput $spawnOk
Check "reset tag: this leg's landed log is landed" ($v.Verdict -eq "landed" -and $v.Reason -notlike "*spawn*") "$($v.Verdict): $($v.Reason)"
$v = Get-TractorResetVerdict -Output (CmdLines ("OCD_LEG=20261019T134012Z`n" + $phases + "OCD_EXIT=0`n__OCD_LOG_RC=0`n__OCD_RUNNING=0`n")) -Tag $tag -SpawnOutput $spawnOk
Check "reset tag: an earlier leg's landed log is failed, not landed" ($v.Verdict -eq "failed" -and $v.Reason -like "*not this leg's*OCD_LEG=20261019T134012Z*not spawned*") "$($v.Verdict): $($v.Reason)"
$v = Get-TractorResetVerdict -Output (CmdLines ($phases + "OCD_EXIT=0`n__OCD_LOG_RC=0`n__OCD_RUNNING=0`n")) -Tag $tag
Check "reset tag: an untagged (older harness) landed log is failed" ($v.Verdict -eq "failed" -and $v.Reason -like "*untagged*") "$($v.Verdict): $($v.Reason)"
$v = Get-TractorResetVerdict -Output (CmdLines ("OCD_LEG=20261019T134012Z`n" + $phases + "OCD_EXIT=0`n__OCD_LOG_RC=0`n__OCD_RUNNING=0`n")) -Tag $tag -SpawnOutput (CmdLines "error: closed`n")
Check "reset tag: the failed spawn's output is in the reason" ($v.Verdict -eq "failed" -and $v.Reason -like "*spawn printed 'error: closed', not swd_reset_spawned*") $v.Reason
$v = Get-TractorResetVerdict -Output (CmdLines "__OCD_LOG_RC=0`n__OCD_RUNNING=0`n") -Tag $tag -SpawnOutput $spawnOk
Check "reset tag: an empty log is failed (cut off at spawn)" ($v.Verdict -eq "failed" -and $v.Reason -like "*empty*") $v.Reason
$v = Get-TractorResetVerdict -Output (CmdLines "__OCD_LOG_RC=1`n__OCD_RUNNING=0`n") -Tag $tag -SpawnOutput @()
Check "reset tag: no log and a silent spawn" ($v.Verdict -eq "failed" -and $v.Reason -like "*never started*spawn printed nothing*") $v.Reason
$v = Get-TractorResetVerdict -Output (CmdLines ("$tag`n" + $phases + "OCD_EXIT=0`n__OCD_LOG_RC=0`n__OCD_RUNNING=0`n"))
Check "reset tag: no -Tag (old call) still judges the log" ($v.Verdict -eq "landed") $v.Verdict

# Signal exits (128+N): SIGHUP before nohup ignored it, SIGKILL mid-run.
$v = Get-TractorResetVerdict -Output (CmdLines "$tag`nOCD_EXIT=129`n__OCD_LOG_RC=0`n__OCD_RUNNING=0`n") -Tag $tag -SpawnOutput $spawnOk
Check "reset signal: OCD_EXIT=129 alone is SIGHUP, openocd never started" ($v.Verdict -eq "failed" -and $v.Reason -like "*SIGHUP*never started*exit 129*" -and $v.Reason -notlike "*openocd exit 129 before*") $v.Reason
$v = Get-TractorResetVerdict -Output (CmdLines "$tag`nOpen On-Chip Debugger 0.11.0`nInfo : SWD DPIDR 0x6ba02477`nOCD_EXIT=129`n__OCD_LOG_RC=0`n__OCD_RUNNING=0`n") -Tag $tag -SpawnOutput $spawnOk
Check "reset signal: OCD_EXIT=129 with the banner is SIGHUP after openocd started (not nohup)" ($v.Verdict -eq "failed" -and $v.Reason -like "*SIGHUP after openocd started*exit 129*" -and $v.Reason -notlike "*reaching nohup*" -and $v.Reason -notlike "*never started*") $v.Reason
$v = Get-TractorResetVerdict -Output (CmdLines "$tag`nOpen On-Chip Debugger 0.11.0`nInfo : SWD DPIDR 0x6ba02477`nOCD_EXIT=137`n__OCD_LOG_RC=0`n__OCD_RUNNING=0`n") -Tag $tag
Check "reset signal: SIGKILL after openocd started" ($v.Verdict -eq "failed" -and $v.Reason -like "*SIGKILL*exit 137*" -and $v.Reason -notlike "*never started*") $v.Reason
$v = Get-TractorResetVerdict -Output (CmdLines "$tag`nOpen On-Chip Debugger 0.11.0`nOCD_EXIT=137`n__OCD_LOG_RC=0`n__OCD_RUNNING=1`n") -Tag $tag
Check "reset signal: an exit line beats a running openocd (not 'hung')" ($v.Verdict -eq "failed" -and $v.Reason -like "*SIGKILL*" -and $v.Reason -notlike "*hung*") $v.Reason
$v = Get-TractorResetVerdict -Output (CmdLines ("$tag`n" + $phases + "OCD_EXIT=129`n__OCD_LOG_RC=0`n__OCD_RUNNING=0`n")) -Tag $tag
Check "reset signal: a pulse read back before the signal is still landed" ($v.Verdict -eq "landed" -and $v.Reason -like "*exit 129 (signal 1)*") $v.Reason
$v = Get-TractorResetVerdict -Output (CmdLines "$tag`nOpen On-Chip Debugger 0.11.0`nError: unable to open imx_gpio`nOCD_EXIT=1`n__OCD_LOG_RC=0`n__OCD_RUNNING=0`n") -Tag $tag
Check "reset signal: a plain exit 1 keeps the error-code wording" ($v.Verdict -eq "failed" -and $v.Reason -like "openocd exit 1 before the NRST pulse: Error: unable*") $v.Reason

# --- 10-19 archived reset logs (bench-evidence, read only) ---------------------
# A2a landed; A2c2 is the first archived SIGHUP miss (tag + OCD_EXIT=129 only).
$legs1019 = Join-Path $PSScriptRoot "..\..\..\bench-evidence\RS_13_range_edge_2026-10-19\legs"
$a2a = Join-Path $legs1019 "legA2a_tractor_ocd_reset.log"
$a2c2 = Join-Path $legs1019 "legA2c2_tractor_ocd_reset.log"
if ((Test-Path -LiteralPath $a2a) -and (Test-Path -LiteralPath $a2c2)) {
    $v = Get-TractorResetVerdict -Output (Get-Content -LiteralPath $a2a -Encoding UTF8) -Tag "OCD_LEG=20261019T185135Z" -SpawnOutput @("swd_reset_spawned")
    Check "10-19 A2a archived log: landed" ($v.Verdict -eq "landed" -and $v.Reason -like "*openocd exit 0*") "$($v.Verdict): $($v.Reason)"
    $v = Get-TractorResetVerdict -Output (Get-Content -LiteralPath $a2c2 -Encoding UTF8) -Tag "OCD_LEG=20261019T190055Z" -SpawnOutput @("swd_reset_spawned")
    Check "10-19 A2c2 archived log: failed, SIGHUP, openocd never started" ($v.Verdict -eq "failed" -and $v.Reason -like "*SIGHUP*never started*exit 129*") "$($v.Verdict): $($v.Reason)"
} else {
    Skip "10-19 archived reset logs" "not found under $legs1019"
}

# --- Test-AdbPushOutput (10-19 transcript lines; failures as adb prints them) --
$pf = "C:\Users\dorkm\Documents\GitHub\LifeTrac-bench-main\LifeTrac-v25\DESIGN-CONTROLLER\firmware\x8_lora_bootloader_helper\method_g_stage1_probe.py"
$r = Test-AdbPushOutput -Output (CmdLines "${pf}: 1 file pushed, 0 skipped. 85.5 MB/s (31529 bytes in 0.000s)`n") -ExitCode 0
Check "push: one file pushed, exit 0" ($r.Ok -and $r.Files -eq 1) $r.Reason
$r = Test-AdbPushOutput -Output "C:\x\image_pipeline\: 19 files pushed, 0 skipped. 2.9 MB/s (171070 bytes in 0.056s)`r`r`n" -ExitCode 0
Check "push: directory, 19 files" ($r.Ok -and $r.Files -eq 19) $r.Reason
$r = Test-AdbPushOutput -Output "C:\x\a.py: 1 file pushed. 2.3 MB/s (52117 bytes in 0.021s)" -ExitCode 0
Check "push: older adb wording (no 'skipped')" ($r.Ok -and $r.Files -eq 1) $r.Reason
$r = Test-AdbPushOutput -Output (CmdLines "adb: error: failed to copy 'C:\x\a.py' to '/tmp/lifetrac_p0c/a.py': remote couldn't create file: Permission denied`n") -ExitCode 1
Check "push: permission denied fails with adb's line" (-not $r.Ok -and $r.Reason -like "adb exit 1: *Permission denied*") $r.Reason
$r = Test-AdbPushOutput -Output "adb.exe: device '2E2C1209DABC240B' not found" -ExitCode 1
Check "push: device not found fails" (-not $r.Ok -and $r.Reason -like "*not found*") $r.Reason
$r = Test-AdbPushOutput -Output $null -ExitCode 0
Check "push: exit 0 without a pushed line fails" (-not $r.Ok -and $r.Reason -like "*no 'file(s) pushed' line*no output*") $r.Reason
$r = Test-AdbPushOutput -Output "C:\x\empty\: 0 files pushed, 0 skipped." -ExitCode 0
Check "push: 0 files pushed fails" (-not $r.Ok -and $r.Reason -like "adb pushed 0 files*") $r.Reason
$r = Test-AdbPushOutput -Output "C:\x\a.py: 1 file pushed, 0 skipped. 1 MB/s (10 bytes in 0.000s)" -ExitCode 1
Check "push: a pushed line with a non-zero exit still fails" (-not $r.Ok) $r.Reason

# --- Get-PushedFileList --------------------------------------------------------
$work = Join-Path ([System.IO.Path]::GetTempPath()) ("lt_harness_work_{0}" -f $PID)
if (Test-Path -LiteralPath $work) { Remove-Item -LiteralPath $work -Recurse -Force }
$pkg = Join-Path $work "pkg"
New-Item -ItemType Directory -Force -Path (Join-Path $pkg "sub"), (Join-Path $pkg "__pycache__") | Out-Null
[System.IO.File]::WriteAllText((Join-Path $pkg "a.py"), "print('a')`n")
[System.IO.File]::WriteAllText((Join-Path $pkg "sub\b.py"), "print('b')`n")
[System.IO.File]::WriteAllText((Join-Path $pkg "__pycache__\a.cpython-311.pyc"), "x")
[System.IO.File]::WriteAllText((Join-Path $pkg "sub\c.pyc"), "x")
$lst = @(Get-PushedFileList -Local $pkg -Remote "/tmp/lifetrac_strict/")
$rem = @($lst | ForEach-Object { $_.Remote })
Check "file list: a directory lands in Remote/<name>/, __pycache__ and .pyc left out" (($rem -join ',') -eq "/tmp/lifetrac_strict/pkg/a.py,/tmp/lifetrac_strict/pkg/sub/b.py") ($rem -join ',')
Check "file list: local paths kept" ($lst[0].Local -eq (Join-Path $pkg "a.py")) $lst[0].Local
$lst = @(Get-PushedFileList -Local (Join-Path $pkg "a.py") -Remote "/tmp/lifetrac_strict/")
Check "file list: a file pushed to a directory keeps its name" ($lst.Count -eq 1 -and $lst[0].Remote -eq "/tmp/lifetrac_strict/a.py") $lst[0].Remote
$lst = @(Get-PushedFileList -Local (Join-Path $pkg "a.py") -Remote "/tmp/lifetrac_p0c/ocd_reset.sh")
Check "file list: a file pushed to a full path takes that path" ($lst[0].Remote -eq "/tmp/lifetrac_p0c/ocd_reset.sh") $lst[0].Remote

# --- Get-CleanPushSource -------------------------------------------------------
# A directory is pushed from a temp copy without __pycache__ / *.pyc (the board's
# __pycache__ dirs are root-owned; a checked push into one would abort the leg).
$stage = Join-Path $work "stage"
$up = Get-CleanPushSource -Local $pkg -StageRoot $stage
$upFiles = @(Get-ChildItem -LiteralPath $up -Recurse -File -Force | ForEach-Object { $_.FullName.Substring($up.Length + 1) } | Sort-Object)
Check "clean source: a directory is staged as <stage>\<name> without __pycache__ / .pyc" ($up -eq (Join-Path $stage "pkg") -and ($upFiles -join ',') -eq "a.py,sub\b.py") "$up | $($upFiles -join ',')"
Check "clean source: the staged copy is byte-identical" ((Get-FileHash -Algorithm MD5 -LiteralPath (Join-Path $up "sub\b.py")).Hash -eq (Get-FileHash -Algorithm MD5 -LiteralPath (Join-Path $pkg "sub\b.py")).Hash)
[System.IO.File]::WriteAllText((Join-Path $up "stale.py"), "old")
$up = Get-CleanPushSource -Local $pkg -StageRoot $stage
Check "clean source: a re-stage drops files left from the last one" (-not (Test-Path -LiteralPath (Join-Path $up "stale.py")))
Check "clean source: a file is pushed as it is" ((Get-CleanPushSource -Local (Join-Path $pkg "a.py") -StageRoot $stage) -eq (Join-Path $pkg "a.py"))
Check "clean source: a missing path is returned unchanged (Invoke-AdbPush then refuses it)" ((Get-CleanPushSource -Local (Join-Path $work "nope") -StageRoot $stage) -eq (Join-Path $work "nope"))

# --- Test-PushedMd5 ------------------------------------------------------------
$m5 = @(
    @{ Remote = "/tmp/lifetrac_strict/a.py"; Md5 = "0cc175b9c0f1b6a831c399e269772661" },
    @{ Remote = "/tmp/lifetrac_strict/pkg/b.py"; Md5 = "92eb5ffee6ae2fec3ad71c777531578f" },
    @{ Remote = "/tmp/lifetrac_strict/c.py"; Md5 = "4a8a08f09d37b73795649038408b5f33" }
)
$mo = CmdLines ("0cc175b9c0f1b6a831c399e269772661  /tmp/lifetrac_strict/a.py`n" +
    "92EB5FFEE6AE2FEC3AD71C777531578E  /tmp/lifetrac_strict/pkg/b.py`n" +
    "md5sum: can't open '/tmp/lifetrac_strict/c.py': No such file or directory`n")
$r = Test-PushedMd5 -Output $mo -Files $m5
Check "md5: one ok, one mismatch, one missing" ($r.Checked -eq 2 -and @($r.Mismatch).Count -eq 1 -and @($r.Missing).Count -eq 1 -and $r.Mismatch[0] -eq "/tmp/lifetrac_strict/pkg/b.py" -and $r.Missing[0] -eq "/tmp/lifetrac_strict/c.py") "checked=$($r.Checked) mis=$($r.Mismatch -join ',') miss=$($r.Missing -join ',')"
Check "md5: rows say ok / MISMATCH / missing" ($r.Rows[0] -like "ok /tmp/lifetrac_strict/a.py *" -and $r.Rows[1] -like "MISMATCH /tmp/lifetrac_strict/pkg/b.py pc=92eb*board=92eb*578e" -and $r.Rows[2] -like "missing /tmp/lifetrac_strict/c.py *board=-") ($r.Rows -join ' | ')
$r = Test-PushedMd5 -Output "0CC175B9C0F1B6A831C399E269772661 */tmp/lifetrac_strict/a.py" -Files @($m5[0])
Check "md5: upper case and binary-mode * accepted" ($r.Checked -eq 1 -and @($r.Mismatch).Count -eq 0) ($r.Rows -join ' | ')
$r = Test-PushedMd5 -Output $null -Files $m5
Check "md5: nothing back = all missing, none checked" ($r.Checked -eq 0 -and @($r.Missing).Count -eq 3) "$($r.Checked)"

# --- Get-ClockOffset -----------------------------------------------------------
$c = Get-ClockOffset -Output (CmdLines "1792340518.500000000`n") -PcBeforeS 1792340518.0 -PcAfterS 1792340518.2
Check "clock: offset at the round trip's midpoint" ($c.Ok -and [math]::Abs($c.OffsetS - 0.4) -lt 1e-6 -and [math]::Abs($c.ErrS - 0.1) -lt 1e-6) "$($c.OffsetS) $($c.ErrS)"
Check "clock: text" ($c.Text -eq "+0.400 s (+0.00 d) +-0.100 s") $c.Text
# The tractor ran ~19 days behind on 10-19 (its logs read 2026-09-30).
$c = Get-ClockOffset -Output "1790700000.250" -PcBeforeS 1792340518.0 -PcAfterS 1792340518.1
Check "clock: tractor ~19 days behind" ($c.Ok -and [math]::Abs($c.OffsetS - (-1640517.8)) -lt 1e-3 -and $c.Text -eq "-1640517.800 s (-18.99 d) +-0.050 s") $c.Text
$c = Get-ClockOffset -Output "1792340518.N" -PcBeforeS 1792340518.0 -PcAfterS 1792340518.2
Check "clock: busybox date without %N = whole seconds, +1 s error" ($c.Ok -and [math]::Abs($c.ErrS - 1.1) -lt 1e-6 -and $c.Text -like "*whole seconds*") $c.Text
$c = Get-ClockOffset -Output "1792340518.%N" -PcBeforeS 1792340518.0 -PcAfterS 1792340518.2
Check "clock: literal %N = whole seconds" ($c.Ok -and $c.Text -like "*whole seconds*") $c.Text
$c = Get-ClockOffset -Output (CmdLines "error: device '2D0A1209DABC240B' not found`n") -PcBeforeS 1.0 -PcAfterS 1.5
Check "clock: unknown when no epoch came back" (-not $c.Ok -and $null -eq $c.OffsetS -and $c.Text -like "unknown (date +%s.%N printed 'error: device*") $c.Text
Check "clock: unknown on no output" ((Get-ClockOffset -Output $null -PcBeforeS 1.0 -PcAfterS 1.5).Text -eq "unknown (date +%s.%N printed nothing)")
$savedCulture = [System.Threading.Thread]::CurrentThread.CurrentCulture
try {
    [System.Threading.Thread]::CurrentThread.CurrentCulture = [System.Globalization.CultureInfo]::GetCultureInfo("de-DE")
    $c = Get-ClockOffset -Output "1792340518.500000000" -PcBeforeS 1792340518.0 -PcAfterS 1792340518.2
    Check "clock: text uses '.' under a ',' culture (params.txt stays parseable)" ($c.Text -eq "+0.400 s (+0.00 d) +-0.100 s") $c.Text
} finally {
    [System.Threading.Thread]::CurrentThread.CurrentCulture = $savedCulture
}
$e0 = Get-PcEpochS
Check "clock: the PC epoch is now-ish" ($e0 -gt 1.7e9 -and $e0 -lt 4.1e9) "$e0"

# --- Invoke-AdbPush / Confirm-AdbPushedFiles / Read-BoardClock (stub adb) ------
# Stub adb .cmd files only: the real adb is never called.
$stubs = Join-Path $work "adbstub"
New-Item -ItemType Directory -Force -Path $stubs | Out-Null
function StubCmd([string]$Name, [string]$Body) {
    $p = Join-Path $stubs $Name
    [System.IO.File]::WriteAllText($p, ("@echo off`r`n" + $Body.Replace("`n", "`r`n")))
    return $p
}
$adbOk = StubCmd "adb_ok.cmd" "echo %~4: 1 file pushed, 0 skipped. 4.2 MB/s (2006 bytes in 0.000s) 1>&2`nexit /b 0`n"
$adbDenied = StubCmd "adb_denied.cmd" "echo adb: error: failed to copy '%~4' to '%5': remote couldn't create file: Permission denied 1>&2`nexit /b 1`n"
$adbSilent = StubCmd "adb_silent.cmd" "exit /b 0`n"
$f1 = Join-Path $pkg "a.py"
$r = Invoke-AdbPush -Adb $adbOk -Serial "TESTSERIAL" -Local $f1 -Remote "/tmp/lifetrac_strict/"
Check "Invoke-AdbPush: ok from adb's stderr summary" ($r.Ok -and $r.Files -eq 1) $r.Reason
# The leg wrapper runs the harness as `& { ... } *>&1 | Tee-Object`: a native
# stderr line then becomes a RemoteException record (25 per 10-19 transcript).
$cap = @(& { $ErrorActionPreference = "Continue"; Invoke-AdbPush -Adb $adbOk -Serial "TESTSERIAL" -Local $f1 -Remote "/tmp/lifetrac_strict/" | Out-Null } *>&1)
Check "Invoke-AdbPush: no error record under *>&1 (stderr merged inside cmd)" (@($cap | Where-Object { $_ -is [System.Management.Automation.ErrorRecord] }).Count -eq 0) "$($cap.Count) records"
$cap = @(& { $ErrorActionPreference = "Continue"; cmd /c "`"$adbOk`" -s TESTSERIAL push `"$f1`" /tmp/lifetrac_strict/" | Out-Null } *>&1)
Check "control: the old push form does leak an error record under *>&1" (@($cap | Where-Object { $_ -is [System.Management.Automation.ErrorRecord] }).Count -eq 1) "$($cap.Count) records"
$msg = ""
$script:pushMsg = ""
$info = @(& { try { Invoke-AdbPush -Adb $adbDenied -Serial "TESTSERIAL" -Local $f1 -Remote "/tmp/lifetrac_p0c/a.py" | Out-Null } catch { $script:pushMsg = "$_" } } 6>&1 | ForEach-Object { "$_" })
Check "Invoke-AdbPush: a failed push throws with adb's reason" ($script:pushMsg -like "*adb push FAILED*TESTSERIAL:/tmp/lifetrac_p0c/a.py*adb exit 1*Permission denied*No daemon was started*") $script:pushMsg
Check "Invoke-AdbPush: the failure is written to the host too (the leg transcript keeps it)" (@($info | Where-Object { $_ -like "*adb push FAILED*" }).Count -eq 1) ($info -join ' | ')
$msg = ""
try { Invoke-AdbPush -Adb $adbSilent -Serial "TESTSERIAL" -Local $f1 -Remote "/tmp/x/" 6>$null | Out-Null } catch { $msg = "$_" }
Check "Invoke-AdbPush: exit 0 with no summary throws" ($msg -like "*no 'file(s) pushed' line*") $msg
$msg = ""
try { Invoke-AdbPush -Adb $adbOk -Serial "TESTSERIAL" -Local (Join-Path $work "nope.py") -Remote "/tmp/x/" 6>$null | Out-Null } catch { $msg = "$_" }
Check "Invoke-AdbPush: a missing PC file throws before adb" ($msg -like "*nope.py does not exist on this PC*") $msg

# md5: the stub prints the board side from md5_out.txt and logs each call.
$adbMd5 = StubCmd "adb_md5.cmd" "echo call>>`"%~dp0md5_calls.txt`"`nif `"%3`"==`"shell`" type `"%~dp0md5_out.txt`"`nexit /b 0`n"
$fA = Join-Path $pkg "a.py"; $fB = Join-Path $pkg "sub\b.py"
$hA = (Get-FileHash -Algorithm MD5 -LiteralPath $fA).Hash.ToLower()
$hB = (Get-FileHash -Algorithm MD5 -LiteralPath $fB).Hash.ToLower()
$pairs = @(@{ Local = $fA; Remote = "/tmp/lifetrac_strict/pkg/a.py" }, @{ Local = $fB; Remote = "/tmp/lifetrac_strict/pkg/sub/b.py" })
function SetBoard([string]$Text) {
    [System.IO.File]::WriteAllText((Join-Path $stubs "md5_out.txt"), $Text.Replace("`n", "`r`n"))
    Remove-Item -LiteralPath (Join-Path $stubs "md5_calls.txt") -ErrorAction SilentlyContinue
}
SetBoard "$hA  /tmp/lifetrac_strict/pkg/a.py`n$hB  /tmp/lifetrac_strict/pkg/sub/b.py`n"
$r = Confirm-AdbPushedFiles -Adb $adbMd5 -Serial "TESTSERIAL" -Files $pairs
Check "Confirm: all match" ($r.Checked -eq 2 -and $r.Total -eq 2 -and $r.Summary -eq "md5 2/2 match") $r.Summary
SetBoard "$hA  /tmp/lifetrac_strict/pkg/a.py`n00000000000000000000000000000000  /tmp/lifetrac_strict/pkg/sub/b.py`n"
$msg = ""
try { Confirm-AdbPushedFiles -Adb $adbMd5 -Serial "TESTSERIAL" -Files $pairs 6>$null | Out-Null } catch { $msg = "$_" }
Check "Confirm: a mismatch throws, naming the file" ($msg -like "*md5 MISMATCH on TESTSERIAL*/tmp/lifetrac_strict/pkg/sub/b.py*No daemon was started*") $msg
SetBoard "$hA  /tmp/lifetrac_strict/pkg/a.py`n"
$mixed = @(& { Confirm-AdbPushedFiles -Adb $adbMd5 -Serial "TESTSERIAL" -Files $pairs } 3>&1)
$warns = @($mixed | Where-Object { $_ -is [System.Management.Automation.WarningRecord] })
$r = @($mixed | Where-Object { $_ -is [hashtable] })[0]
Check "Confirm: a file md5sum did not report is a warning, not a throw" ($r.Checked -eq 1 -and @($r.Missing).Count -eq 1 -and $r.Summary -eq "md5 1/2 match (1 not reported by md5sum)" -and $warns.Count -eq 1 -and "$($warns[0])" -like "*did not report 1 pushed file(s): /tmp/lifetrac_strict/pkg/sub/b.py*") "$($r.Summary) | $($warns -join ' | ')"
$many = @(for ($k = 0; $k -lt 60; $k++) { @{ Local = $fA; Remote = ("/tmp/lifetrac_strict/a_rather_long_package_directory_name/module_{0:D2}.py" -f $k) } })
SetBoard (($many | ForEach-Object { "$hA  $($_.Remote)" }) -join "`n")
$r = Confirm-AdbPushedFiles -Adb $adbMd5 -Serial "TESTSERIAL" -Files $many
$calls = @(Get-Content -LiteralPath (Join-Path $stubs "md5_calls.txt")).Count
Check "Confirm: long lists go in batches (3 md5sum calls for 60 long paths)" ($r.Checked -eq 60 -and $calls -eq 3) "checked=$($r.Checked) calls=$calls"
$odd = @(@{ Local = $fA; Remote = "/tmp/lifetrac_strict/a b.py" })
SetBoard "$hA  /tmp/lifetrac_strict/a b.py`n"
$r = @(& { Confirm-AdbPushedFiles -Adb $adbMd5 -Serial "TESTSERIAL" -Files $odd } 3>$null)[0]
$calls = 0; if (Test-Path -LiteralPath (Join-Path $stubs "md5_calls.txt")) { $calls = @(Get-Content -LiteralPath (Join-Path $stubs "md5_calls.txt")).Count }
Check "Confirm: a path with a space never reaches the board shell (reported missing)" ($calls -eq 0 -and @($r.Missing).Count -eq 1) "calls=$calls"

$adbDate = StubCmd "adb_date.cmd" "echo %*>`"%~dp0date_args.txt`"`necho 1792340518.123456789`nexit /b 0`n"
$c = Read-BoardClock -Adb $adbDate -Serial "TESTSERIAL"
$dateArgs = (Get-Content -LiteralPath (Join-Path $stubs "date_args.txt") -Raw).Trim()
Check "Read-BoardClock: %s.%N reaches adb unexpanded through cmd" ($dateArgs -like '*shell "date +%s.%N*"*') $dateArgs
Check "Read-BoardClock: offset and round trip from the stub" ($c.Ok -and $c.RttS -gt 0 -and $c.RttS -lt 30 -and [math]::Abs(($c.OffsetS + (Get-PcEpochS)) - 1792340518.123) -lt 60) "$($c.OffsetS) rtt=$($c.RttS)"

# --- Leg-end order and push wiring, read from the harness text ----------------
$hLines = [System.IO.File]::ReadAllLines((Resolve-Path $harness).Path)
function LineOf([string]$Needle, [int]$From = 0) {
    for ($i = $From; $i -lt $hLines.Count; $i++) { if ($hLines[$i].Contains($Needle)) { return $i } }
    return -1
}
$s8 = LineOf "# 8. End of leg"
$camStop = LineOf "docker stop -t 1 camera_svc" $s8
$synthStop = LineOf "docker stop -t 1 synth_pub" $s8
$injStop = LineOf '$injWatchCmd stop' $s8
$txStop = LineOf "docker stop -t 1 tx_smoke" $s8
$rxStop = LineOf "docker stop -t 1 rx_smoke" $s8
$clk = LineOf "Read-BoardClock -Adb" $s8
$txLog = LineOf "docker logs tx_smoke 2>&1" $s8
$rxLog = LineOf "docker logs rx_smoke 2>&1" $s8
$camLog = LineOf "docker logs camera_svc 2>&1" $s8
$synthLog = LineOf "docker logs synth_pub 2>&1" $s8
$synthRm = LineOf "docker rm -f synth_pub" $s8
$ocdRead = LineOf "cat /tmp/lifetrac_p0c/ocd_reset.log" $s8
$all = @($s8, $camStop, $synthStop, $injStop, $txStop, $rxStop, $clk, $txLog, $rxLog, $camLog, $synthLog, $synthRm, $ocdRead)
Check "order: every leg-end step found after '# 8. End of leg'" (@($all | Where-Object { $_ -lt 0 }).Count -eq 0) ($all -join ',')
Check "order: feed stop, then injector stop, then tx_smoke, then rx_smoke" ($camStop -lt $injStop -and $synthStop -lt $injStop -and $injStop -lt $txStop -and $txStop -lt $rxStop) ($all -join ',')
Check "order: daemon, camera and synth logs read only after rx_smoke stopped" ($rxStop -lt $clk -and $clk -lt $txLog -and $txLog -lt $rxLog -and $rxStop -lt $camLog -and $rxStop -lt $synthLog -and $synthLog -lt $synthRm) ($all -join ',')
Check "order: the reset log is read after the daemons stop" ($rxStop -lt $ocdRead) ($all -join ',')
Check "params: log_window=after_stop recorded" ((LineOf '"log_window=after_stop",') -gt $ocdRead)
Check "params: clock offsets recorded" ((LineOf '"tractor_clock_offset_s=$tOff",') -gt 0 -and (LineOf '"base_clock_offset_s=$bOff",') -gt 0)
$code = @($hLines | Where-Object { $_ -notmatch '^\s*#' })
Check "push: no adb push piped to Out-Null any more" (@($code | Where-Object { $_ -match '\bpush\b.*\|\s*Out-Null' -and $_ -notmatch 'Invoke-AdbPush' }).Count -eq 0)
Check "push: the only raw adb push is inside Invoke-AdbPush, with 2>&1 in the cmd string" ((@($code | Where-Object { $_ -match '-s \$\w+ push ' }).Count -eq 1) -and (@($code | Where-Object { $_ -match '-s \$Serial push .* 2>&1"$' }).Count -eq 1))
Check "reset: spawned from the pushed ocd_reset.sh" (@($code | Where-Object { $_ -like '*$swdReset = *sh /tmp/lifetrac_p0c/ocd_reset.sh $ocdTag*' }).Count -eq 1)
Check "reset: no inline caught HUP trap left in the code" (@($code | Where-Object { $_ -match 'trap : HUP' }).Count -eq 0)
Check "hint: points at bench_tools/bracket_deltas.py" (@($code | Where-Object { $_ -like '*score its brackets with bench_tools/bracket_deltas.py*' }).Count -eq 1)

# --- SIGHUP race: ocd_reset.sh and inject_watch.sh under dash (Git for Windows) --
$ocdSh = Join-Path $PSScriptRoot "ocd_reset.sh"
$injSh = Join-Path $PSScriptRoot "inject_watch.sh"
foreach ($sp in @($ocdSh, $injSh)) {
    $b = [System.IO.File]::ReadAllBytes($sp)
    Check "$(Split-Path -Leaf $sp): LF only and ASCII (pushed as-is to the board)" (@($b | Where-Object { $_ -eq 13 -or $_ -gt 127 }).Count -eq 0)
}
function Find-GitUsrBin {
    foreach ($n in @("dash", "sh")) {
        $c = Get-Command $n -CommandType Application -ErrorAction SilentlyContinue | Select-Object -First 1
        if ($c -and (Test-Path (Join-Path (Split-Path $c.Source) "bash.exe"))) { return (Split-Path $c.Source) }
    }
    $g = Get-Command git -CommandType Application -ErrorAction SilentlyContinue | Select-Object -First 1
    if ($g) {
        $d = Split-Path $g.Source
        for ($k = 0; $k -lt 3 -and $d; $k++) {
            $cand = Join-Path $d "usr\bin"
            if (Test-Path (Join-Path $cand "bash.exe")) { return $cand }
            $d = Split-Path $d
        }
    }
    return $null
}
function ToMsys([string]$P) {
    $f = [System.IO.Path]::GetFullPath($P)
    return "/" + $f.Substring(0, 1).ToLower() + $f.Substring(2).Replace('\', '/')
}
function WriteLf([string]$Path, [string]$Text) {
    [System.IO.File]::WriteAllText($Path, $Text.Replace("`r", ""))
}
function Section([object]$Lines, [string]$Start, [string]$End) {
    $on = $false
    foreach ($l in @($Lines)) {
        if ($l -eq $End) { $on = $false }
        if ($on) { $l }
        if ($l -eq $Start) { $on = $true }
    }
}
$usrBin = Find-GitUsrBin
if (-not $usrBin) {
    Skip "SIGHUP race (ocd_reset.sh, inject_watch.sh)" "Git for Windows usr\bin (bash.exe, dash.exe) not found"
} else {
    $bashExe = Join-Path $usrBin "bash.exe"
    $shName = "dash"; if (-not (Test-Path (Join-Path $usrBin "dash.exe"))) { $shName = "sh" }
    $sw = Join-Path $work "sh"
    $stubBin = Join-Path $sw "bin"
    New-Item -ItemType Directory -Force -Path $stubBin | Out-Null
    # Stubs. nohup widens the real window (exec until signal(SIGHUP, SIG_IGN)) to
    # 0.6 s so the hangup below always lands inside it.
    WriteLf (Join-Path $stubBin "sudo") @'
#!/bin/sh
while [ $# -gt 0 ]; do case $1 in -S) read -r _pw; shift ;; -p) shift 2 ;; -*) shift ;; *) break ;; esac; done
exec "$@"
'@
    WriteLf (Join-Path $stubBin "nohup") @'
#!/bin/sh
sleep 0.6
trap '' HUP
exec "$@"
'@
    WriteLf (Join-Path $stubBin "openocd") @'
#!/bin/sh
echo "$*" > "$(dirname "$0")/openocd_args.txt"
echo "Open On-Chip Debugger 0.11.0-stub"
sleep 0.3
echo "Phase D: pulse NRST (PF_4 LOW 250 ms then HIGH)"
echo "  GPIOF_IDR.PF_4 (low)  = 0   (expect 0)"
echo "  GPIOF_IDR.PF_4 (high) = 1   (expect 1)"
echo "shutdown command invoked"
exit 0
'@
    WriteLf (Join-Path $stubBin "pkill") @'
#!/bin/sh
echo "$*" > "$(dirname "$0")/pkill_args.txt"
exit 1
'@
    WriteLf (Join-Path $stubBin "docker") @'
#!/bin/sh
case $1 in
  inspect) exit 1 ;;
  run) echo "STUB_INJECTOR_RAN $*"; exit 0 ;;
  *) exit 0 ;;
esac
'@
    # The A2c2 form: the inline spawn the harness used until 2026-10-19.
    WriteLf (Join-Path $sw "old_inline_reset.sh") @'
( trap : HUP; echo "$1"; nohup openocd -f "$OCD_IFACE_CFG" -f "$OCD_DIR/08_boot_user_app.cfg"; echo OCD_EXIT=$? ) > "$OCD_DIR/ocd_reset.log" 2>&1 &
echo swd_reset_spawned
'@
    # A copy of inject_watch.sh without the new trap = the pre-10-19 start.
    $injText = [System.IO.File]::ReadAllText($injSh)
    $injOld = $injText.Replace("    trap '' HUP`n    nohup sh ", "    nohup sh ")
    Check "control: the pre-10-19 inject_watch.sh start could be rebuilt" ($injOld -ne $injText)
    WriteLf (Join-Path $sw "inject_watch_old.sh") $injOld
    # One driver: run the spawn in its own process group (as the adb/sudo
    # session does), let it return, then SIGHUP that whole group 0.2 s later.
    $driver = @'
exec 2>&1
STUB='@STUB@'; DIR='@DIR@'
export PATH="$STUB:/usr/bin:/bin:$PATH"
export OCD_DIR="$DIR" OCD_GPIO="$DIR/gpio" OCD_IFACE_CFG="$DIR/iface.cfg" OPENOCD="$STUB/openocd" PKILL="$STUB/pkill"
export INJECT_DIR="$DIR" DOCKER="$STUB/docker" INJECT_POLL_S=0.2
chmod +x "$STUB"/* 2>/dev/null
rm -rf "$DIR"; mkdir -p "$DIR/gpio"; rm -f "$STUB"/*_args.txt
printf 'print(1)\n' > "$DIR/fake_inject.py"
echo __SPAWN__
set -m
( echo fio | sudo -S -p '' @SH@ @CMD@ 2>&1 ) &
pg=$!
wait $pg
set +m
sleep 0.2
kill -HUP -- -$pg 2>/dev/null
i=0
while ! grep -q '@MARK@' "$DIR/@LOG@" 2>/dev/null && [ $i -lt @MAXI@ ]; do sleep 0.2; i=$((i + 1)); done
sleep 0.3
echo __LOG__
cat "$DIR/@LOG@"; echo "__OCD_LOG_RC=$?"
echo "__OCD_RUNNING=0"
echo __EXTRA__
echo "pkill=$(cat "$STUB/pkill_args.txt" 2>/dev/null)"
echo "openocd=$(cat "$STUB/openocd_args.txt" 2>/dev/null)"
echo "gpio_export=$(cat "$DIR/gpio/export" 2>/dev/null) gpio_unexport=$(cat "$DIR/gpio/unexport" 2>/dev/null)"
echo __END__
'@
    function RunDrive([string]$Name, [string]$Cmd, [string]$Log, [string]$Mark, [int]$MaxI) {
        $d = Join-Path $sw "run_$Name"
        $t = $driver.Replace("@STUB@", (ToMsys $stubBin)).Replace("@DIR@", (ToMsys $d)).Replace("@SH@", $shName)
        $t = $t.Replace("@CMD@", $Cmd).Replace("@LOG@", $Log).Replace("@MARK@", $Mark).Replace("@MAXI@", "$MaxI")
        $p = Join-Path $sw "drive_$Name.sh"
        WriteLf $p $t
        return @(& $bashExe (ToMsys $p) | ForEach-Object { "$_" })
    }
    $tag = "OCD_LEG=20261019T235959Z"

    # 1. ocd_reset.sh with the hangup: openocd runs, the pulse lands.
    $o = RunDrive "ocd_new" "'$(ToMsys $ocdSh)' $tag" "ocd_reset.log" "^OCD_EXIT=" 50
    $spawn = @(Section $o "__SPAWN__" "__LOG__")
    $log = @(Section $o "__LOG__" "__EXTRA__")
    $extra = @(Section $o "__EXTRA__" "__END__")
    $v = Get-TractorResetVerdict -Output $log -Tag $tag -SpawnOutput $spawn
    Check "ocd_reset.sh + SIGHUP at the hangup: landed" ($v.Verdict -eq "landed" -and $v.Reason -like "*openocd exit 0*") "$($v.Verdict): $($v.Reason) | $($o -join ' / ')"
    Check "ocd_reset.sh: prints exactly swd_reset_spawned" (($spawn -join '|') -eq "swd_reset_spawned") ($spawn -join '|')
    Check "ocd_reset.sh: the log starts with this leg's tag" ($log.Count -gt 0 -and $log[0] -eq $tag) ($log -join '|')
    Check "ocd_reset.sh: pkill -9 openocd, both cfgs, gpio 8/10/15 exported and unexported" (
        ($extra -contains "pkill=-9 openocd") -and
        (@($extra | Where-Object { $_ -like "openocd=-f */iface.cfg -f */run_ocd_new/08_boot_user_app.cfg" }).Count -eq 1) -and
        ($extra -contains "gpio_export=15 gpio_unexport=15")) ($extra -join ' | ')

    # 2. Control, the A2c2 form under the same hangup: SIGHUP, never started.
    $o = RunDrive "ocd_old" "'$(ToMsys (Join-Path $sw 'old_inline_reset.sh'))' $tag" "ocd_reset.log" "^OCD_EXIT=" 50
    $v = Get-TractorResetVerdict -Output @(Section $o "__LOG__" "__EXTRA__") -Tag $tag -SpawnOutput @(Section $o "__SPAWN__" "__LOG__")
    Check "control: the old inline form + SIGHUP reproduces A2c2 (failed, SIGHUP, never started)" ($v.Verdict -eq "failed" -and $v.Reason -like "*SIGHUP*never started*exit 129*") "$($v.Verdict): $($v.Reason)"

    # 3. Usage: no tag, no log.
    $o = RunDrive "ocd_usage" "'$(ToMsys $ocdSh)'" "ocd_reset.log" "^OCD_EXIT=" 1
    Check "ocd_reset.sh without a tag: usage, nothing spawned" (((@(Section $o "__SPAWN__" "__LOG__")) -join '|') -like "usage: sh *ocd_reset.sh OCD_LEG=<UTC stamp>" -and (@(Section $o "__LOG__" "__EXTRA__") -contains "__OCD_LOG_RC=1")) ($o -join ' / ')

    # 4. inject_watch.sh start + hangup: the watcher survives and runs the injector.
    $o = RunDrive "inj_new" "'$(ToMsys $injSh)' start now:test 30 fake_inject.py 5 x" "bench_inject.log" "^INJECT_EXIT " 50
    $spawn = @(Section $o "__SPAWN__" "__LOG__")
    $log = @(Section $o "__LOG__" "__EXTRA__" | Where-Object { $_ -notmatch '^__OCD_' })
    $s = Get-InjectSummary (@($log) + @("INJECT_STOP_DONE"))
    Check "inject_watch.sh start + SIGHUP at the hangup: the injector ran" ($null -ne (Get-InjectWatchPid $spawn) -and $s.Status -eq "finished" -and @($log | Where-Object { $_ -like "STUB_INJECTOR_RAN run --rm --name bench_inject *fake_inject.py 5 x" }).Count -eq 1) "$($s.Status) | $($o -join ' / ')"

    # 5. Control, the pre-10-19 start under the same hangup: the watcher dies.
    $o = RunDrive "inj_old" "'$(ToMsys (Join-Path $sw 'inject_watch_old.sh'))' start now:test 30 fake_inject.py 5 x" "bench_inject.log" "^INJECT_EXIT " 10
    $log = @(Section $o "__LOG__" "__EXTRA__" | Where-Object { $_ -notmatch '^__OCD_' })
    Check "control: the old inject_watch.sh start + SIGHUP loses the watcher (no INJECT_START)" ((@($log | Where-Object { $_ -like "INJECT_START *" }).Count -eq 0) -and (@(Section $o "__SPAWN__" "__LOG__" | Where-Object { $_ -like "INJECT_WATCH_SPAWNED pid=*" }).Count -eq 1)) ($o -join ' / ')
}
Remove-Item -LiteralPath $work -Recurse -Force -ErrorAction SilentlyContinue

Remove-Item -LiteralPath $tmp -ErrorAction SilentlyContinue
Write-Host ""
Write-Host "$script:passes passed, $script:fails failed, $script:skips skipped"
if ($script:fails -gt 0) { exit 1 }
exit 0
