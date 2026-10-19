#requires -Version 5.1
<#
.SYNOPSIS
    Offline tests for the helper functions of run_live_radio_monitor.ps1.
.DESCRIPTION
    Loads the helper functions out of the harness through the PowerShell AST
    (the harness body never runs: no adb, no board, no radio) and checks them
    against sample adb output, including the \r\r\n line ends of adb from
    platform-tools 37 as PowerShell really receives them from `cmd /c`.

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
           "Get-InjectWatchPid", "Get-InjectSummary", "Get-TractorResetVerdict")
foreach ($n in $names) {
    $fn = $ast.Find({ param($x) $x -is [System.Management.Automation.Language.FunctionDefinitionAst] -and $x.Name -eq $n }, $true)
    if (-not $fn) { Write-Host "MISSING function $n in the harness"; exit 2 }
    . ([scriptblock]::Create($fn.Extent.Text))
}

$script:fails = 0
$script:passes = 0
function Check([string]$Name, [bool]$Cond, [string]$Got = "") {
    if ($Cond) { $script:passes++; Write-Host "PASS $Name" }
    else { $script:fails++; Write-Host "FAIL $Name  (got: $Got)" -ForegroundColor Red }
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
$v = Get-TractorResetVerdict -Output (CmdLines "$tag`nOpen On-Chip Debugger 0.11.0`nInfo : SWD DPIDR 0x6ba02477`nOCD_EXIT=137`n__OCD_LOG_RC=0`n__OCD_RUNNING=0`n") -Tag $tag
Check "reset signal: SIGKILL after openocd started" ($v.Verdict -eq "failed" -and $v.Reason -like "*SIGKILL*exit 137*" -and $v.Reason -notlike "*never started*") $v.Reason
$v = Get-TractorResetVerdict -Output (CmdLines "$tag`nOpen On-Chip Debugger 0.11.0`nOCD_EXIT=137`n__OCD_LOG_RC=0`n__OCD_RUNNING=1`n") -Tag $tag
Check "reset signal: an exit line beats a running openocd (not 'hung')" ($v.Verdict -eq "failed" -and $v.Reason -like "*SIGKILL*" -and $v.Reason -notlike "*hung*") $v.Reason
$v = Get-TractorResetVerdict -Output (CmdLines ("$tag`n" + $phases + "OCD_EXIT=129`n__OCD_LOG_RC=0`n__OCD_RUNNING=0`n")) -Tag $tag
Check "reset signal: a pulse read back before the signal is still landed" ($v.Verdict -eq "landed" -and $v.Reason -like "*exit 129 (signal 1)*") $v.Reason
$v = Get-TractorResetVerdict -Output (CmdLines "$tag`nOpen On-Chip Debugger 0.11.0`nError: unable to open imx_gpio`nOCD_EXIT=1`n__OCD_LOG_RC=0`n__OCD_RUNNING=0`n") -Tag $tag
Check "reset signal: a plain exit 1 keeps the error-code wording" ($v.Verdict -eq "failed" -and $v.Reason -like "openocd exit 1 before the NRST pulse: Error: unable*") $v.Reason

Remove-Item -LiteralPath $tmp -ErrorAction SilentlyContinue
Write-Host ""
Write-Host "$script:passes passed, $script:fails failed"
if ($script:fails -gt 0) { exit 1 }
exit 0
