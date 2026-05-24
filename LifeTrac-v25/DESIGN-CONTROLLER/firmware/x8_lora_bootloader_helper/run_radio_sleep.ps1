# SPDX-License-Identifier: MIT
# Bench EMC helper: park every L072 SX1276 in LoRa SLEEP (RegOpMode = 0x80).
#
# Usage:
#   .\run_radio_sleep.ps1                   # auto-discover all adb devices
#   .\run_radio_sleep.ps1 -AdbSerial 2D0A1209DABC240B
#   .\run_radio_sleep.ps1 -AdbSerial @('2D0A...','2E2C...')
#
# Per board it (a) ensures /tmp/lifetrac_p0c/radio_sleep.py is fresh,
# (b) invokes it under `sudo -S` (X8 password = 'fio'), (c) parses the
# __RADIO_SLEEP_RESULT__ line, (d) prints a one-row summary.
# Exit code: 0 iff every board returned status=SLEEP_OK or SLEEP_ALREADY.

[CmdletBinding()]
param(
    [string[]]$AdbSerial,
    [string]$ProbePath,
    [string]$V2ProbePath,
    [string]$V1ProbePath,
    [string]$RemoteDir = '/tmp/lifetrac_p0c',
    [string]$Dev = '/dev/ttymxc3',
    [string]$Baud = '921600',
    [switch]$SkipPush
)

$ErrorActionPreference = 'Stop'

# Resolve script-relative defaults here (param-block default-eval can see
# $PSScriptRoot as empty depending on how the script was invoked).
$scriptDir = $PSScriptRoot
if (-not $scriptDir) {
    $scriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
}
if (-not $ProbePath)   { $ProbePath   = Join-Path $scriptDir 'radio_sleep.py' }
if (-not $V2ProbePath) { $V2ProbePath = Join-Path $scriptDir 'method_h_stage2_tx_probe_v2.py' }
if (-not $V1ProbePath) { $V1ProbePath = Join-Path $scriptDir 'method_g_stage1_probe.py' }

function Resolve-AdbSerials {
    param([string[]]$Requested)
    if ($Requested -and $Requested.Count -gt 0) {
        return $Requested
    }
    $raw = & adb devices 2>$null
    $serials = @()
    foreach ($line in $raw) {
        if ($line -match '^(\S+)\s+device$') {
            $serials += $Matches[1]
        }
    }
    if ($serials.Count -eq 0) {
        throw "No adb devices found. Pass -AdbSerial explicitly."
    }
    return $serials
}

function Push-ProbeFiles {
    param(
        [string]$Serial,
        [string]$RemoteDir,
        [string]$RadioSleepLocal,
        [string]$V2ProbeLocal,
        [string]$V1ProbeLocal
    )
    & adb -s $Serial shell "mkdir -p $RemoteDir" | Out-Null
    & adb -s $Serial push $RadioSleepLocal "$RemoteDir/radio_sleep.py" | Out-Null
    
    # V1 (method_g_stage1_probe) is imported by V2 probe. Ensure it exists as well.
    $existsV1 = & adb -s $Serial shell "test -f $RemoteDir/method_g_stage1_probe.py && echo PRESENT || echo MISSING"
    if ($existsV1 -match 'MISSING') {
        & adb -s $Serial push $V1ProbeLocal "$RemoteDir/method_g_stage1_probe.py" | Out-Null
    }

    # V2 probe carries the HostLink class + SX1276 helpers that radio_sleep imports.
    # Only push if remote copy is missing — it's 93 KB and rarely changes here.
    $existsOut = & adb -s $Serial shell "test -f $RemoteDir/method_h_stage2_tx_probe_v2.py && echo PRESENT || echo MISSING"
    if ($existsOut -match 'MISSING') {
        & adb -s $Serial push $V2ProbeLocal "$RemoteDir/method_h_stage2_tx_probe_v2.py" | Out-Null
    }
}

function Invoke-RadioSleep {
    param(
        [string]$Serial,
        [string]$RemoteDir,
        [string]$Dev,
        [string]$Baud
    )
    $remoteCmd = "echo fio | sudo -S sh -c 'cd $RemoteDir && python3 radio_sleep.py --dev $Dev --baud $Baud' 2>&1"
    $stdout = & adb -s $Serial shell $remoteCmd
    return ,$stdout
}

function Parse-Result {
    param([string[]]$Lines)
    $resultLine = $Lines | Where-Object { $_ -match '__RADIO_SLEEP_RESULT__' } | Select-Object -Last 1
    if (-not $resultLine) {
        return [pscustomobject]@{
            status = 'NO_RESULT_LINE'
            opmode_pre = ''
            opmode_post = ''
            raw = ($Lines -join "`n")
        }
    }
    $status = if ($resultLine -match 'status=(\S+)') { $Matches[1] } else { 'UNKNOWN' }
    $pre = if ($resultLine -match 'opmode_pre=(0x[0-9A-Fa-f]+)') { $Matches[1] } else { '' }
    $post = if ($resultLine -match 'opmode_post=(0x[0-9A-Fa-f]+)') { $Matches[1] } else { '' }
    return [pscustomobject]@{
        status = $status
        opmode_pre = $pre
        opmode_post = $post
        raw = $resultLine
    }
}

# --- main ---
$serials = Resolve-AdbSerials -Requested $AdbSerial
Write-Host "Targets: $($serials -join ', ')"

$rows = @()
foreach ($s in $serials) {
    Write-Host "----- $s -----"
    if (-not $SkipPush) {
        try {
            Push-ProbeFiles -Serial $s -RemoteDir $RemoteDir `
                -RadioSleepLocal $ProbePath -V2ProbeLocal $V2ProbePath `
                -V1ProbeLocal $V1ProbePath
        } catch {
            Write-Warning "push failed for $s : $_"
            $rows += [pscustomobject]@{
                serial = $s; status = 'PUSH_FAIL'; pre = ''; post = ''
            }
            continue
        }
    }
    $lines = Invoke-RadioSleep -Serial $s -RemoteDir $RemoteDir -Dev $Dev -Baud $Baud
    $lines | ForEach-Object { Write-Host "  $_" }
    $parsed = Parse-Result -Lines $lines
    $rows += [pscustomobject]@{
        serial = $s
        status = $parsed.status
        pre = $parsed.opmode_pre
        post = $parsed.opmode_post
    }
}

Write-Host ""
Write-Host "=== radio_sleep summary ==="
$rows | Format-Table -AutoSize | Out-String | Write-Host

$ok = $rows | Where-Object { $_.status -in @('SLEEP_OK','SLEEP_ALREADY') }
if ($ok.Count -eq $rows.Count) {
    Write-Host "__BENCH_RADIO_SLEEP_AUDIT__=PASS (all $($rows.Count) board(s) parked at 0x80)"
    exit 0
} else {
    $bad = $rows | Where-Object { $_.status -notin @('SLEEP_OK','SLEEP_ALREADY') }
    Write-Host "__BENCH_RADIO_SLEEP_AUDIT__=FAIL ($($bad.Count)/$($rows.Count) board(s) not in SLEEP)"
    exit 1
}
