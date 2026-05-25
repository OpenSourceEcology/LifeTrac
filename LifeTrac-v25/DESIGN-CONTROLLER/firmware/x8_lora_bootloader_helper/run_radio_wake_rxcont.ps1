# SPDX-License-Identifier: MIT
# Bench helper: wake every L072 SX1276 into LoRa RXCONT (RegOpMode = 0x85).
#
# Usage:
#   .\run_radio_wake_rxcont.ps1                   # auto-discover all adb devices
#   .\run_radio_wake_rxcont.ps1 -AdbSerial 2D0A1209DABC240B
#   .\run_radio_wake_rxcont.ps1 -AdbSerial @('2D0A...','2E2C...')

[CmdletBinding()]
param(
    [string[]]$AdbSerial,
    [string]$WakePath,
    [string]$V2ProbePath,
    [string]$V1ProbePath,
    [string]$RemoteDir = '/tmp/lifetrac_p0c',
    [string]$Dev = '/dev/ttymxc3',
    [string]$Baud = '921600',
    [switch]$SkipPush
)

$ErrorActionPreference = 'Stop'

$scriptDir = $PSScriptRoot
if (-not $scriptDir) {
    $scriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
}
if (-not $WakePath)    { $WakePath    = Join-Path $scriptDir 'w2_02_radio_wake_rxcont.py' }
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
        [string]$WakeLocal,
        [string]$V2ProbeLocal,
        [string]$V1ProbeLocal
    )
    & adb -s $Serial shell "mkdir -p $RemoteDir" | Out-Null
    & adb -s $Serial push $WakeLocal "$RemoteDir/w2_02_radio_wake_rxcont.py" | Out-Null
    & adb -s $Serial push $V1ProbeLocal "$RemoteDir/method_g_stage1_probe.py" | Out-Null
    & adb -s $Serial push $V2ProbeLocal "$RemoteDir/method_h_stage2_tx_probe_v2.py" | Out-Null
}

function Invoke-RadioWake {
    param(
        [string]$Serial,
        [string]$RemoteDir,
        [string]$Dev,
        [string]$Baud
    )
    $remoteCmd = "echo fio | sudo -S sh -c 'cd $RemoteDir && python3 w2_02_radio_wake_rxcont.py --dev $Dev --baud $Baud' 2>&1"
    $stdout = & adb -s $Serial shell $remoteCmd
    return ,$stdout
}

function Parse-Result {
    param([string[]]$Lines)
    $resultLine = $Lines | Where-Object { $_ -match '__W2_02_WAKE_(OK|FAIL)__' } | Select-Object -Last 1
    if (-not $resultLine) {
        return [pscustomobject]@{
            status = 'NO_RESULT_LINE'
            opmode_pre = ''
            opmode_post = ''
            raw = ($Lines -join "`n")
        }
    }
    $status = if ($resultLine -match '__W2_02_WAKE_OK__') { 'WAKE_OK' } else { 'WAKE_FAIL' }
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
                -WakeLocal $WakePath -V2ProbeLocal $V2ProbePath `
                -V1ProbeLocal $V1ProbePath
        } catch {
            Write-Warning "push failed for $s : $_"
            $rows += [pscustomobject]@{
                serial = $s; status = 'PUSH_FAIL'; pre = ''; post = ''
            }
            continue
        }
    }
    $lines = Invoke-RadioWake -Serial $s -RemoteDir $RemoteDir -Dev $Dev -Baud $Baud
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
Write-Host "=== radio_wake summary ==="
$rows | Format-Table -AutoSize | Out-String | Write-Host

$ok = $rows | Where-Object { $_.status -eq 'WAKE_OK' }
if ($ok.Count -ne $rows.Count) {
    Write-Host "__BENCH_RADIO_WAKE_AUDIT__=FAIL ($($rows.Count - $ok.Count)/$($rows.Count) board(s) not in RXCONT)"
    exit 2
}

Write-Host "__BENCH_RADIO_WAKE_AUDIT__=PASS (all $($rows.Count) board(s) in RXCONT 0x85)"
exit 0
