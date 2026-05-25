#requires -Version 5.1
<#
.SYNOPSIS
    Sweep gpio163-NRST settle time + RESET_REQ on/off on the RX 2D0A board
    to find a configuration where VER warm-up reliably succeeds.

.DESCRIPTION
    Falsification harness for the 2026-05-25 smoke failure
    (AI NOTES/2026-05-25_Smoke_RX_VER_Fail_TX_All_TIMEOUT.md). The smoke
    used gpio163 NRST + 1.5 s host sleep + image_rx_daemon._open_link()
    which itself does `link.send(0x03)` (RESET_REQ) + drain_boot(1.5 s) +
    VER_REQ(1.0 s). 2D0A times out at the VER_REQ step every time.

    This script breaks each variable out so we can attribute failure to:
      - too-short post-NRST settle?
      - double-reset (gpio163 + UART RESET_REQ) confusing the L072?
      - residual COBS chatter not draining in 1.5 s?

    For each (settle_s, skip_reset_req) combination, it:
      1) Pulses gpio163 NRST (unless -SkipNrst is also requested),
      2) Sleeps for the host preflight settle window,
      3) Runs rx_ver_warmup_diag.py inside the daemon container with the
         requested settle/skip-reset arguments and --attempts N.

    Output: one line per cell, plus a final pass/fail matrix saved to
    LifeTrac-v25/AI NOTES/.

.EXAMPLE
    powershell -NoProfile -ExecutionPolicy Bypass -File `
        LifeTrac-v25/DESIGN-CONTROLLER/firmware/x8_lora_bootloader_helper/run_rx_ver_warmup_sweep.ps1
#>
param(
    [string]$AdbSerial = "2D0A1209DABC240B",
    [string]$WorkDir   = "/tmp/lifetrac_strict",
    [int]$AttemptsPerCell = 3,
    [double[]]$SettleSeconds = @(1.5, 3.0, 5.0),
    [switch]$IncludeResetReqOn,
    [switch]$SkipNrst,
    [int]$HostPostNrstSleepMs = 1500,
    [string]$OutDir = "LifeTrac-v25/AI NOTES"
)

Set-StrictMode -Version Latest
$ErrorActionPreference = "Stop"

if (-not (Test-Path $OutDir)) {
    New-Item -ItemType Directory -Path $OutDir | Out-Null
}
$stamp  = Get-Date -Format "yyyyMMdd_HHmmss"
$logTxt = Join-Path $OutDir ("2026-05-25_rx_ver_sweep_${AdbSerial}_${stamp}.log")

$image = "hub.foundries.io/arduino/arduino-ootb-python-devel:738bc44"
$nrstCmd = "echo fio | sudo -S -p '' sh -c '[ -d /sys/class/gpio/gpio163 ] || echo 163 > /sys/class/gpio/export; echo out > /sys/class/gpio/gpio163/direction; echo 1 > /sys/class/gpio/gpio163/value; sleep 0.02; echo 0 > /sys/class/gpio/gpio163/value; sleep 0.10; echo 1 > /sys/class/gpio/gpio163/value; echo NRST_RELEASED_AT=`$(date +%s.%N)'"

function Write-Both([string]$msg) {
    Write-Host $msg
    Add-Content -Path $logTxt -Value $msg
}

# Push the diagnostic script onto the board.
$diagSrc = "LifeTrac-v25/DESIGN-CONTROLLER/firmware/x8_lora_bootloader_helper/rx_ver_warmup_diag.py"
Write-Both "=== rx_ver_warmup_sweep ==="
Write-Both "AdbSerial=$AdbSerial AttemptsPerCell=$AttemptsPerCell SettleSeconds=$($SettleSeconds -join ',')"
Write-Both "IncludeResetReqOn=$IncludeResetReqOn SkipNrst=$SkipNrst HostPostNrstSleepMs=$HostPostNrstSleepMs"
Write-Both "Pushing $diagSrc -> ${WorkDir}/rx_ver_warmup_diag.py ..."
& adb -s $AdbSerial push $diagSrc "${WorkDir}/rx_ver_warmup_diag.py" | Out-Null
if ($LASTEXITCODE -ne 0) { throw "adb push failed (exit=$LASTEXITCODE)" }

# Build matrix.
$resetVariants = @($true)        # skip_reset_req = $true (default: no UART reset)
if ($IncludeResetReqOn) {
    $resetVariants = @($true, $false)  # also test reset_req on
}

$results = New-Object System.Collections.Generic.List[object]
$cellIdx = 0
foreach ($settle in $SettleSeconds) {
    foreach ($skipReset in $resetVariants) {
        $cellIdx++
        $cellId = "settle=$settle skipReset=$skipReset"
        Write-Both ""
        Write-Both "--- cell $cellIdx : $cellId ---"

        # NRST pulse (optional).
        if (-not $SkipNrst) {
            Write-Both "  pulsing gpio163 NRST..."
            & adb -s $AdbSerial shell $nrstCmd | Tee-Object -FilePath $logTxt -Append | Out-Null
            Start-Sleep -Milliseconds $HostPostNrstSleepMs
        } else {
            Write-Both "  (SkipNrst: not pulsing gpio163)"
        }

        # Build docker invocation. Container lifetime = settle*attempts + 10 s slack.
        $containerTimeout = [int]([math]::Ceiling($settle * $AttemptsPerCell + 10))
        $skipFlag = if ($skipReset) { "--skip-reset-req" } else { "" }
        $dockerCmd = @(
            "echo fio | sudo -S -p '' docker run --rm --network=host --device=/dev/ttymxc3",
            "-v ${WorkDir}:/work -w /work",
            "-e PYTHONPATH=/work:/work/paho",
            "--entrypoint timeout",
            $image,
            "$containerTimeout python3 -u /work/rx_ver_warmup_diag.py",
            "--attempts $AttemptsPerCell",
            "--settle-s $settle",
            $skipFlag,
            "--log-level INFO"
        ) -join " "

        Write-Both "  cmd: $dockerCmd"
        $cellLog = & adb -s $AdbSerial shell $dockerCmd 2>&1 | Out-String
        Add-Content -Path $logTxt -Value $cellLog

        # Parse RESULT_JSON lines.
        $okCount = 0
        $total = 0
        $latencies = @()
        $faults = @()
        foreach ($line in $cellLog -split "`r?`n") {
            if ($line -match '^RESULT_JSON (.+)$') {
                $total++
                try {
                    $j = $Matches[1] | ConvertFrom-Json
                    if ($j.ver_ok) {
                        $okCount++
                        $latencies += [double]$j.ver_latency_s
                    } else {
                        $faults += "$($j.ver_error)"
                    }
                } catch {
                    Write-Both "  (warn: failed to parse RESULT_JSON: $($_.Exception.Message))"
                }
            }
        }

        $cellResult = [pscustomobject]@{
            settle_s        = $settle
            skip_reset_req  = $skipReset
            attempts        = $total
            ver_ok          = $okCount
            ver_latency_s   = if ($latencies.Count -gt 0) {
                                  [math]::Round(($latencies | Measure-Object -Average).Average, 4)
                              } else { $null }
            first_error     = if ($faults.Count -gt 0) { $faults[0] } else { "" }
        }
        $results.Add($cellResult)
        Write-Both ("  -> ver_ok={0}/{1} avg_latency={2} err='{3}'" -f `
            $cellResult.ver_ok, $cellResult.attempts, $cellResult.ver_latency_s, $cellResult.first_error)
    }
}

Write-Both ""
Write-Both "=== MATRIX ==="
$tbl = $results | Format-Table -AutoSize | Out-String
Write-Both $tbl

# Determine overall success.
$anyAllPass = @($results | Where-Object { $_.attempts -gt 0 -and $_.ver_ok -eq $_.attempts })
if ($anyAllPass.Count -gt 0) {
    Write-Both "SUCCESS: $($anyAllPass.Count) cell(s) achieved ver_ok=attempts"
    Write-Both "Log saved to $logTxt"
    exit 0
} else {
    Write-Both "FAIL: no cell achieved ver_ok=attempts. Log saved to $logTxt"
    exit 1
}
