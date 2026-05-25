#requires -Version 5.1
<#
.SYNOPSIS
    Bisect rx_frames=0 between firmware-side drop vs. host UART drop, using
    NRST pulse (gpio163) to boot the L072 instead of OpenOCD. Avoids the
    wedged 2D0A SWD path that blocks run_w1_10b_rx_pair_end_to_end.ps1.

.DESCRIPTION
    1. Pulses gpio163 NRST on both boards (same recipe as concurrent_smoke).
    2. Runs method_h_stage2_tx_probe_v2.py --probe rx_listen in docker on RX
       (2D0A) — listens on /dev/ttymxc3 for HOST_TYPE_RX_FRAME_URC (0x91)
       and reports rx_count + radio_rx_ok_delta.
    3. Waits for __W1_10B_LISTEN_READY__, then launches
       method_h_stage2_tx_probe_v2.py --probe tx_burst on TX (2E2C).
    4. Collects both stdout streams and prints the headline result.

    Headline interpretation:
      rx_count > 0 AND radio_rx_ok_delta > 0
        => RX path fully working. The original rx_frames=0 was a daemon
           layer or env issue.
      rx_count == 0 AND radio_rx_ok_delta > 0
        => firmware sx1276_rx_service() returned true and emitted URC, but
           UART/COBS host transport is dropping them. Investigate host_uart.
      rx_count == 0 AND radio_rx_ok_delta == 0 AND tx_ok > 0
        => firmware's sx1276_rx_service() is returning false on every
           RxDone. Investigate the FHSS-header gate, CRC error path,
           or DIO0 event delivery.
#>
param(
    [string]$TxAdbSerial = "2E2C1209DABC240B",
    [string]$RxAdbSerial = "2D0A1209DABC240B",
    [int]$TxCycles       = 30,
    [double]$TxInterS    = 0.25,
    [int]$ExtraRxWindowS = 15,
    [string]$RegProfile  = "0"
)

Set-StrictMode -Version Latest
$ErrorActionPreference = "Stop"

$helperDir = Join-Path $PSScriptRoot "."
$stamp = Get-Date -Format "yyyyMMdd_HHmmss"
$evidenceDir = Join-Path $PSScriptRoot ("../../bench-evidence/rx_pair_nrst_{0}" -f $stamp)
$null = New-Item -ItemType Directory -Force -Path $evidenceDir
Write-Host "Evidence dir: $evidenceDir" -ForegroundColor Cyan

$rxLog = Join-Path $evidenceDir "rx_listen.log"
$rxErr = Join-Path $evidenceDir "rx_listen.err"
$txLog = Join-Path $evidenceDir "tx_burst.log"
$txErr = Join-Path $evidenceDir "tx_burst.err"
"" | Out-File -FilePath $rxLog -Encoding ascii
"" | Out-File -FilePath $txLog -Encoding ascii

Write-Host "Cleaning up any prior containers..."
& adb -s $TxAdbSerial shell "echo fio | sudo -S docker rm -f tx_pair" 2>&1 | Out-Null
& adb -s $RxAdbSerial shell "echo fio | sudo -S docker rm -f rx_pair" 2>&1 | Out-Null

Write-Host "Pushing helper dir to both boards..."
foreach ($s in @($RxAdbSerial, $TxAdbSerial)) {
    & adb -s $s shell "echo fio | sudo -S -p '' mkdir -p /tmp/lifetrac_strict; echo fio | sudo -S -p '' chmod 0777 /tmp/lifetrac_strict" | Out-Null
    & adb -s $s push "$helperDir/." /tmp/lifetrac_strict/ | Out-Null
}

Write-Host "Pulsing gpio163 (L072 NRST) on both boards..."
$nrstCmd = "echo fio | sudo -S -p '' sh -c '[ -d /sys/class/gpio/gpio163 ] || echo 163 > /sys/class/gpio/export; echo out > /sys/class/gpio/gpio163/direction; echo 1 > /sys/class/gpio/gpio163/value; sleep 0.02; echo 0 > /sys/class/gpio/gpio163/value; sleep 0.10; echo 1 > /sys/class/gpio/gpio163/value; echo NRST_RELEASED_AT=`$(date +%s.%N)'"
& adb -s $TxAdbSerial shell $nrstCmd | Out-Null
& adb -s $RxAdbSerial shell $nrstCmd | Out-Null
Start-Sleep -Milliseconds 1500

# RX window: TX duration + warm-up + margin.
$rxWindowS = [int][math]::Ceiling($TxCycles * ($TxInterS + 0.1)) + $ExtraRxWindowS + 5
$rxContainerSec = $rxWindowS + 10
$txContainerSec = [int][math]::Ceiling($TxCycles * ($TxInterS + 0.1)) + 15

# Probe-v2 wants --rx-window N (seconds).
$rxArg = "-s $RxAdbSerial shell `"echo fio | sudo -S -p '' docker run --name rx_pair --rm --network=host --device=/dev/ttymxc3 -v /tmp/lifetrac_strict:/work -w /work -e PYTHONPATH=/work:/work/paho -e LIFETRAC_SKIP_RESET_REQ=1 -e LIFETRAC_REG_PROFILE=$RegProfile --entrypoint timeout hub.foundries.io/arduino/arduino-ootb-python-devel:738bc44 $rxContainerSec python3 -u /work/method_h_stage2_tx_probe_v2.py --dev /dev/ttymxc3 --baud 921600 --probe rx_listen --rx-window $rxWindowS`""

Write-Host "Launching RX listener (window=${rxWindowS}s, container-timeout=${rxContainerSec}s)..." -ForegroundColor Yellow
$rxProc = Start-Process -FilePath "adb" -ArgumentList $rxArg -RedirectStandardOutput $rxLog -RedirectStandardError $rxErr -PassThru -NoNewWindow

$readyDeadline = (Get-Date).AddSeconds(40)
$ready = $false
while ((Get-Date) -lt $readyDeadline) {
    if (Test-Path $rxLog) {
        $rxText = Get-Content $rxLog -Raw -ErrorAction SilentlyContinue
        if ($rxText -and $rxText -match "__W1_10B_LISTEN_READY__") { $ready = $true; break }
    }
    Start-Sleep -Milliseconds 250
}
if (-not $ready) {
    try { $rxProc.Kill() } catch {}
    Write-Host "RX listener did NOT print __W1_10B_LISTEN_READY__ within 40 s." -ForegroundColor Red
    Write-Host "--- rx_listen.log (tail 80) ---"
    Get-Content $rxLog -Tail 80 -ErrorAction SilentlyContinue
    Write-Host "--- rx_listen.err (tail 40) ---"
    Get-Content $rxErr -Tail 40 -ErrorAction SilentlyContinue
    exit 2
}
Write-Host "RX listener READY. Keying TX burst (cycles=$TxCycles inter=${TxInterS}s)..." -ForegroundColor Green

$txArg = "-s $TxAdbSerial shell `"echo fio | sudo -S -p '' docker run --name tx_pair --rm --network=host --device=/dev/ttymxc3 -v /tmp/lifetrac_strict:/work -w /work -e PYTHONPATH=/work:/work/paho -e LIFETRAC_SKIP_RESET_REQ=1 -e LIFETRAC_REG_PROFILE=$RegProfile --entrypoint timeout hub.foundries.io/arduino/arduino-ootb-python-devel:738bc44 $txContainerSec python3 -u /work/method_h_stage2_tx_probe_v2.py --dev /dev/ttymxc3 --baud 921600 --probe tx_burst --tx-count $TxCycles --inter-cycle-s $TxInterS`""
$txProc = Start-Process -FilePath "adb" -ArgumentList $txArg -RedirectStandardOutput $txLog -RedirectStandardError $txErr -PassThru -NoNewWindow

$wait = (Get-Date).AddSeconds($rxContainerSec + 15)
while (((Get-Date) -lt $wait) -and (-not $rxProc.HasExited -or -not $txProc.HasExited)) {
    Start-Sleep -Seconds 1
}
if (-not $rxProc.HasExited) { try { $rxProc.Kill() } catch {} }
if (-not $txProc.HasExited) { try { $txProc.Kill() } catch {} }

Write-Host "`n=== RX_LISTEN tail ($rxLog) ===" -ForegroundColor Green
Get-Content $rxLog -Tail 80
Write-Host "`n=== TX_BURST tail ($txLog) ===" -ForegroundColor Green
Get-Content $txLog -Tail 40

# Parse the headline tokens.
$rxFull = (Get-Content $rxLog -Raw -ErrorAction SilentlyContinue)
$txFull = (Get-Content $txLog -Raw -ErrorAction SilentlyContinue)

$rxFrames = -1
$rxOkDelta = -1
$crcDelta = -1
if ($rxFull -and $rxFull -match "__W1_10B_LISTEN_DONE__ rx_frames=(\d+) radio_rx_ok_delta=(\d+) radio_crc_err_delta=(\d+)") {
    $rxFrames = [int]$Matches[1]
    $rxOkDelta = [int]$Matches[2]
    $crcDelta = [int]$Matches[3]
}
$txOk = -1
if ($txFull -and $txFull -match "tx_ok=(\d+)") { $txOk = [int]$Matches[1] }

Write-Host "`n================ HEADLINE ================" -ForegroundColor Cyan
Write-Host ("  TX tx_ok               = {0}/{1}" -f $txOk, $TxCycles)
Write-Host ("  RX rx_frames (URCs)    = {0}" -f $rxFrames)
Write-Host ("  RX radio_rx_ok_delta   = {0}" -f $rxOkDelta)
Write-Host ("  RX radio_crc_err_delta = {0}" -f $crcDelta)
Write-Host "==========================================" -ForegroundColor Cyan

if ($rxFrames -gt 0 -and $rxOkDelta -gt 0) {
    Write-Host "VERDICT: RX path fully working. The rx_frames=0 baseline must be a daemon-layer issue." -ForegroundColor Green
} elseif ($rxFrames -eq 0 -and $rxOkDelta -gt 0) {
    Write-Host "VERDICT: Firmware emits URC but host UART is dropping them. Investigate host_uart / COBS." -ForegroundColor Yellow
} elseif ($rxFrames -eq 0 -and $rxOkDelta -eq 0 -and $crcDelta -eq 0 -and $txOk -gt 0) {
    Write-Host "VERDICT: Firmware sx1276_rx_service() never returns true. Header-gate / DIO0 / opmode." -ForegroundColor Yellow
} elseif ($rxFrames -eq 0 -and $crcDelta -gt 0) {
    Write-Host "VERDICT: Frames demodulating with CRC errors. RF chain or modem config asymmetric." -ForegroundColor Yellow
} else {
    Write-Host "VERDICT: Inconclusive - check logs above." -ForegroundColor Red
}
