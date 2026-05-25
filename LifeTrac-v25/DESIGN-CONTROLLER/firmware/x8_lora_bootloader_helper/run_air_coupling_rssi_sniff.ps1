#requires -Version 5.1
<#
.SYNOPSIS
    Falsify whether 2D0A's SX1276 sees any RF energy when 2E2C is keying
    a tx_burst at REG_PROFILE=0 (915 MHz / BW250 / SF7).

.DESCRIPTION
    Sniffs RegRssiValue (0x1B), RegPktRssi (0x1A), RegIrqFlags (0x12) on the
    RX peer over a fixed window while a known-good tx_burst runs on the TX
    peer. Same NRST pulse + REG_PROFILE override that the daemons use.

    Outcome decision tree:
      RSSI elevates from ~-120 dBm to >= -100 dBm during TX
        => air link OK; bug is in RX-side demod/IRQ/RX_FRAME_URC plumbing.
      RSSI stays flat at noise floor (~-120 dBm) for the entire window
        => air link broken (antenna, RF switch, TX power, or wrong FRF).
      Pkt RSSI non-zero AND irq_flags shows RxDone (0x40)
        => demod fires; bug is *between* RxDone IRQ and host RX_FRAME_URC.

.NOTES
    Uses the existing /tmp/lifetrac_strict staging dir (same as the smoke).
    Pushes air_coupling_rssi_sniff.py and method_h_stage2_tx_probe_v2.py to
    each board before launching.
#>
param(
    [string]$TxAdbSerial = "2E2C1209DABC240B",
    [string]$RxAdbSerial = "2D0A1209DABC240B",
    [int]$DurationS      = 25,
    [int]$TxCycles       = 60,
    [double]$TxInterS    = 0.25,
    [string]$RegProfile  = "0"
)

Set-StrictMode -Version Latest
$ErrorActionPreference = "Stop"

$helperDir = Join-Path $PSScriptRoot "."
$evidenceDir = "air_coupling_evidence_{0:yyyyMMdd_HHmmss}" -f (Get-Date)
New-Item -ItemType Directory -Path $evidenceDir | Out-Null
Write-Host "Evidence dir: $evidenceDir"

$rxLog = Join-Path $evidenceDir "rx_rssi_sniff.log"
$rxErr = Join-Path $evidenceDir "rx_rssi_sniff.err"
$txLog = Join-Path $evidenceDir "tx_burst.log"
$txErr = Join-Path $evidenceDir "tx_burst.err"
"" | Out-File -FilePath $rxLog -Encoding ascii
"" | Out-File -FilePath $txLog -Encoding ascii

Write-Host "Cleaning up any prior containers..."
& adb -s $TxAdbSerial shell "echo fio | sudo -S docker rm -f tx_air_burst" 2>&1 | Out-Null
& adb -s $RxAdbSerial shell "echo fio | sudo -S docker rm -f rx_air_sniff" 2>&1 | Out-Null

Write-Host "Pushing helper files to both boards..."
# 2026-05-25: previously this script pushed only the two probe scripts, but
# method_h_stage2_tx_probe_v2.py imports method_g_stage1_probe (and other
# helper modules). Without the full helper dir on /tmp/lifetrac_strict the
# TX container fails with `ModuleNotFoundError: No module named
# 'method_g_stage1_probe'` and the RSSI sniff measures dead air. Mirror
# what run_method_h_stage2_tx_end_to_end.ps1 does: push the whole helper
# directory so every transitive import resolves inside the container.
foreach ($s in @($RxAdbSerial, $TxAdbSerial)) {
    & adb -s $s shell "echo fio | sudo -S -p '' mkdir -p /tmp/lifetrac_strict; echo fio | sudo -S -p '' chmod 0777 /tmp/lifetrac_strict" | Out-Null
    & adb -s $s push "$helperDir/." /tmp/lifetrac_strict/ | Out-Null
}

Write-Host "Pulsing gpio163 (L072 NRST) on both boards..."
$nrstCmd = "echo fio | sudo -S -p '' sh -c '[ -d /sys/class/gpio/gpio163 ] || echo 163 > /sys/class/gpio/export; echo out > /sys/class/gpio/gpio163/direction; echo 1 > /sys/class/gpio/gpio163/value; sleep 0.02; echo 0 > /sys/class/gpio/gpio163/value; sleep 0.10; echo 1 > /sys/class/gpio/gpio163/value; echo NRST_RELEASED_AT=`$(date +%s.%N)'"
& adb -s $TxAdbSerial shell $nrstCmd | Out-Null
& adb -s $RxAdbSerial shell $nrstCmd | Out-Null
Start-Sleep -Milliseconds 1500

$rxContainerSec = $DurationS + 8
$txContainerSec = [int][math]::Ceiling($TxCycles * $TxInterS) + 8

$rxArg = "-s $RxAdbSerial shell `"echo fio | sudo -S -p '' docker run --name rx_air_sniff --rm --network=host --device=/dev/ttymxc3 -v /tmp/lifetrac_strict:/work -w /work -e PYTHONPATH=/work:/work/paho -e LIFETRAC_SKIP_RESET_REQ=1 -e LIFETRAC_REG_PROFILE=$RegProfile --entrypoint timeout hub.foundries.io/arduino/arduino-ootb-python-devel:738bc44 $rxContainerSec python3 -u /work/air_coupling_rssi_sniff.py --duration-s $DurationS --interval-s 0.25 --role rx`""

Write-Host "Launching RX RSSI sniffer (container timeout = ${rxContainerSec}s, sniff window = ${DurationS}s)..."
$rxProc = Start-Process -FilePath "adb" -ArgumentList $rxArg -RedirectStandardOutput $rxLog -RedirectStandardError $rxErr -PassThru -NoNewWindow

# Wait for the RX sniffer to print __RSSI_SNIFF_READY__ before keying TX.
$readyDeadline = (Get-Date).AddSeconds(30)
$ready = $false
while ((Get-Date) -lt $readyDeadline) {
    if (Test-Path $rxLog) {
        $rxText = Get-Content $rxLog -Raw -ErrorAction SilentlyContinue
        if ($rxText -and $rxText -match "__RSSI_SNIFF_READY__") {
            $ready = $true
            break
        }
        if ($rxText -and $rxText -match "RSSI_FAIL ") {
            Write-Error "RX sniffer reported RSSI_FAIL early. See $rxLog."
            try { $rxProc.Kill() } catch {}
            exit 2
        }
    }
    Start-Sleep -Milliseconds 250
}
if (-not $ready) {
    Write-Error "RX sniffer did not print __RSSI_SNIFF_READY__ within 30s. See $rxLog."
    try { $rxProc.Kill() } catch {}
    exit 3
}
Write-Host "RX sniffer READY. Keying TX burst (cycles=$TxCycles inter=${TxInterS}s)..."

$txArg = "-s $TxAdbSerial shell `"echo fio | sudo -S -p '' docker run --name tx_air_burst --rm --network=host --device=/dev/ttymxc3 -v /tmp/lifetrac_strict:/work -w /work -e PYTHONPATH=/work:/work/paho -e LIFETRAC_SKIP_RESET_REQ=1 -e LIFETRAC_REG_PROFILE=$RegProfile --entrypoint timeout hub.foundries.io/arduino/arduino-ootb-python-devel:738bc44 $txContainerSec python3 -u /work/method_h_stage2_tx_probe_v2.py --probe tx_burst --tx-count $TxCycles --inter-cycle-s $TxInterS`""
$txProc = Start-Process -FilePath "adb" -ArgumentList $txArg -RedirectStandardOutput $txLog -RedirectStandardError $txErr -PassThru -NoNewWindow

$wait = (Get-Date).AddSeconds($rxContainerSec + 10)
while (((Get-Date) -lt $wait) -and (-not $rxProc.HasExited -or -not $txProc.HasExited)) {
    Start-Sleep -Seconds 1
}
if (-not $rxProc.HasExited) { try { $rxProc.Kill() } catch {} }
if (-not $txProc.HasExited) { try { $txProc.Kill() } catch {} }

Write-Host "`n=== RX RSSI SNIFFER LOG ($rxLog) ===" -ForegroundColor Green
Get-Content $rxLog -Tail 200
Write-Host "====================================`n"

Write-Host "`n=== TX BURST LOG ($txLog) ===" -ForegroundColor Green
Get-Content $txLog -Tail 80
Write-Host "==============================`n"

# Parse + verdict.
$rxText = Get-Content $rxLog -Raw -ErrorAction SilentlyContinue
if (-not $rxText) {
    Write-Error "RX log is empty. See $rxErr."
    exit 4
}
$summaryLine = ($rxText -split "`n") | Where-Object { $_ -match "^RSSI_SUMMARY " } | Select-Object -Last 1
if (-not $summaryLine) {
    Write-Error "No RSSI_SUMMARY emitted by sniffer."
    exit 5
}
$json = $summaryLine -replace "^RSSI_SUMMARY ", ""
$summary = $json | ConvertFrom-Json

Write-Host "=== VERDICT ===" -ForegroundColor Cyan
Write-Host ("rssi_min_dbm    = {0} dBm" -f $summary.rssi_min_dbm)
Write-Host ("rssi_median_dbm = {0} dBm" -f $summary.rssi_median_dbm)
Write-Host ("rssi_p90_dbm    = {0} dBm" -f $summary.rssi_p90_dbm)
Write-Host ("rssi_max_dbm    = {0} dBm" -f $summary.rssi_max_dbm)
Write-Host ("pkt_rssi_seen   = {0}, pkt_rssi_max_dbm = {1}" -f $summary.pkt_rssi_seen_count, $summary.pkt_rssi_max_dbm)
Write-Host ("irq_events      = {0}, irq_flags_or = {1}" -f $summary.irq_events, $summary.irq_flags_or_hex)
Write-Host ("opmodes_seen    = {0}" -f ($summary.opmodes_seen_hex -join ","))

if ($summary.rssi_max_dbm -ge -100) {
    Write-Host "RESULT: RF coupling DETECTED (max RSSI $($summary.rssi_max_dbm) dBm >= -100)." -ForegroundColor Green
    if ($summary.irq_events -gt 0) {
        Write-Host "       IRQ flags fired; demod is running. Investigate RX_FRAME_URC plumbing." -ForegroundColor Yellow
    } else {
        Write-Host "       But no IRQs fired -> demod not locking. Investigate SF/BW/preamble/CRC mismatch." -ForegroundColor Yellow
    }
    exit 0
} else {
    Write-Host "RESULT: NO RF coupling detected (max RSSI $($summary.rssi_max_dbm) dBm). RF/antenna or TX-power problem." -ForegroundColor Red
    exit 1
}
