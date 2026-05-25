#requires -Version 5.1
<#
.SYNOPSIS
    Concurrent 30s smoke test for LifeTrac Image TX/RX Daemons.
.DESCRIPTION
    Launches the RX Daemon on Board 2D0A and the TX Daemon on Board 2E2C
    concurrently for 30 seconds using the proper DTS 500 kHz profile config, and verifies
    non-zero LoRa frame demodulation.
#>
param(
    [string]$TxAdbSerial = "2E2C1209DABC240B",
    [string]$RxAdbSerial = "2D0A1209DABC240B",
    [string]$HostIp       = "192.168.1.79",
    [int]$DurationS       = 30
)

Set-StrictMode -Version Latest
$ErrorActionPreference = "Stop"

Write-Host "=== STARTING CONCURRENT 30-SECOND SMOKE TEST ===" -ForegroundColor Cyan
Write-Host "TX Serial: $TxAdbSerial"
Write-Host "RX Serial: $RxAdbSerial"
Write-Host "MQTT Host: $HostIp"

# Clean up any lingering container remnants
Write-Host "Cleaning up lingering containers..."
& adb -s $TxAdbSerial shell "echo fio | sudo -S docker rm -f tx_smoke" 2>&1
& adb -s $RxAdbSerial shell "echo fio | sudo -S docker rm -f rx_smoke" 2>&1

# L072-only soft reset via gpio163 NRST pulse (SOFT_RESET_INDEX 3.1).
# Replaces the prior `dd` UART drain — that drain only flushes the host
# RX buffer; it cannot recover an L072 stuck mid-COBS-transmit from a
# prior aborted dwell. The gpio163 pulse drives the LoRa NRST line via
# the x8h7 bridge (H7 PF4 -> gpio163), giving a clean boot state for
# both peers without touching the i.MX host. See AI NOTES
# 2026-05-25_Concurrent_Smoke_RX_UART_and_TX_DONE_TIMEOUT.md.
Write-Host "Pulsing gpio163 (L072 NRST) on both boards..."
$nrstCmd = "echo fio | sudo -S -p '' sh -c '[ -d /sys/class/gpio/gpio163 ] || echo 163 > /sys/class/gpio/export; echo out > /sys/class/gpio/gpio163/direction; echo 1 > /sys/class/gpio/gpio163/value; sleep 0.02; echo 0 > /sys/class/gpio/gpio163/value; sleep 0.10; echo 1 > /sys/class/gpio/gpio163/value; echo NRST_RELEASED_AT=`$(date +%s.%N)'"
& adb -s $TxAdbSerial shell $nrstCmd
& adb -s $RxAdbSerial shell $nrstCmd
# Allow L072 firmware to finish boot (BOOT_URC ~200ms after NRST release).
Start-Sleep -Milliseconds 1500

# 2026-05-25: previous version assumed image_rx_daemon.py and image_tx_daemon.py
# were already deployed to /tmp/lifetrac_strict on each board. In practice
# image_tx_daemon.py was MISSING on the TX board, so the smoke ran with
# zero TX traffic and the "rx_frames=0" verdict was incorrectly blamed on
# the firmware/RF chain (see AI NOTES 2026-05-25_Smoke_RxFrames_Was_Missing_TX_Daemon.md).
# Push them (plus their dependencies) every run to make this hermetic.
Write-Host "Deploying daemons + helpers to both boards..."
$repoRoot   = (Resolve-Path "$PSScriptRoot\..\..\..\..").Path
$baseStation = Join-Path $repoRoot "LifeTrac-v25\DESIGN-CONTROLLER\base_station"
$tractorX8   = Join-Path $repoRoot "LifeTrac-v25\DESIGN-CONTROLLER\firmware\tractor_x8"
$helperDir   = $PSScriptRoot
$prevEAP = $ErrorActionPreference
$ErrorActionPreference = "Continue"  # adb push reports success on stderr; don't treat as fatal
foreach ($s in @($TxAdbSerial, $RxAdbSerial)) {
    cmd /c "adb -s $s shell `"echo fio | sudo -S -p '' mkdir -p /tmp/lifetrac_strict ; echo fio | sudo -S -p '' chmod 0777 /tmp/lifetrac_strict`"" 2>&1 | Out-Null
    cmd /c "adb -s $s push `"$(Join-Path $helperDir 'method_h_stage2_tx_probe_v2.py')`" /tmp/lifetrac_strict/" 2>&1 | Out-Null
    cmd /c "adb -s $s push `"$(Join-Path $baseStation 'lora_proto.py')`" /tmp/lifetrac_strict/" 2>&1 | Out-Null
    cmd /c "adb -s $s push `"$(Join-Path $baseStation 'image_rx_daemon.py')`" /tmp/lifetrac_strict/" 2>&1 | Out-Null
    cmd /c "adb -s $s push `"$(Join-Path $tractorX8 'image_tx_daemon.py')`" /tmp/lifetrac_strict/" 2>&1 | Out-Null
    cmd /c "adb -s $s push `"$(Join-Path $baseStation 'image_pipeline')`" /tmp/lifetrac_strict/" 2>&1 | Out-Null
    $pahoLocal = Join-Path $repoRoot "_paho_pull\paho"
    if (Test-Path $pahoLocal) {
        cmd /c "adb -s $s push `"$pahoLocal`" /tmp/lifetrac_strict/" 2>&1 | Out-Null
    } else {
        Write-Warning "Local paho dir not found at $pahoLocal - TX/RX daemons may fail at import paho"
    }
}
$ErrorActionPreference = $prevEAP
Write-Host "Daemon deployment complete."

$rxLog = "rx_daemon_smoke.log"
$txLog = "tx_daemon_smoke.log"
"" | Out-File -FilePath $rxLog -Encoding ascii
"" | Out-File -FilePath $txLog -Encoding ascii

# Launch RX Daemon in the background
Write-Host "Launching RX Daemon in the background..."
$rxArg = "-s $RxAdbSerial shell `"echo fio | sudo -S -p '' docker run --name rx_smoke --rm --network=host --device=/dev/ttymxc3 -v /tmp/lifetrac_strict:/work -w /work -e PYTHONPATH=/work:/work/paho -e LIFETRAC_MQTT_HOST=$HostIp -e LIFETRAC_SKIP_RESET_REQ=1 -e LIFETRAC_REG_PROFILE=0 --entrypoint timeout hub.foundries.io/arduino/arduino-ootb-python-devel:738bc44 35 python3 -u /work/image_rx_daemon.py --log-level INFO`""
$rxProc = Start-Process -FilePath "adb" -ArgumentList $rxArg -RedirectStandardOutput $rxLog -RedirectStandardError rx_stderr.log -PassThru -NoNewWindow

Start-Sleep -Seconds 3 # Let RX start up first and get into RXCONT

# Launch TX Daemon
Write-Host "Launching TX Daemon..."
$txArg = "-s $TxAdbSerial shell `"echo fio | sudo -S -p '' docker run --name tx_smoke --rm --network=host --device=/dev/ttymxc3 -v /tmp/lifetrac_strict:/work -w /work -e PYTHONPATH=/work:/work/paho -e LIFETRAC_MQTT_HOST=$HostIp -e LIFETRAC_SKIP_RESET_REQ=1 -e LIFETRAC_REG_PROFILE=0 --entrypoint timeout hub.foundries.io/arduino/arduino-ootb-python-devel:738bc44 30 python3 -u /work/image_tx_daemon.py --log-level INFO`""
$txProc = Start-Process -FilePath "adb" -ArgumentList $txArg -RedirectStandardOutput $txLog -RedirectStandardError tx_stderr.log -PassThru -NoNewWindow

Start-Sleep -Seconds 3 # Let TX start and subscribe

Write-Host "Starting synthetic camera publisher on Host..."
$env:LIFETRAC_MQTT_HOST = $HostIp
# 2026-05-25: must use absolute path to publisher script. When the smoke is
# launched from inside the helper dir, a relative path fails with ENOENT and
# the publisher silently dies, producing frames_in=0 on the TX daemon.
$pubScript = Join-Path $repoRoot "publish_synthetic_frames.py"
$pubLog    = Join-Path $repoRoot "publisher.log"
$pubErrLog = Join-Path $repoRoot "publisher_err.log"
$pubProc = Start-Process -FilePath "C:\Users\dorkm\AppData\Local\Python\pythoncore-3.14-64\python.exe" -ArgumentList "`"$pubScript`"" -PassThru -NoNewWindow -RedirectStandardOutput $pubLog -RedirectStandardError $pubErrLog -WorkingDirectory $repoRoot

Write-Host "Both daemons running concurrently. Waiting for 35 seconds..."
$timeout = (Get-Date).AddSeconds(45)
while (((Get-Date) -lt $timeout) -and (-not $rxProc.HasExited -or -not $txProc.HasExited)) {
    Start-Sleep -Seconds 1
}

# Stop processes if still running
if (-not $rxProc.HasExited) {
    Write-Host "Stopping RX Daemon process..."
    try { $rxProc.Kill() } catch {}
}
if (-not $txProc.HasExited) {
    Write-Host "Stopping TX Daemon process..."
    try { $txProc.Kill() } catch {}
}
if ($pubProc -and -not $pubProc.HasExited) {
    Write-Host "Stopping Synthetic Publisher..."
    try { $pubProc.Kill() } catch {}
}

# Print the outputs/logs
Write-Host "`n=== RX DAEMON LOGS ===" -ForegroundColor Green
if (Test-Path $rxLog) { Get-Content $rxLog }
Write-Host "======================`n"

Write-Host "`n=== TX DAEMON LOGS ===" -ForegroundColor Green
if (Test-Path $txLog) { Get-Content $txLog }
Write-Host "======================`n"

# Verify frame reception
$rxContent = ""
if (Test-Path $rxLog) { $rxContent = Get-Content $rxLog -Raw }
if ($rxContent -match "rx_frames(?:_seen)?=([1-9]\d*)") {
    Write-Host "SUCCESS: Demodulated frame(s) seen by RX daemon! Match group: $($matches[1])" -ForegroundColor Green
    exit 0
} else {
    Write-Error "FAILURE: No frames demodulated or rx_frames_seen remains 0."
    exit 1
}
