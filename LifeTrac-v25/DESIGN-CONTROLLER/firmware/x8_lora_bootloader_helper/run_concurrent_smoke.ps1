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
$ErrorActionPreference = "Continue"

$pubProc = $null
$rxProc = $null
$txProc = $null

$env:PATH += ";C:\Users\dorkm\AppData\Local\Microsoft\WinGet\Packages\Google.PlatformTools_8wekyb3d8bbwe\platform-tools"

Write-Host "=== STARTING CONCURRENT 30-SECOND SMOKE TEST ===" -ForegroundColor Cyan
Write-Host "TX Serial: $TxAdbSerial"
Write-Host "RX Serial: $RxAdbSerial"
Write-Host "MQTT Host: $HostIp"

# Ensure host MQTT broker is running
$socket = New-Object System.Net.Sockets.TcpClient
$asyncResult = $socket.BeginConnect($HostIp, 1883, $null, $null)
$waitResult = $asyncResult.AsyncWaitHandle.WaitOne(1000, $false)
if (-not $waitResult -or -not $socket.Connected) {
    Write-Host "Host MQTT broker on $HostIp:1883 not responding — starting start_mqtt_broker.py..."
    $repoRoot = (Resolve-Path "$PSScriptRoot\..\..\..\..").Path
    Start-Process -FilePath "C:\Users\dorkm\AppData\Local\Python\pythoncore-3.14-64\python.exe" -ArgumentList "start_mqtt_broker.py" -WorkingDirectory $repoRoot -PassThru -NoNewWindow
    Start-Sleep -Seconds 2
} else {
    $socket.Close()
}

# Clean up any lingering container remnants
Write-Host "Cleaning up lingering containers..."
cmd /c "`"$adbExe`" -s $TxAdbSerial shell `"echo fio | sudo -S docker rm -f tx_smoke`"" 2>&1 | Out-Null
cmd /c "`"$adbExe`" -s $RxAdbSerial shell `"echo fio | sudo -S docker rm -f rx_smoke`"" 2>&1 | Out-Null

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
    cmd /c "`"$adbExe`" -s $s shell `"echo fio | sudo -S -p '' mkdir -p /tmp/lifetrac_strict ; echo fio | sudo -S -p '' chmod 0777 /tmp/lifetrac_strict`"" 2>&1 | Out-Null
    cmd /c "`"$adbExe`" -s $s push `"$(Join-Path $helperDir 'method_g_stage1_probe.py')`" /tmp/lifetrac_strict/" 2>&1 | Out-Null
    cmd /c "`"$adbExe`" -s $s push `"$(Join-Path $helperDir 'method_h_stage2_tx_probe_v2.py')`" /tmp/lifetrac_strict/" 2>&1 | Out-Null
    cmd /c "`"$adbExe`" -s $s push `"$(Join-Path $baseStation 'lora_proto.py')`" /tmp/lifetrac_strict/" 2>&1 | Out-Null
    cmd /c "`"$adbExe`" -s $s push `"$(Join-Path $baseStation 'image_rx_daemon.py')`" /tmp/lifetrac_strict/" 2>&1 | Out-Null
    cmd /c "`"$adbExe`" -s $s push `"$(Join-Path $tractorX8 'image_tx_daemon.py')`" /tmp/lifetrac_strict/" 2>&1 | Out-Null
    cmd /c "`"$adbExe`" -s $s push `"$(Join-Path $baseStation 'image_pipeline')`" /tmp/lifetrac_strict/" 2>&1 | Out-Null
    $pahoLocal = Join-Path $repoRoot "_paho_pull\paho"
    if (Test-Path $pahoLocal) {
        cmd /c "`"$adbExe`" -s $s push `"$pahoLocal`" /tmp/lifetrac_strict/" 2>&1 | Out-Null
    } else {
        Write-Warning "Local paho dir not found at $pahoLocal - TX/RX daemons may fail at import paho"
    }
}
$ErrorActionPreference = $prevEAP
Write-Host "Daemon deployment complete."

# L072-only soft reset via gpio163 NRST pulse (SOFT_RESET_INDEX 3.1).
# Run NRST pulse AFTER deploy so boot chatter doesn't fill the UART buffer during adb push.
Write-Host "Pulsing gpio163 (L072 NRST) on both boards..."
$nrstCmd = "echo fio | sudo -S -p '' sh -c '[ -d /sys/class/gpio/gpio163 ] || echo 163 > /sys/class/gpio/export; echo out > /sys/class/gpio/gpio163/direction; echo 1 > /sys/class/gpio/gpio163/value; sleep 0.02; echo 0 > /sys/class/gpio/gpio163/value; sleep 0.10; echo 1 > /sys/class/gpio/gpio163/value; echo NRST_RELEASED_AT=`$(date +%s.%N)'"
cmd /c "`"$adbExe`" -s $TxAdbSerial shell `"$nrstCmd`"" | Out-Null
cmd /c "`"$adbExe`" -s $RxAdbSerial shell `"$nrstCmd`"" | Out-Null
# Allow L072 firmware to finish boot (BOOT_URC ~200ms after NRST release).
Start-Sleep -Milliseconds 1500

$rxLog = Join-Path $repoRoot "rx_daemon_smoke.log"
$txLog = Join-Path $repoRoot "tx_daemon_smoke.log"
"" | Out-File -FilePath $rxLog -Encoding ascii
"" | Out-File -FilePath $txLog -Encoding ascii

$adbExe = "C:\Users\dorkm\AppData\Local\Microsoft\WinGet\Packages\Google.PlatformTools_8wekyb3d8bbwe\platform-tools\adb.exe"

# Launch RX Daemon in background container
Write-Host "Launching RX Daemon in background container..."
cmd /c "`"$adbExe`" -s $RxAdbSerial shell `"echo fio | sudo -S -p '' docker rm -f rx_smoke 2>/dev/null ; echo fio | sudo -S -p '' docker run -d --name rx_smoke --rm --network=host --device=/dev/ttymxc3 -v /tmp/lifetrac_strict:/work -w /work -e PYTHONPATH=/work:/work/paho -e LIFETRAC_MQTT_HOST=$HostIp -e LIFETRAC_SKIP_RESET_REQ=1 -e LIFETRAC_REG_PROFILE=0 lifetrac-v25:latest python3 -u /work/image_rx_daemon.py --log-level INFO`""

Start-Sleep -Seconds 3 # Let RX start up first and get into RXCONT

# Launch TX Daemon in background container
Write-Host "Launching TX Daemon in background container..."
cmd /c "`"$adbExe`" -s $TxAdbSerial shell `"echo fio | sudo -S -p '' docker rm -f tx_smoke 2>/dev/null ; echo fio | sudo -S -p '' docker run -d --name tx_smoke --rm --network=host --device=/dev/ttymxc3 -v /tmp/lifetrac_strict:/work -w /work -e PYTHONPATH=/work:/work/paho -e LIFETRAC_MQTT_HOST=$HostIp -e LIFETRAC_SKIP_RESET_REQ=1 -e LIFETRAC_REG_PROFILE=0 hub.foundries.io/arduino/arduino-ootb-python-devel:738bc44 python3 -u /work/image_tx_daemon.py --log-level INFO`""

Start-Sleep -Seconds 3 # Let TX start and subscribe

Write-Host "Starting synthetic camera publisher on Host..."
$env:LIFETRAC_MQTT_HOST = $HostIp
$pubScript = Join-Path $repoRoot "publish_synthetic_frames.py"
$pubLog    = Join-Path $repoRoot "publisher.log"
$pubErrLog = Join-Path $repoRoot "publisher_err.log"
$pubProc = Start-Process -FilePath "C:\Users\dorkm\AppData\Local\Python\pythoncore-3.14-64\python.exe" -ArgumentList "`"$pubScript`"" -PassThru -NoNewWindow -RedirectStandardOutput $pubLog -RedirectStandardError $pubErrLog -WorkingDirectory $repoRoot

Write-Host "Both daemons running concurrently. Streaming logs for $DurationS seconds..."
Start-Sleep -Seconds $DurationS

if ($pubProc -and -not $pubProc.HasExited) {
    Write-Host "Stopping Synthetic Publisher..."
    try { $pubProc.Kill() } catch {}
}

# Retrieve logs directly from Docker containers
$rxLogs = cmd /c "`"$adbExe`" -s $RxAdbSerial shell `"echo fio | sudo -S -p '' docker logs rx_smoke 2>&1`""
$txLogs = cmd /c "`"$adbExe`" -s $TxAdbSerial shell `"echo fio | sudo -S -p '' docker logs tx_smoke 2>&1`""

cmd /c "`"$adbExe`" -s $RxAdbSerial shell `"echo fio | sudo -S -p '' docker stop -t 1 rx_smoke 2>&1`"" | Out-Null
cmd /c "`"$adbExe`" -s $TxAdbSerial shell `"echo fio | sudo -S -p '' docker stop -t 1 tx_smoke 2>&1`"" | Out-Null

[System.IO.File]::WriteAllText($rxLog, $rxLogs)
[System.IO.File]::WriteAllText($txLog, $txLogs)

# Print the outputs/logs
Write-Host "`n=== RX DAEMON LOGS ===" -ForegroundColor Green
Write-Host $rxLogs
Write-Host "======================`n"

Write-Host "`n=== TX DAEMON LOGS ===" -ForegroundColor Green
Write-Host $txLogs
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
