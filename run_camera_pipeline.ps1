#requires -Version 5.1
<#
.SYNOPSIS
    Live-camera end-to-end pipeline runner for LifeTrac v25.
.DESCRIPTION
    Builds on run_concurrent_smoke.ps1 but replaces the synthetic frame
    publisher with `publish_camera_frames.py` running INSIDE a docker
    container on the TX board, with the Kurokesu C2 (/dev/video1) and a
    static aarch64 `ffmpeg` binary mapped in.

    Resulting chain:

        /dev/video1 (TX board)
          -> publish_camera_frames container
            -> MQTT lifetrac/v25/cmd/image_frame  (host broker)
              -> image_tx_daemon container (TX board)
                -> LoRa air (SF7/BW250/CR4-5, 915 MHz)
                  -> image_rx_daemon container (RX board)
                    -> MQTT lifetrac/v25/video/tile_delta
                      -> uvicorn web_ui (host:8080)
                        -> /ws/state WebSocket -> browser canvas.

    Prereqs (NOT auto-started by this script — same as smoke):
      * amqtt broker on host:1883     (python run_amqtt_broker.py)
      * uvicorn web_ui on host:8080   (cd base_station; uvicorn web_ui:app ...)
.PARAMETER DurationS
    How long to run the camera publisher (seconds). The daemons get
    DurationS + 5s so the publisher exits first.
.PARAMETER VideoDevice
    v4l2 device path on TX board. Defaults to /dev/video1 (Kurokesu C2).
#>
param(
    [string]$TxAdbSerial = "2E2C1209DABC240B",
    [string]$RxAdbSerial = "2D0A1209DABC240B",
    [string]$HostIp       = "192.168.1.79",
    [int]$DurationS       = 60,
    [string]$VideoDevice  = "/dev/video1",
    [string]$V4l2InputSize = "1920x1080",
    [int]$V4l2InputFps    = 30,
    [int]$PublishFps      = 2,
    [int]$FragmentBudget  = 250,
    [double]$KeyframePeriodS = 10.0,
    [ValidateSet('A','B','C')]
    [string]$ImageMethod  = 'C'
)

Set-StrictMode -Version Latest
$ErrorActionPreference = "Stop"

Write-Host "=== LIVE CAMERA PIPELINE: $DurationS s ===" -ForegroundColor Cyan
Write-Host "TX Serial:  $TxAdbSerial    Camera: $VideoDevice"
Write-Host "RX Serial:  $RxAdbSerial"
Write-Host "MQTT Host:  $HostIp"
Write-Host "Capture:    $V4l2InputSize @ $V4l2InputFps fps   Publish: $PublishFps fps"
Write-Host "Image method: $ImageMethod   Fragment budget: $FragmentBudget B   Keyframe: $KeyframePeriodS s"

$repoRoot    = (Resolve-Path "$PSScriptRoot").Path
$baseStation = Join-Path $repoRoot "LifeTrac-v25\DESIGN-CONTROLLER\base_station"
$tractorX8   = Join-Path $repoRoot "LifeTrac-v25\DESIGN-CONTROLLER\firmware\tractor_x8"
$helperDir   = Join-Path $repoRoot "LifeTrac-v25\DESIGN-CONTROLLER\firmware\x8_lora_bootloader_helper"
$ffmpegLocal = Join-Path $repoRoot "ffmpeg-7.0.2-arm64-static\ffmpeg"
$publisherLocal = Join-Path $repoRoot "publish_camera_frames.py"

if (-not (Test-Path $ffmpegLocal))    { throw "ffmpeg static binary not found at $ffmpegLocal" }
if (-not (Test-Path $publisherLocal)) { throw "publish_camera_frames.py not found at $publisherLocal" }

# Clean up any lingering containers (sudo password is inside this script
# file, matching the pattern used by run_concurrent_smoke.ps1 — never
# echoed back to the chat).
Write-Host "Cleaning up lingering containers..."
$ErrorActionPreference = "Continue"
& adb -s $TxAdbSerial shell "echo fio | sudo -S -p '' docker rm -f tx_smoke camera_pub 2>/dev/null" 2>&1 | Out-Null
& adb -s $RxAdbSerial shell "echo fio | sudo -S -p '' docker rm -f rx_smoke 2>/dev/null"           2>&1 | Out-Null

# L072 soft reset via gpio163 NRST (matches smoke).
Write-Host "Pulsing gpio163 (L072 NRST) on both boards..."
$nrstCmd = "echo fio | sudo -S -p '' sh -c '[ -d /sys/class/gpio/gpio163 ] || echo 163 > /sys/class/gpio/export; echo out > /sys/class/gpio/gpio163/direction; echo 1 > /sys/class/gpio/gpio163/value; sleep 0.02; echo 0 > /sys/class/gpio/gpio163/value; sleep 0.10; echo 1 > /sys/class/gpio/gpio163/value; echo NRST_RELEASED_AT=`$(date +%s.%N)'"
& adb -s $TxAdbSerial shell $nrstCmd 2>&1 | Out-Null
& adb -s $RxAdbSerial shell $nrstCmd 2>&1 | Out-Null
Start-Sleep -Milliseconds 1500

# Deploy daemons + camera publisher + ffmpeg + camera_service to BOTH boards
# (RX only needs the daemons; pushing camera bits to RX is harmless and
# keeps the deploy step uniform).
Write-Host "Deploying daemons + camera publisher + ffmpeg..."
foreach ($s in @($TxAdbSerial, $RxAdbSerial)) {
    cmd /c "adb -s $s shell `"echo fio | sudo -S -p '' mkdir -p /tmp/lifetrac_strict ; echo fio | sudo -S -p '' chmod 0777 /tmp/lifetrac_strict`"" 2>&1 | Out-Null
    cmd /c "adb -s $s push `"$(Join-Path $helperDir 'method_h_stage2_tx_probe_v2.py')`" /tmp/lifetrac_strict/" 2>&1 | Out-Null
    cmd /c "adb -s $s push `"$(Join-Path $baseStation 'lora_proto.py')`" /tmp/lifetrac_strict/" 2>&1 | Out-Null
    cmd /c "adb -s $s push `"$(Join-Path $baseStation 'image_rx_daemon.py')`" /tmp/lifetrac_strict/" 2>&1 | Out-Null
    cmd /c "adb -s $s push `"$(Join-Path $tractorX8   'image_tx_daemon.py')`" /tmp/lifetrac_strict/" 2>&1 | Out-Null
    cmd /c "adb -s $s push `"$(Join-Path $tractorX8   'camera_service.py')`" /tmp/lifetrac_strict/" 2>&1 | Out-Null
    cmd /c "adb -s $s push `"$publisherLocal`"                              /tmp/lifetrac_strict/" 2>&1 | Out-Null
    cmd /c "adb -s $s push `"$(Join-Path $baseStation 'image_pipeline')`"   /tmp/lifetrac_strict/" 2>&1 | Out-Null
    cmd /c "adb -s $s push `"$ffmpegLocal`" /tmp/lifetrac_strict/ffmpeg"                              2>&1 | Out-Null
    $pahoLocal = Join-Path $repoRoot "_paho_pull\paho"
    if (Test-Path $pahoLocal) {
        cmd /c "adb -s $s push `"$pahoLocal`" /tmp/lifetrac_strict/" 2>&1 | Out-Null
    }
}
# Ensure ffmpeg is executable on both boards.
& adb -s $TxAdbSerial shell "chmod +x /tmp/lifetrac_strict/ffmpeg" 2>&1 | Out-Null
$ErrorActionPreference = "Stop"

$rxLog  = Join-Path $repoRoot "camera_rx_daemon.log"
$txLog  = Join-Path $repoRoot "camera_tx_daemon.log"
$camLog = Join-Path $repoRoot "camera_publisher.log"
"" | Out-File -FilePath $rxLog  -Encoding ascii
"" | Out-File -FilePath $txLog  -Encoding ascii
"" | Out-File -FilePath $camLog -Encoding ascii

$daemonTimeout = $DurationS + 10
$camTimeout    = $DurationS + 3

# Launch RX Daemon
Write-Host "Launching RX Daemon..."
$rxArg = "-s $RxAdbSerial shell `"echo fio | sudo -S -p '' docker run --name rx_smoke --rm --network=host --device=/dev/ttymxc3 -v /tmp/lifetrac_strict:/work -w /work -e PYTHONPATH=/work:/work/paho -e LIFETRAC_MQTT_HOST=$HostIp -e LIFETRAC_SKIP_RESET_REQ=1 -e LIFETRAC_REG_PROFILE=0 --entrypoint timeout hub.foundries.io/arduino/arduino-ootb-python-devel:738bc44 $daemonTimeout python3 -u /work/image_rx_daemon.py --log-level INFO`""
$rxProc = Start-Process -FilePath "adb" -ArgumentList $rxArg -RedirectStandardOutput $rxLog -RedirectStandardError (Join-Path $repoRoot "camera_rx_stderr.log") -PassThru -NoNewWindow
Start-Sleep -Seconds 3

# Launch TX Daemon
Write-Host "Launching TX Daemon..."
$txArg = "-s $TxAdbSerial shell `"echo fio | sudo -S -p '' docker run --name tx_smoke --rm --network=host --device=/dev/ttymxc3 -v /tmp/lifetrac_strict:/work -w /work -e PYTHONPATH=/work:/work/paho -e LIFETRAC_MQTT_HOST=$HostIp -e LIFETRAC_SKIP_RESET_REQ=1 -e LIFETRAC_REG_PROFILE=0 --entrypoint timeout hub.foundries.io/arduino/arduino-ootb-python-devel:738bc44 $daemonTimeout python3 -u /work/image_tx_daemon.py --log-level INFO`""
$txProc = Start-Process -FilePath "adb" -ArgumentList $txArg -RedirectStandardOutput $txLog -RedirectStandardError (Join-Path $repoRoot "camera_tx_stderr.log") -PassThru -NoNewWindow
Start-Sleep -Seconds 3

# Launch camera publisher container on TX board, with the camera + ffmpeg
# mounted in.
Write-Host "Launching camera publisher container on TX board (device=$VideoDevice)..."
$envFlags = @(
    "-e PYTHONPATH=/work:/work/paho:/work/site",
    "-e LIFETRAC_MQTT_HOST=$HostIp",
    "-e LIFETRAC_CAMERA_DEVICE=$VideoDevice",
    "-e LIFETRAC_V4L2_INPUT_FORMAT=mjpeg",
    "-e LIFETRAC_V4L2_INPUT_SIZE=$V4l2InputSize",
    "-e LIFETRAC_V4L2_INPUT_FPS=$V4l2InputFps",
    "-e LIFETRAC_CAMERA_FPS=$PublishFps",
    "-e LIFETRAC_DURATION_S=$DurationS",
    "-e LIFETRAC_FFMPEG_PATH=/work/ffmpeg",
    "-e LIFETRAC_FRAGMENT_BUDGET=$FragmentBudget",
    "-e LIFETRAC_KEYFRAME_PERIOD_S=$KeyframePeriodS",
    "-e LIFETRAC_IMAGE_METHOD=$ImageMethod",
    "-e LIFETRAC_CAMERA_DESHAKE=0"
) -join " "

$camArg = "-s $TxAdbSerial shell `"echo fio | sudo -S -p '' docker run --name camera_pub --rm --network=host --device=$VideoDevice -v /tmp/lifetrac_strict:/work -w /work $envFlags --entrypoint timeout hub.foundries.io/arduino/arduino-ootb-python-devel:738bc44 $camTimeout python3 -u /work/publish_camera_frames.py`""
$camProc = Start-Process -FilePath "adb" -ArgumentList $camArg -RedirectStandardOutput $camLog -RedirectStandardError (Join-Path $repoRoot "camera_publisher_stderr.log") -PassThru -NoNewWindow

Write-Host "Pipeline running. Waiting up to $($daemonTimeout + 5)s ..."
$deadline = (Get-Date).AddSeconds($daemonTimeout + 5)
while (((Get-Date) -lt $deadline) -and `
       (-not $rxProc.HasExited -or -not $txProc.HasExited -or -not $camProc.HasExited)) {
    Start-Sleep -Seconds 1
}

foreach ($pair in @(@($rxProc, "RX"), @($txProc, "TX"), @($camProc, "CAM"))) {
    $p, $name = $pair
    if (-not $p.HasExited) {
        Write-Host "Stopping $name process..."
        try { $p.Kill() } catch {}
    }
}

Write-Host "`n=== CAMERA PUBLISHER LOG (tail) ===" -ForegroundColor Magenta
if (Test-Path $camLog) { Get-Content $camLog -Tail 30 }
Write-Host "`n=== TX DAEMON LOG (tail) ===" -ForegroundColor Green
if (Test-Path $txLog)  { Get-Content $txLog  -Tail 20 }
Write-Host "`n=== RX DAEMON LOG (tail) ===" -ForegroundColor Green
if (Test-Path $rxLog)  { Get-Content $rxLog  -Tail 25 }

# Verdict: real frames over LoRa is the goal.
$rxContent = if (Test-Path $rxLog) { Get-Content $rxLog -Raw } else { "" }
$txContent = if (Test-Path $txLog) { Get-Content $txLog -Raw } else { "" }
$camContent = if (Test-Path $camLog) { Get-Content $camLog -Raw } else { "" }

$camPublished = ($camContent | Select-String -Pattern 'published\s+\d+\s+B' -AllMatches).Matches.Count
$txFramesIn   = 0
if ($txContent -match 'frames_in=(\d+)') {
    $txFramesIn = [int]($txContent | Select-String -Pattern 'frames_in=(\d+)' -AllMatches).Matches[-1].Groups[1].Value
}
$rxFrames = 0
if ($rxContent -match 'rx_frames=(\d+)') {
    $rxFrames = [int]($rxContent | Select-String -Pattern 'rx_frames=(\d+)' -AllMatches).Matches[-1].Groups[1].Value
}
$framesPublished = 0
if ($rxContent -match 'frames_published=(\d+)') {
    $framesPublished = [int]($rxContent | Select-String -Pattern 'frames_published=(\d+)' -AllMatches).Matches[-1].Groups[1].Value
}

Write-Host "`n=== VERDICT ===" -ForegroundColor Cyan
Write-Host "camera_publisher published frames : $camPublished"
Write-Host "image_tx_daemon frames_in        : $txFramesIn"
Write-Host "image_rx_daemon rx_frames        : $rxFrames"
Write-Host "image_rx_daemon frames_published : $framesPublished"

if ($camPublished -lt 1) {
    Write-Error "FAILURE: camera publisher produced no frames (check camera_publisher.log / camera_publisher_stderr.log)."
    exit 2
}
if ($txFramesIn -lt 1) {
    Write-Error "FAILURE: image_tx_daemon never received MQTT frames."
    exit 3
}
if ($rxFrames -lt 1) {
    Write-Error "FAILURE: image_rx_daemon never demodulated a LoRa frame."
    exit 4
}
if ($framesPublished -lt 1) {
    Write-Error "FAILURE: reassembler produced no tile_delta frames for the UI."
    exit 5
}
Write-Host "SUCCESS: live camera frames flowed end-to-end over LoRa to the base station UI." -ForegroundColor Green
exit 0
