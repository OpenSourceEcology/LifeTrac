[CmdletBinding()]
param(
    [string]$AdbSerial = "2E2C1209DABC240B",
    [string]$ContainerName = "lifetrac-camera-x8",
    [string]$Image = "hub.foundries.io/arduino/arduino-ootb-python-devel:738bc44",
    [string]$CameraDevice = "/dev/video1",
    [string]$V4l2InputFormat = "mjpeg",
    [string]$M7Uart = "/dev/ttymxc1",
    [string]$FfmpegPath = "/tmp/ffmpeg",
    [int]$CameraFps = 2,
    [switch]$NoDeshake,
    [switch]$UseDeshake,
    [switch]$FallbackHostMqtt
)

$ErrorActionPreference = "Stop"

function Resolve-Adb {
    $cmd = Get-Command adb -ErrorAction SilentlyContinue
    if ($cmd) {
        return $cmd.Source
    }
    $fallback = "C:\Users\dorkm\AppData\Local\Microsoft\WinGet\Packages\Google.PlatformTools_Microsoft.Winget.Source_8wekyb3d8bbwe\platform-tools\adb.exe"
    if (Test-Path $fallback) {
        return $fallback
    }
    throw "adb executable not found in PATH or fallback location"
}

function Invoke-Adb {
    param([Parameter(ValueFromRemainingArguments = $true)] [string[]]$Args)
    & $script:AdbExe -s $AdbSerial @Args
    if ($LASTEXITCODE -ne 0) {
        throw "adb $($Args -join ' ') failed (rc=$LASTEXITCODE)"
    }
}

$script:AdbExe = Resolve-Adb

if ($NoDeshake -and $UseDeshake) {
    throw "Use either -NoDeshake or -UseDeshake, not both"
}

# Keep deshake OFF by default because the bench static ffmpeg build has
# repeatedly rejected the deshake filter chain and starved frame production.
$deshakeValue = if ($UseDeshake) { "1" } else { "0" }

$cameraDebugMqtt = "0"
$mqttHost = "localhost"
if ($FallbackHostMqtt) {
    # Fallback mode: mirror camera frames directly to the host broker over
    # adb reverse so the base station can ingest /cmd/image_frame.
    $cameraDebugMqtt = "1"
    $mqttHost = "127.0.0.1"
    Invoke-Adb reverse "tcp:1883" "tcp:1883"
}

Write-Host "[1/4] Verifying camera payload exists on target ..."
Invoke-Adb shell "test -f /tmp/lifetrac_camera/camera_service.py"
Invoke-Adb shell "test -d /tmp/lifetrac_camera/image_pipeline"

Write-Host "[2/4] Stopping any previous camera container ..."
Invoke-Adb shell "echo fio | sudo -S -p '' docker rm -f $ContainerName >/dev/null 2>&1 || true"
Invoke-Adb shell "echo fio | sudo -S -p '' docker rm -f lifetrac-camera-x8-test >/dev/null 2>&1 || true"

Write-Host "[3/4] Starting $ContainerName on $AdbSerial ..."
$runCmd = @(
    "echo fio | sudo -S -p '' docker run -d",
    "--name $ContainerName",
    "--restart unless-stopped",
    "--network host",
    "--entrypoint /bin/sh",
    "--device ${CameraDevice}:${CameraDevice}",
    "--device ${M7Uart}:${M7Uart}",
    "-v /tmp/lifetrac_camera:/opt/lifetrac_camera",
    "-v /tmp/ffmpeg:/tmp/ffmpeg:ro",
    "-e PYTHONPATH=/opt/lifetrac_camera",
    "-e LIFETRAC_CAMERA_SOURCE=v4l2",
    "-e LIFETRAC_CAMERA_DEVICE=$CameraDevice",
    "-e LIFETRAC_V4L2_INPUT_FORMAT=$V4l2InputFormat",
    "-e LIFETRAC_M7_UART=$M7Uart",
    "-e LIFETRAC_FFMPEG_PATH=$FfmpegPath",
    "-e LIFETRAC_CAMERA_FPS=$CameraFps",
    "-e LIFETRAC_CAMERA_DESHAKE=$deshakeValue",
    "-e LIFETRAC_CAMERA_DEBUG_MQTT=$cameraDebugMqtt",
    "-e LIFETRAC_MQTT_HOST=$mqttHost",
    "$Image",
    "-c 'python3 -m pip install --no-cache-dir pillow paho-mqtt pyserial numpy >/tmp/lifetrac_pip.log 2>&1 && exec python3 /opt/lifetrac_camera/camera_service.py'"
) -join " "
Invoke-Adb shell $runCmd

Write-Host "[4/4] Camera container status and recent logs ..."
Invoke-Adb shell "echo fio | sudo -S -p '' docker ps --filter name=$ContainerName --format '{{.Names}} {{.Status}}'"
Invoke-Adb shell "echo fio | sudo -S -p '' docker logs --tail 80 $ContainerName"
Invoke-Adb shell "echo fio | sudo -S -p '' docker exec $ContainerName sh -lc 'tail -n 40 /tmp/lifetrac_pip.log 2>/dev/null || true; pgrep -af camera_service.py || true'"

Write-Host "OK: containerized camera service launch attempted."