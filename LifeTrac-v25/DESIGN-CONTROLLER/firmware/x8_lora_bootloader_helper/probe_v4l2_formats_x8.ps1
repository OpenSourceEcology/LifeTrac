[CmdletBinding()]
param(
    [string]$AdbSerial = "2E2C1209DABC240B",
    [string]$FfmpegPath = "/tmp/ffmpeg"
)

$ErrorActionPreference = "Stop"

function Resolve-Adb {
    $cmd = Get-Command adb -ErrorAction SilentlyContinue
    if ($cmd) { return $cmd.Source }
    $fallback = "C:\Users\dorkm\AppData\Local\Microsoft\WinGet\Packages\Google.PlatformTools_Microsoft.Winget.Source_8wekyb3d8bbwe\platform-tools\adb.exe"
    if (Test-Path $fallback) { return $fallback }
    throw "adb executable not found"
}

function Invoke-Adb {
    param([Parameter(ValueFromRemainingArguments = $true)] [string[]]$Args)
    & $script:AdbExe -s $AdbSerial @Args
    if ($LASTEXITCODE -ne 0) {
        throw "adb $($Args -join ' ') failed (rc=$LASTEXITCODE)"
    }
}

$script:AdbExe = Resolve-Adb

Write-Host "[1/3] Stop camera container so nodes are not busy ..."
Invoke-Adb shell "echo fio | sudo -S -p '' docker rm -f lifetrac-camera-x8 >/dev/null 2>&1 || true"

Write-Host "[2/3] Camera node names ..."
for ($i = 0; $i -le 4; $i++) {
    $name = (& $script:AdbExe -s $AdbSerial shell "cat /sys/class/video4linux/video$i/name" 2>$null).Trim()
    Write-Host "video$i : $name"
}

Write-Host "[3/3] ffmpeg reported formats per node ..."
for ($i = 0; $i -le 4; $i++) {
    Write-Host "----- /dev/video$i -----"
    $cmd = "echo fio | sudo -S -p '' $FfmpegPath -hide_banner -f v4l2 -list_formats all -i /dev/video$i"
    & $script:AdbExe -s $AdbSerial shell $cmd 2>&1 | Select-String -Pattern "Video input device|Raw|Compressed|mjpeg|yuyv|uyvy|nv12|h264|Error opening input|Device or resource busy"
}

Write-Host "OK: v4l2 probe complete."
