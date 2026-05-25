[CmdletBinding()]
param(
    [string]$AdbSerial = "2E2C1209DABC240B",
    [string]$ContainerName = "lifetrac-camera-x8"
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

Write-Host "[1/3] Container status ..."
Invoke-Adb shell "echo fio | sudo -S -p '' docker ps -a --filter name=$ContainerName --format '{{.Names}} {{.Status}}'"

Write-Host "[2/3] Recent container logs ..."
Invoke-Adb shell "echo fio | sudo -S -p '' docker logs --tail 120 $ContainerName"

Write-Host "[3/3] Pip log + camera process probe ..."
Invoke-Adb shell "echo fio | sudo -S -p '' docker exec $ContainerName sh -lc 'tail -n 80 /tmp/lifetrac_pip.log 2>/dev/null || true; pgrep -af camera_service.py || true; pgrep -af ffmpeg || true'"

Write-Host "OK: camera container status checked."
