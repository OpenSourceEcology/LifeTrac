[CmdletBinding()]
param(
    [string]$AdbSerial = "2E2C1209DABC240B",
    [string]$Image = "hub.foundries.io/arduino/arduino-ootb-python-devel:738bc44",
    [string[]]$CameraDevices = @("/dev/video1", "/dev/video3")
)

$ErrorActionPreference = "Stop"

function Resolve-Adb {
    $cmd = Get-Command adb -ErrorAction SilentlyContinue
    if ($cmd) { return $cmd.Source }
    $fallback = "C:\Users\dorkm\AppData\Local\Microsoft\WinGet\Packages\Google.PlatformTools_Microsoft.Winget.Source_8wekyb3d8bbwe\platform-tools\adb.exe"
    if (Test-Path $fallback) { return $fallback }
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
$scriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$innerScript = Join-Path $scriptDir "uvc_probe_inside_container.sh"
if (-not (Test-Path $innerScript)) {
  throw "missing helper script: $innerScript"
}

Write-Host "[1/3] Stop camera containers to free video nodes ..."
Invoke-Adb shell "echo fio | sudo -S -p '' docker rm -f lifetrac-camera-x8 >/dev/null 2>&1 || true"
Invoke-Adb shell "echo fio | sudo -S -p '' docker rm -f lifetrac-camera-x8-test >/dev/null 2>&1 || true"
Invoke-Adb push $innerScript "/tmp/uvc_probe_inside_container.sh" | Out-Null
Invoke-Adb shell "chmod +x /tmp/uvc_probe_inside_container.sh" | Out-Null

Write-Host "[2/3] Probe controls and attempt test-pattern disable ..."
foreach ($dev in $CameraDevices) {
    Write-Host "----- $dev -----"

    $probeCmd = @(
        "echo fio | sudo -S -p '' docker run --rm",
        "--entrypoint /bin/sh",
        "--device ${dev}:${dev}",
        "-v /tmp/uvc_probe_inside_container.sh:/tmp/uvc_probe_inside_container.sh:ro",
        "$Image",
        "-lc 'sh /tmp/uvc_probe_inside_container.sh $dev'"
    ) -join " "

    try {
      $out = & $script:AdbExe -s $AdbSerial shell $probeCmd 2>&1
      $out | Write-Host
      if ($LASTEXITCODE -ne 0) {
        throw "probe command failed (rc=$LASTEXITCODE)"
      }
    }
    catch {
        Write-Warning "Probe failed for $dev : $($_.Exception.Message)"
    }
}

Write-Host "[3/3] Probe complete. Relaunch camera container next."
