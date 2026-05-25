[CmdletBinding()]
param(
    [string]$AdbSerial = "2E2C1209DABC240B",
    [string]$Image = "hub.foundries.io/arduino/arduino-ootb-python-devel:738bc44",
    [string[]]$CameraDevices = @("/dev/video1", "/dev/video3")
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

Write-Host "[1/3] Stop camera containers to free video nodes ..."
Invoke-Adb shell "echo fio | sudo -S -p '' docker rm -f lifetrac-camera-x8 >/dev/null 2>&1 || true"
Invoke-Adb shell "echo fio | sudo -S -p '' docker rm -f lifetrac-camera-x8-test >/dev/null 2>&1 || true"

Write-Host "[2/3] Probe controls and attempt test-pattern disable ..."
foreach ($dev in $CameraDevices) {
    Write-Host "----- $dev -----"

    $inspectInner = @(
        "set -e",
        "if ! command -v v4l2-ctl >/dev/null 2>&1; then",
        "  if command -v apk >/dev/null 2>&1; then apk add --no-cache v4l-utils >/dev/null 2>&1 || true; fi",
        "  if ! command -v v4l2-ctl >/dev/null 2>&1 && command -v apt-get >/dev/null 2>&1; then apt-get update >/dev/null 2>&1 || true; apt-get install -y v4l-utils >/dev/null 2>&1 || true; fi",
        "fi",
        "if ! command -v v4l2-ctl >/dev/null 2>&1; then echo V4L2CTL_MISSING; exit 3; fi",
        "echo DEVICE=$dev",
        "v4l2-ctl -d $dev --all || true",
        "echo __CTRL_LIST__",
        "v4l2-ctl -d $dev --list-ctrls-menus || true"
    ) -join "; "

    $inspectCmd = @(
        "echo fio | sudo -S -p '' docker run --rm",
        "--entrypoint /bin/sh",
        "--device ${dev}:${dev}",
        "$Image",
        "-lc '$inspectInner'"
    ) -join " "

    $inspectOut = & $script:AdbExe -s $AdbSerial shell $inspectCmd 2>&1
    $inspectOut | Write-Host
    if ($LASTEXITCODE -ne 0) {
        Write-Warning "Probe failed for $dev (rc=$LASTEXITCODE)"
        continue
    }

    $inspectText = ($inspectOut | Out-String)
    if ($inspectText -match "test_pattern|test pattern|test-pattern") {
        Write-Host "TEST_PATTERN_LIKE_CONTROL_FOUND on $dev; attempting set-ctrl=test_pattern=0"
        $setInner = "if command -v v4l2-ctl >/dev/null 2>&1; then v4l2-ctl -d $dev --set-ctrl=test_pattern=0 || true; fi"
        $setCmd = @(
            "echo fio | sudo -S -p '' docker run --rm",
            "--entrypoint /bin/sh",
            "--device ${dev}:${dev}",
            "$Image",
            "-lc '$setInner'"
        ) -join " "
        try {
            Invoke-Adb shell $setCmd
        }
        catch {
            Write-Warning "Set-ctrl attempt failed for $dev : $($_.Exception.Message)"
        }
    }
    else {
        Write-Host "NO_TEST_PATTERN_CONTROL on $dev"
    }
}

Write-Host "[3/3] Probe complete. Relaunch camera container with your preferred node next."
