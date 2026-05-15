# IP-W2-05/06 on-device dry-run harness for the X8 image-pipeline
# data-saving stack. Pushes camera_service.py + image_pipeline/ + the
# dry-run script over ADB to a writable home dir, runs it with python3
# on the device, and surfaces pass/fail to the host.
#
# IMPORTANT — Python runtime requirement:
#   The bare Foundries.io LmP rootfs ships a *stripped* Python 3.10 that
#   is missing several stdlib modules our pipeline needs (`logging`,
#   `dataclasses`, `json`, `hashlib`, `select`, `socket`, ...). The
#   intended X8 deployment is to run the camera_service inside a Docker
#   container that bundles a full python3 image. Run this dry-run either
#   (a) from inside that production container, or (b) after installing
#   the missing stdlib packages on the rootfs. Running against the bare
#   rootfs python3 will fail at the first `import camera_service`.
#
# Usage:
#   .\run_w2_05_dry_run.ps1                          # uses default serial
#   .\run_w2_05_dry_run.ps1 -AdbSerial 2E2C1209DABC240B
#
# Exit code 0 on success, non-zero on the first device-side failure.

[CmdletBinding()]
param(
    [string]$AdbSerial = "2E2C1209DABC240B",
    [string]$DeviceDir = "/var/rootdirs/home/fio/lifetrac_w2_05",
    [switch]$KeepDeviceFiles
)

$ErrorActionPreference = "Stop"

$repoRoot = Split-Path -Parent (Split-Path -Parent $PSScriptRoot)
$x8Dir    = Join-Path $repoRoot "firmware\tractor_x8"
$pipeDir  = Join-Path $x8Dir "image_pipeline"
$dryRun   = Join-Path $x8Dir "dry_run_w2_05.py"
$loraProto = Join-Path $repoRoot "base_station\lora_proto.py"

foreach ($p in @($x8Dir, $pipeDir, $dryRun, $loraProto)) {
    if (-not (Test-Path $p)) { throw "missing source path: $p" }
}

function Invoke-Adb {
    param([Parameter(ValueFromRemainingArguments=$true)]$Args)
    & adb -s $AdbSerial @Args
    if ($LASTEXITCODE -ne 0) {
        throw "adb $($Args -join ' ') failed (rc=$LASTEXITCODE)"
    }
}

# 1. Confirm the device is online.
Write-Host "[1/4] Checking ADB device $AdbSerial ..."
$state = & adb -s $AdbSerial get-state 2>&1
if ($LASTEXITCODE -ne 0 -or $state -ne "device") {
    throw "device $AdbSerial not in 'device' state (got: $state)"
}
Write-Host "      device online: $state"

# 2. Stage source files on the device.
Write-Host "[2/4] Pushing sources to $DeviceDir ..."
Invoke-Adb shell "rm -rf $DeviceDir; mkdir -p $DeviceDir/image_pipeline" | Out-Null
Invoke-Adb push $dryRun "$DeviceDir/dry_run_w2_05.py" | Out-Null
Invoke-Adb push (Join-Path $x8Dir "camera_service.py") "$DeviceDir/camera_service.py" | Out-Null
Invoke-Adb push $loraProto "$DeviceDir/lora_proto.py" | Out-Null
Get-ChildItem -Path $pipeDir -File -Filter "*.py" | ForEach-Object {
    Invoke-Adb push $_.FullName "$DeviceDir/image_pipeline/$($_.Name)" | Out-Null
}

# 3. Verify python3 is present and run the dry-run.
Write-Host "[3/4] Locating python3 on the device ..."
$pyPath = (& adb -s $AdbSerial shell "command -v python3 || true").Trim()
if ([string]::IsNullOrWhiteSpace($pyPath)) {
    throw "python3 not found on device (PATH lookup empty)"
}
Write-Host "      python3 -> $pyPath"

Write-Host "[4/4] Running dry-run on the device ..."
# adb shell does not propagate the remote exit code on its own; tag a
# sentinel line we can grep for to decide pass/fail.
$cmd = "cd $DeviceDir && python3 dry_run_w2_05.py; echo __DRYRUN_RC=`$?"
$out = & adb -s $AdbSerial shell $cmd 2>&1
$out | Write-Host
$rcLine = $out | Where-Object { $_ -match '^__DRYRUN_RC=' } | Select-Object -Last 1
if ($null -eq $rcLine) {
    Write-Error "could not parse __DRYRUN_RC sentinel from device output"
    $rc = 1
} else {
    $rc = [int]($rcLine -replace '^__DRYRUN_RC=', '')
}

if (-not $KeepDeviceFiles) {
    & adb -s $AdbSerial shell "rm -rf $DeviceDir" | Out-Null
}

if ($rc -ne 0) {
    Write-Error "device-side dry-run failed (rc=$rc)"
    exit $rc
}

Write-Host "OK: on-device W2-05/06 dry-run passed."
exit 0
