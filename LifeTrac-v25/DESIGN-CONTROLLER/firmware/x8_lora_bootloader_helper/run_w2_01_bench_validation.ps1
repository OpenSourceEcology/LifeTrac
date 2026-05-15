#requires -Version 5.1
<#
.SYNOPSIS
    W2-01 bench validation harness for the production camera USB fixes.

.DESCRIPTION
    Stages the production deliverables from Section 10.1 of
    `AI NOTES/2026-05-14_USB_Wedge_Software_Mitigations.md`
    (`provision_x8.sh`, `99-w2-01-c2.rules`, `lifetrac-no-usb-audio.conf`,
    `lifetrac-camera.service`) onto the bench Portenta X8 over ADB
    WITHOUT installing them into `/etc` by default, then exercises the
    V1..V4 validation matrix from Section 10.3:

        V1  Cold-boot enumeration (loop count via -Cycles)
        V2  snd_usb_audio blocked across boots
        V4  Hotplug recovery (manual unplug/replug if -Interactive)

    V3 (wedge reproducibility), V5 (VBUS scope), and V6 (idle-power
    delta) require either kernel-cmdline changes or external test
    equipment and are out of scope for this script - they are flagged
    in the summary as DEFERRED.

    Artifacts land under
        DESIGN-CONTROLLER/bench-evidence/w2_01_production_<YYYY-MM-DD_HHMMSS>/
    matching the path declared in §10.3 of the mitigations doc.

.PARAMETER AdbSerial
    ADB serial of the bench X8. Required.

.PARAMETER Cycles
    Number of cold-boot validation cycles for V1. Default 1 (smoke).
    Use 20 for the §10.3.V1 production gate.

.PARAMETER Install
    If supplied, copies the staged artifacts into `/etc` on the target
    via `provision_x8.sh`. Default: staged-only (dry run).

.PARAMETER Interactive
    Pause for manual unplug/replug during V4. Without this the V4 row
    is reported as DEFERRED.

.PARAMETER SysfsOnly
    Validate USB enumeration, udev, and driver policy only. This skips
    service-active gating because the production camera container is not
    installed on the bench X8 yet.

.PARAMETER RepoRoot
    Optional override; default derived from script location.

.EXAMPLE
    # Smoke / staging only (no /etc writes, no reboot loop)
    powershell -NoProfile -ExecutionPolicy Bypass `
        -File LifeTrac-v25/DESIGN-CONTROLLER/firmware/x8_lora_bootloader_helper/run_w2_01_bench_validation.ps1 `
        -AdbSerial 2E2C1209DABC240B

.EXAMPLE
    # Full Section 10.3 V1 production gate (installs to /etc, 20 cold boots)
    powershell -NoProfile -ExecutionPolicy Bypass `
        -File LifeTrac-v25/DESIGN-CONTROLLER/firmware/x8_lora_bootloader_helper/run_w2_01_bench_validation.ps1 `
        -AdbSerial 2E2C1209DABC240B -Install -Cycles 20
#>
param(
    [Parameter(Mandatory = $true)][string]$AdbSerial,
    [int]$Cycles = 1,
    [switch]$Install,
    [switch]$Interactive,
    [switch]$SysfsOnly,
    [string]$RepoRoot = ''
)

Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'

$ScriptRoot = if ($PSScriptRoot) { $PSScriptRoot }
              elseif ($PSCommandPath) { Split-Path -Parent $PSCommandPath }
              else { (Get-Location).Path }

if (-not $RepoRoot -or -not (Test-Path -LiteralPath $RepoRoot)) {
    $RepoRoot = (Resolve-Path (Join-Path $ScriptRoot '../../../')).Path
}

$Stamp     = (Get-Date).ToString('yyyy-MM-dd_HHmmss')
$EvidenceDir = Join-Path $RepoRoot ("DESIGN-CONTROLLER/bench-evidence/w2_01_production_" + $Stamp)
New-Item -ItemType Directory -Force -Path $EvidenceDir | Out-Null
Write-Host "Evidence dir: $EvidenceDir"

$TargetTmp = '/tmp/w2_01_production'

function Invoke-Adb {
    param([string[]]$AdbCommand, [switch]$IgnoreExit)
    $adbCommandLine = @('-s', $AdbSerial) + $AdbCommand
    # adb writes progress to stderr; merge to stdout but suppress error-action
    $prevEAP = $ErrorActionPreference
    $ErrorActionPreference = 'Continue'
    try {
        $out = & adb @adbCommandLine 2>&1 | ForEach-Object { "$_" }
    } finally {
        $ErrorActionPreference = $prevEAP
    }
    if (-not $IgnoreExit -and $LASTEXITCODE -ne 0) {
        throw "adb $($AdbCommand -join ' ') failed (exit $LASTEXITCODE): $out"
    }
    return $out
}

function Invoke-AdbShell {
    param([string]$Cmd, [switch]$IgnoreExit)
    return Invoke-Adb -AdbCommand @('shell', $Cmd) -IgnoreExit:$IgnoreExit
}

# ---- 0. Pre-flight ----------------------------------------------------------
Write-Host "`n=== 0. Pre-flight ==="
$devices = (& adb devices 2>&1) | Out-String
$devices | Out-File (Join-Path $EvidenceDir '00_adb_devices.txt') -Encoding utf8
if ($devices -notmatch [regex]::Escape($AdbSerial)) {
    throw "Target $AdbSerial not in 'adb devices' output."
}

$lsusb = (Invoke-AdbShell 'lsusb' -IgnoreExit) | Out-String
$lsusb | Out-File (Join-Path $EvidenceDir '01_lsusb_pre.txt') -Encoding utf8
$c2Present = [bool]($lsusb -match '16d0:0ed4')
Write-Host ("  C2 (16d0:0ed4) present : " + $c2Present)

$cmdline = (Invoke-AdbShell 'cat /proc/cmdline') | Out-String
$cmdline | Out-File (Join-Path $EvidenceDir '02_proc_cmdline.txt') -Encoding utf8
$autosuspendInCmdline = [bool]($cmdline -match 'usbcore\.autosuspend=-1')
Write-Host ("  usbcore.autosuspend=-1 in cmdline : " + $autosuspendInCmdline)

$autosuspendRaw = (Invoke-AdbShell 'cat /sys/module/usbcore/parameters/autosuspend 2>/dev/null || echo MISSING' -IgnoreExit) | Out-String
$autosuspendNow = $autosuspendRaw.Trim()
Write-Host ("  current usbcore.autosuspend       : $autosuspendNow")

$sndUsbAudio = (Invoke-AdbShell 'lsmod 2>/dev/null | grep -i snd_usb_audio || echo NONE' -IgnoreExit) | Out-String
Write-Host ("  snd_usb_audio bound               : " + $sndUsbAudio.Trim())

if (-not $c2Present) {
    Write-Warning "Kurokesu C2 (16d0:0ed4) not enumerated. Plug the camera in before running with -Cycles >1 or -Install. Continuing prep run with V1/V2/V4 marked DEFERRED."
}

# ---- 1. Stage artifacts -----------------------------------------------------
Write-Host "`n=== 1. Staging artifacts to $TargetTmp ==="
Invoke-AdbShell "rm -rf $TargetTmp && mkdir -p $TargetTmp" | Out-Null

$artifacts = @(
    'provision_x8.sh',
    '99-w2-01-c2.rules',
    'lifetrac-no-usb-audio.conf',
    'lifetrac-camera.service'
)
foreach ($a in $artifacts) {
    $local = Join-Path $ScriptRoot $a
    if (-not (Test-Path -LiteralPath $local)) {
        throw "Missing artifact $local"
    }
    Invoke-Adb -AdbCommand @('push', $local, "$TargetTmp/$a") | Out-Null
    Write-Host "  pushed $a"
}
Invoke-AdbShell "chmod +x $TargetTmp/provision_x8.sh" | Out-Null

# ---- 2. Optional install ----------------------------------------------------
if ($Install) {
    Write-Host "`n=== 2. Installing artifacts via provision_x8.sh ==="
    $pw = if ($env:GUARD_SUDO_PASS) { $env:GUARD_SUDO_PASS } else { 'fio' }
    $installLog = Invoke-AdbShell "cd $TargetTmp && echo '$pw' | sudo -S ./provision_x8.sh" -IgnoreExit
    $installLog | Out-File (Join-Path $EvidenceDir '10_install.log') -Encoding utf8
    Write-Host "  install log written"
} else {
    Write-Host "`n=== 2. Install SKIPPED (no -Install flag). Staged only. ==="
    "Skipped (dry run; pass -Install to apply)" |
        Out-File (Join-Path $EvidenceDir '10_install.log') -Encoding utf8
}

# ---- 3. V1 cold-boot loop ---------------------------------------------------
Write-Host "`n=== 3. V1 cold-boot enumeration ($Cycles cycles) ==="
$v1Pass = 0; $v1Fail = 0
$v2Pass = 0; $v2Fail = 0
if (-not $c2Present -and -not $Install) {
    Write-Host "  V1 SKIPPED: C2 not present and -Install not given (prep only)."
} else {
for ($i = 1; $i -le $Cycles; $i++) {
    Write-Host ("--- Cycle $i / $Cycles ---")
    $cycleDir = Join-Path $EvidenceDir ("v1_cycle_{0:D2}" -f $i)
    New-Item -ItemType Directory -Force -Path $cycleDir | Out-Null

    if ($i -gt 1) {
        # Skip reboot on first cycle so we capture pre-state. After that,
        # cold-boot via reboot. Real cold-boot needs power cycling - flag.
        Invoke-AdbShell 'sync' -IgnoreExit | Out-Null
        Invoke-AdbShell 'reboot' -IgnoreExit | Out-Null
        Write-Host "  reboot issued, waiting for ADB to come back..."
        & adb -s $AdbSerial wait-for-device 2>&1 | Out-Null
        Start-Sleep -Seconds 5  # let userspace settle
    }

    $lsusbCycle = (Invoke-AdbShell 'lsusb' -IgnoreExit) | Out-String
    $lsusbCycle | Out-File (Join-Path $cycleDir 'lsusb.txt') -Encoding utf8
    $c2 = [bool]($lsusbCycle -match '16d0:0ed4')

    $lsusbTree = (Invoke-AdbShell 'lsusb -t 2>/dev/null || true' -IgnoreExit) | Out-String
    $lsusbTree | Out-File (Join-Path $cycleDir 'lsusb_tree.txt') -Encoding utf8

    $v4lById = (Invoke-AdbShell 'ls -l /dev/v4l/by-id /dev/lifetrac-c2 2>&1 || true' -IgnoreExit) | Out-String
    $v4lById | Out-File (Join-Path $cycleDir 'v4l_links.txt') -Encoding utf8

    $linkCheck = (Invoke-AdbShell 'if [ -e /dev/lifetrac-c2 ] && [ -e /dev/v4l/by-id/usb-Kurokesu_C2_KTM-QGFST-video-index0 ] && [ "$(readlink -f /dev/lifetrac-c2)" = "$(readlink -f /dev/v4l/by-id/usb-Kurokesu_C2_KTM-QGFST-video-index0)" ]; then echo OK; else echo FAIL; fi' -IgnoreExit) | Out-String
    $linkCheck | Out-File (Join-Path $cycleDir 'lifetrac_c2_link_check.txt') -Encoding utf8
    $linkOk = [bool]($linkCheck -match 'OK')

    $svc = (Invoke-AdbShell 'systemctl is-active lifetrac-camera.service 2>&1 || true' -IgnoreExit) | Out-String
    $svc | Out-File (Join-Path $cycleDir 'systemctl_is_active.txt') -Encoding utf8
    $svcActive = [bool]($svc -match '^active')

    $snd = (Invoke-AdbShell 'lsmod | grep -i snd_usb_audio || echo NONE' -IgnoreExit) | Out-String
    $snd | Out-File (Join-Path $cycleDir 'lsmod_snd.txt') -Encoding utf8

    $dmesgTail = (Invoke-AdbShell 'dmesg | tail -100' -IgnoreExit) | Out-String
    $dmesgTail | Out-File (Join-Path $cycleDir 'dmesg_tail.txt') -Encoding utf8

    $okV1 = $c2 -and $linkOk -and ($svcActive -or -not $Install)
    if ($SysfsOnly) { $okV1 = $c2 -and $linkOk }
    $okV2 = [bool]($snd -match 'NONE')
    if ($okV1) { $v1Pass++ } else { $v1Fail++ }
    if ($okV2) { $v2Pass++ } else { $v2Fail++ }
    Write-Host ("  C2 enum: $c2, lifetrac-c2 index0: $linkOk, svc active: $svcActive, snd_usb_audio absent: $okV2")
}
}

# ---- 4. V4 hotplug recovery -------------------------------------------------
$v4Status = 'DEFERRED'
if ($Interactive) {
    Write-Host "`n=== 4. V4 hotplug recovery (interactive) ==="
    Read-Host 'UNPLUG the C2 USB cable now, then press Enter'
    Invoke-AdbShell 'lsusb' -IgnoreExit | Out-File (Join-Path $EvidenceDir '40_lsusb_unplugged.txt') -Encoding utf8
    Read-Host 'REPLUG the C2 USB cable now, then press Enter'
    Start-Sleep -Seconds 3
    $afterReplug = (Invoke-AdbShell 'lsusb' -IgnoreExit) | Out-String
    $afterReplug | Out-File (Join-Path $EvidenceDir '41_lsusb_replugged.txt') -Encoding utf8
    $svcAfter = (Invoke-AdbShell 'systemctl is-active lifetrac-camera.service 2>&1 || true' -IgnoreExit) | Out-String
    $svcAfter | Out-File (Join-Path $EvidenceDir '42_systemctl_after_replug.txt') -Encoding utf8
    $v4Status = if (($afterReplug -match '16d0:0ed4') -and (($svcAfter -match '^active') -or (-not $Install))) { 'PASS' } else { 'FAIL' }
}

# ---- 5. Summary -------------------------------------------------------------
$summary = [ordered]@{
    timestamp                 = $Stamp
    adb_serial                = $AdbSerial
    install_applied           = [bool]$Install
    sysfs_only                = [bool]$SysfsOnly
    cycles                    = $Cycles
    pre_state = @{
        c2_present              = $c2Present
        autosuspend_in_cmdline  = $autosuspendInCmdline
        autosuspend_runtime     = $autosuspendNow
        snd_usb_audio_pre       = ($sndUsbAudio | Out-String).Trim()
    }
    v1_cold_boot_enum         = @{ pass = $v1Pass; fail = $v1Fail }
    v2_snd_usb_audio_blocked  = @{ pass = $v2Pass; fail = $v2Fail }
    v3_wedge_reproducibility  = 'DEFERRED (needs Section 10.3 V3 stress harness)'
    v4_hotplug_recovery       = $v4Status
    v5_vbus_scope             = 'DEFERRED (needs USB power meter + scope)'
    v6_idle_power_delta       = 'DEFERRED (needs DC current clamp)'
}
$summary | ConvertTo-Json -Depth 5 |
    Out-File (Join-Path $EvidenceDir 'summary.json') -Encoding utf8

Write-Host "`n=== Summary ==="
$summary | ConvertTo-Json -Depth 5 | Write-Host
Write-Host "`nEvidence: $EvidenceDir"
