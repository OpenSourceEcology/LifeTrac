<#
.SYNOPSIS
    W2-01 §10.3.V3 stress harness: gate + reproducibility probe + ADB liveness.

.DESCRIPTION
    Pushes w2_01_camera_usb_guard_gate.sh and w2_01_camera_v3_stress.sh to
    the X8 over ADB, runs the gate (must PASS to proceed), then runs the
    V3 stress loop. Verifies ADB is still alive after each phase and pulls
    artifacts into LifeTrac-v25/DESIGN-CONTROLLER/bench-evidence/.

    Outcome gates §10.1.F (OSTREE_KERNEL_ARGS usbcore.autosuspend=-1):
      - verdict=PASS         => B+C+D+E sufficient; F can stay shelved.
      - verdict=FAIL_WEDGE   => ship F.
      - verdict=FAIL_IO      => investigate before shipping F.

.PARAMETER AdbSerial
    ADB serial of the target X8.

.PARAMETER Iterations
    Stress iterations (default 50, per §10.3.V3 acceptance row).

.PARAMETER SleepMs
    Inter-iteration sleep in milliseconds (default 200).

.PARAMETER EvidenceRoot
    Root for bench-evidence; a timestamped subfolder is created.
#>
[CmdletBinding()]
param(
    [Parameter(Mandatory = $true)] [string] $AdbSerial,
    [int] $Iterations = 50,
    [int] $SleepMs = 200,
    [string] $EvidenceRoot
)

$ErrorActionPreference = 'Stop'
$scriptDir = $PSScriptRoot
if (-not $scriptDir) { $scriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path }
if (-not $EvidenceRoot) { $EvidenceRoot = Join-Path $scriptDir '..\..\bench-evidence' }
$guard     = Join-Path $scriptDir 'w2_01_camera_usb_guard.sh'
$gate      = Join-Path $scriptDir 'w2_01_camera_usb_guard_gate.sh'
$stress    = Join-Path $scriptDir 'w2_01_camera_v3_stress.sh'
foreach ($p in @($guard, $gate, $stress)) {
    if (-not (Test-Path -LiteralPath $p)) { throw "Missing artifact: $p" }
}

$stamp = Get-Date -Format 'yyyy-MM-dd_HHmmss'
$evDir = Join-Path (Resolve-Path $EvidenceRoot) ("w2_01_v3_stress_$stamp")
New-Item -ItemType Directory -Path $evDir -Force | Out-Null
Write-Host "Evidence directory: $evDir"

function Invoke-Adb {
    param([Parameter(Mandatory = $true)][string[]] $AdbArgs, [string] $StdoutFile)
    $cmdArgs = @('-s', $AdbSerial) + $AdbArgs
    $prevEAP = $ErrorActionPreference
    $ErrorActionPreference = 'Continue'
    $output = & adb @cmdArgs 2>&1
    $ErrorActionPreference = $prevEAP
    foreach ($line in $output) { Write-Host $line }
    if ($StdoutFile) {
        ($output | Out-String) | Set-Content -LiteralPath $StdoutFile -Encoding utf8
    }
    return $LASTEXITCODE
}

function Test-AdbAlive {
    $devices = & adb devices 2>&1
    return ($devices -match [regex]::Escape($AdbSerial) -and $devices -match 'device\s*$') -ne $null
}

# 0. ADB sanity.
Write-Host "[0] ADB liveness pre-test"
$adbPre = Join-Path $evDir '00_adb_devices_pre.txt'
$adbPreOut = & adb devices 2>&1
($adbPreOut | Out-String) | Set-Content -LiteralPath $adbPre -Encoding utf8
if (-not (Get-Content $adbPre | Select-String -Pattern $AdbSerial -SimpleMatch)) {
    throw "ADB serial $AdbSerial not present"
}

# 1. Push artifacts.
Write-Host "[1] Push gate + stress to /tmp"
foreach ($f in @($gate, $stress, $guard)) {
    Invoke-Adb -AdbArgs @('push', $f, '/tmp/') | Out-Null
}
Invoke-Adb -AdbArgs @('shell', 'chmod +x /tmp/w2_01_camera_usb_guard.sh /tmp/w2_01_camera_usb_guard_gate.sh /tmp/w2_01_camera_v3_stress.sh') | Out-Null

# 2. Run gate.
Write-Host "[2] Run guard-gate (E)"
$gateOut = Join-Path $evDir '20_gate_output.txt'
Invoke-Adb -AdbArgs @('shell', 'GUARD=/tmp/w2_01_camera_usb_guard.sh /tmp/w2_01_camera_usb_guard_gate.sh; echo __RC__=$?') -StdoutFile $gateOut | Out-Null
$gateText = Get-Content $gateOut -Raw
$gateLine = ($gateText -split "`n" | Where-Object { $_ -match '^gate=' } | Select-Object -Last 1)
$gateRc   = if ($gateText -match '__RC__=(\d+)') { [int]$Matches[1] } else { -1 }
Write-Host "  gate -> $gateLine (rc=$gateRc)"
if ($gateLine -notmatch 'gate=PASS') {
    Write-Warning "Gate did not PASS. Aborting V3 stress to avoid wedging the host."
    @{
        gate_verdict = $gateLine
        gate_rc      = $gateRc
        skipped      = 'stress'
    } | ConvertTo-Json | Set-Content -LiteralPath (Join-Path $evDir 'summary.json') -Encoding utf8
    return
}

# 3. Run V3 stress.
Write-Host "[3] Run V3 stress: iterations=$Iterations sleep_ms=$SleepMs"
$stressOut = Join-Path $evDir '30_v3_stress_output.txt'
# adbd in LmP gives the fio shell an Android-style supplementary group
# set that does NOT include /etc/group memberships (so /dev/video1 opens
# fail with EPERM even though fio is in the video group). Production
# capture runs via systemd+docker which honour /etc/group. Elevate via
# sudo so the V3 stress actually exercises the V4L2 open path.
$envCmd = "echo fio | sudo -S -p '' env V3_ITERATIONS=$Iterations V3_SLEEP_MS=$SleepMs /tmp/w2_01_camera_v3_stress.sh; echo __RC__=`$?"
Invoke-Adb -AdbArgs @('shell', $envCmd) -StdoutFile $stressOut | Out-Null
$stressText = Get-Content $stressOut -Raw

function Get-Field([string]$text, [string]$key) {
    $m = [regex]::Matches($text, "(?m)^$([regex]::Escape($key))=(.*)$")
    if ($m.Count -gt 0) { return $m[$m.Count - 1].Groups[1].Value.Trim() }
    return $null
}
$verdict     = Get-Field $stressText 'verdict'
$wedgeSeen   = Get-Field $stressText 'wedge_seen'
$wedgeIter   = Get-Field $stressText 'wedge_iter'
$itersRun    = Get-Field $stressText 'iterations_run'
$okOpen      = Get-Field $stressText 'ok_open'
$failOpen    = Get-Field $stressText 'fail_open'
$okSysfs     = Get-Field $stressText 'ok_sysfs'
$failSysfs   = Get-Field $stressText 'fail_sysfs'
$dmesgPre    = Get-Field $stressText 'dmesg_pre'
$dmesgPost   = Get-Field $stressText 'dmesg_post'

Write-Host "  V3 verdict=$verdict iters_run=$itersRun open=$okOpen/$failOpen sysfs=$okSysfs/$failSysfs wedge=$wedgeSeen@$wedgeIter"

# 4. Pull dmesg artifacts.
Write-Host "[4] Pull dmesg artifacts"
foreach ($remote in @($dmesgPre, $dmesgPost)) {
    if ($remote) {
        $local = Join-Path $evDir ([System.IO.Path]::GetFileName($remote))
        Invoke-Adb -AdbArgs @('pull', $remote, $local) | Out-Null
    }
}

# 5. ADB liveness post-test.
Write-Host "[5] ADB liveness post-test"
$adbPost = Join-Path $evDir '50_adb_devices_post.txt'
$adbPostOut = & adb devices 2>&1
($adbPostOut | Out-String) | Set-Content -LiteralPath $adbPost -Encoding utf8
$adbAlive = (Get-Content $adbPost | Select-String -Pattern $AdbSerial -SimpleMatch) -ne $null

# 6. C2 still enumerated?
Write-Host "[6] C2 still enumerated"
$c2Post = Join-Path $evDir '60_c2_post.txt'
Invoke-Adb -AdbArgs @('shell', 'lsusb | grep -i "16d0:0ed4"; ls -l /dev/lifetrac-c2 2>&1; readlink -f /dev/lifetrac-c2 2>&1') -StdoutFile $c2Post | Out-Null

# 7. Summary.
$summary = [ordered]@{
    stamp_utc        = (Get-Date).ToUniversalTime().ToString('o')
    adb_serial       = $AdbSerial
    iterations_req   = $Iterations
    iterations_run   = $itersRun
    sleep_ms         = $SleepMs
    gate_verdict     = $gateLine
    gate_rc          = $gateRc
    v3_verdict       = $verdict
    v3_wedge_seen    = $wedgeSeen
    v3_wedge_iter    = $wedgeIter
    v3_open_pass     = $okOpen
    v3_open_fail     = $failOpen
    v3_sysfs_pass    = $okSysfs
    v3_sysfs_fail    = $failSysfs
    adb_alive_post   = $adbAlive
    artifacts        = @{
        gate_output      = (Split-Path $gateOut -Leaf)
        stress_output    = (Split-Path $stressOut -Leaf)
        dmesg_pre        = if ($dmesgPre)  { [System.IO.Path]::GetFileName($dmesgPre)  } else { $null }
        dmesg_post       = if ($dmesgPost) { [System.IO.Path]::GetFileName($dmesgPost) } else { $null }
        c2_post          = (Split-Path $c2Post -Leaf)
    }
}
$summary | ConvertTo-Json -Depth 4 | Set-Content -LiteralPath (Join-Path $evDir 'summary.json') -Encoding utf8

Write-Host ""
Write-Host "=== W2-01 V3 Stress Summary ==="
Write-Host "  gate    = $gateLine"
Write-Host "  v3      = verdict=$verdict iters=$itersRun/$Iterations open_pass=$okOpen open_fail=$failOpen sysfs_pass=$okSysfs sysfs_fail=$failSysfs wedge=$wedgeSeen@$wedgeIter"
Write-Host "  adb_post= $adbAlive"
Write-Host "  evidence= $evDir"
