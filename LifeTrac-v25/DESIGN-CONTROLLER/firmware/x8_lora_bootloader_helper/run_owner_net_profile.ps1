param(
    [string]$Target,
    [string]$ManifestPath = "",
    [string]$AdbSerial = "2E2C1209DABC240B",
    [string]$RepoRoot = "",
    [string]$ProbeDevice = "",
    [int]$Baud = 0,
    [double]$BootTimeout = 0,
    [switch]$SkipDiagProbes
)
<##
.SYNOPSIS
    Manifest-driven halted cfg runner for owner-net profile experiments.

.EXAMPLE
    ./run_owner_net_profile.ps1 -Target pa11_inreset_toggle_recheck

.EXAMPLE
    ./run_owner_net_profile.ps1 -Target ownerexp_onehot_replay -SkipDiagProbes
##>

Set-StrictMode -Version Latest
$ErrorActionPreference = "Stop"
$ScriptRoot = if ($PSScriptRoot) { $PSScriptRoot } else { (Get-Location).Path }

function Resolve-RepoRoot {
    if ($RepoRoot -and (Test-Path -LiteralPath $RepoRoot)) {
        return (Resolve-Path -LiteralPath $RepoRoot).Path
    }

    # Walk up from the script location and pick the nearest ancestor that contains DESIGN-CONTROLLER.
    $current = (Resolve-Path -LiteralPath $ScriptRoot).Path
    for ($i = 0; $i -lt 8 -and $current; $i++) {
        if (Test-Path -LiteralPath (Join-Path $current "DESIGN-CONTROLLER")) {
            return $current
        }
        $parentInfo = [System.IO.Directory]::GetParent($current)
        if (-not $parentInfo) {
            break
        }
        $parentPath = $parentInfo.FullName
        if (-not $parentPath -or $parentPath -eq $current) {
            break
        }
        $current = $parentPath
    }

    throw "Unable to resolve repo root from script location '$ScriptRoot'. Pass -RepoRoot explicitly."
}

function Invoke-AdbRaw {
    param([Parameter(Mandatory = $true)][string]$Command)
    $prefix = if ($AdbSerial) { @("-s", $AdbSerial) } else { @() }
    & adb @prefix exec-out $Command
    if ($LASTEXITCODE -ne 0) {
        throw "adb exec-out command failed (rc=$LASTEXITCODE): $Command"
    }
}

function Invoke-HaltedCase {
    param(
        [Parameter(Mandatory = $true)][string]$CfgName,
        [Parameter(Mandatory = $true)][string]$CaseTag,
        [Parameter(Mandatory = $true)][string]$EvidenceDir,
        [Parameter(Mandatory = $true)][string]$ProbeDev,
        [Parameter(Mandatory = $true)][int]$ProbeBaud,
        [Parameter(Mandatory = $true)][double]$ProbeBootTimeout,
        [string]$ReleaseCfg = "99_release_and_reset"
    )

    $cfgFile = "/tmp/lifetrac_p0c/$CfgName.cfg"
    $relCfgFile = "/tmp/lifetrac_p0c/$ReleaseCfg.cfg"

    $ocdLog = "/tmp/p0c_profile_${CaseTag}_openocd.log"
    $probeLog = "/tmp/p0c_profile_${CaseTag}_probe.log"
    $releaseLog = "/tmp/p0c_profile_${CaseTag}_release.log"

    Write-Host "`n--- Case $CaseTag ($CfgName) ---"

    $ocdCmd = "echo fio | sudo -S -p '' openocd -f /usr/arduino/extra/openocd_script-imx_gpio.cfg -f $cfgFile > $ocdLog 2>&1"
    Write-Host "  Step 1: Apply halted cfg"
    Invoke-AdbRaw -Command $ocdCmd | Out-Null

    Write-Host "  Step 2: Wait 800ms"
    Start-Sleep -Milliseconds 800

    $probeCmd = "echo fio | sudo -S -p '' python3 /tmp/lifetrac_p0c/method_g_stage1_probe.py --dev $ProbeDev --baud $ProbeBaud --boot-timeout $ProbeBootTimeout 2>&1 | tee $probeLog"
    Write-Host "  Step 3: Probe"
    $probeOut = Invoke-AdbRaw -Command $probeCmd
    Write-Host ($probeOut | Out-String)

    $releaseCmd = "echo fio | sudo -S -p '' openocd -f /usr/arduino/extra/openocd_script-imx_gpio.cfg -f $relCfgFile > $releaseLog 2>&1"
    Write-Host "  Step 4: Release/reset"
    Invoke-AdbRaw -Command $releaseCmd | Out-Null

    Write-Host "  Step 5: Pull logs"
    $prefix = if ($AdbSerial) { @("-s", $AdbSerial) } else { @() }

    & adb @prefix pull $ocdLog (Join-Path $EvidenceDir "$CaseTag.openocd.log") | Out-Null
    if ($LASTEXITCODE -ne 0) { throw "Failed to pull $ocdLog" }

    & adb @prefix pull $probeLog (Join-Path $EvidenceDir "$CaseTag.probe.log") | Out-Null
    if ($LASTEXITCODE -ne 0) { throw "Failed to pull $probeLog" }

    & adb @prefix pull $releaseLog (Join-Path $EvidenceDir "$CaseTag.release.log") | Out-Null
    if ($LASTEXITCODE -ne 0) { throw "Failed to pull $releaseLog" }
}

$null = Get-Command adb -ErrorAction Stop
$repo = Resolve-RepoRoot
$helperDir = Join-Path $repo "DESIGN-CONTROLLER/firmware/x8_lora_bootloader_helper"
$manifestResolved = if ($ManifestPath) { (Resolve-Path -LiteralPath $ManifestPath).Path } else { Join-Path $helperDir "owner_net_profiles.json" }

if (-not (Test-Path -LiteralPath $manifestResolved)) {
    throw "Manifest not found: $manifestResolved"
}

if (-not $Target) {
    throw "Profile is required. Example: -Target pa11_inreset_toggle_recheck"
}

$manifestObj = Get-Content -LiteralPath $manifestResolved -Raw | ConvertFrom-Json
$profileObj = $manifestObj.profiles | Where-Object { $_.name -eq $Target } | Select-Object -First 1
if (-not $profileObj) {
    $known = ($manifestObj.profiles | ForEach-Object { $_.name }) -join ", "
    throw "Unknown profile '$Target'. Known: $known"
}

$probeDevFinal = if ($ProbeDevice) { $ProbeDevice } else { [string]$profileObj.probeDevice }
$baudFinal = if ($Baud -gt 0) { $Baud } else { [int]$profileObj.baud }
$timeoutFinal = if ($BootTimeout -gt 0) { $BootTimeout } else { [double]$profileObj.bootTimeout }

$evidRoot = Join-Path $repo "DESIGN-CONTROLLER/bench-evidence"
$stamp = Get-Date -Format "yyyy-MM-dd_HHmmss"
$prefix = if ($profileObj.evidencePrefix) { [string]$profileObj.evidencePrefix } else { "T6_profile_$Target" }
$evidDir = Join-Path $evidRoot ($prefix + "_" + $stamp)
$null = New-Item -ItemType Directory -Force -Path $evidDir

Write-Host "=== Owner-net profile run ==="
Write-Host "Profile: $Target"
Write-Host "Description: $($profileObj.description)"
Write-Host "Manifest: $manifestResolved"
Write-Host "Probe: $probeDevFinal @ $baudFinal, boot-timeout=$timeoutFinal"
Write-Host "Evidence: $evidDir"

Write-Host "`nPushing helper toolkit..."
$adbPrefix = if ($AdbSerial) { @("-s", $AdbSerial) } else { @() }
& adb @adbPrefix push "$helperDir/." "/tmp/lifetrac_p0c/" | Out-Null
if ($LASTEXITCODE -ne 0) {
    throw "adb push failed (rc=$LASTEXITCODE)"
}

if (-not $SkipDiagProbes) {
    Write-Host "`n=== Running probe_lora_dt.sh ==="
    $dtOut = Invoke-AdbRaw -Command "echo fio | sudo -S -p '' bash /tmp/lifetrac_p0c/probe_lora_dt.sh 2>&1"
    $dtStr = $dtOut | Out-String
    Write-Host $dtStr
    $dtStr | Set-Content -LiteralPath (Join-Path $evidDir "probe_lora_dt.txt")

    Write-Host "`n=== Running probe_lora_userspace.sh ==="
    $usOut = Invoke-AdbRaw -Command "echo fio | sudo -S -p '' bash /tmp/lifetrac_p0c/probe_lora_userspace.sh 2>&1"
    $usStr = $usOut | Out-String
    Write-Host $usStr
    $usStr | Set-Content -LiteralPath (Join-Path $evidDir "probe_lora_userspace.txt")
}

foreach ($case in $profileObj.cases) {
    Invoke-HaltedCase -CfgName ([string]$case.cfg) -CaseTag ([string]$case.tag) -EvidenceDir $evidDir -ProbeDev $probeDevFinal -ProbeBaud $baudFinal -ProbeBootTimeout $timeoutFinal
}

Write-Host "`n=== Done. Evidence in: $evidDir ==="
