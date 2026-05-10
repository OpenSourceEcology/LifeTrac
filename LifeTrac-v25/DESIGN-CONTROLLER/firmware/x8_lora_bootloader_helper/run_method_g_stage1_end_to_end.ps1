param(
    [string]$AdbSerial = "",
    [string]$RepoRoot = "",
    [string]$LocalImage = "",
    [switch]$Build
)

Set-StrictMode -Version Latest
$ErrorActionPreference = "Stop"
$ScriptRoot = if ($PSScriptRoot) { $PSScriptRoot } elseif ($PSCommandPath) { Split-Path -Parent $PSCommandPath } else { (Get-Location).Path }

function Resolve-RepoRoot {
    if ($RepoRoot -and (Test-Path -LiteralPath $RepoRoot)) {
        return (Resolve-Path -LiteralPath $RepoRoot).Path
    }

    $candidate = Resolve-Path (Join-Path $ScriptRoot "../../../")
    return $candidate.Path
}

function Invoke-Adb {
    param(
        [Parameter(Mandatory = $true)]
        [string[]]$Args
    )

    $prefix = @()
    if ($AdbSerial) {
        $prefix = @("-s", $AdbSerial)
    }

    & adb @prefix @Args
    return $LASTEXITCODE
}

$null = Get-Command adb -ErrorAction Stop

$repo = Resolve-RepoRoot
$helperDir = Join-Path $repo "DESIGN-CONTROLLER/firmware/x8_lora_bootloader_helper"
$buildScript = Join-Path $repo "DESIGN-CONTROLLER/firmware/murata_l072/build.ps1"
$benchEvidenceRoot = Join-Path $repo "DESIGN-CONTROLLER/bench-evidence"

function New-EvidenceDir {
    $stamp = Get-Date -Format "yyyy-MM-dd_HHmmss"
    $dir = Join-Path $benchEvidenceRoot "T6_bringup_$stamp"
    $null = New-Item -ItemType Directory -Force -Path $dir
    return $dir
}

function Write-RunMetadata {
    param(
        [Parameter(Mandatory = $true)]
        [string]$Path,
        [Parameter(Mandatory = $true)]
        [string]$ImagePath,
        [Parameter(Mandatory = $true)]
        [bool]$WasBuildRequested,
        [Parameter(Mandatory = $true)]
        [string]$Serial
    )

    $hash = Get-FileHash -LiteralPath $ImagePath -Algorithm SHA256
    $payload = [ordered]@{
        timestamp_utc = (Get-Date).ToUniversalTime().ToString("o")
        adb_serial = $Serial
        local_image = $ImagePath
        local_image_sha256 = $hash.Hash.ToLowerInvariant()
        image_size_bytes = (Get-Item -LiteralPath $ImagePath).Length
        build_requested = $WasBuildRequested
        launcher = "run_method_g_stage1_end_to_end.ps1"
        x8_runner = "run_method_g_stage1.sh"
        probe = "method_g_stage1_probe.py"
    }

    $payload | ConvertTo-Json -Depth 4 | Set-Content -LiteralPath $Path
}

function Pull-AdbArtifact {
    param(
        [Parameter(Mandatory = $true)]
        [string]$RemotePath,
        [Parameter(Mandatory = $true)]
        [string]$LocalPath
    )

    $parent = Split-Path -Parent $LocalPath
    if ($parent) {
        $null = New-Item -ItemType Directory -Force -Path $parent
    }

    $rc = Invoke-Adb -Args @("pull", $RemotePath, $LocalPath)
    if ($rc -ne 0) {
        Write-Warning "adb pull failed for $RemotePath (rc=$rc)"
    }
}

if (-not (Test-Path -LiteralPath $helperDir)) {
    throw "Helper directory not found: $helperDir"
}

$null = New-Item -ItemType Directory -Force -Path $benchEvidenceRoot
$evidenceDir = New-EvidenceDir

$imagePath = $LocalImage
if (-not $imagePath) {
    $imagePath = Join-Path $repo "DESIGN-CONTROLLER/firmware/murata_l072/build/firmware.bin"
}

if ($Build -or -not (Test-Path -LiteralPath $imagePath)) {
    if (-not (Test-Path -LiteralPath $buildScript)) {
        throw "Build script not found: $buildScript"
    }

    Write-Host "Building custom Method G firmware..."
    & powershell -NoProfile -ExecutionPolicy Bypass -File $buildScript
    if ($LASTEXITCODE -ne 0) {
        throw "Build failed with exit code $LASTEXITCODE"
    }
}

if (-not (Test-Path -LiteralPath $imagePath)) {
    throw "Custom firmware image not found after build step: $imagePath"
}

$imageResolved = (Resolve-Path -LiteralPath $imagePath).Path
$imageCopyPath = Join-Path $evidenceDir "firmware.bin"
$metadataPath = Join-Path $evidenceDir "run_metadata.json"

Copy-Item -LiteralPath $imageResolved -Destination $imageCopyPath -Force
$metadataSerial = if ($AdbSerial) { $AdbSerial } else { "(auto)" }
Write-RunMetadata -Path $metadataPath -ImagePath $imageResolved -WasBuildRequested ([bool]$Build) -Serial $metadataSerial

Write-Host "Pushing helper toolkit to X8..."
$pushRc = Invoke-Adb -Args @("push", "$helperDir/.", "/tmp/lifetrac_p0c/")
if ($pushRc -ne 0) {
    throw "adb push of helper toolkit failed with exit code $pushRc"
}

Write-Host "Pushing custom Method G image to X8..."
$imagePushRc = Invoke-Adb -Args @("push", $imageResolved, "/tmp/lifetrac_p0c/firmware.bin")
if ($imagePushRc -ne 0) {
    throw "adb push of firmware image failed with exit code $imagePushRc"
}

Write-Host "Ensuring helper shell scripts are executable..."
$chmodCmd = "echo fio | sudo -S -p '' bash -lc 'chmod +x /tmp/lifetrac_p0c/*.sh'"
$chmodRc = Invoke-Adb -Args @("exec-out", $chmodCmd)
if ($chmodRc -ne 0) {
    throw "chmod on X8 failed with exit code $chmodRc"
}

$stage1Cmd = "echo fio | sudo -S -p '' bash /tmp/lifetrac_p0c/run_method_g_stage1.sh /tmp/lifetrac_p0c/firmware.bin"
Write-Host "Running Method G Stage 1 flow on X8..."
$remoteWrappedCmd = "sh -lc '$stage1Cmd; rc=`$?; printf ""__METHOD_G_RC__=%s\\n"" ""`$rc""'"
$stage1Output = & adb @($(if ($AdbSerial) { @("-s", $AdbSerial) } else { @() })) exec-out $remoteWrappedCmd 2>&1
$stage1Rc = $LASTEXITCODE

$stage1Text = ($stage1Output | Out-String)
Write-Host $stage1Text

$remoteRc = $null
$remoteRcMatch = [regex]::Match($stage1Text, "__METHOD_G_RC__=(\d+)")
if ($remoteRcMatch.Success) {
    $remoteRc = [int]$remoteRcMatch.Groups[1].Value
}

$stdoutPath = Join-Path $evidenceDir "stage1_stdout.txt"
Set-Content -LiteralPath $stdoutPath -Value $stage1Text

Pull-AdbArtifact -RemotePath "/tmp/lifetrac_p0c/method_g_stage1.log" -LocalPath (Join-Path $evidenceDir "method_g_stage1.log")
Pull-AdbArtifact -RemotePath "/tmp/lifetrac_p0c/method_g_stage1_ocd.log" -LocalPath (Join-Path $evidenceDir "method_g_stage1_ocd.log")
Pull-AdbArtifact -RemotePath "/tmp/lifetrac_p0c/flash_run.log" -LocalPath (Join-Path $evidenceDir "flash_run.log")
Pull-AdbArtifact -RemotePath "/tmp/lifetrac_p0c/flash_ocd.log" -LocalPath (Join-Path $evidenceDir "flash_ocd.log")

if ($stage1Rc -ne 0) {
    throw "adb exec-out failed with exit code $stage1Rc`nEvidence: $evidenceDir`nOutput:`n$stage1Text"
}

if ($null -eq $remoteRc) {
    throw "Could not parse remote Stage 1 exit code from X8 output.`nEvidence: $evidenceDir`nOutput:`n$stage1Text"
}

if ($remoteRc -ne 0) {
    throw "Method G Stage 1 flow failed on X8 with exit code $remoteRc`nEvidence: $evidenceDir`nOutput:`n$stage1Text"
}

Write-Host "Method G Stage 1 end-to-end run complete."
Write-Host "Image: $imageResolved"
Write-Host "Evidence: $evidenceDir"