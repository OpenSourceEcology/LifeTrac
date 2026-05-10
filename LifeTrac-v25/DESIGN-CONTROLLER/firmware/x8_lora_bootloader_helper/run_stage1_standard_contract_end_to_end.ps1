param(
    [string]$AdbSerial = "",
    [string]$RepoRoot = "",
    [string]$LocalImage = ""
)

Set-StrictMode -Version Latest
$ErrorActionPreference = "Stop"

function Resolve-RepoRoot {
    param(
        [string]$Hint,
        [string]$ScriptRoot
    )

    if ($Hint -and (Test-Path -LiteralPath $Hint)) {
        return (Resolve-Path -LiteralPath $Hint).Path
    }

    return (Resolve-Path (Join-Path $ScriptRoot "../../../")).Path
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
$scriptRoot = if ($PSScriptRoot) { $PSScriptRoot } else { (Get-Location).Path }
$repo = Resolve-RepoRoot -Hint $RepoRoot -ScriptRoot $scriptRoot

$helperDir = Join-Path $repo "DESIGN-CONTROLLER/firmware/x8_lora_bootloader_helper"
$benchEvidenceRoot = Join-Path $repo "DESIGN-CONTROLLER/bench-evidence"
$remoteImage = "/tmp/lifetrac_p0c/firmware.bin"

if (-not (Test-Path -LiteralPath $helperDir)) {
    throw "Helper directory not found: $helperDir"
}

$imagePath = $LocalImage
if (-not $imagePath) {
    $imagePath = Join-Path $repo "DESIGN-CONTROLLER/firmware/murata_l072/build/firmware.bin"
}

if (-not (Test-Path -LiteralPath $imagePath)) {
    throw "Image not found: $imagePath"
}

$resolvedImage = (Resolve-Path -LiteralPath $imagePath).Path
$null = New-Item -ItemType Directory -Force -Path $benchEvidenceRoot

Write-Host "Pushing helper toolkit..."
$pushToolkitRc = Invoke-Adb -Args @("push", "$helperDir/.", "/tmp/lifetrac_p0c/")
if ($pushToolkitRc -ne 0) {
    throw "adb push toolkit failed with exit code $pushToolkitRc"
}

Write-Host "Pushing image..."
$pushImageRc = Invoke-Adb -Args @("push", $resolvedImage, $remoteImage)
if ($pushImageRc -ne 0) {
    throw "adb push image failed with exit code $pushImageRc"
}

Write-Host "Setting execute permissions..."
$chmodCmd = "echo fio | sudo -S -p '' bash -lc 'chmod +x /tmp/lifetrac_p0c/*.sh'"
$chmodRc = Invoke-Adb -Args @("exec-out", $chmodCmd)
if ($chmodRc -ne 0) {
    throw "chmod on remote helper scripts failed with exit code $chmodRc"
}

$serialForRemote = if ($AdbSerial) { $AdbSerial } else { "(auto)" }
$runnerCmd = "echo fio | sudo -S -p '' bash /tmp/lifetrac_p0c/run_stage1_standard_contract.sh $remoteImage '$serialForRemote'"
$remoteWrappedCmd = "sh -lc '$runnerCmd; rc=`$?; printf ""__STD_RC__=%s\n"" ""`$rc""'"

Write-Host "Running one-shot Stage1 standard contract..."
$runnerOutput = & adb @($(if ($AdbSerial) { @("-s", $AdbSerial) } else { @() })) exec-out $remoteWrappedCmd 2>&1
$runnerRc = $LASTEXITCODE
$runnerText = ($runnerOutput | Out-String)
Write-Host $runnerText

if ($runnerRc -ne 0) {
    throw "adb exec-out failed with exit code $runnerRc`nOutput:`n$runnerText"
}

$runDirMatch = [regex]::Match($runnerText, "RUN_OUTDIR=([^\r\n]+)")
if (-not $runDirMatch.Success) {
    throw "Could not parse RUN_OUTDIR from output.`nOutput:`n$runnerText"
}

$remoteRunDir = $runDirMatch.Groups[1].Value.Trim()
$pullRc = Invoke-Adb -Args @("pull", $remoteRunDir, $benchEvidenceRoot)
if ($pullRc -ne 0) {
    throw "adb pull failed for $remoteRunDir with exit code $pullRc"
}

$stdRcMatch = [regex]::Match($runnerText, "__STD_RC__=(\d+)")
if (-not $stdRcMatch.Success) {
    throw "Could not parse __STD_RC__ from output.`nOutput:`n$runnerText"
}

$stdRc = [int]$stdRcMatch.Groups[1].Value
$runFolderName = Split-Path -Leaf $remoteRunDir
$localRunDir = Join-Path $benchEvidenceRoot $runFolderName

Write-Host "Remote run dir: $remoteRunDir"
Write-Host "Local evidence dir: $localRunDir"

if ($stdRc -ne 0) {
    throw "Stage1 standard contract failed (remote rc=$stdRc). See $localRunDir"
}

Write-Host "Stage1 standard contract completed successfully."
