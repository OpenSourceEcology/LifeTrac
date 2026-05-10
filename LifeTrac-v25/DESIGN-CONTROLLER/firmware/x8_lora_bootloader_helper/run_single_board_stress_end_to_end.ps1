param(
    [string]$AdbSerial = "",
    [string]$ImageRemote = "/tmp/lifetrac_p0c/mlm32l07x01.bin",
    [int]$Cycles = 20,
    [int]$RunPostListen = 1,
    [int]$PostListenSec = 10,
    [int]$RunRevive = 0,
    [string]$RepoRoot = ""
)

Set-StrictMode -Version Latest
$ErrorActionPreference = "Stop"
$ScriptRoot = if ($PSScriptRoot) { $PSScriptRoot } elseif ($PSCommandPath) { Split-Path -Parent $PSCommandPath } else { (Get-Location).Path }

function Resolve-RepoRoot {
    if ($RepoRoot -and (Test-Path -LiteralPath $RepoRoot)) {
        return (Resolve-Path -LiteralPath $RepoRoot).Path
    }

    # Script location:
    # LifeTrac-v25/DESIGN-CONTROLLER/firmware/x8_lora_bootloader_helper/
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
$importer = Join-Path $helperDir "pull_stress_report_to_ai_notes.ps1"

if (-not (Test-Path -LiteralPath $helperDir)) {
    throw "Helper directory not found: $helperDir"
}

if (-not (Test-Path -LiteralPath $importer)) {
    throw "Importer script not found: $importer"
}

Write-Host "Pushing helper toolkit to X8..."
$pushRc = Invoke-Adb -Args @("push", "$helperDir/.", "/tmp/lifetrac_p0c/")
if ($pushRc -ne 0) {
    throw "adb push failed with exit code $pushRc"
}

Write-Host "Ensuring shell scripts are executable..."
$chmodCmd = "echo fio | sudo -S -p '' bash -lc 'chmod +x /tmp/lifetrac_p0c/*.sh'"
$chmodRc = Invoke-Adb -Args @("exec-out", $chmodCmd)
if ($chmodRc -ne 0) {
    throw "chmod on X8 failed with exit code $chmodRc"
}

$stressCmd = "echo fio | sudo -S -p '' RUN_POST_LISTEN=$RunPostListen POST_LISTEN_SEC=$PostListenSec RUN_REVIVE=$RunRevive bash /tmp/lifetrac_p0c/run_single_board_stress.sh $ImageRemote $Cycles"
Write-Host "Running stress batch on X8..."
$stressOutput = & adb @($(if ($AdbSerial) { @("-s", $AdbSerial) } else { @() })) exec-out $stressCmd 2>&1
if ($LASTEXITCODE -ne 0) {
    throw "Stress run failed with exit code $LASTEXITCODE`nOutput:`n$stressOutput"
}

$stressText = ($stressOutput | Out-String)
$outdirMatch = $stressText | Select-String -Pattern "(?m)^outdir=.*$" -AllMatches | Select-Object -Last 1
if (-not $outdirMatch) {
    throw "Could not parse stress output directory from stress output.`nOutput:`n$stressText"
}

$outdirLine = $outdirMatch.ToString().Trim()
$runDir = ($outdirLine -replace "^outdir=", "").Trim()
if (-not $runDir) {
    throw "Parsed run directory is empty."
}

Write-Host "Stress run directory: $runDir"
Write-Host "Importing report into AI NOTES..."

$importArgs = @(
    "-ExecutionPolicy", "Bypass",
    "-File", $importer,
    "-RunDir", $runDir,
    "-RepoRoot", $repo
)
if ($AdbSerial) {
    $importArgs += @("-AdbSerial", $AdbSerial)
}

& powershell @importArgs
if ($LASTEXITCODE -ne 0) {
    throw "Importer failed with exit code $LASTEXITCODE"
}

Write-Host "End-to-end run complete."
Write-Host "Run dir: $runDir"
