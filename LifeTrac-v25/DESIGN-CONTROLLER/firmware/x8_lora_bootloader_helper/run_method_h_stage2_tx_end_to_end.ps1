param(
    [string]$AdbSerial = "",
    [string]$RepoRoot = ""
)

Set-StrictMode -Version Latest
$ErrorActionPreference = "Stop"
$ScriptRoot = if ($PSScriptRoot) { $PSScriptRoot } elseif ($PSCommandPath) { Split-Path -Parent $PSCommandPath } else { (Get-Location).Path }

function Resolve-RepoRoot {
    if ($RepoRoot -and (Test-Path -LiteralPath $RepoRoot)) {
        return (Resolve-Path -LiteralPath $RepoRoot).Path
    }
    return (Resolve-Path (Join-Path $ScriptRoot "../../../")).Path
}

function Invoke-Adb {
    param([Parameter(Mandatory = $true)][string[]]$Args)
    $prefix = @()
    if ($AdbSerial) { $prefix = @("-s", $AdbSerial) }
    & adb @prefix @Args
    return $LASTEXITCODE
}

$null = Get-Command adb -ErrorAction Stop

$repo = Resolve-RepoRoot
$helperDir = Join-Path $repo "DESIGN-CONTROLLER/firmware/x8_lora_bootloader_helper"
$benchEvidenceRoot = Join-Path $repo "DESIGN-CONTROLLER/bench-evidence"
$null = New-Item -ItemType Directory -Force -Path $benchEvidenceRoot

$stamp = Get-Date -Format "yyyy-MM-dd_HHmmss"
$evidenceDir = Join-Path $benchEvidenceRoot "W1-9_stage2_tx_$stamp"
$null = New-Item -ItemType Directory -Force -Path $evidenceDir

Write-Host "Pushing helper toolkit (probe + shell) to X8..."
$pushRc = Invoke-Adb -Args @("push", "$helperDir/.", "/tmp/lifetrac_p0c/")
if ($pushRc -ne 0) { throw "adb push failed (rc=$pushRc)" }

$chmodCmd = "echo fio | sudo -S -p '' bash -lc 'chmod +x /tmp/lifetrac_p0c/*.sh'"
$chmodRc = Invoke-Adb -Args @("exec-out", $chmodCmd)
if ($chmodRc -ne 0) { throw "chmod on X8 failed (rc=$chmodRc)" }

$stage2Cmd = "echo fio | sudo -S -p '' bash /tmp/lifetrac_p0c/run_method_h_stage2_tx.sh"
$wrapped = "sh -lc '$stage2Cmd; rc=`$?; printf ""__METHOD_H_RC__=%s\\n"" ""`$rc""'"
Write-Host "Running Method H Stage 2 TX flow on X8..."
$out = & adb @($(if ($AdbSerial) { @("-s", $AdbSerial) } else { @() })) exec-out $wrapped 2>&1
$adbRc = $LASTEXITCODE
$text = ($out | Out-String)
Write-Host $text

Set-Content -LiteralPath (Join-Path $evidenceDir "stage2_tx_stdout.txt") -Value $text

$remoteRcMatch = [regex]::Match($text, "__METHOD_H_RC__=(\d+)")
$remoteRc = if ($remoteRcMatch.Success) { [int]$remoteRcMatch.Groups[1].Value } else { $null }

# Pull logs regardless of outcome.
foreach ($leaf in @("method_h_stage2_tx.log", "method_h_stage2_tx_ocd.log")) {
    $rPath = "/tmp/lifetrac_p0c/$leaf"
    $lPath = Join-Path $evidenceDir $leaf
    $pullRc = Invoke-Adb -Args @("pull", $rPath, $lPath)
    if ($pullRc -ne 0) { Write-Warning "adb pull failed for $rPath (rc=$pullRc)" }
}

if ($adbRc -ne 0) {
    throw "adb exec-out failed (rc=$adbRc). Evidence: $evidenceDir"
}
if ($null -eq $remoteRc) {
    throw "Could not parse __METHOD_H_RC__ from output. Evidence: $evidenceDir"
}
if ($remoteRc -ne 0) {
    throw "Method H Stage 2 TX failed on X8 (rc=$remoteRc). Evidence: $evidenceDir"
}

Write-Host "Method H Stage 2 TX run complete."
Write-Host "Evidence: $evidenceDir"
