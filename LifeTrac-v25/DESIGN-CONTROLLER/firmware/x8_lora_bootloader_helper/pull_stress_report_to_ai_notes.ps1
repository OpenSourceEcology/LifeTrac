param(
    [string]$AdbSerial = "",
    [string]$RunDir = "",
    [string]$RepoRoot = "",
    [string]$AiNotesRelPath = "LifeTrac-v25/AI NOTES"
)

Set-StrictMode -Version Latest
$ErrorActionPreference = "Stop"
$ScriptRoot = if ($PSScriptRoot) { $PSScriptRoot } elseif ($PSCommandPath) { Split-Path -Parent $PSCommandPath } else { (Get-Location).Path }

function Get-RepoRoot {
    if ($RepoRoot -and (Test-Path -LiteralPath $RepoRoot)) {
        return (Resolve-Path -LiteralPath $RepoRoot).Path
    }

    # Assume script lives in:
    # LifeTrac-v25/DESIGN-CONTROLLER/firmware/x8_lora_bootloader_helper/
    $candidate = Resolve-Path (Join-Path $ScriptRoot "../../../")
    return $candidate.Path
}

$repo = Get-RepoRoot
$aiNotesDir = Join-Path $repo "AI NOTES"
if (-not (Test-Path -LiteralPath $aiNotesDir)) {
    throw "AI NOTES directory not found: $aiNotesDir"
}

$adbArgsPrefix = @()
if ($AdbSerial) {
    $adbArgsPrefix = @("-s", $AdbSerial)
}

$remoteSummarize = "/tmp/lifetrac_p0c/summarize_single_board_stress.sh"
$remoteCmd = "echo fio | sudo -S -p '' bash $remoteSummarize"
if ($RunDir) {
    $remoteCmd += " $RunDir"
}

Write-Host "Running summary generation on X8..."
# Normalize line endings in RunDir before passing to bash to avoid $'\r' interpretation errors
$cleanRunDir = if ($RunDir) { $RunDir -replace "`r`n", "`n" -replace "`r", "" } else { "" }
$remoteCmd = "echo fio | sudo -S -p '' bash $remoteSummarize"
if ($cleanRunDir) {
    $remoteCmd += " `"$cleanRunDir`""
}
$summaryOutput = & adb @adbArgsPrefix exec-out $remoteCmd 2>&1
if ($LASTEXITCODE -ne 0) {
    throw "Failed to generate remote summary. adb exit code: $LASTEXITCODE`nOutput:`n$summaryOutput"
}

$summaryText = ($summaryOutput | Out-String)
$reportMatch = $summaryText | Select-String -Pattern "(?m)^report=.*$" -AllMatches | Select-Object -Last 1
if (-not $reportMatch) {
    throw "Could not locate report path in summarize output.`nOutput:`n$summaryText"
}

$reportLine = $reportMatch.ToString().Trim()
$remoteReport = $reportLine -replace "^report=", ""
$remoteReport = $remoteReport.Trim()
if (-not $remoteReport) {
    throw "Parsed report path is empty."
}

# Extract run timestamp folder name for filename uniqueness.
$runStamp = "unknown"
if ($remoteReport -match "/stress_runs/([^/]+)/report\.md$") {
    $runStamp = $Matches[1]
}

$datePart = Get-Date -Format "yyyy-MM-dd"
$destFileName = "${datePart}_Method_G_Single_Board_Stress_Report_${runStamp}_Copilot_v1_0.md"
$destPath = Join-Path $aiNotesDir $destFileName

$tempPath = Join-Path $env:TEMP "lifetrac_stress_report_${runStamp}.md"
if (Test-Path -LiteralPath $tempPath) {
    Remove-Item -LiteralPath $tempPath -Force
}

Write-Host "Pulling report from X8: $remoteReport"
& adb @adbArgsPrefix pull $remoteReport $tempPath | Out-Host
if ($LASTEXITCODE -ne 0) {
    throw "Failed to pull report from X8. adb exit code: $LASTEXITCODE"
}

$header = @(
    "# Method G Single-Board Stress Report (Imported)",
    "",
    "- Imported: $(Get-Date -Format 'yyyy-MM-dd HH:mm:ss')",
    "- Source device report: $remoteReport",
    "- Source run stamp: $runStamp",
    ""
) -join "`r`n"

$body = Get-Content -LiteralPath $tempPath -Raw
$combined = $header + $body
Set-Content -LiteralPath $destPath -Value $combined -Encoding UTF8

Write-Host "Imported report to: $destPath"

$updater = Join-Path $ScriptRoot "update_latest_stress_status.ps1"
if (Test-Path -LiteralPath $updater) {
    & powershell -ExecutionPolicy Bypass -File $updater -RepoRoot $repo -ReportPath $destPath
    if ($LASTEXITCODE -ne 0) {
        throw "Imported report, but failed to update latest status index."
    }
} else {
    Write-Warning "Latest-status updater not found at: $updater"
}
