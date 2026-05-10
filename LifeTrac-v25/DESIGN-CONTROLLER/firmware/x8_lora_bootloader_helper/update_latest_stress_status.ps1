param(
    [string]$RepoRoot = "",
    [string]$ReportPath = ""
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

function Get-LatestImportedReport {
    param(
        [Parameter(Mandatory = $true)]
        [string]$AiNotesDir
    )

    $pattern = "*_Method_G_Single_Board_Stress_Report_*_Copilot_v1_0.md"
    $file = Get-ChildItem -LiteralPath $AiNotesDir -Filter $pattern -File |
        Sort-Object LastWriteTime -Descending |
        Select-Object -First 1
    return $file
}

$repo = Resolve-RepoRoot
$aiNotesDir = Join-Path $repo "AI NOTES"
if (-not (Test-Path -LiteralPath $aiNotesDir)) {
    throw "AI NOTES directory not found: $aiNotesDir"
}

$reportFile = $null
if ($ReportPath) {
    if (-not (Test-Path -LiteralPath $ReportPath)) {
        throw "Provided report path does not exist: $ReportPath"
    }
    $reportFile = Get-Item -LiteralPath $ReportPath
} else {
    $reportFile = Get-LatestImportedReport -AiNotesDir $aiNotesDir
    if (-not $reportFile) {
        throw "No imported stress report files found in: $aiNotesDir"
    }
}

$content = Get-Content -LiteralPath $reportFile.FullName -Raw

$recommendation = "UNKNOWN"
if ($content -match "READY_FOR_NEXT_STAGE") {
    $recommendation = "READY_FOR_NEXT_STAGE"
} elseif ($content -match "STAY_IN_SINGLE_BOARD_HARDENING") {
    $recommendation = "STAY_IN_SINGLE_BOARD_HARDENING"
}

$passRate = "unknown"
if ($content -match "\| Pass rate \| ([0-9]+(?:\.[0-9]+)?%) \|") {
    $passRate = $Matches[1]
}

$bannerRate = "unknown"
if ($content -match "\| Banner hit rate \| ([0-9]+(?:\.[0-9]+)?%) \|") {
    $bannerRate = $Matches[1]
}

$tickRate = "unknown"
if ($content -match "\| Tick hit rate \| ([0-9]+(?:\.[0-9]+)?%) \|") {
    $tickRate = $Matches[1]
}

$statusPath = Join-Path $aiNotesDir "LATEST_STRESS_STATUS.md"
$now = Get-Date -Format "yyyy-MM-dd HH:mm:ss K"

$body = @(
    "# Latest Method G Single-Board Stress Status",
    "",
    "- Updated: $now",
    "- Recommendation: $recommendation",
    "- Pass rate: $passRate",
    "- Banner hit rate: $bannerRate",
    "- Tick hit rate: $tickRate",
    "- Source report: $($reportFile.Name)",
    "",
    "## Notes",
    "",
    "- This is a convenience index to the most recent imported stress report.",
    "- Authoritative details remain in the full per-run report file."
) -join "`r`n"

Set-Content -LiteralPath $statusPath -Value $body -Encoding UTF8
Write-Host "Updated latest status file: $statusPath"
