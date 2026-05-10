param(
    [Parameter(Mandatory=$true)]
    [string]$SummaryPath,

    [int]$ExpectedCycles = 0
)

Set-StrictMode -Version Latest
$ErrorActionPreference = "Stop"

if (-not (Test-Path -LiteralPath $SummaryPath)) {
    Write-Error "Summary file not found: $SummaryPath"
    exit 2
}

$kv = @{}
Get-Content -LiteralPath $SummaryPath | ForEach-Object {
    if ($_ -match '^\s*#') { return }
    $line = $_.Trim()
    if (-not $line) { return }
    $parts = $line -split '=', 2
    if ($parts.Count -eq 2) {
        $k = $parts[0].Trim()
        $v = $parts[1].Trim()
        if ($k) { $kv[$k] = $v }
    }
}

function Get-IntOrDefault {
    param(
        [hashtable]$Map,
        [string]$Key,
        [int]$DefaultValue
    )

    if (-not $Map.ContainsKey($Key)) {
        return $DefaultValue
    }

    $value = 0
    if ([int]::TryParse($Map[$Key], [ref]$value)) {
        return $value
    }

    return $DefaultValue
}

$cycles = Get-IntOrDefault -Map $kv -Key "CYCLES" -DefaultValue -1
$launcherFail = Get-IntOrDefault -Map $kv -Key "LAUNCHER_FAIL_COUNT" -DefaultValue -1
$timeoutCount = Get-IntOrDefault -Map $kv -Key "TIMEOUT_COUNT" -DefaultValue 0
$passCount = Get-IntOrDefault -Map $kv -Key "FINAL_RESULT_PASS" -DefaultValue 0

if ($ExpectedCycles -gt 0) {
    $cycles = $ExpectedCycles
}

$errors = @()

if ($cycles -lt 1) {
    $errors += "Missing or invalid CYCLES in summary"
}

if ($passCount -ne $cycles) {
    $errors += "FINAL_RESULT_PASS ($passCount) does not match CYCLES ($cycles)"
}

if ($launcherFail -ne 0) {
    $errors += "LAUNCHER_FAIL_COUNT is $launcherFail (expected 0)"
}

if ($timeoutCount -ne 0) {
    $errors += "TIMEOUT_COUNT is $timeoutCount (expected 0)"
}

Write-Host "Gate input: $SummaryPath"
Write-Host "CYCLES=$cycles"
Write-Host "FINAL_RESULT_PASS=$passCount"
Write-Host "LAUNCHER_FAIL_COUNT=$launcherFail"
Write-Host "TIMEOUT_COUNT=$timeoutCount"

if ($errors.Count -gt 0) {
    Write-Host "GATE_RESULT=FAIL"
    $errors | ForEach-Object { Write-Host "- $_" }
    exit 1
}

Write-Host "GATE_RESULT=PASS"
exit 0
