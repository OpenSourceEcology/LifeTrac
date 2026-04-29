# DESIGN-CONTROLLER/hil/dispatch.ps1
#
# Wave-4 dispatcher. Reads bench-evidence/W4-XX/results.jsonl files, prints a
# progress table, and recommends the next gate to run based on the per-gate
# PASS-row targets defined in HIL_RUNBOOK.md.

[CmdletBinding()]
param(
    [switch]$Report,        # full report mode (one row per run, not just summary)
    [string]$Gate = ''      # restrict report to a single W4-XX gate
)

. "$PSScriptRoot/_common.ps1"

# Per-gate PASS-row targets. Source of truth: HIL_RUNBOOK.md.
$Script:GateTargets = [ordered]@{
    'W4-01' = @{ Target = 300; Title = 'Handheld E-stop latch (100 per SF rung)' }
    'W4-02' = @{ Target = 1;   Title = 'Link-tune walk-down + revert' }
    'W4-03' = @{ Target = 10;  Title = 'M7<->M4 watchdog trip' }
    'W4-04' = @{ Target = 10;  Title = 'Modbus disconnect -> E-stop' }
    'W4-05' = @{ Target = 30;  Title = 'Proportional ramp-out (5 per axis x 6 axes)' }
    'W4-06' = @{ Target = 10;  Title = 'Mixed-mode skip (5 per axis pair)' }
    'W4-07' = @{ Target = 20;  Title = 'Boot-PHY first-frame (10 per boot order)' }
    'W4-08' = @{ Target = 30;  Title = 'Camera keyframe round-trip' }
    'W4-09' = @{ Target = 4;   Title = 'Async M7 TX IRQ (3 SF in Phase 1 + 1 Phase 2)' }
    'W4-10' = @{ Target = 5;   Title = 'Fleet-key provisioning (5 steps)' }
}

function Read-GateResults {
    param([Parameter(Mandatory)][string]$GateId)
    $root = Get-LifeTracRepoRoot
    $file = Join-Path $root "DESIGN-CONTROLLER/bench-evidence/$GateId/results.jsonl"
    if (-not (Test-Path $file)) { return @() }
    $lines = Get-Content -Path $file -Encoding UTF8 | Where-Object { $_.Trim().Length -gt 0 }
    $results = @()
    foreach ($line in $lines) {
        try {
            $results += ($line | ConvertFrom-Json)
        } catch {
            Write-Warning "Skipping malformed JSONL line in $file"
        }
    }
    return $results
}

function Get-GateStatus {
    param([Parameter(Mandatory)][string]$GateId)
    $results = Read-GateResults -GateId $GateId
    $pass    = @($results | Where-Object { $_.result -eq 'PASS' }).Count
    $fail    = @($results | Where-Object { $_.result -eq 'FAIL' }).Count
    $abort   = @($results | Where-Object { $_.result -eq 'ABORT' }).Count
    $skip    = @($results | Where-Object { $_.result -eq 'SKIP' }).Count
    $target  = $Script:GateTargets[$GateId].Target
    $closed  = ($pass -ge $target) -and ($fail -eq 0) -and ($abort -eq 0)
    return [pscustomobject]@{
        GateId  = $GateId
        Title   = $Script:GateTargets[$GateId].Title
        Pass    = $pass
        Fail    = $fail
        Abort   = $abort
        Skip    = $skip
        Target  = $target
        Status  = if ($closed) { 'CLOSED' } elseif ($fail -gt 0 -or $abort -gt 0) { 'FAILING' } elseif ($pass -eq 0) { 'NOT-STARTED' } else { 'IN-PROGRESS' }
    }
}

# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------
Write-Host ''
Write-Host '========================================================================' -ForegroundColor Yellow
Write-Host ' LifeTrac Wave-4 HIL dispatcher' -ForegroundColor Yellow
Write-Host '========================================================================' -ForegroundColor Yellow
Write-Host ''

$rows = @()
foreach ($gid in $Script:GateTargets.Keys) {
    if ($Gate -and $gid -ne $Gate) { continue }
    $rows += Get-GateStatus -GateId $gid
}

$rows | Format-Table -AutoSize -Property GateId, Status, @{Name='PASS/Target';Expression={('{0}/{1}' -f $_.Pass,$_.Target)}}, Fail, Abort, Skip, Title

if ($Report) {
    foreach ($row in $rows) {
        Write-Host ''
        Write-Host ('-- {0} runs --' -f $row.GateId) -ForegroundColor Cyan
        $results = Read-GateResults -GateId $row.GateId
        if ($results.Count -eq 0) {
            Write-Host '  (no runs logged)'
        } else {
            $results | Format-Table -AutoSize -Property timestamp, run_id, operator, result
        }
    }
}

# ---------------------------------------------------------------------------
# Recommend next gate.
# ---------------------------------------------------------------------------
$next = $rows | Where-Object { $_.Status -ne 'CLOSED' } | Select-Object -First 1
Write-Host ''
if ($null -eq $next) {
    Write-Host 'All Wave-4 gates CLOSED. Time to flip the CI compile-gate from continue-on-error: true to blocking and update SAFETY_CASE.md.' -ForegroundColor Green
} else {
    Write-Host ('Next recommended gate: {0} -- {1} (status: {2}, {3}/{4} PASS)' -f $next.GateId, $next.Title, $next.Status, $next.Pass, $next.Target) -ForegroundColor Cyan
    Write-Host ('Run:  pwsh ./{0}_*.ps1 -Operator <name>' -f $next.GateId.ToLower())
}
Write-Host ''
