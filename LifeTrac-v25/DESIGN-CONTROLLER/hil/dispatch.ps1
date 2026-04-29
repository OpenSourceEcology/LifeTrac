# DESIGN-CONTROLLER/hil/dispatch.ps1
#
# Wave-4 dispatcher. Reads bench-evidence/W4-XX/results.jsonl files, prints a
# progress table, and recommends the next gate to run based on the per-gate
# PASS-row targets defined in HIL_RUNBOOK.md.
#
# Round 34 / BC-06: when a build config is resolvable (via -ConfigPath or the
# canonical default), gates that the active fleet shape doesn't exercise are
# reported as N/A instead of NOT-STARTED, and the "next recommended" line
# skips them. Applicability rules live in hil/gate_applicability.json so the
# SIL gate (test_hil_dispatch_applicability_sil.py) and this dispatcher share
# one source of truth.

[CmdletBinding()]
param(
    [switch]$Report,        # full report mode (one row per run, not just summary)
    [string]$Gate = '',     # restrict report to a single W4-XX gate
    [string]$ConfigPath = '', # BC-06: path to a build.<unit>.toml; '' uses the canonical default
    [switch]$NoApplicability  # BC-06: skip applicability filtering (legacy behavior)
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
    param(
        [Parameter(Mandatory)][string]$GateId,
        [bool]$Applicable = $true
    )
    $results = Read-GateResults -GateId $GateId
    $pass    = @($results | Where-Object { $_.result -eq 'PASS' }).Count
    $fail    = @($results | Where-Object { $_.result -eq 'FAIL' }).Count
    $abort   = @($results | Where-Object { $_.result -eq 'ABORT' }).Count
    $skip    = @($results | Where-Object { $_.result -eq 'SKIP' }).Count
    $target  = $Script:GateTargets[$GateId].Target
    $closed  = ($pass -ge $target) -and ($fail -eq 0) -and ($abort -eq 0)
    if (-not $Applicable) {
        $status = 'N/A'
    } elseif ($closed) {
        $status = 'CLOSED'
    } elseif ($fail -gt 0 -or $abort -gt 0) {
        $status = 'FAILING'
    } elseif ($pass -eq 0) {
        $status = 'NOT-STARTED'
    } else {
        $status = 'IN-PROGRESS'
    }
    return [pscustomobject]@{
        GateId  = $GateId
        Title   = $Script:GateTargets[$GateId].Title
        Pass    = $pass
        Fail    = $fail
        Abort   = $abort
        Skip    = $skip
        Target  = $target
        Status  = $status
    }
}

# ---------------------------------------------------------------------------
# Round 34 / BC-06: build-config + per-gate applicability.
# ---------------------------------------------------------------------------

function Resolve-LifeTracConfigPath {
    param([string]$ConfigPath)
    if ($ConfigPath) {
        if (-not (Test-Path $ConfigPath)) {
            Write-Warning "ConfigPath '$ConfigPath' does not exist; falling back to default"
        } else {
            return (Resolve-Path $ConfigPath).Path
        }
    }
    $root = Get-LifeTracRepoRoot
    $default = Join-Path $root 'DESIGN-CONTROLLER/base_station/config/build.default.toml'
    if (Test-Path $default) { return $default }
    return $null
}

function Get-LifeTracConfigJson {
    param([string]$ConfigPath)
    $resolved = Resolve-LifeTracConfigPath -ConfigPath $ConfigPath
    if (-not $resolved) {
        Write-Warning 'No build config found; applicability filtering disabled.'
        return $null
    }
    $root = Get-LifeTracRepoRoot
    $tool = Join-Path $root 'tools/lifetrac_config.py'
    if (-not (Test-Path $tool)) {
        Write-Warning "lifetrac_config.py not found at $tool; applicability filtering disabled."
        return $null
    }
    $py = Get-Command python -ErrorAction SilentlyContinue
    if (-not $py) { $py = Get-Command python3 -ErrorAction SilentlyContinue }
    if (-not $py) {
        Write-Warning 'python not on PATH; applicability filtering disabled.'
        return $null
    }
    try {
        $json = & $py.Source $tool 'dump-json' $resolved 2>&1 | Out-String
    } catch {
        Write-Warning "lifetrac-config dump-json failed: $($_.Exception.Message); applicability filtering disabled."
        return $null
    }
    if ($LASTEXITCODE -ne 0) {
        Write-Warning "lifetrac-config dump-json exited $LASTEXITCODE; applicability filtering disabled."
        return $null
    }
    try {
        return ($json.Trim() | ConvertFrom-Json)
    } catch {
        Write-Warning "Failed to parse lifetrac-config dump-json output: $($_.Exception.Message)"
        return $null
    }
}

function Get-LifeTracGateApplicabilityRules {
    $path = Join-Path $PSScriptRoot 'gate_applicability.json'
    if (-not (Test-Path $path)) {
        Write-Warning "gate_applicability.json not found at $path; treating all gates as applicable."
        return $null
    }
    return (Get-Content -Path $path -Encoding UTF8 -Raw | ConvertFrom-Json)
}

function Resolve-DottedPath {
    param([Parameter(Mandatory)]$Object, [Parameter(Mandatory)][string]$Path)
    $node = $Object
    foreach ($segment in $Path.Split('.')) {
        if ($null -eq $node) { return $null }
        $node = $node.$segment
    }
    return $node
}

function Test-GateApplicable {
    param(
        [Parameter(Mandatory)][string]$GateId,
        $Rules,
        $ConfigPayload
    )
    if ($null -eq $Rules -or $null -eq $ConfigPayload) { return $true }
    $gate = $Rules.gates.$GateId
    if ($null -eq $gate) { return $true }
    if ($null -eq $gate.applies_when -or $gate.applies_when.Count -eq 0) { return $true }
    foreach ($pred in $gate.applies_when) {
        $value = Resolve-DottedPath -Object $ConfigPayload.config -Path $pred.path
        switch ($pred.op) {
            'eq'     { if ($value -ne $pred.value) { return $false } }
            'gt'     { if (-not ($value -gt $pred.value)) { return $false } }
            'gte'    { if (-not ($value -ge $pred.value)) { return $false } }
            'truthy' { if (-not $value) { return $false } }
            default {
                Write-Warning "Unknown applicability op '$($pred.op)' for $GateId; treating as not-applicable."
                return $false
            }
        }
    }
    return $true
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
$rules = $null
$configPayload = $null
if (-not $NoApplicability) {
    $rules = Get-LifeTracGateApplicabilityRules
    $configPayload = Get-LifeTracConfigJson -ConfigPath $ConfigPath
    if ($configPayload) {
        Write-Host (' Applicability source: {0} (sha256={1})' -f $configPayload.unit_id, $configPayload.config_sha256.Substring(0,8)) -ForegroundColor DarkGray
        Write-Host ''
    }
}
foreach ($gid in $Script:GateTargets.Keys) {
    if ($Gate -and $gid -ne $Gate) { continue }
    $applicable = Test-GateApplicable -GateId $gid -Rules $rules -ConfigPayload $configPayload
    $rows += Get-GateStatus -GateId $gid -Applicable:$applicable
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
$next = $rows | Where-Object { $_.Status -ne 'CLOSED' -and $_.Status -ne 'N/A' } | Select-Object -First 1
Write-Host ''
if ($null -eq $next) {
    $naCount = @($rows | Where-Object { $_.Status -eq 'N/A' }).Count
    if ($naCount -gt 0) {
        Write-Host ("All applicable Wave-4 gates CLOSED ({0} N/A for this build). Time to flip the CI compile-gate from continue-on-error: true to blocking and update SAFETY_CASE.md." -f $naCount) -ForegroundColor Green
    } else {
        Write-Host 'All Wave-4 gates CLOSED. Time to flip the CI compile-gate from continue-on-error: true to blocking and update SAFETY_CASE.md.' -ForegroundColor Green
    }
} else {
    Write-Host ('Next recommended gate: {0} -- {1} (status: {2}, {3}/{4} PASS)' -f $next.GateId, $next.Title, $next.Status, $next.Pass, $next.Target) -ForegroundColor Cyan
    Write-Host ('Run:  pwsh ./{0}_*.ps1 -Operator <name>' -f $next.GateId.ToLower())
}
Write-Host ''
