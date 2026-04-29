# DESIGN-CONTROLLER/hil/w4-06_mixed_mode_skip.ps1
#
# W4-06 -- Mixed-mode skip: sibling-active release drops coil < 50 ms (no ramp).
# Bench-only residual: same probe rig as W4-05.
# Procedure reference: DESIGN-CONTROLLER/HIL_RUNBOOK.md W4-06.

[CmdletBinding()]
param(
    [Parameter(Mandatory)][string]$Operator,
    [Parameter(Mandatory)][ValidateSet('track','arm')][string]$AxisPair,
    [int]$RunNumber = 1,
    [ValidateSet('PASS','FAIL','SKIP','ABORT')][string]$Result = 'PASS',
    [double]$DropMs = -1,
    [bool]$SiblingRampedNormally = $true,
    [string]$ScopeCsvPath = '',
    [string]$Notes = ''
)

. "$PSScriptRoot/_common.ps1"

Write-GateHeader -GateId 'W4-06' -Title "Mixed-mode skip ($AxisPair pair)"
Assert-Section0-Ready

Write-Host '== W4-06 specific pre-conditions ==' -ForegroundColor Cyan
Confirm-OperatorChecklist @(
    'Scope on two coils (one per axis pair)'
)

Write-Host '== W4-06 procedure ==' -ForegroundColor Cyan
Write-Host '  1. Drive both axes full; confirm both coils energised'
Write-Host '  2. Release left stick only; note t_release_left'
Write-Host '  3. Left coil should drop within 50 ms (sibling active triggers mixed-mode skip)'
Write-Host '  4. Release right stick; right coil should ramp normally (W4-05 behaviour)'
Write-Host ''

if ($DropMs -lt 0) { $DropMs = [double](Read-OperatorPrompt 'drop_ms (released-while-sibling-active)') }
if (-not $ScopeCsvPath) {
    $ScopeCsvPath = Read-OperatorPrompt 'Scope CSV path' -Default "DESIGN-CONTROLLER/bench-evidence/W4-06/W4-06_run-${RunNumber}_${AxisPair}.csv"
}
if (-not $Notes) { $Notes = Read-OperatorPrompt 'Operator note' -Default "$AxisPair run $RunNumber" }

if ((($DropMs -ge 50.0) -or (-not $SiblingRampedNormally)) -and $Result -eq 'PASS') {
    Write-Warning 'Pass criterion violated; flipping Result to FAIL'
    $Result = 'FAIL'
}

$runId = New-RunId -GateId 'W4-06' -N $RunNumber -Rung $AxisPair
Write-HilResult -GateId 'W4-06' `
    -RunId $runId `
    -Operator $Operator `
    -Result $Result `
    -Metrics @{
        axis_pair                = $AxisPair
        drop_ms                  = $DropMs
        threshold_ms             = 50.0
        sibling_ramped_normally  = $SiblingRampedNormally
    } `
    -EvidencePaths @($ScopeCsvPath) `
    -Notes $Notes

Write-Host ''
Write-Host 'Gate target: 10 PASS runs (5 per axis pair).'
