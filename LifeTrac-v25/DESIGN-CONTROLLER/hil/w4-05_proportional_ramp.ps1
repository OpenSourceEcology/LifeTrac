# DESIGN-CONTROLLER/hil/w4-05_proportional_ramp.ps1
#
# W4-05 -- Proportional valve ramp-out: track 2 s, arm 1 s; coil stays engaged.
# Bench-only residual: A0602 output probe + valve-coil voltage capture.
# Procedure reference: DESIGN-CONTROLLER/HIL_RUNBOOK.md W4-05.

[CmdletBinding()]
param(
    [Parameter(Mandatory)][string]$Operator,
    [Parameter(Mandatory)][ValidateSet('track','arm')][string]$AxisGroup,
    [Parameter(Mandatory)][string]$AxisName,
    [int]$RunNumber = 1,
    [ValidateSet('PASS','FAIL','SKIP','ABORT')][string]$Result = 'PASS',
    [double]$RampSeconds = -1,
    [double]$LinearR2 = -1,
    [bool]$CoilHeldThroughRamp = $true,
    [string]$ScopeCsvPath = '',
    [string]$Notes = ''
)

. "$PSScriptRoot/_common.ps1"

Write-GateHeader -GateId 'W4-05' -Title "Proportional valve ramp-out ($AxisGroup / $AxisName)"
Assert-Section0-Ready

Write-Host '== W4-05 specific pre-conditions ==' -ForegroundColor Cyan
Confirm-OperatorChecklist @(
    'Multimeter or scope on Opta A0602 analog output (0-10 V) for the axis under test',
    "mosquitto_sub -t 'lifetrac/opta/flow_sp' -v capture running"
)

Write-Host '== W4-05 procedure ==' -ForegroundColor Cyan
Write-Host '  1. Drive full deflection 3 s; confirm flow_sp == 10000 mV (~10 V on scope)'
Write-Host '  2. Release stick to neutral; note t_release'
Write-Host '  3. Capture analog output ramp; record t_zero when output crosses 100 mV'
Write-Host '  4. Confirm coil stays high until t_zero +/- 50 ms'
Write-Host ('  5. Repeat 5x per axis. Track target = 2.00 +/- 0.10 s; arm target = 1.00 +/- 0.05 s')
Write-Host ''

if ($RampSeconds -lt 0) { $RampSeconds = [double](Read-OperatorPrompt 'ramp seconds (t_zero - t_release)') }
if ($LinearR2 -lt 0)    { $LinearR2    = [double](Read-OperatorPrompt 'linear R^2 of ramp') }
if (-not $ScopeCsvPath) {
    $ScopeCsvPath = Read-OperatorPrompt 'Scope CSV path' -Default "DESIGN-CONTROLLER/bench-evidence/W4-05/W4-05_run-${RunNumber}_${AxisName}.csv"
}
if (-not $Notes) { $Notes = Read-OperatorPrompt 'Operator note' -Default "$AxisName run $RunNumber" }

if ($AxisGroup -eq 'track') {
    $target = 2.00; $tolerance = 0.10
} else {
    $target = 1.00; $tolerance = 0.05
}
$delta = [Math]::Abs($RampSeconds - $target)
if ((($delta -gt $tolerance) -or ($LinearR2 -lt 0.98) -or (-not $CoilHeldThroughRamp)) -and $Result -eq 'PASS') {
    Write-Warning "Pass criterion violated (delta=$delta, R2=$LinearR2); flipping Result to FAIL"
    $Result = 'FAIL'
}

$runId = New-RunId -GateId 'W4-05' -N $RunNumber -Rung $AxisName
Write-HilResult -GateId 'W4-05' `
    -RunId $runId `
    -Operator $Operator `
    -Result $Result `
    -Metrics @{
        axis_group             = $AxisGroup
        axis_name              = $AxisName
        ramp_seconds           = $RampSeconds
        target_seconds         = $target
        tolerance_seconds      = $tolerance
        linear_r2              = $LinearR2
        coil_held_through_ramp = $CoilHeldThroughRamp
    } `
    -EvidencePaths @($ScopeCsvPath) `
    -Notes $Notes

Write-Host ''
Write-Host 'Gate target: 5 PASS runs per axis (6 axes => 30 runs total).'
