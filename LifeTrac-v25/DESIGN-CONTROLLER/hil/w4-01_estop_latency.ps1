# DESIGN-CONTROLLER/hil/w4-01_estop_latency.ps1
#
# W4-01 -- Handheld E-stop latch latency
# Gate: PSR-alive falling edge -> all 8 valve coils de-energise within < 100 ms,
#       100x per SF rung (SF7/SF8/SF9). p99 < 80 ms.
# Bench-only residual: real PSR-alive falling edge -> relay terminal de-energise time.
#
# Procedure reference: DESIGN-CONTROLLER/HIL_RUNBOOK.md W4-01.

[CmdletBinding()]
param(
    [Parameter(Mandatory)][string]$Operator,
    [Parameter(Mandatory)][ValidateSet('7','8','9')][string]$Sf,
    [int]$RunNumber = 1,
    [ValidateSet('PASS','FAIL','SKIP','ABORT')][string]$Result = 'PASS',
    [double]$LatencyMs = -1.0,
    [string]$ScopeCsvPath = '',
    [string]$Notes = ''
)

. "$PSScriptRoot/_common.ps1"

Write-GateHeader -GateId 'W4-01' -Title "Handheld E-stop latch latency (SF$Sf)"
Assert-Section0-Ready

Write-Host '== W4-01 specific pre-conditions =='   -ForegroundColor Cyan
Confirm-OperatorChecklist @(
    'Scope CH1 = PSR-alive, CH2 = relay coil 1 terminal',
    'Trigger: CH1 falling edge, threshold 1.5 V, holdoff 500 ms',
    'Time base 20 ms/div, 200 ms total window pre-trigger 50 ms',
    "SF rung set to SF$Sf (mosquitto_pub -t lifetrac/cmd/link_tune -m SF$Sf)",
    'Sticks driving full-deflection so all eight coils energised',
    "mosquitto_sub shows lifetrac/handheld/state at >= 8 Hz"
)

Write-Host '== W4-01 procedure (per scope capture) ==' -ForegroundColor Cyan
Write-Host '  1. Drive sticks to full deflection; verify all 8 coils >= 11.5 V'
Write-Host '  2. Press mushroom button firmly; hold until scope captures'
Write-Host '  3. Capture: PSR-alive falling edge -> CH2 falling edge'
Write-Host '  4. Save scope CSV per HIL_RUNBOOK.md section 0.3'
Write-Host '  5. Reset E-stop (twist-release), wait 5 s for re-pair'
Write-Host '  6. Repeat until 100 captures stored for SF' $Sf
Write-Host ''

if ($LatencyMs -lt 0) {
    $LatencyMs = [double](Read-OperatorPrompt 'Measured latency_ms (PSR-alive -> coil de-energise)')
}
if (-not $ScopeCsvPath) {
    $ScopeCsvPath = Read-OperatorPrompt 'Scope CSV path (workspace-relative)' -Default "DESIGN-CONTROLLER/bench-evidence/W4-01/W4-01_run-${RunNumber}_SF${Sf}.csv"
}
if (-not $Notes) {
    $Notes = Read-OperatorPrompt 'Operator note (one paragraph)' -Default "SF$Sf, run $RunNumber"
}

# Per-run pass criterion: latency_ms < 100. Auto-flip Result to FAIL if exceeded.
if ($LatencyMs -ge 100.0 -and $Result -eq 'PASS') {
    Write-Warning "latency_ms = $LatencyMs >= 100; flipping Result to FAIL"
    $Result = 'FAIL'
}

$runId = New-RunId -GateId 'W4-01' -N $RunNumber -Rung "SF$Sf"
Write-HilResult -GateId 'W4-01' `
    -RunId $runId `
    -Operator $Operator `
    -Result $Result `
    -Metrics @{ sf = [int]$Sf; latency_ms = $LatencyMs; threshold_ms = 100.0 } `
    -EvidencePaths @($ScopeCsvPath) `
    -Notes $Notes

Write-Host ''
Write-Host 'Gate target: 100 PASS rows per SF rung (SF7, SF8, SF9). Run dispatch.ps1 for progress.'
