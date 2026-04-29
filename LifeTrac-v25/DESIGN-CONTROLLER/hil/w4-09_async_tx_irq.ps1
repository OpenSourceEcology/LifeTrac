# DESIGN-CONTROLLER/hil/w4-09_async_tx_irq.ps1
#
# W4-09 -- Async M7 TX state machine: startTransmit() + isTransmitDone() IRQ timing.
# Bench-only residual: real SX1276 IRQ wiring on real H747 silicon -- HARD-BLOCKED, no SIL substitute possible.
# Procedure reference: DESIGN-CONTROLLER/HIL_RUNBOOK.md W4-09.

[CmdletBinding()]
param(
    [Parameter(Mandatory)][string]$Operator,
    [Parameter(Mandatory)][ValidateSet('1','2')][string]$Phase,
    [Parameter(Mandatory)][ValidateSet('7','8','9')][string]$Sf,
    [int]$RunNumber = 1,
    [ValidateSet('PASS','FAIL','SKIP','ABORT')][string]$Result = 'PASS',
    [double]$TimeOnAirObservedMs = -1,
    [double]$TimeOnAirTheoryMs = -1,
    [double]$IrqToFlagP99Ms = -1,
    [double]$AliveTickJitterP99Ms = -1,
    [string]$LogicAnalyzerPath = '',
    [string]$Notes = ''
)

. "$PSScriptRoot/_common.ps1"

Write-GateHeader -GateId 'W4-09' -Title "Async M7 TX state machine (Phase $Phase, SF$Sf)"
Assert-Section0-Ready

Write-Host '== W4-09 specific pre-conditions ==' -ForegroundColor Cyan
Confirm-OperatorChecklist @(
    'Logic analyzer CH5 on SX1276 DIO0 (TxDone IRQ)',
    'Debug-build firmware exposing IP-107 instrumentation timestamps'
)

if ($Phase -eq '1') {
    Write-Host '== Phase 1 procedure (measure) ==' -ForegroundColor Cyan
    Write-Host '  1. Run normal traffic at SF7/SF8/SF9; capture 100 TX cycles per SF'
    Write-Host '  2. Record t_dio0 - t_start (time-on-air; should match LoRa calculator)'
    Write-Host '  3. Record t_isdone - t_dio0 (IRQ -> flag latency, expect < 1 ms)'
    Write-Host ''
    if ($TimeOnAirObservedMs -lt 0) { $TimeOnAirObservedMs = [double](Read-OperatorPrompt 'measured time-on-air (ms)') }
    if ($TimeOnAirTheoryMs -lt 0)   { $TimeOnAirTheoryMs   = [double](Read-OperatorPrompt 'theoretical time-on-air (ms)') }
    if ($IrqToFlagP99Ms -lt 0)      { $IrqToFlagP99Ms      = [double](Read-OperatorPrompt 'IRQ -> flag p99 latency (ms)') }
    $deltaPct = if ($TimeOnAirTheoryMs -gt 0) { [Math]::Abs(($TimeOnAirObservedMs - $TimeOnAirTheoryMs) / $TimeOnAirTheoryMs * 100.0) } else { 999.0 }
    if ((($deltaPct -gt 5.0) -or ($IrqToFlagP99Ms -gt 2.0)) -and $Result -eq 'PASS') {
        Write-Warning "Phase 1 criterion violated (delta=$deltaPct %, irq_p99=$IrqToFlagP99Ms ms); flipping Result to FAIL"
        $Result = 'FAIL'
    }
    $metrics = @{
        phase                  = 1
        sf                     = [int]$Sf
        time_on_air_observed_ms = $TimeOnAirObservedMs
        time_on_air_theory_ms   = $TimeOnAirTheoryMs
        time_on_air_delta_pct   = $deltaPct
        irq_to_flag_p99_ms      = $IrqToFlagP99Ms
    }
} else {
    Write-Host '== Phase 2 procedure (replace hack) ==' -ForegroundColor Cyan
    Write-Host '  1. Land non-blocking TX queue (separate PR; replace refresh_m4_alive_before_tx() calls)'
    Write-Host '  2. Re-run W4-03 and W4-09 Phase 1 -- both must still pass'
    Write-Host '  3. Verify alive_tick_ms keeps ticking within 5 ms even during TX'
    Write-Host ''
    if ($AliveTickJitterP99Ms -lt 0) { $AliveTickJitterP99Ms = [double](Read-OperatorPrompt 'alive_tick_ms jitter p99 (ms)') }
    if (($AliveTickJitterP99Ms -gt 5.0) -and $Result -eq 'PASS') {
        Write-Warning "Phase 2 criterion violated; flipping Result to FAIL"
        $Result = 'FAIL'
    }
    $metrics = @{
        phase                       = 2
        sf                          = [int]$Sf
        alive_tick_jitter_p99_ms    = $AliveTickJitterP99Ms
    }
}

if (-not $LogicAnalyzerPath) {
    $LogicAnalyzerPath = Read-OperatorPrompt 'Logic-analyzer capture path' -Default "DESIGN-CONTROLLER/bench-evidence/W4-09/W4-09_phase${Phase}_run-${RunNumber}_SF${Sf}.sal"
}
if (-not $Notes) { $Notes = Read-OperatorPrompt 'Operator note' -Default "Phase $Phase SF$Sf run $RunNumber" }

$runId = New-RunId -GateId 'W4-09' -N $RunNumber -Rung "P${Phase}_SF${Sf}"
Write-HilResult -GateId 'W4-09' `
    -RunId $runId `
    -Operator $Operator `
    -Result $Result `
    -Metrics $metrics `
    -EvidencePaths @($LogicAnalyzerPath) `
    -Notes $Notes

Write-Host ''
Write-Host 'Gate target: 3 PASS runs (one per SF) for Phase 1; 1 PASS run for Phase 2.'
