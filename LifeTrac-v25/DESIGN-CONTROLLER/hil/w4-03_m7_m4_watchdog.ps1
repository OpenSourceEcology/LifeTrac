# DESIGN-CONTROLLER/hil/w4-03_m7_m4_watchdog.ps1
#
# W4-03 -- M7<->M4 watchdog trip < 200 ms; estop_request = 0xA5A5A5A5 on SRAM4.
# Bench-only residual: SRAM4 capture probe + JTAG halt of M7.
# Procedure reference: DESIGN-CONTROLLER/HIL_RUNBOOK.md W4-03.

[CmdletBinding()]
param(
    [Parameter(Mandatory)][string]$Operator,
    [int]$RunNumber = 1,
    [ValidateSet('PASS','FAIL','SKIP','ABORT')][string]$Result = 'PASS',
    [double]$TripMs = -1,
    [bool]$EstopMagicSeen = $true,
    [string]$ScopeCsvPath = '',
    [string]$Sram4DumpPath = '',
    [string]$Notes = ''
)

. "$PSScriptRoot/_common.ps1"

Write-GateHeader -GateId 'W4-03' -Title 'M7<->M4 watchdog trip + ESTOP magic'
Assert-Section0-Ready

Write-Host '== W4-03 specific pre-conditions ==' -ForegroundColor Cyan
Confirm-OperatorChecklist @(
    'Debug-build firmware on M7 with LIFETRAC_DEBUG_HALT_TRIGGER defined',
    'J-Link / SWD probe attached to H747 debug header',
    'Logic analyzer triggered on PSR-alive falling edge',
    "M7 alive_tick_ms streaming, M4 loop_counter incrementing in MQTT telemetry"
)

Write-Host '== W4-03 procedure ==' -ForegroundColor Cyan
Write-Host '  1. Trigger debug halt (mosquitto_pub -t lifetrac/debug/halt_m7 -m 1) OR J-Link halt M7'
Write-Host '  2. Note t_halt; capture scope: PSR-alive falling edge -> Δt = t_trip - t_halt'
Write-Host '  3. Read SRAM4 via J-Link: confirm estop_request == 0xA5A5A5A5'
Write-Host '  4. Confirm all 8 dummy coils de-energised'
Write-Host '  5. Reset M7; verify PSR-alive returns high only after fresh alive_tick_ms stream'
Write-Host ''

if ($TripMs -lt 0) {
    $TripMs = [double](Read-OperatorPrompt 't_trip - t_halt (ms)')
}
if (-not $ScopeCsvPath) {
    $ScopeCsvPath = Read-OperatorPrompt 'Scope CSV path' -Default "DESIGN-CONTROLLER/bench-evidence/W4-03/W4-03_run-${RunNumber}.csv"
}
if (-not $Sram4DumpPath) {
    $Sram4DumpPath = Read-OperatorPrompt 'SRAM4 dump path' -Default "DESIGN-CONTROLLER/bench-evidence/W4-03/W4-03_run-${RunNumber}_sram4.bin"
}
if (-not $Notes) { $Notes = Read-OperatorPrompt 'Operator note' -Default "Run $RunNumber" }

# Pass criterion: trip < 200 ms AND ESTOP magic confirmed.
if ((($TripMs -ge 200.0) -or (-not $EstopMagicSeen)) -and $Result -eq 'PASS') {
    Write-Warning 'Pass criterion violated; flipping Result to FAIL'
    $Result = 'FAIL'
}

$runId = New-RunId -GateId 'W4-03' -N $RunNumber
Write-HilResult -GateId 'W4-03' `
    -RunId $runId `
    -Operator $Operator `
    -Result $Result `
    -Metrics @{
        trip_ms          = $TripMs
        threshold_ms     = 200.0
        estop_magic_seen = $EstopMagicSeen
    } `
    -EvidencePaths @($ScopeCsvPath, $Sram4DumpPath) `
    -Notes $Notes

Write-Host ''
Write-Host 'Gate target: 10 PASS rows.'
