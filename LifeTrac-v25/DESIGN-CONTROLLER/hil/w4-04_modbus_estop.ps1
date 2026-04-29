# DESIGN-CONTROLLER/hil/w4-04_modbus_estop.ps1
#
# W4-04 -- Modbus failure -> E-stop within 1 s; IP-205 counter + 10-failure latch.
# Bench-only residual: physical RS-485 cable disconnect mid-run.
# Procedure reference: DESIGN-CONTROLLER/HIL_RUNBOOK.md W4-04.

[CmdletBinding()]
param(
    [Parameter(Mandatory)][string]$Operator,
    [int]$RunNumber = 1,
    [ValidateSet('PASS','FAIL','SKIP','ABORT')][string]$Result = 'PASS',
    [double]$ApplyNeg1Ms = -1,
    [bool]$AuditEntryPresent = $true,
    [bool]$NoMotionWithoutRelease = $true,
    [string]$ModbusStatsLogPath = '',
    [string]$AuditCapturePath = '',
    [string]$Notes = ''
)

. "$PSScriptRoot/_common.ps1"

Write-GateHeader -GateId 'W4-04' -Title 'Modbus disconnect -> E-stop'
Assert-Section0-Ready

Write-Host '== W4-04 specific pre-conditions ==' -ForegroundColor Cyan
Confirm-OperatorChecklist @(
    'RS-485 cable terminated in a quick-disconnect (Phoenix or XLR-style)',
    'Track sticks driving slow ramp so coil state is observable',
    "mosquitto_sub -t 'lifetrac/m7/modbus_stats' -v capture running",
    'modbus_stats.failures == 0, modbus_stats.ok ticking'
)

Write-Host '== W4-04 procedure ==' -ForegroundColor Cyan
Write-Host '  1. Disconnect RS-485 cable; note t_disconnect'
Write-Host '  2. Watch failures increment; record t_first_fail and t_apply_-1 (10 consecutive failures)'
Write-Host '  3. Verify all 8 dummy coils de-energise at or before t_apply_-1'
Write-Host '  4. Verify audit log contains modbus_disconnect event'
Write-Host '  5. Reconnect; verify M7 only resumes after 5 ok + stick-release'
Write-Host ''

if ($ApplyNeg1Ms -lt 0) {
    $ApplyNeg1Ms = [double](Read-OperatorPrompt 't_apply_-1 - t_disconnect (ms)')
}
if (-not $ModbusStatsLogPath) {
    $ModbusStatsLogPath = Read-OperatorPrompt 'modbus_stats log path' -Default "DESIGN-CONTROLLER/bench-evidence/W4-04/W4-04_run-${RunNumber}_modbus_stats.log"
}
if (-not $AuditCapturePath) {
    $AuditCapturePath = Read-OperatorPrompt 'audit-log capture path' -Default "DESIGN-CONTROLLER/bench-evidence/W4-04/W4-04_run-${RunNumber}_audit.jsonl"
}
if (-not $Notes) { $Notes = Read-OperatorPrompt 'Operator note' -Default "Run $RunNumber" }

if ((($ApplyNeg1Ms -ge 1000.0) -or (-not $AuditEntryPresent) -or (-not $NoMotionWithoutRelease)) -and $Result -eq 'PASS') {
    Write-Warning 'Pass criterion violated; flipping Result to FAIL'
    $Result = 'FAIL'
}

$runId = New-RunId -GateId 'W4-04' -N $RunNumber
Write-HilResult -GateId 'W4-04' `
    -RunId $runId `
    -Operator $Operator `
    -Result $Result `
    -Metrics @{
        apply_neg1_ms             = $ApplyNeg1Ms
        threshold_ms              = 1000.0
        audit_entry_present       = $AuditEntryPresent
        no_motion_without_release = $NoMotionWithoutRelease
    } `
    -EvidencePaths @($ModbusStatsLogPath, $AuditCapturePath) `
    -Notes $Notes

Write-Host ''
Write-Host 'Gate target: 10 PASS rows.'
