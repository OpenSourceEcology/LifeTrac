# DESIGN-CONTROLLER/hil/w4-07_boot_phy.ps1
#
# W4-07 -- Cold-boot first-frame decode; no bad_header events.
# Bench-only residual: audit-log capture of first 10 s after each cold boot.
# Procedure reference: DESIGN-CONTROLLER/HIL_RUNBOOK.md W4-07.

[CmdletBinding()]
param(
    [Parameter(Mandatory)][string]$Operator,
    [Parameter(Mandatory)][ValidateSet('tractor-first','handheld-first')][string]$BootOrder,
    [int]$RunNumber = 1,
    [ValidateSet('PASS','FAIL','SKIP','ABORT')][string]$Result = 'PASS',
    [int]$BadHeaderCount = -1,
    [double]$FirstFrameSec = -1,
    [string]$AuditCapturePath = '',
    [string]$MqttCapturePath = '',
    [string]$Notes = ''
)

. "$PSScriptRoot/_common.ps1"

Write-GateHeader -GateId 'W4-07' -Title "Cold-boot first-frame decode ($BootOrder)"
Assert-Section0-Ready

Write-Host '== W4-07 specific pre-conditions ==' -ForegroundColor Cyan
Confirm-OperatorChecklist @(
    'Both handheld and tractor fully powered down for >= 30 s',
    'Audit log tail running',
    "mosquitto_sub -t 'lifetrac/audit/bad_header' -v running"
)

Write-Host '== W4-07 procedure ==' -ForegroundColor Cyan
if ($BootOrder -eq 'tractor-first') {
    Write-Host '  1. Power on tractor; wait for lifetrac/m7/state == ready'
    Write-Host '  2. Power on handheld; note t_handheld_on'
} else {
    Write-Host '  1. Power on handheld; note t_handheld_on'
    Write-Host '  2. Power on tractor'
}
Write-Host '  3. Capture first 10 s of audit log + MQTT'
Write-Host '  4. Confirm first lifetrac/handheld/state arrives within 3 s of t_handheld_on'
Write-Host '  5. Confirm 0 bad_header events in 10 s window'
Write-Host ''

if ($BadHeaderCount -lt 0)  { $BadHeaderCount  = [int](Read-OperatorPrompt 'bad_header count in 10 s window') }
if ($FirstFrameSec -lt 0)   { $FirstFrameSec   = [double](Read-OperatorPrompt 'seconds from t_handheld_on to first state frame') }
if (-not $AuditCapturePath) {
    $AuditCapturePath = Read-OperatorPrompt 'audit capture path' -Default "DESIGN-CONTROLLER/bench-evidence/W4-07/W4-07_run-${RunNumber}_audit.jsonl"
}
if (-not $MqttCapturePath) {
    $MqttCapturePath = Read-OperatorPrompt 'mqtt capture path' -Default "DESIGN-CONTROLLER/bench-evidence/W4-07/W4-07_run-${RunNumber}_mqtt.txt"
}
if (-not $Notes) { $Notes = Read-OperatorPrompt 'Operator note' -Default "$BootOrder run $RunNumber" }

if ((($BadHeaderCount -gt 0) -or ($FirstFrameSec -gt 3.0)) -and $Result -eq 'PASS') {
    Write-Warning 'Pass criterion violated; flipping Result to FAIL'
    $Result = 'FAIL'
}

$runId = New-RunId -GateId 'W4-07' -N $RunNumber -Rung $BootOrder
Write-HilResult -GateId 'W4-07' `
    -RunId $runId `
    -Operator $Operator `
    -Result $Result `
    -Metrics @{
        boot_order        = $BootOrder
        bad_header_count  = $BadHeaderCount
        first_frame_sec   = $FirstFrameSec
        threshold_sec     = 3.0
    } `
    -EvidencePaths @($AuditCapturePath, $MqttCapturePath) `
    -Notes $Notes

Write-Host ''
Write-Host 'Gate target: 20 PASS runs (10 per boot order).'
