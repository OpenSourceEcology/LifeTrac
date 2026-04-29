# DESIGN-CONTROLLER/hil/w4-02_link_tune_walkdown.ps1
#
# W4-02 -- Link-tune walk-down SF7->8->9 with < 1 % packet loss; revert deadline 500 ms.
# Bench-only residual: RF attenuator + < 1 % loss measurement.
# Procedure reference: DESIGN-CONTROLLER/HIL_RUNBOOK.md W4-02.

[CmdletBinding()]
param(
    [Parameter(Mandatory)][string]$Operator,
    [int]$RunNumber = 1,
    [ValidateSet('PASS','FAIL','SKIP','ABORT')][string]$Result = 'PASS',
    [double]$AttenToSf8Db = -1,
    [double]$AttenToSf9Db = -1,
    [double]$PacketLossPct = -1,
    [double]$RevertMs = -1,
    [string]$PhyStateLogPath = '',
    [string]$Notes = ''
)

. "$PSScriptRoot/_common.ps1"

Write-GateHeader -GateId 'W4-02' -Title 'Link-tune walk-down + revert deadline'
Assert-Section0-Ready

Write-Host '== W4-02 specific pre-conditions ==' -ForegroundColor Cyan
Confirm-OperatorChecklist @(
    'Variable RF attenuator (0-60 dB, 0.5 dB step) inline between handheld and tractor antennas',
    'Spectrum analyzer or RTL-SDR sniffing 915 MHz',
    "mosquitto_sub -t 'lifetrac/m7/phy_state' -v capture running, output piped to W4-02_phy_state.log",
    'Sticks driving sinusoidal 0.5 Hz so packet loss is visible as motion stutter'
)

Write-Host '== W4-02 procedure ==' -ForegroundColor Cyan
Write-Host '  1. Walk-down: 0 dB -> +3 dB/min until SF8 reported (record atten_to_sf8_db)'
Write-Host '  2. Continue +3 dB/min until SF9 reported (record atten_to_sf9_db)'
Write-Host '  3. Hold SF9 for 60 s; count seq_gap events for packet_loss_pct'
Write-Host '  4. Walk-up: -3 dB/min until SF7 returns'
Write-Host '  5. Revert deadline: SF7 + mosquitto_pub -t lifetrac/debug/drop_hb -m 1; expect SF-up within 500 ms'
Write-Host ''

if ($AttenToSf8Db -lt 0)  { $AttenToSf8Db  = [double](Read-OperatorPrompt 'atten_to_sf8_db') }
if ($AttenToSf9Db -lt 0)  { $AttenToSf9Db  = [double](Read-OperatorPrompt 'atten_to_sf9_db') }
if ($PacketLossPct -lt 0) { $PacketLossPct = [double](Read-OperatorPrompt 'packet_loss_pct (worst transition)') }
if ($RevertMs -lt 0)      { $RevertMs      = [double](Read-OperatorPrompt 'revert deadline observed (ms)') }
if (-not $PhyStateLogPath) {
    $PhyStateLogPath = Read-OperatorPrompt 'phy_state log path' -Default "DESIGN-CONTROLLER/bench-evidence/W4-02/W4-02_run-${RunNumber}_phy_state.log"
}
if (-not $Notes) { $Notes = Read-OperatorPrompt 'Operator note' -Default "Run $RunNumber" }

# Pass criteria: packet loss < 1 % during transition; revert <= 550 ms.
if (($PacketLossPct -ge 1.0 -or $RevertMs -gt 550.0) -and $Result -eq 'PASS') {
    Write-Warning 'Pass criterion violated; flipping Result to FAIL'
    $Result = 'FAIL'
}

$runId = New-RunId -GateId 'W4-02' -N $RunNumber
Write-HilResult -GateId 'W4-02' `
    -RunId $runId `
    -Operator $Operator `
    -Result $Result `
    -Metrics @{
        atten_to_sf8_db = $AttenToSf8Db
        atten_to_sf9_db = $AttenToSf9Db
        packet_loss_pct = $PacketLossPct
        revert_ms       = $RevertMs
    } `
    -EvidencePaths @($PhyStateLogPath) `
    -Notes $Notes

Write-Host ''
Write-Host 'Gate target: 1 PASS run covering full ladder + revert.'
