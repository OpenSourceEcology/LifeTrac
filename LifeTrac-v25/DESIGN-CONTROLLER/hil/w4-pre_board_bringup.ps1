# DESIGN-CONTROLLER/hil/w4-pre_board_bringup.ps1
#
# W4-pre -- Board bring-up sanity (no RF).
# Gate: prove both Portenta + Max Carrier stacks enumerate, echo over USB,
#       have healthy rails, pass the dual-core handshake, and boot firmware
#       without keying the SX1276 PA.
# Procedure reference: DESIGN-CONTROLLER/HIL_RUNBOOK.md W4-pre.

[CmdletBinding()]
param(
    [Parameter(Mandatory)][string]$Operator,
    [Parameter(Mandatory)][ValidateSet('usb','echo','rails','dual-core','firmware-boot')][string]$SubGate,
    [int]$RunNumber = 1,
    [ValidateSet('PASS','FAIL','SKIP','ABORT')][string]$Result = 'PASS',
    [int]$UsbReplugPasses = -1,
    [double]$EchoRoundTripMs = -1.0,
    [double]$Rail3v3 = -1.0,
    [double]$Rail5v0 = -1.0,
    [int]$M4CounterGaps = -1,
    [double]$BootHeartbeatSec = -1.0,
    [string]$EvidencePath = '',
    [string]$Notes = ''
)

. "$PSScriptRoot/_common.ps1"

Write-GateHeader -GateId 'W4-pre' -Title "Board bring-up sanity ($SubGate)"
Assert-FrontGateReady -GateId 'W4-pre'

Write-Host '== W4-pre procedure ==' -ForegroundColor Cyan
Write-Host '  usb           : re-plug each carrier 10x; confirm stable distinct USB-CDC ports'
Write-Host '  echo          : flash blink + SerialEcho; confirm 100 random bytes echo < 50 ms'
Write-Host '  rails         : measure Max Carrier 3V3 and 5V0 rails under no-RF load'
Write-Host '  dual-core     : run PortentaDualCore; confirm M4 counter for 60 s with zero gaps'
Write-Host '  firmware-boot : boot tractor_h7 with radio disabled/stubbed; heartbeat within 5 s'
Write-Host '  Reference: HIL_RUNBOOK.md W4-pre'
Write-Host ''

$metrics = @{
    sub_gate = $SubGate
}

switch ($SubGate) {
    'usb' {
        if ($UsbReplugPasses -lt 0) { $UsbReplugPasses = [int](Read-OperatorPrompt 'successful USB re-plug cycles across both carriers (target 20)') }
        $metrics.usb_replug_passes = $UsbReplugPasses
        $metrics.target_replug_passes = 20
        if (($UsbReplugPasses -lt 20) -and $Result -eq 'PASS') {
            Write-Warning 'USB re-plug target not met; flipping Result to FAIL'
            $Result = 'FAIL'
        }
    }
    'echo' {
        if ($EchoRoundTripMs -lt 0) { $EchoRoundTripMs = [double](Read-OperatorPrompt 'worst USB echo round-trip latency_ms (target < 50)') }
        $metrics.echo_round_trip_ms = $EchoRoundTripMs
        $metrics.threshold_ms = 50.0
        if (($EchoRoundTripMs -ge 50.0) -and $Result -eq 'PASS') {
            Write-Warning 'USB echo threshold exceeded; flipping Result to FAIL'
            $Result = 'FAIL'
        }
    }
    'rails' {
        if ($Rail3v3 -lt 0) { $Rail3v3 = [double](Read-OperatorPrompt 'measured 3V3 rail volts') }
        if ($Rail5v0 -lt 0) { $Rail5v0 = [double](Read-OperatorPrompt 'measured 5V0 rail volts') }
        $metrics.rail_3v3 = $Rail3v3
        $metrics.rail_5v0 = $Rail5v0
        if ((($Rail3v3 -lt 3.135) -or ($Rail3v3 -gt 3.465) -or ($Rail5v0 -lt 4.75) -or ($Rail5v0 -gt 5.25)) -and $Result -eq 'PASS') {
            Write-Warning 'Rail measurement outside +/-5%; flipping Result to FAIL'
            $Result = 'FAIL'
        }
    }
    'dual-core' {
        if ($M4CounterGaps -lt 0) { $M4CounterGaps = [int](Read-OperatorPrompt 'M4 counter gaps during 60 s run (target 0)') }
        $metrics.m4_counter_gaps = $M4CounterGaps
        if (($M4CounterGaps -ne 0) -and $Result -eq 'PASS') {
            Write-Warning 'M4 counter gap observed; flipping Result to FAIL'
            $Result = 'FAIL'
        }
    }
    'firmware-boot' {
        if ($BootHeartbeatSec -lt 0) { $BootHeartbeatSec = [double](Read-OperatorPrompt 'seconds from reset to steady heartbeat (target <= 5)') }
        $metrics.boot_heartbeat_sec = $BootHeartbeatSec
        $metrics.threshold_sec = 5.0
        if (($BootHeartbeatSec -gt 5.0) -and $Result -eq 'PASS') {
            Write-Warning 'Firmware heartbeat target exceeded; flipping Result to FAIL'
            $Result = 'FAIL'
        }
    }
}

if (-not $EvidencePath) {
    $EvidencePath = Read-OperatorPrompt 'evidence path (workspace-relative)' -Default "DESIGN-CONTROLLER/bench-evidence/W4-pre/W4-pre_run-${RunNumber}_${SubGate}.log"
}
if (-not $Notes) {
    $Notes = Read-OperatorPrompt 'Operator note' -Default "W4-pre $SubGate run $RunNumber"
}

$runId = New-RunId -GateId 'W4-pre' -N $RunNumber -Rung $SubGate
Write-HilResult -GateId 'W4-pre' `
    -RunId $runId `
    -Operator $Operator `
    -Result $Result `
    -Metrics $metrics `
    -EvidencePaths @($EvidencePath) `
    -Notes $Notes

Write-Host ''
Write-Host 'Gate target: 5 PASS rows (usb, echo, rails, dual-core, firmware-boot).'
