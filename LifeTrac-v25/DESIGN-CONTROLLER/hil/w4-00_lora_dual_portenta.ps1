# DESIGN-CONTROLLER/hil/w4-00_lora_dual_portenta.ps1
#
# W4-00 -- LoRa stack dual-Portenta bench.
# Gate: prove radio + framing + AES-GCM + replay-window behavior on real
#       SX1276 silicon with both Max Carriers plugged into this workstation.
# Procedure reference: DESIGN-CONTROLLER/HIL_RUNBOOK.md W4-00.

[CmdletBinding()]
param(
    [Parameter(Mandatory)][string]$Operator,
    [Parameter(Mandatory)][ValidateSet('txrx','latency','payload-size','keyframe')][string]$SubGate,
    [ValidateSet('7','8','9')][string]$Sf = '7',
    [int]$RunNumber = 1,
    [ValidateSet('PASS','FAIL','SKIP','ABORT')][string]$Result = 'PASS',
    [int]$FramesSent = 1000,
    [int]$LostFrames = -1,
    [double]$P50Ms = -1.0,
    [double]$P99Ms = -1.0,
    [double]$TheoryRoundTripMs = -1.0,
    [int]$AirBytesDelta = 999,
    [ValidateSet('yes','no','unknown')][string]$KeyframeCrcOk = 'unknown',
    [string]$EvidencePath = '',
    [string]$Notes = ''
)

. "$PSScriptRoot/_common.ps1"

Write-GateHeader -GateId 'W4-00' -Title "LoRa stack dual-Portenta bench ($SubGate)"
Assert-FrontGateReady -GateId 'W4-00'

Write-Host '== W4-00 procedure ==' -ForegroundColor Cyan
Write-Host '  txrx         : send 1000 ControlFrame burst at 1 m; diff TX seq vs RX seq'
Write-Host '  latency      : run 100 round-trips per SF; compare p50/p99 to airtime model'
Write-Host '  payload-size : send each public frame type; compare measured air bytes to model'
Write-Host '  keyframe     : send one SSDV keyframe chunk train; verify CRC and badge'
Write-Host '  Reference: HIL_RUNBOOK.md W4-00'
Write-Host ''

$metrics = @{
    sub_gate = $SubGate
    sf       = [int]$Sf
}

switch ($SubGate) {
    'txrx' {
        if ($LostFrames -lt 0) { $LostFrames = [int](Read-OperatorPrompt 'lost frames in 1000-frame burst (target 0)') }
        $metrics.frames_sent = $FramesSent
        $metrics.lost_frames = $LostFrames
        if (($LostFrames -ne 0) -and $Result -eq 'PASS') {
            Write-Warning 'Lost frame observed at short range; flipping Result to FAIL'
            $Result = 'FAIL'
        }
    }
    'latency' {
        if ($P50Ms -lt 0) { $P50Ms = [double](Read-OperatorPrompt 'round-trip p50_ms') }
        if ($P99Ms -lt 0) { $P99Ms = [double](Read-OperatorPrompt 'round-trip p99_ms') }
        if ($TheoryRoundTripMs -lt 0) { $TheoryRoundTripMs = [double](Read-OperatorPrompt 'model round-trip ms: lora_time_on_air_ms()*2 + 30') }
        $deltaPct = if ($TheoryRoundTripMs -gt 0) { [Math]::Abs(($P50Ms - $TheoryRoundTripMs) / $TheoryRoundTripMs * 100.0) } else { 999.0 }
        $metrics.p50_ms = $P50Ms
        $metrics.p99_ms = $P99Ms
        $metrics.theory_round_trip_ms = $TheoryRoundTripMs
        $metrics.p50_delta_pct = $deltaPct
        if ((($deltaPct -gt 5.0) -or ($P99Ms -ge ($P50Ms + 50.0))) -and $Result -eq 'PASS') {
            Write-Warning 'Latency criterion violated; flipping Result to FAIL'
            $Result = 'FAIL'
        }
    }
    'payload-size' {
        if ($AirBytesDelta -eq 999) { $AirBytesDelta = [int](Read-OperatorPrompt 'maximum measured-vs-predicted air-byte delta (target <= 1)') }
        $metrics.max_air_bytes_delta = $AirBytesDelta
        if (($AirBytesDelta -gt 1) -and $Result -eq 'PASS') {
            Write-Warning 'Payload size drift exceeds +/-1 byte; flipping Result to FAIL'
            $Result = 'FAIL'
        }
    }
    'keyframe' {
        if ($KeyframeCrcOk -eq 'unknown') { $KeyframeCrcOk = Read-OperatorPrompt 'keyframe reassembled with valid CRC and intact badge? (yes/no)' -Default 'no' }
        $metrics.keyframe_crc_ok = ($KeyframeCrcOk -eq 'yes')
        if (($KeyframeCrcOk -ne 'yes') -and $Result -eq 'PASS') {
            Write-Warning 'Keyframe CRC/badge criterion not met; flipping Result to FAIL'
            $Result = 'FAIL'
        }
    }
}

if (-not $EvidencePath) {
    $EvidencePath = Read-OperatorPrompt 'evidence path (workspace-relative)' -Default "DESIGN-CONTROLLER/bench-evidence/W4-00/W4-00_run-${RunNumber}_${SubGate}_SF${Sf}.log"
}
if (-not $Notes) {
    $Notes = Read-OperatorPrompt 'Operator note' -Default "W4-00 $SubGate SF$Sf run $RunNumber"
}

$runId = New-RunId -GateId 'W4-00' -N $RunNumber -Rung "${SubGate}_SF${Sf}"
Write-HilResult -GateId 'W4-00' `
    -RunId $runId `
    -Operator $Operator `
    -Result $Result `
    -Metrics $metrics `
    -EvidencePaths @($EvidencePath) `
    -Notes $Notes

Write-Host ''
Write-Host 'Gate target: 4 PASS rows (txrx, latency, payload-size, keyframe).'
