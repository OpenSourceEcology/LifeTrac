# DESIGN-CONTROLLER/hil/w4-08_camera_keyframe.ps1
#
# W4-08 -- Camera back-channel CMD_REQ_KEYFRAME < 200 ms end-to-end; LoRa leg < 100 ms.
# Bench-only residual: real H747 Serial1 baud stability + camera-encoder I-frame produce time on real X8.
# Procedure reference: DESIGN-CONTROLLER/HIL_RUNBOOK.md W4-08.

[CmdletBinding()]
param(
    [Parameter(Mandatory)][string]$Operator,
    [int]$RunNumber = 1,
    [ValidateSet('PASS','FAIL','SKIP','ABORT')][string]$Result = 'PASS',
    [double]$EndToEndMs = -1,
    [double]$LoraLegMs = -1,
    [string]$LogicAnalyzerPath = '',
    [string]$LatencyCsvPath = '',
    [string]$Notes = ''
)

. "$PSScriptRoot/_common.ps1"

Write-GateHeader -GateId 'W4-08' -Title 'Camera back-channel keyframe round-trip'
Assert-Section0-Ready

Write-Host '== W4-08 specific pre-conditions ==' -ForegroundColor Cyan
Confirm-OperatorChecklist @(
    'Coral X8 attached, camera streaming',
    "lifetrac/image/frame MQTT topic publishing at >= 5 fps",
    'Logic analyzer on X8<->H747 UART decoding ASCII text'
)

Write-Host '== W4-08 procedure ==' -ForegroundColor Cyan
Write-Host '  1. Capture baseline P-frames; keyframe interval ~ 60'
Write-Host '  2. Click "Force keyframe" in web UI; note t_click (browser devtools)'
Write-Host '  3. Capture UART: KEYFRAME ASCII command from H747 to X8; record t_uart_cmd'
Write-Host '  4. Capture next MQTT frame with is_keyframe == true; record t_keyframe'
Write-Host '  5. Compute end-to-end latency'
Write-Host ''

if ($EndToEndMs -lt 0) { $EndToEndMs = [double](Read-OperatorPrompt 'end-to-end latency (ms)') }
if ($LoraLegMs -lt 0)  { $LoraLegMs  = [double](Read-OperatorPrompt 'LoRa leg latency t_uart_cmd - t_click (ms)') }
if (-not $LogicAnalyzerPath) {
    $LogicAnalyzerPath = Read-OperatorPrompt 'Logic-analyzer capture path' -Default "DESIGN-CONTROLLER/bench-evidence/W4-08/W4-08_run-${RunNumber}_la.sal"
}
if (-not $LatencyCsvPath) {
    $LatencyCsvPath = Read-OperatorPrompt 'Latency CSV path' -Default "DESIGN-CONTROLLER/bench-evidence/W4-08/W4-08_latency.csv"
}
if (-not $Notes) { $Notes = Read-OperatorPrompt 'Operator note' -Default "Run $RunNumber" }

if ((($EndToEndMs -ge 200.0) -or ($LoraLegMs -ge 100.0)) -and $Result -eq 'PASS') {
    Write-Warning 'Pass criterion violated; flipping Result to FAIL'
    $Result = 'FAIL'
}

$runId = New-RunId -GateId 'W4-08' -N $RunNumber
Write-HilResult -GateId 'W4-08' `
    -RunId $runId `
    -Operator $Operator `
    -Result $Result `
    -Metrics @{
        end_to_end_ms   = $EndToEndMs
        lora_leg_ms     = $LoraLegMs
        e2e_threshold   = 200.0
        lora_threshold  = 100.0
    } `
    -EvidencePaths @($LogicAnalyzerPath, $LatencyCsvPath) `
    -Notes $Notes

Write-Host ''
Write-Host 'Gate target: 30 PASS runs.'
