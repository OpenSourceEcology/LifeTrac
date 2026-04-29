# DESIGN-CONTROLLER/hil/w4-10_fleet_key_provisioning.ps1
#
# W4-10 -- Fleet-key provisioning sanity: missing key halts M7 + bridge at startup.
# Bench-only residual: OLED visual confirmation; systemd-unit restart behaviour on real X8.
# Procedure reference: DESIGN-CONTROLLER/HIL_RUNBOOK.md W4-10.

[CmdletBinding()]
param(
    [Parameter(Mandatory)][string]$Operator,
    [Parameter(Mandatory)][ValidateSet('compile-fail','oled-halt-handheld','oled-halt-tractor','bypass-build-ok','bridge-exit')][string]$Step,
    [int]$RunNumber = 1,
    [ValidateSet('PASS','FAIL','SKIP','ABORT')][string]$Result = 'PASS',
    [bool]$ObservedAsExpected = $true,
    [string]$EvidencePath = '',
    [string]$Notes = ''
)

. "$PSScriptRoot/_common.ps1"

Write-GateHeader -GateId 'W4-10' -Title "Fleet-key provisioning sanity ($Step)"
Assert-Section0-Ready

Write-Host '== W4-10 specific pre-conditions ==' -ForegroundColor Cyan
Confirm-OperatorChecklist @(
    'Spare H747 + handheld available for destructive flash',
    'Build environment ready to delete/zero firmware/common/lp_keys_secret.h',
    'Spectrum analyzer (or RTL-SDR) ready to verify no 915 MHz TX'
)

switch ($Step) {
    'compile-fail' {
        Write-Host '  Step 1: Delete lp_keys_secret.h; build handheld WITHOUT LIFETRAC_ALLOW_UNCONFIGURED_KEY -> compile MUST fail'
    }
    'oled-halt-handheld' {
        Write-Host '  Step 2: All-zero key, build WITHOUT bypass, flash handheld -> OLED "FLEET KEY NOT PROVISIONED" within 2 s; no LoRa TX in 30 s'
    }
    'oled-halt-tractor' {
        Write-Host '  Step 3: Same as Step 2 but tractor M7'
    }
    'bypass-build-ok' {
        Write-Host '  Step 4: All-zero key, build WITH bypass -> boot must complete (dev path still works)'
    }
    'bridge-exit' {
        Write-Host '  Step 5: Start base-station container with bridge config pointing to non-existent fleet-key file; container exits non-zero in 5 s; audit log fleet_key_missing'
    }
}
Write-Host ''

if (-not $EvidencePath) {
    $defaultExt = if ($Step -eq 'bridge-exit') { 'log' } else { 'jpg' }
    $EvidencePath = Read-OperatorPrompt 'Evidence path' -Default "DESIGN-CONTROLLER/bench-evidence/W4-10/W4-10_run-${RunNumber}_${Step}.${defaultExt}"
}
if (-not $Notes) { $Notes = Read-OperatorPrompt 'Operator note' -Default "$Step run $RunNumber" }

if ((-not $ObservedAsExpected) -and $Result -eq 'PASS') {
    Write-Warning 'Step expectation NOT met; flipping Result to FAIL'
    $Result = 'FAIL'
}

$runId = New-RunId -GateId 'W4-10' -N $RunNumber -Rung $Step
Write-HilResult -GateId 'W4-10' `
    -RunId $runId `
    -Operator $Operator `
    -Result $Result `
    -Metrics @{
        step                  = $Step
        observed_as_expected  = $ObservedAsExpected
    } `
    -EvidencePaths @($EvidencePath) `
    -Notes $Notes

Write-Host ''
Write-Host 'Gate target: 5 PASS rows (one per Step).'
