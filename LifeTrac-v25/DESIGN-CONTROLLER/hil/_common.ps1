# DESIGN-CONTROLLER/hil/_common.ps1
#
# Shared helpers for Wave-4 HIL harness scripts. Dot-source from each
# w4-XX_*.ps1 script via:   . "$PSScriptRoot/_common.ps1"
#
# This file deliberately does NOT depend on any non-stdlib PowerShell module
# so it runs on Windows PowerShell 5.1 (the bench laptop's default) without
# extra installation.

# ---------------------------------------------------------------------------
# Bench wiring placeholders -- edit these once per laptop / bench setup.
# ---------------------------------------------------------------------------
$Script:HIL_HANDHELD_PORT = 'COM7'    # MKR WAN 1310
$Script:HIL_TRACTOR_PORT  = 'COM8'    # Portenta H7
$Script:HIL_OPTA_PORT     = 'COM9'    # Opta
$Script:HIL_BASE_PORT     = 'COM10'   # second Portenta Max Carrier for W4-00
$Script:HIL_MQTT_HOST     = 'localhost'
$Script:HIL_MQTT_PORT     = 1883
$Script:HIL_AUDIT_LOG     = '/var/log/lifetrac/audit.jsonl'

# ---------------------------------------------------------------------------
# Repo-root resolver. All harnesses live in DESIGN-CONTROLLER/hil/, so the
# repo root is two levels up.
# ---------------------------------------------------------------------------
function Get-LifeTracRepoRoot {
    return (Resolve-Path (Join-Path $PSScriptRoot '..\..')).Path
}

function Get-EvidenceDir {
    param([Parameter(Mandatory)][string]$GateId)
    $root = Get-LifeTracRepoRoot
    $dir = Join-Path $root "DESIGN-CONTROLLER/bench-evidence/$GateId"
    if (-not (Test-Path $dir)) { New-Item -ItemType Directory -Path $dir -Force | Out-Null }
    return $dir
}

function Get-FirmwareSha {
    # Returns the current git short-SHA for the repo, used to stamp every
    # firmware-image field. If a future bench setup builds different SHAs
    # for different nodes, override per call site.
    try {
        return (& git -C (Get-LifeTracRepoRoot) rev-parse --short HEAD 2>$null).Trim()
    } catch {
        return 'unknown'
    }
}

function New-FirmwareShaBundle {
    $sha = Get-FirmwareSha
    return @{
        handheld   = $sha
        tractor_h7 = $sha
        tractor_m4 = $sha
        opta       = $sha
        base       = $sha
    }
}

# ---------------------------------------------------------------------------
# Operator interaction helpers.
# ---------------------------------------------------------------------------
function Read-OperatorPrompt {
    param(
        [Parameter(Mandatory)][string]$Question,
        [string]$Default = ''
    )
    if ($Default) {
        $resp = Read-Host "$Question [$Default]"
        if ([string]::IsNullOrWhiteSpace($resp)) { return $Default }
        return $resp
    }
    return (Read-Host $Question)
}

function Confirm-OperatorChecklist {
    param([Parameter(Mandatory)][string[]]$Items)
    Write-Host ''
    Write-Host '== Operator checklist ==' -ForegroundColor Cyan
    foreach ($item in $Items) {
        do {
            $r = Read-Host "  [ ] $item  (y/N)"
        } until ($r -match '^[yY]')
    }
    Write-Host ''
}

function Assert-Section0-Ready {
    # Mirrors HIL_RUNBOOK.md section 0 (common bench setup). Every harness
    # calls this before its gate-specific procedure. Operator must
    # confirm each item; abort criteria are NOT prompted (those trigger
    # mid-run, not pre-run).
    Confirm-OperatorChecklist @(
        'Tractor crate powered from 12 V battery emulator (13.0 V, 30 A limit)',
        'Handheld powered from charged Li-ion (>= 3.9 V)',
        'All 8 valve coils replaced with dummy loads (no real solenoids)',
        'PSR-alive line broken out to scope CH1',
        'One relay coil terminal broken out to scope CH2',
        'Logic analyzer connected per HIL_RUNBOOK.md section 0.1',
        'Base-station container running, MQTT broker reachable',
        "journalctl -u lifetrac-base -f open in one terminal",
        "mosquitto_sub -t 'lifetrac/#' -v open in another terminal",
        "tail -f $Script:HIL_AUDIT_LOG open in a third terminal",
        'Web UI open at http://localhost:8000 with PIN session active',
        'Firmware built without LIFETRAC_ALLOW_UNCONFIGURED_KEY (production gate exercised)'
    )
}

function Assert-FrontGateReady {
    param([Parameter(Mandatory)][ValidateSet('W4-pre','W4-00')][string]$GateId)

    if ($GateId -eq 'W4-pre') {
        Confirm-OperatorChecklist @(
            'Both Portenta + Max Carrier stacks connected by USB-C to this workstation',
            'Windows shows two distinct USB-CDC serial ports for the carriers',
            'SMA antennas and dummy loads removed; no LoRa TX will be commanded',
            'Multimeter available for Max Carrier 3V3 and 5V0 rail checks',
            'Arduino IDE or VS Code Serial Monitor can open both carrier consoles at 115200 8N1'
        )
        return
    }

    Confirm-OperatorChecklist @(
        'W4-pre is signed off green for both Portenta + Max Carrier stacks',
        'Both Max Carriers are connected to this workstation by USB-C',
        "Tractor-side console is open on $Script:HIL_TRACTOR_PORT at 115200 8N1",
        "Base-side console is open on $Script:HIL_BASE_PORT at 115200 8N1",
        'Each Max Carrier has a 915 MHz SMA antenna or 50 ohm dummy load attached before TX',
        'Antennas are separated by at least 30 cm, or conducted attenuator/dummy-load setup is installed',
        'Both nodes are provisioned with the same fleet key/key ID and distinct source IDs'
    )
}

# ---------------------------------------------------------------------------
# JSONL result-line writer. Schema pinned by results_schema.json.
# ---------------------------------------------------------------------------
function Write-HilResult {
    param(
        [Parameter(Mandatory)][string]$GateId,
        [Parameter(Mandatory)][string]$RunId,
        [Parameter(Mandatory)][string]$Operator,
        [Parameter(Mandatory)][ValidateSet('PASS','FAIL','SKIP','ABORT')][string]$Result,
        [hashtable]$Metrics = @{},
        [string[]]$EvidencePaths = @(),
        [string]$Notes = '',
        [hashtable]$HwSerial = @{},
        [hashtable]$FirmwareSha = $null
    )
    if ($null -eq $FirmwareSha) { $FirmwareSha = New-FirmwareShaBundle }

    $line = [ordered]@{
        gate_id        = $GateId
        run_id         = $RunId
        timestamp      = (Get-Date).ToUniversalTime().ToString('yyyy-MM-ddTHH:mm:ssZ')
        operator       = $Operator
        firmware_sha   = $FirmwareSha
        hw_serial      = $HwSerial
        result         = $Result
        metrics        = $Metrics
        evidence_paths = $EvidencePaths
        notes          = $Notes
    }

    $dir = Get-EvidenceDir -GateId $GateId
    $jsonl = Join-Path $dir 'results.jsonl'
    $json = ($line | ConvertTo-Json -Compress -Depth 6)
    Add-Content -Path $jsonl -Value $json -Encoding UTF8

    Write-Host ''
    Write-Host "[ OK ] Logged $Result run $RunId -> $jsonl" -ForegroundColor Green
}

function New-RunId {
    param(
        [Parameter(Mandatory)][string]$GateId,
        [Parameter(Mandatory)][int]$N,
        [string]$Rung = ''
    )
    $date = (Get-Date).ToString('yyyy-MM-dd')
    $base = ('{0}_run-{1:D3}_{2}' -f $GateId, $N, $date)
    if ($Rung) { return "${base}_$Rung" }
    return $base
}

# ---------------------------------------------------------------------------
# Pretty header.
# ---------------------------------------------------------------------------
function Write-GateHeader {
    param(
        [Parameter(Mandatory)][string]$GateId,
        [Parameter(Mandatory)][string]$Title
    )
    Write-Host ''
    Write-Host ('=' * 72) -ForegroundColor Yellow
    Write-Host (" {0} -- {1}" -f $GateId, $Title) -ForegroundColor Yellow
    Write-Host ('=' * 72) -ForegroundColor Yellow
    Write-Host ' Reference: DESIGN-CONTROLLER/HIL_RUNBOOK.md'
    Write-Host ''
}
