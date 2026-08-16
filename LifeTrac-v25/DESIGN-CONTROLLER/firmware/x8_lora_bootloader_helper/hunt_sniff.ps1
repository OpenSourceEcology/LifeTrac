#requires -Version 5.1
<#
.SYNOPSIS
    RS-11.6 interferer hunt loop: one command per suspect, verdict in ~2 min.
.DESCRIPTION
    Runs a short raw-RSSI sniff on a bench radio and prints LINE PRESENT /
    LINE ABSENT for the 7.08 s emitter. Intended use at the bench:

        1. remove / power off ONE suspect near the base carrier
        2. .\hunt_sniff.ps1 -Label "wall-thermometer-removed"
        3. read the verdict; repeat

    The base radio is the default detector (the emitter is ~20 dB hotter
    there; see bench-evidence/RS_11_6_idle_sniff_2026-08-16/RESULTS.md).
    Radios must be otherwise idle: this script frees the UART by removing
    the daemon container on the chosen board (rx_smoke / tx_smoke), which
    the next run_live_radio_monitor.ps1 launch recreates anyway.

    Verdicts (exit code): 10 LINE PRESENT / 11 hot-but-aperiodic /
    12 inconclusive / 0 LINE ABSENT. Every capture is kept as evidence
    under bench-evidence/RS_11_6_hunt_<date>/<label>.jsonl.
#>
param(
    [string]$Label = "unlabeled",
    [ValidateSet("base", "tractor")]
    [string]$Board = "base",
    [int]$DurationS = 120,
    [string]$BaseSerial = "2D0A1209DABC240B",
    [string]$TractorSerial = "2E2C1209DABC240B"
)

$ErrorActionPreference = "Stop"
$helperDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$dcRoot    = (Resolve-Path (Join-Path $helperDir "..\..")).Path
$stamp     = Get-Date -Format "yyyy-MM-dd"
$evDir     = Join-Path $dcRoot "bench-evidence\RS_11_6_hunt_$stamp"
New-Item -ItemType Directory -Force $evDir | Out-Null

# Windows-safe filename from the label.
$safe = ($Label -replace '[^A-Za-z0-9_.-]', '_')
$ts   = Get-Date -Format "HHmmss"
$out  = Join-Path $evDir "$ts`_$safe.jsonl"

if ($Board -eq "base") {
    $serial = $BaseSerial;    $daemon = "rx_smoke"
    $image  = "lifetrac-v25:latest"
} else {
    $serial = $TractorSerial; $daemon = "tx_smoke"
    # Same image the harness uses for the tractor-side daemon.
    $image  = "hub.foundries.io/arduino/arduino-ootb-python-devel:738bc44"
}

Write-Host "[HUNT] board=$Board serial=$serial duration=${DurationS}s label='$Label'"
Write-Host "[HUNT] freeing UART (docker rm -f $daemon; harness recreates it)..."
adb -s $serial push (Join-Path $helperDir "air_coupling_rssi_sniff.py") /tmp/lifetrac_strict/ | Out-Null
adb -s $serial shell "echo fio | sudo -S -p '' docker rm -f $daemon 2>/dev/null" | Out-Null

Write-Host "[HUNT] sniffing ${DurationS}s (~$([math]::Round($DurationS/7.08)) emitter cycles)..."
adb -s $serial shell "echo fio | sudo -S -p '' docker run --rm --name hunt_sniff --network=host --entrypoint python3 --device=/dev/ttymxc3 -v /tmp/lifetrac_strict:/work -w /work -e PYTHONPATH=/work:/work/paho -e LIFETRAC_REG_PROFILE=2 $image -u /work/air_coupling_rssi_sniff.py --duration-s $DurationS --interval-s 0.05" |
    Out-File -Encoding utf8 $out

Write-Host "[HUNT] verdict:"
py -3 (Join-Path $dcRoot "tools\rssi_sniff_periodicity.py") $out --hunt
$code = $LASTEXITCODE
Write-Host "[HUNT] capture kept: $out"
exit $code
