#requires -Version 5.1
<#
.SYNOPSIS
    Side-by-side dump of SX1276 modem/RF state on TX and RX peers, after
    each daemon's _open_link()-equivalent CFG sequence. Falsification
    harness for the rx_frames=0 air-side problem (2026-05-25).

.DESCRIPTION
    For each peer (TX 2E2C, RX 2D0A):
      1) Pulse gpio163 NRST.
      2) Sleep $HostPostNrstSleepMs.
      3) Run radio_state_dump.py inside the daemon container with the
         matching --role tx / --role rx.
      4) Parse the RADIO_DUMP JSON line.

    Then print the two decoded modem configs side-by-side and highlight
    any differences in the params that MUST match for LoRa air decode:
      freq_mhz, sf, bw_hz, cr, sync_word, invert_iq_*, hop_period_symbols,
      preamble_len, implicit_header, rx_payload_crc_on, low_data_rate_optimize.

.EXAMPLE
    powershell -NoProfile -ExecutionPolicy Bypass -File `
        LifeTrac-v25/DESIGN-CONTROLLER/firmware/x8_lora_bootloader_helper/run_radio_state_dump.ps1
#>
param(
    [string]$TxAdbSerial = "2E2C1209DABC240B",
    [string]$RxAdbSerial = "2D0A1209DABC240B",
    [string]$WorkDir     = "/tmp/lifetrac_strict",
    [int]$HostPostNrstSleepMs = 1500,
    [int]$ContainerTimeoutS   = 25,
    [double]$AlsoAfterS = 0.0,
    [string]$RegProfile = "0",
    [string]$OutDir = "LifeTrac-v25/AI NOTES"
)

Set-StrictMode -Version Latest
$ErrorActionPreference = "Stop"

if (-not (Test-Path $OutDir)) {
    New-Item -ItemType Directory -Path $OutDir | Out-Null
}
$stamp  = Get-Date -Format "yyyyMMdd_HHmmss"
$logTxt = Join-Path $OutDir ("2026-05-25_radio_state_dump_${stamp}.log")

$image   = "hub.foundries.io/arduino/arduino-ootb-python-devel:738bc44"
$nrstCmd = "echo fio | sudo -S -p '' sh -c '[ -d /sys/class/gpio/gpio163 ] || echo 163 > /sys/class/gpio/export; echo out > /sys/class/gpio/gpio163/direction; echo 1 > /sys/class/gpio/gpio163/value; sleep 0.02; echo 0 > /sys/class/gpio/gpio163/value; sleep 0.10; echo 1 > /sys/class/gpio/gpio163/value; echo NRST_RELEASED_AT=`$(date +%s.%N)'"

function Write-Both([string]$msg) {
    Write-Host $msg
    Add-Content -Path $logTxt -Value $msg
}

# Stage probe to both peers (chmod via sudo just in case /tmp/lifetrac_strict is root-owned).
$probeSrc = "LifeTrac-v25/DESIGN-CONTROLLER/firmware/x8_lora_bootloader_helper/radio_state_dump.py"
Write-Both "=== run_radio_state_dump ==="
Write-Both "TX=$TxAdbSerial RX=$RxAdbSerial WorkDir=$WorkDir"
Write-Both "HostPostNrstSleepMs=$HostPostNrstSleepMs ContainerTimeoutS=$ContainerTimeoutS AlsoAfterS=$AlsoAfterS RegProfile=$RegProfile"
Write-Both ""

foreach ($serial in @($TxAdbSerial, $RxAdbSerial)) {
    Write-Both "Pushing $probeSrc -> ${serial}:${WorkDir}/radio_state_dump.py"
    & adb -s $serial push $probeSrc "${WorkDir}/radio_state_dump.py" | Out-Null
    if ($LASTEXITCODE -ne 0) { throw "adb push to $serial failed (exit=$LASTEXITCODE)" }
}

function Invoke-Dump([string]$serial, [string]$role) {
    Write-Both ""
    Write-Both "--- $role peer ($serial) ---"
    Write-Both "  pulsing gpio163 NRST..."
    & adb -s $serial shell $nrstCmd 2>&1 | Tee-Object -FilePath $logTxt -Append | Out-Null
    Start-Sleep -Milliseconds $HostPostNrstSleepMs

    $extra = ""
    if ($AlsoAfterS -gt 0) { $extra = "--also-after-s $AlsoAfterS" }
    $inner = "echo fio | sudo -S -p '' docker run --rm --network=host --device=/dev/ttymxc3 -v ${WorkDir}:/work -w /work -e PYTHONPATH=/work:/work/paho -e LIFETRAC_REG_PROFILE=$RegProfile --entrypoint timeout $image $ContainerTimeoutS python3 -u /work/radio_state_dump.py --role $role $extra"
    Write-Both "  cmd: $inner"
    $raw = & adb -s $serial shell $inner 2>&1
    $rawText = ($raw | Out-String)
    Add-Content -Path $logTxt -Value $rawText

    $jsonLine = $null
    foreach ($line in ($rawText -split "`r?`n")) {
        if ($line -match '^RADIO_DUMP\s+(.*)$') {
            $jsonLine = $matches[1]
            break
        }
    }
    if (-not $jsonLine) {
        Write-Both "  ERROR: no RADIO_DUMP line found in container output"
        return $null
    }
    try {
        return $jsonLine | ConvertFrom-Json
    } catch {
        Write-Both "  ERROR: failed to parse RADIO_DUMP JSON: $_"
        return $null
    }
}

$txDump = Invoke-Dump -serial $TxAdbSerial -role "tx"
$rxDump = Invoke-Dump -serial $RxAdbSerial -role "rx"

Write-Both ""
Write-Both "=== DECODED MODEM CONFIG (post-CFG) ==="
$fields = @(
    "freq_mhz", "sf", "bw_hz", "cr", "sync_word",
    "preamble_len", "implicit_header", "rx_payload_crc_on",
    "low_data_rate_optimize", "agc_auto_on",
    "invert_iq_rx", "invert_iq_tx",
    "hop_period_symbols", "fhss_enabled",
    "opmode_hex", "mode_name"
)

function Get-Field($dump, $field) {
    if ($null -eq $dump) { return "<no-dump>" }
    if ($null -eq $dump.decoded_post_config) { return "<no-post-config>" }
    $obj = $dump.decoded_post_config
    if ($obj.PSObject.Properties.Name -contains $field) {
        return [string]$obj.$field
    }
    return "<missing>"
}

$rows = New-Object System.Collections.Generic.List[object]
foreach ($f in $fields) {
    $tv = Get-Field $txDump $f
    $rv = Get-Field $rxDump $f
    $match = if ($tv -eq $rv) { "OK" } else { "**DIFF**" }
    $rows.Add([pscustomobject]@{
        Field = $f
        TX    = $tv
        RX    = $rv
        Match = $match
    })
}
$tableText = ($rows | Format-Table -AutoSize | Out-String)
Write-Both $tableText

# Highlight differences in air-critical params.
$airCritical = @("freq_mhz","sf","bw_hz","cr","sync_word","invert_iq_rx","invert_iq_tx","hop_period_symbols","implicit_header","rx_payload_crc_on","preamble_len","low_data_rate_optimize")
$diffs = $rows | Where-Object { $_.Match -eq "**DIFF**" -and ($airCritical -contains $_.Field) }
$diffs = @($diffs)
if ($diffs.Count -gt 0) {
    Write-Both ""
    Write-Both "*** AIR-CRITICAL MISMATCHES -- these PREVENT LoRa decode ***"
    foreach ($d in $diffs) {
        Write-Both ("  {0,-28} TX={1,-20} RX={2}" -f $d.Field, $d.TX, $d.RX)
    }
} else {
    Write-Both ""
    Write-Both "All air-critical modem params match between TX and RX."
    Write-Both "If rx_frames is still 0, look at:"
    Write-Both "  - FHSS hop schedule (TX vs RX hop_period_symbols + channel mask)"
    Write-Both "  - Antenna / coupling (RSSI floor sniff)"
    Write-Both "  - TX power (RegPaConfig) vs RX LNA (RegLna)"
}

# Always dump raw JSON to the log for full visibility.
Add-Content -Path $logTxt -Value ""
Add-Content -Path $logTxt -Value "=== RAW TX DUMP ==="
Add-Content -Path $logTxt -Value (($txDump | ConvertTo-Json -Depth 10) | Out-String)
Add-Content -Path $logTxt -Value "=== RAW RX DUMP ==="
Add-Content -Path $logTxt -Value (($rxDump | ConvertTo-Json -Depth 10) | Out-String)

Write-Both ""
Write-Both "Log saved to $logTxt"
