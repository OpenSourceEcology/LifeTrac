#requires -Version 5.1
<#
.SYNOPSIS
    RS-12 depth A/B driver: TxPipelineDepth 2 vs 1, interleaved, n=3 each.
.DESCRIPTION
    Leg F (2026-08-16) hinted that dropping the TX mailbox depth from 2 to 1
    halves the penultimate-fragment lock (28% -> 15%) at the cost of ~6.5%
    offered throughput. That was n=1 against a known ~2x run-to-run spread,
    so it is not yet a recommendation. This driver settles it.

    Design notes:
    * INTERLEAVED (2,1,2,1,2,1) rather than blocked. The 2026-08-17 outage
      proved the RF environment drifts on its own schedule; interleaving
      spreads any drift across both arms instead of confounding one.
    * Each leg is counter-bracketed (rs115_stats_probe before/after) so the
      firmware-drop count is per-leg, not just per-series. The base's dead
      NRST makes its counters cumulative, which is what makes this cheap.
    * The channel is re-verified before AND after the series: a hopping
      interferer that wanders onto our carrier mid-series would otherwise
      masquerade as an arm effect.

    Outputs everything under bench-evidence/RS_12_depth_ab_<date>/ and
    prints the archive path of each leg for the analyzer.
#>
param(
    [int]$ForceFrfHz  = 909000000,
    [int]$DurationS   = 300,
    [int]$Repeats     = 3,
    [string]$BaseSerial = "2D0A1209DABC240B"
)

$ErrorActionPreference = "Stop"
$helperDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$dcRoot    = (Resolve-Path (Join-Path $helperDir "..\..")).Path
$evDir     = Join-Path $dcRoot ("bench-evidence\RS_12_depth_ab_" + (Get-Date -Format "yyyy-MM-dd"))
New-Item -ItemType Directory -Force $evDir | Out-Null

function Snap([string]$tag) {
    $out = Join-Path $evDir "stats_$tag.txt"
    adb -s $BaseSerial shell "echo fio | sudo -S -p '' docker rm -f rx_smoke 2>/dev/null; echo fio | sudo -S -p '' docker run --rm --network=host --entrypoint python3 --device=/dev/ttymxc3 -v /tmp/lifetrac_strict:/work -w /work -e PYTHONPATH=/work:/work/paho lifetrac-v25:latest -u /work/rs115_stats_probe.py" |
        Out-File -Encoding utf8 $out
    Write-Host "[AB] snapshot -> $out"
}

function CheckChannel([string]$tag) {
    $out = Join-Path $evDir "chancheck_$tag.jsonl"
    $lo = $ForceFrfHz - 500000; $hi = $ForceFrfHz + 500000
    adb -s $BaseSerial shell "echo fio | sudo -S -p '' docker rm -f rx_smoke 2>/dev/null; echo fio | sudo -S -p '' docker run --rm --name ab_chan --network=host --entrypoint python3 --device=/dev/ttymxc3 -v /tmp/lifetrac_strict:/work -w /work -e PYTHONPATH=/work:/work/paho -e LIFETRAC_REG_PROFILE=2 lifetrac-v25:latest -u /work/channel_survey_sniff.py --start-hz $lo --stop-hz $hi --step-hz 500000 --dwell-s 45 --interval-s 0.05" |
        Out-File -Encoding utf8 $out
    Write-Host "[AB] channel check ($tag):"
    Select-String -Path $out -Pattern "SURVEY_CH" | ForEach-Object { "    " + $_.Line }
}

Write-Host "[AB] RS-12 depth A/B  frf=$ForceFrfHz  n=$Repeats per arm  interleaved"
CheckChannel "pre"

$order = @()
for ($i = 1; $i -le $Repeats; $i++) { $order += 2; $order += 1 }

$legs = @()
$n = 0
foreach ($depth in $order) {
    $n++
    $tag = "leg{0:d2}_depth$depth" -f $n
    Write-Host "[AB] --- $tag ---"
    Snap "${tag}_pre"
    # The harness reports via Write-Host, which does NOT reach the pipeline,
    # so capturing its output does not yield the archive path (2026-08-17:
    # this silently wrote empty paths into legs.json). Instead, note the
    # newest archive directory after the run -- unambiguous because the
    # harness creates exactly one per leg.
    $evRoot = Join-Path $dcRoot "bench-evidence"
    $before = @(Get-ChildItem -Path $evRoot -Directory -Filter "radio_monitor_*" |
                Select-Object -ExpandProperty Name)
    & (Join-Path $helperDir "run_live_radio_monitor.ps1") `
        -TxFeed local -RegProfile 2 -DurationS $DurationS `
        -SynthFps 2 -SynthBudgetB 3000 -KfRequestDisable 1 -ProbeEcho 0 `
        -ForceFrfHz $ForceFrfHz -TxPipelineDepth $depth -Archive | Out-Null
    Snap "${tag}_post"
    $arch = (Get-ChildItem -Path $evRoot -Directory -Filter "radio_monitor_*" |
             Where-Object { $before -notcontains $_.Name } |
             Sort-Object LastWriteTime | Select-Object -Last 1).FullName
    Write-Host "[AB] $tag archive: $arch"
    $legs += [pscustomobject]@{ tag = $tag; depth = $depth; archive = $arch }
}

CheckChannel "post"

$legs | ConvertTo-Json | Out-File -Encoding utf8 (Join-Path $evDir "legs.json")
Write-Host "[AB] DONE. legs.json written to $evDir"
$legs | ForEach-Object { "{0}  depth={1}  {2}" -f $_.tag, $_.depth, $_.archive }
