#requires -Version 5.1
<#
.SYNOPSIS
    RS-12 fix confirmation: -NoParkLast 1 vs control, interleaved, n=3 each.
.DESCRIPTION
    Leg J validated the no-park-last fix at n=1 (lock 42% -> 7%, loss 3.3%
    -> 1.7%). This driver is the n=3 confirmation, patterned on
    run_rs12_depth_ab.ps1: interleaved arms (ctrl,fix,...), per-leg counter
    brackets, channel checks before and after the series, and per-fragment
    arrival logging on EVERY leg so each leg also reports its last-pair
    demod gap (the mechanism-level check rides along with the endpoints).
#>
param(
    [int]$ForceFrfHz  = 927500000,
    [int]$DurationS   = 300,
    [int]$Repeats     = 3,
    [string]$BaseSerial = "2D0A1209DABC240B"
)

$ErrorActionPreference = "Stop"
$helperDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$dcRoot    = (Resolve-Path (Join-Path $helperDir "..\..")).Path
$evDir     = Join-Path $dcRoot ("bench-evidence\RS_12_noparklast_ab_" + (Get-Date -Format "yyyy-MM-dd"))
New-Item -ItemType Directory -Force $evDir | Out-Null

function Snap([string]$tag) {
    $out = Join-Path $evDir "stats_$tag.txt"
    adb -s $BaseSerial shell "echo fio | sudo -S -p '' docker rm -f rx_smoke 2>/dev/null; echo fio | sudo -S -p '' docker run --rm --network=host --entrypoint python3 --device=/dev/ttymxc3 -v /tmp/lifetrac_strict:/work -w /work -e PYTHONPATH=/work:/work/paho lifetrac-v25:latest -u /work/rs115_stats_probe.py" |
        Out-File -Encoding utf8 $out
    Write-Host "[AB] snapshot -> $out"
}

function CheckChannel([string]$tag) {
    $out = Join-Path $evDir "chancheck_$tag.jsonl"
    adb -s $BaseSerial shell "echo fio | sudo -S -p '' docker rm -f rx_smoke ab_chan 2>/dev/null; echo fio | sudo -S -p '' docker run --rm --name ab_chan --network=host --entrypoint python3 --device=/dev/ttymxc3 -v /tmp/lifetrac_strict:/work -w /work -e PYTHONPATH=/work:/work/paho -e LIFETRAC_REG_PROFILE=2 lifetrac-v25:latest -u /work/channel_survey_sniff.py --start-hz $ForceFrfHz --stop-hz $ForceFrfHz --step-hz 500000 --dwell-s 45 --interval-s 0.05" |
        Out-File -Encoding utf8 $out
    Write-Host "[AB] channel check ($tag):"
    Select-String -Path $out -Pattern "SURVEY_CH" | ForEach-Object { "    " + $_.Line }
}

Write-Host "[AB] RS-12 NoParkLast A/B  frf=$ForceFrfHz  n=$Repeats/arm  interleaved"
CheckChannel "pre"

$order = @()
for ($i = 1; $i -le $Repeats; $i++) { $order += "ctrl"; $order += "fix" }

$evRoot = Join-Path $dcRoot "bench-evidence"
$legs = @()
$n = 0
foreach ($arm in $order) {
    $n++
    $tag = "leg{0:d2}_$arm" -f $n
    $npl = if ($arm -eq "fix") { 1 } else { 0 }
    Write-Host "[AB] --- $tag (NoParkLast=$npl) ---"
    Snap "${tag}_pre"
    $before = @(Get-ChildItem -Path $evRoot -Directory -Filter "radio_monitor_*" |
                Select-Object -ExpandProperty Name)
    & (Join-Path $helperDir "run_live_radio_monitor.ps1") `
        -TxFeed local -RegProfile 2 -DurationS $DurationS `
        -SynthFps 2 -SynthBudgetB 3000 -KfRequestDisable 1 -ProbeEcho 0 `
        -ForceFrfHz $ForceFrfHz -LogFragArrivals 1 -NoParkLast $npl -Archive | Out-Null
    Snap "${tag}_post"
    $arch = (Get-ChildItem -Path $evRoot -Directory -Filter "radio_monitor_*" |
             Where-Object { $before -notcontains $_.Name } |
             Sort-Object LastWriteTime | Select-Object -Last 1).FullName
    Write-Host "[AB] $tag archive: $arch"
    $legs += [pscustomobject]@{ tag = $tag; arm = $arm; archive = $arch }
}

CheckChannel "post"

$legs | ConvertTo-Json | Out-File -Encoding utf8 (Join-Path $evDir "legs.json")
Write-Host "[AB] DONE."
$legs | ForEach-Object { "{0}  arm={1}  {2}" -f $_.tag, $_.arm, $_.archive }
