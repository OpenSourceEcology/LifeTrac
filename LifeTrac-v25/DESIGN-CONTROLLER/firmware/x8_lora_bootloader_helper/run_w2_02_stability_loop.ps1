<#
.SYNOPSIS
  W2-02 stability loop: run the end-to-end image-over-LoRa orchestrator N
  times back-to-back, collect each summary.json, and emit aggregate stats.

.DESCRIPTION
  Calls run_w2_02_image_over_lora_end_to_end.ps1 in a loop. After each run
  the orchestrator writes a timestamped evidence dir under
  LifeTrac-v25/DESIGN-CONTROLLER/bench-evidence/W2-02_image_over_lora_*.
  This script captures the per-run summary.json + summary_top.json and
  computes pass/fail per gate + variance across runs.

  Output: a stability evidence dir
    LifeTrac-v25/DESIGN-CONTROLLER/bench-evidence/W2-02_stability_<UTC>/
  containing:
    - run_<NN>_link.txt   (path to the per-run evidence dir)
    - run_<NN>_summary.json (copy)
    - stability_summary.json (aggregate)
    - stability_report.txt   (human-readable)
    - loop.log               (full orchestrator stdout/stderr per run)
#>
[CmdletBinding()]
param(
    [int]$N = 10,
    [int]$InterRunSleepS = 5,
    # Forwarded to the underlying orchestrator
    [string]$TxAdbSerial = "2E2C1209DABC240B",
    [string]$RxAdbSerial = "2D0A1209DABC240B",
    [double]$InterFragS = 0.2,
    [int]$ExtraRxWindowS = 30
)

$ErrorActionPreference = "Stop"
$repoRoot = Resolve-Path (Join-Path $PSScriptRoot "..\..\..\..")
$evidenceRoot = Join-Path $repoRoot "LifeTrac-v25\DESIGN-CONTROLLER\bench-evidence"
$stamp = (Get-Date).ToString("yyyy-MM-dd_HHmmss")
$stabilityDir = Join-Path $evidenceRoot "W2-02_stability_$stamp"
New-Item -ItemType Directory -Path $stabilityDir | Out-Null
$loopLog = Join-Path $stabilityDir "loop.log"

Write-Host "===================================================="
Write-Host "W2-02 STABILITY LOOP N=$N"
Write-Host "Stability dir : $stabilityDir"
Write-Host "InterFragS    : $InterFragS"
Write-Host "ExtraRxWindow : $ExtraRxWindowS s"
Write-Host "TX serial     : $TxAdbSerial"
Write-Host "RX serial     : $RxAdbSerial"
Write-Host "===================================================="

$orchestrator = Join-Path $PSScriptRoot "run_w2_02_image_over_lora_end_to_end.ps1"
if (-not (Test-Path $orchestrator)) {
    throw "Orchestrator not found: $orchestrator"
}

$results = New-Object System.Collections.ArrayList

for ($i = 1; $i -le $N; $i++) {
    $runStart = Get-Date
    Write-Host ""
    Write-Host "--- RUN $i / $N (started $runStart) ---"
    Add-Content -Path $loopLog -Value "===== RUN $i / $N $runStart ====="

    # Snapshot existing W2-02 dirs so we can find the NEW one
    $beforeDirs = @(Get-ChildItem $evidenceRoot -Directory `
        | Where-Object Name -like "W2-02_image_over_lora_*" `
        | ForEach-Object Name)

    try {
        # Temporarily relax error preference: adb writes informational
        # output to stderr (e.g. "287 files pushed"); 2>&1 surfaces it as
        # error records and our Stop-preference would otherwise abort the
        # run in ~1 s without ever invoking the bench.
        $prevPref = $ErrorActionPreference
        $ErrorActionPreference = "Continue"
        & powershell -NoProfile -ExecutionPolicy Bypass -File $orchestrator `
            -TxAdbSerial $TxAdbSerial `
            -RxAdbSerial $RxAdbSerial `
            -InterFragS $InterFragS `
            -ExtraRxWindowS $ExtraRxWindowS `
            2>&1 | Tee-Object -FilePath $loopLog -Append | Out-Null
        $orchRc = $LASTEXITCODE
        $ErrorActionPreference = $prevPref
    } catch {
        $ErrorActionPreference = $prevPref
        Add-Content -Path $loopLog -Value "EXCEPTION: $_"
        $orchRc = -1
    }

    # Find the new evidence dir
    $afterDirs = @(Get-ChildItem $evidenceRoot -Directory `
        | Where-Object Name -like "W2-02_image_over_lora_*" `
        | ForEach-Object Name)
    $newDirs = @($afterDirs | Where-Object { $_ -notin $beforeDirs })
    $newDir = $null
    if ($newDirs.Count -gt 0) {
        $newDir = ($newDirs | Sort-Object -Descending | Select-Object -First 1)
    }

    $runRec = [ordered]@{
        run_index   = $i
        started     = $runStart.ToString("o")
        orch_rc     = $orchRc
        evidence    = $newDir
        verdict     = "UNKNOWN"
        tx_ok_rate  = $null
        rx_match_rate = $null
        frame_complete = $null
        tiles_decoded  = $null
        rssi_median_dbm = $null
        snr_median_db   = $null
        n_fragments = $null
        n_tx_ok     = $null
        n_tx_err    = $null
        n_tx_timeout = $null
        n_rx_frames = $null
    }

    if ($newDir) {
        $newDirFull = Join-Path $evidenceRoot $newDir
        $linkFile = Join-Path $stabilityDir ("run_{0:D2}_link.txt" -f $i)
        Set-Content -Path $linkFile -Value $newDirFull

        $sumPath = Join-Path $newDirFull "summary.json"
        $sumTopPath = Join-Path $newDirFull "summary_top.json"
        $txPath = Join-Path $newDirFull "tx_stdout.txt"
        $rxPath = Join-Path $newDirFull "rx_stdout.txt"

        if (Test-Path $sumPath) {
            Copy-Item $sumPath -Destination (Join-Path $stabilityDir ("run_{0:D2}_summary.json" -f $i))
        }
        if (Test-Path $sumTopPath) {
            try {
                $top = Get-Content $sumTopPath -Raw | ConvertFrom-Json
                # summary_top.json fields produced by orchestrator
                $runRec.tx_ok_rate      = $top.tx_ok_rate
                $runRec.rx_match_rate   = $top.rx_match_rate
                $runRec.frame_complete  = $top.frame_complete
                $runRec.tiles_decoded   = $top.tiles_decoded
                $runRec.rssi_median_dbm = $top.rssi_median_dbm
                $runRec.snr_median_db   = $top.snr_median_db
                $runRec.n_fragments     = $top.n_fragments_planned
                $runRec.n_tx_ok         = $top.n_tx_ok
                $runRec.n_tx_timeout    = $top.n_tx_timeout
                $runRec.n_rx_frames     = $top.n_rx_frames
                # Derive verdict from gates[]
                $allOk = $true
                if ($top.gates) {
                    foreach ($g in $top.gates) { if (-not $g.ok) { $allOk = $false } }
                    $runRec.verdict = if ($allOk) { "PASS" } else { "FAIL" }
                }
            } catch {
                Add-Content -Path $loopLog -Value "WARN: failed to parse $sumTopPath : $_"
            }
        }
        # n_tx_err is not in summary_top.json — always derive from tx_stdout
        if (Test-Path $txPath) {
            $runRec.n_tx_err = (Select-String "__W2_02_TX_ERR__" $txPath -ErrorAction SilentlyContinue).Count
            if ($runRec.n_tx_ok -eq $null) {
                $runRec.n_tx_ok      = (Select-String "__W2_02_TX_FRAG__"    $txPath -ErrorAction SilentlyContinue).Count
                $runRec.n_tx_timeout = (Select-String "__W2_02_TX_TIMEOUT__" $txPath -ErrorAction SilentlyContinue).Count
            }
        }
        if ($runRec.n_rx_frames -eq $null -and (Test-Path $rxPath)) {
            $runRec.n_rx_frames = (Select-String "__RX_FRAME__" $rxPath -ErrorAction SilentlyContinue).Count
        }
    } else {
        Add-Content -Path $loopLog -Value "WARN: no new evidence dir produced for run $i"
    }

    [void]$results.Add([pscustomobject]$runRec)
    $elapsed = (Get-Date) - $runStart
    Write-Host ("  -> verdict={0} tx_ok={1}/{2} err={3} rx={4} tiles={5} elapsed={6:F0}s" -f `
        $runRec.verdict, $runRec.n_tx_ok, $runRec.n_fragments, $runRec.n_tx_err, `
        $runRec.n_rx_frames, $runRec.tiles_decoded, $elapsed.TotalSeconds)

    if ($i -lt $N) {
        Write-Host "  sleeping $InterRunSleepS s before next run..."
        Start-Sleep -Seconds $InterRunSleepS
    }
}

# ========= Aggregate =========
function Get-Median($values) {
    $arr = @($values | Where-Object { $_ -ne $null }) | Sort-Object
    if ($arr.Count -eq 0) { return $null }
    $mid = [int]($arr.Count / 2)
    if ($arr.Count % 2 -eq 0) {
        return ($arr[$mid - 1] + $arr[$mid]) / 2.0
    } else {
        return $arr[$mid]
    }
}

function Get-Stats($values) {
    $arr = @($values | Where-Object { $_ -ne $null })
    if ($arr.Count -eq 0) { return $null }
    $sum = 0.0; foreach ($v in $arr) { $sum += [double]$v }
    $mean = $sum / $arr.Count
    $sq = 0.0; foreach ($v in $arr) { $d = [double]$v - $mean; $sq += $d * $d }
    $std = if ($arr.Count -gt 1) { [Math]::Sqrt($sq / ($arr.Count - 1)) } else { 0.0 }
    return [pscustomobject]@{
        n      = $arr.Count
        min    = ($arr | Measure-Object -Minimum).Minimum
        max    = ($arr | Measure-Object -Maximum).Maximum
        mean   = $mean
        median = (Get-Median $arr)
        std    = $std
    }
}

$passCount = @($results | Where-Object verdict -eq "PASS").Count
$failCount = @($results | Where-Object verdict -ne "PASS").Count

$txOkStats   = Get-Stats ($results | ForEach-Object tx_ok_rate)
$rxMatchStats = Get-Stats ($results | ForEach-Object rx_match_rate)
$tilesStats  = Get-Stats ($results | ForEach-Object tiles_decoded)
$rssiStats   = Get-Stats ($results | ForEach-Object rssi_median_dbm)
$snrStats    = Get-Stats ($results | ForEach-Object snr_median_db)
$errStats    = Get-Stats ($results | ForEach-Object n_tx_err)
$timeoutStats = Get-Stats ($results | ForEach-Object n_tx_timeout)

$agg = [ordered]@{
    n_runs        = $N
    pass_count    = $passCount
    fail_count    = $failCount
    pass_rate     = if ($N -gt 0) { [double]$passCount / $N } else { 0.0 }
    inter_frag_s  = $InterFragS
    tx_ok_rate    = $txOkStats
    rx_match_rate = $rxMatchStats
    tiles_decoded = $tilesStats
    rssi_median_dbm = $rssiStats
    snr_median_db   = $snrStats
    n_tx_err      = $errStats
    n_tx_timeout  = $timeoutStats
    runs          = $results
}

$aggJson = $agg | ConvertTo-Json -Depth 6
Set-Content -Path (Join-Path $stabilityDir "stability_summary.json") -Value $aggJson

# Human-readable report
$report = New-Object System.Text.StringBuilder
[void]$report.AppendLine("W2-02 STABILITY LOOP REPORT")
[void]$report.AppendLine(("Runs       : {0}" -f $N))
[void]$report.AppendLine(("PASS       : {0}/{1} ({2:P1})" -f $passCount, $N, ($passCount / [double]$N)))
[void]$report.AppendLine(("FAIL       : {0}/{1}" -f $failCount, $N))
[void]$report.AppendLine(("inter-s    : {0}" -f $InterFragS))
[void]$report.AppendLine("")
[void]$report.AppendLine("Per-gate aggregates (n, min, mean, median, max, std):")
foreach ($pair in @(
    @("tx_ok_rate",     $txOkStats),
    @("rx_match_rate",  $rxMatchStats),
    @("tiles_decoded",  $tilesStats),
    @("rssi_median_dbm",$rssiStats),
    @("snr_median_db",  $snrStats),
    @("n_tx_err",       $errStats),
    @("n_tx_timeout",   $timeoutStats)
)) {
    $name = $pair[0]; $s = $pair[1]
    if ($s) {
        [void]$report.AppendLine(("  {0,-18} n={1,2} min={2,8:F3} mean={3,8:F3} median={4,8:F3} max={5,8:F3} std={6,7:F3}" `
            -f $name, $s.n, [double]$s.min, [double]$s.mean, [double]$s.median, [double]$s.max, [double]$s.std))
    } else {
        [void]$report.AppendLine(("  {0,-18} no data" -f $name))
    }
}
[void]$report.AppendLine("")
[void]$report.AppendLine("Per-run table:")
[void]$report.AppendLine(("  {0,3} {1,-6} {2,8} {3,8} {4,4} {5,5} {6,5} {7,5}" -f "run","verdict","tx_ok","rx_mat","tile","rssi","snr","err"))
foreach ($r in $results) {
    [void]$report.AppendLine(("  {0,3} {1,-6} {2,8} {3,8} {4,4} {5,5} {6,5} {7,5}" `
        -f $r.run_index, $r.verdict, $r.tx_ok_rate, $r.rx_match_rate, `
        $r.tiles_decoded, $r.rssi_median_dbm, $r.snr_median_db, $r.n_tx_err))
}

Set-Content -Path (Join-Path $stabilityDir "stability_report.txt") -Value $report.ToString()

Write-Host ""
Write-Host "===================================================="
Write-Host "STABILITY LOOP DONE"
Write-Host ("PASS {0}/{1}" -f $passCount, $N)
Write-Host "Report: $(Join-Path $stabilityDir 'stability_report.txt')"
Write-Host "JSON  : $(Join-Path $stabilityDir 'stability_summary.json')"
Write-Host "===================================================="
Write-Host $report.ToString()
