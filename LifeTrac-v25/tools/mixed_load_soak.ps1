# 2026-05-19 S1.4 follow-up: 10-minute paired soak.
#
# Paired tx_burst (board A) + rx_listen (board B) for ~10 minutes wall-clock.
# Catches state-machine drift, leaks, fault counter accumulation that a
# 23 s walk_power step cannot surface.
#
# - Power: whatever the last walk_power left set (was +17 dBm).
# - Payload: tx_burst's fixed 24 B 'W1-10b seq=NNNN ...' tag.
# - Cadence: --inter-cycle-s 0.07 (under the 40 % airtime cap).
# - Budget: 6 500 packets ~= 6500 * (0.0257 + 0.07) = 622 s = ~10.4 min.

$ErrorActionPreference = "Stop"

$adb     = "C:\Users\dorkm\AppData\Local\Microsoft\WinGet\Packages\Google.PlatformTools_Microsoft.Winget.Source_8wekyb3d8bbwe\platform-tools\adb.exe"
$boardA  = "2D0A1209DABC240B"
$boardB  = "2E2C1209DABC240B"
$tag     = "mixed_load_2026-05-19"
$outDir  = Join-Path $PSScriptRoot "..\DESIGN-CONTROLLER\bench-evidence\$tag"
$probe   = "/tmp/method_h_stage2_tx_probe_v2.py"
$baud    = "921600"
$port    = "/dev/ttymxc3"

$burstCount  = 6500
$interCycleS = 0.07
$timeoutS    = 3.0
# Empirically tx_burst wall-clock ~= burstCount * 0.12 s/cycle (probe overhead
# above ToA + inter_s). 6500 packets ~= 13 min. Add 3-min buffer for setup +
# teardown + drift.
$rxWindowS   = 960

New-Item -ItemType Directory -Force -Path $outDir | Out-Null

$nowUtc = (Get-Date).ToUniversalTime().ToString("yyyy-MM-dd HH:mm:ss")
foreach ($brd in @($boardA, $boardB)) {
  & $adb -s $brd shell "echo fio | sudo -S date -u -s '$nowUtc'" 2>$null | Out-Null
}
Write-Host "[clock] synced both boards to UTC $nowUtc"

# Clean any leftover python on board B from a prior run.
& $adb -s $boardB shell "echo fio | sudo -S pkill -f method_h_stage2_tx_probe_v2.py" 2>$null | Out-Null
Start-Sleep -Seconds 1

# Launch RX listener (detached via Start-Process; nohup over adb shell drops the child)
$rxLog = Join-Path $outDir "rx_listen_board_b.log"
if (Test-Path $rxLog) { Remove-Item $rxLog -Force }
$rxArgs = @(
  '-s', $boardB, 'shell',
  "echo fio | sudo -S python3 $probe --dev $port --baud $baud --probe rx_listen --rx-window $rxWindowS"
)
$rxProc = Start-Process -FilePath $adb -ArgumentList $rxArgs -RedirectStandardOutput $rxLog -NoNewWindow -PassThru
Write-Host "[rx] launched pid=$($rxProc.Id), waiting for __W1_10B_LISTEN_READY__ ..."

$deadline = (Get-Date).AddSeconds(20)
$ready = $false
while ((Get-Date) -lt $deadline) {
  if (Test-Path $rxLog) {
    if (Select-String -Path $rxLog -Pattern '__W1_10B_LISTEN_READY__' -SimpleMatch -Quiet) {
      $ready = $true
      break
    }
  }
  Start-Sleep -Milliseconds 500
}
if (-not $ready) {
  Write-Host "[rx] FAIL: never saw __W1_10B_LISTEN_READY__ - check $rxLog"
  exit 2
}
Write-Host "[rx] listener ready"

# Run TX burst on board A (synchronously)
$txLog = Join-Path $outDir "tx_burst_board_a.log"
Write-Host "[tx] launching $burstCount packets @ inter=$interCycleS s (est $([math]::Round(($burstCount*(0.0257+$interCycleS))/60, 1)) min)"
& $adb -s $boardA shell "echo fio | sudo -S python3 $probe --dev $port --baud $baud --probe tx_burst --tx-count $burstCount --inter-cycle-s $interCycleS --timeout $timeoutS" *>&1 | Tee-Object -FilePath $txLog | Out-Null
Write-Host "[tx] burst finished"

Start-Sleep -Seconds 6
& $adb -s $boardB shell "echo fio | sudo -S pkill -f method_h_stage2_tx_probe_v2.py" 2>$null | Out-Null
Start-Sleep -Seconds 2

# FCC-B2-b-b-2: stamp the FCC-B2 artifact-header block onto both captured
# logs so downstream consumers (analysis scripts, retro-stamp sweeps in
# b-b-3, FCC-B3 orchestrator gate) can verify firmware_git_sha,
# build_timestamp_utc, profile_enum/profile_string, and the RFCO schema
# versions without having to cross-reference run-time metadata. Soft-fail
# on stamper non-zero exit: a stamp failure on an already-captured log
# must not erase the bench evidence. Profile is BENCH_ONLY_FIXED_915 (=0)
# because this W1-10b / W2 mixed-load soak runs on a single fixed channel
# per the 2026-05-19 FCC plan §5 #3 BENCH_ONLY callout.
$stamper     = Join-Path $PSScriptRoot 'artifact_header.py'
$profileEnum = 0
foreach ($logPath in @($txLog, $rxLog)) {
  if (Test-Path $logPath) {
    & py -3 $stamper stamp --profile-enum $profileEnum --input $logPath --output $logPath
    if ($LASTEXITCODE -ne 0) {
      Write-Host "[stamp] WARN: artifact_header.py exit=$LASTEXITCODE on $logPath"
    } else {
      Write-Host "[stamp] header prepended to $(Split-Path -Leaf $logPath)"
    }
  }
}

# Summarise both logs
Write-Host ""
Write-Host "=== TX TAIL ==="
Get-Content $txLog -Tail 6
Write-Host ""
Write-Host "=== COUNTERS ==="
$txDone   = (Select-String -Path $txLog -Pattern '__TX_DONE__'   -SimpleMatch).Count
$txTo     = (Select-String -Path $txLog -Pattern '__TX_TIMEOUT__' -SimpleMatch).Count
$txFault  = (Select-String -Path $txLog -Pattern '__TX_FAULT__'   -SimpleMatch).Count
$rxFrames = (Select-String -Path $rxLog -Pattern '__RX_FRAME__'   -SimpleMatch).Count
Write-Host "tx_done    = $txDone / $burstCount"
Write-Host "tx_timeout = $txTo"
Write-Host "tx_fault   = $txFault"
Write-Host "rx_frames  = $rxFrames"
$burstDone = Select-String -Path $txLog -Pattern '__W1_10B_BURST_DONE__'
if ($burstDone) { Write-Host ""; Write-Host $burstDone.Line }

Write-Host ""
Write-Host "[done] artefacts in $outDir"
