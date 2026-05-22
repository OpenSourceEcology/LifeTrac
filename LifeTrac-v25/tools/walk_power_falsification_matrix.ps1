# Walk-power falsification matrix (P1-4 / Copilot Review v3.0 §2, v4.0 §3)
#
# Runs three controlled walk_power passes against the paired bench:
#
#   Pass A  LBT off, inter_cycle_s = 0.05  (50 ms)
#     - Isolates "Python loop stutter" hypothesis. If the 14-15 dBm cliff
#       persists even with 50 ms between packets (>>> ToA + host overhead),
#       it is NOT a host scheduling problem.
#
#   Pass B  LBT on,  inter_cycle_s = 0.05  (50 ms)
#     - Tests the LBT-defer hypothesis. If A is clean but B shows a cliff
#       AND radio_tx_abort_lbt_delta jumps, LBT is the culprit.
#
#   Pass C  LBT off, inter_cycle_s = 0.02  (20 ms) -- legacy control
#     - Reproduces the existing cliff under the original orchestrator
#       cadence so we have a same-day same-environment baseline to compare.
#
# Each pass:
#   - Starts a fresh rx_listen on board B (full window).
#   - Runs walk_power on board A with the matrix arguments above.
#   - Stops the RX listener via SIGINT + waits for __RX_LISTEN_STOPPED__.
#   - Saves per-step CSV (walk_power_tx_side.csv) and per-packet CSV
#     (walk_power_tx_side_perpacket.csv) under
#     bench-evidence/walk_power_matrix_<ts>/pass_<A|B|C>/.
#
# Decision rules (per v3.0 §2 + v4.0 §3):
#   - "Cliff" := tx_per_pct > 5 % in any step where the previous step was
#     <= 1 %. Same definition the pilot used (no quantitative shift).
#   - If Pass A has no cliff and Pass C has the legacy cliff
#       => Python loop stutter at 20 ms cadence FALSIFIED as a root cause
#          ONLY IF the difference is the cadence (not the LBT setting).
#          Since A and C both have LBT off, the only differing variable is
#          inter_cycle_s. Conclusion: the cliff is a host-cadence artifact.
#   - If Pass B has a cliff and Pass A does not AND
#       radio_tx_abort_lbt_delta jumps at the cliff step in B
#       => LBT-defer CONFIRMED as the root cause.
#   - If A == B == C cliff present and lbt_delta flat
#       => host cadence falsified, LBT falsified; the cliff is firmware-
#          or RF-physical (PA compression, supply sag, cal table miss).
#
# Hardware budget: 3 passes x 16 dBm x 50 packets x ToA ~30 ms
#   ~= 3 x 16 x 50 x 0.03 = 72 s of airtime + CFG_SET overhead + 5 s RX teardown
#   per pass. Total wall time ~ 5 min.

param(
  [string]$BoardA = "2D0A1209DABC240B",   # TX
  [string]$BoardB = "2E2C1209DABC240B",   # RX
  [int]   $PerStepCount = 50,
  [int]   $PowerMin = 2,
  [int]   $PowerMax = 17,
  [int]   $PowerStep = 1
)

$ErrorActionPreference = "Stop"
# P2-3: adb prints a transfer summary to stderr on success, which PowerShell
# surfaces as a NativeCommandError record. Under -ErrorActionPreference=Stop
# that aborts mid-matrix. Restore EAP to 'Continue' only around adb pulls
# (we keep 'Stop' for all real failures).
$adbEAP = 'Continue'
$adb = "C:\Users\dorkm\AppData\Local\Microsoft\WinGet\Packages\Google.PlatformTools_Microsoft.Winget.Source_8wekyb3d8bbwe\platform-tools\adb.exe"

# 2026-05-22 P5 (Phase 4): dot-source the sidecar progress tailer so the
# long-running TX walk_power probe's per-step progress is visible in the
# orchestrator console (otherwise the foreground `& adb shell python3 ...`
# blocks silently for several minutes per pass). Helper is best-effort:
# if it can't be loaded the orchestrator still works, just without the
# live progress lines.
$_rptHelper = Join-Path $PSScriptRoot "RemoteProgressTail.ps1"
if (Test-Path -LiteralPath $_rptHelper) { . $_rptHelper } else {
    Write-Warning "RemoteProgressTail.ps1 not found at $_rptHelper; sidecar tailing disabled."
}

$runStartTs = Get-Date -Format 'yyyy-MM-dd_HHmmss'
$runUuid    = [guid]::NewGuid().ToString()
try { $gitSha = (& git rev-parse --short HEAD 2>$null).Trim() } catch { $gitSha = 'unknown' }
if (-not $gitSha) { $gitSha = 'unknown' }

$matrixRoot = "$PSScriptRoot\..\DESIGN-CONTROLLER\bench-evidence\walk_power_matrix_$runStartTs"
New-Item -ItemType Directory -Force -Path $matrixRoot | Out-Null

@{
  run_start_ts_local   = $runStartTs
  run_uuid             = $runUuid
  git_sha_short        = $gitSha
  orchestrator         = 'walk_power_falsification_matrix.ps1'
  boardA_tx            = $BoardA
  boardB_rx            = $BoardB
  per_step_count       = $PerStepCount
  power_min            = $PowerMin
  power_max            = $PowerMax
  power_step           = $PowerStep
  passes = @(
    @{ name="A"; lbt=0; inter_cycle_s=0.05; rationale="Python loop stutter isolation (LBT off, slow cadence)" },
    @{ name="B"; lbt=1; inter_cycle_s=0.05; rationale="LBT-defer hypothesis test (LBT on, slow cadence)" },
    @{ name="C"; lbt=0; inter_cycle_s=0.02; rationale="Legacy cadence reproduction (LBT off, 20 ms inter-packet)" }
  )
} | ConvertTo-Json -Depth 4 | Set-Content -Encoding utf8 "$matrixRoot\matrix_meta.json"

function Invoke-WalkPowerPass {
  param(
    [string]$PassName,
    [int]   $LbtEnable,
    [double]$InterCycleS,
    [string]$Rationale
  )
  $passDir = Join-Path $matrixRoot "pass_$PassName"
  New-Item -ItemType Directory -Force -Path $passDir | Out-Null
  $rxLog = Join-Path $passDir "rx_listen_board_b.log"
  $txLog = Join-Path $passDir "walk_power_board_a.log"

  Write-Host "`n##############################################################"
  Write-Host "## Pass $PassName : LBT=$LbtEnable inter_cycle_s=$InterCycleS"
  Write-Host "## $Rationale"
  Write-Host "##############################################################"

  # Start RX listener (long window: ToA + overhead, scaled by per-step count
  # and number of steps + a safety margin).
  $stepCount = [Math]::Floor(($PowerMax - $PowerMin) / $PowerStep) + 1
  $estWallS  = [int]([Math]::Ceiling(($PerStepCount * $stepCount * (0.03 + $InterCycleS)) + 15))
  Write-Host "RX listener window: $estWallS s"

  $rxArgs = @(
    "-s", $BoardB, "shell",
    "cd /tmp/lifetrac_p0c && echo fio | sudo -S python3 method_h_stage2_tx_probe_v2.py --probe rx_listen --dev /dev/ttymxc3 --baud 921600 --rx-window $estWallS 2>&1"
  )
  $rxProc = Start-Process -FilePath $adb -ArgumentList $rxArgs `
                          -RedirectStandardOutput $rxLog `
                          -NoNewWindow -PassThru

  # Wait for __W1_10B_LISTEN_READY__ (up to 15 s).
  $readyDeadline = (Get-Date).AddSeconds(15)
  while ((Get-Date) -lt $readyDeadline) {
    if (Select-String -Path $rxLog -Pattern "__W1_10B_LISTEN_READY__" -Quiet) { break }
    Start-Sleep -Milliseconds 250
  }
  if (-not (Select-String -Path $rxLog -Pattern "__W1_10B_LISTEN_READY__" -Quiet)) {
    Write-Warning "RX listener never reported READY; pass $PassName aborted."
    if (-not $rxProc.HasExited) { Stop-Process -Id $rxProc.Id -Force -ErrorAction SilentlyContinue }
    return
  }

  # Run walk_power on TX with the matrix arguments.
  $remoteCsv = "/tmp/walk_power_matrix_${PassName}.csv"
  $remoteCsvPerPkt = "/tmp/walk_power_matrix_${PassName}_perpacket.csv"
  $remoteProgress = "/tmp/lifetrac_p0c/progress_walk_${PassName}.txt"
  $txCmd = "cd /tmp/lifetrac_p0c && echo fio | sudo -S python3 method_h_stage2_tx_probe_v2.py --probe walk_power --dev /dev/ttymxc3 --baud 921600 --power-min $PowerMin --power-max $PowerMax --power-step $PowerStep --per-step-count $PerStepCount --walk-payload-len 24 --timeout 3 --inter-cycle-s $InterCycleS --lbt-enable $LbtEnable --csv-out $remoteCsv --progress-file $remoteProgress 2>&1"
  # P5 sidecar tail: start poller, run TX foreground (blocks), stop poller.
  $walkProgressMirror = Join-Path $passDir "walk_progress.log"
  $walkProgressJob = $null
  if (Get-Command Start-RemoteProgressTail -ErrorAction SilentlyContinue) {
      $walkProgressJob = Start-RemoteProgressTail `
          -AdbSerial $BoardA `
          -RemotePath $remoteProgress `
          -LocalMirrorPath $walkProgressMirror `
          -Tag "walk-$PassName"
  }
  & $adb -s $BoardA shell $txCmd 2>&1 | Out-File -Encoding utf8 -FilePath $txLog
  if ($walkProgressJob) { Stop-RemoteProgressTail -Job $walkProgressJob }

  # Stop RX listener.
  & $adb -s $BoardB shell "echo fio | sudo -S pkill -INT -f 'method_h_stage2.*rx_listen' 2>/dev/null; sleep 0.2; pgrep -af method_h_stage2 || echo no-residual"
  $sigintWaitDeadline = (Get-Date).AddSeconds(8)
  while ((Get-Date) -lt $sigintWaitDeadline) {
    if (Select-String -Path $rxLog -Pattern "__RX_LISTEN_STOPPED__|__W1_10B_LISTEN_DONE__" -Quiet) { break }
    Start-Sleep -Milliseconds 250
  }
  if (-not $rxProc.HasExited) {
    Start-Sleep -Seconds 1
    if (-not $rxProc.HasExited) { Stop-Process -Id $rxProc.Id -Force -ErrorAction SilentlyContinue }
  }

  # Pull CSVs. adb writes its progress summary to stderr even on success;
  # tolerate that here so the matrix continues to the next pass.
  $localCsv = Join-Path $passDir "walk_power_tx_side.csv"
  $localCsvPerPkt = Join-Path $passDir "walk_power_tx_side_perpacket.csv"
  $prevEAP = $ErrorActionPreference
  $ErrorActionPreference = $adbEAP
  try {
    & $adb -s $BoardA pull $remoteCsv $localCsv 2>&1 | Out-Null
    & $adb -s $BoardA pull $remoteCsvPerPkt $localCsvPerPkt 2>&1 | Out-Null
  } finally {
    $ErrorActionPreference = $prevEAP
  }

  # Quick visibility into the result.
  $rxCount = (Select-String -Path $rxLog -Pattern "__RX_FRAME__|RX_FRAME_URC:" | Measure-Object).Count
  $txStepLines = (Select-String -Path $txLog -Pattern "__WALK_POWER_STEP__" | Measure-Object).Count
  Write-Host "Pass $PassName : __WALK_POWER_STEP__ count = $txStepLines ; RX __RX_FRAME__ count = $rxCount"
}

Invoke-WalkPowerPass -PassName "A" -LbtEnable 0 -InterCycleS 0.05 -Rationale "Python loop stutter isolation"
Invoke-WalkPowerPass -PassName "B" -LbtEnable 1 -InterCycleS 0.05 -Rationale "LBT-defer hypothesis"
Invoke-WalkPowerPass -PassName "C" -LbtEnable 0 -InterCycleS 0.02 -Rationale "Legacy cadence reproduction"

Write-Host "`n=== Matrix complete: $matrixRoot ==="
Write-Host "Run the analyzer next: py -3 LifeTrac-v25/tools/walk_power_matrix_analyze.py `"$matrixRoot`""
