# Paired LoRa sweep: rx_listen on board B + walk_power on board A
$ErrorActionPreference = "Stop"
$adb = "C:\Users\dorkm\AppData\Local\Microsoft\WinGet\Packages\Google.PlatformTools_Microsoft.Winget.Source_8wekyb3d8bbwe\platform-tools\adb.exe"
$boardA = "2D0A1209DABC240B"  # TX
$boardB = "2E2C1209DABC240B"  # RX
$outDir = "$PSScriptRoot\..\DESIGN-CONTROLLER\bench-evidence\walk_power_pilot_2026-05-19"
New-Item -ItemType Directory -Force -Path $outDir | Out-Null
$rxLog = "$outDir\rx_listen_board_b.log"
$txLog = "$outDir\walk_power_board_a.log"
$csvLocal = "$outDir\walk_power_paired.csv"

Write-Host "=== Pre-flight: confirm both boards reachable ==="
& $adb devices

# Kill any leftover python on board B
& $adb -s $boardB shell "echo fio | sudo -S pkill -f method_h_stage2 2>/dev/null; sleep 0.5; pgrep -af method_h_stage2 || echo no-residual"

Write-Host "`n=== Starting rx_listen on board B (120s window) ==="
$rxProc = Start-Process -FilePath $adb -ArgumentList @('-s',$boardB,'shell','cd /tmp/lifetrac_p0c && echo fio | sudo -S python3 method_h_stage2_tx_probe_v2.py --probe rx_listen --dev /dev/ttymxc3 --baud 921600 --rx-window 120 2>&1') -RedirectStandardOutput $rxLog -NoNewWindow -PassThru
Write-Host "rx adb PID=$($rxProc.Id); waiting 6s for LISTEN_READY..."
Start-Sleep -Seconds 6
$rxReady = Select-String -Path $rxLog -Pattern "__W1_10B_LISTEN_READY__" -Quiet
if (-not $rxReady) {
  Write-Host "ERROR: rx_listen did not signal READY. Tail:"
  Get-Content $rxLog -Tail 30
  exit 1
}
Write-Host "RX ready. Tail:"
Get-Content $rxLog -Tail 8

Write-Host "`n=== Running walk_power on board A: 2..17 dBm step 1, 50 packets/step, payload 24B ==="
$swStart = Get-Date
& $adb -s $boardA shell "cd /tmp/lifetrac_p0c && echo fio | sudo -S python3 method_h_stage2_tx_probe_v2.py --probe walk_power --dev /dev/ttymxc3 --baud 921600 --power-min 2 --power-max 17 --power-step 1 --per-step-count 50 --walk-payload-len 24 --timeout 3 --inter-cycle-s 0.02 --csv-out /tmp/walk_power_paired.csv 2>&1" *> $txLog
$swEnd = Get-Date
Write-Host "walk_power finished in $([math]::Round(($swEnd-$swStart).TotalSeconds,1))s"

Write-Host "`n=== Walk power verdict lines ==="
Select-String -Path $txLog -Pattern "__WALK_POWER_DONE__|__WALK_POWER_VERDICT__|FATAL|Traceback"

Write-Host "`n=== Waiting 5s for any in-flight RX packets to land ==="
Start-Sleep -Seconds 5

Write-Host "`n=== Stopping rx_listen on board B ==="
& $adb -s $boardB shell "echo fio | sudo -S pkill -INT -f 'method_h_stage2.*rx_listen' 2>/dev/null; sleep 1; pgrep -af method_h_stage2 || echo no-residual"
if (-not $rxProc.HasExited) {
  Start-Sleep -Seconds 3
  if (-not $rxProc.HasExited) { Stop-Process -Id $rxProc.Id -Force -ErrorAction SilentlyContinue }
}

Write-Host "`n=== Pulling walk_power CSV from board A ==="
& $adb -s $boardA pull /tmp/walk_power_paired.csv $csvLocal

Write-Host "`n=== RX summary (board B) ==="
$rxFrames = Select-String -Path $rxLog -Pattern "RX_FRAME_URC:" | Measure-Object | Select-Object -ExpandProperty Count
Write-Host "RX_FRAME_URC count = $rxFrames"
$tail = Get-Content $rxLog -Tail 15
$tail | ForEach-Object { Write-Host $_ }

Write-Host "`n=== TX CSV head ==="
Get-Content $csvLocal -TotalCount 5

# FCC-B2-b-b-3-3-3: stamp + lint gate (matches mixed_load_soak.ps1 pattern
# introduced in b-b-3-3-2). Profile enum 0 (REG_PROFILE_BENCH_ONLY_FIXED_915)
# per artifact_profile_map.json entry for walk_power_pilot — this sweep runs
# on a single fixed channel per the 2026-05-19 FCC plan §5 #3 BENCH_ONLY
# callout. Stamp every artifact this run produced; soft-fail per file so a
# stamper error can't erase already-captured bench evidence. The lint gate
# below then converts any unstamped/corrupt/missing-fields/wrong-schema-ver
# artifact in $outDir into a hard exit so an unstamped run can never ship.
$stamper     = Join-Path $PSScriptRoot 'artifact_header.py'
$profileEnum = 0
foreach ($p in @($txLog, $rxLog, $csvLocal)) {
  if (Test-Path $p) {
    & py -3 $stamper stamp --profile-enum $profileEnum --input $p --output $p
    if ($LASTEXITCODE -ne 0) {
      Write-Host "[stamp] WARN: artifact_header.py exit=$LASTEXITCODE on $p"
    } else {
      Write-Host "[stamp] header prepended to $(Split-Path -Leaf $p)"
    }
  }
}
$linter = Join-Path $PSScriptRoot 'lint_artifact_headers.py'
& py -3 $linter $outDir
$lintRc = $LASTEXITCODE
if ($lintRc -ne 0) {
  Write-Host "[lint] FAIL: lint_artifact_headers.py exit=$lintRc on $outDir"
  exit 3
}
Write-Host "[lint] OK: every text artifact in $outDir carries a v1 FCC-B2-b header"

# FCC-B3-3: runtime-profile gate. After header lint passes, verify the
# firmware actually running on each board matches $profileEnum. The
# probe (FCC-B3-1) emits exactly one RUNTIME_PROFILE_ENUM=<N> line per
# log; the checker (FCC-B3-2) exits 0=match, 1=mismatch, 4=probe
# regression (zero/multi/ERR readout). Any non-zero is a hard failure;
# we propagate exit 4 so the orchestrator's exit carries the gate's
# verdict (B3-0 contract: 4 reserved for this gate, distinct from 3
# used by the header lint). Only $txLog/$rxLog are probe-produced and
# carry the readout line; $csvLocal is a firmware CSV dump and is
# intentionally not checked here.
$profileChecker = Join-Path $PSScriptRoot 'check_run_profile.py'
foreach ($logPath in @($txLog, $rxLog)) {
  if (-not (Test-Path $logPath)) { continue }
  & py -3 $profileChecker --log $logPath --expected-enum $profileEnum
  $profRc = $LASTEXITCODE
  if ($profRc -ne 0) {
    Write-Host "[profile-gate] FAIL: check_run_profile.py exit=$profRc on $logPath (expected enum $profileEnum)"
    exit 4
  }
}
Write-Host "[profile-gate] OK: every probe log reports RUNTIME_PROFILE_ENUM=$profileEnum"

Write-Host "`n=== Files ==="
Write-Host "  rx log:  $rxLog"
Write-Host "  tx log:  $txLog"
Write-Host "  tx csv:  $csvLocal"
