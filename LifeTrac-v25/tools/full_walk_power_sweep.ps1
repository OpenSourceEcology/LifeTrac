# Full S1.4 walk_power paired sweep: 16 steps x 200 packets, inter-cycle 0.07s
$ErrorActionPreference = "Stop"
$adb = "C:\Users\dorkm\AppData\Local\Microsoft\WinGet\Packages\Google.PlatformTools_Microsoft.Winget.Source_8wekyb3d8bbwe\platform-tools\adb.exe"
$boardA = "2D0A1209DABC240B"
$boardB = "2E2C1209DABC240B"
$tag = "walk_power_full_2026-05-19"
$outDir = "$PSScriptRoot\..\DESIGN-CONTROLLER\bench-evidence\$tag"
New-Item -ItemType Directory -Force -Path $outDir | Out-Null
$rxLog = "$outDir\rx_listen_board_b.log"
$txLog = "$outDir\walk_power_board_a.log"
$csvLocal = "$outDir\walk_power_paired.csv"

Write-Host "=== Sync clocks ==="
$now = (Get-Date).ToUniversalTime().ToString("yyyy-MM-dd HH:mm:ss")
foreach ($s in @($boardA, $boardB)) {
  & $adb -s $s shell "echo fio | sudo -S date -u -s '$now'" | Out-Null
}

Write-Host "=== Kill any leftover python on board B ==="
& $adb -s $boardB shell "echo fio | sudo -S pkill -f method_h_stage2 2>/dev/null; sleep 0.5"

# Budget: 16 steps * 200 packets * (25.7ms toa + 70ms inter) = ~306s, + per-step overhead ~3s
# RX window 480s gives margin.
Write-Host "`n=== Start rx_listen on board B (480s window) ==="
$rxProc = Start-Process -FilePath $adb -ArgumentList @('-s',$boardB,'shell','cd /tmp/lifetrac_p0c && echo fio | sudo -S python3 method_h_stage2_tx_probe_v2.py --probe rx_listen --dev /dev/ttymxc3 --baud 921600 --rx-window 480 2>&1') -RedirectStandardOutput $rxLog -NoNewWindow -PassThru
Write-Host "rx adb PID=$($rxProc.Id); waiting 6s for LISTEN_READY..."
Start-Sleep -Seconds 6
if (-not (Select-String -Path $rxLog -Pattern "__W1_10B_LISTEN_READY__" -Quiet)) {
  Write-Host "ERROR: rx_listen not ready. Tail:"; Get-Content $rxLog -Tail 30; exit 1
}
Write-Host "RX ready."

Write-Host "`n=== Run walk_power on board A: 2..17 dBm x 200/step, inter 0.07s ==="
$swStart = Get-Date
& $adb -s $boardA shell "cd /tmp/lifetrac_p0c && echo fio | sudo -S python3 method_h_stage2_tx_probe_v2.py --probe walk_power --dev /dev/ttymxc3 --baud 921600 --power-min 2 --power-max 17 --power-step 1 --per-step-count 200 --walk-payload-len 24 --timeout 3 --inter-cycle-s 0.07 --csv-out /tmp/walk_power_full.csv 2>&1" *> $txLog
Write-Host "walk_power finished in $([math]::Round(((Get-Date)-$swStart).TotalSeconds,1))s"

Select-String -Path $txLog -Pattern "__WALK_POWER_DONE__|FATAL|Traceback"

Write-Host "`n=== Wait 8s for last RX packets ==="
Start-Sleep -Seconds 8

Write-Host "`n=== Stop rx_listen ==="
& $adb -s $boardB shell "echo fio | sudo -S pkill -INT -f 'method_h_stage2.*rx_listen' 2>/dev/null; sleep 2"
if (-not $rxProc.HasExited) { Start-Sleep -Seconds 2; if (-not $rxProc.HasExited) { Stop-Process -Id $rxProc.Id -Force -ErrorAction SilentlyContinue } }

Write-Host "`n=== Pull TX CSV ==="
& $adb -s $boardA pull /tmp/walk_power_full.csv $csvLocal | Out-Null

Write-Host "`n=== Run analyser (on board A - host python missing) ==="
& $adb -s $boardA push "$PSScriptRoot\analyse_paired_sweep.py" /tmp/analyse_paired_sweep.py | Out-Null
& $adb -s $boardA push $rxLog /tmp/rx_listen_board_b.log | Out-Null
& $adb -s $boardA shell "python3 /tmp/analyse_paired_sweep.py /tmp/walk_power_full.csv /tmp/rx_listen_board_b.log /tmp/paired_sweep_join.csv"
& $adb -s $boardA pull /tmp/paired_sweep_join.csv "$outDir\paired_sweep_join.csv" | Out-Null

# FCC-B2-b-b-3-3-3: stamp + lint gate (matches mixed_load_soak.ps1 pattern
# introduced in b-b-3-3-2). Profile enum 0 (REG_PROFILE_BENCH_ONLY_FIXED_915)
# per artifact_profile_map.json entry for walk_power_full — this soak runs
# on a single fixed channel per the 2026-05-19 FCC plan §5 #3 BENCH_ONLY
# callout. Stamp every artifact this run produced; soft-fail per file so a
# stamper error can't erase already-captured bench evidence. The lint gate
# below then converts any unstamped/corrupt/missing-fields/wrong-schema-ver
# artifact in $outDir into a hard exit so an unstamped run can never ship.
$stamper     = Join-Path $PSScriptRoot 'artifact_header.py'
$profileEnum = 0
$joinCsv     = "$outDir\paired_sweep_join.csv"
foreach ($p in @($txLog, $rxLog, $csvLocal, $joinCsv)) {
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
# carry the readout line; $csvLocal/$joinCsv are firmware/analysis
# dumps and are intentionally not checked here.
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
Get-ChildItem $outDir | Select-Object Name, Length
