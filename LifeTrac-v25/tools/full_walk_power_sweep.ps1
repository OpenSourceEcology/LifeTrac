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

Write-Host "`n=== Files ==="
Get-ChildItem $outDir | Select-Object Name, Length
