$adb='C:\Users\dorkm\AppData\Local\Microsoft\WinGet\Packages\Google.PlatformTools_Microsoft.Winget.Source_8wekyb3d8bbwe\platform-tools\adb.exe'
$serial='2E2C1209DABC240B'
Write-Host "=== current REG_PROFILE / FHSS state on tractor ==="
& $adb -s $serial shell 'grep -nE "LIFETRAC_REG_PROFILE|LIFETRAC_FHSS_WIDE_MASK|LIFETRAC_FORCE_FRF_HZ" /opt/lifetrac/video-test/docker-compose.yml'
Write-Host "---"
Write-Host "=== push flip script ==="
& $adb -s $serial push C:\Users\dorkm\Documents\GitHub\LifeTrac\LifeTrac-v25\tools\_flip_tractor_compose_to_fhss.py /tmp/_flip_tractor.py
Write-Host "=== run flip (sudo) ==="
& $adb -s $serial shell 'echo fio | sudo -S python3 /tmp/_flip_tractor.py'
Write-Host "=== verify ==="
& $adb -s $serial shell 'grep -nE "LIFETRAC_REG_PROFILE|LIFETRAC_FHSS_WIDE_MASK|LIFETRAC_FORCE_FRF_HZ" /opt/lifetrac/video-test/docker-compose.yml'
Write-Host "---"
Write-Host "=== restart tractor-image-tx-v2 to pick up new env ==="
& $adb -s $serial shell 'echo fio | sudo -S bash -lc "cd /opt/lifetrac/video-test && docker compose up -d --force-recreate image_tx 2>&1 | tail -10"'
Start-Sleep -Seconds 3
& $adb -s $serial shell 'echo fio | sudo -S docker ps --format "table {{.Names}}\t{{.Status}}"'
