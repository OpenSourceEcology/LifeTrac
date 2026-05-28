$ErrorActionPreference = 'Continue'
$adb = 'C:\Users\dorkm\AppData\Local\Microsoft\WinGet\Packages\Google.PlatformTools_Microsoft.Winget.Source_8wekyb3d8bbwe\platform-tools\adb.exe'
$payload = "echo fio | sudo -S bash -lc 'cd /var/rootdirs/opt/lifetrac/DESIGN-CONTROLLER && docker compose -f docker-compose.yml -f docker-compose.video-test.yml build image_rx' 2>&1"
& $adb -s 2D0A1209DABC240B shell $payload 2>&1 | Tee-Object -FilePath C:\Users\dorkm\Documents\GitHub\LifeTrac\_bench_build.log
Write-Host "EXIT=$LASTEXITCODE"
