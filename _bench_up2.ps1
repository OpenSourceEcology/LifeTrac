# DO NOT RUN on a bench board, not even with a radio GO: over adb, force-removes
# design-controller-mosquitto-1 (the bench broker) on the base, then starts
# image_rx under project lifetrac-vtest. image_rx opens /dev/ttymxc3 with a
# RESET_REQ that un-parks the L072.
# May-2026 one-off; see BENCH_BOARDS.md "Retired boot units", rule 4.
$adb='C:\Users\dorkm\AppData\Local\Microsoft\WinGet\Packages\Google.PlatformTools_Microsoft.Winget.Source_8wekyb3d8bbwe\platform-tools\adb.exe'
# Cleanup wrongly-named containers
& $adb -s 2D0A1209DABC240B shell 'echo fio | sudo -S docker rm -f design-controller-image_rx-1 design-controller-mosquitto-1 2>&1'
Write-Host "---CLEAN_DONE---"
# Bring up using correct project name
$payload = "echo fio | sudo -S bash -lc 'cd /var/rootdirs/opt/lifetrac/DESIGN-CONTROLLER && docker compose -p lifetrac-vtest -f docker-compose.yml -f docker-compose.video-test.yml up -d image_rx 2>&1 | tail -30'"
& $adb -s 2D0A1209DABC240B shell $payload
Write-Host "---UP_DONE---"
Start-Sleep -Seconds 3
& $adb -s 2D0A1209DABC240B shell 'echo fio | sudo -S docker ps -a'
