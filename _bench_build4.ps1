# DO NOT RUN on a bench base: over adb, deletes the whole docker build cache
# on the base (`docker builder prune -af`), then builds lora_bridge with
# --no-cache. A build that succeeds moves the tag lifetrac-v25:latest off the
# flown image, and no :previous tag is kept. Starts no container. Build with
# bench_tools/deploy_base.sh instead.
# May-2026 one-off; see BENCH_BOARDS.md "Retired boot units", rule 8.
$adb='C:\Users\dorkm\AppData\Local\Microsoft\WinGet\Packages\Google.PlatformTools_Microsoft.Winget.Source_8wekyb3d8bbwe\platform-tools\adb.exe'
$payload = "echo fio | sudo -S bash -lc 'docker builder prune -af 2>&1 | tail -10 && echo ---PRUNE_DONE--- && cd /var/rootdirs/opt/lifetrac/DESIGN-CONTROLLER && docker compose build --no-cache lora_bridge 2>&1 | tail -60'"
& $adb -s 2D0A1209DABC240B shell $payload
Write-Host "---EXIT=$LASTEXITCODE---"
& $adb -s 2D0A1209DABC240B shell 'echo fio | sudo -S docker images'
