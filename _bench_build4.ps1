$adb='C:\Users\dorkm\AppData\Local\Microsoft\WinGet\Packages\Google.PlatformTools_Microsoft.Winget.Source_8wekyb3d8bbwe\platform-tools\adb.exe'
$payload = "echo fio | sudo -S bash -lc 'docker builder prune -af 2>&1 | tail -10 && echo ---PRUNE_DONE--- && cd /var/rootdirs/opt/lifetrac/DESIGN-CONTROLLER && docker compose build --no-cache lora_bridge 2>&1 | tail -60'"
& $adb -s 2D0A1209DABC240B shell $payload
Write-Host "---EXIT=$LASTEXITCODE---"
& $adb -s 2D0A1209DABC240B shell 'echo fio | sudo -S docker images'
