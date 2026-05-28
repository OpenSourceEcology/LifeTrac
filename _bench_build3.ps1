$adb='C:\Users\dorkm\AppData\Local\Microsoft\WinGet\Packages\Google.PlatformTools_Microsoft.Winget.Source_8wekyb3d8bbwe\platform-tools\adb.exe'
$payload = "echo fio | sudo -S bash -lc 'cd /var/rootdirs/opt/lifetrac/DESIGN-CONTROLLER && docker compose build lora_bridge 2>&1 | tail -40'"
& $adb -s 2D0A1209DABC240B shell $payload
Write-Host "---EXIT=$LASTEXITCODE---"
& $adb -s 2D0A1209DABC240B shell 'echo fio | sudo -S docker images'
