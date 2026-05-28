$adb='C:\Users\dorkm\AppData\Local\Microsoft\WinGet\Packages\Google.PlatformTools_Microsoft.Winget.Source_8wekyb3d8bbwe\platform-tools\adb.exe'
# Build using just the base compose so the build: . directive is honored
$payload = "echo fio | sudo -S bash -lc 'cd /var/rootdirs/opt/lifetrac/DESIGN-CONTROLLER && docker compose build image_rx 2>&1 | tail -80'"
& $adb -s 2D0A1209DABC240B shell $payload
Write-Host "---EXIT=$LASTEXITCODE---"
& $adb -s 2D0A1209DABC240B shell 'echo fio | sudo -S docker images'
