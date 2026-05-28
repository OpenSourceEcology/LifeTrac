$adb='C:\Users\dorkm\AppData\Local\Microsoft\WinGet\Packages\Google.PlatformTools_Microsoft.Winget.Source_8wekyb3d8bbwe\platform-tools\adb.exe'
$serial='2E2C1209DABC240B'
Write-Host "=== inspect tractor compose ==="
& $adb -s $serial shell 'ls /opt/lifetrac/video-test/ 2>&1; echo ---; head -5 /opt/lifetrac/video-test/docker-compose.yml 2>&1'
Write-Host "---"
Write-Host "=== docker ps -a on tractor ==="
& $adb -s $serial shell 'echo fio | sudo -S docker ps -a'
