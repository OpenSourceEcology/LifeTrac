$adb='C:\Users\dorkm\AppData\Local\Microsoft\WinGet\Packages\Google.PlatformTools_Microsoft.Winget.Source_8wekyb3d8bbwe\platform-tools\adb.exe'
Write-Host "=== Wipe /tmp/lifetrac_p0c on tractor ==="
& $adb -s 2E2C1209DABC240B shell 'echo fio | sudo -S rm -rf /tmp/lifetrac_p0c && echo CLEAR_OK'
Write-Host "---"
Write-Host "=== Flash tractor ==="
& 'C:\Users\dorkm\Documents\GitHub\LifeTrac\flash_unitA.ps1'
Write-Host "---EXIT=$LASTEXITCODE---"
