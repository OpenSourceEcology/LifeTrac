$adb='C:\Users\dorkm\AppData\Local\Microsoft\WinGet\Packages\Google.PlatformTools_Microsoft.Winget.Source_8wekyb3d8bbwe\platform-tools\adb.exe'
Write-Host "=== T+0 logs ==="
& $adb -s 2D0A1209DABC240B shell 'echo fio | sudo -S docker logs --tail 60 lifetrac-vtest-image_rx-1 2>&1'
Write-Host "=== sleep 20s ==="
Start-Sleep -Seconds 20
Write-Host "=== T+20 frames_published count ==="
& $adb -s 2D0A1209DABC240B shell 'echo fio | sudo -S docker logs lifetrac-vtest-image_rx-1 2>&1 | grep -cE "frames_published|frame_published"'
Write-Host "=== sleep 40s ==="
Start-Sleep -Seconds 40
Write-Host "=== T+60 last 80 lines ==="
& $adb -s 2D0A1209DABC240B shell 'echo fio | sudo -S docker logs --tail 80 lifetrac-vtest-image_rx-1 2>&1'
Write-Host "=== T+60 frames_published count ==="
& $adb -s 2D0A1209DABC240B shell 'echo fio | sudo -S docker logs lifetrac-vtest-image_rx-1 2>&1 | grep -cE "frames_published|frame_published"'
