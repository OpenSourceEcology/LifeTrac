$adb='C:\Users\dorkm\AppData\Local\Microsoft\WinGet\Packages\Google.PlatformTools_Microsoft.Winget.Source_8wekyb3d8bbwe\platform-tools\adb.exe'
& $adb -s 2D0A1209DABC240B shell 'grep -nE "services:|^  [a-z_]+:|build:|image:" /var/rootdirs/opt/lifetrac/DESIGN-CONTROLLER/docker-compose.yml'
