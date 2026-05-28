$adb='C:\Users\dorkm\AppData\Local\Microsoft\WinGet\Packages\Google.PlatformTools_Microsoft.Winget.Source_8wekyb3d8bbwe\platform-tools\adb.exe'
$serial='2E2C1209DABC240B'
& $adb -s $serial push C:\Users\dorkm\Documents\GitHub\LifeTrac\LifeTrac-v25\tools\_flip_tractor_compose_to_fhss.py /tmp/_flip_tractor.py
& $adb -s $serial push C:\Users\dorkm\Documents\GitHub\LifeTrac\_tractor_flip_remote.sh /tmp/_tractor_flip.sh
& $adb -s $serial shell 'chmod +x /tmp/_tractor_flip.sh; bash /tmp/_tractor_flip.sh'
