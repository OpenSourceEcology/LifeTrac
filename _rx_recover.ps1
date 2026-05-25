Write-Host '=== Disable/Enable RX Composite parent ==='
Disable-PnpDevice -InstanceId 'USB\VID_2341&PID_0061\2D0A1209DABC240B' -Confirm:$false -ErrorAction Continue
Start-Sleep -Seconds 3
Enable-PnpDevice -InstanceId 'USB\VID_2341&PID_0061\2D0A1209DABC240B' -Confirm:$false -ErrorAction Continue
Start-Sleep -Seconds 3
Write-Host '=== pnputil /restart-device ==='
& pnputil /restart-device 'USB\VID_2341&PID_0061\2D0A1209DABC240B' 2>&1
Start-Sleep -Seconds 2
Write-Host '=== Status after recovery ==='
Get-PnpDevice -InstanceId 'USB\VID_2341&PID_0061\2D0A1209DABC240B' | Format-List Status, Problem | Out-String
Write-Host '=== Try COM12 open ==='
try {
  $p = New-Object System.IO.Ports.SerialPort('COM12',115200,'None',8,'One')
  $p.Open(); Start-Sleep -Milliseconds 200; $p.Close()
  Write-Host 'COM12 OPENED ? RX recovered!'
} catch { Write-Host ('COM12 still fails: ' + $_.Exception.Message) }
