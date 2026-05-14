param(
    [string]$ComPort = 'COM8',
    [int]$DurationSec = 18,
    [int]$Baud = 19200,
    [string]$Parity = 'Even',
    [int]$DataBits = 8,
    [string]$StopBits = 'One',
    [switch]$DriveBoard2RomProbe
)

$adb = 'C:\Users\dorkm\AppData\Local\Microsoft\WinGet\Packages\Google.PlatformTools_Microsoft.Winget.Source_8wekyb3d8bbwe\platform-tools\adb.exe'

Write-Host ("=== Sniff {0} @ {1} {2}{3}{4} for {5}s ===" -f $ComPort, $Baud, $DataBits, ($Parity.Substring(0,1)), $StopBits, $DurationSec)

$sp = New-Object System.IO.Ports.SerialPort $ComPort, $Baud, $Parity, $DataBits, $StopBits
$sp.ReadTimeout = 200
try {
    $sp.Open()
} catch {
    Write-Host ("ERR opening port: {0}" -f $_.Exception.Message)
    exit 1
}
$sp.DiscardInBuffer()
Write-Host ("[ok] {0} opened, listening..." -f $ComPort)

if ($DriveBoard2RomProbe) {
    Write-Host '[stim] spawning adb autobaud-pulse loop (decoupled via Start-Process)'
    $adbArgs = @('-s','2E2C1209DABC240B','shell','echo fio | sudo -S sh /tmp/aggressive_rom_reentry.sh')
    $stimLog = Join-Path $env:TEMP ('adb_stim_' + [Guid]::NewGuid().ToString('N') + '.log')
    $proc = Start-Process -FilePath $adb -ArgumentList $adbArgs -NoNewWindow -PassThru -RedirectStandardOutput $stimLog -RedirectStandardError ($stimLog + '.err')
    Write-Host ("[stim] adb pid={0} log={1}" -f $proc.Id, $stimLog)
}

$start = Get-Date
$captured = New-Object 'System.Collections.Generic.List[byte]'
while (((Get-Date) - $start).TotalSeconds -lt $DurationSec) {
    Start-Sleep -Milliseconds 250
    $n = $sp.BytesToRead
    if ($n -gt 0) {
        $buf = New-Object byte[] $n
        $sp.Read($buf, 0, $n) | Out-Null
        $captured.AddRange($buf)
    }
}
$sp.Close()

Write-Host ("[done] captured {0} bytes" -f $captured.Count)
if ($captured.Count -gt 0) {
    $hex = ($captured | ForEach-Object { '{0:X2}' -f $_ }) -join ' '
    $ascii = -join ($captured | ForEach-Object { if ($_ -ge 32 -and $_ -lt 127) { [char]$_ } else { '.' } })
    Write-Host 'HEX  :'
    Write-Host $hex
    Write-Host 'ASCII:'
    Write-Host $ascii
} else {
    Write-Host '(no data on this port at this baud/format)'
}
