<#
.SYNOPSIS
    Prepares host-side MQTT connectivity for a Portenta-X8 (tractor or base)
    so the strict-path image daemons can reach the broker on the Windows host.

.DESCRIPTION
    The historical recipe used 'adb -s <serial> reverse tcp:1883 tcp:1883' to
    expose the host broker as 127.0.0.1:1883 on the device. On LmP factory
    image 934-91 (verified 2026-05-24 on board 2E2C1209DABC240B) that command
    returns exit=0 but does NOT actually install the tunnel:

        adb reverse tcp:1883 tcp:1883   -> exit=0  (silent failure)
        adb reverse --list              -> protocol fault, brief device drop
        device-side: exec 9 to /dev/tcp/127.0.0.1/1883 -> Connection refused

    Worse, a host-side 'adb kill-server' after the protocol fault leaves the
    X8 adb-invisible until USB-C unplug/replug (see board 2D0A 2026-05-24).

    Workaround: Mosquitto on Windows listens on 0.0.0.0:1883, so the device
    can reach it directly via the host LAN IP. This script:
      1. Confirms the device is in 'adb devices'.
      2. Verifies Mosquitto is listening on the host (port 1883).
      3. Resolves the host LAN IP.
      4. From inside the device, confirms TCP connectivity to <HostIp>:1883.
      5. Prints the env-var line the daemons need (LIFETRAC_MQTT_HOST).

    No adb reverse, no kill-server, no risk to the device adbd.

.PARAMETER Serial
    adb serial of the target Portenta-X8 (e.g. 2E2C1209DABC240B tractor,
    2D0A1209DABC240B base).

.PARAMETER HostIp
    LAN IP of this Windows host as seen by the device. If omitted, auto-detect.

.PARAMETER AdbPath
    Absolute path to adb.exe. Defaults to the WinGet platform-tools install.

.NOTES
    DO NOT run 'adb kill-server' against a first-boot Portenta-X8 -- it can
    wedge adbd until a USB-C unplug/replug.
#>
[CmdletBinding()]
param(
    [Parameter(Mandatory)]
    [ValidatePattern('^[0-9A-Fa-f]{16}$')]
    [string]$Serial,

    [string]$HostIp,

    [string]$AdbPath = 'C:\Users\dorkm\AppData\Local\Microsoft\WinGet\Packages\Google.PlatformTools_Microsoft.Winget.Source_8wekyb3d8bbwe\platform-tools\adb.exe'
)

$ErrorActionPreference = 'Stop'

function Write-Step($status, $msg) {
    $color = switch ($status) {
        'OK'   { 'Green' }
        'WARN' { 'Yellow' }
        'FAIL' { 'Red' }
        default { 'Gray' }
    }
    Write-Host ('[{0}] {1}' -f $status, $msg) -ForegroundColor $color
}

# 1. adb available + device present
if (-not (Test-Path $AdbPath)) {
    Write-Step 'FAIL' "adb.exe not found at $AdbPath"
    exit 1
}
$devices = (& $AdbPath devices 2>&1) -join "`n"
if ($devices -notmatch [regex]::Escape($Serial)) {
    Write-Step 'FAIL' "device $Serial not in 'adb devices'. Unplug/replug USB-C; do NOT run 'adb kill-server'."
    Write-Host $devices
    exit 2
}
Write-Step 'OK' "device $Serial present"

# 2. Mosquitto listening locally
$listening = netstat -an | Select-String -Pattern ':1883\s' | Select-String -Pattern 'LISTENING'
if (-not $listening) {
    Write-Step 'FAIL' 'no LISTENING socket on :1883 -- start Mosquitto on the host first'
    exit 3
}
$firstLine = ($listening | Select-Object -First 1).Line.Trim()
Write-Step 'OK' "Mosquitto listening on $firstLine"

# 3. Resolve host LAN IP (first IPv4 with a default gateway)
if (-not $HostIp) {
    $defaultIf = Get-NetRoute -DestinationPrefix '0.0.0.0/0' -ErrorAction SilentlyContinue |
        Sort-Object -Property RouteMetric | Select-Object -First 1
    if ($defaultIf) {
        $HostIp = (Get-NetIPAddress -InterfaceIndex $defaultIf.ifIndex -AddressFamily IPv4 -ErrorAction SilentlyContinue |
            Where-Object { $_.IPAddress -notlike '169.254.*' } |
            Select-Object -First 1).IPAddress
    }
    if (-not $HostIp) {
        Write-Step 'FAIL' 'could not auto-detect host LAN IP; pass -HostIp explicitly'
        exit 4
    }
}
Write-Step 'OK' "host LAN IP = $HostIp"

# 4. Device-side TCP reachability check (no reverse, no kill-server)
$sq = [char]39
$probeInner = 'exec 9<>/dev/tcp/' + $HostIp + '/1883 && echo LAN_MQTT_OK || echo LAN_MQTT_FAIL'
$probe = 'timeout 3 bash -c ' + $sq + $probeInner + $sq
$result = & $AdbPath -s $Serial shell $probe 2>&1
if ($result -match 'LAN_MQTT_OK') {
    Write-Step 'OK' ("device -> " + $HostIp + ":1883 reachable (LAN_MQTT_OK)")
} else {
    Write-Step 'FAIL' ("device cannot reach " + $HostIp + ":1883")
    Write-Host $result
    Write-Step 'WARN' 'check Windows Defender Firewall inbound rule for port 1883 on the LAN interface'
    exit 5
}

# 5. Print the env-var the daemons need
Write-Host ''
Write-Step 'OK' "launch daemons with:  LIFETRAC_MQTT_HOST=$HostIp"
Write-Host ''
Write-Host 'Example TX dry-run (15s timeout):' -ForegroundColor Cyan
$dq = [char]34
$ex = 'adb -s ' + $Serial + ' shell ' + $dq +
      'echo fio | sudo -S -p ' + $sq + $sq +
      ' docker run --rm --network=host --device=/dev/ttymxc3' +
      ' -v /tmp/lifetrac_strict:/work -w /work' +
      ' -e PYTHONPATH=/work:/work/paho' +
      ' -e LIFETRAC_MQTT_HOST=' + $HostIp +
      ' --entrypoint timeout' +
      ' hub.foundries.io/arduino/arduino-ootb-python-devel:738bc44' +
      ' 15 python3 -u /work/image_tx_daemon.py --log-level INFO' + $dq
Write-Host $ex -ForegroundColor DarkGray
