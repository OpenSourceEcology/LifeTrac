$patterns = @('VID_0483', 'VID_1366', 'ST-Link', 'STLink', 'STMicro', 'J-Link', 'SEGGER')

$devices = Get-PnpDevice -ErrorAction SilentlyContinue | Where-Object {
    $name = if ($_.FriendlyName) { $_.FriendlyName } else { $_.Name }
    $instance = $_.InstanceId
    foreach ($pattern in $patterns) {
        if ($name -like "*$pattern*" -or $instance -like "*$pattern*") {
            return $true
        }
    }
    return $false
}

if (-not $devices) {
    Write-Host 'No ST-Link/J-Link-class devices matched VID_0483, VID_1366, ST-Link, or J-Link.'
    exit 0
}

$devices |
    Sort-Object InstanceId |
    Select-Object @{Name='Name';Expression={ if ($_.FriendlyName) { $_.FriendlyName } else { $_.Name } }}, InstanceId, Status, Class |
    Format-Table -AutoSize -Wrap