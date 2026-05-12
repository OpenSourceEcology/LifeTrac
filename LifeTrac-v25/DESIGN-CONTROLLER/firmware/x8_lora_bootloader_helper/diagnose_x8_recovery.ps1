# diagnose_x8_recovery.ps1
# Read-only triage for an unreachable Portenta X8 board.
# Usage:
#   .\diagnose_x8_recovery.ps1 -AdbSerial 2D0A1209DABC240B
#
# Performs Tier-0 and Tier-1 host-side checks from the recovery plan
# at LifeTrac-v25/AI NOTES/2026-05-12_X8_Board1_Recovery_Plan_Copilot_v1_0.md
# and prints a one-line TIER verdict telling the user which level of the
# §3 hierarchy must be escalated to. Never reboots, flashes, or writes
# anything to the X8.

[CmdletBinding()]
param(
    [Parameter(Mandatory = $true)]
    [string]$AdbSerial,

    [int]$ComBaud = 115200,
    [int]$ComProbeTimeoutMs = 1500
)

$ErrorActionPreference = 'Stop'
Set-StrictMode -Version Latest

function Write-Section($title) {
    Write-Host ''
    Write-Host ('=' * 60) -ForegroundColor Cyan
    Write-Host $title -ForegroundColor Cyan
    Write-Host ('=' * 60) -ForegroundColor Cyan
}

# ---------------------------------------------------------------
# Step 1: adb daemon restart + devices listing
# ---------------------------------------------------------------
Write-Section 'Step 1: adb kill-server / start-server / devices'
& adb kill-server | Out-Null
Start-Sleep -Seconds 1
& adb start-server | Out-Null
Start-Sleep -Seconds 2
$adbDevicesRaw = & adb devices
$adbDevicesRaw | ForEach-Object { Write-Host "  $_" }
$adbListed = ($adbDevicesRaw -join "`n") -match [regex]::Escape($AdbSerial)
if ($adbListed) {
    Write-Host ("Board {0} IS listed by adb." -f $AdbSerial) -ForegroundColor Green
} else {
    Write-Host ("Board {0} is NOT listed by adb." -f $AdbSerial) -ForegroundColor Yellow
}

# ---------------------------------------------------------------
# Step 2: Windows PnP enumeration for this serial
# ---------------------------------------------------------------
Write-Section 'Step 2: Windows PnP enumeration (VID_2341 X8 + VID_1366 OB J-Link)'

# Arduino X8 composite (VID_2341 PID_0061): the InstanceId of the parent
# Composite Device contains the serial verbatim. Sub-interfaces (MI_00 / MI_02)
# share an enumerated PnP node id (e.g. 6&27FE2CFF&0&0000) that we discover
# from the parent's children.
$x8Parent = Get-PnpDevice -PresentOnly | Where-Object {
    $_.InstanceId -match ('USB\\VID_2341&PID_0061\\' + [regex]::Escape($AdbSerial) + '$')
}

$x8Children = @()
if ($x8Parent) {
    Write-Host ("Found X8 USB Composite Device for {0}:" -f $AdbSerial) -ForegroundColor Green
    Write-Host ("  Status={0}  Problem={1}" -f $x8Parent.Status, $x8Parent.Problem)

    # Pull child interfaces. PowerShell 5.1 / Windows 10+ supports
    # Get-PnpDeviceProperty for DEVPKEY_Device_Children.
    try {
        $childrenProp = Get-PnpDeviceProperty -InstanceId $x8Parent.InstanceId -KeyName 'DEVPKEY_Device_Children' -ErrorAction Stop
        $x8Children = @($childrenProp.Data) | ForEach-Object { Get-PnpDevice -InstanceId $_ -ErrorAction SilentlyContinue }
    } catch {
        # Fallback: scan all VID_2341 sub-interfaces and match the parent's
        # child-key fragment heuristically.
        $x8Children = Get-PnpDevice -PresentOnly | Where-Object {
            $_.InstanceId -match 'USB\\VID_2341&PID_0061&MI_'
        }
    }

    foreach ($c in $x8Children) {
        if ($null -eq $c) { continue }
        Write-Host ("  - {0,-25} Status={1} Class={2}" -f $c.FriendlyName, $c.Status, $c.Class)
    }
} else {
    Write-Host ("No X8 USB Composite Device present for {0}." -f $AdbSerial) -ForegroundColor Red
    Write-Host '  (USB layer is dead -- escalate straight to Tier 2 hardware reset.)' -ForegroundColor Yellow
}

# Max Carrier on-board J-Link (Segger VID_1366 PID_0105) is shared across
# all carriers and not strictly tied to the X8 serial; report it for
# situational awareness.
$jlink = Get-PnpDevice -PresentOnly | Where-Object {
    $_.InstanceId -match 'USB\\VID_1366&PID_0105'
} | Select-Object -First 4
$carrierAlive = $false
if ($jlink) {
    $carrierAlive = $true
    Write-Host 'Max Carrier OB J-Link interfaces present (BMP is alive):' -ForegroundColor Green
    $jlink | ForEach-Object { Write-Host ("  - {0}  ({1})" -f $_.FriendlyName, $_.InstanceId) -ForegroundColor DarkGray }
} else {
    Write-Host 'No Max Carrier OB J-Link interfaces enumerated.' -ForegroundColor Yellow
    Write-Host '  -> Carrier may not be powered, OR carrier USB-C is not plugged into host.' -ForegroundColor Yellow
    Write-Host '  -> See recovery plan Tier 2.5 (carrier sanity check).' -ForegroundColor Yellow
}

# i.MX 8M Mini Serial Download Protocol (mask-ROM) presence
# VID_1FC9 = NXP. PID varies by mask-ROM rev: PID_012B (older) or PID_0134
# (newer; observed on Board 1 2026-05-12 recovery). Appears only when BOOT
# DIPs put the SoC into SDP mode at power-on (the canonical "interrupt boot
# before Linux loads" state).
$sdp = Get-PnpDevice -PresentOnly | Where-Object {
    $_.InstanceId -match 'USB\\VID_1FC9&PID_(012B|0134)'
}
$sdpPresent = $false
if ($sdp) {
    $sdpPresent = $true
    Write-Host 'i.MX 8M Mini SDP (mask-ROM) device PRESENT -- ready for uuu reflash.' -ForegroundColor Green
    $sdp | ForEach-Object { Write-Host ("  - {0}  Status={1}" -f $_.FriendlyName, $_.Status) -ForegroundColor Green }
} else {
    Write-Host 'No i.MX 8M Mini SDP device present (BOOT DIPs are OFF or board not in SDP mode).' -ForegroundColor DarkGray
}

# ---------------------------------------------------------------
# Step 3: COM serial probe (USB Serial Device under the X8 composite)
# ---------------------------------------------------------------
Write-Section 'Step 3: COM serial probe of X8 USB Serial Device'

$comName = $null
foreach ($c in $x8Children) {
    if ($null -eq $c) { continue }
    if ($c.FriendlyName -match 'USB Serial Device \((COM\d+)\)') {
        $comName = $matches[1]
        break
    }
}

$comAlive = $false
if ($comName) {
    Write-Host ("Found {0} for X8 -- attempting open at {1} 8N1..." -f $comName, $ComBaud)
    $port = New-Object System.IO.Ports.SerialPort $comName, $ComBaud, None, 8, One
    $port.ReadTimeout = $ComProbeTimeoutMs
    $port.WriteTimeout = $ComProbeTimeoutMs
    $port.DtrEnable = $true
    $port.RtsEnable = $true
    try {
        $port.Open()
        try { $port.WriteLine('') } catch {}
        Start-Sleep -Milliseconds 200
        try { $port.WriteLine('uname -a') } catch { Write-Host ("  WriteLine timed out: {0}" -f $_.Exception.Message) -ForegroundColor Yellow }
        Start-Sleep -Milliseconds 600
        $buf = ''
        try { while ($true) { $b = $port.ReadByte(); $buf += [char]$b } } catch {}
        if ($buf.Length -gt 0) {
            $comAlive = $true
            Write-Host '  COM responded with:' -ForegroundColor Green
            $buf -split "`r?`n" | ForEach-Object { if ($_ -ne '') { Write-Host ("    > {0}" -f $_) } }
        } else {
            Write-Host '  COM opened but NO bytes received (Linux getty not servicing the port).' -ForegroundColor Yellow
        }
    } catch {
        Write-Host ("  COM open/probe failed: {0}" -f $_.Exception.Message) -ForegroundColor Yellow
    } finally {
        if ($port.IsOpen) { $port.Close() }
    }
} else {
    Write-Host 'No USB Serial Device sub-interface present for this X8.' -ForegroundColor Yellow
}

# ---------------------------------------------------------------
# Verdict
# ---------------------------------------------------------------
Write-Section 'TIER verdict'

if ($adbListed) {
    Write-Host 'TIER 0 -- Board is reachable via adb. No recovery needed.' -ForegroundColor Green
    Write-Host '  Suggested next: run your normal pipeline (e.g. run_w1_10b_rx_pair_end_to_end.ps1).'
    exit 0
}

if ($sdpPresent) {
    Write-Host 'TIER 3 READY -- i.MX SDP mode is ACTIVE. Proceed with uuu reflash:' -ForegroundColor Green
    Write-Host '  cd <mfgtool-files-portenta-x8>'
    Write-Host '  .\uuu.exe .\full_image.uuu'
    Write-Host '  Then set both BOOT DIPs OFF and power-cycle.'
    Write-Host '  See LifeTrac-v25/AI NOTES/2026-05-12_X8_Board1_Recovery_Plan_Copilot_v1_0.md Tier 3 for full procedure.'
    exit 3
}

if (-not $carrierAlive) {
    Write-Host 'TIER 2.5 -- Max Carrier BMP not enumerating. Verify BEFORE Tier 3:' -ForegroundColor Red
    Write-Host '  - Carrier on its OWN USB-C cable into the host (not just the X8 USB-C).'
    Write-Host '  - 12V barrel plugged in; carrier power LED lit.'
    Write-Host '  - Re-run this script and confirm 2+ VID_1366&PID_0105 CDC ports appear.'
    Write-Host 'If carrier still does not enumerate, the carrier itself may be at fault, not Linux.'
    exit 25
}

if (-not $x8Parent) {
    Write-Host 'TIER 2 -- USB enumeration FAILED entirely. Try:' -ForegroundColor Red
    Write-Host '  4) Press the Max Carrier RESET button.'
    Write-Host '  5) Re-plug USB-C only (use a USB-C to USB-A cable).'
    Write-Host '  6) Full power-cycle: unplug USB-C AND 12V barrel for >=10 s.'
    Write-Host 'If still missing after Tier 2, escalate to Tier 3 (uuu reflash).'
    exit 2
}

if ($comAlive) {
    Write-Host 'TIER 1 -- USB enumerated and serial console is alive, but adb daemon is dead.' -ForegroundColor Yellow
    Write-Host '  Log in over the COM serial (fio / fio) and run: sudo systemctl restart adbd'
    Write-Host '  If adbd is missing or wedged, sudo reboot will recover.'
    exit 1
}

Write-Host 'TIER 3 RECOMMENDED -- USB enumerated but Linux user-space is hung' -ForegroundColor Red
Write-Host '  (adbd + getty both unresponsive). Repeated power-cycles will reproduce the same hang' -ForegroundColor Red
Write-Host '  because the wedge is *during* Linux boot. Escalate directly to Tier 3:' -ForegroundColor Red
Write-Host ''
Write-Host '  1) Power off X8 completely (USB-C unplug + 12V barrel unplug).'
Write-Host '  2) Set Max Carrier BOOT SEL + BOOT DIPs to ON (puts i.MX into mask-ROM SDP mode).'
Write-Host '  3) Power on (USB-C from host, 12V barrel).'
Write-Host '  4) Re-run this script -- it should now report TIER 3 READY (SDP device present).'
Write-Host '  5) cd into mfgtool-files-portenta-x8 and run: .\uuu.exe .\full_image.uuu'
Write-Host '  6) After flash completes: power off, BOOT DIPs back to OFF, power on.'
Write-Host ''
Write-Host '  Full procedure: LifeTrac-v25/AI NOTES/2026-05-12_X8_Board1_Recovery_Plan_Copilot_v1_0.md (Tier 3).'
exit 2
