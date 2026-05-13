#requires -Version 5.1
<#
.SYNOPSIS
    W2-01 camera first-light end-to-end orchestrator.

.DESCRIPTION
    Pushes `w2_01_camera_capture.py` + `w2_01_camera_first_light.sh` to the
    target Portenta X8 board (via ADB), runs the probe (which iterates every
    /dev/video* node and tries a single MJPEG frame grab using pure-Python
    V4L2 ioctl - no v4l-utils / ffmpeg / gstreamer required, since the stock
    Foundries.io LmP image ships none of those), pulls the captured frame
    + the probe stdout back to `bench-evidence/W2-01_camera_first_light_<stamp>/`,
    parses the W2_01_PROBE / W2_01_CAMERA marker blocks, and emits a
    `summary.json` with the gates:

        G1  device_present       : at least one /dev/video* node existed
        G2  driver_uvc           : kernel reported uvcvideo driver in QUERYCAP
        G3  capture_capable      : at least one node had VIDEO_CAPTURE+STREAMING
        G4  format_negotiated    : VIDIOC_S_FMT returned a supported pixfmt
        G5  frame_captured       : DQBUF returned bytesused>0 within timeout
        G6  jpeg_sane            : MJPG payload begins FFD8 and ends FFD9
        G7  file_pulled          : adb pull recovered a non-empty file

    Designed to be invoked from the repo root on the host PC. Requires the
    target board present in `adb devices` and reachable.

.PARAMETER AdbSerial
    ADB serial of the X8 with the camera attached. The first probe in this
    session showed the camera on `2D0A1209DABC240B` (Board 1).

.PARAMETER Width
    Requested capture width (default 1280). The driver may negotiate down.

.PARAMETER Height
    Requested capture height (default 720).

.PARAMETER PixFmt
    Preferred pixel format fourcc (default `MJPG`). Falls back to whatever
    the device enumerates if the requested format is absent.

.PARAMETER RepoRoot
    Optional override for the repo root. Default: derived relative to the
    script location.

.EXAMPLE
    powershell -NoProfile -ExecutionPolicy Bypass `
        -File LifeTrac-v25/DESIGN-CONTROLLER/firmware/x8_lora_bootloader_helper/run_w2_01_camera_first_light_end_to_end.ps1 `
        -AdbSerial 2D0A1209DABC240B
#>
param(
    [Parameter(Mandatory = $true)][string]$AdbSerial,
    [int]$Width = 1280,
    [int]$Height = 720,
    [string]$PixFmt = 'MJPG',
    [switch]$UseSudo,
    [switch]$FullCapture,           # default OFF -> safe enumerate-only mode
    [switch]$NoQuirks,              # default OFF -> reload uvcvideo with FIX_BANDWIDTH
    [int]$PerAttemptTimeoutS = 8,
    [string]$RepoRoot = ''
)

Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'

$ScriptRoot = if ($PSScriptRoot) { $PSScriptRoot }
              elseif ($PSCommandPath) { Split-Path -Parent $PSCommandPath }
              else { (Get-Location).Path }

function Resolve-RepoRoot {
    if ($RepoRoot -and (Test-Path -LiteralPath $RepoRoot)) {
        return (Resolve-Path -LiteralPath $RepoRoot).Path
    }
    return (Resolve-Path (Join-Path $ScriptRoot '../../../')).Path
}

$null = Get-Command adb -ErrorAction Stop

$devicesOut = (& adb devices) | Out-String
if ($devicesOut -notmatch [regex]::Escape($AdbSerial)) {
    throw "Board '$AdbSerial' not present in 'adb devices'.`nadb devices output:`n$devicesOut"
}

$repo = Resolve-RepoRoot
$helperDir = Join-Path $repo 'DESIGN-CONTROLLER/firmware/x8_lora_bootloader_helper'
$benchEvidenceRoot = Join-Path $repo 'DESIGN-CONTROLLER/bench-evidence'
$null = New-Item -ItemType Directory -Force -Path $benchEvidenceRoot | Out-Null

$stamp = Get-Date -Format 'yyyy-MM-dd_HHmmss'
$evidenceDir = Join-Path $benchEvidenceRoot "W2-01_camera_first_light_${stamp}"
$null = New-Item -ItemType Directory -Force -Path $evidenceDir | Out-Null
Write-Host "Evidence dir: $evidenceDir"

$pyLocal  = Join-Path $helperDir 'w2_01_camera_capture.py'
$shLocal  = Join-Path $helperDir 'w2_01_camera_first_light.sh'
foreach ($p in @($pyLocal, $shLocal)) {
    if (-not (Test-Path -LiteralPath $p)) {
        throw "Missing local file: $p"
    }
}

$pyRemote   = '/tmp/w2_01_camera_capture.py'
$shRemote   = '/tmp/w2_01_camera_first_light.sh'
$snapRemote = '/tmp/w2_01_snap.jpg'

# 1. Push helper files
Write-Host "Pushing helper files to $AdbSerial ..."
& adb -s $AdbSerial push $pyLocal $pyRemote | Out-Null
& adb -s $AdbSerial push $shLocal $shRemote | Out-Null
& adb -s $AdbSerial shell "chmod +x $pyRemote $shRemote; rm -f $snapRemote" | Out-Null

# 2. Run the probe and capture stdout as raw UTF-8 (avoid PowerShell's
#    default UTF-16 redirect that corrupts byte-oriented adb output).
$probeStdoutPath = Join-Path $evidenceDir 'probe_stdout.txt'
Write-Host "Running probe ..."
$psi = New-Object System.Diagnostics.ProcessStartInfo
$psi.FileName = (Get-Command adb).Source
$psi.Arguments = "-s $AdbSerial exec-out `"bash --noprofile --norc $shRemote $pyRemote $snapRemote $Width $Height $([int][bool]$UseSudo) $PerAttemptTimeoutS $([int](-not $FullCapture)) $([int](-not $NoQuirks))`""
$psi.RedirectStandardOutput = $true
$psi.RedirectStandardError = $true
$psi.UseShellExecute = $false
$psi.StandardOutputEncoding = [System.Text.Encoding]::UTF8
$psi.StandardErrorEncoding = [System.Text.Encoding]::UTF8
$proc = [System.Diagnostics.Process]::Start($psi)
$probeStdout = $proc.StandardOutput.ReadToEnd()
$probeStderr = $proc.StandardError.ReadToEnd()
$proc.WaitForExit()
$probeExit = $proc.ExitCode
[System.IO.File]::WriteAllText($probeStdoutPath, $probeStdout, [System.Text.UTF8Encoding]::new($false))
if ($probeStderr) {
    [System.IO.File]::WriteAllText((Join-Path $evidenceDir 'probe_stderr.txt'), $probeStderr, [System.Text.UTF8Encoding]::new($false))
}
Write-Host "Probe exit: $probeExit  ->  $probeStdoutPath"

# 3. Pull the captured frame (if any). Wrap in try/catch because adb pull
#    writes to stderr when the remote file does not exist, which under our
#    ErrorActionPreference='Stop' would otherwise abort the orchestrator
#    before we could write summary.json.
$snapLocal = Join-Path $evidenceDir 'snap.jpg'
try {
    & adb -s $AdbSerial pull $snapRemote $snapLocal 2>&1 | Out-Null
} catch {
    Write-Host "adb pull non-fatal: $_"
}
$snapExists = Test-Path -LiteralPath $snapLocal
$snapSize   = if ($snapExists) { (Get-Item -LiteralPath $snapLocal).Length } else { 0 }
Write-Host "Pulled snap: exists=$snapExists size=$snapSize"

# 4. Parse marker blocks (strip ANSI escape sequences first - the X8's
#    bash startup may emit ESC[?1l / ESC[6n cursor queries even under exec-out).
$probeText = Get-Content -LiteralPath $probeStdoutPath -Raw -Encoding UTF8
$probeText = [regex]::Replace($probeText, "\x1b\[[0-9;?]*[a-zA-Z]", "")
function Get-MarkerBlock([string]$text, [string]$beginMarker, [string]$endMarker) {
    $startIdx = $text.IndexOf($beginMarker)
    $endIdx   = $text.IndexOf($endMarker)
    if ($startIdx -lt 0 -or $endIdx -lt 0 -or $endIdx -lt $startIdx) { return $null }
    $startLineEnd = $text.IndexOf("`n", $startIdx)
    if ($startLineEnd -lt 0) { return $null }
    return $text.Substring($startLineEnd + 1, $endIdx - ($startLineEnd + 1))
}
function Parse-KV([string]$block) {
    $h = @{}
    if (-not $block) { return $h }
    foreach ($line in $block -split "`r?`n") {
        if ($line -match '^([A-Za-z0-9_]+)=(.*)$') {
            $h[$Matches[1]] = $Matches[2]
        }
    }
    return $h
}

$probeBlock  = Get-MarkerBlock $probeText '__W2_01_PROBE_BEGIN__' '__W2_01_PROBE_END__'
$probeKV     = Parse-KV $probeBlock

# Find the LAST camera marker block (the winning attempt) - there can be one
# per /dev/videoN that was tried.
$cameraBlocks = @()
$searchFrom = 0
while ($true) {
    $bIdx = $probeText.IndexOf('__W2_01_CAMERA_BEGIN__', $searchFrom)
    if ($bIdx -lt 0) { break }
    $eIdx = $probeText.IndexOf('__W2_01_CAMERA_END__', $bIdx)
    if ($eIdx -lt 0) { break }
    $startLineEnd = $probeText.IndexOf("`n", $bIdx)
    if ($startLineEnd -lt 0 -or $startLineEnd -ge $eIdx) { break }
    $cameraBlocks += $probeText.Substring($startLineEnd + 1, $eIdx - ($startLineEnd + 1))
    $searchFrom = $eIdx + 1
}
Write-Host "Camera attempt blocks parsed: $($cameraBlocks.Count)"

$winningKV = $null
foreach ($blk in $cameraBlocks) {
    $kv = Parse-KV $blk
    if ($kv['verdict'] -eq 'PASS' -or $kv['verdict'] -eq 'WARN_NON_JPEG_PAYLOAD') {
        $winningKV = $kv
        break
    }
}
if (-not $winningKV -and $cameraBlocks.Count -gt 0) {
    # Fallback: keep last attempt for diagnostics
    $winningKV = Parse-KV $cameraBlocks[-1]
}

# 5. Evaluate gates
function Get-KV([hashtable]$h, [string]$k, $default = $null) {
    if ($h -and $h.ContainsKey($k)) { return $h[$k] } else { return $default }
}

$nodeCount = [int](Get-KV $probeKV 'node_count' 0)
$g1 = $nodeCount -gt 0

$driver = Get-KV $winningKV 'driver' ''
$g2 = ($driver -match 'uvcvideo')

$g3 = ((Get-KV $winningKV 'has_video_capture' '0') -eq '1') -and `
      ((Get-KV $winningKV 'has_streaming' '0') -eq '1')

$negFourcc = Get-KV $winningKV 'negotiated_fourcc' ''
$g4 = -not [string]::IsNullOrWhiteSpace($negFourcc)

$g5 = ([int](Get-KV $winningKV 'captured_bytes' 0)) -gt 0

if ($negFourcc -eq 'MJPG') {
    $g6 = ((Get-KV $winningKV 'jpeg_soi_eoi_ok' '0') -eq '1')
} else {
    $g6 = $true   # not applicable for non-MJPG payloads
}

$g7 = $snapExists -and ($snapSize -gt 0)

$gates = [ordered]@{
    G1_device_present      = $g1
    G2_driver_uvc          = $g2
    G3_capture_capable     = $g3
    G4_format_negotiated   = $g4
    G5_frame_captured      = $g5
    G6_jpeg_sane           = $g6
    G7_file_pulled         = $g7
}
$overallPass = ($gates.Values | Where-Object { -not $_ } | Measure-Object).Count -eq 0

# 6. Write summary.json
$summary = [ordered]@{
    schema           = 'w2_01_camera_first_light/1'
    utc              = (Get-Date).ToUniversalTime().ToString('yyyy-MM-ddTHH:mm:ssZ')
    adb_serial       = $AdbSerial
    requested = [ordered]@{
        width  = $Width
        height = $Height
        pixfmt = $PixFmt
    }
    probe = [ordered]@{
        exit_code   = $probeExit
        node_count  = $nodeCount
        winning_node = (Get-KV $probeKV 'winning_node' '')
        overall_verdict_remote = (Get-KV $probeKV 'overall_verdict' '')
    }
    capture = $winningKV
    pulled = [ordered]@{
        exists = $snapExists
        size_bytes = $snapSize
        path = (Resolve-Path -LiteralPath $snapLocal -ErrorAction SilentlyContinue | ForEach-Object { $_.Path })
    }
    gates = $gates
    overall_pass = $overallPass
}
$summaryPath = Join-Path $evidenceDir 'summary.json'
$summary | ConvertTo-Json -Depth 6 | Set-Content -LiteralPath $summaryPath -Encoding UTF8
Write-Host "Wrote: $summaryPath"

# 7. Print one-line verdict
$gateStr = ($gates.GetEnumerator() | ForEach-Object { "$($_.Key)=$($_.Value)" }) -join ' '
if ($overallPass) {
    Write-Host ("VERDICT: PASS  {0}" -f $gateStr) -ForegroundColor Green
} else {
    Write-Host ("VERDICT: FAIL  {0}" -f $gateStr) -ForegroundColor Yellow
}

exit ([int](-not $overallPass))
