#requires -Version 5.1
<#
.SYNOPSIS
    W2-02 end-to-end image-over-LoRa bench orchestrator (single-frame
    proof-of-life).

.DESCRIPTION
    Captures one frame from the USB camera on the TX board, host-side
    encodes it into a TileDeltaFrame, fragments it under the L072
    HostLink 64 B TX_FRAME_REQ cap, ships each fragment over the air via
    the proven W1-10b TX/RX probe pair, reassembles on the host, decodes
    the WebP tiles, and writes a reconstructed PNG side-by-side with the
    original.

    No M7 firmware required. No PIL/numpy required on the X8. Reuses the
    L072 firmware already validated by W1-10b (100/100 pkts).

    Wire path under test:

        ffmpeg /dev/videoN -> raw RGB24 384x256 -> [pull to host]
          -> per-tile WebP (host PIL)
          -> encode_tile_delta_frame  (base_station/image_pipeline)
          -> hand-chunked 0xFE fragments  (max 60 B data)
          -> [push fragments.hex to TX board]
          -> w2_02_tx_fragments.py  (HostLink TX_FRAME_REQ per fragment)
          -> SX1276 air -> SX1276 RX (other board)
          -> method_h_stage2_tx_probe.py --probe rx_listen
             (RX_FRAME_URC -> __RX_FRAME__ stdout lines)
          -> w2_02_host_pipeline.py decode
             (FragmentReassembler -> TileDeltaFrame -> canvas PNG)

.PARAMETER TxAdbSerial
    Board with the camera. Default: 2E2C1209DABC240B (camera observed
    there on 2026-05-18 preflight).

.PARAMETER RxAdbSerial
    Board that receives. Default: 2D0A1209DABC240B.

.PARAMETER VideoDev
    v4l2 device on the TX board. Default: /dev/video1 (Kurokesu C2 main
    UVC node).

.PARAMETER InterFragS
    Per-fragment TX inter-spacing (seconds). Default 0.05.

.PARAMETER Quality
    WebP quality (1..100). Default 55.

.EXAMPLE
    powershell -NoProfile -ExecutionPolicy Bypass `
        -File LifeTrac-v25/DESIGN-CONTROLLER/firmware/x8_lora_bootloader_helper/run_w2_02_image_over_lora_end_to_end.ps1
#>
param(
    [string]$TxAdbSerial = "2E2C1209DABC240B",
    [string]$RxAdbSerial = "2D0A1209DABC240B",
    [string]$VideoDev    = "/dev/video1",
    [double]$InterFragS = 0.2,
    [int]$Quality        = 55,
    [int]$ExtraRxWindowS = 30,
    [string]$RepoRoot    = ""
)

Set-StrictMode -Version Latest
$ErrorActionPreference = "Stop"

$ScriptRoot = if ($PSScriptRoot) { $PSScriptRoot }
              elseif ($PSCommandPath) { Split-Path -Parent $PSCommandPath }
              else { (Get-Location).Path }

function Resolve-RepoRoot {
    if ($RepoRoot -and (Test-Path -LiteralPath $RepoRoot)) {
        return (Resolve-Path -LiteralPath $RepoRoot).Path
    }
    return (Resolve-Path (Join-Path $ScriptRoot "../../../")).Path
}

$null = Get-Command adb -ErrorAction Stop
if ($TxAdbSerial -eq $RxAdbSerial) {
    throw "TxAdbSerial and RxAdbSerial must differ."
}

$repo       = Resolve-RepoRoot
$helperDir  = Join-Path $repo "DESIGN-CONTROLLER/firmware/x8_lora_bootloader_helper"
$benchRoot  = Join-Path $repo "DESIGN-CONTROLLER/bench-evidence"
$null       = New-Item -ItemType Directory -Force -Path $benchRoot
$stamp      = Get-Date -Format "yyyy-MM-dd_HHmmss"
$evidence   = Join-Path $benchRoot "W2-02_image_over_lora_${stamp}"
$null       = New-Item -ItemType Directory -Force -Path $evidence
Write-Host "Evidence dir: $evidence"

# Locate the host ffmpeg arm64 static binary so we can push it if absent.
$hostFfmpeg = Join-Path (Resolve-Path (Join-Path $repo "../../")).Path "ffmpeg-7.0.2-arm64-static/ffmpeg"
if (-not (Test-Path -LiteralPath $hostFfmpeg)) {
    # Fall back to repo-relative location used by the W2-01 work.
    $alt = Join-Path (Get-Location).Path "ffmpeg-7.0.2-arm64-static/ffmpeg"
    if (Test-Path -LiteralPath $alt) { $hostFfmpeg = $alt }
}
Write-Host "Host ffmpeg static: $hostFfmpeg (present=$(Test-Path -LiteralPath $hostFfmpeg))"

# ---------------------------------------------------------------------------
# 1. Verify both boards are present.
# ---------------------------------------------------------------------------
$devicesOut = (& adb devices) | Out-String
foreach ($pair in @(@("TX", $TxAdbSerial), @("RX", $RxAdbSerial))) {
    if ($devicesOut -notmatch [regex]::Escape($pair[1])) {
        throw "$($pair[0]) board '$($pair[1])' not present in 'adb devices'."
    }
}
Write-Host "Both boards present: TX=$TxAdbSerial RX=$RxAdbSerial"

# ---------------------------------------------------------------------------
# 2. Push helper toolkit to both boards (mirrors W1-10b pattern).
# ---------------------------------------------------------------------------
foreach ($serial in @($TxAdbSerial, $RxAdbSerial)) {
    Write-Host "Pushing helper toolkit to $serial..."
    & adb -s $serial push "$helperDir/." "/tmp/lifetrac_p0c/" | Out-Null
    if ($LASTEXITCODE -ne 0) { throw "adb push to $serial failed (rc=$LASTEXITCODE)" }
    & adb -s $serial exec-out "echo fio | sudo -S -p '' bash -lc 'chmod +x /tmp/lifetrac_p0c/*.sh'" | Out-Null
}

# ---------------------------------------------------------------------------
# 3. Ensure ffmpeg static binary present on the TX board.
# ---------------------------------------------------------------------------
$ffCheck = (& adb -s $TxAdbSerial exec-out "ls /tmp/ffmpeg 2>/dev/null") | Out-String
if ([string]::IsNullOrWhiteSpace($ffCheck)) {
    if (-not (Test-Path -LiteralPath $hostFfmpeg)) {
        throw "ffmpeg missing on TX board and not found at host path: $hostFfmpeg"
    }
    Write-Host "Pushing ffmpeg static to TX board ($hostFfmpeg)..."
    & adb -s $TxAdbSerial push $hostFfmpeg /tmp/ffmpeg | Out-Null
    if ($LASTEXITCODE -ne 0) { throw "ffmpeg push failed (rc=$LASTEXITCODE)" }
    & adb -s $TxAdbSerial exec-out "chmod +x /tmp/ffmpeg" | Out-Null
} else {
    Write-Host "ffmpeg already present on TX board."
}

# ---------------------------------------------------------------------------
# 4. Capture one raw RGB frame on the TX board.
# ---------------------------------------------------------------------------
Write-Host "Capturing single RGB frame on TX board ($VideoDev)..."
$capOut = & adb -s $TxAdbSerial exec-out "echo fio | sudo -S -p '' env LT_VIDEO_DEV=$VideoDev bash /tmp/lifetrac_p0c/w2_02_capture_raw_rgb.sh"
$capRc = $LASTEXITCODE
$capText = ($capOut | Out-String)
Set-Content -LiteralPath (Join-Path $evidence "capture_stdout.txt") -Value $capText
if ($capRc -ne 0 -or -not ($capText -match "__W2_02_CAPTURE_OK__")) {
    throw "Capture failed (rc=$capRc).`n$capText"
}
Write-Host "Capture OK."

# Pull raw RGB bytes.
$rawLocal = Join-Path $evidence "frame.rgb"
& adb -s $TxAdbSerial pull /tmp/w2_02_frame.rgb $rawLocal | Out-Null
if ($LASTEXITCODE -ne 0) { throw "adb pull failed (rc=$LASTEXITCODE)" }
$rawSize = (Get-Item -LiteralPath $rawLocal).Length
Write-Host "Pulled raw frame: $rawSize bytes"
if ($rawSize -ne 294912) { throw "Raw frame size $rawSize != 294912 (RGB24 384x256)" }

# ---------------------------------------------------------------------------
# 5. Host-side encode -> fragments.hex.
# ---------------------------------------------------------------------------
$fragLocal = Join-Path $evidence "fragments.hex"
$origPng   = Join-Path $evidence "original.png"
$encLog    = Join-Path $evidence "encode.log"
Write-Host "Encoding TileDeltaFrame + fragments on host..."
$encArgs = @(
    (Join-Path $helperDir "w2_02_host_pipeline.py"),
    "encode",
    "--raw", $rawLocal,
    "--out", $fragLocal,
    "--orig-png", $origPng,
    "--quality", $Quality.ToString()
)
$encOut = & py -3 @encArgs 2>&1
$encRc = $LASTEXITCODE
$encText = ($encOut | Out-String)
Set-Content -LiteralPath $encLog -Value $encText
Write-Host $encText
if ($encRc -ne 0) { throw "Encode failed (rc=$encRc)" }
$nFragments = (Get-Content -LiteralPath $fragLocal | Where-Object { $_.Trim() -ne "" }).Count
Write-Host "Encoded $nFragments fragments."

# ---------------------------------------------------------------------------
# 6. Push fragments to TX board.
# ---------------------------------------------------------------------------
& adb -s $TxAdbSerial push $fragLocal /tmp/w2_02_fragments.hex | Out-Null
if ($LASTEXITCODE -ne 0) { throw "fragments push failed (rc=$LASTEXITCODE)" }

# ---------------------------------------------------------------------------
# 7. Plan RX window based on fragment count + airtime budget.
# ---------------------------------------------------------------------------
# Per-fragment air = ~25 ms (PHY_IMAGE budget) + 50 ms inter-spacing + slack.
$txDurationS = [int][Math]::Ceiling($nFragments * ($InterFragS + 0.15)) + 10
$rxWindowS = $txDurationS + $ExtraRxWindowS
Write-Host "Plan: tx_duration~${txDurationS}s rx_window=${rxWindowS}s"

# ---------------------------------------------------------------------------
# 8. Start RX listener (background).
#
#    Two-step on the RX board:
#      a. w2_02_radio_wake_rxcont.py: write SX1276 RegOpMode=0x85 so the
#         radio is in LoRa+RXCONTINUOUS. Required because every prior
#         probe call ends with __RADIO_SLEEP_ON_EXIT__ writing 0x80, so
#         after a fresh bench cycle the chip is asleep and rx_frames=0.
#      b. method_h_stage2_tx_probe.py --probe rx_listen: passive RX
#         window that prints __W1_10B_LISTEN_READY__ before listening.
#
#    We deliberately skip run_method_h_stage2_tx.sh because its openocd
#    warm-boot step fails on Board 1 with "Error connecting DP: cannot
#    read IDR" (SWD bridge unreliable on the RX X8). The wake helper +
#    direct probe call achieves the same end-state without SWD.
# ---------------------------------------------------------------------------
$rxStdout = Join-Path $evidence "rx_stdout.txt"
$rxStderr = Join-Path $evidence "rx_stderr.txt"
"" | Set-Content -LiteralPath $rxStdout
"" | Set-Content -LiteralPath $rxStderr

Write-Host "Waking RX SX1276 to LoRa RXCONTINUOUS (write_reg 0x01=0x85)..."
$wakeOut = & adb -s $RxAdbSerial exec-out "cd /tmp/lifetrac_p0c && echo fio | sudo -S -p '' python3 -u w2_02_radio_wake_rxcont.py --dev /dev/ttymxc3 --baud 921600" 2>&1
$wakeText = ($wakeOut | Out-String)
Set-Content -LiteralPath (Join-Path $evidence "rx_wake.log") -Value $wakeText
Write-Host $wakeText
if ($wakeText -notmatch "__W2_02_WAKE_OK__") {
    throw "RX wake to RXCONT failed. See $((Join-Path $evidence 'rx_wake.log'))"
}

$rxRemoteCmd = "cd /tmp/lifetrac_p0c && echo fio | sudo -S -p '' python3 -u method_h_stage2_tx_probe.py --dev /dev/ttymxc3 --baud 921600 --probe rx_listen --rx-window $rxWindowS"
$rxWrapBody = "#!/bin/sh`n$rxRemoteCmd`nrc=`$?`nprintf '__METHOD_H_RC__=%s\n' `"`$rc`"`n"
$rxWrapLocal = Join-Path $evidence "_rx_wrap.sh"
[System.IO.File]::WriteAllText($rxWrapLocal, ($rxWrapBody -replace "`r`n", "`n"))
& adb -s $RxAdbSerial push $rxWrapLocal /tmp/lifetrac_p0c/_rx_wrap.sh | Out-Null
& adb -s $RxAdbSerial exec-out "chmod +x /tmp/lifetrac_p0c/_rx_wrap.sh" | Out-Null

Write-Host "Starting RX listener on $RxAdbSerial (window=${rxWindowS}s)..."
$rxProc = Start-Process -FilePath "adb" `
    -ArgumentList @("-s", $RxAdbSerial, "exec-out", "bash /tmp/lifetrac_p0c/_rx_wrap.sh") `
    -RedirectStandardOutput $rxStdout `
    -RedirectStandardError $rxStderr `
    -PassThru -NoNewWindow

# Wait for __W1_10B_LISTEN_READY__ token.
$readyDeadline = (Get-Date).AddSeconds(60)
$ready = $false
while ((Get-Date) -lt $readyDeadline -and -not $rxProc.HasExited) {
    Start-Sleep -Milliseconds 500
    try {
        $content = Get-Content -LiteralPath $rxStdout -Raw -ErrorAction SilentlyContinue
        if ($content -and ($content -match "__W1_10B_LISTEN_READY__")) {
            $ready = $true; break
        }
    } catch { }
}
if (-not $ready) {
    if (-not $rxProc.HasExited) { try { $rxProc.Kill() | Out-Null } catch { } }
    throw "RX listener never reached __W1_10B_LISTEN_READY__ within 60s. See $rxStdout"
}
Write-Host "RX listener READY."

# ---------------------------------------------------------------------------
# 9. Run TX fragment burst (foreground, sync). Stage a real .sh wrapper to
#    avoid the CRLF/quoting traps the W1-10b orchestrator already had to
#    work around. Use absolute paths because sudo doesn't propagate cwd.
# ---------------------------------------------------------------------------
$txStdout = Join-Path $evidence "tx_stdout.txt"
$interStr = $InterFragS.ToString([System.Globalization.CultureInfo]::InvariantCulture)
$txWrapBody = @"
#!/bin/sh
cd /tmp/lifetrac_p0c
echo fio | sudo -S -p '' python3 -u /tmp/lifetrac_p0c/w2_02_tx_fragments.py \
    --fragments /tmp/w2_02_fragments.hex \
    --inter-s $interStr
rc=`$?
printf '__METHOD_H_RC__=%s\n' "`$rc"
"@
$txWrapLocal = Join-Path $evidence "_tx_wrap.sh"
[System.IO.File]::WriteAllText($txWrapLocal, ($txWrapBody -replace "`r`n", "`n"))
& adb -s $TxAdbSerial push $txWrapLocal /tmp/lifetrac_p0c/_tx_wrap.sh | Out-Null
& adb -s $TxAdbSerial exec-out "chmod +x /tmp/lifetrac_p0c/_tx_wrap.sh" | Out-Null

Write-Host "Starting TX fragment burst on $TxAdbSerial ($nFragments fragments)..."
$tStart = Get-Date
$txOut = & adb -s $TxAdbSerial exec-out "bash /tmp/lifetrac_p0c/_tx_wrap.sh" 2>&1
$txAdbRc = $LASTEXITCODE
$tEnd = Get-Date
$txText = ($txOut | Out-String)
Set-Content -LiteralPath $txStdout -Value $txText
$txDurationActualS = ($tEnd - $tStart).TotalSeconds
Write-Host "TX done in $([int]$txDurationActualS)s (adb rc=$txAdbRc)."

# ---------------------------------------------------------------------------
# 10. Wait for RX listener window to elapse.
# ---------------------------------------------------------------------------
$rxFinishDeadline = (Get-Date).AddSeconds($rxWindowS + 30)
while (-not $rxProc.HasExited -and (Get-Date) -lt $rxFinishDeadline) {
    Start-Sleep -Milliseconds 500
}
if (-not $rxProc.HasExited) {
    Write-Warning "RX listener still running past deadline; killing."
    try { $rxProc.Kill() | Out-Null } catch { }
}
Start-Sleep -Milliseconds 500
$rxText = Get-Content -LiteralPath $rxStdout -Raw

# ---------------------------------------------------------------------------
# 11. Host-side decode -> PNG + summary.
# ---------------------------------------------------------------------------
$reconPng = Join-Path $evidence "reconstructed.png"
$summaryJ = Join-Path $evidence "summary.json"
Write-Host "Decoding RX log -> reconstructed PNG..."
$decArgs = @(
    (Join-Path $helperDir "w2_02_host_pipeline.py"),
    "decode",
    "--rx-log", $rxStdout,
    "--out-png", $reconPng,
    "--summary", $summaryJ
)
$decOut = & py -3 @decArgs 2>&1
$decRc = $LASTEXITCODE
$decText = ($decOut | Out-String)
Set-Content -LiteralPath (Join-Path $evidence "decode.log") -Value $decText
Write-Host $decText

# ---------------------------------------------------------------------------
# 12. Parse TX/RX stats, write top-level summary.
# ---------------------------------------------------------------------------
$txFragMatches = [regex]::Matches($txText, '__W2_02_TX_FRAG__\s+idx=(\d+).*?status=(\d+).*?payload_hex=([0-9a-fA-F]+)')
$txOkCount = 0
$txByHex = @{}
foreach ($m in $txFragMatches) {
    $st = [int]$m.Groups[2].Value
    $hx = $m.Groups[3].Value.ToLower()
    if ($st -eq 0) { $txOkCount++ }
    $txByHex[$hx] = $st
}
$txTimeoutCount = ([regex]::Matches($txText, '__W2_02_TX_TIMEOUT__|__W2_02_TX_ERR__|__W2_02_TX_SEND_ERR__')).Count

$rxFrameMatches = [regex]::Matches($rxText, '__RX_FRAME__\s+.*?rssi=(-?\d+)\s+snr=(-?\d+)\s+len=(\d+).*?payload_hex=([0-9a-fA-F]+)')
$rxMatched = 0
$rssiSamples = @()
$snrSamples = @()
foreach ($m in $rxFrameMatches) {
    $rssiSamples += [int]$m.Groups[1].Value
    $snrSamples  += [int]$m.Groups[2].Value
    $hx = $m.Groups[4].Value.ToLower()
    if ($txByHex.ContainsKey($hx)) { $rxMatched++ }
}
$nTx = $txFragMatches.Count
$nRx = $rxFrameMatches.Count
$txOkRate = if ($nFragments -gt 0) { [Math]::Round($txOkCount / $nFragments, 4) } else { 0 }
$rxMatchRate = if ($nFragments -gt 0) { [Math]::Round($rxMatched / $nFragments, 4) } else { 0 }
$rssiSorted = @($rssiSamples | Sort-Object)
$snrSorted = @($snrSamples | Sort-Object)
$rssiMed = if ($rssiSorted.Count -gt 0) { $rssiSorted[[Math]::Floor($rssiSorted.Count / 2)] } else { $null }
$snrMed = if ($snrSorted.Count -gt 0) { $snrSorted[[Math]::Floor($snrSorted.Count / 2)] } else { $null }

# Did the decoder complete a frame?
$decodeSummary = if (Test-Path -LiteralPath $summaryJ) {
    Get-Content -LiteralPath $summaryJ -Raw | ConvertFrom-Json
} else { $null }
$frameComplete = $false
$tilesDecoded = 0
if ($decodeSummary -and $decodeSummary.completed_frames -gt 0) {
    $frameComplete = $true
    $tilesDecoded = $decodeSummary.first_frame.tiles_decoded
}

# Compose top-level summary.
$top = [pscustomobject]@{
    timestamp           = $stamp
    tx_board            = $TxAdbSerial
    rx_board            = $RxAdbSerial
    video_dev           = $VideoDev
    n_fragments_planned = $nFragments
    n_tx_done           = $nTx
    n_tx_ok             = $txOkCount
    n_tx_timeout        = $txTimeoutCount
    n_rx_frames         = $nRx
    n_rx_matched        = $rxMatched
    tx_ok_rate          = $txOkRate
    rx_match_rate       = $rxMatchRate
    rssi_median_dbm     = $rssiMed
    snr_median_db       = $snrMed
    tx_duration_s       = [Math]::Round($txDurationActualS, 1)
    frame_complete      = $frameComplete
    tiles_decoded       = $tilesDecoded
    decode              = $decodeSummary
    gates               = @(
        [pscustomobject]@{ id="V1"; label="tx_ok_rate >= 0.99 (got $txOkRate)"; ok=($txOkRate -ge 0.99) }
        [pscustomobject]@{ id="V2"; label="rx_match_rate >= 0.95 (got $rxMatchRate)"; ok=($rxMatchRate -ge 0.95) }
        [pscustomobject]@{ id="V3"; label="frame_complete (got $frameComplete)"; ok=$frameComplete }
        [pscustomobject]@{ id="V4"; label="tiles_decoded == 96 (got $tilesDecoded)"; ok=($tilesDecoded -eq 96) }
    )
}
($top | ConvertTo-Json -Depth 8) | Set-Content -LiteralPath (Join-Path $evidence "summary_top.json")

# Pretty print gates.
Write-Host ""
Write-Host "============================================================"
Write-Host "W2-02 IMAGE-OVER-LORA BENCH SUMMARY"
Write-Host "============================================================"
Write-Host ("  Fragments planned : {0}" -f $nFragments)
Write-Host ("  TX done           : {0}  (ok={1} timeout={2})" -f $nTx, $txOkCount, $txTimeoutCount)
Write-Host ("  RX frames         : {0}  (matched-by-payload={1})" -f $nRx, $rxMatched)
Write-Host ("  TX ok rate        : {0}" -f $txOkRate)
Write-Host ("  RX match rate     : {0}" -f $rxMatchRate)
Write-Host ("  RSSI median dBm   : {0}" -f $rssiMed)
Write-Host ("  SNR median dB     : {0}" -f $snrMed)
Write-Host ("  Frame complete    : {0}  (tiles_decoded={1}/96)" -f $frameComplete, $tilesDecoded)
Write-Host ""
$allOk = $true
foreach ($g in $top.gates) {
    $tag = if ($g.ok) { "PASS" } else { "FAIL" }
    Write-Host ("  [{0}] {1}: {2}" -f $tag, $g.id, $g.label)
    if (-not $g.ok) { $allOk = $false }
}
Write-Host ""
Write-Host "  Original     : $origPng"
Write-Host "  Reconstructed: $reconPng"
Write-Host "  Evidence dir : $evidence"
Write-Host ""
if ($allOk) {
    Write-Host "VERDICT: PASS"
    exit 0
} else {
    Write-Host "VERDICT: FAIL"
    exit 1
}
