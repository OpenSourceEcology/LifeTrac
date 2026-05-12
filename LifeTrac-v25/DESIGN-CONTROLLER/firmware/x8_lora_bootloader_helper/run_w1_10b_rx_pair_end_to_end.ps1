#requires -Version 5.1
<#
.SYNOPSIS
    W1-10b two-board RX/TX-pair orchestrator (Phase B end-to-end LoRa receive
    validation).

.DESCRIPTION
    Pushes the helper toolkit to BOTH boards (Portenta X8 over ADB), starts
    `--probe rx_listen` on the RX board (background), waits for the RX
    listener to print `__W1_10B_LISTEN_READY__`, runs `--probe tx_burst` on
    the TX board (foreground, N cycles), then collects both stdout streams
    and evaluates Phase B gates B1..B6 per
    `LifeTrac-v25/AI NOTES/2026-05-12_W1-10_RX_Validation_Plan_Copilot_v1_0.md`
    section 3.3.

    Designed to be invoked from the repo root on the host PC. Requires both
    Portenta X8 boards present in `adb devices` and reachable.

.PARAMETER RxAdbSerial
    ADB serial of the receiver board (Board 2 by W1-10 convention =
    `2E2C1209DABC240B`).

.PARAMETER TxAdbSerial
    ADB serial of the transmitter board (Board 1 by W1-10 convention =
    `2D0A1209DABC240B`).

.PARAMETER Cycles
    Number of TX cycles to send (default 100, per plan section 3.2 first-light).

.PARAMETER InterCycleS
    Seconds to sleep between TX cycles (default 0.2). With SF7/BW125 ToA ~18
    ms a 0.2 s spacing keeps the TX duty cycle well under the 1 % regional
    cap and gives the RX side time to drain its host UART between packets.

.PARAMETER Timeout
    Per-cycle TX_DONE_URC timeout in seconds (default 5.0).

.PARAMETER ExtraRxWindowS
    Safety margin (seconds) added to the RX listen window beyond the
    expected TX duration (default 30).

.PARAMETER RepoRoot
    Optional override for the repo root. Default: derived relative to the
    script location.

.EXAMPLE
    powershell -NoProfile -ExecutionPolicy Bypass `
        -File LifeTrac-v25/DESIGN-CONTROLLER/firmware/x8_lora_bootloader_helper/run_w1_10b_rx_pair_end_to_end.ps1 `
        -RxAdbSerial 2E2C1209DABC240B -TxAdbSerial 2D0A1209DABC240B -Cycles 100
#>
param(
    [Parameter(Mandatory = $true)][string]$RxAdbSerial,
    [Parameter(Mandatory = $true)][string]$TxAdbSerial,
    [int]$Cycles = 100,
    [double]$InterCycleS = 0.2,
    [double]$Timeout = 5.0,
    [int]$ExtraRxWindowS = 30,
    [ValidateSet("tx_burst", "ping_pong")]
    [string]$Probe = "tx_burst",
    [double]$RttTimeout = 5.0,
    [string]$RepoRoot = ""
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

function Format-Invariant([double]$value) {
    return $value.ToString([System.Globalization.CultureInfo]::InvariantCulture)
}

$null = Get-Command adb -ErrorAction Stop

if ($RxAdbSerial -eq $TxAdbSerial) {
    throw "RxAdbSerial and TxAdbSerial must differ (got '$RxAdbSerial')."
}

# ---------------------------------------------------------------------------
# 1. Verify both boards are present in `adb devices`.
# ---------------------------------------------------------------------------
$devicesOut = (& adb devices) | Out-String
foreach ($pair in @(@("RX", $RxAdbSerial), @("TX", $TxAdbSerial))) {
    $role = $pair[0]; $serial = $pair[1]
    if ($devicesOut -notmatch [regex]::Escape($serial)) {
        throw "$role board '$serial' not present in 'adb devices'.`nadb devices output:`n$devicesOut"
    }
}
Write-Host "Both boards present: RX=$RxAdbSerial TX=$TxAdbSerial"

# ---------------------------------------------------------------------------
# 2. Set up evidence directory.
# ---------------------------------------------------------------------------
$repo = Resolve-RepoRoot
$helperDir = Join-Path $repo "DESIGN-CONTROLLER/firmware/x8_lora_bootloader_helper"
if (-not (Test-Path -LiteralPath $helperDir)) {
    throw "Helper dir not found: $helperDir"
}
$benchEvidenceRoot = Join-Path $repo "DESIGN-CONTROLLER/bench-evidence"
$null = New-Item -ItemType Directory -Force -Path $benchEvidenceRoot

$stamp = Get-Date -Format "yyyy-MM-dd_HHmmss"
$dirSuffix = if ($Probe -eq "ping_pong") { "W1-11_pingpong_${stamp}" } else { "W1-10b_rx_pair_${stamp}" }
$evidenceDir = Join-Path $benchEvidenceRoot $dirSuffix
$null = New-Item -ItemType Directory -Force -Path $evidenceDir
Write-Host "Evidence dir: $evidenceDir"

# Map orchestrator -Probe to (rx_mode, tx_mode) pairs.
$rxMode = if ($Probe -eq "ping_pong") { "rx_echo" } else { "rx_listen" }
$txMode = $Probe
Write-Host "Probe pair: rx=$rxMode tx=$txMode"

# ---------------------------------------------------------------------------
# 3. Push helper toolkit to both boards.
# ---------------------------------------------------------------------------
foreach ($serial in @($RxAdbSerial, $TxAdbSerial)) {
    Write-Host "Pushing helper toolkit to $serial..."
    & adb -s $serial push "$helperDir/." "/tmp/lifetrac_p0c/" | Out-Null
    if ($LASTEXITCODE -ne 0) { throw "adb push to $serial failed (rc=$LASTEXITCODE)" }
    & adb -s $serial exec-out "echo fio | sudo -S -p '' bash -lc 'chmod +x /tmp/lifetrac_p0c/*.sh'" | Out-Null
    if ($LASTEXITCODE -ne 0) { throw "chmod on $serial failed (rc=$LASTEXITCODE)" }
}

# ---------------------------------------------------------------------------
# 4. Compute RX listen window (TX duration + boot/openocd warm-up + margin).
# ---------------------------------------------------------------------------
$txDurationS = [int][Math]::Ceiling($Cycles * ($InterCycleS + 0.1)) + 5
$rxWindowS = $txDurationS + $ExtraRxWindowS
Write-Host ("Plan: TX cycles={0} inter={1}s -> tx_duration~{2}s; rx_window={3}s" `
    -f $Cycles, (Format-Invariant $InterCycleS), $txDurationS, $rxWindowS)

# ---------------------------------------------------------------------------
# 5. Start RX listener as background process; redirect stdout to file.
# ---------------------------------------------------------------------------
$rxStdout = Join-Path $evidenceDir "rx_stdout.txt"
$rxStderr = Join-Path $evidenceDir "rx_stderr.txt"
"" | Set-Content -LiteralPath $rxStdout
"" | Set-Content -LiteralPath $rxStderr

# Stage the RX wrapper as a real .sh file on the X8 instead of inlining it
# through Start-Process -ArgumentList, which re-quotes single-quoted args and
# breaks the nested `sh -lc '...'` quoting (the inline form works only with
# the PowerShell call operator, as in the TX path below).
$rxRemoteCmd = "echo fio | sudo -S -p '' env LIFETRAC_PROBE_MODE=$rxMode LIFETRAC_RX_WINDOW=$rxWindowS bash /tmp/lifetrac_p0c/run_method_h_stage2_tx.sh"
$rxWrapBody = "#!/bin/sh`n$rxRemoteCmd`nrc=`$?`nprintf '__METHOD_H_RC__=%s\n' `"`$rc`"`n"
$rxWrapLocal = Join-Path $evidenceDir "_rx_wrap.sh"
# Force LF line endings so /bin/sh on the X8 doesn't choke on CRLF.
[System.IO.File]::WriteAllText($rxWrapLocal, ($rxWrapBody -replace "`r`n", "`n"))
& adb -s $RxAdbSerial push $rxWrapLocal /tmp/lifetrac_p0c/_rx_wrap.sh | Out-Null
& adb -s $RxAdbSerial exec-out "chmod +x /tmp/lifetrac_p0c/_rx_wrap.sh" | Out-Null

Write-Host "Starting RX listener on $RxAdbSerial (window=${rxWindowS}s)..."
$rxProc = Start-Process -FilePath "adb" `
    -ArgumentList @("-s", $RxAdbSerial, "exec-out", "bash /tmp/lifetrac_p0c/_rx_wrap.sh") `
    -RedirectStandardOutput $rxStdout `
    -RedirectStandardError $rxStderr `
    -PassThru -NoNewWindow

# ---------------------------------------------------------------------------
# 6. Poll RX stdout for the __W1_10B_LISTEN_READY__ token (max 60 s).
# ---------------------------------------------------------------------------
$readyDeadline = (Get-Date).AddSeconds(60)
$ready = $false
while ((Get-Date) -lt $readyDeadline -and -not $rxProc.HasExited) {
    Start-Sleep -Milliseconds 500
    $content = $null
    try { $content = Get-Content -LiteralPath $rxStdout -Raw -ErrorAction SilentlyContinue } catch { }
    if ($content -and ($content -match "__W1_10B_LISTEN_READY__")) {
        $ready = $true
        break
    }
}
if (-not $ready) {
    if (-not $rxProc.HasExited) { try { $rxProc.Kill() | Out-Null } catch { } }
    throw "RX listener never reached __W1_10B_LISTEN_READY__ within 60s. See $rxStdout"
}
Write-Host "RX listener READY."

# ---------------------------------------------------------------------------
# 7. Run TX burst sync.
# ---------------------------------------------------------------------------
$txStdout = Join-Path $evidenceDir "tx_stdout.txt"
$txCountStr = $Cycles.ToString([System.Globalization.CultureInfo]::InvariantCulture)
$interStr = Format-Invariant $InterCycleS
$txRemoteCmd = "echo fio | sudo -S -p '' env LIFETRAC_PROBE_MODE=$txMode LIFETRAC_TX_COUNT=$txCountStr LIFETRAC_INTER_CYCLE_S=$interStr"
if ($Probe -eq "ping_pong") {
    $rttStr = Format-Invariant $RttTimeout
    $txRemoteCmd += " LIFETRAC_RTT_TIMEOUT=$rttStr"
}
$txRemoteCmd += " bash /tmp/lifetrac_p0c/run_method_h_stage2_tx.sh"
$txWrapped = "sh -lc '$txRemoteCmd; rc=`$?; printf ""__METHOD_H_RC__=%s\n"" ""`$rc""'"

Write-Host "Starting TX burst on $TxAdbSerial (cycles=$Cycles)..."
$txOut = & adb -s $TxAdbSerial exec-out $txWrapped 2>&1
$txAdbRc = $LASTEXITCODE
$txText = ($txOut | Out-String)
Set-Content -LiteralPath $txStdout -Value $txText
Write-Host "TX burst adb rc=$txAdbRc."

# ---------------------------------------------------------------------------
# 8. Wait for RX listener to finish.
# ---------------------------------------------------------------------------
$rxFinishDeadline = (Get-Date).AddSeconds($rxWindowS + 30)
while (-not $rxProc.HasExited -and (Get-Date) -lt $rxFinishDeadline) {
    Start-Sleep -Milliseconds 500
}
if (-not $rxProc.HasExited) {
    Write-Warning "RX listener still running past deadline; killing."
    try { $rxProc.Kill() | Out-Null } catch { }
}
Start-Sleep -Milliseconds 500  # let the file flush
$rxText = Get-Content -LiteralPath $rxStdout -Raw

# ---------------------------------------------------------------------------
# 9. Parse outputs.
# ---------------------------------------------------------------------------
function Parse-TxDones([string]$text) {
    $rows = New-Object System.Collections.ArrayList
    foreach ($line in ($text -split "`n")) {
        if ($line -match '__TX_DONE__\s+idx=(\d+)\s+tx_id=0x([0-9A-Fa-f]+)\s+status=(\d+).*?toa_us=(\d+).*?payload_hex=([0-9a-fA-F]+)') {
            [void]$rows.Add([pscustomobject]@{
                idx         = [int]$matches[1]
                tx_id       = [Convert]::ToInt32($matches[2], 16)
                status      = [int]$matches[3]
                toa_us      = [int]$matches[4]
                payload_hex = $matches[5].ToLower()
            })
        }
    }
    return ,$rows
}

function Parse-RxFrames([string]$text) {
    $rows = New-Object System.Collections.ArrayList
    foreach ($line in ($text -split "`n")) {
        if ($line -match '__RX_FRAME__\s+idx=(\d+)\s+rssi=(-?\d+)\s+snr=(-?\d+)\s+len=(\d+)\s+timestamp_us=(\d+)\s+payload_hex=([0-9a-fA-F]+)') {
            [void]$rows.Add([pscustomobject]@{
                idx          = [int]$matches[1]
                rssi         = [int]$matches[2]
                snr          = [int]$matches[3]
                len          = [int]$matches[4]
                timestamp_us = [int64]$matches[5]
                payload_hex  = $matches[6].ToLower()
            })
        }
    }
    return ,$rows
}

$txDones = Parse-TxDones $txText
$rxFrames = Parse-RxFrames $rxText

# W1-11 L-1: parse __PINGPONG__ rtt_ms lines (ping_pong mode only).
function Parse-PingPongs([string]$text) {
    $rows = New-Object System.Collections.ArrayList
    foreach ($line in ($text -split "`n")) {
        if ($line -match '__PINGPONG__\s+idx=(\d+)\s+tx_id=0x([0-9A-Fa-f]+)\s+status=(\d+).*?rtt_ms=(-?\d+(?:\.\d+)?)') {
            [void]$rows.Add([pscustomobject]@{
                idx    = [int]$matches[1]
                tx_id  = [Convert]::ToInt32($matches[2], 16)
                status = [int]$matches[3]
                rtt_ms = [double]$matches[4]
            })
        }
    }
    return ,$rows
}
$pingPongs = if ($Probe -eq "ping_pong") { Parse-PingPongs $txText } else { @() }
$rttSamples = @($pingPongs | Where-Object { $_.rtt_ms -ge 0 } | ForEach-Object { $_.rtt_ms } | Sort-Object)
$rttCount = $rttSamples.Count
$rttMatchRate = if ($Probe -eq "ping_pong" -and $Cycles -gt 0) {
    [Math]::Round($rttCount / $Cycles, 4)
} else { $null }
function Get-Pct([double[]]$xs, [double]$pct) {
    if ($xs.Count -eq 0) { return $null }
    $i = [int][Math]::Floor(($pct / 100.0) * ($xs.Count - 1))
    if ($i -lt 0) { $i = 0 }
    if ($i -ge $xs.Count) { $i = $xs.Count - 1 }
    return $xs[$i]
}
$rttP50 = Get-Pct $rttSamples 50
$rttP99 = Get-Pct $rttSamples 99
$rttMax = if ($rttCount -gt 0) { $rttSamples[$rttCount - 1] } else { $null }

$txTimeoutCount = ([regex]::Matches($txText, '__TX_TIMEOUT__|__TX_ERR__|__TX_SEND_ERR__')).Count
$txBurstDone = [regex]::Match($txText, '__W1_10B_BURST_DONE__\s+tx_count=(\d+)\s+tx_done_ok=(\d+)\s+tx_done_fail=(\d+)\s+tx_timeout=(\d+).*?radio_tx_abort_airtime_delta=(-?\d+).*?real_faults=(\d+)\s+invariants_violated=(\d+)')
$rxListenDone = [regex]::Match($rxText, '__W1_10B_LISTEN_DONE__\s+rx_frames=(\d+)\s+radio_rx_ok_delta=(-?\d+)\s+radio_crc_err_delta=(-?\d+)\s+real_faults=(\d+)\s+invariants_violated=(\d+)')

# ---------------------------------------------------------------------------
# 10. Correlate by payload_hex.
# ---------------------------------------------------------------------------
$txByPayload = @{}
foreach ($t in $txDones) { $txByPayload[$t.payload_hex] = $t }

$matched = 0
$rxOrphans = 0
foreach ($r in $rxFrames) {
    if ($txByPayload.ContainsKey($r.payload_hex)) { $matched++ }
    else { $rxOrphans++ }
}

$txDoneOkCount = @($txDones | Where-Object { $_.status -eq 0 }).Count
$txDoneRate = if ($Cycles -gt 0) { [Math]::Round($txDoneOkCount / $Cycles, 4) } else { 0.0 }
$rxMatchRate = if ($Cycles -gt 0) { [Math]::Round($matched / $Cycles, 4) } else { 0.0 }

$rssiList = @($rxFrames | ForEach-Object { $_.rssi } | Sort-Object)
$snrList  = @($rxFrames | ForEach-Object { $_.snr  } | Sort-Object)
$rssiMedian = if ($rssiList.Count -gt 0) { $rssiList[[Math]::Floor($rssiList.Count / 2)] } else { $null }
$snrMedian  = if ($snrList.Count  -gt 0) { $snrList[[Math]::Floor($snrList.Count / 2)] }   else { $null }

$txAbortAirDelta  = if ($txBurstDone.Success)  { [int]$txBurstDone.Groups[5].Value } else { 999 }
$txRealFaults     = if ($txBurstDone.Success)  { [int]$txBurstDone.Groups[6].Value } else { 999 }
$txInvariantsBad  = if ($txBurstDone.Success)  { [int]$txBurstDone.Groups[7].Value } else { 999 }
$rxRealFaults     = if ($rxListenDone.Success) { [int]$rxListenDone.Groups[4].Value } else { 999 }
$rxInvariantsBad  = if ($rxListenDone.Success) { [int]$rxListenDone.Groups[5].Value } else { 999 }

# ---------------------------------------------------------------------------
# 11. Phase B gates B1..B6.
# ---------------------------------------------------------------------------
$gateB1 = [bool]($txDoneRate -ge 0.99)
$gateB2 = [bool]($rxMatchRate -ge 0.99)
$gateB3 = [bool]($txAbortAirDelta -eq 0)
$gateB4 = [bool](($txRealFaults -eq 0) -and ($rxRealFaults -eq 0))
$gateB5 = [bool](($txInvariantsBad -eq 0) -and ($rxInvariantsBad -eq 0))
$gateB6 = [bool](($null -ne $rssiMedian) -and ($rssiMedian -ge -120) -and ($rssiMedian -le -30))

$gates = @(
    [pscustomobject]@{ id = "B1"; label = "tx_done_rate >= 0.99 (got $txDoneRate)"; ok = $gateB1 }
    [pscustomobject]@{ id = "B2"; label = "rx_match_rate >= 0.99 (got $rxMatchRate)"; ok = $gateB2 }
    [pscustomobject]@{ id = "B3"; label = "radio_tx_abort_airtime_delta == 0 (got $txAbortAirDelta)"; ok = $gateB3 }
    [pscustomobject]@{ id = "B4"; label = "no real FAULT_URC (tx=$txRealFaults rx=$rxRealFaults)"; ok = $gateB4 }
    [pscustomobject]@{ id = "B5"; label = "host invariants stable (tx_violated=$txInvariantsBad rx_violated=$rxInvariantsBad)"; ok = $gateB5 }
    [pscustomobject]@{ id = "B6"; label = "median RSSI in [-120,-30] dBm (got $rssiMedian)"; ok = $gateB6 }
)
if ($Probe -eq "ping_pong") {
    $gateB7 = [bool]($null -ne $rttMatchRate -and $rttMatchRate -ge 0.99)
    $gates += [pscustomobject]@{
        id = "B7"
        label = ("rtt_match_rate >= 0.99 (got {0}); rtt_ms p50={1} p99={2} max={3}" `
            -f $rttMatchRate, $rttP50, $rttP99, $rttMax)
        ok = $gateB7
    }
}

Write-Host ""
Write-Host "=== W1-10b Phase B gate evaluation ==="
$failed = 0
foreach ($g in $gates) {
    $st = if ($g.ok) { "PASS" } else { $failed++; "FAIL" }
    Write-Host ("  [{0}] {1}: {2}" -f $st, $g.id, $g.label)
}
Write-Host ""
Write-Host "Summary:"
Write-Host "  tx_cycles=$Cycles tx_done_ok=$txDoneOkCount tx_done_rate=$txDoneRate tx_timeouts=$txTimeoutCount"
Write-Host "  rx_frames=$($rxFrames.Count) rx_match=$matched rx_orphan=$rxOrphans rx_match_rate=$rxMatchRate"
Write-Host "  rssi_median=$rssiMedian dBm  snr_median=$snrMedian dB"
Write-Host "  radio_tx_abort_airtime_delta=$txAbortAirDelta"
Write-Host "  tx_real_faults=$txRealFaults rx_real_faults=$rxRealFaults"
Write-Host "  tx_invariants_violated=$txInvariantsBad rx_invariants_violated=$rxInvariantsBad"

# ---------------------------------------------------------------------------
# 12. Save summary JSON.
# ---------------------------------------------------------------------------
$verdict = if ($failed -eq 0) { "RX_PAIR_PASS" } else { "RX_PAIR_FAIL_${failed}_GATES" }
$summary = [ordered]@{
    stamp                          = $stamp
    probe                          = $Probe
    rx_serial                      = $RxAdbSerial
    tx_serial                      = $TxAdbSerial
    cycles                         = $Cycles
    inter_cycle_s                  = $InterCycleS
    rx_window_s                    = $rxWindowS
    tx_done_ok                     = $txDoneOkCount
    tx_done_rate                   = $txDoneRate
    tx_timeouts                    = $txTimeoutCount
    rx_frames_received             = $rxFrames.Count
    rx_match                       = $matched
    rx_orphan                      = $rxOrphans
    rx_match_rate                  = $rxMatchRate
    rssi_median_dbm                = $rssiMedian
    snr_median_db                  = $snrMedian
    radio_tx_abort_airtime_delta   = $txAbortAirDelta
    tx_real_faults                 = $txRealFaults
    rx_real_faults                 = $rxRealFaults
    tx_invariants_violated         = $txInvariantsBad
    rx_invariants_violated         = $rxInvariantsBad
    rtt_match_rate                 = $rttMatchRate
    rtt_count                      = $rttCount
    rtt_ms_p50                     = $rttP50
    rtt_ms_p99                     = $rttP99
    rtt_ms_max                     = $rttMax
    gates                          = $gates
    verdict                        = $verdict
}
$summary | ConvertTo-Json -Depth 5 | Set-Content -LiteralPath (Join-Path $evidenceDir "summary.json")

# ---------------------------------------------------------------------------
# 13. Pull per-board logs (best-effort).
# 14. Optional: post-process latency stats (LATENCY_BUDGET.md §1 confirmation).
# Both steps are post-gate / informational — they must NEVER fail the run.
# Scope $ErrorActionPreference=Continue across both: under EAP=Stop, native
# tools (adb, python) emitting stderr text raise NativeCommandError records
# that are converted to terminating errors and would abort the orchestrator
# after all gate evaluation is complete.
# ---------------------------------------------------------------------------
$savedEAP = $ErrorActionPreference
$ErrorActionPreference = "Continue"
try {
    foreach ($pair in @(@($RxAdbSerial, $rxMode), @($TxAdbSerial, $txMode))) {
        $serial = $pair[0]; $mode = $pair[1]
        foreach ($leaf in @("method_h_stage2_${mode}.log", "method_h_stage2_${mode}_ocd.log")) {
            $rPath = "/tmp/lifetrac_p0c/$leaf"
            $lPath = Join-Path $evidenceDir "${serial}_${leaf}"
            & adb -s $serial pull $rPath $lPath 2>&1 | Out-Null
        }
    }

    $analyzer = Join-Path $helperDir "analyze_rtt.py"
    if (Test-Path -LiteralPath $analyzer) {
        Write-Host ""
        Write-Host "Running latency analyzer..."
        $pyCandidates = @("python", "python3", "py")
        $analyzerRan = $false
        foreach ($pyName in $pyCandidates) {
            $argList = @($analyzer, $evidenceDir, "--no-stdout", "--merge-summary")
            if ($pyName -eq "py") { $argList = @("-3") + $argList }
            $output = $null
            try {
                $output = & $pyName @argList 2>&1
                $rc = $LASTEXITCODE
            } catch {
                Write-Host ("  [$pyName] not invokable: {0}" -f $_.Exception.Message)
                continue
            }
            if ($null -ne $output) {
                $output | ForEach-Object { Write-Host "  [$pyName] $_" }
            }
            if ($rc -eq 0) {
                $analyzerRan = $true
                break
            }
            Write-Host ("  [$pyName] exited rc={0}, trying next interpreter" -f $rc)
        }
        if ($analyzerRan) {
            $rttMd = Join-Path $evidenceDir "rtt_report.md"
            if (Test-Path -LiteralPath $rttMd) {
                Write-Host "--- rtt_report.md ---"
                Get-Content -LiteralPath $rttMd | Write-Host
                Write-Host "---"
            }
        } else {
            Write-Warning "Latency analyzer did not run; invoke manually: python `"$analyzer`" `"$evidenceDir`""
        }
    }
} finally {
    $ErrorActionPreference = $savedEAP
}

Write-Host ""
Write-Host "__W1_10B_VERDICT__=$verdict"
Write-Host "Evidence: $evidenceDir"
if ($failed -eq 0) {
    Write-Host "VERDICT: PASS -- Phase B closure achieved."
    exit 0
}
Write-Host "VERDICT: FAIL ($failed gates failed)."
exit 1
