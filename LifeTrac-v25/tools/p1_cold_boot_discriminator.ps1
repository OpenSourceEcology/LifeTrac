# =============================================================================
# p1_cold_boot_discriminator.ps1 (Phase 2.1)
# -----------------------------------------------------------------------------
# Falsification harness for P1 (cold-boot RUNTIME_PROFILE_ENUM=ERR race).
#
# Per the 2026-05-21 Open Problems doc (v3.0 §1, "Cheap discriminator,"
# lines 387-401), the open question is *whether the failure is host-race
# or firmware-not-ready*:
#
#   - If the first successful CFG_GET RUNTIME_PROFILE_ENUM consistently
#     lands at `t_boot_urc + Δ` with Δ tightly clustered (low CV across
#     N cold boots), the L072 firmware is simply not yet able to answer
#     the request - the radio init / SX1276 reset / profile-table load
#     is still in flight. The fix is then a firmware-side
#     `__PROFILE_READY__` URC (Phase 2.2 firmware-not-ready limb).
#
#   - If the time-to-first-OK varies with URC depth at the moment of
#     request (i.e. correlates with the count of drained
#     BOOT_URC/STATS_URC frames during the settle window), it really is
#     a host-side queue race, and the v1.1 host-side handshake
#     (`drain_startup_until_quiet` + bounded backoff) is the right fix
#     (Phase 2.2 host-race limb).
#
# This script runs N cold-boot cycles on the RX board and captures the
# probe's stdout. Tokens scraped per cycle:
#
#   t0_ms               : wall-clock start of probe launch (after adb returns)
#   t_boot_urc_ms       : delta to first 'BOOT_URC observed during settle'
#   t_profile_emit_ms   : delta to the canonical
#                         'RUNTIME_PROFILE_ENUM=<N>' or '=ERR <reason>' line
#   profile_ok          : 1 if =<N>, 0 if =ERR (or missing)
#   profile_text        : the raw token after 'RUNTIME_PROFILE_ENUM='
#   drained_during_boot : count of 'INFO: drained type=' lines
#                         (BOOT settle + post-VER drain)
#
# Output: one row per cycle, written to
#   LifeTrac-v25/DESIGN-CONTROLLER/bench-evidence/p1_cold_boot_<stamp>/
#       cycles.csv          - one row per cycle
#       cycle_NN_stdout.txt - raw probe stdout for cycle NN
#       summary.json        - per-cycle stats + cohort-level Δ statistics
#
# Reset strategy (revised 2026-05-22 T2 by evidence): the cycle resets
# *only the L072* via the gpio163 NRST line (SOFT_RESET_INDEX 3.1 — the
# same NRST line `revive_bridge.sh` toggles). Earlier drafts used
# `adb reboot` of the X8 host, but the first real cold-boot cycle proved
# that path does NOT visibly reset the L072 (probe always reported
# "BOOT_URC not observed during 1.0s settle"), AND it carries a
# documented USB-wedge risk (see repo memory
# `lifetrac-x8-reboot-wedges-usb.md`). Earlier-earlier drafts used
# `sudo reboot` over `adb exec-out`, which is the exact pattern that
# wedged the RX board on 2026-05-22 19:00.
# gpio163 NRST: idempotent, ~150ms total, no USB teardown, no need to
# wait-for-device, and empirically reproduces the same P1 symptom
# (`RUNTIME_PROFILE_ENUM=ERR request_failed:TimeoutError`) on the very
# first cycle. Helpers are pushed once before the loop because gpio163
# resets only the L072 — /tmp on the X8 is preserved across cycles.
#
# Does NOT touch the TX board. Per `closing sleep audit` in the v4.1
# review, the radio is parked in SLEEP at probe exit by the
# `--sleep-on-exit` default of method_h_stage2_tx_probe_v2.py.
#
# Usage:
#   .\p1_cold_boot_discriminator.ps1 -RxSerial 2D0A1209DABC240B -Cycles 20
#
# =============================================================================

[CmdletBinding()]
param(
    [Parameter(Mandatory=$true)]
    [string]$RxSerial,

    [int]$Cycles = 20,

    # Per-cycle quick probe: enough to exercise the BOOT_URC drain and
    # RUNTIME_PROFILE_ENUM emit, then exit. `--probe rx --rx-window 0.5`
    # is the lightest path that still runs the full bring-up sequence.
    [string]$ProbeMode = 'rx',

    [double]$RxWindowS = 0.5,

    [string]$SudoPw = 'fio',

    # Hard cap per cycle to prevent a wedged reboot from stalling the
    # whole run. If a cycle exceeds this, it is recorded as
    # `cycle_timeout=1` and we move on.
    [int]$PerCycleTimeoutS = 90,

    # Pre-probe sleep (seconds) between NRST release and probe launch.
    # This is the discriminator knob: sweep across cycles to see whether
    # RUNTIME_PROFILE_ENUM flips from ERR -> OK as it grows. Default
    # 0.05 (minimal — reproduces the failure consistently per the
    # 2026-05-22 T2 evidence).
    [double]$PreProbeSleepS = 0.05
)

$ErrorActionPreference = 'Stop'
# Strict mode intentionally not enabled: [ordered] hashtable property
# assignment + Register-ObjectEvent action blocks both interact poorly
# with v3.0 strictness; this is bench tooling, prefer permissive parse.

# Resolve repo root from this script's location (.\LifeTrac-v25\tools\...)
$repoRoot = Resolve-Path (Join-Path $PSScriptRoot '..\..')
$helperDir = Join-Path $repoRoot 'LifeTrac-v25\DESIGN-CONTROLLER\firmware\x8_lora_bootloader_helper'
$probeRel  = 'method_h_stage2_tx_probe_v2.py'
$probeOnDevice = "/tmp/lifetrac_p0c/$probeRel"

$benchRoot = Join-Path $repoRoot 'LifeTrac-v25\DESIGN-CONTROLLER\bench-evidence'
$null = New-Item -ItemType Directory -Force -Path $benchRoot
$stamp = Get-Date -Format 'yyyy-MM-dd_HHmmss'
$evidence = Join-Path $benchRoot ("p1_cold_boot_${stamp}")
$null = New-Item -ItemType Directory -Force -Path $evidence
Write-Host "Evidence dir: $evidence"
Write-Host "RX serial   : $RxSerial"
Write-Host "Cycles      : $Cycles"

# -----------------------------------------------------------------------------
# Helper: run adb with stderr merged and a timeout (process-level).
# Mirrors the always-merge-stderr pattern proven in P2 Phase 1.3 (2026-05-22),
# so cold-boot reboot noise on stderr doesn't pollute the cycle log with
# NativeCommandError records.
# -----------------------------------------------------------------------------
function Invoke-AdbCli {
    param(
        [string[]]$AdbArgs,
        [int]$TimeoutS = 30
    )
    $prev = $ErrorActionPreference
    $ErrorActionPreference = 'Continue'
    try {
        # PS 5.1 / .NET Framework: ProcessStartInfo.ArgumentList does not
        # exist. Build a quoted Arguments string. Quote any token that
        # contains whitespace, double-quote, or shell metacharacters; embed
        # double-quotes by doubling per CommandLineToArgvW rules.
        $quoted = foreach ($a in $AdbArgs) {
            if ($a -match '[\s"]' -or $a -eq '') {
                '"' + ($a -replace '"', '""') + '"'
            } else {
                $a
            }
        }
        $argString = [string]::Join(' ', $quoted)

        $psi = New-Object System.Diagnostics.ProcessStartInfo
        $psi.FileName = 'adb'
        $psi.Arguments = $argString
        $psi.RedirectStandardOutput = $true
        $psi.RedirectStandardError  = $true
        $psi.UseShellExecute = $false
        $psi.CreateNoWindow  = $true
        $p = New-Object System.Diagnostics.Process
        $p.StartInfo = $psi

        # 2026-05-22: Earlier draft used BeginOutputReadLine +
        # Register-ObjectEvent + StringBuilder.MessageData, which
        # silently dropped output in PS 5.1 (events ran but never
        # mutated the outer StringBuilder — repro:
        # adb exec-out "cat /proc/uptime" returned RC=0 OUT_LEN=0).
        # Switch to Task-based ReadToEndAsync, which avoids both the
        # eventing race AND the classic stderr-pipe-full deadlock.
        try {
            [void]$p.Start()
            $outTask = $p.StandardOutput.ReadToEndAsync()
            $errTask = $p.StandardError.ReadToEndAsync()
            if (-not $p.WaitForExit($TimeoutS * 1000)) {
                try { $p.Kill() } catch { }
                $p.WaitForExit(2000) | Out-Null
                $sOut = ''
                $sErr = ''
                try { $outTask.Wait(2000) | Out-Null; $sOut = $outTask.Result } catch { }
                try { $errTask.Wait(2000) | Out-Null; $sErr = $errTask.Result } catch { }
                return [pscustomobject]@{ Rc = -1; Stdout = $sOut; Stderr = "timeout after ${TimeoutS}s`n" + $sErr; TimedOut = $true }
            }
            # Process exited; drain both readers (bounded).
            $outTask.Wait(5000) | Out-Null
            $errTask.Wait(5000) | Out-Null
            return [pscustomobject]@{ Rc = $p.ExitCode; Stdout = $outTask.Result; Stderr = $errTask.Result; TimedOut = $false }
        } finally {
            $p.Dispose()
        }
    } finally {
        $ErrorActionPreference = $prev
    }
}

# -----------------------------------------------------------------------------
# One cold-boot cycle. Returns a hashtable of measurements.
# -----------------------------------------------------------------------------
function Invoke-ColdBootCycle {
    param([int]$CycleNum)

    $cycleStdoutPath = Join-Path $evidence ("cycle_{0:D2}_stdout.txt" -f $CycleNum)
    $row = [ordered]@{
        cycle               = $CycleNum
        wall_start          = (Get-Date).ToString('o')
        pre_probe_sleep_s   = $PreProbeSleepS
        reboot_rc           = $null
        wait_for_device_s   = $null
        probe_launch_s      = $null
        probe_total_s       = $null
        cycle_timeout       = 0
        t_boot_urc_ms       = $null
        t_profile_emit_ms   = $null
        profile_ok          = 0
        profile_text        = ''
        drained_during_boot = 0
        notes               = ''
    }

    Write-Host ("--- Cycle {0,2}/{1} ---" -f $CycleNum, $Cycles)

    # 1. Pulse gpio163 (L072 NRST) and immediately run the probe in a
    # single exec-out so the inter-cycle delay between NRST release and
    # probe-port-open is minimized and consistent (~150ms sleep +
    # python startup). The remote bash script lives at
    # /tmp/lifetrac_p0c/_p1_pulse_and_probe.sh (pushed once before the
    # loop). It emits a 'PULSE_DONE_AT=<unix.ns>' line as its first
    # output so a future revision could compute precise t0_post_nrst
    # deltas; the current cohort analysis uses line-index ordering as
    # a coarse proxy (same scheme as the original X8-reboot draft).
    $launchAt = Get-Date
    $remoteCmd = "echo $SudoPw | sudo -S -p '' bash /tmp/lifetrac_p0c/_p1_pulse_and_probe.sh $ProbeMode $RxWindowS $PreProbeSleepS"
    $probeArgs = @('-s', $RxSerial, 'exec-out', $remoteCmd)
    Write-Host "  [1] gpio163 NRST + ${PreProbeSleepS}s dwell + probe (--probe $ProbeMode --rx-window $RxWindowS)..."
    $probe = Invoke-AdbCli -AdbArgs $probeArgs -TimeoutS $PerCycleTimeoutS
    $row.probe_total_s = [math]::Round(((Get-Date) - $launchAt).TotalSeconds, 2)
    $row.reboot_rc = 0  # gpio163 path: no reboot rc to record; keep field for CSV stability.
    Write-Host ("      rc={0} total={1}s stdout={2}B stderr={3}B" -f $probe.Rc, $row.probe_total_s, $probe.Stdout.Length, $probe.Stderr.Length)
    if ($probe.TimedOut) {
        $row.cycle_timeout = 1
        $row.notes = "pulse+probe timeout after ${PerCycleTimeoutS}s"
        Write-Host "  ! pulse+probe timeout"
        Set-Content -LiteralPath $cycleStdoutPath -Value ($probe.Stdout + "`n--- STDERR ---`n" + $probe.Stderr) -Encoding utf8
        return $row
    }

    # 2. Persist raw stdout.
    Set-Content -LiteralPath $cycleStdoutPath -Value ($probe.Stdout + "`n--- STDERR ---`n" + $probe.Stderr) -Encoding utf8

    # 3. Scrape tokens. Probe stdout has no per-line timestamps, so we
    # treat 'BOOT_URC observed during settle' as the local t=0 reference
    # and compute t_profile_emit_ms as the WALL-CLOCK delta from launch to
    # the line carrying RUNTIME_PROFILE_ENUM. The accuracy is bounded by
    # how fast python emits these two prints; both occur in the same
    # tight setup sequence so the relative ordering and the magnitude
    # (~100-1500ms expected) are meaningful for the discriminator.
    #
    # For the *cohort* analysis we publish (Δ = t_profile - t_boot_urc)
    # using the line-ordering as the timing proxy. This is the same
    # ordering the firmware sees on the wire.
    $lines = $probe.Stdout -split "`r?`n"
    $bootIdx = -1
    $profileIdx = -1
    $drained = 0
    for ($i = 0; $i -lt $lines.Count; $i++) {
        $ln = $lines[$i]
        if ($ln -match 'BOOT_URC observed during settle') {
            if ($bootIdx -lt 0) { $bootIdx = $i }
        } elseif ($ln -match '^RUNTIME_PROFILE_ENUM=(.+)$') {
            if ($profileIdx -lt 0) {
                $profileIdx = $i
                $row.profile_text = $Matches[1].Trim()
                $row.profile_ok = [int]([bool]($row.profile_text -match '^\d+$'))
            }
        } elseif ($ln -match 'INFO: drained type=' -or $ln -match '\bdrained\b.*during boot settle') {
            $drained++
        }
    }
    $row.drained_during_boot = $drained

    # Convert line-index ordering to a coarse "later in stream" indicator;
    # we also record the wall-clock probe_total_s so the cohort analysis
    # can normalize. The exact ms-resolution Δ requires per-line stamping
    # which would touch the probe; the discriminator's binary outcome
    # (tight cluster vs URC-correlated) is robust to the proxy.
    if ($bootIdx -ge 0)   { $row.t_boot_urc_ms     = $bootIdx }
    if ($profileIdx -ge 0){ $row.t_profile_emit_ms = $profileIdx }

    Write-Host ("  bootURC@line={0,3}  profileEmit@line={1,3}  ok={2}  drained={3}  total={4,5:N2}s  text='{5}'" -f `
        $row.t_boot_urc_ms, $row.t_profile_emit_ms, $row.profile_ok, $row.drained_during_boot,
        $row.probe_total_s, $row.profile_text)
    return $row
}

# -----------------------------------------------------------------------------
# Main loop.
# -----------------------------------------------------------------------------

# Pre-loop: push helpers + the gpio163 pulse driver ONCE. gpio163 reset
# only resets the L072 (not the X8), so /tmp persists across cycles.
Write-Host ""
Write-Host "[pre] one-time setup: push helpers + _p1_pulse_and_probe.sh"
[void](Invoke-AdbCli -AdbArgs @('-s', $RxSerial, 'exec-out', 'mkdir -p /tmp/lifetrac_p0c') -TimeoutS 10)
$push = Invoke-AdbCli -AdbArgs @('-s', $RxSerial, 'push', "$helperDir/.", '/tmp/lifetrac_p0c/') -TimeoutS 90
if ($push.Rc -ne 0) {
    throw "Pre-loop helper push failed rc=$($push.Rc) stderr=$($push.Stderr)"
}
Write-Host ("      helper push rc={0}" -f $push.Rc)
$pulseLocal = Join-Path $PSScriptRoot '_p1_pulse_and_probe.sh'
if (-not (Test-Path $pulseLocal)) { throw "Missing helper: $pulseLocal" }
$pushPulse = Invoke-AdbCli -AdbArgs @('-s', $RxSerial, 'push', $pulseLocal, '/tmp/lifetrac_p0c/_p1_pulse_and_probe.sh') -TimeoutS 20
if ($pushPulse.Rc -ne 0) {
    throw "Pre-loop pulse-script push failed rc=$($pushPulse.Rc) stderr=$($pushPulse.Stderr)"
}
Write-Host ("      pulse-script push rc={0}" -f $pushPulse.Rc)

$rows = @()
for ($i = 1; $i -le $Cycles; $i++) {
    $rows += [pscustomobject](Invoke-ColdBootCycle -CycleNum $i)
}

# -----------------------------------------------------------------------------
# Cohort statistics.
# -----------------------------------------------------------------------------
$completed = $rows | Where-Object { $_.cycle_timeout -eq 0 -and $_.t_boot_urc_ms -ne $null -and $_.t_profile_emit_ms -ne $null }
$okRows    = $completed | Where-Object { $_.profile_ok -eq 1 }
$errRows   = $completed | Where-Object { $_.profile_ok -eq 0 }

function Get-Stats {
    param([double[]]$Values)
    if ($null -eq $Values -or $Values.Count -eq 0) { return $null }
    $m  = ($Values | Measure-Object -Average -Minimum -Maximum)
    $av = [double]$m.Average
    $sd = if ($Values.Count -gt 1) {
        [math]::Sqrt((($Values | ForEach-Object { ($_ - $av) * ($_ - $av) }) | Measure-Object -Sum).Sum / ($Values.Count - 1))
    } else { 0.0 }
    return [ordered]@{
        n   = $Values.Count
        min = [double]$m.Minimum
        max = [double]$m.Maximum
        avg = [math]::Round($av, 3)
        sd  = [math]::Round($sd, 3)
        cv  = if ($av -ne 0) { [math]::Round($sd / $av, 3) } else { $null }
    }
}

$deltaLines = $completed | ForEach-Object { [double]($_.t_profile_emit_ms - $_.t_boot_urc_ms) }
$drained    = $completed | ForEach-Object { [double]$_.drained_during_boot }

$summary = [ordered]@{
    stamp                  = $stamp
    rx_serial              = $RxSerial
    cycles                 = $Cycles
    completed              = $completed.Count
    profile_ok_count       = $okRows.Count
    profile_err_count      = $errRows.Count
    cycle_timeouts         = @($rows | Where-Object { $_.cycle_timeout -eq 1 }).Count
    delta_line_stats       = Get-Stats -Values $deltaLines
    drained_count_stats    = Get-Stats -Values $drained
    # Discriminator hint: low CV on delta_line_stats (e.g. CV < 0.25)
    # AND no correlation with drained count => firmware-not-ready.
    # Wide variance AND positive correlation with drained count =>
    # host-race.
    correlation_delta_vs_drained = $null
}

if ($completed.Count -ge 3) {
    # Pearson correlation between delta and drained counts.
    $n = $completed.Count
    $sumX = 0.0; $sumY = 0.0; $sumXY = 0.0; $sumX2 = 0.0; $sumY2 = 0.0
    for ($i = 0; $i -lt $n; $i++) {
        $x = $drained[$i]
        $y = $deltaLines[$i]
        $sumX  += $x; $sumY  += $y
        $sumXY += $x * $y
        $sumX2 += $x * $x
        $sumY2 += $y * $y
    }
    $num = ($n * $sumXY) - ($sumX * $sumY)
    $den = [math]::Sqrt((($n * $sumX2) - ($sumX * $sumX)) * (($n * $sumY2) - ($sumY * $sumY)))
    if ($den -ne 0) {
        $summary.correlation_delta_vs_drained = [math]::Round($num / $den, 3)
    }
}

# Write artifacts.
$csvPath = Join-Path $evidence 'cycles.csv'
$rows | Export-Csv -LiteralPath $csvPath -NoTypeInformation -Encoding utf8
$summaryPath = Join-Path $evidence 'summary.json'
$summary | ConvertTo-Json -Depth 8 | Out-File -LiteralPath $summaryPath -Encoding utf8

Write-Host ""
Write-Host "==========================================================="
Write-Host "P1 cold-boot discriminator (Phase 2.1) - cohort summary"
Write-Host "==========================================================="
$summary | ConvertTo-Json -Depth 8 | Write-Host

# Discriminator verdict.
$verdict = 'INCONCLUSIVE'
if ($summary.completed -ge 10 -and $null -ne $summary.delta_line_stats) {
    $cv  = $summary.delta_line_stats.cv
    $cor = $summary.correlation_delta_vs_drained
    if ($null -ne $cv -and $cv -lt 0.25 -and ($null -eq $cor -or [math]::Abs([double]$cor) -lt 0.4)) {
        $verdict = 'FIRMWARE_NOT_READY (tight Δ cluster, weak drained correlation) -> Phase 2.2 firmware-side __PROFILE_READY__ URC'
    } elseif ($null -ne $cor -and $cor -gt 0.5) {
        $verdict = 'HOST_RACE (Δ correlates with drained count) -> Phase 2.2 drain_startup_until_quiet + bounded backoff in method_h_stage2_tx_probe_v2.py'
    } else {
        $verdict = 'MIXED (neither limb dominant; investigate per-cycle stdout before committing to a fix path)'
    }
}
Write-Host ""
Write-Host "VERDICT: $verdict"
"VERDICT: $verdict" | Out-File -LiteralPath (Join-Path $evidence 'verdict.txt') -Encoding utf8
Write-Host ""
Write-Host "Evidence: $evidence"
