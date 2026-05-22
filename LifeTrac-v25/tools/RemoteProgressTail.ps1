# RemoteProgressTail.ps1 — Phase 4 / P5 sidecar poller
# ----------------------------------------------------------------------------
# Dot-source from a bench orchestrator and call Start-RemoteProgressTail
# before launching a long-running probe that was invoked with
# `--progress-file /tmp/lifetrac_p0c/progress_<tag>.txt`. The poller pulls
# the sidecar over `adb -s <serial> exec-out cat <path>` once per second,
# compares to the previously seen content, and emits a `[progress <tag>] ...`
# line to the host console + appends to a local mirror file whenever the
# remote content changes. Call Stop-RemoteProgressTail when the probe
# finishes so the background job is cleaned up.
#
# Design rationale (see 2026-05-21 Open Problems doc v4.1 §4):
#   * The probe writes a single-line overwrite-truncate progress.txt at
#     every major event. A single stat()+read() is sufficient — no log
#     rotation, no tail bookkeeping.
#   * Polling via `adb exec-out cat` avoids any persistent shell that might
#     wedge if the X8 reboots mid-probe (the cold-boot discriminator's
#     historical foot-gun). Each poll is a complete, independent transaction.
#   * Sidecar bypasses PowerShell's Tee-Object / NativeCommandError stream
#     multiplexing entirely (which is what P2 and P5 were both ultimately
#     about): the sidecar is a file, not a stream.
# ----------------------------------------------------------------------------

function Start-RemoteProgressTail {
    [CmdletBinding()]
    param(
        [Parameter(Mandatory)] [string]$AdbSerial,
        [Parameter(Mandatory)] [string]$RemotePath,
        [Parameter(Mandatory)] [string]$LocalMirrorPath,
        [string]$Tag = 'progress',
        [int]$PollIntervalMs = 1000
    )

    # Truncate / create the local mirror up-front so the orchestrator can
    # safely tail it even if the remote file is never written.
    "" | Set-Content -LiteralPath $LocalMirrorPath -Encoding UTF8

    $job = Start-Job -Name "rpt-$Tag" -ArgumentList @(
        $AdbSerial, $RemotePath, $LocalMirrorPath, $Tag, $PollIntervalMs
    ) -ScriptBlock {
        param($serial, $remote, $mirror, $tag, $intervalMs)
        $last = $null
        while ($true) {
            # `adb exec-out cat` is the most predictable read path: no shell
            # quoting, no stdin/stdout interleaving issues, hard fails fast
            # if the device is offline (which the parent job can observe via
            # exit-code or empty output).
            $cur = $null
            try {
                $cur = & adb -s $serial exec-out cat $remote 2>$null
            } catch {
                $cur = $null
            }
            if ($cur -is [array]) { $cur = ($cur -join "`n") }
            if ($null -ne $cur -and $cur -ne $last) {
                $line = $cur.TrimEnd("`r","`n")
                if ($line) {
                    $iso = (Get-Date).ToString("yyyy-MM-ddTHH:mm:ss")
                    $rendered = "[progress $tag $iso] $line"
                    Write-Output $rendered
                    Add-Content -LiteralPath $mirror -Value $rendered -Encoding UTF8
                }
                $last = $cur
            }
            Start-Sleep -Milliseconds $intervalMs
        }
    }
    return $job
}

function Stop-RemoteProgressTail {
    [CmdletBinding()]
    param(
        [Parameter(Mandatory)] $Job,
        [int]$DrainMs = 250
    )
    if (-not $Job) { return }
    try {
        # Give the job up to DrainMs to emit one last poll cycle so we
        # capture the probe's terminal progress line (mode_exit / link_closed).
        Start-Sleep -Milliseconds $DrainMs
        # Receive whatever's queued before we kill it so the orchestrator's
        # console captures the final line(s).
        $tail = Receive-Job -Job $Job -ErrorAction SilentlyContinue
        if ($tail) { $tail | ForEach-Object { Write-Host $_ } }
    } catch { }
    try { Stop-Job -Job $Job -ErrorAction SilentlyContinue | Out-Null } catch { }
    try { Remove-Job -Job $Job -Force -ErrorAction SilentlyContinue | Out-Null } catch { }
}
