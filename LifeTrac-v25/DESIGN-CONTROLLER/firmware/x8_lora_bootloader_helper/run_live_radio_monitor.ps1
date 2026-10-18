#requires -Version 5.1
<#
.SYNOPSIS
    Live LoRa Radio Transmission Monitor & Speed Benchmark.
.DESCRIPTION
    Launches image_rx_daemon on 2D0A and image_tx_daemon on 2E2C, streams
    synthetic frames from host, and logs real-time throughput / goodput (B/s),
    fragment counts, and demodulation metrics (RSSI, SNR).

    PC requirements (bench kit, 2026-10-10): adb on PATH, the Python launcher
    `py` (or -PythonExe), and the packages in bench_tools/requirements-pc.txt.
    The PC MQTT broker this script starts when none answers on -HostIp:1883
    is start_mqtt_broker.py at the repo root, which needs `amqtt`
    (`py -3 -m pip install -r bench_tools/requirements-pc.txt`).

    Parameter names are checked ([CmdletBinding()]): a misspelled name such as
    -DurationSeconds is an error, not a silently ignored argument.

    Profile-2 (DTS) legs must pin a spot-checked carrier with -ForceFrfHz; the
    profile default is 915.000 MHz, the channel of the RS-11.6 external
    emitter. -AllowDefaultCarrier overrides the refusal.

    Bench kit (2026-10-18): -InjectCmd "<script.py> [args]" starts a command
    injector on the base (kf_inject.py, dual_inject.py) when -InjectOn fires:
    first_frame (default; the base rx daemon's first published frame, read on
    the base from its own log) or fault:<code>[:<sub>|:final] (a tractor
    tx_smoke FAULT line, polled every 2 s). The trigger time and the
    injector's output go into the archive (inject.log, inject_trigger.txt).
    At the end every leg prints and archives TRACTOR_L072_RESET
    landed|failed|unknown (from the openocd log of the leg-start SWD reset)
    and BASE_L072_RESET unverified (the gpio163 pulse is not checked).
#>
[CmdletBinding()]
param(
    [string]$TxAdbSerial = "2E2C1209DABC240B",
    [string]$RxAdbSerial = "2D0A1209DABC240B",
    [string]$HostIp       = "192.168.1.79",
    [int]$DurationS       = 30,
    # Saturation mode (2026-07-24): offered load must EXCEED capacity for
    # the reported goodput to be a measurement rather than a traffic
    # report (post-impl review §7.3). At 194 B frames the single-channel
    # BW250 ceiling is ~2 frames/s, so 6 fps ≈ 3x saturation. The
    # publisher prints its own OFFERED line for the evidence bundle.
    [double]$SynthFps     = 2,
    [int]$SynthBudgetB    = 250,
    # LIFETRAC_TX_PIPELINE for the TX daemon: 'v2' serial, 'v3' pipelined
    # (depth-2 firmware mailbox).
    # 2026-07-29: default flipped v2 -> v3 after the RS-0.13b bisection. The
    # v2 default silently changed the code path under test across ~30 runs and
    # cost 38 POINTS of in-stream command delivery (v3 53% vs v2 15% at
    # identical frame size) — a test-fixture regression no code diff could
    # reveal. v3 is the pipelined path with mid-burst RX dispatch and an
    # end-of-train RXCONT re-arm; it is what Run J actually used.
    [string]$TxPipeline   = "v3",
    # RS-2.3 (2026-07-25): outstanding TX_FRAME_REQs the v3 daemon keeps in
    # flight. The new firmware TX ring holds 1 transmitting + 4 parked, so
    # <=4 never NAKs. Keep <=3 at FHSS (P0 latency math, RS-9.7); 4 = DTS.
    [int]$TxPipelineDepth = 2,
    # RS-3.1 (2026-07-25): batch multiple non-key frames per fragment train
    # (amortizes the measured ~44 ms host per-handoff overhead). 1 = on.
    [int]$TxBatch         = 1,
    # RS-3.10 (2026-07-26): prepare train N+1 during train N's airtime;
    # the inter-train gap becomes a designed listening reverse-slot.
    [int]$TxPrepareAhead  = 1,
    # RS-4.14 diagnostic (2026-07-26): 1 = suppress self-heal keyframe
    # requests on the RX side (tests whether command TX feeds FHSS loss).
    [int]$KfRequestDisable = 0,
    # RS-4.1 (2026-07-26): XOR parity per N data fragments (0 = off).
    # 4 aligns one parity with each 4-fragment batched train — absorbs
    # single-fragment loss so evictions (and the request storm they
    # spark at FHSS) never start. ~+25% airtime per train.
    [int]$ParityGroup     = 0,
    # RS-3.10 gap knob passthrough for the policy A/B (ms).
    [int]$TrainGapMs      = 40,
    # RS-5.9 (2026-07-25): where tx_smoke gets its frame feed.
    #   local = broker + synth publisher run ON the tractor (127.0.0.1).
    #           Required now that tractor WiFi is off per MASTER_PLAN §8.13.
    #   host  = legacy wiring via the bench PC's broker over the LAN.
    #   camera = RS-3.3 real-camera path: camera_service captures the USB UVC
    #            camera and runs the encode-to-fit packer, which the synthetic
    #            feed bypasses entirely.
    [ValidateSet("local", "host", "camera")]
    [string]$TxFeed       = "local",
    # UVC capture node for -TxFeed camera. The board enumerates a "C2"
    # (16d0:0ed4) UVC 1.00 device; /dev/video0 is the separate built-in
    # mx6s-csi MIPI bridge, NOT the USB camera.
    [string]$CameraDevice = "/dev/video1",
    # Regulatory profile for BOTH daemons: 0=BENCH_ONLY_FIXED_915 (single
    # channel), 1=FCC_FHSS_50CH_BW250 (wide mask auto-enabled — the firmware
    # validator rejects popcount<50 for profile 1), 2=FCC_DTS_BW500.
    [int]$RegProfile      = 0,
    # RS-11.6 channel escape (2026-08-16): override the carrier both daemons
    # program at profile init (LIFETRAC_FORCE_FRF_HZ in the shared HostLink
    # helper). 0 = profile default (915.000 MHz at profile 2). Routed through
    # $profEnv so BOTH peers always get the same carrier by construction --
    # the same argument as FhssLinkId. The known bench interferer sits in the
    # 915.0 MHz channel; e.g. -ForceFrfHz 917000000 moves the link off it.
    [int]$ForceFrfHz      = 0,
    # Bench kit (2026-10-10): a profile-2 (DTS) leg with -ForceFrfHz 0 is
    # REFUSED unless this switch is given. Profile 2's default carrier is
    # 915.000 MHz, where the RS-11.6 external emitter bursts (~25 ms at -43 to
    # -45 dBm every ~7.08 s); every RS-13.1 DTS leg flew there by accident and
    # its losses fold on the emitter's period (RS-13.1 A19). Pin the carrier
    # picked by a same-day receive-only spot-check instead (BENCH_RUNBOOK prep
    # step 8). Use this switch only for a deliberate 915.000 MHz control leg.
    [switch]$AllowDefaultCarrier,
    # Bench kit (2026-10-10): Python used for the two PC-side processes this
    # script may start -- the PC MQTT broker (start_mqtt_broker.py, needs the
    # `amqtt` package) and the -TxFeed host synthetic publisher. Default `py`
    # (the Windows Python launcher, run as `py -3`); pass a full python.exe
    # path to use a specific interpreter or venv. Replaces a hardcoded
    # per-user interpreter path. See bench_tools/requirements-pc.txt.
    [string]$PythonExe    = "py",
    # RS-12 (2026-08-17): 1 = RX daemon logs one line per data fragment with
    # the firmware us RX timestamp (frag_arrival:). Diagnostic instrument for
    # the silent URC-drop timing shape; adds ~2300 log lines per 300 s leg.
    [int]$LogFragArrivals = 0,
    # RS-12 fix candidate (2026-08-17): 1 = TX daemon never parks the final
    # fragment (restores paced spacing for the last pair; see daemon comment).
    [int]$NoParkLast      = 0,
    # RS-12.11 (2026-09-12): idle-drain quiet gate on the base rx daemon
    # (LIFETRAC_IDLE_DRAIN_QUIET_S). "0" = pre-RS-12.11 behaviour (A/B control).
    [string]$IdleDrainQuietS = "1.5",
    # RS-12.14 (2026-09-12): minimum spacing between base commands while a
    # stream is active (LIFETRAC_CMD_STREAM_MIN_GAP_S); "0.12" = pre-RS-12.14.
    # Backoff/cooldown ride LIFETRAC_PENDING_RETRY_BACKOFF=1 and
    # LIFETRAC_PENDING_GIVEUP_COOLDOWN_S=0 for the A/B control (-RxExtraEnv).
    [string]$CmdStreamMinGapS = "1.0",
    [string]$RxExtraEnv = "",
    # RS-13.1 (2026-09-26): extra `-e K=V` pairs for the camera_service
    # container of -TxFeed camera. "-e LIFETRAC_ENCODE_MODE=9" boots the
    # tractor in VECTOR (codec 6); "-e LIFETRAC_VECTOR_DETAIL=80" is its dial.
    # Same PR #121 rule as -RxExtraEnv: the assembled local below is named
    # differently from this parameter in every letter case.
    [string]$CamExtraEnv = "",
    # RS-12.13 (2026-09-12): the hold applies only to trains of at least this
    # many fragments (LIFETRAC_NO_PARK_LAST_MIN_FRAGS on the tx daemon).
    [int]$NoParkLastMinFrags = 3,
    # RS-0.13b (2026-07-27): 1 = aligned command pump on (default), 0 =
    # idle-drain-only. The Run-J bisection toggles this + Batch + PrepareAhead
    # + ParityGroup one at a time to find which killed the 58% alignment.
    [int]$AlignedPump     = 1,
    # RS-0.12 (2026-07-27): reactive-fire probe. 1 = base fires a no-op PROBE
    # at every fragment-RX-complete and measures echo delivery — the honest
    # in-stream base->tractor delivery experiment. Off = normal run.
    [int]$ReactiveFire    = 0,
    # RS-0.12 phase sweep (ms, comma-separated) e.g. "0,20,40,60,80,100,120".
    # Empty = fire immediately at completion (offset 0).
    [string]$ProbePhaseSweepMs = "",
    # RS-0.12 SIZE sweep (total on-air payload bytes incl the 8 B hop hdr),
    # comma-separated. Prices the "ditto" repeat frame: "23,38" compares a
    # D13 ditto against a D13 control frame at every phase bin. Empty = no
    # padding (probe stays at its natural 16 B).
    [string]$ProbeSizesB  = "",
    # 2026-07-29 ack-cost experiment: copies used for queued tractor replies
    # (echoes/encode acks). Historical default 2. Profile ACKs always use 2.
    [int]$AckCopies       = 2,
    # 0 = tractor does NOT echo probes, so the run measures the IMAGE cost of
    # carrying acks at all (delivery still scored tractor-side).
    [int]$ProbeEcho       = 1,
    # 2026-07-29 FHSS seed identity (RS-10.4). Seeds the FNV-1a ->
    # Fisher-Yates channel permutation. Before the firmware fix the only
    # production call site passed a hardcoded (0,0,0), so every radio in
    # the fleet hopped in the same order.
    #
    # Both are fed via $profEnv, which goes to BOTH daemons — that is
    # deliberate and load-bearing. FhssLinkId is LINK-scoped: if the two
    # ends ever receive different values their permutations diverge, the
    # follower retunes to the wrong channel every slot and never holds
    # lock. Routing it through the shared env block makes that failure
    # unreachable from this harness.
    #
    # 0/0 (default) reproduces the historical shared permutation, so
    # existing evidence stays comparable. Only profile 1 (FHSS) hops, so
    # these have no effect at profiles 0/2.
    [string]$FhssFarmId   = "0",
    [string]$FhssLinkId   = "0",
    # 2026-07-30 RS-11.4: override the tractor's host airtime pacer
    # (_PROFILE_TO_BUDGET_US, default 930000 us at DTS over a rolling 1.0 s
    # window). Diagnostic only -- this paces our own duty cycle, so never ship
    # a non-zero value as a default. 0 = leave the profile default alone.
    # First blocked fragment index = floor(budget/99904) + TxPipelineDepth - 1.
    [int]$AirtimeBudgetUs = 0,
    # 2026-07-30 RS-11.4: tractor airtime pacing mode.
    #   bucket = token bucket (default) — bursts to budget then hard-stalls,
    #            which is what opens the mid-train dead-air hole.
    #   smooth = hold each fragment to the spacing the duty target implies,
    #            at _PACING_HEADROOM (0.92) of budget so the rolling-window
    #            backstop stays a backstop rather than a co-participant.
    # The first headroom-less smooth attempt REGRESSED loss 2.45% -> 19.41%;
    # with headroom it was verified n=2 and smooth became the daemon default
    # on 2026-07-30. Kept here so bucket can be re-run against the evidence.
    [ValidateSet("bucket", "smooth")]
    [string]$PacingMode   = "smooth",
    # 2026-07-30 RS-11.4: fraction of the airtime budget smooth pacing aims at.
    # 0 = leave the daemon default (0.92). Raising it recovers saturated
    # goodput but only until the rolling-window backstop starts firing again,
    # which is exactly the compounding failure headroom exists to prevent.
    [double]$PacingHeadroom = 0,
    # RS-11.5 diagnostic (2026-08-02): 0 = tractor never arms RXCONT, so
    # the firmware's per-fragment re-arm chain never starts and the radio
    # stays STANDBY between fragments. Command downlink fully deaf —
    # diagnostic runs only (discriminates the slot-(total-2) CRC
    # corruption mechanism). Production default 1.
    [ValidateSet(0, 1)]
    [int]$RxcontArm = 1,
    # RS-11.5 diagnostic (2026-08-02): TX power override in dBm (empty =
    # profile default 14). CFG_SET at daemon startup; the firmware's ERP
    # clamp still applies, so this can only go below the ceiling. Used for
    # the SIR discriminator on the fixed RF switch.
    [string]$TxPowerDbm = "",
    # Bench kit (2026-10-18): command injector on the BASE, started by
    # -InjectOn. The value is the injector script and its arguments, e.g.
    # "dual_inject.py 250" or "kf_inject.py 15 20"; it runs in
    # lifetrac-v25:latest with /tmp/lifetrac_strict as /work, driven by
    # bench_tools/inject_watch.sh (pushed with the injector when the script is
    # in bench_tools). Leg W (2026-10-18) started its injector from this
    # script's 2-line console tail and fired 2 min 41 s late; the trigger now
    # reads the daemon's own log. Empty (default) = no injector, as before.
    [string]$InjectCmd    = "",
    # When the -InjectCmd injector starts:
    #   first_frame (default)  the base rx daemon's first `published frame_id`,
    #                          polled every 0.5 s on the base from rx_smoke's log;
    #   fault:<code>           the first tractor tx_smoke FAULT line with that
    #                          code; fault:<code>:<sub> also needs that sub byte,
    #                          fault:<code>:final needs sub bit 7 (the
    #                          RX_SCAN_FAILED that enters FAILED: fault:0x0D:final
    #                          is RS-12.15 leg X). Polled from the PC every ~2 s.
    [string]$InjectOn     = "",
    # Archive final logs + parameters under bench-evidence/ with the git
    # SHA in the folder name (evidence discipline per CODE REVIEWS docs).
    [switch]$Archive
)

Set-StrictMode -Version Latest
$ErrorActionPreference = "Continue"

# --- Helpers (bench kit, 2026-10-18) -----------------------------------------
# Pure functions, no board access: bench_tools/test_run_live_radio_monitor.ps1
# loads them from this file's AST (the script body never runs) and feeds them
# sample adb output.

function ConvertFrom-AdbOutput {
    # adb from platform-tools 37 ends every line with \r\r\n (see adb_sh in
    # bench_tools/build_tractor_image.sh, commit fb96439c). PowerShell's
    # native-output reader treats a lone CR as a line end, so
    # `$x = cmd /c "adb ..."` gives every line followed by an EMPTY element and
    # no trailing CR; a single captured string (Out-String, a file, a .NET
    # reader) keeps the CRs instead. Either way this returns the non-empty
    # lines with trailing whitespace (CRs included) removed. Use it before a
    # whole-line compare, an [int] parse or Select-Object -First/-Last.
    param([object]$Output)
    foreach ($o in @($Output)) {
        if ($null -eq $o) { continue }
        foreach ($l in ("$o" -split "[`r`n]+")) {
            $t = $l.TrimEnd()
            if ($t -ne "") { $t }
        }
    }
}

function ConvertTo-InjectSpec {
    # Checks -InjectCmd / -InjectOn (throws with the reason) and returns
    # @{ Cmd; Script; On = first_frame|fault; Code; Sub; Final; Label }.
    # The command reaches the base shell inside nested cmd/adb/sh quotes, so
    # only plain words pass: <script>.py, then arguments of [A-Za-z0-9_.,:=+-].
    param([string]$Cmd, [string]$On)
    $c = "$Cmd".Trim()
    $o = "$On".Trim()
    if ($c -eq "") {
        throw "-InjectOn '$o' needs -InjectCmd (the injector to start on the base, e.g. -InjectCmd `"dual_inject.py 250`")."
    }
    $toks = @($c -split '\s+')
    if ($toks[0] -notmatch '^[A-Za-z0-9_][A-Za-z0-9_.-]*\.py$') {
        throw "-InjectCmd '$c': the first word must be the injector script, a .py file name (e.g. dual_inject.py)."
    }
    foreach ($t in @($toks | Select-Object -Skip 1)) {
        if ($t -notmatch '^[A-Za-z0-9_.,:=+-]+$') {
            throw "-InjectCmd '$c': argument '$t' may hold only letters, digits and _ . , : = + -"
        }
    }
    $spec = @{ Cmd = ($toks -join ' '); Script = $toks[0]; On = ""; Code = -1; Sub = -1; Final = $false; Label = "" }
    if ($o -eq "" -or $o -ieq "first_frame") {
        $spec.On = "first_frame"
        $spec.Label = "first_frame"
    } elseif ($o -match '^fault:(?:0x)?([0-9A-Fa-f]{1,2})(?::(?:(final)|(?:0x)?([0-9A-Fa-f]{1,2})))?$') {
        $spec.On = "fault"
        $spec.Code = [Convert]::ToInt32($Matches[1], 16)
        $spec.Label = "fault:0x{0:X2}" -f $spec.Code
        if ($Matches[2]) {
            $spec.Final = $true
            $spec.Label += ":final"
        } elseif ($Matches[3]) {
            $spec.Sub = [Convert]::ToInt32($Matches[3], 16)
            $spec.Label += (":0x{0:X2}" -f $spec.Sub)
        }
    } else {
        throw "-InjectOn '$o': use first_frame, fault:<code>, fault:<code>:<sub> or fault:<code>:final (hex bytes, e.g. fault:0x0D:final)."
    }
    return $spec
}

function Test-InjectFaultLine {
    # True when a tx_smoke log line is a FAULT matching the fault: spec. The
    # daemon prints `code=0x%02X sub=0x%02X` (method_g_stage1_probe.py
    # format_fault_payload). Code 0x0D means both TX_FIFO_RB_MISMATCH (sub =
    # fragment index) and RX_SCAN_FAILED (host_types.h); `final` tests sub
    # bit 7, which only RX_SCAN_FAILED sets (fragment indexes stay < 0x80).
    param([string]$Line, [hashtable]$Spec)
    if ($Line -notmatch 'FAULT') { return $false }
    if ($Line -notmatch 'code=0x([0-9A-Fa-f]{2}) sub=0x([0-9A-Fa-f]{2})') { return $false }
    $code = [Convert]::ToInt32($Matches[1], 16)
    $sub = [Convert]::ToInt32($Matches[2], 16)
    if ($code -ne $Spec.Code) { return $false }
    if ($Spec.Final) { return (($sub -band 0x80) -ne 0) }
    if ($Spec.Sub -ge 0) { return ($sub -eq $Spec.Sub) }
    return $true
}

function Get-InjectWatchPid {
    # The watcher pid from `inject_watch.sh start` ("INJECT_WATCH_SPAWNED pid=N"), or $null.
    param([object]$Output)
    foreach ($l in @(ConvertFrom-AdbOutput $Output)) {
        if ($l -match '^INJECT_WATCH_SPAWNED pid=(\d+)$') { return [int]$Matches[1] }
    }
    return $null
}

function Get-InjectSummary {
    # Reads the base watcher log (inject_watch.sh marker lines) and returns
    # @{ Status; Detail; Trigger; Start; Exit }; Status is finished, failed,
    # stopped_at_leg_end, not_started or unknown.
    param([object]$Output)
    $r = @{ Status = "unknown"; Detail = "no injector log came back from the base"; Trigger = ""; Start = ""; Exit = "" }
    $armed = $false; $notStarted = ""; $stopInj = $false; $stopWatch = $false
    foreach ($l in @(ConvertFrom-AdbOutput $Output)) {
        if ($l -match '^INJECT_(ARMED|WATCH) ') { $armed = $true }
        elseif ($l -match '^INJECT_TRIGGER (\S+) \S+ ?(.*)$') { $r.Trigger = ("$($Matches[1]) $($Matches[2])").Trim() }
        elseif ($l -match '^INJECT_START (\S+) ') { $r.Start = $Matches[1] }
        elseif ($l -match '^INJECT_EXIT \S+ \S+ rc=(\d+)') { $r.Exit = $Matches[1] }
        elseif ($l -match '^INJECT_NOT_STARTED \S+ \S+ ?(.*)$') { $notStarted = $Matches[1] }
        elseif ($l -match '^INJECT_STOP .*injector_running=([01]) watcher_running=([01])') {
            $stopInj = ($Matches[1] -eq '1'); $stopWatch = ($Matches[2] -eq '1')
        }
    }
    if ($notStarted -ne "") {
        $r.Status = "not_started"; $r.Detail = $notStarted
    } elseif ($r.Start -ne "" -and $stopInj) {
        $r.Status = "stopped_at_leg_end"; $r.Detail = "still running when the leg ended; the harness removed it"
    } elseif ($r.Start -ne "" -and $r.Exit -ne "") {
        if ($r.Exit -eq "0") { $r.Status = "finished"; $r.Detail = "ran to its end" }
        else { $r.Status = "failed"; $r.Detail = "injector exit status $($r.Exit)" }
    } elseif ($r.Start -ne "") {
        $r.Status = "unknown"; $r.Detail = "started, but its log has no exit marker"
    } elseif ($stopWatch) {
        $r.Status = "not_started"; $r.Detail = "its trigger had not fired when the leg ended"
    } elseif ($armed) {
        $r.Status = "not_started"; $r.Detail = "its trigger never fired"
    }
    return $r
}

function Get-TractorResetVerdict {
    # Verdict on the leg-start SWD reset of the tractor's L072 from the
    # tractor's /tmp/lifetrac_p0c/ocd_reset.log (openocd + 08_boot_user_app.cfg,
    # with OCD_EXIT=<rc> appended) and the __OCD_LOG_RC / __OCD_RUNNING markers
    # of the end-of-leg read. Returns @{ Verdict = landed|failed|unknown; Reason }.
    # "landed" = openocd drove H7 PF4 (the L072 NRST net) low for 250 ms and
    # read it back low, then high (Phase D of the cfg). It is not a readback
    # from the L072 itself; the counter brackets (deltas.py) stay the final word.
    # -Tag is this leg's OCD_LEG=<stamp>, the log's first line: a log without
    # it is an earlier leg's (only this harness writes ocd_reset.log, and each
    # spawn truncates it). -SpawnOutput is what the leg-start spawn printed.
    param([object]$Output, [bool]$CfgOk = $true, [string]$Tag = "", [object]$SpawnOutput = $null)
    $lines = @(ConvertFrom-AdbOutput $Output)
    $logRc = $null; $running = $false; $exit = $null; $low = $null; $high = $null; $firstErr = $null
    foreach ($l in $lines) {
        if ($l -match '^__OCD_LOG_RC=(\d+)$') { $logRc = [int]$Matches[1] }
        elseif ($l -match '^__OCD_RUNNING=1$') { $running = $true }
        elseif ($l -match '^OCD_EXIT=(\d+)$') { $exit = [int]$Matches[1] }
        elseif ($l -match 'GPIOF_IDR\.PF_4 \(low\)\s*=\s*(\d+)') { $low = $Matches[1] }
        elseif ($l -match 'GPIOF_IDR\.PF_4 \(high\)\s*=\s*(\d+)') { $high = $Matches[1] }
        elseif ($null -eq $firstErr -and $l -match '^(Error|error)\b') { $firstErr = $l }
    }
    $cfgNote = ""
    if (-not $CfgOk) { $cfgNote = "; 08_boot_user_app.cfg was missing at launch" }
    $spawnNote = ""
    if ($null -ne $SpawnOutput) {
        $sp = @(ConvertFrom-AdbOutput $SpawnOutput)
        if ($sp -notcontains "swd_reset_spawned") {
            $spText = "nothing"
            if ($sp.Count -gt 0) { $spText = "'" + ($sp -join ' | ') + "'" }
            $spawnNote = "; the leg-start spawn printed $spText, not swd_reset_spawned"
        }
    }
    if ($null -eq $logRc) {
        return @{ Verdict = "unknown"; Reason = "could not read /tmp/lifetrac_p0c/ocd_reset.log from the tractor (adb failed?)$spawnNote$cfgNote" }
    }
    if ($logRc -ne 0) {
        return @{ Verdict = "failed"; Reason = "no openocd log on the tractor: the background reset never started$spawnNote$cfgNote" }
    }
    if ($Tag -ne "" -and $lines -notcontains $Tag) {
        $content = @($lines | Where-Object { $_ -notmatch '^__OCD_' })
        if ($content.Count -eq 0) {
            return @{ Verdict = "failed"; Reason = "the openocd log on the tractor is empty: the reset subshell was cut off before it wrote $Tag (a hangup at spawn?)$spawnNote$cfgNote" }
        }
        $whose = "an untagged log from an older harness"
        $prevTag = @($content | Where-Object { $_ -match '^OCD_LEG=' })
        if ($prevTag.Count -gt 0) { $whose = "the log of $($prevTag[0])" }
        return @{ Verdict = "failed"; Reason = "the openocd log on the tractor is not this leg's (no $Tag; it is $whose): the leg-start reset was not spawned$spawnNote$cfgNote" }
    }
    $banner = (@($lines | Where-Object { $_ -match '^Open On-Chip Debugger' }).Count -gt 0)
    $exitNote = ""
    if ($null -ne $exit) {
        $exitNote = ", openocd exit $exit"
        if ($exit -ge 128) { $exitNote += " (signal $($exit - 128))" }
    } elseif ($running) { $exitNote = ", openocd still running" }
    if ($null -ne $low -and $null -ne $high) {
        if ($low -eq "0" -and $high -eq "1") {
            return @{ Verdict = "landed"; Reason = "PF4/NRST read back low then high$exitNote" }
        }
        return @{ Verdict = "failed"; Reason = "PF4/NRST read back low=$low high=$high (expect 0 then 1)$exitNote" }
    }
    # With OCD_EXIT in the log this leg's job is over; an openocd still seen
    # running is some other one, so the exit status decides.
    if ($running -and $null -eq $exit) {
        return @{ Verdict = "failed"; Reason = "openocd still running at leg end and no NRST pulse in its log (hung)$cfgNote" }
    }
    # The shell reports a job killed by signal N as 128+N. 129 (SIGHUP) is the
    # likely one here: the adb/sudo session hangup reaching nohup before nohup
    # ignored HUP (the subshell's `trap :` is reset in its children). Without
    # the openocd banner, openocd itself never started.
    if ($null -ne $exit -and $exit -ge 128) {
        $sig = $exit - 128
        $what = "killed by signal $sig"
        if ($sig -eq 1) { $what = "killed by SIGHUP (likely the adb/sudo session hangup reaching nohup before nohup ignored HUP)" }
        elseif ($sig -eq 9) { $what = "killed by SIGKILL" }
        elseif ($sig -eq 15) { $what = "killed by SIGTERM" }
        if (-not $banner) { $what += "; openocd never started (no banner in its log)" }
        return @{ Verdict = "failed"; Reason = "the nohup openocd job was $what (exit $exit) before the NRST pulse$cfgNote" }
    }
    if ($null -ne $exit -and $exit -ne 0) {
        $why = "no first error line"
        if ($firstErr) { $why = $firstErr }
        return @{ Verdict = "failed"; Reason = "openocd exit $exit before the NRST pulse: $why$cfgNote" }
    }
    if ($firstErr) {
        return @{ Verdict = "failed"; Reason = "no NRST pulse in the openocd log: $firstErr$cfgNote" }
    }
    return @{ Verdict = "unknown"; Reason = "the openocd log has no NRST pulse and no error$exitNote$cfgNote" }
}

# Bench kit (2026-10-10): refuse a DTS leg on the default carrier BEFORE
# anything touches a board. Legs that pass -ForceFrfHz are unaffected.
if ($RegProfile -eq 2 -and $ForceFrfHz -eq 0 -and -not $AllowDefaultCarrier) {
    throw ("Refusing a profile-2 (DTS) leg with -ForceFrfHz 0: the profile default " +
        "carrier is 915.000 MHz, the channel of the known RS-11.6 external emitter " +
        "(~25 ms bursts at -43 to -45 dBm every ~7.08 s; RS-13.1 DTS losses folded on " +
        "its period). Run a same-day receive-only channel spot-check with the tractor " +
        "parked (channel_survey_sniff.py or hunt_sniff.ps1, then tools/survey_compare.py; " +
        "BENCH_RUNBOOK prep step 8) and pass -ForceFrfHz <the pick>, e.g. -ForceFrfHz " +
        "927500000 if that channel is clean today. For a deliberate 915.000 MHz control " +
        "leg add -AllowDefaultCarrier.")
}
if ($RegProfile -eq 2 -and $ForceFrfHz -eq 0) {
    Write-Warning "-AllowDefaultCarrier: this DTS leg flies on 915.000 MHz, the RS-11.6 emitter channel."
}
# Bench kit (2026-10-18): check the injector options before anything touches a
# board. $injectSpec stays $null (no injector, the old behaviour) without them.
$injectSpec = $null
if ($InjectCmd -ne "" -or $InjectOn -ne "") {
    $injectSpec = ConvertTo-InjectSpec -Cmd $InjectCmd -On $InjectOn
}

# Profile env for both daemons; profile 1 needs the 50-ch wide mask.
$profEnv = "-e LIFETRAC_REG_PROFILE=$RegProfile"
if ($RegProfile -eq 1) { $profEnv = "$profEnv -e LIFETRAC_FHSS_WIDE_MASK=1" }
# FHSS seed — same value to both daemons by construction (see the
# FhssLinkId param comment for why that matters).
$profEnv = "$profEnv -e LIFETRAC_FHSS_FARM_ID=$FhssFarmId -e LIFETRAC_FHSS_LINK_ID=$FhssLinkId"
if ($AirtimeBudgetUs -gt 0) { $profEnv = "$profEnv -e LIFETRAC_AIRTIME_BUDGET_US=$AirtimeBudgetUs" }
$profEnv = "$profEnv -e LIFETRAC_AIRTIME_PACING=$PacingMode"
if ($PacingHeadroom -gt 0) { $profEnv = "$profEnv -e LIFETRAC_PACING_HEADROOM=$PacingHeadroom" }
if ($NoParkLast -eq 1) { $profEnv = "$profEnv -e LIFETRAC_NO_PARK_LAST=1 -e LIFETRAC_NO_PARK_LAST_MIN_FRAGS=$NoParkLastMinFrags" }
if ($LogFragArrivals -eq 1) {
    # One knob, both ends: RX logs frag_arrival (fw us + host wall), TX logs
    # txdone_arrival (host wall + toa_us). Routed via $profEnv so both
    # daemons get their flag; each reads only its own.
    $profEnv = "$profEnv -e LIFETRAC_LOG_FRAG_ARRIVALS=1 -e LIFETRAC_LOG_TX_DONE=1"
}
if ($ForceFrfHz -gt 0) {
    # Review catch (PR #106): the shared helper writes this straight into the
    # 24-bit FRF registers with no band check, so a typo could key BOTH
    # radios outside 902-928 MHz. Reject any center whose 500 kHz occupied
    # bandwidth would cross the band edges.
    if ($ForceFrfHz -lt 902250000 -or $ForceFrfHz -gt 927750000) {
        throw "ForceFrfHz $ForceFrfHz out of range: center must be 902.25-927.75 MHz so the 500 kHz occupied BW stays inside 902-928"
    }
    $profEnv = "$profEnv -e LIFETRAC_FORCE_FRF_HZ=$ForceFrfHz"
}

# Resolve adb from PATH (the winget package dir name varies per machine/source).
# No per-user fallback path: on another PC it only produced a confusing
# "not found" later. Nothing has touched a board yet at this point.
$adbExe = (Get-Command adb -ErrorAction SilentlyContinue).Source
if (-not $adbExe) {
    throw "adb not on PATH. Install Android platform-tools (bench_tools/PC_SETUP.md: winget install Google.PlatformTools), open a new shell, and check 'adb version'."
}
$repoRoot = (Resolve-Path "$PSScriptRoot\..\..\..\..").Path
$baseStation = Join-Path $repoRoot "LifeTrac-v25\DESIGN-CONTROLLER\base_station"
$tractorX8   = Join-Path $repoRoot "LifeTrac-v25\DESIGN-CONTROLLER\firmware\tractor_x8"
$helperDir   = $PSScriptRoot

# PC-side Python (broker + -TxFeed host publisher). `py` is the Windows Python
# launcher and takes `-3` to pick the newest Python 3; any other value is used
# as the interpreter itself. The broker needs `amqtt`
# (bench_tools/requirements-pc.txt).
$pyPrefixArgs = @()
if ([System.IO.Path]::GetFileNameWithoutExtension($PythonExe) -ieq "py") { $pyPrefixArgs = @("-3") }
if (-not (Get-Command $PythonExe -ErrorAction SilentlyContinue)) {
    Write-Warning "PythonExe '$PythonExe' not found; the PC broker / host publisher cannot start. Install Python 3 (py launcher) or pass -PythonExe <path\to\python.exe>."
}

Write-Host "=== STARTING LIVE LORA RADIO MONITOR ($DurationS s) ===" -ForegroundColor Cyan
Write-Host "TX Serial: $TxAdbSerial"
Write-Host "RX Serial: $RxAdbSerial"
Write-Host "MQTT Host: $HostIp"
# -HostIp must be THIS PC's LAN address (the boards dial it for the control
# broker). The default is the original bench PC's DHCP lease; warn, do not
# change it, when it is not one of this machine's addresses.
try {
    $localIps = @(Get-NetIPAddress -AddressFamily IPv4 -ErrorAction Stop | ForEach-Object { $_.IPAddress })
    if ($localIps.Count -gt 0 -and $localIps -notcontains $HostIp) {
        Write-Warning "HostIp $HostIp is not an address of this PC ($($localIps -join ', ')). Pass -HostIp <this PC's LAN IPv4>."
    }
} catch { }

# 1. Ensure Host MQTT Broker is running
$socket = New-Object System.Net.Sockets.TcpClient
$asyncResult = $socket.BeginConnect($HostIp, 1883, $null, $null)
$waitResult = $asyncResult.AsyncWaitHandle.WaitOne(1000, $false)
if (-not $waitResult -or -not $socket.Connected) {
    Write-Host "[MQTT] Starting host broker on $($HostIp):1883 ($PythonExe $($pyPrefixArgs -join ' ') start_mqtt_broker.py; needs amqtt)..." -ForegroundColor Yellow
    Start-Process -FilePath $PythonExe -ArgumentList (@($pyPrefixArgs) + @("start_mqtt_broker.py")) -WorkingDirectory $repoRoot -PassThru -NoNewWindow
    Start-Sleep -Seconds 2
    # Advisory re-probe only (the leg continues either way, as before): a
    # broker that did not come up is almost always a missing `amqtt`.
    $probe2 = New-Object System.Net.Sockets.TcpClient
    try {
        $ar2 = $probe2.BeginConnect($HostIp, 1883, $null, $null)
        if (-not $ar2.AsyncWaitHandle.WaitOne(1000, $false) -or -not $probe2.Connected) {
            Write-Warning "PC broker not answering on $($HostIp):1883 after start. Check: $PythonExe $($pyPrefixArgs -join ' ') -c `"import amqtt`" (install with: py -3 -m pip install -r bench_tools\requirements-pc.txt)."
        }
    } catch { } finally { $probe2.Close() }
} else {
    $socket.Close()
    Write-Host "[MQTT] Host broker active at $($HostIp):1883" -ForegroundColor Green
}

# 2. Clean up previous containers
Write-Host "[CLEANUP] Stopping old containers..."
cmd /c "`"$adbExe`" -s $TxAdbSerial shell `"echo fio | sudo -S docker rm -f tx_smoke 2>/dev/null`"" | Out-Null
# 2026-10-18: bench_inject is the -InjectCmd injector container. Removed on
# every launch, with or without -InjectCmd, so one left by an interrupted leg
# cannot publish commands into this one (a no-op when it does not exist).
cmd /c "`"$adbExe`" -s $RxAdbSerial shell `"echo fio | sudo -S docker rm -f rx_smoke bench_inject 2>/dev/null`"" | Out-Null
# 2026-08-08: the PRODUCTION `tractor-camera` container maps
# /dev/ttymxc3 and is recreated by compose on every reboot. It steals the
# radio UART, and the failure looks nothing like contention: the L072
# still TRANSMITS (raw capture shows `C:SEND_BOOT` + a COBS READY URC)
# but never answers a HostLink request, and SWD resets do not help — it
# reads as the documented break-storm ISR livelock. Cost an hour of
# misdiagnosis after an unplanned reboot. `stop`, not `rm`: the
# container stays defined so `docker start tractor-camera` restores the
# production stack after bench work.
# 2026-09-12: a bare `docker stop` is undone by the container's systemd unit
# within the same boot (it came back mid-session twice). Stop the unit first;
# it restarts at the next boot, so production is untouched long-term.
foreach ($s in @($TxAdbSerial, $RxAdbSerial)) {
    cmd /c "`"$adbExe`" -s $s shell `"echo fio | sudo -S -p '' systemctl stop lifetrac-camera.service 2>/dev/null`"" | Out-Null
    cmd /c "`"$adbExe`" -s $s shell `"echo fio | sudo -S -p '' docker stop -t 3 tractor-camera 2>/dev/null`"" | Out-Null
}

# 3. Deploy code
Write-Host "[DEPLOY] Syncing code to both boards..."
foreach ($s in @($TxAdbSerial, $RxAdbSerial)) {
    cmd /c "`"$adbExe`" -s $s shell `"echo fio | sudo -S -p '' mkdir -p /tmp/lifetrac_strict ; echo fio | sudo -S -p '' chmod 0777 /tmp/lifetrac_strict`"" | Out-Null
    cmd /c "`"$adbExe`" -s $s push `"$(Join-Path $helperDir 'method_g_stage1_probe.py')`" /tmp/lifetrac_strict/" | Out-Null
    cmd /c "`"$adbExe`" -s $s push `"$(Join-Path $helperDir 'method_h_stage2_tx_probe_v2.py')`" /tmp/lifetrac_strict/" | Out-Null
    cmd /c "`"$adbExe`" -s $s push `"$(Join-Path $baseStation 'lora_proto.py')`" /tmp/lifetrac_strict/" | Out-Null
    cmd /c "`"$adbExe`" -s $s push `"$(Join-Path $baseStation 'image_rx_daemon.py')`" /tmp/lifetrac_strict/" | Out-Null
    cmd /c "`"$adbExe`" -s $s push `"$(Join-Path $baseStation 'cmd_timing.py')`" /tmp/lifetrac_strict/" | Out-Null
    cmd /c "`"$adbExe`" -s $s push `"$(Join-Path $tractorX8 'image_tx_daemon.py')`" /tmp/lifetrac_strict/" | Out-Null
    cmd /c "`"$adbExe`" -s $s push `"$(Join-Path $tractorX8 'camera_service.py')`" /tmp/lifetrac_strict/" | Out-Null
    cmd /c "`"$adbExe`" -s $s push `"$(Join-Path $repoRoot 'publish_synthetic_frames.py')`" /tmp/lifetrac_strict/" | Out-Null
    cmd /c "`"$adbExe`" -s $s push `"$(Join-Path $helperDir 'bench_mqtt.conf')`" /tmp/lifetrac_strict/" | Out-Null
    cmd /c "`"$adbExe`" -s $s push `"$(Join-Path $baseStation 'image_pipeline')`" /tmp/lifetrac_strict/" | Out-Null
    # 2026-07-30 RS-3.3: the TRACTOR's pipeline package too — image_tx_daemon
    # needs frame_format from the base copy while camera_service needs
    # ipc_to_h747/tile_cache from this one, so the tractor genuinely requires
    # both. (Until RS-5.7 landed 2026-08-16 both packages were named
    # `image_pipeline` and MERGED into one on-device directory; the X8-side
    # package is now `x8_image_pipeline`, so they simply coexist.)
    cmd /c "`"$adbExe`" -s $s push `"$(Join-Path $tractorX8 'x8_image_pipeline')`" /tmp/lifetrac_strict/" | Out-Null
    $pahoLocal = Join-Path $repoRoot "_paho_pull\paho"
    if (Test-Path $pahoLocal) {
        cmd /c "`"$adbExe`" -s $s push `"$pahoLocal`" /tmp/lifetrac_strict/" | Out-Null
    }
}
# 3a. -InjectCmd (2026-10-18): the base needs bench_tools/inject_watch.sh and the
# injector script in /tmp/lifetrac_strict. The watcher goes up LF-only (a CRLF
# checkout would break it on the board); the injector is pushed when it is in
# bench_tools, otherwise it must already be staged there (legs/stage_boards.sh).
if ($injectSpec) {
    $injWatchSrc = Join-Path $helperDir "bench_tools\inject_watch.sh"
    $injWatchTmp = Join-Path ([System.IO.Path]::GetTempPath()) "lifetrac_inject_watch.sh"
    [System.IO.File]::WriteAllText($injWatchTmp, ((Get-Content -Raw -LiteralPath $injWatchSrc) -replace "`r", ""))
    cmd /c "`"$adbExe`" -s $RxAdbSerial push `"$injWatchTmp`" /tmp/lifetrac_strict/inject_watch.sh" | Out-Null
    $injScriptLocal = Join-Path $helperDir "bench_tools\$($injectSpec.Script)"
    if (Test-Path -LiteralPath $injScriptLocal) {
        cmd /c "`"$adbExe`" -s $RxAdbSerial push `"$injScriptLocal`" /tmp/lifetrac_strict/" | Out-Null
    }
    $injCheck = @(ConvertFrom-AdbOutput (cmd /c "`"$adbExe`" -s $RxAdbSerial shell `"test -s /tmp/lifetrac_strict/inject_watch.sh && test -s /tmp/lifetrac_strict/$($injectSpec.Script) && echo INJ-STAGED || echo INJ-MISSING`"" 2>$null))
    if ($injCheck -notcontains "INJ-STAGED") {
        throw "-InjectCmd: /tmp/lifetrac_strict/inject_watch.sh or /tmp/lifetrac_strict/$($injectSpec.Script) is not on the base ($RxAdbSerial). Stage it (legs/stage_boards.sh) or put the script in bench_tools. No daemon was started."
    }
}

# 3b. Warm up the LAN path board->host. The X8 eth0 ARP entry for the host can
# sit (incomplete) until the HOST talks first; without this, the RX daemon's
# MQTT connect times out even though the broker is up. Ping each board from the
# host, then have each board ping the host, then verify TCP 1883 opens.
Write-Host "[NET] Warming board<->host LAN path + verifying broker reachability..."
# RS-5.9: in local feed mode the tractor never touches the LAN, so only the
# RX (base, ethernet) leg is warmed; the TX-FATAL branch applies to host mode.
$warmPairs = @(, @($RxAdbSerial, "RX"))
if ($TxFeed -eq "host") { $warmPairs = @(, @($TxAdbSerial, "TX")) + $warmPairs }
foreach ($pair in $warmPairs) {
    $s = $pair[0]; $tag = $pair[1]
    $m = cmd /c "`"$adbExe`" -s $s shell `"ip -4 addr show eth0 2>/dev/null; ip -4 addr show wlan0 2>/dev/null`"" 2>$null |
        Select-String -Pattern 'inet (\d+\.\d+\.\d+\.\d+)/' | Select-Object -First 1
    $bIp = if ($m) { $m.Matches.Groups[1].Value } else { "" }
    $ok = $false
    for ($try = 1; $try -le 5; $try++) {
        # Host->board ping is the key step: the host's ARP request broadcast
        # teaches the board the host's MAC even if the ping itself is lost.
        if ($bIp) { cmd /c "ping -n 2 -w 1500 $bIp" 2>$null | Out-Null }
        $probe = cmd /c "`"$adbExe`" -s $s shell `"ping -c 1 -W 2 $HostIp >/dev/null 2>&1; timeout 3 sh -c 'echo > /dev/tcp/$HostIp/1883' && echo TCP-OK || echo TCP-FAIL`"" 2>$null
        if ("$probe" -match "TCP-OK") { $ok = $true; break }
        Start-Sleep -Milliseconds 700
    }
    if ($ok) {
        Write-Host "  [$tag $s] board->host TCP 1883 OK (board ip=$bIp, try $try)"
        # Pin the host's ARP entry statically on the board. The base X8's
        # eth0 ARP for the host expires within ~1 min and its own ARP
        # broadcasts get no reply (host on Wi-Fi; AP eats board->host
        # broadcasts), so a warm-up alone rots before the daemon connects.
        cmd /c "`"$adbExe`" -s $s shell `"echo fio | sudo -S -p '' sh -c 'M=`$(arp -n $HostIp | grep -o -E ..:..:..:..:..:..); arp -s $HostIp `$M 2>/dev/null && echo ARP-PINNED `$M'`"" 2>$null
    } elseif ($tag -eq "TX") {
        Write-Host "  [$tag $s] FATAL: cannot reach broker $($HostIp):1883 from board (ip=$bIp) after 5 tries" -ForegroundColor Red
        exit 1
    } else {
        # RX publishes to the base board's own mosquitto (127.0.0.1),
        # so host-broker reachability is only needed for the TX feed.
        Write-Host "  [$tag $s] WARN: host broker unreachable from board (ip=$bIp) - RX uses its local broker, continuing" -ForegroundColor Yellow
    }
}

# 4. Reset L072s and attach daemons.
# RX board (base): a gpio163 pulse, UNVERIFIED. gpio163 is not an i.MX pin:
# it is line 3 of the x8h7 GPIO chip (gpiochip5, DESIGN-LORAFIRMWARE/04), so the
# write only reaches the L072's NRST if the base's x8h7 bridge is up and its
# H7 firmware drives PF4 (the L072 NRST net) for that line. It has not reset
# the base L072 since at least 2026-08-02 (RS-11.4 RESULTS: counters survived
# ~8 launches) nor on 2026-10-18 (counters carried across legs V-W-W2-X; RS-12.15
# RESULTS Observation 2), while openocd driving PF4 over SWD did reset it
# (RS-11.4 leg E). Score base brackets as post - pre. The pulse is kept
# unchanged (changing the base reset needs its own GO); its output, which
# used to be discarded, now goes into the archive.
# TX board (tractor): gpio163 is dead (writes no-op, RS-5.2) — use the OpenOCD SWD
# path (H7 PA11/BOOT0 low + PF4/NRST via /dev/mem). CRITICAL ORDER: the
# tractor L072 wedges (TX-line break storm -> ISR livelock) when it runs
# with /dev/ttymxc3 CLOSED, so launch the TX daemon FIRST (it holds the
# port open and retries VER x5), then fire the SWD reset into the waiting
# reader.
Write-Host "[RESET] RX (base) L072: gpio163 pulse (UNVERIFIED: it has not reset the base L072 since 2026-08; base counters carry across legs)..."
$nrstCmd = "echo fio | sudo -S -p '' sh -c '[ -d /sys/class/gpio/gpio163 ] || echo 163 > /sys/class/gpio/export; echo out > /sys/class/gpio/gpio163/direction; echo 1 > /sys/class/gpio/gpio163/value; sleep 0.02; echo 0 > /sys/class/gpio/gpio163/value; sleep 0.10; echo 1 > /sys/class/gpio/gpio163/value'"
# Same pulse as before; stderr joins stdout on the board so a failed sysfs
# write (e.g. "Connection timed out" from a stalled x8h7 bridge) is recorded.
$nrstOut = @(ConvertFrom-AdbOutput (cmd /c "`"$adbExe`" -s $RxAdbSerial shell `"$nrstCmd 2>&1`""))
if ($nrstOut.Count -gt 0) {
    Write-Host "  [RESET] gpio163 pulse output: $($nrstOut -join ' | ')" -ForegroundColor Yellow
}
# Bench kit (2026-10-10): /tmp is tmpfs (wiped by every reboot/flash, aged at
# 5 d) and the flash pipeline may have left /tmp/lifetrac_p0c root-owned, so
# make the directory writable before the push and check the cfg landed -- the
# SWD reset below needs it and its push used to fail silently.
cmd /c "`"$adbExe`" -s $TxAdbSerial shell `"echo fio | sudo -S -p '' mkdir -p /tmp/lifetrac_p0c ; echo fio | sudo -S -p '' chmod 0777 /tmp/lifetrac_p0c`"" | Out-Null
cmd /c "`"$adbExe`" -s $TxAdbSerial push `"$(Join-Path $helperDir '08_boot_user_app.cfg')`" /tmp/lifetrac_p0c/08_boot_user_app.cfg" | Out-Null
$cfgCheck = cmd /c "`"$adbExe`" -s $TxAdbSerial shell `"test -s /tmp/lifetrac_p0c/08_boot_user_app.cfg && echo CFG-OK || echo CFG-MISSING`"" 2>$null
$cfgOk = ("$cfgCheck" -match "CFG-OK")
if (-not $cfgOk) {
    Write-Warning "/tmp/lifetrac_p0c/08_boot_user_app.cfg is not on the tractor ($TxAdbSerial); the SWD reset of the TX L072 will fail."
}

# 5. Fire the tractor SWD reset in the BACKGROUND, then launch the TX daemon
# immediately. openocd needs ~7-10 s to reach its NRST pulse; the daemon needs
# ~5 s (docker) + runs 5 VER retries over ~12 s. Backgrounding the reset makes
# the NRST land mid-retry-window instead of after it.
# 2026-10-18: the reset is still fired and forgotten here (no new wait, no
# retry), but openocd now runs in a subshell that first writes this leg's
# OCD_LEG=<UTC stamp> tag and then appends OCD_EXIT=<rc> to
# /tmp/lifetrac_p0c/ocd_reset.log (`trap : HUP` keeps the subshell alive for
# that line when adb hangs up). openocd is started the same way (nohup, same
# cfgs) but one fork later, inside the subshell, and the caught trap does not
# pass to that child: a hangup that lands before nohup ignores HUP kills the
# job, which the log shows as OCD_EXIT=129 with no openocd banner. The log is
# read back at leg end and judged into TRACTOR_L072_RESET landed/failed/unknown;
# without this leg's tag it is an earlier leg's log (the spawn never ran).
# Why: on 2026-10-18 the reset did not land on leg V, the first leg after a
# flash reboot (RS-12.15 RESULTS Observation 3; also O, Q, R, S, T, U of
# 09-14/15, while it landed on P and Q2: RS_12_15_clock_authority_2026-09-14/
# ERRATA.md section 1), and
# the archives cannot say why -- the only witness, this log, stayed on the
# tractor's tmpfs and the next leg's reset overwrote it.
Write-Host "[RESET] TX L072 via OpenOCD SWD (background, BOOT0 low + NRST)..."
# [A-Z0-9=] only: it passes cmd, adb and the single-quoted sh -c unchanged.
$ocdTag = "OCD_LEG=" + (Get-Date).ToUniversalTime().ToString("yyyyMMddTHHmmssZ")
$swdReset = "echo fio | sudo -S -p '' sh -c 'mkdir -p /tmp/lifetrac_p0c; pkill -9 openocd 2>/dev/null; sleep 0.5; for g in 8 10 15; do [ -d /sys/class/gpio/gpio`$g ] || echo `$g > /sys/class/gpio/export 2>/dev/null; done; echo out > /sys/class/gpio/gpio10/direction 2>/dev/null; echo 1 > /sys/class/gpio/gpio10/value 2>/dev/null; sleep 1; for g in 8 10 15; do echo `$g > /sys/class/gpio/unexport 2>/dev/null; done; ( trap : HUP; echo $ocdTag; nohup openocd -f /usr/arduino/extra/openocd_script-imx_gpio.cfg -f /tmp/lifetrac_p0c/08_boot_user_app.cfg; echo OCD_EXIT=`$? ) >/tmp/lifetrac_p0c/ocd_reset.log 2>&1 & echo swd_reset_spawned'"
# Kept for the leg-end verdict (it used to go straight to the console).
$swdOut = @(ConvertFrom-AdbOutput (cmd /c "`"$adbExe`" -s $TxAdbSerial shell `"$swdReset`""))
if ($swdOut.Count -gt 0) { Write-Host ($swdOut -join "`n") }
if ($swdOut -notcontains "swd_reset_spawned") {
    Write-Warning "No swd_reset_spawned from the tractor ($TxAdbSerial): the SWD reset of the TX L072 may not have been fired; its counters may carry over."
}

# RS-5.9: local feed = bench broker on the tractor's own loopback. The
# conf mount pins the listener to 127.0.0.1 (host netns via --network=host).
$txMqtt = $HostIp
# "camera" is a local-broker feed too: camera_service publishes to the tractor
# loopback exactly as the synthetic publisher does. Omitting it here left the TX
# daemon pointed at $HostIp with no broker reachable ("Network unreachable").
if ($TxFeed -eq "local" -or $TxFeed -eq "camera") {
    $txMqtt = "127.0.0.1"
    Write-Host "[MQTT] Starting bench broker on tractor loopback (RS-5.9 local feed)..." -ForegroundColor Yellow
    cmd /c "`"$adbExe`" -s $TxAdbSerial shell `"echo fio | sudo -S -p '' docker rm -f bench_mqtt 2>/dev/null ; echo fio | sudo -S -p '' docker run -d --name bench_mqtt --network=host -v /tmp/lifetrac_strict/bench_mqtt.conf:/mosquitto/config/mosquitto.conf eclipse-mosquitto:2`"" | Out-Null
    Start-Sleep -Seconds 2
}

Write-Host "[LAUNCH] Starting TX Daemon on Board $TxAdbSerial (mqtt=$txMqtt, depth=$TxPipelineDepth)..." -ForegroundColor Yellow
cmd /c "`"$adbExe`" -s $TxAdbSerial shell `"echo fio | sudo -S -p '' docker rm -f tx_smoke 2>/dev/null ; echo fio | sudo -S -p '' docker run -d --name tx_smoke --network=host --entrypoint python3 --device=/dev/ttymxc3 -v /tmp/lifetrac_strict:/work -w /work -e PYTHONPATH=/work:/work/paho -e LIFETRAC_MQTT_HOST=$txMqtt -e LIFETRAC_SKIP_RESET_REQ=1 $profEnv -e LIFETRAC_TX_PIPELINE=$TxPipeline -e LIFETRAC_TX_PIPELINE_DEPTH=$TxPipelineDepth -e LIFETRAC_TX_BATCH=$TxBatch -e LIFETRAC_TX_PREPARE_AHEAD=$TxPrepareAhead -e LIFETRAC_TRAIN_GAP_MS=$TrainGapMs -e LIFETRAC_PARITY_GROUP=$ParityGroup -e LIFETRAC_ACK_COPIES=$AckCopies -e LIFETRAC_PROBE_ECHO=$ProbeEcho -e LIFETRAC_RXCONT_ARM=$RxcontArm -e LIFETRAC_TX_POWER_DBM=$TxPowerDbm hub.foundries.io/arduino/arduino-ootb-python-devel:738bc44 -u /work/image_tx_daemon.py --log-level INFO`""

# 6. Launch RX daemon.
# RX publishes to the BASE BOARD's own mosquitto (127.0.0.1 via
# --network=host) rather than the Windows host broker: the base→host
# Wi-Fi leg drops ARP within a minute (see [NET] warm-up), and
# production topology has the base web_ui subscribed to the base
# broker anyway — tile_delta + link_stats land where the web UI reads.
# RS-0.13b/RS-0.12 (2026-07-27): aligned-pump A/B + reactive-fire probe env.
# PR #121 review (2026-09-14): PowerShell names are case-INsensitive, so the
# assembled docker args must not reuse the -RxExtraEnv parameter name in any
# letter case: a same-name local overwrote the parameter (caller overrides
# dropped, defaults appended to themselves; leg J params.txt shows the doubled
# defaults). Never flown with an override, so no leg was affected.
$rxEnvArgs = "-e LIFETRAC_ALIGNED_PUMP=$AlignedPump -e LIFETRAC_REACTIVE_FIRE=$ReactiveFire -e LIFETRAC_IDLE_DRAIN_QUIET_S=$IdleDrainQuietS -e LIFETRAC_CMD_STREAM_MIN_GAP_S=$CmdStreamMinGapS"
if ($RxExtraEnv -ne "") { $rxEnvArgs = "$rxEnvArgs $RxExtraEnv" }
if ($ProbePhaseSweepMs -ne "") { $rxEnvArgs = "$rxEnvArgs -e LIFETRAC_PROBE_PHASE_SWEEP_MS=$ProbePhaseSweepMs" }
if ($ProbeSizesB -ne "") { $rxEnvArgs = "$rxEnvArgs -e LIFETRAC_PROBE_SIZES_B=$ProbeSizesB" }
Write-Host "[LAUNCH] Starting RX Daemon on Board $RxAdbSerial..." -ForegroundColor Yellow
cmd /c "`"$adbExe`" -s $RxAdbSerial shell `"echo fio | sudo -S -p '' docker rm -f rx_smoke 2>/dev/null ; echo fio | sudo -S -p '' docker run -d --name rx_smoke --network=host --device=/dev/ttymxc3 -v /tmp/lifetrac_strict:/work -w /work -e PYTHONPATH=/work:/work/paho -e LIFETRAC_MQTT_HOST=127.0.0.1 -e LIFETRAC_CTRL_MQTT_HOST=$HostIp -e LIFETRAC_SKIP_RESET_REQ=1 -e LIFETRAC_KF_REQUEST_DISABLE=$KfRequestDisable $rxEnvArgs $profEnv lifetrac-v25:latest python3 -u /work/image_rx_daemon.py --log-level INFO`""

# 6a. -InjectCmd (2026-10-18): arm the injector now that this leg's rx_smoke
# exists. first_frame: the watcher runs on the base and pins this rx_smoke's
# container id, so no adb polling is needed during the leg. fault: the base
# only gets a fresh log here; the monitor loop polls the tractor and starts the
# injector when the FAULT line appears. All adb calls stay in this thread,
# one after another (RS-5.4: concurrent host-side adb has wedged adb).
$injDeadlineS = $DurationS + 60
$injPc = New-Object System.Collections.Generic.List[string]   # PC-side record -> inject_trigger.txt
$injWatchCmd = "echo fio | sudo -S -p '' sh /tmp/lifetrac_strict/inject_watch.sh"
if ($injectSpec) {
    $injPc.Add("$((Get-Date).ToUniversalTime().ToString('HH:mm:ss')) PC armed on=$($injectSpec.Label) cmd=$($injectSpec.Cmd) base=$RxAdbSerial")
    if ($injectSpec.On -eq "first_frame") {
        $armOut = cmd /c "`"$adbExe`" -s $RxAdbSerial shell `"$injWatchCmd start first_frame $injDeadlineS $($injectSpec.Cmd)`""
        $armPid = Get-InjectWatchPid $armOut
        if ($armPid) {
            Write-Host "[INJECT] armed on the base: $($injectSpec.Cmd) at rx_smoke's first published frame (watcher pid $armPid)" -ForegroundColor Yellow
        } else {
            Write-Warning "[INJECT] the base watcher did not report a pid; the injector may not start. Output: $(@(ConvertFrom-AdbOutput $armOut) -join ' | ')"
        }
        $injPc.Add("$((Get-Date).ToUniversalTime().ToString('HH:mm:ss')) PC base watcher pid=$armPid")
    } else {
        cmd /c "`"$adbExe`" -s $RxAdbSerial shell `"$injWatchCmd reset $($injectSpec.Label)`"" | Out-Null
        Write-Host "[INJECT] armed: $($injectSpec.Cmd) on the base at the first tractor FAULT matching $($injectSpec.Label) (polled every ~2 s)" -ForegroundColor Yellow
    }
}

Start-Sleep -Seconds 4

# 7. Start Synthetic Frame Publisher (RS-5.9: on the tractor in local mode)
Write-Host "[STREAM] Starting frame source (feed=$TxFeed, fps=$SynthFps, budget=$SynthBudgetB B)..." -ForegroundColor Yellow
$pubProc = $null
if ($TxFeed -eq "camera") {
    # RS-3.3 REAL-CAMERA path. The synthetic feed publishes PRE-BUILT tile-delta
    # frames straight to image_tx_daemon, so camera_service -- and with it the
    # whole encode-to-fit budget packer, carry fix, age-escalation and liveness
    # valve -- is bypassed and has NEVER run on air.
    #
    # Runs in lifetrac-tractor-x8:latest because that image carries ffmpeg
    # (Dockerfile:8); the host has none, which is why camera_service otherwise
    # logs "v4l2/ffmpeg unavailable ... falling back to synthetic" and silently
    # tests nothing. --device is required or the container cannot see the UVC
    # node (a "C2" 16d0:0ed4 UVC 1.00 device on /dev/video1; /dev/video0 is the
    # separate built-in mx6s-csi MIPI bridge).
    #
    # LIFETRAC_USE_LORA_BRIDGE=1 is THE flag for this path and is REQUIRED.
    # It is not merely a label -- camera_service branches on it three ways:
    #   1. skips the M7 IpcWriter (camera_service.py:1354), which otherwise
    #      imports x8_image_pipeline.ipc_to_h747 AND opens the M7 IPC UART
    #      (LIFETRAC_M7_UART, default /dev/ttymxc1) -- a transport nothing is
    #      listening to on this path. (An earlier revision of this comment
    #      claimed it opened /dev/ttymxc3; wrong -- ttymxc3 is the L072 radio
    #      UART, owned by image_tx_daemon, and IpcWriter never touches it.)
    #   2. creates the MQTT client, without which `client is None` and the
    #      publish at :1563 is silently skipped -- frames are captured and
    #      thrown away, with no log line at all;
    #   3. turns deshake OFF (_default_deshake, :240). That default matters:
    #      the i.MX8 cannot produce a single frame through
    #      "deshake=open2=1:search=16" in 30 s (measured: 0 bytes, vs 589824 B
    #      = 2 frames exactly without it).
    # Omitting it makes camera_service look broken in three unrelated-looking
    # ways at once. It is not broken; the flag declares which transport owns
    # the frames.
    $camEnvArgs = ""
    if ($CamExtraEnv -ne "") { $camEnvArgs = $CamExtraEnv }
    $camRun = "echo fio | sudo -S -p '' docker rm -f camera_svc 2>/dev/null ; echo fio | sudo -S -p '' docker run -d --name camera_svc --network=host --entrypoint python3 --device=$CameraDevice -v /tmp/lifetrac_strict:/work -w /work -e PYTHONPATH=/work:/work/paho -e LIFETRAC_MQTT_HOST=127.0.0.1 -e LIFETRAC_CAMERA_SOURCE=v4l2 -e LIFETRAC_CAMERA_DEVICE=$CameraDevice -e LIFETRAC_CAMERA_FPS=$SynthFps -e LIFETRAC_USE_LORA_BRIDGE=1 $profEnv $camEnvArgs lifetrac-tractor-x8:latest -u /work/camera_service.py"
    cmd /c "`"$adbExe`" -s $TxAdbSerial shell `"$camRun`"" | Out-Null
    Start-Sleep -Seconds 4
    Write-Host "  [CAMERA] camera_service launched on $CameraDevice - encode-to-fit IS in the loop" -ForegroundColor Green
    cmd /c "`"$adbExe`" -s $TxAdbSerial shell `"echo fio | sudo -S -p '' docker logs camera_svc 2>&1 | tail -6`""
} elseif ($TxFeed -eq "local") {
    # Runs in the lifetrac-tractor-x8 image because camera_service's
    # synthetic path needs its numpy/cv2 deps (the ootb python image
    # lacks them). Publishes to the tractor-loopback bench broker.
    $synthDur = $DurationS + 10
    # LIFETRAC_SYNTH_PREBUILD=32: replay a prebuilt frame bank so the paced
    # loop is CPU-free — the i.MX can only *build* ~3.3 fps, which silently
    # de-saturates the measurement (first local-feed run, 2026-07-25).
    cmd /c "`"$adbExe`" -s $TxAdbSerial shell `"echo fio | sudo -S -p '' docker rm -f synth_pub 2>/dev/null ; echo fio | sudo -S -p '' docker run -d --name synth_pub --network=host --entrypoint python3 -v /tmp/lifetrac_strict:/work -w /work -e PYTHONPATH=/work:/work/paho -e LIFETRAC_MQTT_HOST=127.0.0.1 -e LIFETRAC_SYNTH_FPS=$SynthFps -e LIFETRAC_SYNTH_DURATION_S=$synthDur -e LIFETRAC_SYNTH_BYTE_BUDGET=$SynthBudgetB -e LIFETRAC_SYNTH_PREBUILD=32 lifetrac-tractor-x8:latest -u /work/publish_synthetic_frames.py`"" | Out-Null
} else {
    $env:LIFETRAC_MQTT_HOST = $HostIp
    $env:LIFETRAC_SYNTH_FPS = "$SynthFps"
    $env:LIFETRAC_SYNTH_DURATION_S = "$($DurationS + 10)"
    $env:LIFETRAC_SYNTH_BYTE_BUDGET = "$SynthBudgetB"
    $pubScript = Join-Path $repoRoot "publish_synthetic_frames.py"
    $pubProc = Start-Process -FilePath $PythonExe -ArgumentList (@($pyPrefixArgs) + @("`"$pubScript`"")) -PassThru -NoNewWindow -WorkingDirectory $repoRoot
}

Write-Host "=== MONITORING RADIO TRANSMISSIONS FOR $DurationS SECONDS ===" -ForegroundColor Green
$startT = Get-Date
# -InjectCmd state (2026-10-18). Without -InjectCmd the loop is the old one:
# sleep 5 s, print the last 2 lines of each daemon log.
$injFaultPending = [bool]($injectSpec -and $injectSpec.On -eq "fault")
$injStatusPending = [bool]($injectSpec -and $injectSpec.On -eq "first_frame")
$injMarkersShown = 0
$injFaultsSeen = @{}
while (((Get-Date) - $startT).TotalSeconds -lt $DurationS) {
    if ($injFaultPending) {
        # fault: trigger: read the tractor's tx_smoke log for the FAULT line
        # every ~2 s inside the usual 5 s console tick.
        $tickEnd = (Get-Date).AddSeconds(5)
        while ($injFaultPending -and (Get-Date) -lt $tickEnd) {
            Start-Sleep -Seconds 2
            $codeHex = "{0:X2}" -f $injectSpec.Code
            $faultRaw = cmd /c "`"$adbExe`" -s $TxAdbSerial shell `"echo fio | sudo -S -p '' docker logs tx_smoke 2>&1 | grep -a -i 'code=0x$codeHex'`"" 2>$null
            foreach ($fl in @(ConvertFrom-AdbOutput $faultRaw)) {
                if (-not $injFaultsSeen.ContainsKey($fl)) {
                    $injFaultsSeen[$fl] = $true
                    $injPc.Add("$((Get-Date).ToUniversalTime().ToString('HH:mm:ss')) PC tractor FAULT: $fl")
                    Write-Host "  [INJECT] tractor FAULT: $fl" -ForegroundColor Yellow
                }
                if (Test-InjectFaultLine -Line $fl -Spec $injectSpec) {
                    $null = $fl -match 'code=0x([0-9A-Fa-f]{2}) sub=0x([0-9A-Fa-f]{2})'
                    $reason = "fault:code=0x$($Matches[1].ToUpper()),sub=0x$($Matches[2].ToUpper())"
                    $startOut = cmd /c "`"$adbExe`" -s $RxAdbSerial shell `"$injWatchCmd start now:$reason $injDeadlineS $($injectSpec.Cmd)`""
                    $trigPid = Get-InjectWatchPid $startOut
                    $tsec = [int]((Get-Date) - $startT).TotalSeconds
                    $injPc.Add("$((Get-Date).ToUniversalTime().ToString('HH:mm:ss')) PC TRIGGER $($injectSpec.Label) at $tsec s after MONITORING: started $($injectSpec.Cmd) on the base (watcher pid $trigPid)")
                    Write-Host "  [INJECT] TRIGGER $($injectSpec.Label) ($tsec s after MONITORING): starting $($injectSpec.Cmd) on the base (watcher pid $trigPid)" -ForegroundColor Yellow
                    $injFaultPending = $false
                    $injStatusPending = $true
                    break
                }
            }
        }
        $left = ($tickEnd - (Get-Date)).TotalMilliseconds
        if ($left -gt 0) { Start-Sleep -Milliseconds ([int]$left) }
    } else {
        Start-Sleep -Seconds 5
    }
    $txStats = cmd /c "`"$adbExe`" -s $TxAdbSerial shell `"echo fio | sudo -S -p '' docker logs --tail 2 tx_smoke 2>&1`""
    $rxStats = cmd /c "`"$adbExe`" -s $RxAdbSerial shell `"echo fio | sudo -S -p '' docker logs --tail 2 rx_smoke 2>&1`""
    Write-Host "--- TX Log ---" -ForegroundColor Cyan
    Write-Host $txStats
    Write-Host "--- RX Log ---" -ForegroundColor Magenta
    Write-Host $rxStats
    if ($injStatusPending) {
        # Show the base watcher's new marker lines until the injector has
        # started or given up (then nothing more is polled until leg end).
        $marks = @(ConvertFrom-AdbOutput (cmd /c "`"$adbExe`" -s $RxAdbSerial shell `"sh /tmp/lifetrac_strict/inject_watch.sh status`"" 2>$null) | Where-Object { $_ -like 'INJECT_*' -and $_ -ne 'INJECT_STATUS_DONE' })
        for ($k = $injMarkersShown; $k -lt $marks.Count; $k++) {
            Write-Host "  [INJECT] base: $($marks[$k])" -ForegroundColor Yellow
            $injPc.Add("$((Get-Date).ToUniversalTime().ToString('HH:mm:ss')) PC saw on the base: $($marks[$k])")
        }
        if ($marks.Count -gt $injMarkersShown) { $injMarkersShown = $marks.Count }
        if (@($marks | Where-Object { $_ -match '^INJECT_(START|NOT_STARTED) ' }).Count -gt 0) { $injStatusPending = $false }
    }
}

# 8. Clean up publisher & containers
if ($pubProc -and -not $pubProc.HasExited) {
    try { $pubProc.Kill() } catch {}
}

# 8a. -InjectCmd (2026-10-18): the leg is over, so stop the injector (or the
# watcher still waiting for its trigger) and collect the base log.
$injLog = @()
$injSum = $null
if ($injectSpec) {
    $injLog = @(ConvertFrom-AdbOutput (cmd /c "`"$adbExe`" -s $RxAdbSerial shell `"$injWatchCmd stop`""))
    $injSum = Get-InjectSummary $injLog
    if ($injectSpec.On -eq "fault" -and $injFaultPending) {
        $injPc.Add("$((Get-Date).ToUniversalTime().ToString('HH:mm:ss')) PC leg ended; no tractor FAULT matched $($injectSpec.Label) ($($injFaultsSeen.Count) FAULT line(s) with code 0x$('{0:X2}' -f $injectSpec.Code) seen)")
    }
}


Write-Host "`n=== FINAL LOG SUMMARY ===" -ForegroundColor Cyan
$txFinal = cmd /c "`"$adbExe`" -s $TxAdbSerial shell `"echo fio | sudo -S -p '' docker logs tx_smoke 2>&1`""
$rxFinal = cmd /c "`"$adbExe`" -s $RxAdbSerial shell `"echo fio | sudo -S -p '' docker logs rx_smoke 2>&1`""
$synthFinal = ""
$camFinal = $null
if ($TxFeed -eq "camera") {
    # RS-13.1: keep camera_service's log (encoder `vector_stats` lines, mode
    # acks) in the evidence bundle and stop the container like tx_smoke, so
    # the camera is free for the next leg's launch (which rm -f's it anyway).
    $camFinal = cmd /c "`"$adbExe`" -s $TxAdbSerial shell `"echo fio | sudo -S -p '' docker logs camera_svc 2>&1`""
    cmd /c "`"$adbExe`" -s $TxAdbSerial shell `"echo fio | sudo -S -p '' docker stop -t 1 camera_svc 2>&1`"" | Out-Null
    Write-Host "--- CAMERA SERVICE LOGS (last 8) ---" -ForegroundColor Yellow
    # adb's \r\r\n puts an empty element after every line (ConvertFrom-AdbOutput),
    # so a plain -Last 8 showed only 4 lines (leg V, 2026-10-18).
    Write-Host ((@(ConvertFrom-AdbOutput $camFinal) | Select-Object -Last 8) -join "`n")
}
if ($TxFeed -eq "local") {
    # Capture the publisher's OFFERED line for the evidence bundle, then
    # remove it (it self-exits after DurationS+10). bench_mqtt stays up
    # for reuse across runs.
    $synthFinal = cmd /c "`"$adbExe`" -s $TxAdbSerial shell `"echo fio | sudo -S -p '' docker logs synth_pub 2>&1`""
    cmd /c "`"$adbExe`" -s $TxAdbSerial shell `"echo fio | sudo -S -p '' docker rm -f synth_pub 2>/dev/null`"" | Out-Null
    Write-Host "--- SYNTH PUBLISHER LOGS ---" -ForegroundColor Yellow
    Write-Host $synthFinal
}

cmd /c "`"$adbExe`" -s $TxAdbSerial shell `"echo fio | sudo -S -p '' docker stop -t 1 tx_smoke 2>&1`"" | Out-Null
cmd /c "`"$adbExe`" -s $RxAdbSerial shell `"echo fio | sudo -S -p '' docker stop -t 1 rx_smoke 2>&1`"" | Out-Null

# 8b. Tractor L072 reset check (2026-10-18): read back the openocd log of the
# leg-start SWD reset (read-only; nothing is reset or re-run here). After the
# daemon stops, so the extra adb round trip does not lengthen the leg's TX.
$ocdRaw = cmd /c "`"$adbExe`" -s $TxAdbSerial shell `"cat /tmp/lifetrac_p0c/ocd_reset.log 2>&1; echo __OCD_LOG_RC=`$?; if pgrep -x openocd >/dev/null 2>&1 || pidof openocd >/dev/null 2>&1; then echo __OCD_RUNNING=1; else echo __OCD_RUNNING=0; fi`"" 2>$null
$ocdLines = @(ConvertFrom-AdbOutput $ocdRaw)
$resetV = Get-TractorResetVerdict -Output $ocdLines -CfgOk $cfgOk -Tag $ocdTag -SpawnOutput $swdOut

Write-Host "--- TX DAEMON FINAL LOGS ---" -ForegroundColor Yellow
Write-Host $txFinal
Write-Host "--- RX DAEMON FINAL LOGS ---" -ForegroundColor Yellow
Write-Host $rxFinal

# 8c. Leg checks (2026-10-18), printed before the [EVIDENCE] line that
# legs/leg_post.sh looks for, and repeated in params.txt.
$baseResetLine = "BASE_L072_RESET unverified (gpio163 pulse sent; nothing checks that the base L072 rebooted, and it has not since 2026-08: score base brackets as post - pre)"
$nrstNote = "none"
if ($nrstOut.Count -gt 0) { $nrstNote = $nrstOut -join ' | ' }
$tractorResetLine = "TRACTOR_L072_RESET $($resetV.Verdict) ($($resetV.Reason))"
Write-Host "`n=== LEG CHECKS ===" -ForegroundColor Cyan
$resetColor = "Yellow"
if ($resetV.Verdict -eq "landed") { $resetColor = "Green" } elseif ($resetV.Verdict -eq "failed") { $resetColor = "Red" }
Write-Host $tractorResetLine -ForegroundColor $resetColor
if ($resetV.Verdict -ne "landed") {
    Write-Host "  The tractor's L072 counters may carry over from before this leg: score its brackets with deltas.py (reset-aware)." -ForegroundColor Yellow
}
Write-Host $baseResetLine -ForegroundColor Yellow
Write-Host "  gpio163 pulse output: $nrstNote"
$injectLine = ""
if ($injectSpec) {
    $injTrigTxt = "-"; if ($injSum.Trigger -ne "") { $injTrigTxt = $injSum.Trigger }
    $injStartTxt = "-"; if ($injSum.Start -ne "") { $injStartTxt = $injSum.Start }
    $injExitTxt = "-"; if ($injSum.Exit -ne "") { $injExitTxt = $injSum.Exit }
    $injectLine = "INJECT $($injSum.Status) (on=$($injectSpec.Label) cmd=$($injectSpec.Cmd) started=$injStartTxt exit=$injExitTxt; $($injSum.Detail)) trigger: $injTrigTxt"
    $injColor = "Yellow"
    if ($injSum.Status -eq "finished") { $injColor = "Green" } elseif ($injSum.Status -eq "not_started" -or $injSum.Status -eq "failed") { $injColor = "Red" }
    Write-Host $injectLine -ForegroundColor $injColor
}

# 9. Evidence bundle (per the CODE REVIEWS evidence discipline: no
#    verification claim without a SHA + raw transcript).
if ($Archive) {
    $sha = (git -C $repoRoot rev-parse --short HEAD 2>$null)
    $stamp = Get-Date -Format "yyyyMMdd_HHmmss"
    $dir = Join-Path $repoRoot "LifeTrac-v25\DESIGN-CONTROLLER\bench-evidence\radio_monitor_${stamp}_${sha}"
    New-Item -ItemType Directory -Force -Path $dir | Out-Null
    $txFinal | Out-File -FilePath (Join-Path $dir "tx_daemon.log") -Encoding utf8
    $rxFinal | Out-File -FilePath (Join-Path $dir "rx_daemon.log") -Encoding utf8
    if ($synthFinal) {
        $synthFinal | Out-File -FilePath (Join-Path $dir "synth_pub.log") -Encoding utf8
    }
    if ($camFinal) {
        $camFinal | Out-File -FilePath (Join-Path $dir "camera_service.log") -Encoding utf8
    }
    # 2026-10-18: the openocd log of the tractor's leg-start reset (the only
    # witness of whether it landed), and the -InjectCmd records: the base
    # watcher log (trigger, injector output, exit) and the PC-side timeline.
    $ocdLines | Out-File -FilePath (Join-Path $dir "tractor_ocd_reset.log") -Encoding utf8
    if ($injectSpec) {
        $injLog | Out-File -FilePath (Join-Path $dir "inject.log") -Encoding utf8
        @($injPc) + @($injectLine) | Out-File -FilePath (Join-Path $dir "inject_trigger.txt") -Encoding utf8
    }
    $injStatus = ""; $injTrigger = ""; $injOnLabel = ""
    if ($injSum) { $injStatus = $injSum.Status; $injTrigger = $injSum.Trigger }
    if ($injectSpec) { $injOnLabel = $injectSpec.Label }
    @(
        "git_sha=$sha",
        "duration_s=$DurationS",
        "synth_fps=$SynthFps",
        "synth_budget_b=$SynthBudgetB",
        "tx_pipeline=$TxPipeline",
        "tx_pipeline_depth=$TxPipelineDepth",
        "tx_batch=$TxBatch",
        "tx_prepare_ahead=$TxPrepareAhead",
        "train_gap_ms=$TrainGapMs",
        "kf_request_disable=$KfRequestDisable",
        "no_park_last=$NoParkLast",
        "no_park_last_min_frags=$NoParkLastMinFrags",
        "idle_drain_quiet_s=$IdleDrainQuietS",
        "cmd_stream_min_gap_s=$CmdStreamMinGapS",
        "rx_extra_env=$RxExtraEnv",
        "rx_env_args=$rxEnvArgs",
        "cam_extra_env=$CamExtraEnv",
        "parity_group=$ParityGroup",
        "aligned_pump=$AlignedPump",
        "reactive_fire=$ReactiveFire",
        "probe_phase_sweep_ms=$ProbePhaseSweepMs",
        "probe_sizes_b=$ProbeSizesB",
        "ack_copies=$AckCopies",
        "probe_echo=$ProbeEcho",
        "fhss_farm_id=$FhssFarmId",
        "fhss_link_id=$FhssLinkId",
        "airtime_budget_us=$AirtimeBudgetUs",
        "pacing_mode=$PacingMode",
        "pacing_headroom=$PacingHeadroom",
        "rxcont_arm=$RxcontArm",
        "tx_power_dbm=$TxPowerDbm",
        "tx_feed=$TxFeed",
        "reg_profile=$RegProfile",
        "force_frf_hz=$ForceFrfHz",
        "allow_default_carrier=$([bool]$AllowDefaultCarrier)",
        "tx_serial=$TxAdbSerial",
        "rx_serial=$RxAdbSerial",
        "host_ip=$HostIp",
        "inject_cmd=$InjectCmd",
        "inject_on=$injOnLabel",
        "inject_status=$injStatus",
        "inject_trigger=$injTrigger",
        "tractor_l072_reset=$($resetV.Verdict)",
        "tractor_l072_reset_reason=$($resetV.Reason)",
        "tractor_ocd_tag=$ocdTag",
        "base_l072_reset=unverified",
        "base_gpio163_output=$nrstNote",
        "timestamp=$stamp"
    ) | Out-File -FilePath (Join-Path $dir "params.txt") -Encoding utf8
    Write-Host "[EVIDENCE] archived to $dir" -ForegroundColor Green
}

Write-Host "=== LIVE LORA RADIO MONITOR COMPLETE ===" -ForegroundColor Green
