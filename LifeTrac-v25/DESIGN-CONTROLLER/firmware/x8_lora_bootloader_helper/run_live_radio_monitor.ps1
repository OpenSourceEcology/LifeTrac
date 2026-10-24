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

    Bench kit (2026-10-19): at leg end the frame source stops first, then
    tx_smoke (after -StopDrainS s), then rx_smoke, and only then are the
    daemon logs read, so both logs cover the same window as the post
    brackets. Since 2026-10-22 that is checked, not assumed: params.txt says
    log_window=after_stop only when both docker stops worked (container name
    printed, exit 0) and both daemon logs hold their SIGTERM shutdown line;
    otherwise log_window=after_stop_unverified with the reason in
    log_window_detail=, and LEG CHECKS prints the same verdict as LOG_WINDOW.
    The frame source's stop is recorded too (feed_stop=), as a detail only.

    Also 2026-10-19: the tractor reset runs from the pushed
    bench_tools/ocd_reset.sh, which ignores SIGHUP before it forks
    (the 10-19 A2c2 miss, OCD_EXIT=129). Every adb push is checked (output,
    exit status, then md5 against the PC copy) and a failure stops the leg
    before any daemon starts. params.txt records each board's clock offset
    from the PC (tractor_clock_offset_s, base_clock_offset_s; read only).
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
    # Bench kit (2026-10-19): seconds between stopping the frame source and
    # stopping tx_smoke at leg end, so the frames already queued go on air and
    # into the TX log. 2 s covers the DTS / BENCH single-fragment legs; give a
    # saturated multi-fragment FHSS leg more.
    [ValidateRange(0, 60)]
    [double]$StopDrainS   = 2,
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
    # from the L072 itself; the counter brackets (bench_tools/bracket_deltas.py)
    # stay the final word.
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
    # ignored HUP (the old inline subshell's `trap :` was reset in its
    # children; 2026-10-19 A2c2). bench_tools/ocd_reset.sh (bench kit
    # 2026-10-19) ignores HUP before it forks, which closes that window.
    # openocd installs its own SIGHUP handler once it starts (server_preinit,
    # after its banner), so a 129 WITH the banner is a hangup that reached
    # openocd later; only a 129 WITHOUT the banner from ocd_reset.sh would
    # point to a different cause. Without the openocd banner, openocd never
    # started.
    if ($null -ne $exit -and $exit -ge 128) {
        $sig = $exit - 128
        $what = "killed by signal $sig"
        if ($sig -eq 1) {
            if ($banner) { $what = "killed by SIGHUP after openocd started (openocd catches SIGHUP itself once running)" }
            else { $what = "killed by SIGHUP (likely the adb/sudo session hangup reaching nohup before nohup ignored HUP)" }
        }
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

# --- Helpers (bench kit, 2026-10-19) -----------------------------------------
# Checked adb pushes and the board clock read. Test-AdbPushOutput,
# Get-PushedFileList, Test-PushedMd5 and Get-ClockOffset are pure; the others
# call adb (the offline tests give them a stub adb).

function Test-AdbPushOutput {
    # Judges one `adb push` from its output (stdout and stderr merged) and its
    # exit status. Returns @{ Ok; Files; Reason }. A push that worked prints
    # "<src>: N file(s) pushed, M skipped. <rate> (<bytes> bytes in <t>s)" (on
    # stderr) and exits 0; a failure prints "adb: error: ..." or "error: device
    # '...' not found" and exits non-zero. Ok needs exit 0 AND N >= 1.
    param([object]$Output, [int]$ExitCode)
    $lines = @(ConvertFrom-AdbOutput $Output)
    $files = -1; $errLine = $null
    foreach ($l in $lines) {
        if ($l -match '\b(\d+) files? pushed\b') { $files = [int]$Matches[1] }
        elseif ($null -eq $errLine -and $l -match '(?i)\berror\b|\bfailed\b|not found|no such file|denied|offline|unauthorized|cannot') { $errLine = $l }
    }
    $text = "no output"
    if ($null -ne $errLine) { $text = $errLine } elseif ($lines.Count -gt 0) { $text = $lines[-1] }
    if ($ExitCode -ne 0) { return @{ Ok = $false; Files = $files; Reason = "adb exit ${ExitCode}: $text" } }
    if ($files -lt 0) { return @{ Ok = $false; Files = $files; Reason = "adb printed no 'file(s) pushed' line: $text" } }
    if ($files -eq 0) { return @{ Ok = $false; Files = 0; Reason = "adb pushed 0 files: $text" } }
    return @{ Ok = $true; Files = $files; Reason = "$files file(s) pushed" }
}

function Get-PushedFileList {
    # The files one `adb push <Local> <Remote>` writes, as @{ Local; Remote }.
    # A directory pushed to an existing remote directory (Remote ends in /)
    # lands in Remote/<its name>/, as every directory push here does;
    # __pycache__ content is pushed too but not listed (not code under test,
    # and a container may rewrite it). A file pushed to a path ending in /
    # keeps its name.
    param([string]$Local, [string]$Remote)
    if (Test-Path -LiteralPath $Local -PathType Container) {
        $root = (Resolve-Path -LiteralPath $Local).Path.TrimEnd('\', '/')
        $dest = $Remote.TrimEnd('/') + "/" + (Split-Path -Leaf $root)
        foreach ($f in @(Get-ChildItem -LiteralPath $root -Recurse -File -Force | Sort-Object FullName)) {
            $rel = $f.FullName.Substring($root.Length + 1).Replace('\', '/')
            if ($rel -match '(^|/)__pycache__/' -or $rel -like '*.pyc') { continue }
            @{ Local = $f.FullName; Remote = "$dest/$rel" }
        }
    } else {
        $dest = $Remote
        if ($Remote.EndsWith('/')) { $dest = $Remote + (Split-Path -Leaf $Local) }
        @{ Local = $Local; Remote = $dest }
    }
}

function Get-CleanPushSource {
    # The path to push for -Local. A directory goes up from a temp copy under
    # -StageRoot without __pycache__ / *.pyc: the PC's bytecode is of no use on
    # the board, and the board's own __pycache__ directories are root-owned
    # (the containers write them), so pushing a PC __pycache__ into one fails,
    # which the checked push would turn into an aborted leg (the old unchecked
    # push could stop there silently and skip the files after it). A file is
    # pushed as it is.
    param([string]$Local, [string]$StageRoot)
    if (-not (Test-Path -LiteralPath $Local -PathType Container)) { return $Local }
    $root = (Resolve-Path -LiteralPath $Local).Path.TrimEnd('\', '/')
    $dest = Join-Path $StageRoot (Split-Path -Leaf $root)
    if (Test-Path -LiteralPath $dest) { Remove-Item -LiteralPath $dest -Recurse -Force }
    New-Item -ItemType Directory -Force -Path $dest | Out-Null
    foreach ($f in @(Get-ChildItem -LiteralPath $root -Recurse -File -Force)) {
        $rel = $f.FullName.Substring($root.Length + 1)
        if ($rel -match '(^|\\)__pycache__\\' -or $rel -like '*.pyc') { continue }
        $d = Join-Path $dest $rel
        $pd = Split-Path -Parent $d
        if (-not (Test-Path -LiteralPath $pd)) { New-Item -ItemType Directory -Force -Path $pd | Out-Null }
        Copy-Item -LiteralPath $f.FullName -Destination $d -Force
    }
    return $dest
}

function Test-PushedMd5 {
    # Compares a board's `md5sum <remote paths>` output with the PC copies.
    # -Files: @{ Remote; Md5 (lower-case hex) }. Returns @{ Checked; Mismatch;
    # Missing; Rows }: Mismatch / Missing are remote paths, Rows one line per
    # file, "ok|MISMATCH|missing <remote> pc=<md5> board=<md5|->".
    param([object]$Output, [object[]]$Files)
    $board = @{}
    foreach ($l in @(ConvertFrom-AdbOutput $Output)) {
        if ($l -match '^([0-9a-fA-F]{32})\s+\*?(\S.*)$') { $board[$Matches[2]] = $Matches[1].ToLower() }
    }
    $mis = @(); $miss = @(); $rows = @(); $n = 0
    foreach ($f in @($Files)) {
        if ($board.ContainsKey($f.Remote)) {
            $n++
            if ($board[$f.Remote] -eq $f.Md5) { $rows += "ok $($f.Remote) pc=$($f.Md5) board=$($board[$f.Remote])" }
            else { $mis += $f.Remote; $rows += "MISMATCH $($f.Remote) pc=$($f.Md5) board=$($board[$f.Remote])" }
        } else {
            $miss += $f.Remote; $rows += "missing $($f.Remote) pc=$($f.Md5) board=-"
        }
    }
    return @{ Checked = $n; Mismatch = $mis; Missing = $miss; Rows = $rows }
}

function Get-ClockOffset {
    # Board clock minus PC clock from one `date +%s.%N` on the board, between
    # PC UTC epoch reads taken just before and after the adb call. Returns
    # @{ Ok; OffsetS; RttS; ErrS; Text }. The offset is taken at the round
    # trip's midpoint, so ErrS = RttS/2 (+1 s when the board's date has no %N
    # and prints a literal N: whole seconds).
    param([object]$Output, [double]$PcBeforeS, [double]$PcAfterS)
    $inv = [System.Globalization.CultureInfo]::InvariantCulture
    $rtt = $PcAfterS - $PcBeforeS
    $lines = @(ConvertFrom-AdbOutput $Output)
    $boardS = $null; $coarse = $false
    foreach ($l in $lines) {
        if ($l -match '^(\d{9,11})\.(\d{1,9})$') {
            $boardS = [double]::Parse("$($Matches[1]).$($Matches[2])", $inv); break
        } elseif ($l -match '^(\d{9,11})(\.%?N)?$') {
            $boardS = [double]::Parse($Matches[1], $inv); $coarse = $true; break
        }
    }
    if ($null -eq $boardS) {
        $got = "nothing"
        if ($lines.Count -gt 0) { $got = "'" + ($lines -join ' | ') + "'" }
        return @{ Ok = $false; OffsetS = $null; RttS = $rtt; ErrS = $null; Text = "unknown (date +%s.%N printed $got)" }
    }
    $off = $boardS - (($PcBeforeS + $PcAfterS) / 2.0)
    $err = $rtt / 2.0
    if ($coarse) { $err += 1.0 }
    $sign = ""; if ($off -ge 0) { $sign = "+" }
    $text = $sign + $off.ToString("F3", $inv) + " s (" + $sign + ($off / 86400.0).ToString("F2", $inv) + " d) +-" + $err.ToString("F3", $inv) + " s"
    if ($coarse) { $text += " (whole seconds: no %N)" }
    return @{ Ok = $true; OffsetS = $off; RttS = $rtt; ErrS = $err; Text = $text }
}

function Get-PcEpochS {
    # The PC clock as UTC epoch seconds (the clock-offset reference).
    return ([DateTime]::UtcNow.Ticks - 621355968000000000) / 1e7
}

function Read-BoardClock {
    # Reads a board's clock (read only: no clock is ever set) and returns
    # Get-ClockOffset's result. `%s.%N` passes cmd unchanged (cmd would expand
    # only a defined environment variable named "s.").
    param([string]$Adb, [string]$Serial)
    $t0 = Get-PcEpochS
    $o = cmd /c "`"$Adb`" -s $Serial shell `"date +%s.%N 2>&1`"" 2>$null
    $t1 = Get-PcEpochS
    return (Get-ClockOffset -Output $o -PcBeforeS $t0 -PcAfterS $t1)
}

function Invoke-AdbPush {
    # One `adb push`, checked (Test-AdbPushOutput); throws on a failure, which
    # stops the leg before any daemon starts. stderr is merged INSIDE the cmd
    # string: adb writes its "N file(s) pushed" summary on stderr, which
    # PowerShell 5.1 turned into one RemoteException / NativeCommandError block
    # per push in every transcript (25 per leg on 2026-10-19), and the old
    # `| Out-Null` hid real failures. Returns the Test-AdbPushOutput result.
    # The reason is also written with Write-Host first, so the leg transcript
    # (*>&1 | Tee-Object) keeps it: a throw ends that pipeline unrecorded.
    param([string]$Adb, [string]$Serial, [string]$Local, [string]$Remote)
    if (-not (Test-Path -LiteralPath $Local)) {
        $msg = "[DEPLOY] $Local does not exist on this PC (push to ${Serial}:$Remote). No daemon was started."
        Write-Host $msg -ForegroundColor Red
        throw $msg
    }
    $out = cmd /c "`"$Adb`" -s $Serial push `"$Local`" $Remote 2>&1"
    $rc = $LASTEXITCODE
    $v = Test-AdbPushOutput -Output $out -ExitCode $rc
    if (-not $v.Ok) {
        $msg = "[DEPLOY] adb push FAILED: $Local -> ${Serial}:$Remote ($($v.Reason)). No daemon was started."
        Write-Host $msg -ForegroundColor Red
        throw $msg
    }
    return $v
}

function Confirm-AdbPushedFiles {
    # md5sum on the board of every file the pushes wrote, against the PC copy
    # (one adb call per ~1500 characters of paths). Throws on a mismatch (the
    # board holds something other than this tree; no daemon has started yet);
    # a file md5sum did not report is a warning only. Returns @{ Checked;
    # Total; Mismatch; Missing; Rows; Summary }.
    param([string]$Adb, [string]$Serial, [object[]]$Files)
    $list = @(foreach ($f in @($Files)) {
        @{ Local = $f.Local; Remote = $f.Remote; Md5 = (Get-FileHash -Algorithm MD5 -LiteralPath $f.Local).Hash.ToLower() }
    })
    # Only plain path characters go into the remote shell; a file with any
    # other character is left out (and so reported missing).
    $batches = New-Object System.Collections.Generic.List[object]
    $cur = New-Object System.Collections.Generic.List[string]
    $len = 0
    foreach ($f in $list) {
        if ($f.Remote -notmatch '^[A-Za-z0-9_./+-]+$') { continue }
        if ($cur.Count -gt 0 -and ($len + $f.Remote.Length) -gt 1500) {
            $batches.Add($cur.ToArray()); $cur.Clear(); $len = 0
        }
        $cur.Add($f.Remote); $len += $f.Remote.Length + 1
    }
    if ($cur.Count -gt 0) { $batches.Add($cur.ToArray()) }
    $out = New-Object System.Collections.Generic.List[string]
    foreach ($b in $batches) {
        foreach ($l in @(ConvertFrom-AdbOutput (cmd /c "`"$Adb`" -s $Serial shell `"md5sum $($b -join ' ') 2>&1`"" 2>$null))) { $out.Add($l) }
    }
    $r = Test-PushedMd5 -Output $out.ToArray() -Files $list
    if ($r.Mismatch.Count -gt 0) {
        $msg = "[DEPLOY] md5 MISMATCH on $Serial after the push: $($r.Mismatch -join ', '). No daemon was started."
        Write-Host $msg -ForegroundColor Red
        throw $msg
    }
    $summary = "md5 $($r.Checked)/$($list.Count) match"
    if ($r.Missing.Count -gt 0) {
        $summary += " ($($r.Missing.Count) not reported by md5sum)"
        Write-Warning "[DEPLOY] md5sum on $Serial did not report $($r.Missing.Count) pushed file(s): $(@($r.Missing | Select-Object -First 5) -join ', '). The pushes themselves reported success."
    }
    return @{ Checked = $r.Checked; Total = $list.Count; Mismatch = $r.Mismatch; Missing = $r.Missing; Rows = $r.Rows; Summary = $summary }
}

# --- Helpers (bench kit, 2026-10-22) -----------------------------------------
# The leg-end log window, checked. Up to 2026-10-22 every leg-end docker stop
# went to Out-Null and params.txt said log_window=after_stop whatever happened,
# so a stop that failed (the daemon still running while its log was read, the
# pre-10-19 artefact) looked the same as one that worked. Get-DockerStopResult,
# Test-DaemonShutdownLine and Get-LogWindowVerdict are pure; Invoke-DockerStop
# calls adb (the offline tests give it a stub adb).

function Invoke-DockerStop {
    # One leg-end `docker stop -t 1 <Name>` on a board, its output and exit
    # status kept: the board shell appends __STOP_RC=<docker's exit status>
    # (the sudo pipeline's status is docker's), and adb's own stderr is merged
    # INSIDE the cmd string (as in Invoke-AdbPush), so a failed adb call comes
    # back as its error line with no __STOP_RC. Returns the cleaned lines, for
    # Get-DockerStopResult / Get-LogWindowVerdict.
    param([string]$Adb, [string]$Serial, [string]$Name)
    return @(ConvertFrom-AdbOutput (cmd /c "`"$Adb`" -s $Serial shell `"echo fio | sudo -S -p '' docker stop -t 1 $Name 2>&1; echo __STOP_RC=`$?`" 2>&1"))
}

function Get-DockerStopResult {
    # Judges one Invoke-DockerStop. docker prints the container's name and
    # exits 0 when the stop worked, also for a container that had already
    # exited (the daemon's shutdown line tells those apart, see
    # Get-LogWindowVerdict); a failure prints "Error response from daemon: ..."
    # and exits 1. No __STOP_RC line = the adb call failed (device gone, adb
    # error), so nothing says the container stopped. Returns @{ Ok; Rc; Text },
    # Text one line (it goes into params.txt).
    param([object]$Output, [string]$Name)
    $rc = $null
    $rest = New-Object System.Collections.Generic.List[string]
    foreach ($l in @(ConvertFrom-AdbOutput $Output)) {
        if ($l -match '^__STOP_RC=(\d+)$') { $rc = [int]$Matches[1] }
        else { $rest.Add($l) }
    }
    $text = "no output"
    if ($rest.Count -gt 0) { $text = $rest -join ' | ' }
    if ($text.Length -gt 200) { $text = $text.Substring(0, 200) + "..." }
    if ($null -eq $rc) { return @{ Ok = $false; Rc = $null; Text = "no exit status came back (adb failed?): $text" } }
    if ($rc -ne 0) { return @{ Ok = $false; Rc = $rc; Text = "docker stop exit ${rc}: $text" } }
    if (-not $rest.Contains($Name)) { return @{ Ok = $false; Rc = 0; Text = "docker stop exit 0 without the container name: $text" } }
    return @{ Ok = $true; Rc = 0; Text = "stopped" }
}

function Test-DaemonShutdownLine {
    # True when a daemon log (`docker logs` output) holds the line the
    # daemon's SIGTERM handler writes, "<Daemon>: signal received; shutting
    # down" (_handle_signal in image_tx_daemon.py and image_rx_daemon.py,
    # logged as the signal lands, so it is there even when docker stop's 1 s
    # grace runs out and SIGKILL follows).
    param([object]$Log, [string]$Daemon)
    $needle = "${Daemon}: signal received; shutting down"
    foreach ($l in @(ConvertFrom-AdbOutput $Log)) {
        if ($l.Contains($needle)) { return $true }
    }
    return $false
}

function Get-LogWindowVerdict {
    # The window the archived daemon logs cover. after_stop only when both
    # daemon stops worked (Get-DockerStopResult) AND both logs, read after the
    # stops, hold their daemon's SIGTERM shutdown line; anything else is
    # after_stop_unverified with the reasons in Detail (one line, for
    # params.txt log_window_detail=). Color for LEG CHECKS: Red when a stop
    # failed (the daemon may still have been running when its log was read),
    # Yellow when only a shutdown line is missing (the daemon had already
    # exited, a crash, or its log did not come back). The frame-source stop
    # is not part of the verdict. Hint is the LEG CHECKS line printed under
    # an unverified LOG_WINDOW, worded by reason (2026-10-22 review): a stop
    # that failed leaves the old artefact possible (a daemon still ran while
    # its log was read), while a missing shutdown line after a stop that
    # worked is the opposite case, a daemon that died (or a log that did not
    # come back whole) BEFORE the stop, so its log ends early.
    # Returns @{ Value; Verified; Detail; Color; Hint }.
    param([object]$TxStopOutput, [object]$RxStopOutput, [object]$TxLog, [object]$RxLog)
    $why = New-Object System.Collections.Generic.List[string]
    $color = "Green"
    $noShut = $false
    $txStop = Get-DockerStopResult -Output $TxStopOutput -Name "tx_smoke"
    $rxStop = Get-DockerStopResult -Output $RxStopOutput -Name "rx_smoke"
    if (-not $txStop.Ok) { $why.Add("tx_smoke stop: $($txStop.Text)"); $color = "Red" }
    if (-not $rxStop.Ok) { $why.Add("rx_smoke stop: $($rxStop.Text)"); $color = "Red" }
    if (-not (Test-DaemonShutdownLine -Log $TxLog -Daemon "image_tx_daemon")) {
        $why.Add("tx_smoke log has no shutdown line")
        $noShut = $true
        if ($color -eq "Green") { $color = "Yellow" }
    }
    if (-not (Test-DaemonShutdownLine -Log $RxLog -Daemon "image_rx_daemon")) {
        $why.Add("rx_smoke log has no shutdown line")
        $noShut = $true
        if ($color -eq "Green") { $color = "Yellow" }
    }
    if ($why.Count -eq 0) {
        return @{ Value = "after_stop"; Verified = $true; Detail = "both stops ok, both daemon logs hold their SIGTERM shutdown line"; Color = "Green"; Hint = "" }
    }
    if ($color -eq "Red") {
        $hint = "A daemon stop was not confirmed: that daemon may still have run when its log was read, so the logs may not cover the post-stop window and rs12_leg_report.py's 'received > sent' and FIRMWARE DROP figures can carry the old log-window artefact (logs read while a daemon ran)."
        if ($noShut) { $hint += " A log without its shutdown line can also mean that daemon died before the stop (look for a traceback in its final log above)." }
    } else {
        $hint = "Both daemon stops worked, but a daemon log has no SIGTERM shutdown line: that daemon may have died before the stop (look for a traceback in its final log above) or its log did not come back whole, so the log can end before the leg did and rs12_leg_report.py's figures can cover less than the leg."
    }
    return @{ Value = "after_stop_unverified"; Verified = $false; Detail = ($why -join '; '); Color = $color; Hint = $hint }
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
# Bench kit (2026-10-19): every push is checked (Invoke-AdbPush: adb's output
# and exit status) and then every file it wrote is md5-compared with the PC copy
# (Confirm-AdbPushedFiles); a failure throws here, before any daemon starts.
Write-Host "[DEPLOY] Syncing code to both boards..."
$deployMd5 = New-Object System.Collections.Generic.List[string]   # -> deploy_md5.txt
$deployMd5Note = @{}                                                # serial -> summary, for params.txt
# Directories go up from clean temp copies (no __pycache__; Get-CleanPushSource).
$pushStage = Join-Path ([System.IO.Path]::GetTempPath()) "lifetrac_push"
$deploySrc = @(
    (Join-Path $helperDir 'method_g_stage1_probe.py'),
    (Join-Path $helperDir 'method_h_stage2_tx_probe_v2.py'),
    (Join-Path $baseStation 'lora_proto.py'),
    (Join-Path $baseStation 'image_rx_daemon.py'),
    (Join-Path $baseStation 'cmd_timing.py'),
    (Join-Path $tractorX8 'image_tx_daemon.py'),
    (Join-Path $tractorX8 'camera_service.py'),
    (Join-Path $repoRoot 'publish_synthetic_frames.py'),
    (Join-Path $helperDir 'bench_mqtt.conf'),
    (Join-Path $baseStation 'image_pipeline'),
    # 2026-07-30 RS-3.3: the TRACTOR's pipeline package too — image_tx_daemon
    # needs frame_format from the base copy while camera_service needs
    # ipc_to_h747/tile_cache from this one, so the tractor genuinely requires
    # both. (Until RS-5.7 landed 2026-08-16 both packages were named
    # `image_pipeline` and MERGED into one on-device directory; the X8-side
    # package is now `x8_image_pipeline`, so they simply coexist.)
    (Join-Path $tractorX8 'x8_image_pipeline')
)
$pahoLocal = Join-Path $repoRoot "_paho_pull\paho"
if (Test-Path $pahoLocal) { $deploySrc += $pahoLocal }
$deployUp = @(foreach ($src in $deploySrc) { Get-CleanPushSource -Local $src -StageRoot $pushStage })
foreach ($s in @($TxAdbSerial, $RxAdbSerial)) {
    cmd /c "`"$adbExe`" -s $s shell `"echo fio | sudo -S -p '' mkdir -p /tmp/lifetrac_strict ; echo fio | sudo -S -p '' chmod 0777 /tmp/lifetrac_strict`"" | Out-Null
    $pushed = New-Object System.Collections.Generic.List[object]
    foreach ($up in $deployUp) {
        Invoke-AdbPush -Adb $adbExe -Serial $s -Local $up -Remote "/tmp/lifetrac_strict/" | Out-Null
        foreach ($f in @(Get-PushedFileList -Local $up -Remote "/tmp/lifetrac_strict/")) { $pushed.Add($f) }
    }
    $md5 = Confirm-AdbPushedFiles -Adb $adbExe -Serial $s -Files $pushed.ToArray()
    foreach ($row in $md5.Rows) { $deployMd5.Add("$s $row") }
    $deployMd5Note[$s] = $md5.Summary
    Write-Host "  [DEPLOY $s] $($deployUp.Count) pushes OK, $($md5.Summary)"
}
# 3a. -InjectCmd (2026-10-18): the base needs bench_tools/inject_watch.sh and the
# injector script in /tmp/lifetrac_strict. The watcher goes up LF-only (a CRLF
# checkout would break it on the board); the injector is pushed when it is in
# bench_tools, otherwise it must already be staged there (legs/stage_boards.sh).
if ($injectSpec) {
    $injWatchSrc = Join-Path $helperDir "bench_tools\inject_watch.sh"
    $injWatchTmp = Join-Path ([System.IO.Path]::GetTempPath()) "lifetrac_inject_watch.sh"
    [System.IO.File]::WriteAllText($injWatchTmp, ((Get-Content -Raw -LiteralPath $injWatchSrc) -replace "`r", ""))
    Invoke-AdbPush -Adb $adbExe -Serial $RxAdbSerial -Local $injWatchTmp -Remote "/tmp/lifetrac_strict/inject_watch.sh" | Out-Null
    $injPushed = @(Get-PushedFileList -Local $injWatchTmp -Remote "/tmp/lifetrac_strict/inject_watch.sh")
    $injScriptLocal = Join-Path $helperDir "bench_tools\$($injectSpec.Script)"
    if (Test-Path -LiteralPath $injScriptLocal) {
        Invoke-AdbPush -Adb $adbExe -Serial $RxAdbSerial -Local $injScriptLocal -Remote "/tmp/lifetrac_strict/" | Out-Null
        $injPushed += @(Get-PushedFileList -Local $injScriptLocal -Remote "/tmp/lifetrac_strict/")
    }
    $injMd5 = Confirm-AdbPushedFiles -Adb $adbExe -Serial $RxAdbSerial -Files $injPushed
    foreach ($row in $injMd5.Rows) { $deployMd5.Add("$RxAdbSerial $row") }
    $deployMd5Note[$RxAdbSerial] = "$($deployMd5Note[$RxAdbSerial]); injector $($injMd5.Summary)"
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
# 2026-10-19: bench_tools/ocd_reset.sh (the background reset of step 5) goes
# up next to the cfg, LF-only like inject_watch.sh; both pushes are checked and
# md5-compared, and a failure stops the leg here (no daemon has started).
cmd /c "`"$adbExe`" -s $TxAdbSerial shell `"echo fio | sudo -S -p '' mkdir -p /tmp/lifetrac_p0c ; echo fio | sudo -S -p '' chmod 0777 /tmp/lifetrac_p0c`"" | Out-Null
$cfgLocal = Join-Path $helperDir '08_boot_user_app.cfg'
$ocdShSrc = Join-Path $helperDir "bench_tools\ocd_reset.sh"
$ocdShTmp = Join-Path ([System.IO.Path]::GetTempPath()) "lifetrac_ocd_reset.sh"
if (-not (Test-Path -LiteralPath $ocdShSrc)) {
    $msg = "[DEPLOY] $ocdShSrc is missing from this checkout (the leg-start SWD reset runs from it). No daemon was started."
    Write-Host $msg -ForegroundColor Red
    throw $msg
}
[System.IO.File]::WriteAllText($ocdShTmp, ([System.IO.File]::ReadAllText($ocdShSrc) -replace "`r", ""))
Invoke-AdbPush -Adb $adbExe -Serial $TxAdbSerial -Local $cfgLocal -Remote "/tmp/lifetrac_p0c/08_boot_user_app.cfg" | Out-Null
Invoke-AdbPush -Adb $adbExe -Serial $TxAdbSerial -Local $ocdShTmp -Remote "/tmp/lifetrac_p0c/ocd_reset.sh" | Out-Null
$p0cFiles = @(Get-PushedFileList -Local $cfgLocal -Remote "/tmp/lifetrac_p0c/08_boot_user_app.cfg") +
    @(Get-PushedFileList -Local $ocdShTmp -Remote "/tmp/lifetrac_p0c/ocd_reset.sh")
$p0cMd5 = Confirm-AdbPushedFiles -Adb $adbExe -Serial $TxAdbSerial -Files $p0cFiles
foreach ($row in $p0cMd5.Rows) { $deployMd5.Add("$TxAdbSerial $row") }
$deployMd5Note[$TxAdbSerial] = "$($deployMd5Note[$TxAdbSerial]); reset files $($p0cMd5.Summary)"
$cfgCheck = cmd /c "`"$adbExe`" -s $TxAdbSerial shell `"test -s /tmp/lifetrac_p0c/08_boot_user_app.cfg && echo CFG-OK || echo CFG-MISSING; test -s /tmp/lifetrac_p0c/ocd_reset.sh && echo OCDSH-OK || echo OCDSH-MISSING`"" 2>$null
$cfgOk = ("$cfgCheck" -match "CFG-OK")
if (-not $cfgOk) {
    Write-Warning "/tmp/lifetrac_p0c/08_boot_user_app.cfg is not on the tractor ($TxAdbSerial); the SWD reset of the TX L072 will fail."
}
if ("$cfgCheck" -notmatch "OCDSH-OK") {
    Write-Warning "/tmp/lifetrac_p0c/ocd_reset.sh is not on the tractor ($TxAdbSerial); the SWD reset of the TX L072 will not be spawned."
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
# 2026-10-19: the reset runs from the pushed bench_tools/ocd_reset.sh (same
# steps, same cfgs, same OCD_LEG tag / OCD_EXIT lines and swd_reset_spawned
# reply). Leg A2c2 that day logged only its tag and OCD_EXIT=129, no openocd
# banner: the hangup reached nohup before nohup ignored HUP, because the
# inline subshell's caught `trap :` is reset on exec. The script sets an
# IGNORED disposition (`trap '' HUP`, which cannot be written inside the
# single-quoted sh -c this used to be) before it forks, and an ignored
# disposition survives fork and exec into nohup and openocd, which closes the
# A2c2 window. openocd then installs its own SIGHUP handler once it starts
# (server_preinit, after its banner), so a later hangup can still end it with
# 129; the verdict tells the two apart by the banner.
Write-Host "[RESET] TX L072 via OpenOCD SWD (background, BOOT0 low + NRST)..."
# [A-Z0-9=] only: it passes cmd, adb and sh unchanged.
$ocdTag = "OCD_LEG=" + (Get-Date).ToUniversalTime().ToString("yyyyMMddTHHmmssZ")
# stderr joins stdout on the board, so a sudo or sh error lands in $swdOut
# (and in the verdict when swd_reset_spawned is missing).
$swdReset = "echo fio | sudo -S -p '' sh /tmp/lifetrac_p0c/ocd_reset.sh $ocdTag 2>&1"
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

# 8. End of leg (bench kit, 2026-10-19): stop the frame source, let the TX
# queue drain (-StopDrainS), stop tx_smoke, then rx_smoke 0.5 s later, and
# only THEN read the daemon logs. Up to the 10-19 legs the harness read both
# logs while the daemons still ran (TX first, then RX) and stopped them
# afterwards, so each log ended at a different moment, both before the post
# brackets (read after the stop): the base sent one command more than its log
# showed (10-19 A2c 211/210, A2a 217/216, A2a2 218/217, A2c2 211/210) and
# rs12_leg_report.py saw "received > sent" (A2c 612 > 611, A2a 609 > 607). A
# stopped container keeps its log until it is removed (the next launch's
# rm -f), so reading after the stop loses nothing.
# 2026-10-22: each stop's output and exit status are kept (Invoke-DockerStop;
# they went to Out-Null before) and judged in 8c with the SIGTERM shutdown
# line of each daemon's log (Get-LogWindowVerdict): params.txt says
# log_window=after_stop only when both stops worked and both lines are there,
# else log_window=after_stop_unverified and log_window_detail=<reason>. The
# frame source's stop is recorded as feed_stop= only (not in the verdict).
Write-Host "[STOP] frame source, then tx_smoke after a $StopDrainS s drain, then rx_smoke; the logs are read after the stop..." -ForegroundColor Cyan
$feedStopNote = "none"
if ($pubProc) {
    if ($pubProc.HasExited) {
        $feedStopNote = "host publisher on the PC had already exited"
    } else {
        try { $pubProc.Kill(); $feedStopNote = "host publisher on the PC killed" } catch { $feedStopNote = "host publisher on the PC: kill failed: " + ($_.Exception.Message -replace '[\r\n]+', ' ') }
    }
}
if ($TxFeed -eq "camera") {
    # Stopped, not removed: its log is read below (the next launch rm -f's it),
    # and the camera is free for the next leg.
    $feedStopOut = Invoke-DockerStop -Adb $adbExe -Serial $TxAdbSerial -Name "camera_svc"
    $feedStopNote = "camera_svc " + (Get-DockerStopResult -Output $feedStopOut -Name "camera_svc").Text
} elseif ($TxFeed -eq "local") {
    # Stopped, not removed: its OFFERED line is read below, then it is removed.
    $feedStopOut = Invoke-DockerStop -Adb $adbExe -Serial $TxAdbSerial -Name "synth_pub"
    $feedStopNote = "synth_pub " + (Get-DockerStopResult -Output $feedStopOut -Name "synth_pub").Text
}
$feedStoppedAt = Get-Date

# 8a. -InjectCmd (2026-10-18): the leg is over, so stop the injector (or the
# watcher still waiting for its trigger) and collect the base log. Before
# tx_smoke stops (2026-10-19), so every command it made the base send can
# still reach the tractor's log; it counts toward the drain.
$injLog = @()
$injSum = $null
if ($injectSpec) {
    $injLog = @(ConvertFrom-AdbOutput (cmd /c "`"$adbExe`" -s $RxAdbSerial shell `"$injWatchCmd stop`""))
    $injSum = Get-InjectSummary $injLog
    if ($injectSpec.On -eq "fault" -and $injFaultPending) {
        $injPc.Add("$((Get-Date).ToUniversalTime().ToString('HH:mm:ss')) PC leg ended; no tractor FAULT matched $($injectSpec.Label) ($($injFaultsSeen.Count) FAULT line(s) with code 0x$('{0:X2}' -f $injectSpec.Code) seen)")
    }
}

$drainLeftMs = [int][math]::Round(($StopDrainS * 1000.0) - ((Get-Date) - $feedStoppedAt).TotalMilliseconds)
if ($drainLeftMs -gt 0) { Start-Sleep -Milliseconds $drainLeftMs }
$txStopOut = Invoke-DockerStop -Adb $adbExe -Serial $TxAdbSerial -Name "tx_smoke"
# The base keeps listening a moment longer, for a frame still on air.
Start-Sleep -Milliseconds 500
$rxStopOut = Invoke-DockerStop -Adb $adbExe -Serial $RxAdbSerial -Name "rx_smoke"
Write-Host "  [STOP] feed: $feedStopNote; tx_smoke: $((Get-DockerStopResult -Output $txStopOut -Name 'tx_smoke').Text); rx_smoke: $((Get-DockerStopResult -Output $rxStopOut -Name 'rx_smoke').Text)"

# 8b0. Board clocks (2026-10-19), read only: each board's `date +%s.%N` minus
# the PC's UTC epoch at the adb call's midpoint. The tractor clock runs ~19 days
# behind (its logs read 2026-09-30 on 10-19); it is never set, so cross-board
# timing uses these offsets (or matched events).
$clkTractor = Read-BoardClock -Adb $adbExe -Serial $TxAdbSerial
$clkBase = Read-BoardClock -Adb $adbExe -Serial $RxAdbSerial

Write-Host "`n=== FINAL LOG SUMMARY ===" -ForegroundColor Cyan
$txFinal = cmd /c "`"$adbExe`" -s $TxAdbSerial shell `"echo fio | sudo -S -p '' docker logs tx_smoke 2>&1`""
$rxFinal = cmd /c "`"$adbExe`" -s $RxAdbSerial shell `"echo fio | sudo -S -p '' docker logs rx_smoke 2>&1`""
$synthFinal = ""
$camFinal = $null
if ($TxFeed -eq "camera") {
    # RS-13.1: keep camera_service's log (encoder `vector_stats` lines, mode
    # acks) in the evidence bundle (the container was stopped above).
    $camFinal = cmd /c "`"$adbExe`" -s $TxAdbSerial shell `"echo fio | sudo -S -p '' docker logs camera_svc 2>&1`""
    Write-Host "--- CAMERA SERVICE LOGS (last 8) ---" -ForegroundColor Yellow
    # adb's \r\r\n puts an empty element after every line (ConvertFrom-AdbOutput),
    # so a plain -Last 8 showed only 4 lines (leg V, 2026-10-18).
    Write-Host ((@(ConvertFrom-AdbOutput $camFinal) | Select-Object -Last 8) -join "`n")
}
if ($TxFeed -eq "local") {
    # Capture the publisher's OFFERED line for the evidence bundle, then
    # remove it (stopped above). bench_mqtt stays up for reuse across runs.
    $synthFinal = cmd /c "`"$adbExe`" -s $TxAdbSerial shell `"echo fio | sudo -S -p '' docker logs synth_pub 2>&1`""
    cmd /c "`"$adbExe`" -s $TxAdbSerial shell `"echo fio | sudo -S -p '' docker rm -f synth_pub 2>/dev/null`"" | Out-Null
    Write-Host "--- SYNTH PUBLISHER LOGS ---" -ForegroundColor Yellow
    Write-Host $synthFinal
}

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
# legs/leg_post.sh looks for, and repeated in params.txt. LOG_WINDOW
# (2026-10-22) is judged here, from the 8. stop outputs and the final logs
# read above: green after_stop, or after_stop_unverified (red when a daemon
# stop failed, yellow when only a shutdown line is missing) with the reason.
$logWin = Get-LogWindowVerdict -TxStopOutput $txStopOut -RxStopOutput $rxStopOut -TxLog $txFinal -RxLog $rxFinal
$baseResetLine = "BASE_L072_RESET unverified (gpio163 pulse sent; nothing checks that the base L072 rebooted, and it has not since 2026-08: score base brackets as post - pre)"
$nrstNote = "none"
if ($nrstOut.Count -gt 0) { $nrstNote = $nrstOut -join ' | ' }
$tractorResetLine = "TRACTOR_L072_RESET $($resetV.Verdict) ($($resetV.Reason))"
Write-Host "`n=== LEG CHECKS ===" -ForegroundColor Cyan
$resetColor = "Yellow"
if ($resetV.Verdict -eq "landed") { $resetColor = "Green" } elseif ($resetV.Verdict -eq "failed") { $resetColor = "Red" }
Write-Host $tractorResetLine -ForegroundColor $resetColor
if ($resetV.Verdict -ne "landed") {
    Write-Host "  The tractor's L072 counters may carry over from before this leg: score its brackets with bench_tools/bracket_deltas.py (reset-aware: py -3 bracket_deltas.py <legs dir> <leg>)." -ForegroundColor Yellow
}
Write-Host $baseResetLine -ForegroundColor Yellow
Write-Host "  gpio163 pulse output: $nrstNote"
# 2026-10-19: board clocks (read after the stop, never set) and the window the
# daemon logs cover.
$clockLine = "CLOCKS board minus PC: tractor $($clkTractor.Text); base $($clkBase.Text) (read only)"
Write-Host $clockLine
# The feed's own stop result speaks for itself (feed: ...), since it is not
# part of the verdict and can have failed on a verified window.
if ($logWin.Verified) {
    $logWinLine = "LOG_WINDOW after_stop (feed: $feedStopNote; $StopDrainS s drain, tx_smoke stopped, rx_smoke stopped 0.5 s later, then the logs were read; $($logWin.Detail))"
} else {
    $logWinLine = "LOG_WINDOW after_stop_unverified ($($logWin.Detail); feed: $feedStopNote)"
}
Write-Host $logWinLine -ForegroundColor $logWin.Color
if (-not $logWin.Verified) {
    Write-Host "  $($logWin.Hint)" -ForegroundColor Yellow
}
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
    # 2026-10-19: the md5 of every file this leg pushed, board against PC
    # ("<serial> ok|MISMATCH|missing <remote path> pc=<md5> board=<md5|->").
    $deployMd5 | Out-File -FilePath (Join-Path $dir "deploy_md5.txt") -Encoding utf8
    if ($injectSpec) {
        $injLog | Out-File -FilePath (Join-Path $dir "inject.log") -Encoding utf8
        @($injPc) + @($injectLine) | Out-File -FilePath (Join-Path $dir "inject_trigger.txt") -Encoding utf8
    }
    $injStatus = ""; $injTrigger = ""; $injOnLabel = ""
    if ($injSum) { $injStatus = $injSum.Status; $injTrigger = $injSum.Trigger }
    if ($injectSpec) { $injOnLabel = $injectSpec.Label }
    # 2026-10-19: clock offsets as plain numbers (seconds, board minus PC).
    $inv = [System.Globalization.CultureInfo]::InvariantCulture
    $tOff = "unknown"; $tRtt = "unknown"; $bOff = "unknown"; $bRtt = "unknown"
    if ($clkTractor.Ok) { $tOff = $clkTractor.OffsetS.ToString("F3", $inv) }
    if ($null -ne $clkTractor.RttS) { $tRtt = $clkTractor.RttS.ToString("F3", $inv) }
    if ($clkBase.Ok) { $bOff = $clkBase.OffsetS.ToString("F3", $inv) }
    if ($null -ne $clkBase.RttS) { $bRtt = $clkBase.RttS.ToString("F3", $inv) }
    # 2026-10-22: log_window_detail= only on an unverified window; the empty
    # placeholder is dropped below (every other line starts with its key).
    $logWinDetailLine = ""
    if (-not $logWin.Verified) { $logWinDetailLine = "log_window_detail=$($logWin.Detail)" }
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
        "log_window=$($logWin.Value)",
        $logWinDetailLine,
        "feed_stop=$feedStopNote",
        "stop_drain_s=$StopDrainS",
        "tractor_clock_offset_s=$tOff",
        "base_clock_offset_s=$bOff",
        "tractor_clock_rtt_s=$tRtt",
        "base_clock_rtt_s=$bRtt",
        "clock_offset_rule=board date +%s.%N minus the PC UTC epoch at the midpoint of the adb read (error at most rtt/2, +1 s if the CLOCKS line says whole seconds), read after the daemons stopped; no clock was set",
        "tractor_deploy_md5=$($deployMd5Note[$TxAdbSerial])",
        "base_deploy_md5=$($deployMd5Note[$RxAdbSerial])",
        "timestamp=$stamp"
    ) | Where-Object { $_ -ne "" } | Out-File -FilePath (Join-Path $dir "params.txt") -Encoding utf8
    Write-Host "[EVIDENCE] archived to $dir" -ForegroundColor Green
}

Write-Host "=== LIVE LORA RADIO MONITOR COMPLETE ===" -ForegroundColor Green
