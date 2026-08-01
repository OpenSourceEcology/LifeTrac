#requires -Version 5.1
<#
.SYNOPSIS
    Live LoRa Radio Transmission Monitor & Speed Benchmark.
.DESCRIPTION
    Launches image_rx_daemon on 2D0A and image_tx_daemon on 2E2C, streams
    synthetic frames from host, and logs real-time throughput / goodput (B/s),
    fragment counts, and demodulation metrics (RSSI, SNR).
#>
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
    # Archive final logs + parameters under bench-evidence/ with the git
    # SHA in the folder name (evidence discipline per CODE REVIEWS docs).
    [switch]$Archive
)

Set-StrictMode -Version Latest
$ErrorActionPreference = "Continue"

# Profile env for both daemons; profile 1 needs the 50-ch wide mask.
$profEnv = "-e LIFETRAC_REG_PROFILE=$RegProfile"
if ($RegProfile -eq 1) { $profEnv = "$profEnv -e LIFETRAC_FHSS_WIDE_MASK=1" }
# FHSS seed — same value to both daemons by construction (see the
# FhssLinkId param comment for why that matters).
$profEnv = "$profEnv -e LIFETRAC_FHSS_FARM_ID=$FhssFarmId -e LIFETRAC_FHSS_LINK_ID=$FhssLinkId"
if ($AirtimeBudgetUs -gt 0) { $profEnv = "$profEnv -e LIFETRAC_AIRTIME_BUDGET_US=$AirtimeBudgetUs" }
$profEnv = "$profEnv -e LIFETRAC_AIRTIME_PACING=$PacingMode"
if ($PacingHeadroom -gt 0) { $profEnv = "$profEnv -e LIFETRAC_PACING_HEADROOM=$PacingHeadroom" }

# Resolve adb dynamically (the winget package dir name varies per
# machine/source); fall back to the historical hardcoded path.
$adbExe = (Get-Command adb -ErrorAction SilentlyContinue).Source
if (-not $adbExe) {
    $adbExe = "C:\Users\dorkm\AppData\Local\Microsoft\WinGet\Packages\Google.PlatformTools_8wekyb3d8bbwe\platform-tools\adb.exe"
}
$repoRoot = (Resolve-Path "$PSScriptRoot\..\..\..\..").Path
$baseStation = Join-Path $repoRoot "LifeTrac-v25\DESIGN-CONTROLLER\base_station"
$tractorX8   = Join-Path $repoRoot "LifeTrac-v25\DESIGN-CONTROLLER\firmware\tractor_x8"
$helperDir   = $PSScriptRoot

Write-Host "=== STARTING LIVE LORA RADIO MONITOR ($DurationS s) ===" -ForegroundColor Cyan
Write-Host "TX Serial: $TxAdbSerial"
Write-Host "RX Serial: $RxAdbSerial"
Write-Host "MQTT Host: $HostIp"

# 1. Ensure Host MQTT Broker is running
$socket = New-Object System.Net.Sockets.TcpClient
$asyncResult = $socket.BeginConnect($HostIp, 1883, $null, $null)
$waitResult = $asyncResult.AsyncWaitHandle.WaitOne(1000, $false)
if (-not $waitResult -or -not $socket.Connected) {
    Write-Host "[MQTT] Starting host broker on $HostIp:1883..." -ForegroundColor Yellow
    Start-Process -FilePath "C:\Users\dorkm\AppData\Local\Python\pythoncore-3.14-64\python.exe" -ArgumentList "start_mqtt_broker.py" -WorkingDirectory $repoRoot -PassThru -NoNewWindow
    Start-Sleep -Seconds 2
} else {
    $socket.Close()
    Write-Host "[MQTT] Host broker active at $HostIp:1883" -ForegroundColor Green
}

# 2. Clean up previous containers
Write-Host "[CLEANUP] Stopping old containers..."
cmd /c "`"$adbExe`" -s $TxAdbSerial shell `"echo fio | sudo -S docker rm -f tx_smoke 2>/dev/null`"" | Out-Null
cmd /c "`"$adbExe`" -s $RxAdbSerial shell `"echo fio | sudo -S docker rm -f rx_smoke 2>/dev/null`"" | Out-Null

# 3. Deploy code
Write-Host "[DEPLOY] Syncing code to both boards..."
foreach ($s in @($TxAdbSerial, $RxAdbSerial)) {
    cmd /c "`"$adbExe`" -s $s shell `"echo fio | sudo -S -p '' mkdir -p /tmp/lifetrac_strict ; echo fio | sudo -S -p '' chmod 0777 /tmp/lifetrac_strict`"" | Out-Null
    cmd /c "`"$adbExe`" -s $s push `"$(Join-Path $helperDir 'method_g_stage1_probe.py')`" /tmp/lifetrac_strict/" | Out-Null
    cmd /c "`"$adbExe`" -s $s push `"$(Join-Path $helperDir 'method_h_stage2_tx_probe_v2.py')`" /tmp/lifetrac_strict/" | Out-Null
    cmd /c "`"$adbExe`" -s $s push `"$(Join-Path $baseStation 'lora_proto.py')`" /tmp/lifetrac_strict/" | Out-Null
    cmd /c "`"$adbExe`" -s $s push `"$(Join-Path $baseStation 'image_rx_daemon.py')`" /tmp/lifetrac_strict/" | Out-Null
    cmd /c "`"$adbExe`" -s $s push `"$(Join-Path $tractorX8 'image_tx_daemon.py')`" /tmp/lifetrac_strict/" | Out-Null
    cmd /c "`"$adbExe`" -s $s push `"$(Join-Path $tractorX8 'camera_service.py')`" /tmp/lifetrac_strict/" | Out-Null
    cmd /c "`"$adbExe`" -s $s push `"$(Join-Path $repoRoot 'publish_synthetic_frames.py')`" /tmp/lifetrac_strict/" | Out-Null
    cmd /c "`"$adbExe`" -s $s push `"$(Join-Path $helperDir 'bench_mqtt.conf')`" /tmp/lifetrac_strict/" | Out-Null
    cmd /c "`"$adbExe`" -s $s push `"$(Join-Path $baseStation 'image_pipeline')`" /tmp/lifetrac_strict/" | Out-Null
    # 2026-07-30 RS-3.3: the TRACTOR's image_pipeline too. Two different
    # packages share the name `image_pipeline` (RS-5.7), and only the base
    # station's was being deployed — so camera_service died on
    # "No module named 'image_pipeline.ipc_to_h747'" the first time it was
    # ever run on hardware. They merge safely: of 16 + 11 modules the ONLY
    # overlapping filename is __init__.py, and both are docstring-only.
    # image_tx_daemon needs frame_format from the base copy while
    # camera_service needs ipc_to_h747/tile_cache from this one, so the
    # tractor genuinely requires both.
    cmd /c "`"$adbExe`" -s $s push `"$(Join-Path $tractorX8 'image_pipeline')`" /tmp/lifetrac_strict/" | Out-Null
    $pahoLocal = Join-Path $repoRoot "_paho_pull\paho"
    if (Test-Path $pahoLocal) {
        cmd /c "`"$adbExe`" -s $s push `"$pahoLocal`" /tmp/lifetrac_strict/" | Out-Null
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
# RX board (base): gpio163 NRST works; reset THEN launch.
# TX board (tractor): gpio163 is dead (writes no-op) — use the OpenOCD SWD
# path (H7 PA11/BOOT0 low + PF4/NRST via /dev/mem). CRITICAL ORDER: the
# tractor L072 wedges (TX-line break storm -> ISR livelock) when it runs
# with /dev/ttymxc3 CLOSED, so launch the TX daemon FIRST (it holds the
# port open and retries VER x5), then fire the SWD reset into the waiting
# reader.
Write-Host "[RESET] RX L072 via gpio163 NRST..."
$nrstCmd = "echo fio | sudo -S -p '' sh -c '[ -d /sys/class/gpio/gpio163 ] || echo 163 > /sys/class/gpio/export; echo out > /sys/class/gpio/gpio163/direction; echo 1 > /sys/class/gpio/gpio163/value; sleep 0.02; echo 0 > /sys/class/gpio/gpio163/value; sleep 0.10; echo 1 > /sys/class/gpio/gpio163/value'"
cmd /c "`"$adbExe`" -s $RxAdbSerial shell `"$nrstCmd`"" | Out-Null
cmd /c "`"$adbExe`" -s $TxAdbSerial push `"$(Join-Path $helperDir '08_boot_user_app.cfg')`" /tmp/lifetrac_p0c/08_boot_user_app.cfg" | Out-Null

# 5. Fire the tractor SWD reset in the BACKGROUND, then launch the TX daemon
# immediately. openocd needs ~7-10 s to reach its NRST pulse; the daemon needs
# ~5 s (docker) + runs 5 VER retries over ~12 s. Backgrounding the reset makes
# the NRST land mid-retry-window instead of after it.
Write-Host "[RESET] TX L072 via OpenOCD SWD (background, BOOT0 low + NRST)..."
$swdReset = "echo fio | sudo -S -p '' sh -c 'mkdir -p /tmp/lifetrac_p0c; pkill -9 openocd 2>/dev/null; sleep 0.5; for g in 8 10 15; do [ -d /sys/class/gpio/gpio`$g ] || echo `$g > /sys/class/gpio/export 2>/dev/null; done; echo out > /sys/class/gpio/gpio10/direction 2>/dev/null; echo 1 > /sys/class/gpio/gpio10/value 2>/dev/null; sleep 1; for g in 8 10 15; do echo `$g > /sys/class/gpio/unexport 2>/dev/null; done; nohup openocd -f /usr/arduino/extra/openocd_script-imx_gpio.cfg -f /tmp/lifetrac_p0c/08_boot_user_app.cfg >/tmp/lifetrac_p0c/ocd_reset.log 2>&1 & echo swd_reset_spawned'"
cmd /c "`"$adbExe`" -s $TxAdbSerial shell `"$swdReset`""

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
cmd /c "`"$adbExe`" -s $TxAdbSerial shell `"echo fio | sudo -S -p '' docker rm -f tx_smoke 2>/dev/null ; echo fio | sudo -S -p '' docker run -d --name tx_smoke --network=host --entrypoint python3 --device=/dev/ttymxc3 -v /tmp/lifetrac_strict:/work -w /work -e PYTHONPATH=/work:/work/paho -e LIFETRAC_MQTT_HOST=$txMqtt -e LIFETRAC_SKIP_RESET_REQ=1 $profEnv -e LIFETRAC_TX_PIPELINE=$TxPipeline -e LIFETRAC_TX_PIPELINE_DEPTH=$TxPipelineDepth -e LIFETRAC_TX_BATCH=$TxBatch -e LIFETRAC_TX_PREPARE_AHEAD=$TxPrepareAhead -e LIFETRAC_TRAIN_GAP_MS=$TrainGapMs -e LIFETRAC_PARITY_GROUP=$ParityGroup -e LIFETRAC_ACK_COPIES=$AckCopies -e LIFETRAC_PROBE_ECHO=$ProbeEcho hub.foundries.io/arduino/arduino-ootb-python-devel:738bc44 -u /work/image_tx_daemon.py --log-level INFO`""

# 6. Launch RX daemon.
# RX publishes to the BASE BOARD's own mosquitto (127.0.0.1 via
# --network=host) rather than the Windows host broker: the base→host
# Wi-Fi leg drops ARP within a minute (see [NET] warm-up), and
# production topology has the base web_ui subscribed to the base
# broker anyway — tile_delta + link_stats land where the web UI reads.
# RS-0.13b/RS-0.12 (2026-07-27): aligned-pump A/B + reactive-fire probe env.
$rxExtraEnv = "-e LIFETRAC_ALIGNED_PUMP=$AlignedPump -e LIFETRAC_REACTIVE_FIRE=$ReactiveFire"
if ($ProbePhaseSweepMs -ne "") { $rxExtraEnv = "$rxExtraEnv -e LIFETRAC_PROBE_PHASE_SWEEP_MS=$ProbePhaseSweepMs" }
if ($ProbeSizesB -ne "") { $rxExtraEnv = "$rxExtraEnv -e LIFETRAC_PROBE_SIZES_B=$ProbeSizesB" }
Write-Host "[LAUNCH] Starting RX Daemon on Board $RxAdbSerial..." -ForegroundColor Yellow
cmd /c "`"$adbExe`" -s $RxAdbSerial shell `"echo fio | sudo -S -p '' docker rm -f rx_smoke 2>/dev/null ; echo fio | sudo -S -p '' docker run -d --name rx_smoke --network=host --device=/dev/ttymxc3 -v /tmp/lifetrac_strict:/work -w /work -e PYTHONPATH=/work:/work/paho -e LIFETRAC_MQTT_HOST=127.0.0.1 -e LIFETRAC_CTRL_MQTT_HOST=$HostIp -e LIFETRAC_SKIP_RESET_REQ=1 -e LIFETRAC_KF_REQUEST_DISABLE=$KfRequestDisable $rxExtraEnv $profEnv lifetrac-v25:latest python3 -u /work/image_rx_daemon.py --log-level INFO`""

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
    # LIFETRAC_CAMERA_DESHAKE=0 is REQUIRED, not cosmetic. camera_service
    # defaults deshake ON (_default_deshake = "0" if USE_LORA_BRIDGE else "1",
    # and USE_LORA_BRIDGE defaults false), and measured 2026-07-30 the i.MX8
    # cannot produce a SINGLE frame through
    # "fps=2,scale=384:256,deshake=open2=1:search=16" in 30 s -- 0 bytes, vs
    # 589824 bytes (2 frames, exact) with the identical pipeline minus deshake.
    # Left at its default, camera_service restarts ffmpeg forever and delivers
    # nothing. Set explicitly rather than via LIFETRAC_USE_LORA_BRIDGE=1, which
    # would also flip the image method A->C and confound the encode-to-fit test.
    $camRun = "echo fio | sudo -S -p '' docker rm -f camera_svc 2>/dev/null ; echo fio | sudo -S -p '' docker run -d --name camera_svc --network=host --entrypoint python3 --device=$CameraDevice -v /tmp/lifetrac_strict:/work -w /work -e PYTHONPATH=/work:/work/paho -e LIFETRAC_MQTT_HOST=127.0.0.1 -e LIFETRAC_CAMERA_SOURCE=v4l2 -e LIFETRAC_CAMERA_DEVICE=$CameraDevice -e LIFETRAC_CAMERA_FPS=$SynthFps -e LIFETRAC_CAMERA_DESHAKE=0 $profEnv lifetrac-tractor-x8:latest -u /work/camera_service.py"
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
    $pubProc = Start-Process -FilePath "C:\Users\dorkm\AppData\Local\Python\pythoncore-3.14-64\python.exe" -ArgumentList "`"$pubScript`"" -PassThru -NoNewWindow -WorkingDirectory $repoRoot
}

Write-Host "=== MONITORING RADIO TRANSMISSIONS FOR $DurationS SECONDS ===" -ForegroundColor Green
$startT = Get-Date
while (((Get-Date) - $startT).TotalSeconds -lt $DurationS) {
    Start-Sleep -Seconds 5
    $txStats = cmd /c "`"$adbExe`" -s $TxAdbSerial shell `"echo fio | sudo -S -p '' docker logs --tail 2 tx_smoke 2>&1`""
    $rxStats = cmd /c "`"$adbExe`" -s $RxAdbSerial shell `"echo fio | sudo -S -p '' docker logs --tail 2 rx_smoke 2>&1`""
    Write-Host "--- TX Log ---" -ForegroundColor Cyan
    Write-Host $txStats
    Write-Host "--- RX Log ---" -ForegroundColor Magenta
    Write-Host $rxStats
}

# 8. Clean up publisher & containers
if ($pubProc -and -not $pubProc.HasExited) {
    try { $pubProc.Kill() } catch {}
}

Write-Host "`n=== FINAL LOG SUMMARY ===" -ForegroundColor Cyan
$txFinal = cmd /c "`"$adbExe`" -s $TxAdbSerial shell `"echo fio | sudo -S -p '' docker logs tx_smoke 2>&1`""
$rxFinal = cmd /c "`"$adbExe`" -s $RxAdbSerial shell `"echo fio | sudo -S -p '' docker logs rx_smoke 2>&1`""
$synthFinal = ""
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

Write-Host "--- TX DAEMON FINAL LOGS ---" -ForegroundColor Yellow
Write-Host $txFinal
Write-Host "--- RX DAEMON FINAL LOGS ---" -ForegroundColor Yellow
Write-Host $rxFinal

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
        "tx_feed=$TxFeed",
        "reg_profile=$RegProfile",
        "tx_serial=$TxAdbSerial",
        "rx_serial=$RxAdbSerial",
        "host_ip=$HostIp",
        "timestamp=$stamp"
    ) | Out-File -FilePath (Join-Path $dir "params.txt") -Encoding utf8
    Write-Host "[EVIDENCE] archived to $dir" -ForegroundColor Green
}

Write-Host "=== LIVE LORA RADIO MONITOR COMPLETE ===" -ForegroundColor Green
