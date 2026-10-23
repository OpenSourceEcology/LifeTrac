# Bench runbook — radio legs on the LifeTrac v25 bench

Written 2026-09-14 at the end of the RS-12.14 / RS-12.15 campaign so the
next session starts from a checklist instead of memory. Everything that
keys a radio needs the operator's GO; radios are parked (LoRa SLEEP,
`0x80` readback) whenever a session ends.

**New to the bench?** Start at [BENCH_QUICKSTART.md](BENCH_QUICKSTART.md).
It walks through PC and board setup, radio firmware, deploy, a first leg,
analysis and park, in order, with the commands. This runbook is the
per-session reference and the trap list. The rest of the bench kit:

- [PC_SETUP.md](PC_SETUP.md)
- [BENCH_SETUP.md](BENCH_SETUP.md)
- [DEPLOY.md](DEPLOY.md)
- [`bench.env.example`](bench.env.example)
- [legs/README.md](legs/README.md)

## What is here

| file | runs on | purpose |
|---|---|---|
| `run_flash_bench.sh` + `stamp.py` + `kmsg_log.py` | board, `/home/fio` (persistent) | instrumented wrapper around `full_flash_pipeline.sh`: monotonic-stamped pipeline log + kernel log, fsync'd so a PMIC power-cycle cannot lose them. `REVIVE_MODE=reboot bash /home/fio/run_flash_bench.sh <bin>`; success = `Verify OK` + `flash_rc=0` in `/home/fio/pipeline_stamped.log` (~2.5 min incl. reboot). |
| `radio_park.py` | board, daemon container | STANDBY → SLEEP, reads RegOpMode back, waits 1.5 s, reads again; prints `PARK_OK` only if still `0x80` (`PARK_TRANSIENT`, exit 5, if the firmware re-armed it). **The readback is NOT a persistent-off guarantee** (PR #125 review): the firmware's scan walker, γ-1 retune and slot follower all call `sx1276_rx_arm()` and will bring the modem back to RXCONT (receive-only) while any of them still has work. The park holds once the scan SM has exhausted into FAILED — up to ~2 min after the daemons stop (500 ms/channel, hard FAIL at 30 s per attempt, 3 retries). Park after that, and confirm later with `radio_state.py`. Firmware park command = TODO RS-12.21. |
| `radio_state.py` | board, daemon container | read-only RegOpMode decode — answers "is it off?" without touching it. |
| `clear_retained.py` / `clear_retained_host.py <host>` | base (container) / PC | clear the retained control topics (`encode_mode_override`, `req_keyframe`, `radio_profile`) on the base broker / on another broker. A stale retained pin has flipped the camera and re-commanded a profile mid-leg; clear before EVERY leg. |
| `kf_inject.py <period_s> <count>` | base (container) | REQ_KEYFRAME storm; the unacked retries at 0.4/0.8 s make the 2–3-command bursts that LOCK the tractor's scan machine (the RS-12.15 trigger). |
| `dual_inject.py <duration_s>` | base (container) | two-opcode contention (encode_mode every 0.7 s + req_keyframe every 5 s) for the shared-gate legs; gets ~50 commands into the tractor per 5-min leg vs ~3 for kf_inject alone. |
| `inject_watch.sh` | base (pushed and driven by the harness) | the harness's `-InjectCmd` trigger. It waits on the base for rx_smoke's first `published frame_id`, or the PC tells it to start on a tractor FAULT. It runs the injector in `lifetrac-v25:latest` as container `bench_inject` and logs `INJECT_*` markers plus the injector's output to `/tmp/lifetrac_strict/bench_inject.log`. Since 2026-10-19 `start` ignores HUP (`trap '' HUP`) before it forks the watcher, so the adb/sudo session hangup cannot kill it. Not run by hand. |
| `ocd_reset.sh` | tractor (pushed and run by the harness) | the leg-start SWD reset of the tractor's L072 (bench kit 2026-10-19; it used to be inline in the harness): `pkill -9 openocd`, export gpio 8/10/15, drive gpio10 (H7 NRST) high, unexport, then start openocd with `08_boot_user_app.cfg` in the background and print `swd_reset_spawned`. It ignores HUP before it forks (the 2026-10-19 A2c2 miss, "L072 resets at leg start"). Log `/tmp/lifetrac_p0c/ocd_reset.log`: `OCD_LEG=<UTC stamp>`, openocd's output, `OCD_EXIT=<rc>`. The harness pushes it LF-only next to the cfg at every launch. Not run by hand. |
| `test_run_live_radio_monitor.ps1` | PC | offline tests of the harness's parsers: adb `\r\r\n` output, `-InjectOn` specs, the injector log, the openocd reset log (the 10-19 archived logs too), the checked adb push and md5 compare (stub adb files, never the real adb), the clock-offset parser, the leg-end order, the leg-end log-window verdict (docker stop results and the daemons' SIGTERM shutdown lines, 2026-10-22), and `ocd_reset.sh` / `inject_watch.sh start` under a SIGHUP (Git for Windows' dash; skipped without it). No board. `powershell -NoProfile -File bench_tools\test_run_live_radio_monitor.ps1`. |
| `frag_gap_report.py <archive>` | PC | lock-loss episodes from the base's fragment-arrival timeline (needs `-LogFragArrivals 1`). Gaps > 3 s are lock losses; healthy links never exceed ~1 s. |
| `bracket_deltas.py <legs dir> <leg>` | PC | one leg's L072 counter deltas from the rs115 brackets (`leg<X>_{pre,post}_{base,tractor}.txt`), reset-aware and cross-checked against the log counts in `leg<X>_gaps.txt`; the RS-12.15 bracket criteria and STOP rule on FHSS legs only (profile from `leg<X>_params.txt`, or `--params <archive>` / `--profile fhss\|dts`). The kit copy (2026-10-19) of RS-12.15's `scripts/deltas.py`, which stays as flown ("A leg", below). `leg_post.sh` writes it to `leg<X>_deltas.txt`. Exit 1 on a FAIL or UNVERIFIED criterion, 2 on incomplete brackets or an unknown profile; `--selftest`. |
| `rs13_rf_extract.py <archive> [...]` | PC | received level per leg from the base's `rx_rf:` windows (median of the window medians, minimums, n) and crc_dumps; R0 over the 0 dB legs with the −65 to −75 dBm check; checks the FRF read-back of both boards at the pick (an abort condition) and flags 915 MHz by params or read-back. `--leak`, `--r0 --step [--floor]` (tracking rule), `--r0 --closing`, `--lines`. `--path antenna` (2026-10-19) is for legs flown on the antennas: the level per leg and their median, without R0's verdict or pad advice; it refuses `--leak`, `--r0` / `--step` and `--closing`, which need the conducted path. Exit 1 when the windows cannot stand for the leg (stats thread or receive worker crashed, RS-12.17), or with `--leak` on a decoded frame or `NOT SCORED` (no `rx_frames=` line, a crashed stats thread, receive worker or receiver, stats lines that stop before the log does or cover less than the leg's `duration_s` (params must give it as a positive number), or a DTS leg off the pick); 2 on garbled input. |
| `rs13_probe_score.py [--margin PP] <archive> [...]` | PC | base→tractor command delivery (tractor `LoRa cmd: PROBE` seqs that the base sent / base `PROBE TX`). Pools VECTOR against `mono_g4` per profile (params `reg_profile`) with a two-sided Fisher exact test, only over legs whose params show `probe_echo=0`, a VECTOR or `mono_g4` codec (`cam_extra_env` `LIFETRAC_ENCODE_MODE=9` / `6`) and one profile: DTS (`reg_profile=2`) gives the VECTOR_SCENE §8.6 verdict, FHSS (`1`) the same comparison labelled FHSS (since 2026-10-19). Otherwise the pooled verdict is withheld, exit 1. Each leg's `tail:` line lists the base PROBE seqs sent after the tractor's last logged one: they count as undelivered, but at a leg's end they are often end-of-log artefacts, so check them before quoting a miss. Since 2026-10-22: a leg whose params `log_window` is not `after_stop` says so in its block (absent = an older harness that read the logs while the daemons ran; `after_stop_unverified` with its `log_window_detail`), and legs whose `log_window` values differ (absent counts as one value) are not pooled, exit 1. On a `-ReactiveFire 1` leg a `!! base command TX attempts other than PROBE (0x69)` line tallies every other opcode the base sent (0x60 REQ_KEYFRAME, 0x63 ENCODE_MODE, ...): control or profile traffic on the command channel during the probes. On a 2d switch leg the 0x63 commands are expected (`leg2d_switch.sh` sends them). The line withholds nothing. `--margin PP` adds a non-inferiority rule under the Fisher lines, which print unchanged: VECTOR minus `mono_g4` delivery, its one-sided 95 % lower bound (Newcombe hybrid score interval), `NOT WORSE (margin rule)` when that bound is above −PP pp and `not shown at this margin` otherwise, and about how many probes per arm the bound would need at the observed rates. The Fisher rule passes whenever the sample is too small (390/400 against 396/400 gives p = 0.176), so state the margin and the legs per codec in RESULTS before leg 1 (RS13_RESULTS_TEMPLATE.md, "Pass rule"). Legs that share a `log_window` other than `after_stop` still pool, but with `--margin` a `!!` line under the verdict says the pool does not meet a pass rule that needs `log_window=after_stop` on every leg. Exit 2 on an empty or error-only daemon log (a failed `docker logs`), a tractor log showing its `image-tx-worker` crashed, a fatal HostLink open, or RXCONT arming disabled (`-RxcontArm 0`) (it was not receiving), counter skips, or logs from different legs; exit 1 when a leg fired no probe. |
| `rs13_digest_trace.py <leg>_base.jsonl [...]` | PC | the A16 confirmation table and per-row DIGEST trace (encoder n_live against the base's live count) of a VECTOR capture, replayed through this checkout's store (run it at the base image's SHA); flags `!! A16 shape` in not-scored episodes. A gap of two or more frames is a burst, not scored. The join ends at the first epoch start the store accepts (a key frame, or its repeat-once copy when the key was lost inside the capture), and DIGEST mismatch runs split at epoch switches and starts. Refuses (exit 2) a capture with a second meta line (two capture sessions appended into one file), a payload the replay cannot parse or a VECTOR body the store rejects. |
| [`RS13_VECTOR_LEG.md`](RS13_VECTOR_LEG.md) + [`RS13_RESULTS_TEMPLATE.md`](RS13_RESULTS_TEMPLATE.md) | PC | RS-13.1 VECTOR (codec 6) legs: image smoke, camera-only dry run, radio legs with pass criteria, and the `RESULTS.md` skeleton (since 2026-10-22 with an RF-setup record and a pass rule written before leg 1: the margin, its one-sided 95 % bound and the legs per codec). Adds `-CamExtraEnv` to the harness (env for the `camera_svc` container) and archives `camera_service.log`. |
| [`RS13_RANGE_EDGE_LEG.md`](RS13_RANGE_EDGE_LEG.md) | PC | written 2026-10-10 as the next radio session after RS-13.1 (not flown; radios only on an explicit GO per round): conducted SMA-attenuator path, leakage check, 0 dB step with VECTOR / mono_g4 interleaved and `-ReactiveFire 1 -ProbeEcho 0` (the `VECTOR_SCENE.md` §8.6 command-delivery row, the A16-fix confirmation), then the 3 dB walk-down at both profiles, stop rules and its own RESULTS template. **Not planned since 2026-10-19:** the bench has no attenuator path, so the conducted path, the leakage check and the walks stay as reference only. Its interleaved DTS legs `A2c`, `A2a`, `A2c2`, `A2a2` and their scoring still apply, on the antennas; they flew 2026-10-19 (`bench-evidence/RS_13_range_edge_2026-10-19/RESULTS.md`). |
| `../../../tools/vector_dry_run.py capture\|replay\|tractor-log` | both boards (daemon container) / PC | captures `cmd/image_frame` (tractor) or `video/tile_delta` (base) payloads to JSONL, replays them through the base station's own parser + `VectorSceneStore`, and prints per-frame size vs the one-fragment limit, store verdict, orphans, DIGEST, arrival timing and PASS/FAIL checks (`--strict` exits 1). `tractor-log` summarises camera_service's `vector_stats` lines (encoder ms per stage) and totals their cumulative counters on a `counters` line: frames, epoch starts per cause, and the A16 repair's share frames, re-sends, carousel re-sends and DEL repeats. |
| [`RS13_2_DESK_CHECK.md`](RS13_2_DESK_CHECK.md) + [`rs13_2_desk.compose.yml`](rs13_2_desk.compose.yml) | base (compose) + PC browser | RS-13.2 desk check: the rebuilt web_ui on the production compose (mosquitto + web_ui only; the override fences off `lora_bridge` and the bench radio endpoint), the round-4 captures replayed into it, and the §7.5 checklist. No radio. |
| `../../../tools/vector_replay_publish.py <capture>` | base (in `lifetrac-v25:latest`) / PC | publishes a `vector_dry_run.py` capture to a broker at its recorded timing (`--speed`, `--loop`, row windows, `--drop-rows` / `--drop-every` to inject seq-gap losses, `--log`, `--dry-run`). Never retains. |

The probes (`rs115_stats_probe.py`, `rs116_health_probe.py`, `method_g/h_*`) and
the harness (`run_live_radio_monitor.ps1`) live one directory up. Leg reports:
`tools/rs12_leg_report.py <archive> --pre <bracket> --post <bracket>`.
Its `shared-window loss` line (2026-10-19) is the fragment loss without the
log-window artefact. The `loss … [whole logs]` line is the old figure.
It reads `n/a, withheld (…)` when the logs cannot be aligned (an outage of
about one u8 seq wrap, key-frame copies or parity).

## Board facts

* **What is on each chip and board** (custom firmware, OS customizations,
  images, and where each is saved): [BENCH_BOARDS.md](BENCH_BOARDS.md).
  Capture a board read-only with `bash pull_board_state.sh <base|tractor>`.
* **Powering down.** `systemctl poweroff` keeps the tractor off, but the
  base booted again about 4.5 min later (2026-10-04). Remove power to keep the
  base down.
* **Powering up.** The L072 boots into RXCONT (listening).
  `legs/power_up_guard.sh` reads each radio with `radio_state.py` and parks
  any that is listening with `radio_park.py` ([legs/README.md](legs/README.md)).
* **PC toolchain.** arm-none-eabi-gcc 12.2.1 (Arm GNU Toolchain
  12.2.MPACBTI-Rel1) gives byte-identical L072 builds. Also: WinLibs
  `mingw32-make` (PowerShell only), arduino-cli with
  `arduino:mbed_portenta` 4.5.0, `py -3`. The PC has no Docker; images are
  built on the base X8. Installation steps: [PC_SETUP.md](PC_SETUP.md).
* Original bench (the `bench.env.example` values; yours are in `bench.env`):
  base `192.168.1.117` via ssh key `~/.ssh/lifetrac_base_ed25519`; tractor
  ONLY via `adb -s 2E2C1209DABC240B` (WiFi stays off); base adb
  `2D0A1209DABC240B`. Password `fio` (LmP default) → `echo fio | sudo -S -p ''`;
  after provisioning, the sudoers drop-in makes `sudo -n` work without it.
* USB is for programming and debug logs only. Never run the harness or
  `mingw32-make` from bash — PowerShell only.
* `/tmp` is tmpfs on both boards: **every reboot (every flash) wipes
  `/tmp/lifetrac_p0c` (flash tooling + staged bins) AND `/tmp/lifetrac_strict`
  (probes, injectors, park script).** Re-push both after any reboot.
  `/home/fio` persists (flash wrapper, logs).
* **A reboot is not the only way staging disappears.** `systemd-tmpfiles`
  ages `/tmp` at **5 days** (`/usr/lib/tmpfiles.d/tmp.conf`: `q /tmp 1777
  root root 5d`), so files rot out with the boards up. Observed 2026-09-15:
  19 h uptime, no reboot, yet `/tmp/lifetrac_p0c` held only the bin pushed
  the night before — the flash scripts had aged out. A flash attempted then
  fails as `run_flash_bench.sh: line 5: 1: image` (rc 127) and touches no
  radio. **Check the staging before trusting a flash:** `flash_l072.sh`
  re-stages the pipeline and `run_flash_bench.sh` now refuses to run on an
  incomplete `/tmp/lifetrac_p0c` (preflight); for legs, `ls /tmp/lifetrac_strict |
  wc -l` ≥ 19, 23 on a fresh full staging (`legs/stage_boards.sh` re-stages it).
* The L072 boots into RXCONT (`sx1276_rx_arm()`), so a flash brings the
  receiver up — a flash IS a radio-on event. A probe HostLink connect also
  auto-wakes it.
* The HC-02 health check (`../x8_max_carrier_healthcheck.sh`) touches the
  radio UART too: its L072 GET probe sets `/dev/ttymxc3` to 19200 8E1 and
  writes two bytes (`00 FF`) to it. Run it only with the UART free, and on a
  GO like any probe ([BENCH_SETUP.md](BENCH_SETUP.md) §7).
* The tractor's camera unit/container come back on reboot and steal the
  radio UART: `systemctl stop lifetrac-camera.service; docker stop tractor-camera`
  before any probe. Confirm `fuser /dev/ttymxc3` is empty.
* Container images: base `lifetrac-v25:latest`; tractor
  `hub.foundries.io/arduino/arduino-ootb-python-devel:738bc44` for probes,
  `lifetrac-tractor-x8:latest` for the camera feed (it ships `/usr/bin/ffmpeg`;
  the host has none — a wiped `/tmp/ffmpeg` does not matter).
* Bench firmware is byte-reproducible.
  - Both boards run `e083ae46` since 2026-10-18: the RS-12.15
    reverse-delivery bench build of `13f20aa1` (#152, merged as `main`
    `3c8ced60` with the L072 source unchanged; production twin `148de24e`).
    Flash record and a copy of the image:
    `bench-evidence/RS_12_15_reverse_delivery_2026-10-18/firmware_flash/`.
  - The previous bench build, `0c1bb0a9` (RS-12.15 v2, on both boards
    2026-09-15 → 2026-10-18), comes from `main` before #152 (re-checked at
    `f6e4b463` on 2026-10-14). It is the rollback image.
  - The old RS-12.10 build `e8ad8424` comes from `main` `3a0cb524` (`e02ab86e`).
  - Copies of these and of the other surviving bench builds (`5a160e4a`,
    `8c112e6f`) are in `bench-evidence/RS_13_vector_scene_2026-09-26/firmware/`,
    along with the inventory of every flashed build.
  - Stage `e8ad8424` for A/Bs against the pre-clock-authority firmware.

## Prep — every session, and again after every flash

1. `git archive` the helper tooling to `/tmp/lifetrac_strict` on both boards
   (probes, `lora_proto.py`, and everything in this directory except the
   `/home/fio` files). `adb push` needs `MSYS_NO_PATHCONV=1` and a
   Windows-style `C:/...` source path. `legs/stage_boards.sh` does this
   step from `bench.env` ([legs/README.md](legs/README.md)). Before it,
   point `EVIDENCE_DIR` in `bench.env` at a new dated folder for this
   session. Since 2026-10-22 the leg scripts refuse an `EVIDENCE_DIR` that
   already holds files tracked by git, a committed evidence record
   (`stage_boards.sh` and `leg_prep.sh` exit 7 before any board action;
   `leg_post.sh` only warns, so the folder never stops its park; it still
   stops before the park at exit 3 while a leg daemon runs), and warn when the last
   date in its path is not today. A stale `bench.env` would otherwise append
   a new session's files to the old record (the flight `bench.env` still
   pointed at `RS_13_range_edge_2026-10-19/legs` on 2026-10-22). For a
   deliberate append, prefix every script of the leg with
   `BENCH_EVIDENCE_APPEND=1` (an environment setting only; a line in
   `bench.env` is ignored with a WARN).
2. Flash staging: push the helper `.sh`/`.cfg`/`.py` pipeline files plus the
   bins to `/tmp/lifetrac_p0c` on both boards **LF-clean** (`tr -d '\r'`);
   a CRLF script fails silently as `1: image` from the wrapper. `scp` to the
   base; never `git archive | ssh sudo -S tar` (sudo -S eats the stream).
3. Stop the tractor camera unit + container; check both UARTs are free.
4. `rs116_health_probe.py` on both: `STATS-OK`, `radio_state=4`, and the
   counter families the flashed build should carry (v2: `fhss_dec_*`,
   `clk_demotion_*`, `tx_first_anchor`, `tx_stream_streak_max`).
5. `clear_retained.py` on the base broker (and `clear_retained_host.py` on the
   PC broker if it is up).
6. Pre-brackets: `rs115_stats_probe.py` on both boards → `leg<X>_pre_*.txt`.
7. Camera legs: camera aimed at the PC screen; put moving content on it —
   the railroad reference video `https://www.youtube.com/watch?v=B1yUQwpNhJA`
   in a **normal Firefox window** of a separate profile (`--no-remote -profile
   <dir>`, `user.js`: `media.autoplay.default=0`, `media.volume_scale="0.0"`),
   then the player's own fullscreen (`f` / the expand button), which the
   operator leaves with Esc. **Never `firefox --kiosk`**: it has no close
   control and locked the operator out of the PC for the RS-13.1 legs
   (2026-09-27). The `youtube-nocookie.com/embed/…` URL now fails with
   "Video player configuration error / Error 153"; two pre-roll ads (~40 s)
   play first; the window drops behind any app the operator clicks, so bring
   it to the front and verify **before every leg** with frames from
   `/dev/video1` (`legs/scene_check.sh`: ≥ 8 % of pixels change over 10 s,
   frame saved). A static scene makes every keyframe fit one
   fragment and does not exercise the break.
8. DTS (profile 2) legs: run a same-day receive-only channel spot-check with
   the tractor parked (`../channel_survey_sniff.py`, which
   `legs/stage_boards.sh` stages, on the base; or `../hunt_sniff.ps1`;
   then `tools/survey_compare.py`). Put the pick in `bench.env`
   (`DTS_CARRIER_HZ`, `DTS_CARRIER_DATE`) and pass `-ForceFrfHz <the pick>`
   on every profile-2 leg; the harness refuses a profile-2 leg without it.
   `-ForceFrfHz 0` pins 915.000 MHz, where the RS-11.6
   external emitter sends ~25 ms bursts at −43 to −45 dBm every ~7.08 s. All
   RS-13.1 DTS legs flew there by accident and their losses fold on its
   period (RS-13.1 A19). The band changes from day to day, so re-check rather
   than reuse a pick.

## A leg

```
.\run_live_radio_monitor.ps1 -TxAdbSerial <TRACTOR_SERIAL> -RxAdbSerial <BASE_SERIAL> -HostIp <PC_HOST> `
   -TxFeed camera|local -RegProfile 1|2 [-ForceFrfHz <DTS_CARRIER_HZ>: profile 2 only, prep 8] -DurationS 300 `
   [-SynthFps 2 -SynthBudgetB 3000] -KfRequestDisable 0 -ProbeEcho 0 `
   -NoParkLast 0 -LogFragArrivals 1 -IdleDrainQuietS 1.5 -CmdStreamMinGapS 1.0 `
   [-InjectCmd "<injector.py> <args>" [-InjectOn first_frame|fault:<code>[:<sub>|:final]]] -Archive
```
The `<...>` values come from `bench.env`; without them the harness falls
back to the original bench's serials and PC address. `-ForceFrfHz <the pick>`
is **required with `-RegProfile 2`** (prep 8): the harness refuses a
profile-2 leg without it, and `-AllowDefaultCarrier` is only for a deliberate
915 MHz control. Leave it out with `-RegProfile 1`. For an RS-13 leg,
`legs/leg_prep.sh` writes the complete line ([legs/README.md](legs/README.md)),
`-StopDrainS` included since 2026-10-22 ("Leg end", below).

Command injectors: add `-InjectCmd "kf_inject.py 15 20"` or
`-InjectCmd "dual_inject.py 250"` to the harness line. With the default
`-InjectOn first_frame`, the harness starts the injector on the base the
moment the base rx daemon logs its first `published frame_id`. It reads that
line on the base from rx_smoke's own log (`inject_watch.sh`), not from the
console tail that started leg W (2026-10-18) 2 min 41 s late.
`-InjectOn fault:0x0D:final` starts it at the tractor's final
RX_SCAN_FAILED (leg X); `fault:<code>` and `fault:<code>:<sub>` also work
(the PC polls the tractor's tx_smoke log about every 2 s). At leg end the
harness stops the injector if it is still running, prints
`INJECT <status> ... trigger: ...`, and archives `inject.log` (trigger,
injector output, exit) and `inject_trigger.txt` (PC-side timeline). Not yet
flown.

After `archived to ...`, `legs/leg_post.sh <leg>` does the rest. It takes
the post-brackets on both boards, writes `rs12_leg_report.py` and
`frag_gap_report.py` whole into `legs/`, and copies the archive's
`tractor_ocd_reset.log`, `inject.log` and `inject_trigger.txt` there as
`leg<X>_*`. It then runs the RS-13 scorers and `bracket_deltas.py`, and parks
both radios last ([legs/README.md](legs/README.md)). The archive's
`deploy_md5.txt` stays in the archive.

**Leg end** (bench kit 2026-10-19, checked since 2026-10-22; not yet
flown). The harness stops the frame source (`camera_svc` or `synth_pub` with
`docker stop -t 1`, or the PC publisher), then the injector, waits out the
rest of `-StopDrainS` (default 2 s, 0–60) counted from the feed stop, stops
tx_smoke, waits 0.5 s and stops rx_smoke. It reads both board clocks, and
only then the daemon logs (`docker logs` of tx_smoke, rx_smoke and
`camera_svc` / `synth_pub`; `synth_pub` is removed after its log is read).
Up to the 10-19 legs it read both logs while the daemons ran (TX first, then
RX) and stopped them afterwards, so the TX log ended first, the RX log next,
and both before the post brackets: on 2026-10-19 the base brackets counted
one send more than `base sends on air` on every leg, and
`rs12_leg_report.py` printed `received > sent` on A2c and A2a
(`bench-evidence/RS_13_range_edge_2026-10-19/RESULTS.md`, Anomalies).

**Log window** (2026-10-22, not yet flown). Each leg-end `docker stop` keeps
its output and exit status (they went to `Out-Null` before). params.txt says
`log_window=after_stop` only when both daemon stops worked (docker printed
the container name and exited 0) and both daemon logs hold their SIGTERM
shutdown line (`image_tx_daemon: signal received; shutting down`, and the
same for `image_rx_daemon`). Otherwise it says `log_window=after_stop_unverified`,
with the reasons on one line in `log_window_detail=`. `feed_stop=` records
the frame source's own stop (a detail, not part of the verdict) and
`stop_drain_s=` the drain. LEG CHECKS prints the verdict as `LOG_WINDOW`:

- green, `LOG_WINDOW after_stop (feed: ...; <s> s drain, ...; both stops ok,
  both daemon logs hold their SIGTERM shutdown line)`;
- red, `LOG_WINDOW after_stop_unverified (tx_smoke stop: ...; feed: ...)`: a
  daemon stop was not confirmed (docker's error and exit status, or no exit
  status at all when the adb call failed). That daemon may still have run
  when its log was read (the old artefact), and it may still be running:
  `leg_post.sh` stops before the brackets and the park while a leg daemon
  runs (exit 3); `legs/power_up_guard.sh --stop-leg-daemons` stops it and
  parks, then re-run `leg_post.sh`;
- yellow, `LOG_WINDOW after_stop_unverified (rx_smoke log has no shutdown
  line; feed: ...)`: both stops worked, but a daemon log has no shutdown
  line. That daemon may have died before the stop (look for a traceback in
  its final log, printed above LEG CHECKS), or its log did not come back
  whole: the log ends early, and the figures from it cover less than the
  leg. This is not the read-while-running artefact.

Under an unverified LOG_WINDOW a yellow hint line names the likely cause.
`rs12_leg_report.py` drops its log-window note only on
`log_window=after_stop`. On `after_stop_unverified` it prints `!! LOG WINDOW
UNVERIFIED (log_window_detail: ...)`, worded by the reason, after the loss
lines (whole logs, shared window, capture seq gaps); a `received > sent`
line ends with a pointer to it, and FIRMWARE DROP gets a note for both
causes. The note comes after the loss lines so that the capture seq-gap
line (the loss of record) keeps its place in `leg_post.sh`'s 8-line console
excerpt; the note itself can fall below that excerpt. Read the coloured
LOG_WINDOW line, the `log_window=` value on the report's first line, and
the report file. An
archive without `log_window` (every leg flown so far) keeps the old note.

**Stop drain** (2026-10-22, not yet flown). `legs/leg_prep.sh` writes
`-StopDrainS`: 4 s on FHSS (`image_bw250`, profile 1), 2 s (the harness
default) on DTS (`image_bw500`, profile 2); `--stop-drain S` (0 < S ≤ 30)
overrides it. 2 s is short for a saturated multi-fragment FHSS leg: a train
still on the air at the stop is decoded but missing from `sent`, the last
reactive PROBE lands on a stopped daemon, and `bracket_deltas.py` marks the
FHSS criteria UNVERIFIED, all of which favour the codec with the shorter
queue. Fly both codecs of a round (its `mono_g4` and VECTOR legs) with the
same value; the prep prints a NOTE when it is not the default.

**Deploy check** (bench kit 2026-10-19, not yet flown). Every adb push is checked (adb's
`N file(s) pushed` line and its exit status), and every pushed file is then
md5-compared on the board with the PC copy (`md5sum`, a few adb calls per
board). A failed push or an md5 mismatch stops the leg at `[DEPLOY]` with
`adb push FAILED ...` or `md5 MISMATCH ...` and `No daemon was started.`
(nothing was transmitted). Usual cause: a root-owned file or folder in
`/tmp/lifetrac_strict` or `/tmp/lifetrac_p0c` on that board; remove it with
sudo and re-run the leg. A pushed file that `md5sum` does not report is a
warning only. Directories go up from a clean temp copy without `__pycache__`
/ `*.pyc`. The console prints `[DEPLOY <serial>] N pushes OK, md5 N/M match`;
the archive gets `deploy_md5.txt` (`<serial> ok|MISMATCH|missing <remote
path> pc=<md5> board=<md5|->`), and params.txt `tractor_deploy_md5=` and
`base_deploy_md5=` (the same summary, plus the reset files and the
injector). The 25 `RemoteException` / `NativeCommandError` blocks per
transcript should be gone (first leg to confirm): adb's stderr is merged inside the `cmd` string.

**Board clocks** (bench kit 2026-10-19). After the daemons stop the harness
reads each board's `date +%s.%N` (read only: no clock is set) and prints
`CLOCKS board minus PC: tractor ...; base ...` in LEG CHECKS. params.txt gets
`tractor_clock_offset_s=` and `base_clock_offset_s=` (seconds, board minus
the PC's UTC epoch at the adb call's midpoint; error at most
`*_clock_rtt_s`/2, plus 1 s when the board's `date` has no `%N`),
`tractor_clock_rtt_s=`, `base_clock_rtt_s=` and `clock_offset_rule=`;
`unknown` when the read failed. The tractor clock ran about 19 days behind
on 2026-10-19 (its logs read 2026-09-30). For cross-board timing use these
offsets, or matched events.

**First flown leg on the new kit** (bench kit 2026-10-19 with the
2026-10-22 fixes, PR #161). Confirm:

- base brackets == `base sends on air`, and received ≤ sent;
- `TRACTOR_L072_RESET landed`, with the openocd banner and `OCD_EXIT=0`;
- `deploy_md5` all ok, and no `RemoteException` block in the transcript;
- both clock offsets numeric;
- `LOG_WINDOW after_stop` (green): both stops confirmed and both daemon logs
  hold their SIGTERM shutdown line;
- on a `-ReactiveFire 1` leg, every base `command TX opcode=` line is 0x69
  (no `!! base command TX attempts other than PROBE` line in
  `leg<X>_probe_score.txt`), except the 0x63 commands of a 2d switch leg;
- the leg-end park (`leg_post.sh`), then `legs/power_up_guard.sh
  --check-only` at least ~2 min after the daemons stopped: a park readback
  right after the stop is not authoritative (below);
- and see whether FIRMWARE DROP falls to about the `rx_urc_lost` level (0–2
  on the 10-18/10-19 legs): a prediction, not a measurement.

Counter deltas: the harness fires an SWD reset of the tractor's L072 at leg
start. At leg end it judges that reset from its openocd log:
`TRACTOR_L072_RESET landed|failed|unknown (...)` in the `=== LEG CHECKS ===`
block, `tractor_l072_reset=` in params.txt, and the log itself archived as
`tractor_ocd_reset.log`. `landed` means openocd read PF4 (the L072 NRST net)
back low, then high; the log must also start with this leg's
`OCD_LEG=<UTC stamp>` tag (repeated as `tractor_ocd_tag=` in params.txt). An
earlier leg's log, left behind when this leg's spawn never ran (adb
`error: closed`, sudo failure), therefore reads `failed (... not this leg's
...)` and never `landed`; the harness also warns at leg start when the spawn
does not print `swd_reset_spawned`. An `OCD_EXIT` of 128 or more is a
signal: 129 = SIGHUP, the adb/sudo session hangup reaching nohup before
nohup ignored HUP; if the log has no `Open On-Chip Debugger` banner, openocd
never started. Since the bench kit of 2026-10-19 the reset runs from
`bench_tools/ocd_reset.sh`, which the harness pushes (LF-only) to
`/tmp/lifetrac_p0c` and which ignores HUP before it forks, so a 129 without
the openocd banner would point to a cause other than the session hangup
(openocd catches SIGHUP itself once started, so a 129 after its banner can
still be a late hangup; the verdict then reads `killed by SIGHUP after
openocd started`). The brackets stay the final word. The reset did not land on
the first leg after a flash reboot (2026-10-18 leg V, RESULTS Observation 3;
legs O, Q, R, S, T and U of 09-14/15 too: their FAULT-URC uptimes put the
L072 boot at the flash reboot, 120–631 s before the TX daemon opened). It did
land on 09-14 legs P and Q2, the two legs there flown without a flash before
them: the L072 booted 3.6 s and 3.4 s before the TX daemon opened. On Q2,
post − pre would read 250 TXs for a leg of 1136
(`bench-evidence/RS_12_15_clock_authority_2026-09-14/ERRATA.md` §1). When it
matters: a FAULT URC's last u32 (payload bytes 20–23) is the L072's uptime
in ms, so log time minus uptime gives the boot time. The cause of the
first-leg misses is unknown because that log was never kept (candidates in
"L072 resets at leg start", below). The base's L072 is pulsed on gpio163
but not reset (from 2026-08-02 on, and on 2026-10-18): the harness prints
`BASE_L072_RESET unverified` and archives the pulse output as
`base_gpio163_output=`. Base counters accumulate across legs: score them
post − pre. Score with `bench_tools/bracket_deltas.py <legs dir> <leg>`
(the kit copy of `RS_12_15_reverse_delivery_2026-10-18/scripts/deltas.py`,
which stays as flown; `leg_post.sh` writes it to `leg<X>_deltas.txt`). It
takes the post value after a reset and post minus pre otherwise. It counts a
reset when a counter or high-water mark fell, or when there were more TXs
than parsed host frames; `radio_state` never counts. It reads the leg's
profile from `leg<X>_params.txt` (or `--params <archive>`; `--profile
fhss|dts` when there are no params, and it must agree with params that give
one). On FHSS (reg_profile=1) it applies the RS-12.15 criteria. The tractor's
`tx_first_anchor` is judged by the leg's delta: each profile-1 activation
anchors once, so a tractor whose reset did not land reads post 2 on its
second FHSS leg and passes; delta 2 or more trips STOP. The base's
`tx_stream_streak_max` is a high-water mark since the base L072 booted: at 8
or more it fails only if it rose during the leg, and if it stood there before
the leg it is UNVERIFIED. An FHSS leg that sent no command (brackets and logs
0/0, e.g. the RS-13 FHSS walk, `-ReactiveFire 0` with no `-InjectCmd`) has
command delivery N/A, not FAIL. Brackets at 0/0 that the logs do not confirm
are UNVERIFIED. On DTS (reg_profile=2), and on the bench-only fixed 915 MHz
profile (0), every RS-12.15 criterion is FHSS-only, so it prints the deltas,
the reset evidence and the cross-check and exits 0 (DTS command delivery
comes from `rs13_probe_score.py`). It cross-checks delivery against the log
counts in `leg<X>_gaps.txt`. If they disagree, or the file or its counts line
is missing, an FHSS criterion that would pass becomes UNVERIFIED and the STOP
rule UNDETERMINED (exit 1), so keep the gaps file next to the brackets. It
refuses incomplete brackets or an unknown profile (exit 2).
Then `radio_park.py` both → `PARK_OK` with both readbacks `0x80`. Do it
after the post-brackets and reports, i.e. a couple of minutes after the
daemons stopped, so the firmware's scan SM has already failed out and has
nothing left to re-arm; a `PARK_TRANSIENT` means it was still walking —
wait and repeat. Before claiming the bench is off for the night, re-check
with the read-only `radio_state.py` (it writes nothing), or
`legs/power_up_guard.sh --check-only`, at least ~2 min after the daemons
stopped: a park readback taken right after the stop is not authoritative,
because the scan SM may still re-arm the receiver. Empirically the
post-leg park has held for hours this way (both boards `0x80` an hour after
leg V), but that is because of the timing above, not because the write is
authoritative. RXCONT is receive-only — the TX path only fires on a host
request, and there is none with the daemons down.

### L072 resets at leg start

Analysis of 2026-10-18 (no board access). The harness reports both resets
in its `=== LEG CHECKS ===` block (above); nothing here changes how either
board is reset. (The bench kit of 2026-10-19 moved the tractor reset into
the pushed `ocd_reset.sh`, same steps and cfgs: candidate (c), below.)

**Base: the gpio163 pulse does not reset the L072.**
- gpio163 is not an i.MX pin. It is line 3 of the x8h7 GPIO chip
  (gpiochip5), so a write reaches the L072's NRST only through a working
  x8h7 bridge and H7 firmware that drives PF4 for that line.
- It has not reset the base L072 since at least 2026-08-02 (RS-11.4
  RESULTS: the counters survived about 8 launches), nor on 2026-10-18. The
  2026-05-25 sweep neither confirms nor rules it out: 0 boot URCs in 9
  pulse-only attempts, but the host slept 1.5 s after each pulse before
  opening the port, so a boot URC would probably have been missed.
- openocd driving PF4 over SWD did reset the base L072 (RS-11.4 §10 leg E),
  so the PF4-to-NRST wiring is intact. The fault lies between gpio163 and
  PF4: the driver, the bridge, or the line mapping in the base's newer x8h7
  firmware (0.0.5 from 2025-03, against the tractor's 0.0.2 from 2022-10).
- Next steps, in order:
  1. Read-only, no radio: on the base, list `/sys/class/gpio/gpiochip*`
     (label, base, ngpio), `/sys/kernel/debug/gpio` if it is mounted (is
     163 x8h7 line 3? does a kernel consumer hold it?), and
     `cat /sys/kernel/x8h7_firmware/version`.
  2. On a GO, because an NRST boots the L072 into RXCONT: read
     `base_gpio163_output=` after the next leg (failed sysfs writes show
     there), or pulse once with rs115 brackets around it (counters at 0
     after = landed).
  3. If it does not land: give the base reset to the daemon's own RESET_REQ
     (`LIFETRAC_SKIP_RESET_REQ=0` on rx_smoke, gpio163 pulse removed; the
     2026-05-25 double-reset race applies only when the NRST really fires)
     and confirm with brackets. Fallback: the tractor's openocd path. It
     worked on the base on 2026-08-02, but it halts the base H7 and stalls
     x8h7 until a reboot; never re-insmod x8h7 on the base's 6.1 kernel.

**Tractor: the SWD reset misses on the first leg after a flash reboot**
(2026-10-18 leg V; legs O, Q, R, S, T and U of 09-14/15). It landed on
09-14 legs P and Q2, the two legs there flown without a flash before them.
Why those first-leg misses happen is still unknown: none of those legs kept
the openocd log (leg V's stayed on the tractor's tmpfs and was overwritten).
The harness has archived it as `tractor_ocd_reset.log` since the 2026-10-18
bench-kit change.
**2026-10-19, the first archived logs**
(`bench-evidence/RS_13_range_edge_2026-10-19/legs/leg*_tractor_ocd_reset.log`):
the reset landed on A2c, A2a and A2a2. On A2c2 it missed with candidate (c)'s signature:
`OCD_EXIT=129` and no openocd banner. A2c2 was the third leg of the day and
the tractor had been up since the 2026-10-18 flash, so it did not need a
fresh boot.
Candidates; only (c) has been seen in an archived reset log (once, A2c2):
- (a) A SWD attach failure: the DPIDR / sysfs-mmap race (FLASH_RUNBOOK and
  README_P0C TT-1).
- (b) An openocd hang (RS-5.2), cleared by the next leg's `pkill -9 openocd`.
- (c) A SIGHUP race (first proposed as a cold-page-cache effect; A2c2 hit it
  with openocd run minutes earlier). The adb/sudo session hangup
  reaches the forked `nohup` before it ignores HUP: the subshell's `trap :`
  is reset in its children, and the new wrapper (bench kit 2026-10-18, flown
  since 2026-10-19) starts nohup one fork later than the code that flew legs
  O–X. It shows as `OCD_EXIT=129` with no openocd banner (verdict `killed by
  SIGHUP ... openocd never started`). Seen on 2026-10-19 A2c2, on a board up
  since the day before, so it does not need a fresh boot. Fixed in the bench
  kit of 2026-10-19, not yet flown: the harness pushes
  `bench_tools/ocd_reset.sh` and runs it under sudo. The script sets
  `trap '' HUP` before it forks, and an ignored disposition is inherited
  through fork and exec, so the subshell and nohup start with HUP already
  ignored, and so does openocd until its own startup (server_preinit, after
  its banner) installs a SIGHUP handler. That closes the window A2c2 died
  in, from the fork to nohup's own SIG_IGN. A hangup that reaches openocd
  after its banner can still end it with 129; the verdict then says `killed
  by SIGHUP after openocd started` rather than `openocd never started`.
  `inject_watch.sh start` sets `trap '' HUP` the same way before its
  `nohup`. The offline test (`bench_tools/test_run_live_radio_monitor.ps1`,
  Git for Windows dash) reproduces A2c2's signature with the old inline form
  when the spawn's process group gets a SIGHUP, and shows ocd_reset.sh land
  under the same hangup. The first flown legs should read `landed`, with the
  banner and `OCD_EXIT=0`.
- (d) `08_boot_user_app.cfg` does `init; halt` and never resumes the H7 (it
  ends with `shutdown`). Only the first leg after a boot halts an H7 that is
  serving a live x8h7 bridge; later legs attach to an H7 the previous leg
  left halted. The flash pipeline runs `prep_bridge.sh` before openocd for
  this reason (in-flight SPI requests time out and wedge the x8h7 driver). It
  shows as an attach or halt error, or a hang, in that leg's log.

`tractor_ocd_reset.log` from the first leg after the next flash reboot tells
whether those misses are (c) too, or (a), (b) or (d). A log has now shown
(c), so its fix did not wait for that leg (`ocd_reset.sh`, above). If a
first-leg miss still shows with the fix in, its log shows (a), (b) or (d) as
described, or a 129 without the banner that would point to a cause other
than the session hangup.

## Bench web UI on the base (optional)

The operator console (`base_station/web_ui.py`) is not part of the harness
loop. Run it on the base only when a leg needs it, for example the RS-1.4
Auto selector (leg V) or a look at the tiles. While it runs it adds base
commands to the control plane (TILE_STALE scans, profile switches), so say in
RESULTS that it ran. This recipe comes from the F10/F11 sessions and leg V.

1. **Push** `web_ui.py`, `lora_proto.py`, `image_pipeline/` and `web/` from
   the checkout to `/tmp/lifetrac_strict` on the base. The harness pushes
   `lora_proto.py` and `image_pipeline/` at every launch, but after a reboot
   nothing is there until you push it. If `scp` cannot overwrite
   `image_pipeline/`, remove its root-owned `__pycache__` first.

   ```bash
   K="-i $HOME/.ssh/lifetrac_base_ed25519"; B=fio@192.168.1.117
   S=LifeTrac-v25/DESIGN-CONTROLLER/base_station
   ssh $K $B "sudo rm -rf /tmp/lifetrac_strict/image_pipeline/__pycache__"
   scp $K -r $S/web_ui.py $S/lora_proto.py $S/image_pipeline $S/web $B:/tmp/lifetrac_strict/
   ```

2. **Run** it in `lifetrac-v25:latest` with host networking: uvicorn on port
   **8090**, away from the production compose's 8080,
   MQTT to the base broker at `127.0.0.1`, and PIN **2525**. That PIN is for
   the bench only and is never a field PIN; production reads its PIN from
   `secrets/lifetrac_pin`.

   ```bash
   ssh $K $B "sudo docker rm -f bench_webui 2>/dev/null; sudo docker run -d --name bench_webui --network=host \
     -v /tmp/lifetrac_strict:/work -w /work -e PYTHONPATH=/work:/work/paho \
     -e LIFETRAC_MQTT_HOST=127.0.0.1 -e LIFETRAC_PIN=2525 \
     --entrypoint python3 lifetrac-v25:latest -m uvicorn web_ui:app --host 0.0.0.0 --port 8090"
   ```

   Browse to `http://<base IP>:8090`. Some modules, such as `audit_log`,
   `build_config` and `settings_store`, are imported lazily, and web_ui only
   degrades when they are missing. If a feature needs one, append
   `:/app/base_station` to `PYTHONPATH` so the image's deployed copy fills
   the gap.

3. **Record what it did** with an MQTT tap. web_ui has no root logger, so its
   info lines never reach `docker logs`:

   ```bash
   ssh $K $B "sudo docker exec design-controller-mosquitto-1 mosquitto_sub -h 127.0.0.1 -F '%U %t %p' \
     -t lifetrac/v25/control/radio_profile -t 'lifetrac/v25/status/radio_profile/#' -t lifetrac/v25/video/link_stats" | tee webui_tap.txt
   ```

4. **Auto selector.** Arm Auto only **after** the daemons are up. With no
   daemon running, Auto pins FHSS within 60 s (the stale-link rule). At the
   end of the session, set a concrete profile (e.g. 2) before you stop. A
   retained `auto` otherwise re-commands the tractor at the next start. Then
   run `clear_retained.py`.

5. **Stop it** at the end of the session:
   `ssh $K $B "sudo docker rm -f bench_webui"`. Port 8090 then closes.

## Traps that cost time this campaign

* Bash-tool heredocs mangle backslashes — write patch scripts to files.
* **Misspelled harness parameters were ignored without a warning.**
  `run_live_radio_monitor.ps1` used to put an unknown name into `$args`, so
  `-DurationSeconds 300` ran the 30 s default of `-DurationS`. The harness
  now refuses unknown names at launch. Even so, read the archive's
  `params.txt` for the values that actually ran (`duration_s`,
  `force_frf_hz`, `reg_profile`, ...) before you quote a leg.
* `git add <directory>` sweeps the untracked bench binary
  (`build/firmware_bench_diag.bin`) into the commit; add firmware by path.
* The committed `build/firmware.bin` is the PRODUCTION image (no diag flag);
  the bench boards need `firmware_bench_diag.bin` (`mingw32-make bench`).
  Rebuild and commit the production image with every firmware PR.
* The synthetic feed cannot reproduce the FHSS lock-loss break: it never
  pauses TX > 2 s and delivers almost no reverse-path commands. Only the
  camera keyframe workload does.
* Evidence discipline: no verification claim without SHA + raw transcript
  under `bench-evidence/`; corrections go in the document, not over it.
