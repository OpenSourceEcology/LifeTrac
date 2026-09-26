# Bench runbook — radio legs on the LifeTrac v25 bench

Written 2026-09-14 at the end of the RS-12.14 / RS-12.15 campaign so the
next session starts from a checklist instead of memory. Everything that
keys a radio needs the operator's GO; radios are parked (LoRa SLEEP,
`0x80` readback) whenever a session ends.

## What is here

| file | runs on | purpose |
|---|---|---|
| `run_flash_bench.sh` + `stamp.py` + `kmsg_log.py` | board, `/home/fio` (persistent) | instrumented wrapper around `full_flash_pipeline.sh`: monotonic-stamped pipeline log + kernel log, fsync'd so a PMIC power-cycle cannot lose them. `REVIVE_MODE=reboot bash /home/fio/run_flash_bench.sh <bin>`; success = `Verify OK` + `flash_rc=0` in `/home/fio/pipeline_stamped.log` (~2.5 min incl. reboot). |
| `radio_park.py` | board, daemon container | STANDBY → SLEEP, reads RegOpMode back, waits 1.5 s, reads again; prints `PARK_OK` only if still `0x80` (`PARK_TRANSIENT`, exit 5, if the firmware re-armed it). **The readback is NOT a persistent-off guarantee** (PR #125 review): the firmware's scan walker, γ-1 retune and slot follower all call `sx1276_rx_arm()` and will bring the modem back to RXCONT (receive-only) while any of them still has work. The park holds once the scan SM has exhausted into FAILED — up to ~2 min after the daemons stop (500 ms/channel, hard FAIL at 30 s per attempt, 3 retries). Park after that, and confirm later with `radio_state.py`. Firmware park command = TODO RS-12.21. |
| `radio_state.py` | board, daemon container | read-only RegOpMode decode — answers "is it off?" without touching it. |
| `clear_retained.py` / `clear_retained_host.py <host>` | base (container) / PC | clear the retained control topics (`encode_mode_override`, `req_keyframe`, `radio_profile`) on the base broker / on another broker. A stale retained pin has flipped the camera and re-commanded a profile mid-leg; clear before EVERY leg. |
| `kf_inject.py <period_s> <count>` | base (container) | REQ_KEYFRAME storm; the unacked retries at 0.4/0.8 s make the 2–3-command bursts that LOCK the tractor's scan machine (the RS-12.15 trigger). |
| `dual_inject.py <duration_s>` | base (container) | two-opcode contention (encode_mode every 0.7 s + req_keyframe every 5 s) for the shared-gate legs; gets ~50 commands into the tractor per 5-min leg vs ~3 for kf_inject alone. |
| `frag_gap_report.py <archive>` | PC | lock-loss episodes from the base's fragment-arrival timeline (needs `-LogFragArrivals 1`). Gaps > 3 s are lock losses; healthy links never exceed ~1 s. |
| [`RS13_VECTOR_LEG.md`](RS13_VECTOR_LEG.md) + [`RS13_RESULTS_TEMPLATE.md`](RS13_RESULTS_TEMPLATE.md) | PC | RS-13.1 VECTOR (codec 6) legs: image smoke, camera-only dry run, radio legs with pass criteria, and the `RESULTS.md` skeleton. Adds `-CamExtraEnv` to the harness (env for the `camera_svc` container) and archives `camera_service.log`. |
| `../../../tools/vector_dry_run.py capture\|replay\|tractor-log` | both boards (daemon container) / PC | captures `cmd/image_frame` (tractor) or `video/tile_delta` (base) payloads to JSONL, replays them through the base station's own parser + `VectorSceneStore`, and prints per-frame size vs the one-fragment limit, store verdict, orphans, DIGEST, arrival timing and PASS/FAIL checks (`--strict` exits 1). `tractor-log` summarises camera_service's `vector_stats` lines (encoder ms per stage). |

The probes (`rs115_stats_probe.py`, `rs116_health_probe.py`, `method_g/h_*`) and
the harness (`run_live_radio_monitor.ps1`) live one directory up. Leg reports:
`tools/rs12_leg_report.py <archive> --pre <bracket> --post <bracket>`.

## Board facts

* Base `192.168.1.117` via ssh key `~/.ssh/lifetrac_base_ed25519`; tractor
  ONLY via `adb -s 2E2C1209DABC240B` (WiFi stays off); base adb
  `2D0A1209DABC240B`. Password `fio` → `echo fio | sudo -S -p ''`.
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
  radio. **Count the staging before trusting a flash:**
  `ls /tmp/lifetrac_p0c | wc -l` must be 11, and `ls /tmp/lifetrac_strict |
  wc -l` ≥ 19.
* The L072 boots into RXCONT (`sx1276_rx_arm()`), so a flash brings the
  receiver up — a flash IS a radio-on event. A probe HostLink connect also
  auto-wakes it.
* The tractor's camera unit/container come back on reboot and steal the
  radio UART: `systemctl stop lifetrac-camera.service; docker stop tractor-camera`
  before any probe. Confirm `fuser /dev/ttymxc3` is empty.
* Container images: base `lifetrac-v25:latest`; tractor
  `hub.foundries.io/arduino/arduino-ootb-python-devel:738bc44` for probes,
  `lifetrac-tractor-x8:latest` for the camera feed (it ships `/usr/bin/ffmpeg`;
  the host has none — a wiped `/tmp/ffmpeg` does not matter).
* Old RS-12.10 firmware is byte-reproducible: `mingw32-make bench` on `main`
  → `e8ad8424`. Keep it staged for A/Bs.

## Prep — every session, and again after every flash

1. `git archive` the helper tooling to `/tmp/lifetrac_strict` on both boards
   (probes, `lora_proto.py`, and everything in this directory except the
   `/home/fio` files). `adb push` needs `MSYS_NO_PATHCONV=1` and a
   Windows-style `C:/...` source path.
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
   `Start-Process firefox --kiosk "<video URL with autoplay=1&mute=1>"` —
   and verify with two frames from `/dev/video1` 1 s apart that differ
   (`ffmpeg -f v4l2 -input_format mjpeg -video_size 1920x1080 -i /dev/video1
   -frames:v 1` inside the tractor image). A static scene makes every
   keyframe fit one fragment and does not exercise the break.

## A leg

```
.\run_live_radio_monitor.ps1 -TxFeed camera|local -RegProfile 1|2 -DurationS 300 `
   [-SynthFps 2 -SynthBudgetB 3000] -KfRequestDisable 0 -ProbeEcho 0 `
   -NoParkLast 0 -LogFragArrivals 1 -IdleDrainQuietS 1.5 -CmdStreamMinGapS 1.0 -Archive
```
On the first `published frame_id` line start the injector on the base
(`kf_inject.py 15 20` or `dual_inject.py 250`). After `archived to ...`:
post-brackets both boards, `rs12_leg_report.py`, `frag_gap_report.py`,
copy brackets/injector/report transcripts into the evidence `legs/` dir.
Then `radio_park.py` both → `PARK_OK` with both readbacks `0x80`. Do it
after the post-brackets and reports, i.e. a couple of minutes after the
daemons stopped, so the firmware's scan SM has already failed out and has
nothing left to re-arm; a `PARK_TRANSIENT` means it was still walking —
wait and repeat. Before claiming the bench is off for the night, re-check
with the read-only `radio_state.py` (it writes nothing). Empirically the
post-leg park has held for hours this way (both boards `0x80` an hour after
leg V), but that is because of the timing above, not because the write is
authoritative. RXCONT is receive-only — the TX path only fires on a host
request, and there is none with the daemons down.

## Traps that cost time this campaign

* Bash-tool heredocs mangle backslashes — write patch scripts to files.
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
