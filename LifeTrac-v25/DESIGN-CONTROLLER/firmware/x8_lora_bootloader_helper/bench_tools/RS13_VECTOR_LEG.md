# RS-13.1 — VECTOR (codec 6) on the bench: image smoke, camera dry run, first radio legs

*Procedure, 2026-09-26 (corrected the same day against the code, see the
RS-13.1 RESULTS Anomalies; the lessons of the RS-13.1 radio rounds of
2026-09-27 and 2026-10-03 folded in on 2026-10-03, cited by their anomaly ids
in the [RS-13.1 RESULTS](../../../bench-evidence/RS_13_vector_scene_2026-09-26/RESULTS.md)).
Software under test: PR #135 (`rs13-vector-phase1` — the Phase 1 codec, encoder,
store and renderer) and the encoder sync fix PR #138
(`rs13-vector-encoder-sync-fix`, A1–A5). Design: `VECTOR_SCENE.md` (PR #129).
Campaign context: [BENCH_RUNBOOK.md](BENCH_RUNBOOK.md) — its prep, brackets,
park and evidence discipline apply unchanged; this file adds only what VECTOR
needs. Results go into `bench-evidence/RS_13_vector_scene_<date>/RESULTS.md`,
started from [RS13_RESULTS_TEMPLATE.md](RS13_RESULTS_TEMPLATE.md). The RS-13.1
session scripts that did each step below are kept in
[`bench-evidence/RS_13_vector_scene_2026-09-26/scripts/`](../../../bench-evidence/RS_13_vector_scene_2026-09-26/scripts/)
(they carry that session's paths).*
Parameterised copies that run from any checkout (settings in `bench.env`) are in [`legs/`](legs/README.md).

**No firmware change.** VECTOR rides the strict image path as ordinary
one-fragment 0xFE trains (a 0xFD copies train only if `LIFETRAC_KEYFRAME_COPIES`
is raised on the tx daemon, which RS-13.1 does not do), so both L072 boards keep
their current build. Record it in the RESULTS header all the same.

## What RS-13.1 proves, and what it does not

Proves, in order:

1. the tractor image built from the branch imports numpy, OpenCV and the
   encoder (step 0);
2. the encoder turns real camera frames into codec-6 frames that fit one
   fragment, at the camera rate, inside the camera period (step 1);
3. those frames cross the radio at profile 2 and profile 1 and rebuild a scene
   at the base with the store's checks clean, and the mode can be entered and
   left over the air with the tile quality dial untouched (step 2).

Does not prove: the `VECTOR_SCENE.md` §8 field criteria beyond a same-day
`mono_g4` control leg; the V1–V3 degradation ladder (no policy is built — every
frame is V0 at the requested detail); the self-model and the Vector Lab; the
browser rendering (web_ui is not in the harness loop — a desk check with the
production compose follows as RS-13.2); the range edge.

## Numbers to hold in mind

| | FHSS profile 1 (`image_bw250`) | DTS profile 2 (`image_bw500`) |
|---|---|---|
| One-fragment payload, delta frame (0xFE header, 4 B) | 203 B | 243 B |
| One-fragment payload, epoch start (0xFD copies header, 5 B — §3.1 F−1) | 202 B | 242 B |
| VS body F after the 6-byte TileDeltaFrame header | 197 / 196 B | 237 / 236 B |
| Fragment airtime at the 170 ms cap | 169.1 ms | 99.9 ms |

The budget reaches `camera_service` from `image_tx_daemon` (retained
`lifetrac/v25/tractor/link_budget`, `n_fragments=1`) on the strict path; the
standalone camera run of step 1 sets it by env instead (`LIFETRAC_FRAGMENT_BUDGET=1`,
`LIFETRAC_FRAGMENT_PROFILE`). [`tools/vector_dry_run.py`](../../../tools/vector_dry_run.py)
derives the same limits from `lora_proto` and flags any frame over them.

The tool's checks (`--strict` exits 1 when one fails):

| check | meaning |
|---|---|
| `parse_ok` | every payload parses as a TileDeltaFrame |
| `all_vector` | every frame is codec 6 — a codec-4 stream here means the tractor clamped VECTOR to Y_ONLY (no numpy/OpenCV in the image, step 0) |
| `one_fragment` | delta ≤ 203 / 243 B, epoch start ≤ 202 / 242 B |
| `store_clean` | the store rejected nothing (`frames_bad = 0`, reasons listed otherwise) |
| `no_orphans` | no record referenced an unknown shape or contradicted its stored state (CONFIRM tag) |
| `digest` | no DIGEST mismatch, and the scene does not end `digest_ok False` |
| `epoch_seen` | at least one epoch start applied |
| `first_apply` | the first vector frame received was applied (no wait for a later epoch start) |
| `applied_ratio` | ≥ 90 % of vector frames applied (`--min-applied-ratio`) |
| `min_frames` | at least `--min-frames` vector frames received |

## Every session, every run — the RS-13.1 lessons

Each row is spelled out in the step it belongs to; this is the checklist.

| when | do | why |
|---|---|---|
| session start | count the staging on both boards and re-stage `/tmp/lifetrac_strict` if it is older than 5 days or a board rebooted; md5-check the pushed tree against the working tree (Step 0.3) | `systemd-tmpfiles` ages `/tmp` at 5 days; both trees had aged out before round 3 |
| session start | moving content up: the railroad video in a **normal** browser window with the player's own fullscreen — never a kiosk (Scene) | the kiosk locked the operator out of the PC for the 2026-09-27 legs |
| before **every** camera run (each step-1 pass, each leg) | bring the video window to the front and run the scene check: ≥ 8 % of pixels changed over 10 s, frame saved, abort otherwise (Scene) | the window drops behind any app the operator clicks; 2026-09-27's legs could only infer the scene |
| before **every** step-1 pass | recreate the tractor's `bench_mqtt` and confirm 0 retained messages (Step 1) | a radio leg leaves a retained `encode_mode_override` / `link_budget` that silently overrides the env (A12) |
| after every leg | `rs12_leg_report.py … --capture legs/leg<X>_base.jsonl`; the loss of record is its `capture seq gaps` line (Step 2) | the report's counter line read a stale `rx_frames` on every round-3 leg (A14) |
| after every leg | park last, after the post-brackets and reports; `PARK_TRANSIENT` → wait ~60 s and park again until `PARK_OK` (Step 2) | after 2b_yt the base still read `PARK_TRANSIENT` (`opmode_after_settle 0x85`, its scan walking) 2 min after the daemons stopped; the second attempt parked |
| FHSS legs at 1 fps | read P2/P3 from lock, or fly at 2 fps (Step 2, P2/P3) | acquisition is deterministic (first frame at seq 58 on both 1-fps flights, A7); no FHSS time authority at 1 fps (A11) |
| P3 | compare single legs only interleaved with the control, or against a margin stated before the flight | the same control lost 6.5 % and 0.3 % a week apart (A15) |
| P6 | the bound includes the encoder's own time; stamp and publish in **one** board call | A9; a two-call stamp once ran 16.6 s early (A13) |

## Step 0 — image and staging (tractor)

1. Build the tractor image from the branch for **linux/arm64** and load it on
   the tractor. [README-DEPLOY.md](../../tractor_x8/README-DEPLOY.md) step 1
   builds it; its steps 2–3 are the production deploy and are **not** run on the
   bench: the `lifetrac-camera` unit's compose file
   (`firmware/tractor_x8/docker-compose.yml`) maps `/dev/ttymxc3` — the L072
   radio UART on this bench — as its M7 port and takes `/dev/video1`, and it has
   no `build:`, so restarting it deploys nothing. The tractor is offline (Wi-Fi
   off) and cannot pip-install the wheels itself. Two routes, both ending in
   `docker load` on the tractor:

   - **(a) natively on the base X8** (aarch64, has internet; used 2026-09-26,
     when the PC had no Docker): `git archive` the branch's
     `LifeTrac-v25/DESIGN-CONTROLLER/firmware/tractor_x8` → `scp` to
     `/home/fio/…` on the base → `docker build -t lifetrac-tractor-x8:latest .`
     there → `docker save … | gzip > /home/fio/…tgz`;
   - **(b) on a PC with Docker:** `docker buildx build --platform linux/arm64 -t lifetrac-tractor-x8:latest --load .`,
     then `docker save`.

   Copy the tarball to the tractor's disk (`/home/fio`, not the 1 GB tmpfs
   `/tmp`), keep the old image (`docker tag lifetrac-tractor-x8:latest lifetrac-tractor-x8:pre-rs13`),
   then `docker load -i <tgz>` — `-i`, not stdin, because `sudo -S` eats stdin.
   Check the md5 at every hop. Record the image id, the source SHA and the
   resolved numpy / OpenCV versions from the build log. The image needs numpy and
   `opencv-python-headless` (both in `requirements.txt`) and `libglib2.0-0`
   (Dockerfile) — an image built before PR #135 lacks them and fails silently
   (see Traps).
2. Smoke test on the tractor:

   ```sh
   mkdir -p /tmp/lifetrac_strict/legs
   echo fio | sudo -S -p '' docker run --rm -w /app --entrypoint python3 lifetrac-tractor-x8:latest -c \
     "import numpy, cv2; import x8_image_pipeline.encode_vector as e; print('numpy', numpy.__version__, 'cv2', cv2.__version__, 'encoder', e.VectorEncoder.__name__)" \
     2>&1 | tee /tmp/lifetrac_strict/legs/step0_image_smoke.txt
   ```

   Expected: `numpy 2.<n> cv2 <m>.<n>.<n> encoder VectorEncoder`.
   `requirements.txt` floors only `numpy>=1.26.0` and
   `opencv-python-headless>=4.9.0`; CI pins 4.14.0.94
   (`base_station/requirements-dev.txt`), but a tractor build resolves the
   newest cp311 aarch64 wheel — on 2026-09-26 that was **OpenCV 5.0.0.93** with
   numpy 2.4.6. The failure is an `ImportError`, a missing module, or no
   `encoder VectorEncoder` tail — not a version mismatch. An `ImportError`
   naming `libgthread-2.0.so.0` means the image predates the Dockerfile's
   `libglib2.0-0` line — rebuild. When the OpenCV major differs from CI's, also
   run the encoder suites inside the image (the same commit's `DESIGN-CONTROLLER`
   tree mounted at `/r/DESIGN-CONTROLLER`); `OK` with no `(skipped=N)` means
   the cv2-gated classes ran:

   ```sh
   docker run --rm -v <tree>:/r -w /r/DESIGN-CONTROLLER/base_station --entrypoint python3 \
     lifetrac-tractor-x8:latest -m unittest tests.test_vector_encoder tests.test_vector_interop tests.test_vs1_codec_parity_sil -v
   ```

3. Stage the bench tool. The harness pushes `camera_service.py`,
   `image_tx_daemon.py`, `image_rx_daemon.py`, `lora_proto.py`,
   `image_pipeline/`, `x8_image_pipeline/` and `paho/` to `/tmp/lifetrac_strict`
   on both boards at every launch; push the tool next to them on both boards
   (the base gets its other files from the harness; the tractor needs
   `lora_proto.py` and `image_pipeline/` for step 1, so push those too if no
   harness leg has run yet on this boot):

   ```powershell
   # run from inside the checkout (the harness derives its root the same way)
   $dc = "$(git rev-parse --show-toplevel)/LifeTrac-v25/DESIGN-CONTROLLER"
   foreach ($s in "2E2C1209DABC240B", "2D0A1209DABC240B") {
     adb -s $s shell "mkdir -p /tmp/lifetrac_strict/legs"
     adb -s $s push "$dc/tools/vector_dry_run.py" /tmp/lifetrac_strict/
   }
   adb -s 2E2C1209DABC240B push "$dc/base_station/lora_proto.py" /tmp/lifetrac_strict/
   adb -s 2E2C1209DABC240B push "$dc/base_station/image_pipeline" /tmp/lifetrac_strict/
   adb -s 2E2C1209DABC240B push "$dc/firmware/x8_lora_bootloader_helper/bench_mqtt.conf" /tmp/lifetrac_strict/
   ```

   (From Git Bash instead of PowerShell, set `MSYS_NO_PATHCONV=1` and use a
   Windows-style `C:/...` source path, as BENCH_RUNBOOK prep 1 says.)

   **Staging rots.** `/tmp` is tmpfs (a reboot wipes it) and `systemd-tmpfiles`
   also ages it at **5 days** with the boards up (BENCH_RUNBOOK, Board facts).
   Before round 3 (2026-10-03) both boards' `/tmp/lifetrac_strict` trees had
   aged out after the 2026-09-27 round and were re-pushed. At the start of
   every session count the staging on both boards
   (`ls /tmp/lifetrac_strict | wc -l` ≥ 19 for the helper tooling, plus
   `vector_dry_run.py`, `bench_mqtt.conf` and, for a branch encoder, the
   pushed tree) and re-stage everything — helper tooling (BENCH_RUNBOOK
   prep 1), this step's files and any branch tree — if it is short, if a board
   rebooted, or if the last push is 5 days old (RS-13.1
   `scripts/stage_boards.sh`). md5-check every pushed file against the working
   tree and record the md5s; the harness re-pushes its own files at every
   launch, but nothing re-pushes the rest.

   The tool runs inside each board's daemon image (paho lives there; the host
   python has none), with `/tmp/lifetrac_strict` mounted as `/work`:

   ```sh
   # tractor
   echo fio | sudo -S -p '' docker run --rm --network=host -v /tmp/lifetrac_strict:/work -w /work \
     -e PYTHONPATH=/work:/work/paho --entrypoint python3 lifetrac-tractor-x8:latest /work/vector_dry_run.py ...
   # base
   echo fio | sudo -S -p '' docker run --rm --network=host -v /tmp/lifetrac_strict:/work -w /work \
     -e PYTHONPATH=/work:/work/paho --entrypoint python3 lifetrac-v25:latest /work/vector_dry_run.py ...
   ```

## Scene — moving content, and the check before every run

**Moving content** is the railroad reference video
`https://www.youtube.com/watch?v=B1yUQwpNhJA` (the one RS-12.11–12.15 used;
sky, clouds, trees, mountains and track; round 3 ran on it), set up as
BENCH_RUNBOOK prep 7 says: a **normal** Firefox window of a separate profile
(`--no-remote -profile <dir>`, `user.js` with `media.autoplay.default=0` and
`media.volume_scale="0.0"`), then the **player's own fullscreen** (`f` or the
expand button), which the operator leaves at any time with Esc; the window
keeps its own close button (RS-13.1 `scripts/youtube_window.ps1 -Open /
-Front / -Close`). **Never `firefox --kiosk`** and nothing else that takes the
whole screen: the 2026-09-27 kiosk had no way out and locked the operator out
of the PC for the whole of round 2 (RESULTS, Evidence limitations). The
`youtube-nocookie.com/embed/…` URL the earlier campaigns used now fails with
"Video player configuration error / Error 153" — use the watch URL. Two
pre-roll ads (~40–45 s) play first: start the video at least a minute before
the first check, and expect the check to fail on an ad card. The landscape
still of step 1 is the only other scene; RS-13.1 showed it in a kiosk
(`scripts/landscape_still.html`), which the same rule now forbids — open it
in a normal, maximised window of the separate profile and frame the camera
on the picture.

**Scene check before every camera run** — every step-1 pass and every leg,
not once per session: the video window drops behind any app the operator
clicks, and an unchecked leg can only infer what the camera saw (round 2's
legs, RESULTS Evidence limitations). Bring the window to the front, then grab
frames from the tractor's `/dev/video1` for ~10 s with nothing else holding
the node (`camera_svc` not running yet — the check runs in the prep, before
the capture and the harness; it is camera-only and touches no radio), and
record the share of pixels whose largest channel changed by more than 25
levels over 1.9 / 5 / 10 s (RS-13.1 `scripts/scene_check.sh`). Save the last
frame as `legs/<run>_scene.jpg` and the numbers as `legs/<run>_scene.txt`.

| run | criterion | RS-13.1 reference |
|---|---|---|
| moving (step-1 bw250 moving, every leg) | ≥ **8 %** over 10 s; abort the run otherwise (paused, behind another window, on an ad card, or the camera off target — look at the saved frame) | 38–57 % on the video, round 3 |
| landscape (step-1 bw250 / bw500 landscape) | record it; the still is expected to change only with AE (≈ 2 %) — the frame proves the still is up and framed | 1.9–2.0 % |

## Step 1 — camera-only dry run (tractor, no radio)

Prerequisites: the production camera unit and container stopped (BENCH_RUNBOOK
prep 3: `systemctl stop lifetrac-camera`, `docker stop tractor-camera`) so the
UVC node is free, and a **fresh** broker on the tractor at `127.0.0.1:1883`.
Recreate `bench_mqtt` exactly as [run_live_radio_monitor.ps1](../run_live_radio_monitor.ps1)
does at every launch, and never reuse a running one: a broker left by a harness
leg holds retained `lifetrac/v25/tractor/link_budget` and
`lifetrac/v25/tractor/encode_mode_override`, which override
`LIFETRAC_FRAGMENT_BUDGET` / `LIFETRAC_FRAGMENT_PROFILE` and
`LIFETRAC_ENCODE_MODE` in `camera_service` (observed 2026-09-26: a broker up
10 days still held the profile-2 budget and a retained switch to mono_g4). The
conf has no persistence, so a recreate wipes them. `bench_mqtt.conf` must be in
`/tmp/lifetrac_strict` (step 0.3). Recreate it before **every** pass, not once
per session — any radio leg flown since the last pass re-fills it — and
confirm it is empty before starting the capture:

```sh
echo fio | sudo -S -p '' docker rm -f bench_mqtt 2>/dev/null
echo fio | sudo -S -p '' docker run -d --name bench_mqtt --network=host \
  -v /tmp/lifetrac_strict/bench_mqtt.conf:/mosquitto/config/mosquitto.conf eclipse-mosquitto:2
sleep 2
# must print 0
echo fio | sudo -S -p '' docker exec bench_mqtt timeout 2 mosquitto_sub -h 127.0.0.1 -t 'lifetrac/#' -v --retained-only | wc -l
```

(A12: on 2026-10-03 this step was missed; the broker, up 6 days, still held
round 2's retained `encode_mode_override {"mode": 6}` and a bw500
`link_budget`, so the first step-1 attempt on the video ran mono_g4 and was
voided as `legs/*_youtube_fix_ABORTED1.*`. RS-13.1 `scripts/step1_pass_fix.sh`
now does the recreate and the check itself. Keep the before/after retained
listing as `legs/step1_bench_mqtt_reset.txt`.)

Every pass is **capture first, then camera**, so the first captured frame is
`camera_svc`'s boot epoch start: a capture that joins a running epoch misses its
defines and fails `first_apply` / `no_orphans` for no real reason. Name every
file with its profile and scene — `--out` appends, so a reused name merges two
passes.

1. Start the capture and wait for its
   `connected to 127.0.0.1:1883 … subscribing lifetrac/v25/cmd/image_frame` line
   (2 fps → 240 frames in 120 s; `--duration 130` absorbs start-up,
   `--min-frames 200` tolerates it):

   ```sh
   echo fio | sudo -S -p '' docker run --rm --network=host -v /tmp/lifetrac_strict:/work -w /work \
     -e PYTHONPATH=/work:/work/paho --entrypoint python3 lifetrac-tractor-x8:latest \
     /work/vector_dry_run.py capture --topic lifetrac/v25/cmd/image_frame --profile image_bw250 \
     --duration 130 --min-frames 200 --out /work/legs/step1_tractor_bw250_moving.jsonl \
     --json /work/legs/step1_tractor_bw250_moving.json --strict \
     | tee /tmp/lifetrac_strict/legs/step1_tractor_bw250_moving.txt
   ```

2. Then, from a second shell, launch `camera_service` from the image in VECTOR
   at boot with a one-fragment budget — the harness's camera line, including
   its `/work` mount and `PYTHONPATH`, plus the four VECTOR/budget variables:

   ```sh
   echo fio | sudo -S -p '' docker rm -f camera_svc 2>/dev/null
   echo fio | sudo -S -p '' docker run -d --name camera_svc --network=host --device=/dev/video1 \
     -v /tmp/lifetrac_strict:/work -w /app -e PYTHONPATH=/work:/work/paho --entrypoint python3 \
     -e LIFETRAC_MQTT_HOST=127.0.0.1 -e LIFETRAC_CAMERA_SOURCE=v4l2 -e LIFETRAC_CAMERA_DEVICE=/dev/video1 \
     -e LIFETRAC_CAMERA_FPS=2 -e LIFETRAC_USE_LORA_BRIDGE=1 \
     -e LIFETRAC_ENCODE_MODE=9 -e LIFETRAC_VECTOR_DETAIL=80 \
     -e LIFETRAC_FRAGMENT_BUDGET=1 -e LIFETRAC_FRAGMENT_PROFILE=image_bw250 \
     lifetrac-tractor-x8:latest -u camera_service.py
   ```

   The `/work` mount is not optional: `camera_service` sizes the budget through
   `x8_image_pipeline/fragment.py`, which imports `lora_proto`, and the image
   carries no `lora_proto.py` (its build context is `firmware/tractor_x8`).
   Without it the budget silently falls back to `n_fragments * 40` = **40 B**
   (`camera_service.py`, the "conservative fallback"). `/app` stays first on
   `sys.path` (it is the script's directory), so the image's own
   `camera_service.py` and `x8_image_pipeline` run; only `lora_proto` comes
   from `/work`. While the capture runs, `docker logs camera_svc` must show
   `camera_service: byte_budget=203 B/frame (LIFETRAC_FRAGMENT_BUDGET, phy=image_bw250)`
   near the top (243 for bw500), `vector_stats` lines every 2 s once frames
   flow, and **no** `link_budget: ->` line and no `[lora_cmd]` encode-mode
   line — either means a retained message on the broker overrode the env
   (recreate it). `byte_budget=40` means `lora_proto` did not import: the pass
   is void. (The harness's step 2 runs the checkout's `camera_service.py` and
   `x8_image_pipeline` from `/work` on the same image; build the image from the
   same commit so P8 compares like with like.)

   **A branch encoder without an image rebuild** (how the RS-13.1 re-run and
   round 3 tested the fix): push the branch's `camera_service.py`,
   `x8_image_pipeline/` (with `vs1_codec.py`), `image_pipeline/` and
   `vector_dry_run.py` to `/tmp/lifetrac_strict` on both boards, md5-checked
   against the working tree (RS-13.1 `scripts/push_fix_to_board.sh`), and run
   the camera line above with `-w /work` and `/work/camera_service.py` instead
   of `-w /app … camera_service.py`, so `/work/x8_image_pipeline` shadows the
   image's copy (RS-13.1 `scripts/step1_pass_fix.sh`, `_fix` file suffix).
   Confirm the encoder's module path inside the container
   (`/work/x8_image_pipeline/encode_vector.py`) and keep that line with the
   pass. The harness launches `/work/camera_service.py` for step 2, so the legs
   then fly the same tree, and P8 compares like with like.
3. After the capture ends, the encoder timing, then remove the container:

   ```sh
   echo fio | sudo -S -p '' docker logs camera_svc > /tmp/lifetrac_strict/legs/step1_camera_service_bw250_moving.log 2>&1
   echo fio | sudo -S -p '' docker run --rm -v /tmp/lifetrac_strict:/work -w /work -e PYTHONPATH=/work \
     --entrypoint python3 lifetrac-tractor-x8:latest /work/vector_dry_run.py tractor-log \
     /work/legs/step1_camera_service_bw250_moving.log | tee /tmp/lifetrac_strict/legs/step1_tractor_log_bw250_moving.txt
   echo fio | sudo -S -p '' docker rm -f camera_svc
   ```

4. Three passes, each repeating 1–3 with its own `_<bw>_<scene>` suffix, a
   scene check (Scene), a recreated broker and a new capture before each new
   `camera_svc`: **bw250 moving** (the railroad video, Scene / BENCH_RUNBOOK
   prep 7, the same content as the legs and the RS-12 tile legs),
   **bw250 landscape** (a static landscape on the screen — horizon, sky, ground,
   a tree or two — so L0–L2 have something to find) and **bw500 landscape**
   (`LIFETRAC_FRAGMENT_PROFILE=image_bw500`, `--profile image_bw500`, the 243 B
   budget). The harness recreates `camera_svc` for step 2.

Pass (step 1) — record every row in the RESULTS step-1 table:

| Check | Criterion | Source |
|---|---|---|
| dry-run checks | all ten PASS — `RESULT: PASS` (under `--strict` the tool exits 1 exactly when it prints `RESULT: FAIL`; the `\| tee` hides the exit code), at both profiles | `step1_tractor_*.txt` |
| wire size | max in the `wire bytes (vector)` line ≤ 203 / 243, epoch starts ≤ 202 / 242 (the `one_fragment` check) | same |
| rate | fps mean in the `arrival:` line ≥ 1.8 at `LIFETRAC_CAMERA_FPS=2` (≥ 90 % of the camera rate, §8) | same |
| encoder time | `ms_total` p95 ≤ 350 ms at 2 fps (period 500 ms; leaves capture and publish headroom); p50 recorded | `tractor-log` |
| epoch starts | ≥ 1 applied (`epoch starts: N received, M applied`); no run of more than 2 consecutive `K=1` rows with the same `ep=` in the capture (a failed epoch-start attempt is re-sent with the key bit set). `tractor-log`'s `lines with a pending epoch start` sees only range-2 (safety refresh) starts, because `camera_service` logs `int(bool(range))` — a stuck range-0 start reads 0 there (RESULTS Anomalies) | both |
| scene | landscape passes: horizon `abs` or `resid` (both mean an anchor is held; `none` = NO_HORIZON or a pending hand-over), L1 ≥ 1, L2 ≥ 1 in the final `scene:` line (informational — a blank wall legitimately gives L1 = 0) | summary |

If only the encoder time fails, run the radio legs at `-SynthFps 1` and record
the miss: it is a Phase 4 optimisation item, not a blocker for the radio
evidence. At 1 fps a 300 s leg carries at most ~300 frames, so use
`--min-frames 250` on the base capture and read P2 as ≥ 0.9 frames/s (90 % of
1 fps); P8 still compares against the step-1 figure measured at 2 fps — say so
in RESULTS. (RS-13.1: p95 449–478 ms on the i.MX8 against 350 ms, A4, so both
radio rounds flew at 1 fps.)

1 fps costs the FHSS leg (2b) twice, and neither is a VECTOR defect:

- **Acquisition (A7).** After the harness resets the base's L072 the follower
  re-acquires the tractor's hop sequence by scanning; at one transmission per
  five 200 ms slots that meeting point is deterministic — the first frame was
  heard at **seq 58** on both 1-fps FHSS flights (2026-09-27 and 2026-10-03),
  57 s into the leg, so P2 (0.82 fps over 300 s) and a whole-leg P3 miss by the
  letter while the link after lock delivered 247/247 and 247/248. Read 2b's P2
  and P3 **from lock** (P2, P3 below), or fly it at 2 fps.
- **No FHSS time authority (A11).** The tractor becomes time authority only
  after `SX1276_FHSS_AUTHORITY_MIN_STREAK` = 8 own transmissions each
  **strictly** `< SX1276_FHSS_AUTHORITY_STREAK_GAP_MS` = 1000 ms apart
  (`firmware/murata_l072/include/sx1276_fhss_authority.h`); the 1 fps cadence
  sits on 1000 ms (tractor gaps p10/p50/p90 993/999/1005 ms) and the 2b
  post-brackets read `tx_stream_streak_max` 2 and 3 (RS-12.15 legs: 1246). The
  base still locks and holds by following the tractor, but the RS-12.15
  clock-authority protection is off: do not put base commands on a 1-fps FHSS
  leg and expect RS-12.15 behaviour. Record the tractor's post-bracket
  `tx_stream_streak_max` on every FHSS leg (≥ 8 = authority reached; RS-13.1
  `scripts/streak_check.py` finds the first 8-streak in `tx_daemon.log`).

2 fps on air removes both (the RS-12.15 legs locked in seconds and streaked
to 1246) and needs A4 (encoder time) first. Do **not** relax the gap test in
firmware instead: the strict `< 1000 ms` chain is the discriminator that keeps
command traffic (admitted at ≥ 1.0 s by the RS-12.14 gate) from chaining into
authority (`sx1276_fhss_authority.h:38-42`); a 1500 ms gap let the base's
paced commands hold authority in review. Only an explicit stream
discriminator (image-stream transmissions alone count toward the streak)
could fix A11 at 1 fps.

## Step 2 — radio legs

Prep exactly as BENCH_RUNBOOK "Prep", in this order (RS-13.1
`scripts/leg_prep.sh`): the **scene check** for this leg (Scene; abort the leg
below 8 %), production camera stopped and no `/dev/ttymxc3` holder on either
board, both health probes (`rs116_health_probe.py` → keep the **whole** output
as `legs/leg<X>_health_{base,tractor}.txt`; round 2 kept only the last 12
lines and lost the `STATS-OK` / `radio_state=` lines),
[`clear_retained.py`](clear_retained.py) on the base broker,
pre-brackets [`rs115_stats_probe.py`](../rs115_stats_probe.py) on both boards
→ `legs/leg<X>_pre_*.txt`, then the base-side capture below. Same camera
scene for every leg (the railroad video, Scene; the landscape is a step-1
matter).

After each leg (RS-13.1 `scripts/leg_post.sh`): wait for the base capture to
finish, post-brackets on both boards, then the reports — the leg report
**with the base capture**, so it prints the loss from the TileDeltaFrame
sequence gaps (A14). From `DESIGN-CONTROLLER`, with `<legs>` the evidence
directory's `legs/` and the base capture already pulled into it:

```powershell
py -3 tools/rs12_leg_report.py <archive> --pre <legs>/leg<X>_pre_base.txt `
   --post <legs>/leg<X>_post_base.txt --capture <legs>/leg<X>_base.jsonl
py -3 firmware/x8_lora_bootloader_helper/bench_tools/frag_gap_report.py <archive>
```

(Hand the Python tools and `adb pull` Windows-style `C:/…` paths, not MSYS
`/c/…` ones: the first version of round 2's collection script handed them
MSYS paths and 2a's files were re-collected by hand, A6.)

The report's `capture seq gaps` line is the loss of record. Its counter-based
`loss` line reads the base's **last** periodic `stats:` line, written up to
10 s before the log ends: on round 3's 2a / 2b / 2c / 2d it printed 4.3 / 22.2 /
3.3 / 5.0 % against 1.3 / 0.4 from lock (19.0 counting from seq 1) / 0.3 /
2.0 % from the sequence gaps (A14; `legs/leg<X>_yt_report_seq.txt` in the
RS-13.1 record). The report now floors that counter with the per-frame
`published frame_id` / `frag_arrival` events and says so in a
`(rx rx_frames counter N is stale; …)` note, but only the capture sees the
frames that never reached the base at all. The seq-gap line counts within each
codec run (each encoder path numbers its own frames, so a 2d switch is not
loss) and reports any frames numbered before the first one heard on a separate
`first frame heard at seq N` line (the FHSS acquisition head, A7).

**Park last** — [`radio_park.py`](radio_park.py) on both boards → `PARK_OK`
with both readbacks `0x80`, after the post-brackets and reports (BENCH_RUNBOOK,
A leg). `PARK_TRANSIENT` (`opmode_after_settle` ≠ `0x80`) means the base's
channel scan was still walking: wait ~60 s and park again until `PARK_OK`, and
record both attempts (2b_yt: `PARK_TRANSIENT`, `0x85`, two minutes after the
daemons stopped; the second attempt parked, `legs/leg2b_yt_park.txt`). At the
end of the session re-check both boards with the read-only `radio_state.py`.

**Base-side capture, every leg.** From a second shell on the base board,
started before the harness prints its RX-daemon launch line (it subscribes to
what `rx_smoke` publishes on the base broker; `--duration 480` covers the 300 s
leg plus the harness's start-up however early the capture starts — idle
capture time affects neither the checks nor fps_mean). Profile `image_bw500` for profile-2 legs, `image_bw250` for
profile 1:

```sh
echo fio | sudo -S -p '' docker run --rm --network=host -v /tmp/lifetrac_strict:/work -w /work \
  -e PYTHONPATH=/work:/work/paho --entrypoint python3 lifetrac-v25:latest \
  /work/vector_dry_run.py capture --topic lifetrac/v25/video/tile_delta --profile image_bw500 \
  --duration 480 --min-frames 500 --out /work/legs/leg2a_base.jsonl --json /work/legs/leg2a_base.json --strict \
  | tee /tmp/lifetrac_strict/legs/leg2a_base.txt
```

**Legs** (300 s each, `-Archive`; run the harness from
`firmware/x8_lora_bootloader_helper/`). `-KfRequestDisable 1` keeps
REQ_KEYFRAME off the air: it drops the base's decode-error keyframe pokes and
every `cmd/req_keyframe` relay at the subscription (reassembly-timeout
requests are already off by default, `LIFETRAC_KF_ON_REASM_TIMEOUT=0`); in
VECTOR a REQ_KEYFRAME only forces an epoch start, which would blur the epoch
count. `-TxBatch 0` on every leg: the harness default (1) packs frames that
queue behind a stall into one multi-fragment train logged under the first
frame's seq, which P4 would read as K = 2 although no frame exceeded one
fragment (2c keeps it too, so the control stays like-for-like). `-ProbeEcho 0`
as in the RS-3.3 legs.

**Pin the DTS carrier on every profile-2 leg.** Before the first DTS leg of the
day, run a receive-only channel spot-check with the tractor parked
([`channel_survey_sniff.py`](../channel_survey_sniff.py) or
[`hunt_sniff.ps1`](../hunt_sniff.ps1), then `tools/survey_compare.py`). Add
`-ForceFrfHz <the pick>` to every 2a/2c/2d line below, and record the pick in
RESULTS. Without it, `-ForceFrfHz 0` pins the profile-2 default of
915.000 MHz. That is the channel of the RS-11.6 external emitter (about 25 ms
bursts at −43 to −45 dBm on a hard 7.07–7.09 s grid). All nine RS-13.1 DTS legs
flew there, and their losses fold on its period (RESULTS A19). The RS-12 legs
used `-ForceFrfHz 927500000` after a same-day spot-check; the band changes
from day to day, so re-check rather than reuse a pick.

- **2a — DTS (profile 2), VECTOR at boot:**

  ```powershell
  .\run_live_radio_monitor.ps1 -TxFeed camera -RegProfile 2 -DurationS 300 -SynthFps 2 `
     -KfRequestDisable 1 -ProbeEcho 0 -LogFragArrivals 1 -TxBatch 0 `
     -CamExtraEnv "-e LIFETRAC_ENCODE_MODE=9 -e LIFETRAC_VECTOR_DETAIL=80" -Archive
  ```

- **2b — FHSS (profile 1), VECTOR at boot:** the same with `-RegProfile 1`;
  base capture with `--profile image_bw250`. At `-SynthFps 1` expect the first
  frame at the base near seq 58 (A7) and no time authority on the tractor
  (A11, `tx_stream_streak_max` < 8): read P2/P3 from lock and record the
  streak; no base commands on this leg. Its base capture takes
  `--min-frames 200` at 1 fps: after a 57 s acquisition only ~245 frames can
  arrive, and `min_frames 247 < 250` failed both rounds by the letter.
- **2c — control, DTS, `mono_g4` at boot:** the same as 2a with
  `-CamExtraEnv "-e LIFETRAC_ENCODE_MODE=6"`; base capture without `--strict`
  (its checks are for VECTOR; the `frames:` line and `by codec` still count,
  while the `wire bytes (vector)` and `arrival:` lines cover vector frames only
  — zeros on 2c — and `over limit` counts every multi-fragment mono_g4 frame;
  take mono_g4 sizes from the per-frame rows or the JSONL `hex` lengths). This
  leg is the loss baseline the §8 table compares against; for P3 fly it
  interleaved with the VECTOR legs (P3, A15). It carries no base commands, so it
  is **not** a command-delivery baseline. The §8.6 command-delivery row needs
  legs flown with `-ReactiveFire 1 -ProbeEcho 0`, scored as tractor
  `LoRa cmd: PROBE … (rx#N)` lines over base `PROBE TX … (tx#N)` lines, and kept
  out of the image-loss record. RS-13.1 left that row unmeasured (RESULTS,
  "Remaining radio tests").
- **2d — switch leg, DTS, boots `mono_g4`** (same command as 2c). At T+60 s
  and T+180 s, on the base board, publish the operator override the way
  `web_ui` does (`rx_smoke` relays it as a 0x63 command and republishes the
  tractor's ack, retained — no web_ui needed). T is the base capture's first
  received frame, not the harness launch, so the switch lands 60 s into the
  flowing leg. Stamp each publish and keep the acks; on the bench the base's
  `mosquitto_pub` / `mosquitto_sub` live in the `design-controller-mosquitto-1`
  container (`docker exec …`; its `/tmp` is not the host's), or use the
  `lifetrac-v25:latest` image (RS-13.1 `scripts/leg2d_switch.sh`):

  ```sh
  # before T+60 s: one ack subscriber for the whole leg (-R skips the retained
  # replay, %U stamps arrival on the base clock); kill it after the leg
  mosquitto_sub -h 127.0.0.1 -R -F '%U %p' -t lifetrac/v25/status/encode_mode | tee -a /tmp/lifetrac_strict/legs/leg2d_acks.txt &
  # T+60 s -- stamp and publish in ONE command line:
  date +%s.%N | tee -a /tmp/lifetrac_strict/legs/leg2d_switch_times.txt && mosquitto_pub -h 127.0.0.1 -t lifetrac/v25/control/encode_mode_override -r -m '{"mode":"vector","quality":80}'
  # T+180 s -- the same:
  date +%s.%N | tee -a /tmp/lifetrac_strict/legs/leg2d_switch_times.txt && mosquitto_pub -h 127.0.0.1 -t lifetrac/v25/control/encode_mode_override -r -m '{"mode":"mono_g4"}'
  ```

  Driven from the PC, each stamp-and-publish pair must be **one** `adb shell` /
  `ssh` call: as two calls, the return publish once started 16.6 s after its
  stamp, which read as a 17 s switch (A13). Cross-check each stamp against the
  base's own `command TX … (on air)` line in the archive's `rx_daemon.log`
  (same clock): the command goes on air about one command gate after the
  publish.

  `status/encode_mode` is retained, and `camera_service` sends a
  `"source": "boot"` ack at every leg start, so a `mosquitto_sub -C 1` after a
  publish can return a stale ack; P6 reads the first `"source": "lora_cmd"` ack
  after each stamp.

  Base capture without `--strict` (the capture holds `mono_g4` frames on both
  sides of the VECTOR window). Afterwards `clear_retained.py`, or the retained
  override rides into the next leg.

Also record, once per leg while VECTOR frames flow:
`mosquitto_sub -h 127.0.0.1 -C 1 -t lifetrac/v25/video/link_stats` (base broker)
— `rx_codec_name` must read `vector`.

### Pass criteria (step 2)

| # | Criterion | 2a / 2b | 2c | 2d | Measured from |
|---|---|---|---|---|---|
| P1 | dry-run checks on the base capture: PASS on `parse_ok`, `all_vector`, `one_fragment`, `store_clean`, `epoch_seen`, `first_apply`, `applied_ratio`, `min_frames`; `no_orphans` and `digest` judged by the store's own loss rule — record the `store:` line (orphans, digest checked / mismatched, resync) and require resync 0 or every resync ended — by the next epoch start, or by 3 consecutive matching DIGESTs (the store's DIGEST exit, `resync_digest_ends`; read the dry-run `resync:` episodes line, not the `resync N` count) — and the final `scene:` line must not read `digest_ok False` (the dry run's own end check, `ends_ok`); it reads `None` when no DIGEST has arrived since the last epoch start — the store's `_reset` clears the flag, and after a range-1/2 start the encoder sends no DIGEST until TTL_FRAMES + 3 frames after the youngest freed id (`encode_vector.py` `_digest_quiet_until`) — so record `scripts/leg_replay.py`'s `last DIGEST checked` line next to it: a run that the last epoch start ended is recovered by the store's rule but not confirmed by a DIGEST inside the capture, and the record says so; and every mismatch run must start at a recorded frame loss (a sequence gap in the base capture) or at the base joining mid-epoch after acquisition — a mismatch with no loss before it fails P1 (it is an encoder/store defect; RS-13.1 A1 would have passed the bare rule). The encoder's DIGEST mirror is "the state the base holds if every frame arrived", so one lost frame carrying an UPD, define or GAIN makes the next DIGEST mismatch and later records orphans: the raw `RESULT: PASS` is expected only on a loss-free leg — record it as well | required | n/a | `store_clean`, `epoch_seen`, and `no_orphans` / `digest` by the same rule (capture without `--strict`; read the `[PASS]`/`[FAIL]` lines) | `vector_dry_run.py` |
| P2 | frames published per second ≥ 90 % of the camera rate: `published frame_id` lines in `rx_daemon.log` ÷ 300 ≥ 1.8 at 2 fps (≥ 0.9 at 1 fps). An FHSS leg at 1 fps is read **from lock**: published frames ÷ the span from the first to the last published frame (`frag_gap_report.py`'s `n_frag … span …`), with the whole-leg figure recorded beside it (A7: 245 ÷ 300 = 0.82 whole-leg, 1.00 from lock, on both rounds) | required | baseline | n/a | harness archive |
| P3 | frame loss not worse than the control: the `capture seq gaps` loss of `rs12_leg_report.py --capture` (A14; on a 1-fps FHSS leg this is the loss from lock — record the `counting from seq 1` figure too, A7), the report's counter `loss` line, and radio Δtx_ok ↔ Δrx_ok from the brackets. **Single legs compare only** (a) **interleaved** — control and VECTOR legs alternated in one session (e.g. 2c, 2a, 2c′, 2a′) and their pooled seq-gap losses compared — or (b) against a **margin stated in RESULTS before the legs**; absent another statement, "not worse" means not significantly worse: two-sided Fisher exact test on lost / received of the VECTOR leg against the control, p ≥ 0.05 (round 3's 2a: 4 / 303 against 1 / 304, p ≈ 0.2). The same control lost 6.5 % on 2026-09-27 and 0.3 % on 2026-10-03, more than the effect P3 tests, and VECTOR frames ride fuller than mono_g4 (p50 242 B vs 148 B on the video), i.e. longer on air per frame (A15) | required | baseline | n/a | `leg*_report.txt`, brackets |
| P4 | one fragment per VECTOR frame: every per-frame completion line in `tx_daemon.log` has K = 1. The harness runs `-TxPipeline v3`, which logs `frame seq=N done (pipelined): K fragments ok` (only v2 logs `done: K`): `grep -cE "done( \(pipelined\))?: 1 fragments ok"` > 0, so an empty grep cannot pass; `grep -cE "done( \(pipelined\))?: ([2-9]\|[1-9][0-9]+) fragments ok"` = 0; `ABORTED( \(pipelined\))?` lines listed. The legs run `-TxBatch 0`, so K counts one frame's fragments, not a batched train. At 1 fps the encode-to-fit packer kept mono_g4 to one fragment as well, so P4 is not a contrast with 2c (A10) | required | n/a | inside the VECTOR window | harness archive |
| P5 | keyframe-storm signature absent on FHSS: `frag_gap_report.py` shows no gap > 3 s during image traffic | 2b required | n/a | n/a | `leg2b_gaps.txt` |
| P6 | over-the-air switch: first `"source": "lora_cmd"` ack after the publish stamp `requested 9, effective 9, effective_name vector, codec 6, clamped false, quality 80`; first codec-6 payload in the base capture ≤ 1 camera period + the **encoder's own time** (`ms_total` p95, the leg's P8 figure) + 1 fragment airtime + one command gate after the publish stamp (both on the base clock; A9 — without the encode time the bound is ≈ 2.5 s at 1 fps and RS-13.1 measured 2.55 / 2.68 s, with it ≈ 3.0 s); the stamp taken in the **same board call** as the publish and checked against the base's `command TX … (on air)` line (A13); return ack `effective 6, quality 55` (tile dial untouched, §6) and codec-1 frames resume within one camera period of the return command going on air | n/a | n/a | required | `leg2d_acks.txt`, `leg2d_switch_times.txt`, `leg2d_base.jsonl`, archive `rx_daemon.log` |
| P7 | link_stats `rx_codec_name` = `vector` during VECTOR | required | n/a | required | `mosquitto_sub` line |
| P8 | encoder keeps up on air: `vector_stats` p95 in the archived `camera_service.log` within the step-1 figure ± 20 % | required | n/a | window | `tractor-log` on the archive's `camera_service.log` |

GO for RS-13.2 (browser desk check on the production compose, then the
attenuator range-edge leg) when 2a, 2b and 2d pass and 2c's baseline is on
record. NO-GO otherwise, with the failing row named in the RESULTS verdict.

## Evidence layout

```
bench-evidence/RS_13_vector_scene_<date>/
  RESULTS.md                                  from RS13_RESULTS_TEMPLATE.md
  legs/step0_image_smoke.txt  step0_image_build.txt  step0_encoder_tests_in_image.txt
  legs/step1_bench_mqtt_reset.txt              retained state before/after the recreate
  legs/step1_<bw>_<scene>_scene.{txt,jpg}      scene check before the pass
  legs/step1_tractor_bw250_{moving,landscape}.{jsonl,json,txt}
  legs/step1_tractor_bw500_landscape.{jsonl,json,txt}
  legs/step1_camera_service_<bw>_<scene>.log  step1_tractor_log_<bw>_<scene>.txt
  legs/leg2a_scene.{txt,jpg}                   scene check before the leg
  legs/leg2a_health_base.txt leg2a_health_tractor.txt   rs116, whole output
  legs/leg2a_pre_base.txt leg2a_pre_tractor.txt leg2a_post_base.txt leg2a_post_tractor.txt
  legs/leg2a_base.{jsonl,json,txt}            vector_dry_run capture (+ --json)
  legs/leg2a_report.txt leg2a_gaps.txt        rs12_leg_report --capture, frag_gap_report
  legs/leg2a_park.txt                          every park attempt, PARK_TRANSIENT included
  ...                                          same set for 2b, 2c, 2d
  legs/leg2d_switch_times.txt leg2d_acks.txt leg2d_link_stats.txt
  scripts/                                     the session's prep/post/switch/scene scripts as run
```

The harness archives (`bench-evidence/radio_monitor_<stamp>_<sha>/` with
`tx_daemon.log`, `rx_daemon.log`, `camera_service.log`, `params.txt`) stay where
the harness puts them; RESULTS.md names each one per leg. Pull the board files
with `adb -s <serial> pull /tmp/lifetrac_strict/legs/ <evidence dir>/legs/`
(Windows-style destination path from Git Bash, A6). Evidence headers carry the
PC clock: the tractor's clock runs without network time (Wi-Fi off) and read
13 days behind during RS-13.1, so its container-log timestamps are only
consistent within one log; P6 timing uses the base clock alone.

## Traps

- `LIFETRAC_USE_LORA_BRIDGE=1` is mandatory for `camera_service` on this path
  (the harness comment above its camera line lists the three ways it looks
  broken without it: no MQTT client, deshake on, M7 UART opened).
- An image without numpy/OpenCV makes `camera_service` refuse VECTOR silently
  at boot: frames come out as codec 4 (Y_ONLY) and the tool's `all_vector`
  check fails; a 0x63 command to mode 9 acks `requested 9, effective 1,
  clamped true`. Step 0 exists for this. If the production unit ever runs
  `camera_service` on the host python instead of the image, the same
  requirements must be installed there. If OpenCV is installed but cannot
  load (any `ImportError` from `import cv2`), the clamp does not bite,
  because `camera_service` only checks `find_spec('cv2')`: the ack reads
  `effective 9, clamped false, codec 6`, but every frame logs
  `camera_service: frame build failed (encode_vector requires OpenCV on the
  tractor X8)` and nothing is published — the tool fails `min_frames` while
  `all_vector` passes vacuously. Step 0's `import cv2` catches both cases.
- A running `bench_mqtt` keeps the retained `tractor/link_budget` and
  `tractor/encode_mode_override` of the last harness leg (no persistence, but
  it lives in memory for as long as the container does): recreate it before
  every step-1 pass and check it is empty (Step 1, A12). The production
  `lifetrac-camera` unit maps the radio UART `/dev/ttymxc3` on this bench:
  never restart it here (Step 0).
- Never put the moving content in a kiosk or any fullscreen the operator
  cannot leave with Esc (Scene): the 2026-09-27 kiosk locked the operator out
  of the PC for a whole round. The nocookie embed fails with Error 153; the
  watch URL plays two pre-roll ads first.
- The leg report's counter `loss` line is not the loss of record: the base's
  last `stats:` line is up to 10 s old when the log ends (A14), and a dead
  stats thread freezes it (RS-12.17). Use the `capture seq gaps` line.
- A1 / A1b / A1c (the encoder's unverified live shapes, stale-colour re-sends,
  the range-2 refresh that keeps masses) are fixed in PR #138; on an image or a
  `/work` tree without the fix a static scene desyncs on a loss-free path
  20 applied frames after the first unverified shape (the store's TTL). Check
  which encoder ran (Step 1.2) before reading P1.
- The retained override survives a leg: run `clear_retained.py` before every
  boot-mode leg, or the tractor is switched by the relay as soon as the daemons
  reconnect.
- AE/AWB lock (B5) is not built: sky and ground colours drift with exposure;
  GAIN records absorb part of it. Not a failure — note it.
- 2 fps × 203 B is about 3.3 kbit/s of image traffic in one fragment per 200 ms
  FHSS slot, so a VECTOR leg cannot storm. If `frag_gap_report.py` shows gaps
  on 2b, look at the command plane first.
- `LIFETRAC_KEYFRAME_COPIES` stays at its default (1) in RS-13.1. The tx daemon
  also switches keyframes to 2 copies on its own when recent fragment loss
  exceeds 0.5 %; if that happens, epoch starts go out as 0xFD trains — still
  one chunk (that is what F−1 buys), two copies. Note it in Anomalies.
