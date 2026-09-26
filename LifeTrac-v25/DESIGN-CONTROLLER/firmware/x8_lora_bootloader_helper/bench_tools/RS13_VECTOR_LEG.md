# RS-13.1 — VECTOR (codec 6) on the bench: image smoke, camera dry run, first radio legs

*Procedure, 2026-09-26. Software under test: PR #135 (`rs13-vector-phase1` — the
Phase 1 codec, encoder, store and renderer). Design: `VECTOR_SCENE.md` (PR #129).
Campaign context: [BENCH_RUNBOOK.md](BENCH_RUNBOOK.md) — its prep, brackets,
park and evidence discipline apply unchanged; this file adds only what VECTOR
needs. Results go into `bench-evidence/RS_13_vector_scene_<date>/RESULTS.md`,
started from [RS13_RESULTS_TEMPLATE.md](RS13_RESULTS_TEMPLATE.md).*

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
| `no_orphans` | no record referenced an unknown shape |
| `digest` | no DIGEST mismatch and the scene ends `digest_ok` |
| `epoch_seen` | at least one epoch start applied |
| `first_apply` | the first vector frame received was applied (no wait for a later epoch start) |
| `applied_ratio` | ≥ 90 % of vector frames applied (`--min-applied-ratio`) |
| `min_frames` | at least `--min-frames` vector frames received |

## Step 0 — image and staging (tractor)

1. Build the tractor image from the branch and deploy it exactly as
   [README-DEPLOY.md](../../tractor_x8/README-DEPLOY.md) steps 1–3 say. The
   only RS-13 addition is that the image now needs numpy and
   `opencv-python-headless` (both in `requirements.txt`) and `libglib2.0-0`
   (Dockerfile) — an image built before PR #135 lacks them and fails silently
   (see Traps).
2. Smoke test on the tractor; save the output as `legs/step0_image_smoke.txt`:

   ```sh
   echo fio | sudo -S -p '' docker run --rm -w /app --entrypoint python3 lifetrac-tractor-x8:latest -c \
     "import numpy, cv2; import x8_image_pipeline.encode_vector as e; print('numpy', numpy.__version__, 'cv2', cv2.__version__, 'encoder', e.VectorEncoder.__name__)"
   ```

   Expected: `numpy 2.x cv2 4.14.0 encoder VectorEncoder`. An `ImportError`
   naming `libgthread-2.0.so.0` means the image predates the Dockerfile's
   `libglib2.0-0` line — rebuild.
3. Stage the bench tool. The harness pushes `camera_service.py`,
   `image_tx_daemon.py`, `image_rx_daemon.py`, `lora_proto.py`,
   `image_pipeline/`, `x8_image_pipeline/` and `paho/` to `/tmp/lifetrac_strict`
   on both boards at every launch; push the tool next to them on both boards
   (the base gets its other files from the harness; the tractor needs
   `lora_proto.py` and `image_pipeline/` for step 1, so push those too if no
   harness leg has run yet on this boot):

   ```powershell
   $env:MSYS_NO_PATHCONV = "1"
   $dc = "C:/GitHub/LifeTrac/LifeTrac-v25/DESIGN-CONTROLLER"
   foreach ($s in "2E2C1209DABC240B", "2D0A1209DABC240B") {
     adb -s $s shell "mkdir -p /tmp/lifetrac_strict/legs"
     adb -s $s push "$dc/tools/vector_dry_run.py" /tmp/lifetrac_strict/
   }
   adb -s 2E2C1209DABC240B push "$dc/base_station/lora_proto.py" /tmp/lifetrac_strict/
   adb -s 2E2C1209DABC240B push "$dc/base_station/image_pipeline" /tmp/lifetrac_strict/
   ```

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

## Step 1 — camera-only dry run (tractor, no radio)

Prerequisites: the production camera unit and container stopped (BENCH_RUNBOOK
prep 3: `systemctl stop lifetrac-camera`, `docker stop tractor-camera`) so the
UVC node is free; a broker on the tractor at `127.0.0.1:1883` — the harness's
`bench_mqtt` container stays up between legs (`docker ps | grep bench_mqtt`);
if it is absent, start it as [run_live_radio_monitor.ps1](../run_live_radio_monitor.ps1)
does (its `bench_mqtt.conf` is pushed to `/tmp/lifetrac_strict` at every launch):

```sh
echo fio | sudo -S -p '' docker run -d --name bench_mqtt --network=host \
  -v /tmp/lifetrac_strict/bench_mqtt.conf:/mosquitto/config/mosquitto.conf eclipse-mosquitto:2
```

1. Launch `camera_service` from the image, in VECTOR at boot, with a
   one-fragment budget — the harness's camera line plus the four VECTOR/budget
   variables:

   ```sh
   echo fio | sudo -S -p '' docker rm -f camera_svc 2>/dev/null
   echo fio | sudo -S -p '' docker run -d --name camera_svc --network=host --device=/dev/video1 \
     -w /app --entrypoint python3 \
     -e LIFETRAC_MQTT_HOST=127.0.0.1 -e LIFETRAC_CAMERA_SOURCE=v4l2 -e LIFETRAC_CAMERA_DEVICE=/dev/video1 \
     -e LIFETRAC_CAMERA_FPS=2 -e LIFETRAC_USE_LORA_BRIDGE=1 \
     -e LIFETRAC_ENCODE_MODE=9 -e LIFETRAC_VECTOR_DETAIL=80 \
     -e LIFETRAC_FRAGMENT_BUDGET=1 -e LIFETRAC_FRAGMENT_PROFILE=image_bw250 \
     lifetrac-tractor-x8:latest -u camera_service.py
   ```

   `docker logs camera_svc` must show `byte_budget=203 B/frame
   (LIFETRAC_FRAGMENT_BUDGET, phy=image_bw250)` within the first lines, and
   `vector_stats` lines every 2 s once frames flow. (Running from `/app`
   exercises the deployed image itself; the harness's step 2 runs the same
   file from `/work` inside the same image.)
2. Capture 120 s on the tractor (2 fps → 240 frames; `--min-frames 200`
   tolerates start-up):

   ```sh
   echo fio | sudo -S -p '' docker run --rm --network=host -v /tmp/lifetrac_strict:/work -w /work \
     -e PYTHONPATH=/work:/work/paho --entrypoint python3 lifetrac-tractor-x8:latest \
     /work/vector_dry_run.py capture --topic lifetrac/v25/cmd/image_frame --profile image_bw250 \
     --duration 120 --min-frames 200 --out /work/legs/step1_tractor_bw250.jsonl \
     --json /work/legs/step1_tractor_bw250.json --strict \
     | tee /tmp/lifetrac_strict/legs/step1_tractor_bw250.txt
   ```

3. Encoder timing:

   ```sh
   echo fio | sudo -S -p '' docker logs camera_svc > /tmp/lifetrac_strict/legs/step1_camera_service_bw250.log 2>&1
   echo fio | sudo -S -p '' docker run --rm -v /tmp/lifetrac_strict:/work -w /work -e PYTHONPATH=/work \
     --entrypoint python3 lifetrac-tractor-x8:latest /work/vector_dry_run.py tractor-log \
     /work/legs/step1_camera_service_bw250.log | tee /tmp/lifetrac_strict/legs/step1_tractor_log_bw250.txt
   ```

4. Repeat 1–3 with `LIFETRAC_FRAGMENT_PROFILE=image_bw500` and
   `--profile image_bw500` (the 243 B budget; file names `_bw500`).
5. Scene. Run the bw250 pass twice: once with the RS-3.3 moving content
   (BENCH_RUNBOOK prep 7, the PC screen playing video) for comparability with
   the tile legs, once with a static landscape on the screen (horizon, sky,
   ground, a tree or two) so L0–L2 have something to find. Name the files
   `_moving` / `_landscape`. Stop and remove `camera_svc` when done
   (`docker rm -f camera_svc`); the harness recreates it.

Pass (step 1) — record every row in the RESULTS step-1 table:

| Check | Criterion | Source |
|---|---|---|
| dry-run checks | all ten PASS, `--strict` exit 0, at both profiles | `step1_tractor_*.txt` |
| wire size | `wire max` ≤ 203 / 243, epoch starts ≤ 202 / 242 (the `one_fragment` check) | same |
| rate | `fps_mean` ≥ 1.8 at `LIFETRAC_CAMERA_FPS=2` (≥ 90 % of the camera rate, §8) | same |
| encoder time | `ms_total` p95 ≤ 350 ms at 2 fps (period 500 ms; leaves capture and publish headroom); p50 recorded | `tractor-log` |
| epoch starts | ≥ 1 applied; `lines with a pending epoch start` ≤ 2 (a persistent pending start means the budget could not carry anchor + LAYER_CLEAR, impossible at F ≥ 196 — a bug) | both |
| scene | landscape pass: horizon `abs`, L1 ≥ 1, L2 ≥ 1 in the final `scene:` line (informational — a blank wall legitimately gives L1 = 0) | summary |

If only the encoder time fails, run the radio legs at `-SynthFps 1` and record
the miss: it is a Phase 4 optimisation item, not a blocker for the radio
evidence.

## Step 2 — radio legs

Prep exactly as BENCH_RUNBOOK "Prep" (production camera stopped, both health
probes, [`clear_retained.py`](clear_retained.py) on the base broker,
pre-brackets [`rs115_stats_probe.py`](../rs115_stats_probe.py) on both boards
→ `legs/leg<X>_pre_*.txt`). After each leg: post-brackets,
[`rs12_leg_report.py`](../../../tools/rs12_leg_report.py),
[`frag_gap_report.py`](frag_gap_report.py), then [`radio_park.py`](radio_park.py)
on both → `PARK_OK`. Same camera scene for every leg (moving content per
prep 7; the landscape is a step-1 matter).

**Base-side capture, every leg.** From a second shell on the base board,
started before the harness prints its RX-daemon launch line (it subscribes to
what `rx_smoke` publishes on the base broker; `--duration` covers the 300 s leg
plus start-up). Profile `image_bw500` for profile-2 legs, `image_bw250` for
profile 1:

```sh
echo fio | sudo -S -p '' docker run --rm --network=host -v /tmp/lifetrac_strict:/work -w /work \
  -e PYTHONPATH=/work:/work/paho --entrypoint python3 lifetrac-v25:latest \
  /work/vector_dry_run.py capture --topic lifetrac/v25/video/tile_delta --profile image_bw500 \
  --duration 330 --min-frames 500 --out /work/legs/leg2a_base.jsonl --json /work/legs/leg2a_base.json --strict \
  | tee /tmp/lifetrac_strict/legs/leg2a_base.txt
```

**Legs** (300 s each, `-Archive`; run the harness from
`firmware/x8_lora_bootloader_helper/`). `-KfRequestDisable 1` keeps the base's
reassembly-timeout keyframe requests off: a REQ_KEYFRAME only forces an epoch
start in VECTOR, which would blur the epoch count. `-ProbeEcho 0` as in the
RS-3.3 legs.

- **2a — DTS (profile 2), VECTOR at boot:**

  ```powershell
  .\run_live_radio_monitor.ps1 -TxFeed camera -RegProfile 2 -DurationS 300 -SynthFps 2 `
     -KfRequestDisable 1 -ProbeEcho 0 -LogFragArrivals 1 `
     -CamExtraEnv "-e LIFETRAC_ENCODE_MODE=9 -e LIFETRAC_VECTOR_DETAIL=80" -Archive
  ```

- **2b — FHSS (profile 1), VECTOR at boot:** the same with `-RegProfile 1`;
  base capture with `--profile image_bw250`.
- **2c — control, DTS, `mono_g4` at boot:** the same as 2a with
  `-CamExtraEnv "-e LIFETRAC_ENCODE_MODE=6"`; base capture without `--strict`
  (its checks are for VECTOR; the frame count, codec and size lines still
  count). This leg is the loss and command-delivery baseline the §8 table
  compares against.
- **2d — switch leg, DTS, boots `mono_g4`** (same command as 2c). At T+60 s
  and T+180 s, on the base board, publish the operator override the way
  `web_ui` does (`rx_smoke` relays it as a 0x63 command and republishes the
  tractor's ack, retained — no web_ui needed). Stamp each publish and keep the
  acks; use the `lifetrac-v25:latest` image if the host has no
  `mosquitto_pub`:

  ```sh
  date +%s.%N | tee -a /tmp/lifetrac_strict/legs/leg2d_switch_times.txt
  mosquitto_pub -h 127.0.0.1 -t lifetrac/v25/control/encode_mode_override -r -m '{"mode":"vector","quality":80}'
  sleep 5; mosquitto_sub -h 127.0.0.1 -C 1 -t lifetrac/v25/status/encode_mode | tee -a /tmp/lifetrac_strict/legs/leg2d_acks.txt
  # ... at T+180 s:
  date +%s.%N | tee -a /tmp/lifetrac_strict/legs/leg2d_switch_times.txt
  mosquitto_pub -h 127.0.0.1 -t lifetrac/v25/control/encode_mode_override -r -m '{"mode":"mono_g4"}'
  sleep 5; mosquitto_sub -h 127.0.0.1 -C 1 -t lifetrac/v25/status/encode_mode | tee -a /tmp/lifetrac_strict/legs/leg2d_acks.txt
  ```

  Base capture without `--strict` (the capture holds `mono_g4` frames on both
  sides of the VECTOR window). Afterwards `clear_retained.py`, or the retained
  override rides into the next leg.

Also record, once per leg while VECTOR frames flow:
`mosquitto_sub -h 127.0.0.1 -C 1 -t lifetrac/v25/video/link_stats` (base broker)
— `rx_codec_name` must read `vector`.

### Pass criteria (step 2)

| # | Criterion | 2a / 2b | 2c | 2d | Measured from |
|---|---|---|---|---|---|
| P1 | dry-run checks all PASS on the base capture (`RESULT: PASS` in `leg*_base.txt`) | required | n/a | `store_clean`, `no_orphans`, `digest`, `epoch_seen` only (replay without `--strict`) | `vector_dry_run.py` |
| P2 | frames published per second ≥ 90 % of the camera rate: `published frame_id` lines in `rx_daemon.log` ÷ 300 ≥ 1.8 at 2 fps | required | baseline | n/a | harness archive |
| P3 | fragment loss not worse than the control: raw loss from `rs12_leg_report.py` ≤ leg 2c's, radio Δtx_ok ↔ Δrx_ok from the brackets | required | baseline | n/a | `leg*_report.txt`, brackets |
| P4 | one fragment per VECTOR frame: every `frame seq=N done: K fragments ok` line in `tx_daemon.log` has K = 1 | required | n/a | inside the VECTOR window | harness archive |
| P5 | keyframe-storm signature absent on FHSS: `frag_gap_report.py` shows no gap > 3 s during image traffic | 2b required | n/a | n/a | `leg2b_gaps.txt` |
| P6 | over-the-air switch: first ack `requested 9, effective 9, effective_name vector, codec 6, clamped false, quality 80`; first codec-6 payload in the base capture ≤ 1 camera period + 1 fragment airtime + one command gate after the publish stamp (both on the base clock); return ack `effective 6, quality 55` (tile dial untouched, §6) and codec-1 frames resume within one camera period | n/a | n/a | required | `leg2d_acks.txt`, `leg2d_switch_times.txt`, `leg2d_base.jsonl` |
| P7 | link_stats `rx_codec_name` = `vector` during VECTOR | required | n/a | required | `mosquitto_sub` line |
| P8 | encoder keeps up on air: `vector_stats` p95 in the archived `camera_service.log` within the step-1 figure ± 20 % | required | n/a | window | `tractor-log` on the archive's `camera_service.log` |

GO for RS-13.2 (browser desk check on the production compose, then the
attenuator range-edge leg) when 2a, 2b and 2d pass and 2c's baseline is on
record. NO-GO otherwise, with the failing row named in the RESULTS verdict.

## Evidence layout

```
bench-evidence/RS_13_vector_scene_<date>/
  RESULTS.md                                  from RS13_RESULTS_TEMPLATE.md
  legs/step0_image_smoke.txt
  legs/step1_tractor_bw250_{moving,landscape}.{jsonl,json,txt}
  legs/step1_tractor_bw500_landscape.{jsonl,json,txt}
  legs/step1_camera_service_bw*.log  step1_tractor_log_bw*.txt
  legs/leg2a_pre_base.txt leg2a_pre_tractor.txt leg2a_post_base.txt leg2a_post_tractor.txt
  legs/leg2a_base.{jsonl,json,txt}            vector_dry_run capture (+ --json)
  legs/leg2a_report.txt leg2a_gaps.txt        rs12_leg_report, frag_gap_report
  ...                                          same set for 2b, 2c, 2d
  legs/leg2d_switch_times.txt leg2d_acks.txt leg2d_link_stats.txt
```

The harness archives (`bench-evidence/radio_monitor_<stamp>_<sha>/` with
`tx_daemon.log`, `rx_daemon.log`, `camera_service.log`, `params.txt`) stay where
the harness puts them; RESULTS.md names each one per leg. Pull the board files
with `adb -s <serial> pull /tmp/lifetrac_strict/legs/ <evidence dir>/legs/`.

## Traps

- `LIFETRAC_USE_LORA_BRIDGE=1` is mandatory for `camera_service` on this path
  (the harness comment above its camera line lists the three ways it looks
  broken without it: no MQTT client, deshake on, M7 UART opened).
- An image without numpy/OpenCV makes `camera_service` refuse VECTOR silently
  at boot: frames come out as codec 4 (Y_ONLY) and the tool's `all_vector`
  check fails; a 0x63 command to mode 9 acks `requested 9, effective 1,
  clamped true`. Step 0 exists for this. If the production unit ever runs
  `camera_service` on the host python instead of the image, the same
  requirements must be installed there.
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
