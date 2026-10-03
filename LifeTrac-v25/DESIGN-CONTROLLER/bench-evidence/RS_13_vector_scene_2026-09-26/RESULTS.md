# RS-13.1 — VECTOR (codec 6) first bench legs (2026-09-26)

**Status: complete (2026-10-03). Verdict: GO for RS-13.2 (desk check).**
Step 1 failed on 2026-09-26 with the shipped encoder (loss-free sync
defects A1–A5); the encoder was fixed on `rs13-vector-encoder-sync-fix`
(`291fc90e`, review rounds 3–4 `ac199b1a`) and step 1 now **passes 10/10**
on the moving page, the landscape still (both profiles) and the railroad
video. The radio legs were flown twice at 1 fps (encoder time still misses
350 ms, A4): on 2026-09-27 on a synthetic moving page, and on 2026-10-03 on
the railroad video with the final encoder (Round 3). In both rounds the
VECTOR-specific rows pass on every leg — P1 by the loss rule, one fragment
per frame (P4), no lock-loss gap (P5), the over-the-air switch (P6), codec
reporting (P7), encoder time on air (P8). The rows that miss by the letter
are radio-side and explained: 2b's P2/P3 from a deterministic 57 s FHSS
acquisition at 1 fps (A7, A11), and in round 3 2a's P3 against an unusually
clean control (A15). Radios parked `0x80` on both boards.

## Software under test

| item | value |
|---|---|
| PR / branch / SHA | #135 `rs13-vector-phase1` @ `ebd9204b` (merged as `dd70cda7`), design PR #129 @ `f32a9ed2` (merged as `65869517`); everything on the bench is from `main` `65869517` |
| encoder fix (re-run + legs) | branch `rs13-vector-encoder-sync-fix` @ `291fc90e` (`fix(vector): keep the encoder mirror in step with the base store (RS-13.1 A1-A5)`), on `main` `65869517`. On the bench it ran **from the pushed `/tmp/lifetrac_strict` tree** on both boards (`push_fix_to_board.sh`: `camera_service.py`, `x8_image_pipeline/` incl. `vs1_codec.py`, `image_pipeline/` incl. the store, `vector_dry_run.py`, md5-verified against the working tree; import smoke inside the image: `encoder from /work/x8_image_pipeline/encode_vector.py … TTL_FRAMES 20`). The harness's camera path launches `/work/camera_service.py`, so the tractor image `2727dfd36f9f` was **not** rebuilt; leg 2a's archive carries `git_sha=65869517` because the fix was committed (`291fc90e`) between 2a and 2b — the tree was identical (the push script's md5 lines in `legs/step1_bw250_moving_fix.capture.out` and the harness pushes the same files at every launch). |
| tractor image | `lifetrac-tractor-x8:latest` id `2727dfd36f9f` (755 MB), built 2026-09-26 19:49–19:55 UTC from `65869517` `firmware/tractor_x8/` — **built natively on the base X8 (aarch64), not on the PC** (the PC has no Docker; the tractor is offline and cannot pip-install), then `docker save` → PC → `adb push` → `docker load -i` on the tractor. The tractor's previous image (`9bfbbc8d06cb`, 2026-05-26, no numpy/OpenCV/encoder) is kept as `lifetrac-tractor-x8:pre-rs13`. Resolved wheels: numpy 2.4.6, **opencv-python-headless 5.0.0.93** (`requirements.txt` allows `>=4.9.0`; CI pins 4.14.0.94 in `base_station/requirements-dev.txt`), Pillow 12.3.0, paho-mqtt 2.1.0. Step 0 smoke: `<numpy x cv2 y encoder VectorEncoder>` (`legs/step0_image_smoke.txt`) |
| encoder tests in this image | `test_vector_encoder` + `test_vector_interop` + `test_vs1_codec_parity_sil` run inside `2727dfd36f9f` on the base: **46 tests, OK, 0 skipped** (python 3.11.16 aarch64, numpy 2.4.6, cv2 5.0.0) — `legs/step0_encoder_tests_in_image.txt`. The cv2-gated classes ran, so the Phase 1 encoder is exercised under OpenCV 5.0, not only the CI's 4.14. |
| base image | `lifetrac-v25:latest` id `4623980c2dac` (deployed from `main` `d3751286`, 2026-09-15; the harness pushes the current `base_station/` code at every launch) |
| bench files | harness push at launch (`camera_service.py`, `image_pipeline/`, `x8_image_pipeline/`, `lora_proto.py`, `paho/`) + `vector_dry_run.py` @ `65869517` (md5 `cda4620a…` on both boards) |
| dials | `LIFETRAC_VECTOR_DETAIL=80` (band V0), camera 2 fps, `LIFETRAC_KEYFRAME_COPIES` default (1), `LIFETRAC_WEBP_QUALITY` default (55) |
| scene (re-run + legs) | **moving**: `moving_scene.html` in a Firefox kiosk — the Sunrise wallpaper panning/zooming over a 24 s cycle with three posts passing every 7–11 s and a drifting rock (Claude put it up at the operator's GO; scene check `legs/step1fix_scene_check_moving.txt`: 28.8 % of pixels changed in 1.9 s); **landscape**: the same still as 2026-09-26 (`legs/step1fix_scene_check_landscape.txt`: 1.9 %). The same moving page was on screen for every radio leg. |
| scene (2026-09-26) | **moving**: video content on the PC screen, started by the operator (scene check `legs/step1_scene_check_moving.txt`: two frames 1.2 s apart differ); **landscape still**: Windows 11 "Sunrise" wallpaper `C:\Windows\Web\Wallpaper\ThemeC\img29.jpg` (3840×2400; sky, snowy mountains, conifer treeline, lake, rocky shore), fullscreen in a separate Firefox kiosk instance (`--no-remote`, throwaway profile, `object-fit: cover`), put up by Claude at the operator's request; after AE settled two frames 1.88 s apart differ in 2.0 % of pixels (moving content: 20 %) — `legs/step1_scene_check_landscape.txt`, camera's view `legs/step1_scene_landscape_camera_frame.jpg` |

## Firmware on the boards

Unchanged for RS-13.1 (VECTOR is host-only on the strict path); recorded so
the counters in the brackets are attributable.

| board | build | md5 | flashed |
|---|---|---|---|
| base 2D0A1209DABC240B | as left by the RS-12.15 campaign (production `589c1203` build family; nothing was flashed in this campaign — the brief forbids it) | not re-read | 2026-09-15 |
| tractor 2E2C1209DABC240B | same | not re-read | 2026-09-15 |

Health probes before every leg (`rs116_health_probe.py`, both boards,
`legs/leg<X>_health_{base,tractor}.txt`): counter families
`RS115-INSTRUMENTED-FIRMWARE=YES RS12-URC-COUNTERS=YES RS12-10-COUNTERS=YES`
on both boards for all four legs; the collection script kept only the probe's
last 12 lines, so the `STATS-OK (attempt n)` / `radio_state=` lines are not in
the files (the pre-brackets that followed read the counters over the same
UART, which is the same evidence). Radios parked (`PARK_OK`, readback and
settle `0x80`) after every leg (`legs/leg<X>_park.txt`) and re-checked
read-only at the end (`legs/session_end_park_2026-09-27.txt`).

## Step 0 — image smoke

```
# RS-13.1 step 0.2 image smoke test, tractor 2E2C1209DABC240B, 2026-09-26T20:03:47Z (PC clock)
# image: lifetrac-tractor-x8:latest 2727dfd36f9f 2026-09-26 19:55:38 +0000 UTC
numpy 2.4.6 cv2 5.0.0 encoder VectorEncoder
exit=0
```

**PASS** (`legs/step0_image_smoke.txt`): numpy, OpenCV and the VECTOR encoder
import from the deployed image's `/app`, no `libgthread` error. The
procedure's expected line reads `cv2 4.14.0`; this image resolved OpenCV
**5.0.0** because `firmware/tractor_x8/requirements.txt` asks only
`opencv-python-headless>=4.9.0` (the 4.14.0.94 pin exists only in the CI's
`base_station/requirements-dev.txt`). Because that is a major-version step
past what CI tests, the three encoder suites were also run inside this image
(46 tests OK, 0 skipped — Software under test). The image **was rebuilt** for
this campaign: the tractor's previous image (`9bfbbc8d06cb`, 2026-05-26) had
no numpy, no OpenCV and no `x8_image_pipeline/encode_vector.py` — exactly the
silent Y_ONLY clamp step 0 exists to catch (checked before the rebuild,
read-only). README-DEPLOY steps 2–3 were **not** run on the bench: step 3
restarts the production `lifetrac-camera` unit, whose compose file maps
`/dev/ttymxc3` (the L072 radio UART on this bench) as its M7 port and takes
`/dev/video1`; the procedure is corrected on this branch (see Anomalies).

## Step 1 — camera-only dry run (tractor, no radio)

| profile | scene | frames | fps_mean | wire p50 / p95 / max | over limit | epoch starts (applied) | store bad / orphans / digest checked-mismatched | encoder ms p50 / p95 / max | pending-epoch lines | checks | verdict |
|---|---|---|---|---|---|---|---|---|---|---|---|
| image_bw250 | moving | 247 | 1.98 | 201 / 202 / 203 | 0 | 202 (202); longest same-`ep` K=1 run 1 | 0 / 0 / 247 checked, **2 mismatched** (rows #29–#30, ep 0, K=0; resync 0; final `digest_ok True`) | 490.6 / **514.7** / 526.0 | 0 | 9/10 (`digest` FAIL) | **FAIL** — encoder time (p95 514.7 > 350) and `digest` (2 mismatches on a loss-free local path) |
| image_bw250 | landscape | 250 | 2.00 | 125 / 199 / 202 | 0 | 5 (5); longest same-`ep` K=1 run 1 | 0 / **33** / 250 checked, **97 mismatched** (BAD from ~16 s into epochs 0 and 1 until the next epoch start; epochs 2–4 clean; `ttl_dropped 8`, `resync 2`; final `digest_ok True`) | 374.9 / **432.5** / 490.3 | 0 | 8/10 (`no_orphans`, `digest` FAIL) | **FAIL** — encoder time (p95 432.5 > 350), `no_orphans`, `digest` on a loss-free local path |
| image_bw500 | landscape | 250 | 2.00 | 102 / 174 / 240 | 0 | 4 (4); longest same-`ep` K=1 run 1 | 0 / **33** / 250 checked, **217 mismatched** (BAD from row 23, ~11 s into epoch 1, and the epoch-start frames of epochs 2 and 3 are themselves BAD; `ttl_dropped 23`, `resync 3`; final **`digest_ok False`**) | 362.5 / **395.3** / 453.4 | 0 | 8/10 (`no_orphans`, `digest` FAIL) | **FAIL** — encoder time (p95 395.3 > 350), `no_orphans`, `digest` on a loss-free local path |

Final `scene:` lines (from `legs/step1_tractor_*.txt`):

```
bw250 moving    scene: epoch 9, level 0, badge 7, digest_ok True, horizon none, L1=16 L2=0 L3=0 L4=0, cal_rev None, anchor age 169 ms
bw250 landscape scene: epoch 4, level 0, badge 7, digest_ok True, horizon resid, L1=23 L2=0 L3=6 L4=0, cal_rev None, anchor age 113 ms
bw500 landscape scene: epoch 3, level 0, badge 7, digest_ok False, horizon resid, L1=19 L2=0 L3=4 L4=0, cal_rev None, anchor age 46 ms
```

The landscape `scene` row (informational): horizon `resid` on both landscape
passes (anchor held), L1 ≥ 1 yes (23 / 19), **L2 = 0** on both (the criterion
asks L2 ≥ 1; the procedure marks this informational).

Stage p50 ms from `tractor-log` (bw250 landscape): `resize=0.5 l0=18.6 l1=176.9 l3=55.2 temporal_pack=124.0`
(bw250 moving: `resize=0.5 l0=19.2 l1=191.7 l3=71.8 temporal_pack=203.6`;
bw500 landscape: `resize=0.5 l0=18.6 l1=167.8 l3=54.7 temporal_pack=118.2`).
L1 (≈170–190 ms) and temporal_pack (≈120–200 ms) are the two stages that put
`ms_total` over the 350 ms budget.

Every capture was also replayed on the PC (`py -3 tools/vector_dry_run.py
replay --profile <p> legs/step1_tractor_<pass>.jsonl`), and each gave the
identical `store:` line, so every store verdict above is deterministic and
depends only on the payloads.

**Step 1: FAIL.**

- **Encoder time failed on all three passes**, p95 514.7 / 432.5 / 395.3 ms
  against ≤ 350. Per the operator's rule, any radio legs run at **1 fps**
  (`-SynthFps 1`, base capture `--min-frames 250`, P2 ≥ 0.9 fps).
- **The sync checks failed on a loss-free path.** `digest` failed on all three
  passes and `no_orphans` failed on both landscape passes, even though every
  frame arrived (gap max ≤ 0.62 s, 100 % applied).
- **Wire size, rate, epoch-start and parse checks passed everywhere.** 0 frames
  were over the one-fragment limit (max 203 / 202 / 240 B) and fps_mean was
  1.98–2.00.

By the procedure's own logic the raw `RESULT: PASS` is expected on a loss-free
path, and it was not obtained on any pass. Step 1 was finished as the operator
instructed ("finish the step, record it, and ask me whether to continue").
Diagnosis: Anomalies A1–A3.

## Step 1 — re-run with the encoder fix (2026-09-27)

After the operator's "proceed with fixing", the A1–A5 defects were fixed on
branch `rs13-vector-encoder-sync-fix` (PR: see the verdict section) and
step 1 was run again, camera only, with the fixed encoder loaded from the
pushed `/work` tree (`step1_pass_fix.sh`: `camera_service.py` run from
`/work`, so `/work/x8_image_pipeline` — the fix — shadows the image's `/app`
copy; the container's `encoder module in use` line in each `.capture.out`
confirms `/work/x8_image_pipeline/encode_vector.py True`). Same camera, same
budgets, same dial (80); the moving content this time was
`moving_scene.html` in a Firefox kiosk (the Sunrise wallpaper panning and
zooming with passing posts; scene check 28.8 % of pixels changed in 1.9 s,
`legs/step1fix_scene_check_moving.txt`), the landscape the same still as
before (1.9 % changed, `legs/step1fix_scene_check_landscape.txt`).

| profile | scene | frames | fps_mean | wire p50 / p95 / max | over limit | epoch starts (applied), trigger | store bad / orphans / digest checked-mismatched / ttl_dropped / resync | encoder ms p50 / p95 / max | checks | verdict |
|---|---|---|---|---|---|---|---|---|---|---|
| image_bw250 | moving | 251 | 2.00 | 201 / 203 / 203 | 0 | 3 (3), all `safety` | 0 / 0 / 250-**0** / 0 / 0 | 400.6 / **448.7** / 518.1 | **10/10** | **RESULT: PASS**; encoder time still > 350 |
| image_bw250 | landscape | 250 | 2.00 | 158 / 200 / 203 | 0 | 3 (3), all `safety` | 0 / 0 / 247-**0** / 0 / 0 | 441.9 / **466.1** / 501.2 | **10/10** | **RESULT: PASS**; encoder time still > 350 |
| image_bw500 | landscape | 250 | 2.00 | 160 / 237 / 242 | 0 | 3 (3), all `safety` | 0 / 0 / 248-**0** / 0 / 0 | 422.5 / **455.5** / 504.3 | **10/10** | **RESULT: PASS**; encoder time still > 350 |

Final `scene:` lines (`legs/step1_tractor_*_fix.txt`):

```
bw250 moving    scene: epoch 2, level 0, badge 7, digest_ok True, horizon abs, L1=19 L2=1 L3=1 L4=0, cal_rev None, anchor age 169 ms
bw250 landscape scene: epoch 2, level 0, badge 7, digest_ok True, horizon resid, L1=22 L2=0 L3=5 L4=0, cal_rev None, anchor age 123 ms
bw500 landscape scene: epoch 2, level 0, badge 7, digest_ok True, horizon resid, L1=21 L2=0 L3=6 L4=0, cal_rev None, anchor age 69 ms
```

`tractor-log` last lines (the new fields): moving `epochs 3 trigger safety
ttl_dropped 0 waiting 5`; bw250 landscape `epochs 3 trigger safety
ttl_dropped 0 waiting 0`; bw500 landscape `epochs 3 trigger safety
ttl_dropped 0 waiting 0`. Stage p50 ms: moving `resize=0.5 l0=18.5 l1=170.6
l3=48.2 temporal_pack=146.0`; bw250 landscape `l1=188.3 l3=57.7
temporal_pack=170.2`; bw500 landscape `l1=187.4 l3=60.3 temporal_pack=153.2`.

Against the first run: digest mismatches 2 / 97 / 217 → **0 / 0 / 0**,
orphans 0 / 33 / 33 → **0 / 0 / 0**, epoch starts on moving content 202 →
**3** (the three 60 s safety refreshes), `ttl_dropped` 0 / 8 / 23 → **0**,
`resync` 0 / 2 / 3 → **0**; the moving pass's `temporal_pack` p50 fell from
203.6 to 146.0 ms with the per-frame double build gone, but `ms_total` p95
(449 / 466 / 456 ms) still misses the 350 ms criterion (A4), so per the
operator's rule the radio legs below ran at **1 fps** (`-SynthFps 1`, base
capture `--min-frames 250`, P2 ≥ 0.9). Step 1 with the fix: **PASS on the
sync criteria, encoder-time miss recorded.**

## Step 2 — radio legs

### Legs

| leg | profile | boot mode | harness archive | base capture | brackets | duration | verdict |
|---|---|---|---|---|---|---|---|
| 2a | 2 (DTS) | vector | `radio_monitor_20260927_112218_65869517` | `legs/leg2a_base.jsonl` | `legs/leg2a_pre_*`, `legs/leg2a_post_*` | 300 s (tx 303 frames, base published 300, capture 302) | **PASS** |
| 2b | 1 (FHSS) | vector | `radio_monitor_20260927_113144_291fc90e` | `legs/leg2b_base.jsonl` | `legs/leg2b_pre_*`, `legs/leg2b_post_*` | 300 s (tx 302, published 245, capture 247; FHSS lock at seq 58) | VECTOR criteria PASS; **P2/P3 missed by the letter** (A7) |
| 2c | 2 (DTS) | mono_g4 (control) | `radio_monitor_20260927_114053_291fc90e` | `legs/leg2c_base.jsonl` | `legs/leg2c_pre_*`, `legs/leg2c_post_*` | 300 s (tx 304, published 284, capture 286) | baseline on record |
| 2d | 2 (DTS) | mono_g4 → vector @ T+60 s → mono_g4 @ T+180 s | `radio_monitor_20260927_114959_291fc90e` | `legs/leg2d_base.jsonl` | `legs/leg2d_pre_*`, `legs/leg2d_post_*` | 300 s (tx 305, published 297, capture 299: 179 mono_g4 + 120 vector) | **PASS** |

All four legs flown 2026-09-27 16:16–16:52 UTC with the operator's GO, at
**1 fps** (`-SynthFps 1`, the procedure's fallback for the encoder-time
miss; base capture `--min-frames 250`). Leg prep per leg
(`leg<X>_prep.txt` in the session scratchpad; the products are the
`legs/leg<X>_*` files): production camera unit inactive, no `/dev/ttymxc3`
holder on either board, `rs116` both boards, `clear_retained.py` on the base
broker (`RETAINED-CLEARED …` in `legs/leg<X>_clear_retained.txt`),
`rs115` pre-brackets, then the base-side `vector_dry_run.py capture
--duration 480` started detached before the harness. Post: `docker wait` on
the capture, `rs115` post-brackets, `rs12_leg_report.py`,
`frag_gap_report.py`, `radio_park.py` both boards.

Harness line per leg (PowerShell, from `firmware/x8_lora_bootloader_helper/`):

```
2a: .\run_live_radio_monitor.ps1 -TxFeed camera -RegProfile 2 -DurationS 300 -SynthFps 1 -KfRequestDisable 1 -ProbeEcho 0 -LogFragArrivals 1 -TxBatch 0 -CamExtraEnv "-e LIFETRAC_ENCODE_MODE=9 -e LIFETRAC_VECTOR_DETAIL=80" -Archive
2b: the same with -RegProfile 1
2c: .\run_live_radio_monitor.ps1 -TxFeed camera -RegProfile 2 -DurationS 300 -SynthFps 1 -KfRequestDisable 1 -ProbeEcho 0 -LogFragArrivals 1 -TxBatch 0 -CamExtraEnv "-e LIFETRAC_ENCODE_MODE=6" -Archive
2d: the same as 2c, plus the switch script (legs/leg2d_switch_times.txt, legs/leg2d_acks.txt)
```

`params.txt` of every archive: `synth_fps=1 tx_batch=0 kf_request_disable=1
probe_echo=0 tx_feed=camera` and the profile above. Channel / spot-check: no
survey was run before the legs (bench distance, same antennas as RS-12.15).
Radios parked between legs: `PARK_OK` × 2 after each of 2a, 2b, 2c, 2d
(`legs/leg<X>_park.txt`), no transient.

### Numbers

| # | criterion | 2a | 2b | 2c (baseline) | 2d | pass? |
|---|---|---|---|---|---|---|
| P1 | dry-run checks on the base capture (`RESULT:` line; failing check names) | raw `RESULT: FAIL` (`no_orphans`, `digest`); `store: applied 302, bad 0, epoch_behind 0, orphans 44, digest 301 checked / 30 mismatched, epochs 12, handovers 11, ttl_dropped 1, resync 2`; resync episodes rows 4→59 and 296→299, **both ended by the next epoch start** (key rows 59, 299); final `scene: … digest_ok True`; the other 8 checks PASS | raw `RESULT: FAIL` (`no_orphans`, `digest`, `min_frames` 247 < 250); `store: applied 247, bad 0, epoch_behind 0, orphans 56, digest 244 checked / 19 mismatched, epochs 8, handovers 7, ttl_dropped 0, resync 1`; one resync episode rows 0→19 (the base joined mid-epoch after acquisition), **ended by the key at row 19**; final `digest_ok True`; 0 frames lost after lock | n/a (286 mono_g4 frames, `store_clean` PASS, 0 unparseable) | VECTOR window: `store: applied 120, bad 0, epoch_behind 0, orphans 0, digest 120 checked / 0 mismatched, epochs 7, handovers 6, ttl_dropped 0, resync 0`; `[PASS] store_clean`, `[PASS] no_orphans`, `[PASS] digest`, `[PASS] epoch_seen 7/7`; final `digest_ok True` | **PASS** (loss rule) 2a, 2b, 2d |
| P2 | frames published / s (`published frame_id` lines ÷ 300) | 300 ÷ 300 = **1.00** | 245 ÷ 300 = **0.82** (1.00 over the 245 s after lock; 57 frames sent before the base locked) | 284 ÷ 300 = 0.95 (baseline) | n/a | 2a PASS; **2b FAIL by the letter** (A7) |
| P3 | fragment loss: raw loss (`rs12_leg_report.py`), Δtx_ok ↔ Δrx_ok | `loss 3/303 = 1.0%   crc_dumps=10`; brackets base Δrx_ok +303, Δcrc_err +11; tractor tx 303 frames (per-frame TX events; tx counter 535→841 = +306, not reset for this leg) | `loss 66/302 = 21.9%` — inflated: it takes `rx_frames=236` from the base's last `stats:` line, 9 s before the log ends (the RS-12.17 stale-counter pattern; 245 were published). Per-frame events: 304 sent (tx `done` events + the capture's two tail frames), **247 received = 57 lost = 18.8 %, all before the FHSS lock**; after lock `seq gaps … 0 in 0 gaps` = **0 %**; brackets base Δrx_ok +247, Δcrc_err 0 | `loss 20/304 = 6.6%   crc_dumps=19`; base Δrx_ok +287, Δcrc_err +20 | `loss 9/305 = 3.0%   crc_dumps=5`; base Δrx_ok +302, Δcrc_err +5, Δtx_ok +2 (the two switch commands) | 2a 1.0 % ≤ 6.6 % PASS; 2b raw 18.8 % (report: 21.9 %) > 6.6 % **FAIL by the letter**, 0 % after lock |
| P4 | max fragments per frame (`done (pipelined): K fragments ok`, max K) | K=1 lines 303, K≥2 0, ABORTED 0, **max K 1** | 302 / 0 / 0, **max K 1** | (304 / 0 / 0, max K 1 — the encode-to-fit packer also kept mono_g4 to one fragment at 243 B) | 305 / 0 / 0, **max K 1** (whole leg, so the window too) | **PASS** |
| P5 | lock-loss gaps > 3 s (`frag_gap_report.py`) | n/a (`gaps>3s=0`, max 2.0 s) | `n_frag=245 span=244s max_gap=1.2s gaps>3s=0 total_silent=0.0s` | n/a (`gaps>3s=0`) | n/a (`gaps>3s=0`) | **PASS** |
| P6 | switch: first ack JSON; publish stamp → first codec-6 payload (s); return ack JSON; codec-1 resumed after (s) | n/a | n/a | n/a | first `lora_cmd` ack **+1.40 s** after the vector stamp: `{"requested": 9, "effective": 9, "effective_name": "vector", "clamped": false, "codec": 6, "quality": 80, "source": "lora_cmd"}`; first codec-6 payload **+2.55 s** (seq 1, 240 B; the last codec-1 frame −0.26 s); return ack **+1.31 s**: `{"requested": 6, "effective": 6, "effective_name": "mono_g4", "clamped": false, "codec": 1, "quality": 55, "source": "lora_cmd"}`; last codec-6 +0.63 s, first codec-1 **+1.22 s** after the mono_g4 stamp; codec runs 60 × codec-1, 120 × codec-6, 119 × codec-1 | **PASS** — acks exact; 2.55 s = 1.40 s delivery + one 1 s period + the encode (0.42 s p50) + airtime; the procedure's bound omits the encode time (A9) |
| P7 | link_stats `rx_codec_name` | `"rx_codec": 6, "rx_codec_name": "vector", "radio_profile": 2` (126/126 frames, 0 missing) | `"rx_codec_name": "vector", "radio_profile": 1` (after lock) | `"rx_codec_name": "mono_g4"` | `"mono_g4"` (2 frames seen) → `"vector"` (64 frames seen) after the T+60 override | **PASS** |
| P8 | on-air encoder ms p95 vs step 1 | `p50 421.4 / p95 470.5 / max 524.7` vs step-1 bw500 landscape 455.5 (+3 %) | `p50 411.4 / p95 463.9 / max 555.1` vs bw250 moving 448.7 (+3 %) | n/a | `p50 418.5 / p95 464.3 / max 495.4` (window) vs 455.5 (+2 %) | **PASS** (within ±20 %) |

`vector_dry_run.py` summaries (paste the `== vector dry run summary ==` block
of each base capture):

```
== vector dry run summary ==   (2a, legs/leg2a_base.txt)
profile image_bw500: one-fragment limit 243 B (epoch starts 242 B)
frames: 302 total, 302 vector, 0 other, 0 unparseable; by codec {'vector': 302}
wire bytes (vector): min 215 / p50 241 / p95 243 / max 243; over limit: 0
epoch starts: 12 received, 12 applied
arrival: 0.99 fps mean over 303.6 s; gap p50 1.00 s, p95 1.08 s, max 2.03 s; first applied at +0.00 s (0 frame(s) before it)
store: applied 302, bad 0 , epoch_behind 0, orphans 44, digest 301 checked / 30 mismatched, epochs 12, handovers 11, ttl_dropped 1, resync 2, records 13661
scene: epoch 11, level 0, badge 7, digest_ok True, horizon abs, L1=18 L2=0 L3=0 L4=0, cal_rev None, anchor age 99 ms
RESULT: FAIL   (no_orphans, digest — the loss rule applies; see P1)
```

```
== vector dry run summary ==   (2b, legs/leg2b_base.txt)
profile image_bw250: one-fragment limit 203 B (epoch starts 202 B)
frames: 247 total, 247 vector, 0 other, 0 unparseable; by codec {'vector': 247}
wire bytes (vector): min 197 / p50 202 / p95 203 / max 203; over limit: 0
epoch starts: 7 received, 7 applied
arrival: 1.00 fps mean over 246.1 s; gap p50 1.00 s, p95 1.01 s, max 1.21 s; first applied at +0.00 s (0 frame(s) before it)
store: applied 247, bad 0 , epoch_behind 0, orphans 56, digest 244 checked / 19 mismatched, epochs 8, handovers 7, ttl_dropped 0, resync 1, records 9626
scene: epoch 8, level 0, badge 7, digest_ok True, horizon abs, L1=17 L2=0 L3=1 L4=0, cal_rev None, anchor age 167 ms
RESULT: FAIL   (no_orphans, digest, min_frames 247 < 250 — see P1/P2)
```

```
== vector dry run summary ==   (2d, legs/leg2d_base.txt; capture without --strict)
profile image_bw500: one-fragment limit 243 B (epoch starts 242 B)
frames: 299 total, 120 vector, 179 other, 0 unparseable; by codec {'mono_g4': 179, 'vector': 120}
wire bytes (vector): min 231 / p50 241 / p95 243 / max 243; over limit: 0
epoch starts: 7 received, 7 applied
arrival: 1.00 fps mean over 118.6 s; gap p50 1.00 s, p95 1.08 s, max 1.09 s; first applied at +62.76 s (0 frame(s) before it)
store: applied 120, bad 0 , epoch_behind 0, orphans 0, digest 120 checked / 0 mismatched, epochs 7, handovers 6, ttl_dropped 0, resync 0, records 5415
scene: epoch 6, level 0, badge 7, digest_ok True, horizon abs, L1=20 L2=0 L3=0 L4=0, cal_rev None, anchor age 100 ms
RESULT: FAIL   (all_vector, min_frames — by design on the switch leg; the window checks all PASS)
```

2c (control, `legs/leg2c_base.txt`): `frames: 286 total, 0 vector, 286 other …
by codec {'mono_g4': 286}`; mono_g4 wire bytes from the JSONL payload lengths:
min 121 / p50 199 / p95 241 / max 243, 0 over 243 B.

### Path features exercised

- [x] codec-6 TileDeltaFrame over the strict path as a single 0xFE fragment, profile 2 (2a, 2d: max K 1, max 243 B) and profile 1 (2b: max K 1, max 203 B)
- [x] epoch start (K = 1) as the first frame: anchor + LAYER_CLEAR in one frame, F−1 body (2a: 12 received / 12 applied at 242 B max; 2b 7/7; 2d 7/7)
- [x] CONFIRM / DIGEST carousel over 5 min: digest checks 301 (2a) / 244 (2b) / 120 (2d); 0 mismatches only on 2d's window — 2a/2b had 30 / 19 after lost frames, each run ended by the next epoch start (the designed recovery; A8)
- [x] 0x63 mode switch into and out of VECTOR; the ack's quality byte reports the dial of the acked mode (80 in, 55 back)
- [x] base `link_stats` codec reporting for codec 6 (`rx_codec 6 / rx_codec_name vector`)
- [ ] 0xFD copies path for an epoch start — not exercised (`LIFETRAC_KEYFRAME_COPIES` default 1; no `ABORTED` / copies lines in any `tx_daemon.log`)

### What these legs did NOT test

- V1–V3 (no policy built; every frame V0 at detail 80)
- self-model, Vector Lab, browser rendering (RS-13.2 desk check)
- range edge / attenuator sweep
- AE/AWB lock (B5), tractor self-select (D-VS6b), the ladder's loss-driven floor (B4)
- 2 fps on air (all legs at 1 fps because of the encoder-time miss, A4)
- the first run's video as moving content (the re-run and the legs used `moving_scene.html`; comparability of the moving scene between 2026-09-26 and 09-27 is by scene-check numbers only)
- a channel survey / spot-check before the legs

### Evidence limitations

- **Tractor clock.** It reads **2026-09-13** (13 d 7.9 h behind the PC and the
  base, measured 2026-09-27T00:58:57Z; no network time with Wi-Fi off), so
  every tractor-side container-log timestamp is off by that much. Evidence
  headers carry the PC clock. `legs/step1_scene_check_moving.txt`'s header
  says "tractor clock", but that value is the PC clock.
- **Sample size.** n = 1 per pass, 2 min each, one moving video and one
  landscape image. A2 needed a UPD followed by a same-hash redefine within one
  epoch, and A1 needs an epoch longer than about 20 frames plus the time to the
  first dead-zone shape. Their rates on other content are unknown.
- **No per-candidate log on the tractor.** The *path* that silenced each A1
  shape, and the reason for the A2 geometry rejection, are inferred from the
  wire and the code, not observed in the encoder.
- **Radio legs.** P6 timing rests on the base clock alone (publish stamps
  `date +%s.%N` on the base, ack arrival `%U` from `mosquitto_sub` on the
  base, capture `ts` on the base): no cross-board clock is involved. The
  tractor's clock is still ~13 d behind (its `tx_daemon.log` / `camera_service.log`
  timestamps). Leg 2a's tractor `radio_tx_ok` bracket ran on from the previous
  session's count (535 → 841) while 2b–2d's start from 0 after the harness's
  TX reset — the per-frame TX events (`rs12_leg_report.py`'s "using N
  fragments from per-frame TX events") are the loss denominator in all four.
  The `rs116` health files hold the counter-family lines only (the collection
  kept the last 12 lines). n = 1 per leg; the FHSS acquisition delay (A7) was
  seen once.
- **Scene during the legs.** The moving page was put up in a Firefox **kiosk**
  window at 16:16 UTC and the camera view was checked once before the step-1
  re-run (16:07, 28.8 % changed) but **not before each leg**; no camera frame
  was captured during 2a–2d. The operator did not use the PC during the legs —
  because the kiosk left no way to close it or reach the desktop (reported by
  the operator afterwards), which is also why nothing brought the Claude app in
  front of it; still, that each leg saw the page is inferred, not measured.
  Kiosk windows are no longer used on this bench (next section). `legs/leg2b_link_stats.txt` holds several pre-lock samples with
  `rx_codec_name: null` before the post-lock `vector` one.

### Next round (prepared 2026-09-28, not flown — radios parked, waiting for GO)

- **Moving content:** the railroad reference video
  `https://www.youtube.com/watch?v=B1yUQwpNhJA` (the one RS-12.11–12.15 used) in
  a **normal** Firefox window of a separate profile (audio scaled to 0,
  autoplay allowed), with the **player's own fullscreen** (`f`, the expand
  button) so the operator can leave it with Esc — never a kiosk. The
  `youtube-nocookie.com/embed/…` URL the earlier campaigns used now fails with
  "Video player configuration error / Error 153"; two pre-roll ads (~40 s)
  play first. Tried 2026-09-27 23:53–23:58 UTC (in a kiosk, before the rule):
  the camera sees sky, clouds, trees, mountains and track; once the train rolls
  12.4 % / 19.4 % / 23.1 % of pixels change over 1.9 / 5 / 9.7 s
  (`legs/youtube_scene_check.txt`, `legs/youtube_scene_camera_frame.jpg`).
- **Before every camera run** the prep brings that window to the front and
  runs a scene check (≥ 8 % change over 10 s, frame saved as
  `legs/<leg>_scene.jpg`), aborting the leg otherwise; the `rs116` health
  output is now kept whole.
- **Proposed order at GO:** step-1 bw250 moving on the video (camera only), then
  2b re-flown on the video (the row this record still owes), then 2a/2c/2d on
  the video for like-for-like numbers, all at 1 fps with the final encoder
  (`ac199b1a`, review rounds 3–4, already pushed to both boards' `/work`).

### Anomalies for the record

Abbreviations: **enc** = `firmware/tractor_x8/x8_image_pipeline/encode_vector.py`,
**store** = `base_station/image_pipeline/vector_scene_store.py`, **spec** =
`VECTOR_SCENE.md`, all at `65869517`; the image on the tractor was built from
that tree (`legs/step0_image_build.txt`). The diagnosis was read-only: two
multi-agent investigations, each with independent investigators and two
adversarial skeptics, working only in scratch scripts outside the repo.
Every repro command below was re-run by hand on the PC.

- **A1 — Loss-free desync on a static scene: the encoder leaves live shapes
  unverified, and the store's 20-frame TTL deletes them.** Both skeptics upheld
  it.
  - **Store (per spec).** The store drops a shape after 20 applied frames
    without a verifying define, UPD or digest-gated CONFIRM: `TTL_FRAMES = 20`
    (store:35), `_tick_ttl` (store:433–442), spec:610 "Shape TTL: 20 frames
    without a verified define, CONFIRM or UPD". A UCOL does not verify.
  - **Encoder mirror.** It has no TTL (no TTL identifier anywhere in
    `firmware/tractor_x8/` except `roi.hint_ttl_ms`), and the DIGEST covers
    the whole mirror (enc:681–683).
  - **The dead zone.** A matched live L1 shape can get no record at all: no
    DEL (enc:773–775), no CONFIRM or carousel unless it is in `verified`
    (enc:794, :799–801). Two paths lead there:
    - (a) geometry passes, ΔE76 > 6, and the quantised fill is unchanged, so
      there is no UCOL (enc:790);
    - (b) geometry fails, and the same-id redefine (enc:803–805) is rejected
      inside `_add_define` (ΔD gate enc:951–953 or EG2 `ValueError`
      enc:941–944).
    This contradicts spec:602 ("If it fails, the tractor redefines or deletes
    it").
  - **bw250 landscape timeline.** id 6 (a dark mass, likely the treeline) was
    last named by its row-11 redefine and is silent in rows 12–55. The store
    TTL-drops it at the end of row 31, and row 32 (+15.85 s) is the first BAD,
    because the digest check (store:416) runs before the tick (store:428). The
    DIGEST's crc 82 against the store's 160 at row 32 is restored exactly by
    adding id 6 back. Epoch 1 behaves the same with id 14 (last CONFIRM row 67,
    first BAD row 88, crc 79 against 202).
  - **Cascade.** Primary drops 6@31, 5@36, 14@87 and 23@97. After that, BAD
    digests make the store refuse CONFIRMs (store:678, spec:607), so four more
    shapes age out (secondary drops). 3 BAD in a row enters resync
    (store:646–649), which only a range-0 epoch start ends. That accounts for
    the 33 orphans (11 CONFIRMs and 5 UCOLs of missing ids, 14 CONFIRM tag
    contradictions, 3 DELs of missing ids).
  - **Why epoch 3 stayed clean for 26 s.** Chance: with the TTL off, its
    longest unverified run is 11 frames, against 44 and 41 in epochs 0 and 1.
  - **Counterfactual (store TTL disabled in memory only):**
    `PYTHONIOENCODING=utf-8 py -3 -c "import sys; sys.path.insert(0,'tools'); import vector_dry_run as v, image_pipeline.vector_scene_store as s; s.TTL_FRAMES=10**9; sys.exit(v.main(['replay','--quiet','--profile','image_bw250','bench-evidence/RS_13_vector_scene_2026-09-26/legs/step1_tractor_bw250_landscape.jsonl']))"`
    → `orphans 0, digest 250 checked / 0 mismatched, ttl_dropped 0, resync 0`.
    For bw500 landscape: `orphans 0, digest 250 checked / 0 mismatched,
    ttl_dropped 0, resync 0` (as shipped: 33 / 217 / 23 / 3).
  - **A1b — Re-sends carry a stale colour** (one skeptic found it, and the
    store side was re-checked). Repeat and carousel re-send `s.define` with its
    define-time fill (enc:797, :800), not "plus its current UPD/UCOL"
    (spec:601). This re-created dropped ids 23 (row 111) and 12 (row 143) with
    stale colours.
  - **A1c — The 60 s safety refresh does not heal it.** On bw500 landscape the
    epoch starts at rows 122 and 243 are LAYER_CLEAR range 2, which keeps the
    masses (enc:461–462, :588–589). `dig=BAD` runs straight through both, and
    the capture ends in resync with `digest_ok False`. This contradicts
    spec:609 ("a desynchronised base recovers within one safety period").
  - **A1d (minor).** The store compares the DIGEST crc only, not n_live
    (store:641). Rows 115 and 123 read `ok` through CRC-8 collisions (n_live
    27 against 26).
  - **On-air consequence.** The defect does not depend on loss and appears in
    any static epoch longer than (first dead-zone shape + 20 applied frames).
    Under P1's loss rule it would *pass* the bw250 capture (resyncs ended by
    range-0 starts, final `digest_ok True`) and *fail* bw500, so P1's
    digest/orphan/resync counts on a radio leg cannot be read as loss
    metrics. The operator sees regions vanish (the store deletes shapes the
    tractor believes are drawn), stale colours re-appear, and a RESYNC chip
    shows.
  - **Where a fix would go** (none made; out of scope this session):
    - the enc verify pass (:785–805) together with the ΔD gate (:951–953), so
      every live id gets a verifying record or a DEL within 20 frames;
    - the re-send at enc:797/:800;
    - the safety-refresh clear range (enc:461–462), or else the promise at
      spec:609;
    - optionally an n_live check at store:641.

- **A2 — Moving scene: a byte-identical same-id redefine resets the encoder
  mirror's offset, while the store keeps it.** Both skeptics upheld it.
  - **Frames.** #28 sends `Upd(1,3,-1)` to id 1 (the frame-filling background
    POLY, define-hash 0x40b9), and both sides hold off (3,−1) (DIGEST crc 03 =
    store 03). #29 re-emits `Poly(1)` through the slot-5 same-id redefine path;
    its wire position is between new-id defines, and the pack sorts by (slot,
    order) at enc:1016. The re-quantised record is byte-identical (same
    define-hash).
  - **Encoder side.** `_apply_define` (enc:958–962) builds a fresh mirror
    `_Shape` with dx = dy = 0. The identical-define shortcut that would have
    kept the offset requires `s.dx == 0 and s.dy == 0` (enc:912).
  - **Store side, per spec:507.** A same-hash define moves only the geometry
    age and keeps the offset (store:571–574).
  - **Result.** Claim crc 23 against store ad at #29, ef against 5d at #30,
    with n_live equal. #31, a range-0 epoch start, rebuilds both sides. A
    wire-rebuilt encoder mirror reproduces all 247 DIGESTs. The real store
    with only #29's id-1 offset zeroed gives 0 mismatches.
  - **The 4-bit epoch wrap is a coincidence.** With every header epoch rotated
    by +k mod 16 (records untouched), the mismatches stay at exactly #29/#30
    for all 16 values of k.
  - **Open** (hypothesis, not reproduced): why `_verify_geometry` rejected id 1
    at #29. An idealised simulation passes it. Settling it needs the camera
    frames or a per-candidate log.
  - **Latent, same class.** `_apply_define` also resets the fill, while the
    store returns before its fill update (store:574 vs :581–582).
  - **No test covers "UPD, then a byte-identical redefine"** in
    `base_station/tests/test_vector_encoder.py` / `test_vector_interop.py`.
    With the TTL disabled the moving capture still shows its 2 mismatches, so
    A1 and A2 are independent.

- **A3 — Epoch starts on moving content: 202 of 247 frames (82 %), all
  LAYER_CLEAR range 0, every anchor `NO_HORIZON`.**
  - Ruled out: the safety refresh (it sends range 2), a horizon flip (no
    horizon at all), and the force flag (only the boot line). A relabel bound
    peaks at 0.366, under the 0.40 threshold.
  - **Hypothesis, not skeptic-reviewed:** mass-id exhaustion. There are 31 mass
    ids against up to 40 regions. `_IdExhausted` (enc:565, :826–829) rebuilds
    the frame as a range-0 epoch start (enc:331–333). In 19 of 45 K=0 frames
    the fresh id handed out is 31, and K=1 frames spend about 207 ms in
    temporal_pack against about 140 ms on K=0, which fits a double build.
  - The trigger is not logged. Spec:546–556 budgets a moving scene at 6–10 %
    of a frame and principle 7 rules out keyframe trains, so this rate is not
    what the design expects.

- **A4 — Encoder time.** Two stages carry `ms_total` over the 350 ms budget on
  the i.MX8: L1 at ≈170–190 ms p50 and temporal_pack at ≈120–200 ms p50
  (Step 1 stage lines). Moving content is worst (p95 514.7 ms). A3's double
  build is a candidate contributor (hypothesis).

- **A5 — `vector_stats` logs `detail=0` on every line.** camera_service.py:894
  formats `detail=%d` of `int(st.get("detail", 0))` (:897), but
  `detail_of_quality(80)` = 0.5 (enc:128–132). All 169 lines read `detail=0`.
  The same line reports `epoch_pending` as `int(bool(...))` (:899).
  Cosmetic, but tractor-log's `detail` column and its docstring expectation
  (`detail=80`) are wrong.

- **A7 — FHSS acquisition after the harness's RX reset took 57 s at 1 fps
  (leg 2b).** The harness resets the base's L072 (`[RESET] RX L072 via
  gpio163 NRST`) and the follower re-acquires the tractor's hop sequence by
  scanning; the base's first received frame was `seq 58` (`legs/leg2b_base.txt`
  row 0: `seq= 58 … EPOCH-SWITCH`), i.e. the tractor's first 57 frames went
  unheard, and `rx_smoke` logged `rx_frames=0` until 16:27:40 (leg start
  16:26:5x). With a 500 ms dwell per channel on a 50-channel mask and one
  frame per second the expected time for a frame to land on the listened
  channel is of that order; the RS-12.15 legs (2 fps and denser synth feeds)
  locked in seconds. Consequence: 2b's P2 (0.82 fps over 300 s) and raw P3
  (18.8 % from per-frame events; `rs12_leg_report.py` prints 21.9 % because it
  reads a stale `rx_frames` counter) miss by the letter while after lock the link delivered 247/247
  (0 seq gaps, `max_gap=1.2s`). Not a VECTOR defect; re-fly 2b at 2 fps once
  A4 is addressed, or start the base capture window at lock.
- **A8 — Epoch starts on the moving page are `relabel` triggers.** With the
  fix the moving passes no longer restart on id exhaustion (3 starts in
  2 min, all `safety`), but on air the pan/zoom page tripped the 40 %
  relabel rule: 2a 12 epoch starts in 300 s (`trigger=` counts in
  `camera_service.log`: 94 lines `relabel`, 3 `safety`), 2b 7 (97 `relabel`,
  14 `safety`), 2d's window 7 (41 `relabel`, 5 `forced`). Every start was
  applied at the base and each ended the running resync, which is the
  designed recovery — but a start costs a 242 B key frame and re-defines
  the masses, so the relabel rule's sensitivity (its denominator is the
  labelled area, review follow-up C9) is worth a look before range legs.
- **A9 — The P6 bound omits the encoder's own time.** Measured: ack +1.40 s,
  first codec-6 payload +2.55 s after the publish stamp. The procedure's
  bound "1 camera period + 1 fragment airtime + one command gate" gives
  ≈ 1 + 0.1 + 1.4 = 2.5 s; the tractor also spends 0.42 s (p50) encoding the
  first VECTOR frame after the switch. The switch is prompt; the bound should
  add the encode time.
- **A10 — mono_g4 also rode as single fragments.** In 2c and 2d the
  encode-to-fit packer kept every mono_g4 frame within the 243 B budget
  (2c: max 243 B, `tx done K=1` 304, K≥2 0), so P4's "one fragment" is not a
  VECTOR-only property at 1 fps on this scene; the control's value is the
  loss baseline (6.6 %), not a fragment-count contrast.
- **A6 — Operational.**
  - **Aborted first attempt.** The first bw250-moving attempt was aborted:
    `| tr -d '\r'` on the PC block-buffered the capture's "subscribing" line,
    so camera_svc was never launched inside the window. Its files are kept as
    `legs/step1_tractor_bw250_moving_ABORTED1.*` (`--out` appends, so the
    re-run used fresh names).
  - **Unpinned OpenCV.** `requirements.txt` allows `opencv-python-headless>=4.9.0`,
    which resolved 5.0.0.93 on 2026-09-26 against CI's 4.14.0.94. The encoder
    suites pass under 5.0 (Software under test).
  - **Image built on the base.** The PC has no Docker, so the image was built
    natively on the base X8.
  - **Production unit left alone.** README-DEPLOY step 3 would restart the
    production unit that maps the radio UART; it was not run, and the
    procedure is fixed on this branch.
  - **Broker recreated.** `bench_mqtt` was recreated before step 1 to wipe
    in-memory retained state (`legs/step1_bench_mqtt_reset.txt`).
  - **Landscape still.** It was put up by Claude (Firefox kiosk, separate
    profile) at the operator's request and closed after the bw500 pass.
  - **Re-run and legs (2026-09-27).** The post-leg collection script's first
    version handed MSYS `/c/…` paths to `adb pull` and to the Python report
    tools; leg 2a's three base files and reports were re-collected by hand
    with Windows paths before the script was fixed for 2b–2d (`legs/leg2a_*`
    are complete). The `rs116` health files hold only the last 12 probe
    lines (see Evidence limitations).

## Round 3 — the railroad video, final encoder (2026-10-03)

Operator GO 2026-10-03; flown 21:18–22:07 UTC with the final encoder
(`rs13-vector-encoder-sync-fix` @ `ac199b1a`, review rounds 3–4) staged on
both boards' `/work` (md5 `efa00d1e…` = the working tree; both staging trees
had aged out of `/tmp` after 5 days and were re-pushed, `stage_boards.sh`).
Moving content: the railroad reference video
`https://www.youtube.com/watch?v=B1yUQwpNhJA` in a **normal** Firefox window
with the player's fullscreen (no kiosk; Esc returned the screen at any
time), brought to the front and checked on the camera before every run
(`legs/<run>_scene.{txt,jpg}`: 38–57 % of pixels changed over 10 s). All
legs at 1 fps, 300 s, `-TxBatch 0 -KfRequestDisable 1`, same procedure as
round 2; file tags `*_yt` / `*_youtube_fix`.

**Step 1 on the video (bw250, camera only, 2 fps):** **RESULT: PASS** —
`frames: 253 total, 253 vector`; `wire bytes (vector): min 33 / p50 201 /
p95 203 / max 203; over limit: 0`; `epoch starts: 40 received, 40 applied`
(all `relabel`); `arrival: 2.00 fps`; `store: applied 253, bad 0,
epoch_behind 0, orphans 0, digest 253 checked / 0 mismatched, … ttl_dropped
0, resync 0`; encoder `p50 432.7 / p95 477.7 / max 491.5` ms (still over
350 → legs at 1 fps). The first attempt (`legs/*_youtube_fix_ABORTED1.*`)
ran mono_g4: the tractor's bench broker, up 6 days, still held round 2's
retained `tractor/encode_mode_override {"mode": 6}` and a bw500
`tractor/link_budget` (`legs/step1_youtube_bench_mqtt_reset.txt`); the broker
was recreated and the step-1 script now recreates it itself (A12).

| # | criterion | 2a_yt (DTS, VECTOR) | 2b_yt (FHSS, VECTOR) | 2c_yt (control) | 2d_yt (switch) | pass? |
|---|---|---|---|---|---|---|
| archive | | `radio_monitor_20261003_164601_ac199b1a` | `radio_monitor_20261003_163532_ac199b1a` | `radio_monitor_20261003_165432_ac199b1a` | `radio_monitor_20261003_170327_ac199b1a` | |
| P1 | dry-run checks, loss rule | raw `RESULT: FAIL` (`no_orphans` 101, `digest` 297/60); `ttl_dropped 2, resync 3`; resync episodes rows 106→120, 199→213, 236→272, **each ended by the next epoch start**, each opened by one of the 4 lost frames; final `digest_ok True`; other 8 checks PASS | raw `RESULT: FAIL` (`no_orphans` 148, `digest` 190/47, `min_frames` 247); `ttl_dropped 3, resync 2`; episodes rows 0→28 (joined mid-epoch after acquisition) and 107→124 (the one post-lock loss), **both ended by an epoch start**; final `digest_ok True`; 190 DIGESTs in 247 frames (the round-4 rule withholds the DIGEST after a range-2 refresh while freed ids are young) | n/a (303 mono_g4) | window: `store: applied 135, bad 0, orphans 12, digest 135 checked / 1 mismatched, ttl_dropped 0, resync 0`, epoch starts 35/35; the 12 orphans and 1 BAD come from the one lost frame (row 108); final `digest_ok True` | **PASS** 2a, 2b, 2d |
| P2 | `published frame_id` ÷ 300 | 297 → **0.99** | 245 → **0.82** (seq 58 first heard again, A7) | 302 → 1.01 | n/a (296) | 2a PASS; **2b by the letter FAIL** |
| P3 | loss vs control | report `13/301 = 4.3%` reads a stale `rx_frames` (288 vs 297 published); **seq gaps 4 / 303 = 1.3 %** | report `67/302 = 22.2%` (stale); raw **58 / 305 = 19.0 %** = 57 frames before lock + 1 after; after lock **1 / 248 = 0.4 %** | report `10/302 = 3.3%` (stale); **seq gaps 1 / 304 = 0.3 %** | report `15/302 = 5.0%` (stale); seq gaps within same-codec runs **6 / 304 = 2.0 %** (the 193/183 "gaps" across the two switches are the per-codec seq counters restarting, not loss) | 2a 1.3 % > 0.3 % and 2b 19.0 % raw (acquisition 57 + 1 post-lock): **by the letter FAIL** (see A7, A15) |
| P4 | max K, `-TxBatch 0` | K=1 301, K≥2 0, ABORTED 0, **max 1** | 302 / 0 / 0, **max 1** | (302 / 0 / 0, max 1) | 302 / 0 / 0, **max 1**; every VECTOR frame within its limit (the one `over limit` row is #254, a **mono_g4** key frame at 243 B vs 242) | **PASS** |
| P5 | gaps > 3 s | n/a (`gaps>3s=0`, max 2.1 s) | `n_frag=245 span=245s max_gap=2.0s gaps>3s=0` | n/a (`gaps>3s=0`) | n/a (`gaps>3s=0`) | **PASS** |
| P6 | switch | n/a | n/a | n/a | vector ack **+1.55 s** `{"requested": 9, "effective": 9, "effective_name": "vector", "clamped": false, "codec": 6, "quality": 80, "source": "lora_cmd"}`; first codec-6 **+2.68 s** (seq 1, 242 B, K=1); return: command on air 22:01:39.344 (base log), ack **+0.64 s** later `{"requested": 6, "effective": 6, "effective_name": "mono_g4", "clamped": false, "codec": 1, "quality": 55, "source": "lora_cmd"}`, codec-1 resumed +0.55 s after on-air — the script's stamp was 16.6 s early (A13); runs 62 × codec-1, 135 × codec-6, 101 × codec-1 | **PASS** |
| P7 | `rx_codec_name` | `"vector"` (`rx_codec 6`, profile 2) | `"vector"` (profile 1, SNR 10 dB, after lock) | `"mono_g4"` | `"mono_g4"` (2 frames) → `"vector"` (64 frames) | **PASS** |
| P8 | encoder p95 vs step 1 (477.7) | `p50 453.9 / p95 492.1 / max 586.5` (+3.0 %) | `p50 435.2 / p95 488.2 / max 611.1` (+2.2 %) | n/a | `p50 448.1 / p95 493.6 / max 536.6` (+3.3 %) | **PASS** |

Parks: `PARK_OK` × 2 after every leg except 2b_yt, where the base read
`PARK_TRANSIENT` (`opmode_after_settle 0x85`: its scan was still walking);
re-parked 75 s later `PARK_OK` (`legs/leg2b_yt_park.txt`). End of round:
both `RADIO_STATE 0x80 SLEEP`, no UART holder (`legs/session_end_park_2026-10-03.txt`).

P1 under the stricter reading (review of #139): every mismatch run in every
leg of both rounds starts at a recorded sequence gap (a lost frame) or at the
base joining mid-epoch after FHSS acquisition, and recovers by the end
(final `digest_ok True`) — round 3: 2a's three runs open at its 4 lost frames,
2b's at the join and at its one post-lock loss, 2d's single BAD at its lost
frame; round 2 likewise (`scripts/leg_replay.py`). No mismatch occurred
without a preceding loss, so the verdicts hold.

**Round 3 verdict:** the VECTOR-specific rows (P1 by the loss rule, P4, P5,
P6, P7, P8) pass on every leg on real footage with the final encoder. The
two rows that miss by the letter are radio-side: 2b's P2/P3 from the FHSS
acquisition (first frame heard at seq 58 on both flights, A7/A11), and 2a's
P3 against a control that happened to lose 1 frame in 304 (A15).

Round-3 anomalies:

- **A11 — At 1 fps the tractor never becomes FHSS time authority.**
  `sx1276_fhss_authority.h`: authority needs `MIN_STREAK = 8` own
  transmissions each **strictly** `< STREAK_GAP_MS = 1000` ms apart; the
  1 fps cadence sits on 1000 ms (tractor log gaps p10/p50/p90 993/999/1005 ms),
  and the 2b post-brackets read **`tx_stream_streak_max=2`** (round 2) and **`=3`** (round 3;
  RS-12.15 legs: 1246; 8 needed). The base still locked and held (it follows the tractor's
  transmissions; `fhss_dec_snapped` +1, `fhss_dec_aligned` +246), so it is
  not the acquisition delay, but the RS-12.15 clock-authority protection is
  inactive at 1 fps on FHSS — a leg carrying base commands would be exposed
  to the lock-loss mechanism it fixed. The strict `< 1000 ms` chain test is
  deliberate (`sx1276_fhss_authority.h:38-42`, PR #125 review round 3): the
  RS-12.14 gate admits commands at ≥ 1.0 s, and a command sender must never
  chain into authority — by timing alone a 1 fps image stream is
  indistinguishable from it, so `<=` or a wider gap would undo that
  discriminator. Options that keep it: fly at 2 fps once A4 is done (500 ms
  gaps chain), or an explicit stream discriminator (only image-stream
  transmissions count toward the streak) before any change to the gap.
- **A7 (update) — the FHSS acquisition point is deterministic.** Both 2b
  flights (2026-09-27 synthetic page, 2026-10-03 video) heard their first
  frame at **seq 58** after the harness's resets — a fixed meeting point of
  the base's channel scan and the tractor's hop sequence at one transmission
  per five slots, not chance. P2/P3 for an FHSS leg at 1 fps should be read
  from lock, or the leg flown at 2 fps.
- **A8 (confirmed on footage) — relabel storms.** The video drove 40 epoch
  starts in 126 s (step 1), 45 in 300 s (2a; consecutive key frames at rows
  143–148 and 155–165) and 35 in 135 s (2d window). All were applied, each
  costs a 242 B key frame; the 40 % relabel rule's sensitivity (labelled-area
  denominator, review C9) is the next encoder item after A4.
- **A12 — Retained state on the tractor broker spoiled the first step-1
  attempt.** See above; the procedure already prescribes recreating the
  broker before step 1 (the step was missed); `step1_pass_fix.sh` now does it.
- **A13 — P6 stamp artefact.** The switch script stamped with one `adb`
  call and published with a second; once the second started 16.6 s late,
  which read as a 17 s return. Corrected against the base's
  `command TX … (on air)` log line (same clock); the script now stamps and
  publishes in one call.
- **A14 — `rs12_leg_report.py` read a stale `rx_frames` counter on every
  leg of this round** (the base's last `stats:` line, a few seconds before
  the log ends — RS-12.17 class): 4.3 / 22.2 / 3.3 / 5.0 % printed against
  1.3 / 18.8 (0.4 after lock) / 0.3 / ≈2 % from sequence gaps. Loss in this
  record is from sequence gaps.
  *Follow-up (2026-10-03, after the round, PC only):* the report now floors
  `rx_frames` with the per-frame `published frame_id` / `frag_arrival`
  events (with a `(rx rx_frames counter N is stale; …)` note) and, given
  `--capture legs/leg<X>_base.jsonl`, prints the loss from the TileDeltaFrame
  sequence gaps per codec run. Re-run on the four round-3 archives with their
  captures (`legs/leg<X>_yt_report_seq.txt`): counter line 1.3 / 18.9 / 0.0 /
  2.0 %, seq gaps **1.3 / 0.4 from lock / 0.3 / 2.0 %** (2d: 6 / 304 over three
  codec runs, no switch counted). For 2b_yt the frames before the first heard
  (seq 58) count to 58 / 305 = 19.0 % from seq 1; the 18.8 % above is round 2's
  57 / 304. The procedure now takes loss from the seq-gap line
  (`RS13_VECTOR_LEG.md`, Step 2).
- **A15 — Control loss varies more than the effect P3 tests.** The same
  control leg lost 6.5 % on 2026-09-27 and 0.3 % on 2026-10-03; 2a's 4 lost
  frames against the control's 1 (two-sided Fisher exact on lost / received, 4 / 299 vs 1 / 303: p = 0.22) is within that spread.
  VECTOR frames also ride fuller (p50 242 B vs mono_g4 148 B on this video),
  i.e. longer on air per frame. A P3 that compares single legs needs either
  interleaved control/VECTOR legs or a margin.

### GO / NO-GO

**GO for RS-13.2 (desk check).** Deciding rows:
- Step 1 with the fix: **RESULT: PASS** on all four passes (moving page,
  landscape bw250 and bw500, railroad video): 0 mismatches, 0 orphans,
  0 TTL drops, 0 resyncs on a loss-free path.
- Both rounds of legs: P1 (loss rule), P4, P5, P6, P7 and P8 pass on every
  leg; the switch acks are exact (+1.40 / +1.55 s in, +1.31 / +0.64 s back).
- Misses by the letter, explained, not VECTOR defects: 2b's P2 (0.82 fps
  both rounds) and P3 from the FHSS acquisition at seq 58 (A7); 2a's P3 in
  round 3 (1.3 % vs a 0.3 % control; the same control lost 6.5 % a week
  earlier, A15).

Before range-edge legs: A11 (FHSS authority inactive at 1 fps — firmware),
A4 (encoder time → 2 fps, which also removes A7/A11 on the bench), A8 (relabel
sensitivity on real footage), and the VECTOR_SCENE.md amendments listed in
the fix PR.

Fix PR: `rs13-vector-encoder-sync-fix` @ `ac199b1a`.
