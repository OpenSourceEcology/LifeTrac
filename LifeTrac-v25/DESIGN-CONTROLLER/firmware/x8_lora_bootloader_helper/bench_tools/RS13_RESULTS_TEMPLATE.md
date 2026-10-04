# RS-13.1 — VECTOR (codec 6) first bench legs (<YYYY-MM-DD>)

<!-- Copy to bench-evidence/RS_13_vector_scene_<date>/RESULTS.md and fill every
     <placeholder>. Procedure and pass criteria: firmware/x8_lora_bootloader_helper/
     bench_tools/RS13_VECTOR_LEG.md — run its "Every session, every run"
     checklist (the RS-13.1 lessons, anomaly ids A1-A15 of
     bench-evidence/RS_13_vector_scene_2026-09-26/RESULTS.md). Keep the section
     order: readers of the other RS_* records expect it. -->

**Status: <in progress | complete>. Verdict: <GO | NO-GO> for RS-13.2 —
<one sentence: which legs passed, which criterion failed if any>.**

## Software under test

| item | value |
|---|---|
| PR / branch / SHA | #135 `rs13-vector-phase1` @ `<sha>` (design PR #129 @ `<sha>`); encoder fix PR #138 `rs13-vector-encoder-sync-fix` @ `<sha>`; <any further branch under test @ sha> |
| encoder that ran | <the image's `/app` copy, or the pushed `/work` tree (`-w /work … /work/camera_service.py`)>; module line from the container: `<… /work/x8_image_pipeline/encode_vector.py …>`; md5 of the pushed files against the working tree: `<md5s>` |
| staging | `/tmp/lifetrac_strict` on both boards pushed <date, time> (< 5 days before the legs — `systemd-tmpfiles` ages `/tmp` at 5 days); entry count base `<n>` / tractor `<n>` |
| tractor image | `lifetrac-tractor-x8:latest` id `<docker images -q>`, built <date> from `<sha>`; step 0 smoke: `<numpy x cv2 y encoder VectorEncoder>` (`legs/step0_image_smoke.txt`) |
| base image | `lifetrac-v25:latest` id `<docker images -q>` |
| bench files | harness push at launch (`camera_service.py`, `image_pipeline/`, `x8_image_pipeline/`, `lora_proto.py`, `paho/`) + `vector_dry_run.py` @ `<sha>`; leg report `tools/rs12_leg_report.py` @ `<sha>` (with `--capture`) |
| dials | `LIFETRAC_VECTOR_DETAIL=80` (band V0), camera 2 fps, `LIFETRAC_KEYFRAME_COPIES` default (1), `LIFETRAC_WEBP_QUALITY` default (55) |
| scene | moving: the railroad video `https://www.youtube.com/watch?v=B1yUQwpNhJA` in a **normal** Firefox window (separate profile) with the player's fullscreen — no kiosk; landscape: <still, in a normal maximised window>. Scene checks before every run: `legs/<run>_scene.{txt,jpg}` (10 s change per run in the step-1 and legs sections) |

## Firmware on the boards

Unchanged for RS-13.1 (VECTOR is host-only on the strict path); recorded so
the counters in the brackets are attributable.

| board | build | md5 | flashed |
|---|---|---|---|
| base 2D0A1209DABC240B | <PR # @ sha, build flags> | `<md5>` | <date> |
| tractor 2E2C1209DABC240B | <PR # @ sha, build flags> | `<md5>` | <date> |

Health probes at bracket time: <`rs116_health_probe.py` one-liners, both boards;
the whole output kept in `legs/leg<X>_health_{base,tractor}.txt`>.

## Step 0 — image smoke

```
<paste of the docker run ... -c "import numpy, cv2; ..." output>
```

<PASS | FAIL — what was rebuilt if it failed>

## Step 1 — camera-only dry run (tractor, no radio)

| profile | scene | frames | fps_mean | wire p50 / p95 / max | over limit | epoch starts (applied) | store bad / orphans / digest checked-mismatched | encoder ms p50 / p95 / max | pending-epoch lines | checks | verdict |
|---|---|---|---|---|---|---|---|---|---|---|---|
| image_bw250 | moving | | | | | | | | | 10/10 | |
| image_bw250 | landscape | | | | | | | | | | |
| image_bw500 | landscape | | | | | | | | | | |

Before each pass: scene check <moving: x % over 10 s (≥ 8 %); landscape: y %>
(`legs/step1_<bw>_<scene>_scene.{txt,jpg}`); tractor `bench_mqtt` recreated,
retained messages after the recreate: <0> (`legs/step1_bench_mqtt_reset.txt`, A12).

Final `scene:` lines (from `legs/step1_tractor_*.txt`):

```
<paste>
```

Stage p50 ms from `tractor-log` (bw250 landscape): `<resize= l0= l1= l3= temporal_pack=>`.

<PASS | FAIL per the step-1 table in the procedure; if the encoder time failed,
say that the radio legs ran at 1 fps — and that the FHSS leg's P2/P3 are then
read from lock (A7) and the tractor is not FHSS time authority (A11)>

## Step 2 — radio legs

### Legs

| leg | profile | boot mode | harness archive | base capture | brackets | duration | verdict |
|---|---|---|---|---|---|---|---|
| 2a | 2 (DTS) | vector | `radio_monitor_<stamp>_<sha>` | `legs/leg2a_base.jsonl` | `legs/leg2a_pre_*`, `legs/leg2a_post_*` | 300 s | |
| 2b | 1 (FHSS) | vector | | | | 300 s | |
| 2c | 2 (DTS) | mono_g4 (control) | | | | 300 s | |
| 2d | 2 (DTS) | mono_g4 → vector @ T+60 s → mono_g4 @ T+180 s | | | | 300 s | |

Flight order: <e.g. 2c, 2a, 2c′, 2a′, 2b, 2d — interleave control and VECTOR
legs if P3 compares them leg-for-leg (A15)>.
Harness line per leg: <paste each `.\run_live_radio_monitor.ps1 ...` command>.
Scene check per leg: <leg: x % over 10 s> (`legs/leg<X>_scene.{txt,jpg}`; a leg
below 8 % is aborted, not flown).
Channel / spot-check: <frequency, survey result, time before the leg>.
Radios parked between legs: <`PARK_OK` × 2 per leg after the post-brackets and
reports; every `PARK_TRANSIENT` and its re-park ~60 s later
(`legs/leg<X>_park.txt`); end of session `radio_state.py` both boards>.

### Numbers

| # | criterion | 2a | 2b | 2c (baseline) | 2d | pass? |
|---|---|---|---|---|---|---|
| P1 | dry-run checks on the base capture (`RESULT:` line as written; failing check names; `store:` line orphans / digest mismatched / resync; final `digest_ok`) | | | n/a | window checks | |
| P2 | frames published / s (`published frame_id` lines ÷ 300; a 1-fps FHSS leg also from lock: `n_frag` ÷ `span` of `frag_gap_report.py`, A7) | | | | n/a | |
| P3 | loss: `capture seq gaps` line of `rs12_leg_report.py --capture` (the loss of record, A14; 1-fps FHSS: from lock + the `counting from seq 1` figure, A7); the report's counter `loss` line; Δtx_ok ↔ Δrx_ok; comparison basis (interleaved legs or the margin stated before the legs, A15) | | | | n/a | |
| P4 | max fragments per frame (`done (pipelined): K fragments ok`: max K, count of K = 1 lines, ABORTED lines) | | | n/a | window | |
| P5 | lock-loss gaps > 3 s (`frag_gap_report.py`) | n/a | | n/a | n/a | |
| P6 | switch: first ack JSON; publish stamp → first codec-6 payload (s) vs the bound incl. the encoder p95 (A9); return ack JSON; return command on air (`command TX … (on air)`, base log) and codec-1 resumed after (s); stamp and publish in one board call (A13) | n/a | n/a | n/a | | |
| P7 | link_stats `rx_codec_name` | | | n/a | | |
| P8 | on-air encoder ms p95 vs step 1 | | | n/a | | |

FHSS time authority (A11): tractor `tx_stream_streak_max` in the 2b
post-bracket: <n> (≥ 8 = authority reached; < 8 expected at 1 fps).
P3 basis: <interleaved: pooled control x / y vs VECTOR x / y | margin stated
before the legs (date and rule, e.g. two-sided Fisher exact p ≥ 0.05)>.

`vector_dry_run.py` summaries (paste the `== vector dry run summary ==` block
of each base capture):

```
<2a>
```

```
<2b>
```

```
<2d>
```

### Path features exercised

- [ ] codec-6 TileDeltaFrame over the strict path as a single 0xFE fragment, profile 2 and profile 1
- [ ] epoch start (K = 1) as the first frame: anchor + LAYER_CLEAR in one frame, F−1 body
- [ ] CONFIRM / DIGEST carousel over 5 min: digest checks > 0 and **zero mismatches unless each mismatch run starts at a recorded frame loss** (a sequence gap in the base capture, `scripts/leg_replay.py`) or at the base joining mid-epoch, and every run recovers (the final `digest_ok` not `False`; `None` = no DIGEST since the last epoch start, so quote `leg_replay.py`'s `last DIGEST checked` line and say whether the last run was confirmed by a DIGEST or only ended by that epoch start). A mismatch with no loss before it is an encoder/store defect even if a later epoch start ends the resync (RS-13.1 A1 would have passed the bare loss rule on bw250)
- [ ] 0x63 mode switch into and out of VECTOR; the ack's quality byte reports the dial of the acked mode
- [ ] base `link_stats` codec reporting for codec 6
- [ ] 0xFD copies path for an epoch start (`LIFETRAC_KEYFRAME_COPIES` > 1 or the tx daemon's auto-copies) — <not exercised | exercised by accident, see Anomalies>

### What these legs did NOT test

- V1–V3 (no policy built; every frame V0 at detail 80)
- self-model, Vector Lab, browser rendering (RS-13.2 desk check)
- range edge / attenuator sweep
- AE/AWB lock (B5), tractor self-select (D-VS6b), the ladder's loss-driven floor (B4)
- <anything else cut on the day>

### Evidence limitations

- <clock alignment between boards; what P6 timing rests on (base clock only;
  the tractor clock has no network time and ran 13 days behind in RS-13.1)>
- <n = 1 caveats; what a static scene hides>
- <scene during the legs: checked before each leg (`legs/leg<X>_scene.*`), not
  during it — any operator use of the PC during a leg>
- <1 fps caveats if the encoder time missed: FHSS acquisition head (A7), no
  time authority (A11)>

### Anomalies for the record

- <anything the logs show that the criteria do not cover>

### GO / NO-GO

<GO | NO-GO> — <the deciding rows>. Next: <RS-13.2 desk check / re-run of leg X after fix Y>.
