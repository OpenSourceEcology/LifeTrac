# RS-13.1 — VECTOR (codec 6) first bench legs (<YYYY-MM-DD>)

<!-- Copy to bench-evidence/RS_13_vector_scene_<date>/RESULTS.md and fill every
     <placeholder>. Procedure and pass criteria: firmware/x8_lora_bootloader_helper/
     bench_tools/RS13_VECTOR_LEG.md. Keep the section order: readers of the other
     RS_* records expect it. -->

**Status: <in progress | complete>. Verdict: <GO | NO-GO> for RS-13.2 —
<one sentence: which legs passed, which criterion failed if any>.**

## Software under test

| item | value |
|---|---|
| PR / branch / SHA | #135 `rs13-vector-phase1` @ `<sha>` (design PR #129 @ `<sha>`) |
| tractor image | `lifetrac-tractor-x8:latest` id `<docker images -q>`, built <date> from `<sha>`; step 0 smoke: `<numpy x cv2 y encoder VectorEncoder>` (`legs/step0_image_smoke.txt`) |
| base image | `lifetrac-v25:latest` id `<docker images -q>` |
| bench files | harness push at launch (`camera_service.py`, `image_pipeline/`, `x8_image_pipeline/`, `lora_proto.py`, `paho/`) + `vector_dry_run.py` @ `<sha>` |
| dials | `LIFETRAC_VECTOR_DETAIL=80` (band V0), camera 2 fps, `LIFETRAC_KEYFRAME_COPIES` default (1), `LIFETRAC_WEBP_QUALITY` default (55) |
| scene | <moving content on the PC screen (which video) / landscape still> |

## Firmware on the boards

Unchanged for RS-13.1 (VECTOR is host-only on the strict path); recorded so
the counters in the brackets are attributable.

| board | build | md5 | flashed |
|---|---|---|---|
| base 2D0A1209DABC240B | <PR # @ sha, build flags> | `<md5>` | <date> |
| tractor 2E2C1209DABC240B | <PR # @ sha, build flags> | `<md5>` | <date> |

Health probes at bracket time: <`rs116_health_probe.py` one-liners, both boards>.

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

Final `scene:` lines (from `legs/step1_tractor_*.txt`):

```
<paste>
```

Stage p50 ms from `tractor-log` (bw250 landscape): `<resize= l0= l1= l3= temporal_pack=>`.

<PASS | FAIL per the step-1 table in the procedure; if the encoder time failed,
say that the radio legs ran at 1 fps>

## Step 2 — radio legs

### Legs

| leg | profile | boot mode | harness archive | base capture | brackets | duration | verdict |
|---|---|---|---|---|---|---|---|
| 2a | 2 (DTS) | vector | `radio_monitor_<stamp>_<sha>` | `legs/leg2a_base.jsonl` | `legs/leg2a_pre_*`, `legs/leg2a_post_*` | 300 s | |
| 2b | 1 (FHSS) | vector | | | | 300 s | |
| 2c | 2 (DTS) | mono_g4 (control) | | | | 300 s | |
| 2d | 2 (DTS) | mono_g4 → vector @ T+60 s → mono_g4 @ T+180 s | | | | 300 s | |

Harness line per leg: <paste each `.\run_live_radio_monitor.ps1 ...` command>.
Channel / spot-check: <frequency, survey result, time before the leg>.
Radios parked between legs: <`PARK_OK` × 2 per leg, or the transient noted>.

### Numbers

| # | criterion | 2a | 2b | 2c (baseline) | 2d | pass? |
|---|---|---|---|---|---|---|
| P1 | dry-run checks on the base capture (`RESULT:` line as written; failing check names; `store:` line orphans / digest mismatched / resync; final `digest_ok`) | | | n/a | window checks | |
| P2 | frames published / s (`published frame_id` lines ÷ 300) | | | | n/a | |
| P3 | fragment loss: raw loss (`rs12_leg_report.py`), Δtx_ok ↔ Δrx_ok | | | | n/a | |
| P4 | max fragments per frame (`done (pipelined): K fragments ok`: max K, count of K = 1 lines, ABORTED lines) | | | n/a | window | |
| P5 | lock-loss gaps > 3 s (`frag_gap_report.py`) | n/a | | n/a | n/a | |
| P6 | switch: first ack JSON; publish stamp → first codec-6 payload (s); return ack JSON; codec-1 resumed after (s) | n/a | n/a | n/a | | |
| P7 | link_stats `rx_codec_name` | | | n/a | | |
| P8 | on-air encoder ms p95 vs step 1 | | | n/a | | |

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
- [ ] CONFIRM / DIGEST carousel over 5 min (digest checks > 0; mismatches recorded — resync 0, or each resync ended by the next epoch start)
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

- <clock alignment between boards; what P6 timing rests on>
- <n = 1 caveats; what a static scene hides>

### Anomalies for the record

- <anything the logs show that the criteria do not cover>

### GO / NO-GO

<GO | NO-GO> — <the deciding rows>. Next: <RS-13.2 desk check / re-run of leg X after fix Y>.
