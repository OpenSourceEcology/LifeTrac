# RS-13.1 — VECTOR (codec 6) first bench legs (<YYYY-MM-DD>)

<!-- Copy to bench-evidence/RS_13_vector_scene_<date>/RESULTS.md and fill every
     <placeholder>. Procedure and pass criteria: firmware/x8_lora_bootloader_helper/
     bench_tools/RS13_VECTOR_LEG.md — run its "Every session, every run"
     checklist (the RS-13.1 lessons, anomaly ids A1-A19 of
     bench-evidence/RS_13_vector_scene_2026-09-26/RESULTS.md) and use P6 / P8
     as amended 2026-10-10 (its "Amendments 2026-10-10" section). Keep the
     section order: readers of the other RS_* records expect it. The range-edge
     session has its own template, at the end of RS13_RANGE_EDGE_LEG.md. -->

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

## RF setup

<!-- Added 2026-10-22. Fill before leg 1 and check again at the start of every
     later session: a physical change between sessions (an antenna swapped, a
     board moved or turned, a cable or adapter added) leaves no trace in the
     logs, so it cannot be reconstructed afterwards. -->

| item | base 2D0A1209DABC240B | tractor 2E2C1209DABC240B |
|---|---|---|
| antenna part / model | <maker, part number, gain dBi> | |
| cable / adapter | <cable type and length, connectors, every adapter in the path; or "antenna on the board connector"> | |
| position | <where on the bench or in the room, height above the surface, distance to walls and metal> | |
| orientation | <antenna axis (vertical / horizontal, tilt), which way the board faces> | |

Placement and separation: <antenna-to-antenna distance (cm), line of sight or
what is between the boards, metal, the PC, other radios nearby and their state>.
Unchanged since <previous session, date>? <yes | no — what changed>.
Photo: `legs/rf_setup_<date>.jpg` <both boards and antennas in one frame, or one
per board>.

## Pass rule (written before leg 1)

<!-- Added 2026-10-22. Write this section before the first leg flies, and do
     not change it afterwards: a rule chosen after the numbers are seen is not
     a test. One row per criterion that compares VECTOR with mono_g4, each with
     its own metric, margin and bound. A two-sided Fisher p ≥ 0.05 is not a
     pass rule: it passes whenever the sample is too small to tell (390/400
     against 396/400 gives p = 0.176). Count per arm (frames or probes) for
     80 % power at a one-sided 95 % bound (normal approximation): true rates
     98 % / 98 % need 606 / 270 / 152 for a 2 / 3 / 4 pp margin, 97 % / 97 %
     need 900 / 400 / 225; the score bound is a little stricter (about 25 to 50 more
     per arm for the same power: 654 / 306 / 186 and 926 / 424 / 255). A 300 s leg at 2 fps publishes about 600
     frames, but frames are lost in bursts, not one at a time, so a pooled
     frame count overstates the sample and the bound is optimistic: the
     interleaved legs (A15) are the check on it. These legs fire no PROBE;
     command delivery (§8.6) is scored on -ReactiveFire 1 -ProbeEcho 0 legs,
     recorded with the range-edge template (a 300 s DTS leg fired about 210
     probes on 2026-10-19). -->

| item | value |
|---|---|
| P3 frame loss | frames received / expected per codec, pooled over each leg's `capture seq gaps` line (`rs12_leg_report.py --capture`; expected = received + lost): VECTOR legs <2a, 2a′> against control legs <2c, 2c′>, VECTOR minus mono_g4. Margin <PP> percentage points: NOT WORSE when the one-sided 95 % lower bound (Newcombe hybrid score) is above −<PP> pp. Bound, run in `bench_tools`: `py -3 -c "import rs13_probe_score as s; print(*s.margin_rule((<rx V>, <exp V>), (<rx M>, <exp M>), <PP>, 'P3 frame loss'), sep='\n')"` (its lines say delivery and probes: read frames) |
| command delivery (§8.6) | <n/a: these legs fire no PROBE> — only on legs flown with `-ReactiveFire 1 -ProbeEcho 0`: tractor PROBE seqs that the base sent / base PROBE TX per codec, margin <PP> pp, `py -3 rs13_probe_score.py --margin <PP> <archive> [<archive> …]` over every such leg of one profile |
| legs per codec | <n> |
| frames per leg | <expected per leg, e.g. ~600 at 2 fps over 300 s>; per arm <legs × per leg>, against the <n> per arm the margin needs at the expected rates |
| interleaving order | <e.g. mono_g4, VECTOR, mono_g4, VECTOR, …; the flight order below follows it> |
| kit commit | `<sha>` (the harness and bench kit every leg flies with) |
| `-StopDrainS` | <s>, the same on every leg the rule pools (params `stop_drain_s`; `leg_prep.sh` writes 4 s on FHSS, 2 s on DTS, `--stop-drain S` to change it) |
| `log_window` | `after_stop` on every leg (params). A leg that is not (`after_stop_unverified`, or absent) is re-flown, or dropped under this rule: <rule> |
| a leg whose base never locks (FHSS) | **PROPOSAL (2026-10-24), not agreed yet.** The operator agrees to it, changes it or strikes it before leg 1, and this cell says which. Proposed: a leg whose base never locks the FHSS grid, as flagged by the acquisition check, is void and is re-flown once, in the same slot of the interleaving order (the same codec, before the order goes on). The flag is the leg report's `!! ACQUISITION FAILED` (also at the end of its first line): no frame decoded within 120 s (4 scan attempts × 30 s) of the base's profile-1 activation. `rs13_probe_score.py` prints `!! base ACQUISITION FAILED` in that leg's block by the same rule. The command-delivery pool is then scored with `py -3 rs13_probe_score.py --margin <PP> --void-unlocked <every archive, the void leg's included>`, which lists the voided leg; the P3 sums leave it out. A second failure in a row ends the round for diagnosis: park, record, fly nothing more. A leg whose acquisition reads "not determined" or n/a is not void. `bracket_deltas.py`'s `!! BASE NEVER LOCKED` is a cross-check, not the trigger. The 2026-10-24 round had no such clause and stays scored as written, with its leg 6 (2b_f3, 4/5 probes from a base that never locked) counted. Agreed: <date, time, the operator's words \| changed to: … \| struck> |
| written | <date, time, by whom — before leg 1> |

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
Boot-log check per pass (`legs/step1_<bw>_<scene>_bootcheck.txt`) and the pulled
`legs/step1_camera_service_<bw>_<scene>.log`: `byte_budget=<203 | 243> B` and no
`VECTOR budget … until link_budget` line; `legs/step1_pass.sh` ended `done` (exit 0).
Exit 6 = camera_service held VECTOR at the cold-start clamp, or its camera log with its
byte_budget line did not come back: not a valid pass at the profile's budget, re-run it
with a new `--suffix`.

Final `scene:` lines (from `legs/step1_tractor_*.txt`):

```
<paste>
```

Stage p50 ms from `tractor-log` (bw250 landscape): `<resize= l0= l1= l3= temporal_pack=>`.

Epoch starts per cause from `tractor-log`'s `counters` line, per pass (exact, A8):
`<bw250 moving: first forced horizon safety relabel>`, `<bw250 landscape: …>`, `<bw500 landscape: …>`.

<PASS | FAIL per the step-1 table in the procedure; if the encoder time failed,
say that the radio legs ran at 1 fps — and that the FHSS leg's P2/P3 are then
read from lock (A7) and the tractor is not FHSS time authority (A11)>

## Step 2 — radio legs

### Legs

| leg | profile | boot mode | harness archive | log_window | stop_drain_s | base capture | brackets | duration | verdict |
|---|---|---|---|---|---|---|---|---|---|
| 2a | 2 (DTS) | vector | `radio_monitor_<stamp>_<sha>` | <`after_stop`> | <s> | `legs/leg2a_base.jsonl` | `legs/leg2a_pre_*`, `legs/leg2a_post_*` | 300 s | |
| 2b | 1 (FHSS) | vector | | | | | | 300 s | |
| 2c | 2 (DTS) | mono_g4 (control) | | | | | | 300 s | |
| 2d | 2 (DTS) | mono_g4 → vector @ T+60 s → mono_g4 @ T+180 s | | | | | | 300 s | |

`log_window` and `stop_drain_s` are copied from each archive's `params.txt`
(2026-10-22). `after_stop`: the harness read both daemon logs after both
daemons stopped. `after_stop_unverified`: a daemon stop, or a daemon log's
SIGTERM shutdown line, was not confirmed; copy `log_window_detail` beside
it. Absent: an older harness read them while the daemons ran. The pass rule
needs `after_stop` on every leg, and every leg should carry its
`-StopDrainS`.

Flight order: <e.g. 2c, 2a, 2c′, 2a′, 2b, 2d — interleave control and VECTOR
legs if P3 compares them leg-for-leg (A15)>.
Harness line per leg: <paste each `.\run_live_radio_monitor.ps1 ...` command>.
Scene check per leg: <leg: x % over 10 s> (`legs/leg<X>_scene.{txt,jpg}`; a leg
below 8 % is aborted, not flown).
Channel / spot-check: <frequency, survey result, time before the leg; the `-ForceFrfHz` on every DTS line and the `FRF readback:` line from both daemon logs. `-ForceFrfHz 0` means 915.000 MHz, the RS-11.6 emitter's channel (RS-13.1 A19)>.
Radios parked between legs: <`PARK_OK` × 2 per leg after the post-brackets and
reports; every `PARK_TRANSIENT` and its re-park ~60 s later
(`legs/leg<X>_park.txt`); end of session `radio_state.py` both boards>.

### Numbers

| # | criterion | 2a | 2b | 2c (baseline) | 2d | pass? |
|---|---|---|---|---|---|---|
| P1 | dry-run checks on the base capture (`RESULT:` line as written; failing check names; `store:` line orphans / digest mismatched / resync; final `digest_ok`) | | | n/a | window checks | |
| P2 | frames published / s (`published frame_id` lines ÷ 300; a 1-fps FHSS leg also from lock: `n_frag` ÷ `span` of `frag_gap_report.py`, A7) | | | | n/a | |
| P3 | loss: `capture seq gaps` line of `rs12_leg_report.py --capture` (the loss of record, A14; 1-fps FHSS: from lock + the `counting from seq 1` figure, A7; since 2026-10-24 the head line places the first frame heard on the tractor's TX log, so after a u8 seq wrap it names the tractor's frame #N, and it reads AMBIGUOUS, with no counting figure, when it cannot be placed; the `tractor TX log:` line under it gives the share of the tractor's frames on air that never reached the capture, head and tail included); the report's `shared-window loss` line (the `loss … [whole logs]` line is the old whole-log figure); Δtx_ok ↔ Δrx_ok; comparison basis (interleaved legs or the margin stated before the legs, A15) | | | | n/a | |
| P4 | max fragments per frame (`done (pipelined): K fragments ok`: max K, count of K = 1 lines, ABORTED lines) | | | n/a | window | |
| P5 | lock-loss gaps > 3 s (`frag_gap_report.py`; the gaps run from the first fragment to the last, and since 2026-10-24 its `head:` and `tail:` lines give the silence before and after them) | n/a | | n/a | n/a | |
| P6 | switch: first ack JSON; publish stamp → first codec-6 payload (s) vs the bound incl. the encoder p95 (A9); return ack JSON; return command on air (`command TX … (on air)`, base log) and codec-1 resumed after (s) vs (1 + n) camera periods + 1 fragment airtime, n = codec-6 frames received after the on-air line (amended 2026-10-10, A18); stamp and publish in one board call (A13) | n/a | n/a | n/a | | |
| P7 | link_stats `rx_codec_name` | | | n/a | | |
| P8 | on-air encoder ms p95 vs the budget at the leg's camera rate (350 ms at 2 fps; amended 2026-10-10, A17); vs step 1 (%) and tx daemon `drop_full` / `drop_stale` for information | | | n/a | | |

FHSS time authority (A11): tractor `tx_stream_streak_max` in the 2b
post-bracket: <n> (≥ 8 = authority reached; < 8 expected at 1 fps).
FHSS acquisition (2b; since 2026-10-24): the leg report's `acquisition:` line
<first frame decoded N s after the base's profile-1 activation, within | past
the 120 s budget>; base `FAULT_URC: code=0x0D` lines in `rx_daemon.log`
<none | per line: name, attempt, final, hw_suspect>; `bracket_deltas.py`
`!! BASE NEVER LOCKED` <no | yes>. A failed acquisition goes by the pass rule's
"never locks" row as it stood at leg 1 (agreed, changed or struck).
P3 basis: <interleaved: pooled control x / y vs VECTOR x / y | margin stated
before the legs (the "P3 frame loss" row of the "Pass rule (written before
leg 1)" section above): the bound and verdict line>.

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
- <WARNING `FAULT_URC:` lines on either board (2026-10-24 daemons): code, name,
  sub and log time; count them on the `FAULT_URC: code=0x.. sub=0x.. name=`
  lines only, since the same URC can also leave a legacy line>

### GO / NO-GO

<GO | NO-GO> — <the deciding rows>. Next: <RS-13.2 desk check / re-run of leg X after fix Y>.
