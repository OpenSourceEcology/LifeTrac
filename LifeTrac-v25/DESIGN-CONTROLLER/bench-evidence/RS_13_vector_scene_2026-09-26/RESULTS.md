# RS-13.1 — VECTOR (codec 6) first bench legs (2026-09-26)

**Status: paused after step 1 at the operator's request (2026-09-27 ~00:55 UTC);
radio legs 2a–2d not run; radios parked, `0x80` on both, re-checked read-only
(`legs/session_end_park.txt`). Verdict: NO-GO for RS-13.2. Step 1 failed on a
loss-free path: the encoder-time criterion (p95 395–515 ms against ≤ 350) and the
base/encoder sync checks (`digest` on all three passes, `no_orphans` on both
landscape passes; two encoder-side defects, Anomalies A1 and A2). The radio legs
that GO requires were not run.**

## Software under test

| item | value |
|---|---|
| PR / branch / SHA | #135 `rs13-vector-phase1` @ `ebd9204b` (merged as `dd70cda7`), design PR #129 @ `f32a9ed2` (merged as `65869517`); everything on the bench is from `main` `65869517` |
| tractor image | `lifetrac-tractor-x8:latest` id `2727dfd36f9f` (755 MB), built 2026-09-26 19:49–19:55 UTC from `65869517` `firmware/tractor_x8/` — **built natively on the base X8 (aarch64), not on the PC** (the PC has no Docker; the tractor is offline and cannot pip-install), then `docker save` → PC → `adb push` → `docker load -i` on the tractor. The tractor's previous image (`9bfbbc8d06cb`, 2026-05-26, no numpy/OpenCV/encoder) is kept as `lifetrac-tractor-x8:pre-rs13`. Resolved wheels: numpy 2.4.6, **opencv-python-headless 5.0.0.93** (`requirements.txt` allows `>=4.9.0`; CI pins 4.14.0.94 in `base_station/requirements-dev.txt`), Pillow 12.3.0, paho-mqtt 2.1.0. Step 0 smoke: `<numpy x cv2 y encoder VectorEncoder>` (`legs/step0_image_smoke.txt`) |
| encoder tests in this image | `test_vector_encoder` + `test_vector_interop` + `test_vs1_codec_parity_sil` run inside `2727dfd36f9f` on the base: **46 tests, OK, 0 skipped** (python 3.11.16 aarch64, numpy 2.4.6, cv2 5.0.0) — `legs/step0_encoder_tests_in_image.txt`. The cv2-gated classes ran, so the Phase 1 encoder is exercised under OpenCV 5.0, not only the CI's 4.14. |
| base image | `lifetrac-v25:latest` id `4623980c2dac` (deployed from `main` `d3751286`, 2026-09-15; the harness pushes the current `base_station/` code at every launch) |
| bench files | harness push at launch (`camera_service.py`, `image_pipeline/`, `x8_image_pipeline/`, `lora_proto.py`, `paho/`) + `vector_dry_run.py` @ `65869517` (md5 `cda4620a…` on both boards) |
| dials | `LIFETRAC_VECTOR_DETAIL=80` (band V0), camera 2 fps, `LIFETRAC_KEYFRAME_COPIES` default (1), `LIFETRAC_WEBP_QUALITY` default (55) |
| scene | **moving**: video content on the PC screen, started by the operator (scene check `legs/step1_scene_check_moving.txt`: two frames 1.2 s apart differ); **landscape still**: Windows 11 "Sunrise" wallpaper `C:\Windows\Web\Wallpaper\ThemeC\img29.jpg` (3840×2400; sky, snowy mountains, conifer treeline, lake, rocky shore), fullscreen in a separate Firefox kiosk instance (`--no-remote`, throwaway profile, `object-fit: cover`), put up by Claude at the operator's request; after AE settled two frames 1.88 s apart differ in 2.0 % of pixels (moving content: 20 %) — `legs/step1_scene_check_landscape.txt`, camera's view `legs/step1_scene_landscape_camera_frame.jpg` |

## Firmware on the boards

Unchanged for RS-13.1 (VECTOR is host-only on the strict path); recorded so
the counters in the brackets are attributable.

| board | build | md5 | flashed |
|---|---|---|---|
| base 2D0A1209DABC240B | not re-read: no radio leg was run, and nothing was flashed this session (the brief forbids flashing) | — | — |
| tractor 2E2C1209DABC240B | same | — | — |

Health probes at bracket time: not taken (no radio leg). The only radio contact this
session was the session-end `radio_state.py` / `radio_park.py` in `legs/session_end_park.txt`
(both boards were already `0x80 SLEEP` before the park).

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

## Step 2 — radio legs

### Legs

| leg | profile | boot mode | harness archive | base capture | brackets | duration | verdict |
|---|---|---|---|---|---|---|---|
| 2a | 2 (DTS) | vector | `radio_monitor_<stamp>_<sha>` | `legs/leg2a_base.jsonl` | `legs/leg2a_pre_*`, `legs/leg2a_post_*` | 300 s | |
| 2b | 1 (FHSS) | vector | | | | 300 s | |
| 2c | 2 (DTS) | mono_g4 (control) | | | | 300 s | |
| 2d | 2 (DTS) | mono_g4 → vector @ T+60 s → mono_g4 @ T+180 s | | | | 300 s | |

**Not run.** After step 1 the operator chose to turn the radios off and regroup
rather than run the legs at 1 fps. The tables below are left unfilled on purpose.

### Numbers

| # | criterion | 2a | 2b | 2c (baseline) | 2d | pass? |
|---|---|---|---|---|---|---|
| P1 | dry-run checks on the base capture (`RESULT:` line; failing check names) | | | n/a | window checks | |
| P2 | frames published / s (`published frame_id` lines ÷ 300) | | | | n/a | |
| P3 | fragment loss: raw loss (`rs12_leg_report.py`), Δtx_ok ↔ Δrx_ok | | | | n/a | |
| P4 | max fragments per frame (`done: K fragments ok`, max K) | | | n/a | window | |
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
- [ ] CONFIRM / DIGEST carousel over 5 min (digest checks > 0, 0 mismatches)
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

### GO / NO-GO

**NO-GO**. Deciding rows:
- Step 1 `digest` failed on every pass: 2/247, 97/250 and 217/250
  mismatched on a loss-free path.
- `no_orphans` failed on both landscape passes (33 each).
- Encoder-time p95 was 514.7 / 432.5 / 395.3 ms, against ≤ 350.
- The loss-free sync failures are encoder-side (A1 dead zone vs the store's
  20-frame TTL; A2 same-hash redefine resets the mirror offset); the store
  follows the spec.
- Step 2 was not run.

Next steps:
1. Fix A1/A1b/A1c/A2 in the encoder, with the three step-1 captures as
   replay fixtures (each must replay `RESULT: PASS`).
2. Address A3/A4 (encoder time).
3. Re-run step 1.
4. Run legs 2a–2d.
