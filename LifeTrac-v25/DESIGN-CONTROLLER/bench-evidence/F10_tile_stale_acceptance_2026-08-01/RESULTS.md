# F10 on-air acceptance — 0x6C stale-tile report (2026-08-01)

**SHA under test:** main @ PR #88 merge (F10 + RS-5.8 + F9 flip; F10 code
commit `cb71ee37`, test-stub fix `aedb1d41`).
**Verdict: PASS** — full chain verified on air; one pre-existing non-F10
defect found and filed (Canvas.apply silent desync, see §6).

## 1. Setup

- Harness: `run_live_radio_monitor.ps1 -TxFeed camera -RegProfile 2
  -DurationS 780 -SynthFps 2 -KfRequestDisable 0 -ProbeEcho 0 -Archive`
  (v3 pipeline depth 2, smooth pacing, TrainGapMs 40 — the pinned operating
  point). Real USB camera, encode-to-fit in the loop, link budget 243 B/frame
  (1 fragment) at DTS BW500.
- camera_service: Method C, SWEEP_STEP 2 @ 2 fps → **sweep rotation 24 s**.
- web_ui (production F10 scanner) run on the base board in `lifetrac-v25:latest`
  (container `f10_webui`, uvicorn, base-local broker), with
  `LIFETRAC_TILE_STALE_AFTER_MS=30000` — the horizon must exceed the sweep
  rotation (24 s), so the repo default of 20 s would false-positive AT THIS
  BENCH CADENCE; deployments must set it per `n_tiles/(SWEEP_STEP*fps)`.
  `LIFETRAC_TILE_STALE_PERIOD_S=3`.
- Evidence taps: `mosquitto_sub` on the base broker
  (`lifetrac/v25/cmd/tile_stale`, `lifetrac/v25/cmd/req_keyframe`) and the
  tractor bench broker (`lifetrac/v25/tractor/tile_stale`); all timestamps
  below are host/base clock (identical), tractor board clock offset
  ≈ −14 h 03 m 30 s.

## 2. Leg 0 — cold-fill acceleration (PASS)

web_ui started mid-stream with an empty canvas; after its cold-start keyframe
the canvas filled progressively (243 B/frame budget = partial coverage per
frame). Never-arrived tiles (arrived_ms == 0) were reported stale immediately
and front-run by the tractor:

    23:09:02  tile_stale 5e00 ffffffffff11000000c0ffff   (~50 tiles)
    23:09:05  tile_stale 6400 ffffffffff110000000000f8
    23:09:08  tile_stale 6a00 80ffffffff11000000000000
    23:09:11  tile_stale 7000 0000f8ffff11000000000000
    23:09:14  tile_stale 7600 0000000080ff11000000000000  (~11 tiles)
    then silence — converged.

base_seq advanced +6 per 3 s report = 2 fps ✓. Tractor receipts
(`LoRa cmd: TILE_STALE 14 B`) at the matching 3 s cadence — the predicted
14 B body (u16le base_seq + 12 B bitmap for 96 tiles) exactly.

## 3. Leg A — no false positives on a healthy link (PASS)

With threshold (30 s) > sweep rotation (24 s): **zero** tile_stale publishes
23:09:14 → 23:12:14 (first outage) and again 23:17:47 → session end. The
report stream is silent whenever the canvas is actually fresh.

## 4. Leg B — real-loss outage and recovery (PASS, with one bench fault)

23:12:09 rx daemon stopped (docker stop), restarted 23:12:21, publishing
again by 23:12:30 — ~21 s during which the tractor kept transmitting into
the void: **31 frames genuinely lost on air** (web_ui's designed seq-gap
keyframe request fired once: payload "base_seq gap: got 241, expected 210").

Level-triggered behaviour under outage, exactly as designed:
- web_ui kept publishing reports every 3 s while the radio owner was down
  (23:12:14, :17, :20, :23, :26 — stale set GROWING as tiles crossed 30 s);
  those reports died with no subscriber (QoS0, no retry machinery) and were
  superseded by the next one — first post-restart report radiated 23:12:29
  (`command TX opcode=0x6c copy=1/1 OK (on air)`).
- The seq-gap req_keyframe (23:12:23) also died with the daemon down —
  QoS1 to a broker with no consumer attached. The canvas recovered WITHOUT
  any keyframe: repairs arrived tile-by-tile via marks + sweep.

**Bench fault that interrupted recovery (not an F10 defect):** on reconnect
the rx daemon re-radiated the RETAINED encode-mode override — mono_g4
(mode 6), stale state from an earlier bench session — at 23:12:35, and the
bench `lifetrac-v25:latest` image lacks Pillow, so every mono_g4 tile
transcode raised ModuleNotFoundError inside Canvas.apply. Result: canvas
tracked base_seq but applied ZERO tiles (see §6 — a real pre-existing canvas
bug this run exposed). F10 behaved correctly throughout the fault: honest
all-96-stale reports, one 14 B frame per 3 s (≈0.5% duty — the bounded
failure mode), tractor receiving every report.

Republished the override as webp (mode 0, retained) at 23:16:27; camera
switched (`encode_mode -> 0 (full) [lora_cmd]`) and the full canvas
repaired **all 96 tiles in ~24 s** (23:16:26 all-ff → 23:16:50 one tile →
silence), bounded by the 243 B/frame budget:

    23:16:26  tile_stale d700 ffffffffffffffffffffffff    (96)
    23:16:29  tile_stale dd00 fbcfffffff1ffffffffffbda
    23:16:35  tile_stale e900 e00fe8ffff1f00cffffff3d0
    23:16:41  tile_stale f500 e00f2001000f00ccfffff310
    23:16:47  tile_stale 0100 00062001000800000060f310
    23:16:50  tile_stale 0700 000000000000000000008000    (1)
    then silence.

## 5. Surgical repair — lost-repair re-mark and timed partial outage (PASS)

**Natural event:** two tiles (32 and 88) whose mass-repair copies were
themselves lost on air crossed the 30 s threshold at 23:17:44, were
re-marked, and cleared within ≤6 s (two report periods):

    23:17:44  tile_stale 7300 000000010000000000000100    (tiles 32, 88)
    23:17:47  tile_stale 7900 000000010000000000000100
    then silence — both repaired.

**Timed partial outage:** rx daemon stopped 23:28:20, restarted 23:28:27,
publishing ~23:28:30 — web_ui's seq-gap shows exactly **18 frames lost**
("base_seq gap: got 124, expected 106"). Lost tiles crossed the 30 s
threshold progressively; the reports churn (new marks appearing, repaired
marks clearing) from 23:28:30 to a 2-tile report at 23:28:54, then
**silence at 23:28:57 — 27 s from resume to fully fresh**, with individual
marks visibly clearing within 1–3 report periods (3–9 s) of appearing.
Sweep-only counterfactual per tile: up to 24 s after its last (lost) ship.
The re-mark-until-repaired churn is the level-triggered design absorbing
repair loss with no retry machinery.

**Keyframe accounting during the surgical window:** three seq-gap
req_keyframe events (the 18-frame resume gap, plus two SINGLE-frame air
losses at 23:28:45/23:28:51). The rx-side KeyframeRequester throttled them
to two radiated 0x60 commands 17 s apart (10 s min-gap working) and both
converged in 1.0 s. Observation for follow-up (NOT an F10 defect): web_ui's
gap-tolerant apply requests a keyframe on EVERY base_seq gap — including a
single lost delta frame — which F10's stale reporting now makes largely
redundant; each one costs a ~2.4 KB keyframe at this budget. Candidate next
increment: demote/gate the seq-gap keyframe request the way the
reassembly-timeout one was (measured, then default off).

## 6. Findings

1. **PASS — F10 end-to-end**: scan → 0x6C publish → single-copy radiate →
   tractor dispatch → camera_service age-escalation front-run → repair →
   report convergence. No keyframe used for any tile repair.
2. **Reassembly-timeout keyframes stayed off**: `reassembler_timeouts=0`
   throughout; the only req_keyframe events were cold-start and the designed
   seq-gap request. Nothing poked keyframes on the loss path.
3. **Pre-existing bug (filed as its own task, NOT fixed here):**
   `Canvas.apply` catches only `CodecDecodeError` around the tile transcode;
   any other exception (here: ImportError from the lazy PIL import) unwinds
   AFTER `_last_base_seq` is adopted and BEFORE any tile applies — canvas
   silently freezes while appearing to track the stream, and the
   request_keyframe publish path is skipped because the exception unwinds
   through web_ui's ingest. Surfaced only because F10's reports showed a
   canvas that never freshened.
4. **Deployment note:** `LIFETRAC_TILE_STALE_AFTER_MS` default (20 s) is
   below this bench's 24 s sweep rotation; the horizon must be configured
   per deployment as ≥ ~1.25 × `n_tiles/(SWEEP_STEP × fps)`. On this bench:
   30 s. A false positive costs one early tile re-ship (advisory semantics),
   but a systematically low horizon wastes airtime every rotation.
5. **Bench-state hygiene — the retained override lives on TWO brokers.**
   The rx daemon subscribes control topics on both its image broker
   (base-local mosquitto) and the ctrl broker
   (`LIFETRAC_CTRL_MQTT_HOST` = the bench PC, 192.168.1.79); BOTH deliver
   their retained `control/encode_mode_override` on every daemon
   (re)connect. Overwriting only the base copy (23:16:27) looked fixed
   until the next daemon relaunch, when the bench PC's stale retained
   `{"mode": "mono_g4", "ts": 1785619181}` (2026-07-30 vintage) re-flipped
   the camera and reproduced the §4 canvas freeze identically at 23:22:02.
   Both copies now cleared to mode 0 retained. Bench rule going forward:
   retained control state must be cleared on BOTH brokers between
   campaigns; and note the double-radiate is a real (if benign today)
   double-apply hazard of the dual-broker control plane.

## 7. Raw artifacts

- Harness `-Archive` bundle: `bench-evidence/radio_monitor_20260801_182119_0697e5dd/`
  (full tx/rx daemon logs, parameters, SHA).
- `tap_base.log` (this directory) — base-broker tap: every tile_stale +
  req_keyframe with host timestamps, payload hex.
- `tap_tractor.log` (this directory) — tractor-broker tap (received marks).
- Untracked working copies at repo root: `_f10_run1.log`,
  `_f10_tap_base.log`, `_f10_tap_tractor.log`.
- Post-run daemon relaunch (surgical legs) used the identical operating
  point via a scripted docker run — same env as the harness launch.
