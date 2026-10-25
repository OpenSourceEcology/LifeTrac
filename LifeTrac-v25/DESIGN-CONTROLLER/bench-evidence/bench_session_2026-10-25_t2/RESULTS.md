# Bench session 2026-10-25 (T2) — FHSS cold-start set on the RS-12.21 image

**Status: flown 2026-10-25 17:43-18:56Z, PASS.** 10 of 10 cold starts
acquired (first frame 11.0-62.4 s after activation, median 11.1 s), none
stranded, every park `PARK_OK` on the firmware path, both radios `0x80` at
the end-of-session check (18:58:35Z / 18:58:40Z). The plan and pass rule
below were written before leg 1 and are unchanged.

GO 2026-10-25 about 15:20Z: "Feel free to merge when you are confident.
Also it is clear to GO on the next test". The next test in
`FHSS_SCAN_FAILED_OPTIONS.md` section 6.1 that needs no new tooling is T2,
a set of about 10 cold starts.

Both L072s run bench `0daae279` (flashed earlier today,
[bench_session_2026-10-25](../bench_session_2026-10-25/RESULTS.md)). Kit:
main `5f84b748`.

## What T2 measures

Each leg's daemon start activates profile 1 on both boards, which resets
the base's scan to a cold start (BOOT -> BEGIN_SCAN at channel 0). T2 runs
ten such starts and records, per start, how the base acquired the
tractor's hop grid:

- the first decoded frame's time after the base's profile activation (the
  report's `acquisition:` line);
- the verdict (`confirmed acquisition: ACQUIRED / NOT DETERMINED` or
  `!! ACQUISITION FAILED`), read with the restart image (cap `0xFF`);
- any base `FAULT_URC code=0x0D name=RX_SCAN_FAILED` lines (attempt,
  final) before the lock.

## Legs (written before leg 1)

Ten legs of 180 s on profile 1 (`image_bw250`), 2 fps,
`-ReactiveFire 1 -ProbeEcho 0 -TxBatch 0 -StopDrainS 4` (`leg_prep.sh`
defaults), alternating the two workloads the memo's walker model
separates (`mono_g4` and VECTOR frames have different airtime):
`2c_t2a`, `2b_t2b`, `2c_t2c`, `2b_t2d`, `2c_t2e`, `2b_t2f`, `2c_t2g`,
`2b_t2h`, `2c_t2i`, `2b_t2j`. Each leg is parked by `leg_post.sh`.

## Pass rule (written before leg 1)

| item | rule |
|---|---|
| **no stranding (the fix)** | Every start reaches `confirmed acquisition: ACQUIRED` within its leg. A start that never locks, or a `!! BASE NEVER LOCKED` on this image, is a FAIL of the RS-12.21 fix: park, stop the round, read the base's 0x0D lines. |
| late starts | A start with no frame by 120 s must show the base's final `0x0D` repeating about every 30 s and then a lock (option (b)); it is recorded as a late acquisition, not a failure. |
| cold-start rate | Descriptive only: the count of starts without a frame by 120 s, against the SIL (1.3 % of cold starts at 2 fps with the 2 s dwell) and the old image (walker model 7-9 %, archive 1 of 27 legs). Ten starts cannot separate those rates statistically. |
| park | every leg-end park `PARK_OK` on the firmware path; the end-of-session `--check-only`, at least 2 min after the last park, reads 0x80 on both boards |
| kit | per leg: `LOG_WINDOW after_stop`, tractor reset landed, deploy md5 ok |
| privacy | the bench video window in front for every scene check (`scene_check.sh` takes no frame otherwise); before the evidence is committed, the base captures are rendered and checked for non-bench content |

## Flight log

| # | leg | workload | base activation (Z) | first frame | verdict | base `0x0D` | seq loss | PROBE | park |
|---|---|---|---|---|---|---|---|---|---|
| 1 | 2c_t2a | mono_g4 | 17:44:08 | 11.0 s | ACQUIRED | none | 0/365 | 124/124 | PARK_OK fw ×2 |
| 2 | 2b_t2b | VECTOR | 17:51:48 | 11.1 s | ACQUIRED | none | 0/364 | 123/123 | PARK_OK fw ×2 |
| 3 | 2c_t2c | mono_g4 | 17:59:02 | 26.5 s | ACQUIRED | none | 0/333 | 115/115 | PARK_OK fw ×2 |
| 4 | 2b_t2d | VECTOR | 18:06:08 | 11.1 s | ACQUIRED | none | 0/361 | 124/124 | PARK_OK fw ×2 |
| 5 | 2c_t2e | mono_g4 | 18:13:37 | 62.4 s (locked in attempt 2) | ACQUIRED | attempt 0, 1 (not final; `warm=1 hw_suspect=1 crc_seen=0`), 30.0 s apart | 0/260 | 89/89 | PARK_OK fw ×2 |
| 6 | 2b_t2f | VECTOR | 18:21:00 | 11.1 s | ACQUIRED | none | 0/366 | 124/125 | PARK_OK fw ×2 |
| 7 | 2c_t2g | mono_g4 | 18:28:32 | 26.4 s | ACQUIRED | none | 0/332 | 114/114 | PARK_OK fw ×2 |
| 8 | 2b_t2h | VECTOR | 18:35:44 | 11.1 s | ACQUIRED | none | 1/359 | 122/122 | PARK_OK fw ×2 |
| 9 | 2c_t2i | mono_g4 | 18:42:52 | 26.9 s | ACQUIRED | none | 1/330 | 112/112 | PARK_OK fw ×2 |
| 10 | 2b_t2j | VECTOR | 18:50:05 | 11.1 s | ACQUIRED | none | 1/365 | 125/125 | PARK_OK fw ×2 |

Columns: "base activation" is the base's profile-1 activation
(`rx_daemon.log`, board time; the base clock is on UTC); "first frame" is
the report's `acquisition:` line; "verdict" is its `confirmed
acquisition:` line; "seq loss" is the capture's seq gaps after the first
frame; "PROBE" is the tractor's received base PROBEs over the base's PROBE
TX (`-ReactiveFire 1`); "park" is the leg-end `radio_park.py` result. Per
leg, `legs/leg<X>_*` and the harness archive
`radio_monitor_20261025_<hhmmss>_5f84b748/` (local time in the name).

## Result against the pass rule

| item | result |
|---|---|
| **no stranding (the fix)** | **PASS.** 10 of 10 starts `confirmed acquisition: ACQUIRED`, the base's first frame 11.0-62.4 s after activation. No `!! ACQUISITION FAILED`, no `!! BASE NEVER LOCKED`; base Δ`clk_demotion_reset` 1 on every leg. |
| late starts | Not exercised: no start went past 120 s, so no final `0x0D` and no option (b) restart on the base. One start (leg 5) needed more than one attempt: base `0x0D` attempt 0 and attempt 1 (not final), 30.0 s apart (L072 uptime 14 458.9 s and 14 488.9 s), then the lock in attempt 2 at 62.4 s. The (b) path on hardware is R2's (the tractor, this morning). |
| cold-start rate | 0 of 10 starts without a frame by 120 s (SIL-E2 at 2 fps: 1.3 %; old image, walker model 7-9 %). Descriptive only, and see "What T2 did and did not sample" below. |
| park | PASS. 20 of 20 leg-end parks `PARK_OK`, `"path": "firmware"`, at the first attempt. End-of-session `--check-only` 2 min 26 s / 2 min 31 s after the last park: `0x80` on both (tractor 18:58:35Z, base 18:58:40Z). |
| kit | PASS. Every leg: `LOG_WINDOW after_stop`; the tractor L072 reset landed (`DONE: BOOT0 LOW, NRST released`); deploy md5 52/52 on both boards; base `L072 VER warm-up ok (attempt 1) cap=0x000000FF`; all seven scorers exit 0; `leg<X>_deltas.txt` 6 PASS, no FAIL or STOP (tractor Δ`clk_demotion_reset` 0, Δ`tx_first_anchor` 1, Δ`fhss_dec_snapped` 0, Δ`fhss_dec_rej_epoch_drift` 0, base `tx_stream_streak_max` 1). No `0x0F` on any leg. No tractor `0x0D` except on leg 5: attempt 0 and 1 (not final) at L072 uptime 34.0 s and 64.0 s, from the tractor's own scan, which has nothing to lock on until the base's PROBEs start (the base's first PROBE at 62.4 s, 18:14:39.5Z). |
| privacy | PASS. `youtube_window.ps1 -Front` exit 0 (bench window in front) before every prep, and every `scene_check.sh` frame taken with it in front. The captures were rendered and checked before commit (below). |

**Acquisition times.** Sorted: 11.0, 11.1, 11.1, 11.1, 11.1, 11.1, 26.4,
26.5, 26.9, 62.4 s (median 11.1 s). By workload: all five VECTOR starts
11.1 s; the five `mono_g4` starts 11.0, 26.5, 62.4, 26.4 and 26.9 s. R1
this morning (`mono_g4`, same image) was 11.0 s; the 2026-10-24 legs on
the old image (500 ms dwell; d2 made it 2 s) took 15-75 s.

**What T2 did and did not sample.** The times fall on three values, not a
spread: 6 starts at 11.0-11.1 s, 3 at 26.4-26.9 s, 1 at 62.4 s. The
harness starts both boards the same way every leg (tractor L072 NRST
reset, then the daemons in a fixed order), so each start begins from
nearly the same relative hop phase; only the `mono_g4` legs moved off it.
T2 therefore shows that these ten starts did not strand on the RS-12.21
image. It does not measure the random-phase cold-start rate the SIL
predicts, and it is no evidence on how often a field start (independent
power-up times) runs past 120 s. Varying the start phase on the bench
would need a random delay between the tractor reset and the base's
profile activation; no harness option does that yet.

## Kit and procedure notes

- **Leg runner.** Each leg ran `leg_prep.sh <tag> image_bw250
  --reactive-fire 1 --duration 180` and the harness wrapper it wrote from
  one PowerShell task, the prep's exit checked before the harness; then
  `link_sample.sh` while frames flowed and `leg_post.sh` after
  `[EVIDENCE] archived to`. The kit scripts were used unchanged (main
  `5f84b748`; `stage_boards.sh` at 17:40Z, 37 files md5-verified per
  board).
- **PC broker log.** The harness's PC broker (`start_mqtt_broker.py`,
  amqtt) logs an unhandled `asyncio.IncompleteReadError` when the base's
  TCP reachability probe connects and closes without an MQTT CONNECT; the
  next harness line is `board->host TCP 1883 OK`. It reached only the
  console of the leg that started the broker (leg 1), once per leg, and
  is not in the committed harness transcripts. Cosmetic. The broker was
  stopped after the round.
- **End-of-session check timing.** The first two `--check-only` runs
  (18:57:03Z and 18:57:26Z, in `legs/power_up_guard_2026-10-25_t2.log`)
  came 54 s and 77 s after the park ended (18:56:09Z): too early to count
  for the 2 min rule. Both read `0x80`. The third run (18:58:31Z) is the
  session's record.
- **Bench video content.** The fresh bench Firefox profile (2026-10-25)
  has no ad blocking, so the bench window played YouTube ads, some of them
  long captioned interview videos, rather than the opened train video
  for most of the round. Leg 1's scene check (git-ignored) shows a
  regional retailer's ad with street addresses, which would hint at the
  bench's location if it reached a committed capture. None did (checked
  below). Suggested follow-up: give the bench profile an ad blocker or
  play a local video file, so no geo-targeted content can reach the
  camera.

## Privacy check (before commit)

- The ten scene jpgs are git-ignored (`*.jpg`).
- The five `mono_g4` captures were rendered through the base station's
  own path (`FragmentReassembler` -> `Canvas.apply`), saving the canvas
  before any tile write would overwrite one not yet captured, plus the
  last frame: 27 canvases and 5 pre-keyframe tile images, every tile write
  covered (0 overwritten unrendered). They show the bench monitor's
  YouTube content: captioned interview footage under the ad overlay.
- The five VECTOR captures (1 813 frames, 0 bad) were fed through the base
  station's `VectorSceneStore` as `web_ui` does and its snapshot drawn
  every 15 frames plus after the last: 124 images of abstract coloured
  polygons (VS1 carries about 35 shapes on a 4-8 px grid; no legible text
  is representable).
- No addresses, phone numbers, location names or anything from outside
  the bench window in any render. A text scan of the 329 new files found
  no email address, phone number, key or credential.
