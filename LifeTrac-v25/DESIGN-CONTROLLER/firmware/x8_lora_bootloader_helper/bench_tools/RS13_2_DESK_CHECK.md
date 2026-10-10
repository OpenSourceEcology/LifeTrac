# RS-13.2 — VECTOR desk check: the operator console on the production compose, replaying round 4

*Procedure, 2026-10-10. Written without touching a board; a PC rehearsal of
every step that needs no board ran the same day (Appendix A, findings F1–F10).
Input record: [RS-13.1 RESULTS](../../../bench-evidence/RS_13_vector_scene_2026-09-26/RESULTS.md)
(round 4, A16, "Remaining radio tests"). Design: [`VECTOR_SCENE.md`](../../../VECTOR_SCENE.md)
§7.5 (renderer and fail-closed rules), §4.3 (resync, TTL, age styling), §3.5
(epochs and hand-over), §6 (mode integration). TODO: RS-13 "RS-13.2 desk check".*

The desk check is the first look at VECTOR **in the operator's browser**:
RS-13.1 judged every leg from captures and the store's numbers, and web_ui was
never in the loop. Here the rebuilt base image serves the console from the
production compose, and the four round-4 base captures are published back
onto its broker at their recorded timing with
[`tools/vector_replay_publish.py`](../../../tools/vector_replay_publish.py).
Nothing is transmitted: **no radio is used, and the radios stay parked**.

## What it checks, and what it does not

Checks, on real round-4 streams:

1. the renderer and fail-closed rules of §7.5 as the operator sees them: the
   banner, the chips (epoch and anchor age, RESYNC, DIGEST MISMATCH), age
   styling, the epoch hand-over, `BADGE?`, `NO VECTOR DATA`, and that the old
   photo never shows through during VECTOR;
2. the codec switch into and out of VECTOR (leg 2d) and the photo path on
   the `mono_g4` control (leg 2c);
3. that web_ui stays quiet on the uplink while VECTOR is active: no
   `cmd/tile_stale` (`0x6C`) and no `cmd/req_keyframe` (§6, RS-13.1 RESULTS
   "Remaining radio tests");
4. **what A16 looks like on screen**: the two recorded episodes (2a rows
   525–589, 2d rows 174–231) and injected single drops;
5. the operator-facing half of the A16 design decision (accepting a matching
   CONFIRM in resync against the §4.3 tag guard): how visible a ~30 s
   one-shape-short scene is, and whether the RESYNC cue reaches the operator.

Does not check:

- **the radio, the encoder or the A16 fix.** The captures are fixed bytes from
  the round-4 encoder (`4ab58b8d`). An encoder-side A16 fix does not change
  them, so A16 still shows in these replays; that is expected. The fix is
  verified by `scripts/a16_sil.py` and the range-edge session's 0 dB step. If
  the merged fix also changed the **store**, the replays will differ from the
  RESULTS episode lists: record the difference (see Step 5).
- live timing. The replay reproduces the base's capture timing (on-air jitter
  included); web_ui stamps arrival with its own clock and an airtime of 0, as it
  always does. Shape ages are therefore right at `--speed 1` only.
- the self-model (Phase 3, not built), the Vector Lab (§8, not built), the
  degradation ladder (only V0 has flown), and the old-client path of rule 5
  (not implemented, F4).

## Rules (read first)

1. **Radios stay parked; nothing opens `/dev/ttymxc3`.** Do not run any probe
   on either board — not `radio_state.py`, `radio_park.py` or `rs11x_*`: a
   HostLink connect wakes the L072 ([BENCH_RUNBOOK](BENCH_RUNBOOK.md), Board
   facts). Do not touch the tractor at all. The read-only
   `sudo fuser /dev/ttymxc3` on the base (it opens nothing) is the only
   radio-side check, before and after.
2. **Never start `lora_bridge`**, never the video-test compose (its `image_rx`
   opens `/dev/ttymxc3`), never `lifetrac-base*.service` (enabled but failing;
   it would bring up that video-test stack). Start only `mosquitto` and
   `web_ui` (and, optionally, `audit_tail`) through the desk-check override
   [`rs13_2_desk.compose.yml`](rs13_2_desk.compose.yml), which puts
   `lora_bridge` behind a profile nobody enables and turns web_ui's bench radio
   endpoint off (`LIFETRAC_ENABLE_BENCH_RADIO=0`).
3. **The broker is the desk check's own** (`-p lifetrac-desk`, its own
   volumes). web_ui publishes retained `control/encode_mode_override` and
   `control/radio_profile` (`{"profile": 2}`) at every start; they must not
   survive into the next radio session. Tear down with `down -v` (Step 8).
4. In the console, **do not use** the radio-profile selector, Auto, the bench
   radio buttons, E-STOP or the joysticks. With no daemon running nothing goes
   on air, but retained state and audit entries would. The encode-mode
   selector is used once, deliberately (C18), and is undone by Step 8.
5. A normal browser window, never a kiosk or a fully fullscreen browser
   ([bench screen rule](RS13_VECTOR_LEG.md)): the operator must keep control
   of the PC.

## Files

| file | what |
|---|---|
| `tools/vector_replay_publish.py` | publishes a `vector_dry_run.py` capture to a broker at the capture's timing; `--speed`, `--loop`, `--start-row`/`--end-row`/`--start-at-key`, `--drop-rows`/`--drop-every`, `--log`, `--dry-run`. Stdlib + paho only, so it runs inside `lifetrac-v25:latest`. Rows are numbered as the dry-run report and RESULTS number them. Test: `base_station/tests/test_vector_replay_publish.py` |
| `bench_tools/rs13_2_desk.compose.yml` | the compose override of Rule 2 |
| `bench-evidence/RS_13_vector_scene_2026-09-26/legs/leg2{a,b,c,d}_r4_base.jsonl` | the four round-4 base captures (`video/tile_delta`, what web_ui would have ingested) |

## Step 0 — at the PC, before anything touches the base

1. Note the SHA you will deploy (the merged A16-fix SHA when it exists) and
   the SHA of the tool: `git rev-parse HEAD`.
2. Dry-run the tool on the four captures; it reads them and prints the plan,
   with no broker and no waiting:

   ```powershell
   cd LifeTrac-v25/DESIGN-CONTROLLER
   $L = "bench-evidence/RS_13_vector_scene_2026-09-26/legs"
   foreach ($c in "2a","2b","2c","2d") { py -3 tools/vector_replay_publish.py "$L/leg${c}_r4_base.jsonl" --dry-run --quiet }
   ```

   Expect 596 / 558 / 591 / 600 rows and 305.4 / 279.5 / 305.4 / 307.0 s per
   pass.
3. Optional: the PC rehearsal of Appendix A, to learn the screens before the
   session.

## Step 1 — rebuild the base image

The deployed `lifetrac-v25:latest` (`4623980c2dac`, built from `d3751286` on
2026-09-15) **predates every VECTOR web_ui, store and renderer commit** (#135,
#138): its web_ui has no vector store and no renderer, and during VECTOR its
stale-tile worker would publish `0x6C` at least every 10 s (RS-13.1 RESULTS,
"Remaining radio tests"). Rebuild it first:

- with `deploy_base.sh` (this directory, from the bench-kit change) at the
  SHA of Step 0: it copies the tree and builds the image, and starts nothing;
- or, if that script is not in your tree yet, by hand: `git archive` of
  `base_station/ firmware/ Dockerfile docker-compose*.yml deploy/` →
  `scp` the tarball to the base (never `git archive | ssh sudo -S tar`:
  `sudo -S` eats the stream) → extract over
  `/var/rootdirs/opt/lifetrac/DESIGN-CONTROLLER` → write the SHA to
  `DEPLOYED_FROM.txt` there → `docker build --pull=false -t lifetrac-v25:latest .`
  in that directory.

Either way, start and restart nothing. Then check that the image carries the
VECTOR code, without importing web_ui (its import connects to a broker):

```sh
docker images -q lifetrac-v25:latest                       # record the id
docker run --rm lifetrac-v25:latest sh -c \
  'grep -c CODEC_VECTOR /app/base_station/web_ui.py; ls -l /app/base_station/web/img/vector_renderer.js /app/base_station/image_pipeline/vector_scene_store.py'
cat /var/rootdirs/opt/lifetrac/DESIGN-CONTROLLER/DEPLOYED_FROM.txt
```

A count of 0 or a missing file means the old tree: stop.

## Step 2 — stage the desk-check directory on the base

Base access per BENCH_RUNBOOK (Board facts: ssh key, address). From the PC,
in the repository root:

```powershell
$B  = "fio@192.168.1.117"; $K = "$HOME/.ssh/lifetrac_base_ed25519"
$DC = "LifeTrac-v25/DESIGN-CONTROLLER"; $L = "$DC/bench-evidence/RS_13_vector_scene_2026-09-26/legs"
ssh -i $K $B "mkdir -p /tmp/rs13_2/out"
scp -i $K "$DC/tools/vector_replay_publish.py" "$DC/firmware/x8_lora_bootloader_helper/bench_tools/rs13_2_desk.compose.yml" `
    "$L/leg2a_r4_base.jsonl" "$L/leg2b_r4_base.jsonl" "$L/leg2c_r4_base.jsonl" "$L/leg2d_r4_base.jsonl" "${B}:/tmp/rs13_2/"
```

On the base, the two secret files the override points at (the deployed
tree's `./secrets` is neither used nor touched):

```sh
cd /tmp/rs13_2 && umask 077
printf '%s' '<a desk-check PIN, 4-6 digits>' > lifetrac_pin
head -c 16 /dev/urandom > unused_fleet_key   # satisfies compose only; lora_bridge cannot start
```

`/tmp` is tmpfs and ages out after 5 days; stage on the day.

## Step 3 — bring up `mosquitto` + `web_ui`, nothing else

On the base (prefix `sudo` to the docker commands if `fio` is not in the
`docker` group):

```sh
sudo fuser /dev/ttymxc3; echo "uart holders: (expect none above)"
docker ps --format '{{.Names}}\t{{.Image}}\t{{.Status}}'      # expect no leg or compose containers
ss -ltn | grep -E ':(1883|8080) ' || echo "1883 and 8080 free"
cd /var/rootdirs/opt/lifetrac/DESIGN-CONTROLLER
C="docker compose -p lifetrac-desk -f docker-compose.yml -f /tmp/rs13_2/rs13_2_desk.compose.yml"
$C config --services        # must print mosquitto, web_ui, audit_tail -- never lora_bridge
$C up -d --no-build mosquitto web_ui
$C ps
$C logs --tail 30 web_ui    # uvicorn "Application startup complete"
```

If 1883 or 8080 is already taken, find out what holds it before going on;
do not stop a container you did not start without recording it. If
`config --services` lists `lora_bridge`, stop: the override is not applied.

## Step 4 — open the console on the PC

1. A normal browser window at `http://192.168.1.117:8080/login`, PIN from
   Step 2. Record the window's inner size (devtools console:
   `innerWidth + 'x' + innerHeight`): F1 and F2 depend on it. Run the replays at
   the size the operator uses (1280×720 in the rehearsal), and repeat one
   screenshot set at a second size (e.g. 1920×1080).
2. Paste the state recorder into the devtools console. It writes nothing to
   the page and sends nothing; it keeps the store state each `/ws/state`
   snapshot carried, so the RESULTS can quote when the browser saw each
   resync:

   ```js
   window.rs132 = []; addEventListener('lifetrac-state', (ev) => { const s = ev.detail || {}, v = s.vector_scene;
     const sh = v ? (v.layers || []).flatMap(L => L.shapes || []) : [];
     rs132.push({t: Date.now(), mode: s.encode_mode, sd: s.safety_detector, epoch: v && v.epoch, resync: v && v.resync,
       dig: v && v.digest_ok, anchor: v && v.anchor_age_ms, shapes: v ? sh.length : null,
       gt5: sh.filter(x => x.age_ms > 5000).length, gt10: sh.filter(x => x.age_ms > 10000).length,
       cached: sh.filter(x => x.badge === 1).length, orphans: v && v.stats && v.stats.orphans}); });
   // later: copy(JSON.stringify(rs132)) and save it as evidence; transitions only:
   // rs132.filter((e, i, a) => !i || ['epoch','resync','dig','mode'].some(k => e[k] !== a[i-1][k]))
   ```

## Step 5 — replay the four captures

Run the tool inside the rebuilt image on the base, against the desk broker
(`--network host`: the override keeps mosquitto on the base's loopback):

```sh
R="docker run --rm --network host -v /tmp/rs13_2:/rs13_2 lifetrac-v25:latest python /rs13_2/vector_replay_publish.py"
$R /rs13_2/leg2c_r4_base.jsonl --log /rs13_2/out/2c_full.jsonl
```

Every per-frame line (and the `--log` JSONL) carries the UTC time, the
capture row, `seq`, K, codec and `published` / `DROPPED`, so a screenshot
time maps to a row. Between captures, `$C restart web_ui` for a clean store,
then reload the page and log in again (sessions do not survive the restart)
and re-paste the recorder. A restart republishes the retained pins on the
desk broker; Step 8 removes them.

| order | capture | content | rows, span | watch for (rows) |
|---|---|---|---|---|
| 1 | `leg2c_r4_base.jsonl` | DTS, `mono_g4` control (591 × codec 1) | 591, 305 s | the photo path; the vector overlay stays clear, no banner, `safety_detector` = `pixels` |
| 2 | `leg2a_r4_base.jsonl` | DTS bw500 VECTOR, saturated (236–243 B) | 596, 305 s | on-air losses at rows 1, 14, 29, 42, 69, 149, 181, 194, 261, 486, 499, 512, 525, 538; 12 resync episodes (11 ended by DIGEST in 1.5–10 s); epoch starts at 116, 233, …; **A16 #527–#589** (31.5 s), ended by the 60 s safety refresh at 589. NO_HORIZON epochs throughout (F7) |
| 3 | `leg2b_r4_base.jsonl` | FHSS bw250 VECTOR | 558, 280 s | joins mid-epoch: resync #0–#34 (17 s, the join), first epoch start in the capture at row 44; lost frame at 194 (no resync); resync #393–#401 (4 s) |
| 4 | `leg2d_r4_base.jsonl` | DTS switch: 124 × codec 1, 235 × codec 6 (rows 124–358), 241 × codec 1 | 600, 307 s | **switch in at row 124** (K=1, plus its copy at 125); resync #139–#158 (9.5 s); **A16 #176–#231** (28.3 s, ended by the epoch start at 231); **switch out at row 359** |

Then the targeted windows. Replayed from the epoch start before it, the 2a
window reproduces the full capture's episodes exactly (checked with the
store off line; the same holds for 2d from row 124):

```sh
$R /rs13_2/leg2a_r4_base.jsonl --start-row 520 --start-at-key --end-row 595 --log /rs13_2/out/2a_a16.jsonl   # starts at 473, 63.5 s
$R /rs13_2/leg2d_r4_base.jsonl --start-row 104 --end-row 140 --log /rs13_2/out/2d_in.jsonl                 # switch in at 124
$R /rs13_2/leg2d_r4_base.jsonl --start-row 348 --end-row 400 --log /rs13_2/out/2d_out.jsonl                # switch out at 359
```

The 2a window starts with a 12.5 s join resync (#473–#497) because the store
starts mid-history; the episodes of interest follow unchanged.

**Injected drops.** 96 % of the VECTOR frames in these captures carry a DEL
(2a 571/596, 2b 529/558), so any dropped frame is a lost DEL, the A16
trigger. The receiver sees a `seq` gap exactly as after an air loss. Expected
results, from the same store replayed off line with the drop:

| command (add `--log`) | expected in the browser |
|---|---|
| `$R …/leg2a_r4_base.jsonl --drop-rows 450` | resync #453–#473 (10 s), ended by the epoch start at 473; the base TTL-drops 2 shapes the encoder still counts |
| `$R …/leg2b_r4_base.jsonl --drop-rows 348` | resync #351–#388 (18.5 s), ended by 3 matching DIGESTs; 6 shapes TTL-dropped on the way |
| `$R …/leg2b_r4_base.jsonl --drop-rows 84` | resync #87–#116 (14.4 s), ended by the epoch start at 116 |
| `$R …/leg2a_r4_base.jsonl --drop-rows 360` | resync #363–#383 (10 s), ended by DIGEST: a repaired loss, for contrast |
| `$R …/leg2b_r4_base.jsonl --drop-every 50` | 11 drops (#49, #99, …, #549): after the join, 9 resyncs of 1.4–12 s (3 ended by an epoch start, 6 by DIGEST); the drops at #299 and #549 open none |

`--loop` repeats a window until Ctrl-C (4 s between passes, longer than the
store's 3 s outage rule, so each pass is accepted as a fresh epoch) for a
reviewer at the screen; `--speed 2` halves the wait but distorts every age.

If the deployed SHA changed the store, the browser's episodes will differ
from the tables above; record both.

## Step 6 — the checklist

Record each row PASS / FINDING / N/A with the capture, row and screenshot.
"Rehearsal" is the PC result of 2026-10-10 (Appendix A); confirm it on the
base, do not copy it.

| # | check | how | expected (spec) | rehearsal |
|---|---|---|---|---|
| C1 | banner "VECTOR — NOT CAMERA PIXELS" | any VECTOR frame; also in raw mode | always shown while VECTOR, inside `#image-vector` so raw mode cannot hide it; text "· age · loss" (§0.2, §7.5) | shown, survives raw mode; **covered by the CURL/DUMP/REQ CTRL buttons at 1280×720** (F1); no age/loss (F5) |
| C2 | RESYNC chip | 2a #527–#589; 2b #0–#34; injected drops | shown for every resync episode, cleared when it ends (§4.3) | the browser received `resync` for exactly those rows, but **the chip is drawn past the canvas edge and clipped** (F2) |
| C3 | DIGEST MISMATCH chip | 2a rows 525–526 (mismatch before resync) | shown when `digest_ok` is false outside a resync | same clipping as C2 (F2) |
| C4 | epoch / anchor chip | epoch starts (2a 116, 233, 589) | epoch number steps; anchor age resets at an absolute L0 record | steps correctly; **anchor age climbs to ~52 s on 2a** (NO_HORIZON epochs, F7) |
| C5 | age styling | A16 window (2a 533–543), and after any replay ends | > 1.5 s tint, > 5 s desaturate + age label, > 10 s outline only (§4.3 table) | confirmed: outline + labels once the stream stops; ageing shapes in the A16 run |
| C6 | epoch hand-over | 2a rows 116, 589; 2d 231 | old epoch shown CACHED (desaturated, outline-weighted) until the new anchor and ≥ 50 % of shapes; no black flash (§3.5) | 8 CACHED shapes in the snapshot at 589; look at it on screen |
| C7 | rule 1: scene fault → black "VECTOR?" + `POST /api/health/refusal` | console injection (below) | `v` ≠ 1, scene badge ≠ 7 or no `anchor_age_ms` is refused | **not implemented**: the scene is drawn, chip reads "anchor ?", no POST (F3) |
| C8 | rule 2: shape fault → bounding box black, "BADGE?" | console injection | shape badge ∉ {1, 4, 7} or unknown `k` not painted | works |
| C9 | rule 3: self-model faults | — | — | N/A (Phase 3 not built) |
| C10 | rule 4: "NO VECTOR DATA" | console injection | `encode_mode` vector, `vector_scene` null | works |
| C11 | rule 5: old clients get black tiles (`/ws/state?schema=2`) | code reading | §7.5 rule 5 | **not implemented** in web_ui (F4) |
| C12 | never the old photo during VECTOR | 2d rows 124–358 | the panel is black + scene; no photo tile shows through | confirmed |
| C13 | back to tiles | 2d row 359 on | overlay clears at the first codec-1 frame; tiles older than the VECTOR window show their true age (staleness overlay) | overlay clears, photo paints; age of pre-VECTOR tiles not judged |
| C14 | pixel overlays stay under the scene | all VECTOR rows | staleness ages, RAW badges and detections never paint over `#image-vector` (z-index 5) | none seen |
| C15 | "SAFETY DETECTOR: NO PIXELS" chip | any VECTOR frame | shown whenever the base detector has no pixels < 3 s old (§7.5) | **not drawn** (F6); the snapshot does carry `safety_detector: "no_pixels"` |
| C16 | raw mode | toggle RAW MODE during 2a | vector overlay stays; shapes as received, true ages, no blur | overlay stays (blur is not implemented, so nothing to switch off) |
| C17 | uplink quiet in VECTOR | tap below, during a VECTOR replay that follows a codec-1 one | no `cmd/tile_stale`, no `cmd/req_keyframe` while VECTOR is active (§6, §7.3) | confirmed: 0 of either (and no `status/tile_age`) over a 125 s 2a replay, after 14 `cmd/tile_stale` in the 40 s of 2c before it |
| C18 | the encode-mode selector | settings → "Vector scene" once, then back | slider becomes "detail" 60–100, default 80 (§6); the console's "enc:" pill reads the stored override, not the received codec (F8) | options present |
| C19 | snapshot `encode_mode` | recorder | "vector" on codec 6, the tile codec's name after a tile frame; `safety_detector` flips with it (TODO: "populating `encode_mode` not verified") | verified both ways |
| C20 | browser vs record | recorder transitions vs the tables of Step 5 | the RESYNC rows the browser saw equal the dry-run episodes | 2a window: resync from 29.0 s to 60.5 s after the window's first frame = #527 → #589, 146 orphans; identical to the off-line store |

**Console injections (C7, C8, C10).** In the devtools console, best between
replays. web_ui sends a snapshot every 0.5 s and the renderer repaints from
each, so the snippet holds web_ui's own snapshots back until you release it:

```js
const od = dispatchEvent.bind(window); let hold = true;
window.dispatchEvent = (e) => (hold && e.type === 'lifetrac-state' && !e.fake) ? true : od(e);
const fake = (scene) => { const e = new CustomEvent('lifetrac-state', {detail: {encode_mode: 'vector',
  vector_scene: scene, grid: {w: 12, h: 8, tile_px: 32}, tiles: []}}); e.fake = true; od(e); };
const hz = {mode: 'none', pts: [], sky: ['#8fb3d9', '#cfe0ef'], ground: ['#6b8f3a', '#4f6b2b'], skyline: [], badge: 7, age_ms: 500};
// C7: v=2, scene badge 9, no anchor_age_ms -> expect black "VECTOR?" and a POST to /api/health/refusal
fake({v: 2, epoch: 3, badge: 9, digest_ok: true, resync: false, horizon: hz, layers: []});
// C8: one shape with badge 9, one unknown kind -> expect two black "BADGE?" boxes
fake({v: 1, epoch: 3, badge: 7, anchor_age_ms: 500, digest_ok: true, resync: false, horizon: hz, layers: [{id: 'L1', badge: 7,
  shapes: [{id: 4, k: 'poly', badge: 9, age_ms: 300, pts: [250, 60, 340, 60, 340, 140, 250, 140], fill: {c0: '#30a050'}},
           {id: 5, k: 'spline', badge: 7, age_ms: 300, pts: [260, 170, 360, 170, 360, 230, 260, 230]}]}]});
// C10: vector mode, no scene -> expect "NO VECTOR DATA"
fake(null);
// done: hold = false; window.dispatchEvent = od;
```

Screenshot each, and check the devtools network tab for the refusal POST.

**Uplink tap (C17).** In a second ssh session, during "2c (first ~40 s), then
2a" (`$R …/leg2c_r4_base.jsonl --end-row 79`, then `$R …/leg2a_r4_base.jsonl
--end-row 239`):

```sh
docker run --rm --network host lifetrac-v25:latest \
  mosquitto_sub -h 127.0.0.1 -v -t lifetrac/v25/cmd/tile_stale -t lifetrac/v25/cmd/req_keyframe -t lifetrac/v25/status/tile_age \
  | while read -r line; do echo "$(date -u +%T.%N | cut -c1-12) $line"; done | tee /tmp/rs13_2/out/uplink_tap.txt
```

Expected: nothing on `cmd/tile_stale` or `cmd/req_keyframe` from the first
VECTOR frame on, although the 2c tiles pass the 20 s stale horizon during
the VECTOR run. During the codec-1 replay before it, `cmd/tile_stale` and
`status/tile_age` every 3 s are normal: a replay that starts mid-stream
leaves most tiles never received. (`cmd/control` heartbeats at 20 Hz from the
console's control socket are normal too and are not subscribed here; no
bridge runs, so none of this goes anywhere.)

## What A16 looks like on screen

From the rehearsal and the store replay of 2a #520–#595 (the browser saw the
same rows):

1. Row 525 arrives after a lost frame (`seq` 26 missing). DIGEST mismatches at
   525–526 (`DIGEST MISMATCH`), and the store enters resync at 527 (`RESYNC`).
2. In resync the store resets no ages from CONFIRMs, so the shapes the
   tractor only CONFIRMs start to age, and so does the ghost the lost DEL
   left behind: from row ~533 four or five shapes are desaturated with an age
   label, at 542–543 three or four are outline-only, and at 544 they vanish
   (TTL, `ttl_dropped` 1 → 4).
3. From then on the base is **one shape short**: a region the tractor still
   describes is simply absent, with no hatch and no mark of its own. Orphan
   records arrive every frame (+1–2), which the screen does not show.
4. This lasts until row 589 (31.5 s after 527): the 60 s safety refresh
   arrives, the old epoch shows briefly as CACHED (8 shapes), the new epoch
   fills in, and the RESYNC chip clears.

So the operator's cues are the RESYNC chip and the ageing, then vanishing,
shapes. With F2 the chip is not visible, leaving only the ageing. That is
the input the A16 decision needs from this desk check: whether the encoder-only
repair is enough, or the base must keep CONFIRM-only shapes alive through a
ghost resync.

## Step 7 — recording findings

Evidence directory `bench-evidence/RS_13_2_desk_check_<YYYY-MM-DD>/`:

- `RESULTS.md` (skeleton below);
- `screenshots/<capture>_r<row>_<what>.png` (Win+Shift+S or the devtools
  "Capture screenshot"), the row taken from the replay log by its UTC time;
  one set per window size;
- `logs/`: the replay `--log` files from `/tmp/rs13_2/out`, the uplink tap,
  `$C logs web_ui` and `$C logs mosquitto`, the recorder JSON (`copy(...)` in
  the console) per capture;
- `versions.txt`: deployed SHA (`DEPLOYED_FROM.txt`), image id, tool SHA,
  browser and window size.

```markdown
# RS-13.2 — VECTOR desk check (<YYYY-MM-DD>)

**Status / verdict:** <one sentence: which checks pass, which findings block what>.

## Setup
| item | value |
|---|---|
| deployed tree / image | `<sha>` (`DEPLOYED_FROM.txt`), `lifetrac-v25:latest` `<id>` |
| stack | `docker compose -p lifetrac-desk -f docker-compose.yml -f rs13_2_desk.compose.yml`: mosquitto + web_ui only; `fuser /dev/ttymxc3` empty before and after |
| tool | `tools/vector_replay_publish.py` @ `<sha>`, speed 1 |
| browser | <browser, version>, window <w×h> (and <w×h>) |

## Replays
| capture / window | drops | resync episodes seen (rows, s, ended by) | vs RS-13.1 record | screenshots |
|---|---|---|---|---|

## Checklist
| # | result | evidence | note |
|---|---|---|---|
| C1 … C20 | | | |

## A16 on screen
<what the operator saw in 2a #527–#589 and 2d #176–#231; the operator's view on the A16 decision>

## Findings
| id | finding | severity (blocks VECTOR in the field? / honesty cue / cosmetic) | owner |
|---|---|---|---|

## Teardown
<`down -v` output, `ss -ltn` and `fuser` after, retained topics gone>
```

## Step 8 — tear down

```sh
cd /var/rootdirs/opt/lifetrac/DESIGN-CONTROLLER
C="docker compose -p lifetrac-desk -f docker-compose.yml -f /tmp/rs13_2/rs13_2_desk.compose.yml"
$C down -v                       # containers and the desk broker's volumes, with its retained topics
docker ps -a --filter name=lifetrac-desk --format '{{.Names}}'     # expect nothing
ss -ltn | grep -E ':(1883|8080) ' || echo "1883 and 8080 free"
sudo fuser /dev/ttymxc3; echo "uart holders: (expect none above)"
```

Copy `/tmp/rs13_2/out` to the PC, then `rm -rf /tmp/rs13_2` (it holds the
desk-check PIN). The image stays built, which is what the next radio session
needs (TODO RS-13 "Images").

## Appendix A — PC rehearsal (no board), 2026-10-10

The same check runs entirely on the PC with the working tree's web_ui: useful
to learn the screens, and it is how F1–F10 were found. It is not the desk
check of record, which is the rebuilt image on the base.

Broker on the PC's loopback only, so no LAN client (a bench board) can reach
it: Eclipse Mosquitto 2.x for Windows started with no configuration file (it
then listens on localhost:1883 only), or the pure-Python `amqtt` (what the
rehearsal used; `py -3 -m pip install --user amqtt`) with this config file:

```yaml
listeners:
  default: {type: tcp, bind: 127.0.0.1:1883}
plugins:
  amqtt.plugins.authentication.AnonymousAuthPlugin: {allow_anonymous: true}
```

```powershell
py -3 -m amqtt.scripts.broker_script -c <that file>          # window 1
# window 2: web_ui from the working tree; its stores go to a scratch directory, not the tree
cd LifeTrac-v25/DESIGN-CONTROLLER/base_station
$S = "$env:TEMP/rs13_2_webui"; mkdir $S -Force | Out-Null
$env:LIFETRAC_MQTT_HOST = "127.0.0.1"; $env:LIFETRAC_PIN = "<test PIN>"; $env:LIFETRAC_ENABLE_BENCH_RADIO = "0"
$env:LIFETRAC_PIN_STORE = "$S/.operator_pin"; $env:LIFETRAC_ENCODE_MODE_STORE = "$S/.encode_mode_override"
$env:LIFETRAC_RADIO_PROFILE_STORE = "$S/.radio_profile_override"; $env:LIFETRAC_AUDIT_PATH = "$S/audit.jsonl"
py -3 -m uvicorn web_ui:app --host 127.0.0.1 --port 8091
# window 3: the replays (web_ui connects to port 1883; it has no port setting)
cd LifeTrac-v25/DESIGN-CONTROLLER
py -3 tools/vector_replay_publish.py bench-evidence/RS_13_vector_scene_2026-09-26/legs/leg2a_r4_base.jsonl --start-row 520 --start-at-key --end-row 595
```

Browser at `http://127.0.0.1:8091/login`. Working tree `origin/main`
`bb4a2071` (the RS-13.1 code), browser inner size 1280×720.

**Confirmed:** codec-6 frames reach the store and the renderer; the 2a window
reproduced the dry run's episodes in the browser (C20); `encode_mode` and
`safety_detector` flip at the 2d switch rows both ways (C19); the panel is
fully covered during VECTOR (C12); the overlay clears on the first codec-1
frame (C13); age styling reaches outline + age labels once frames stop (C5);
`BADGE?` and `NO VECTOR DATA` work (C8, C10); raw mode keeps `#image-vector`
visible (C16); no pixel overlay paints over the scene (C14); "Vector scene" is
in the encode-mode selector (C18); the stale-tile worker and the keyframe
request path stay silent through 125 s of VECTOR after 40 s of codec 1 (C17).
A fresh subscriber to the rehearsal broker received the retained
`control/encode_mode_override` and `control/radio_profile` that web_ui had
published at startup, which is why Rule 3 and Step 8 exist.

**Findings to confirm on the base** (measured in the browser unless marked
"code"):

| id | finding | where |
|---|---|---|
| F1 | The banner is drawn in the overlay's own 384×256 pixels at (6, 6) and scaled with the panel; at 1280×720 the CURL / DUMP / REQ CTRL buttons sit on its first ~80 canvas px, so the operator reads "…OT CAMERA PIXELS". §7.5 guards the banner against raw mode, not against the console's own chrome. | `vector_renderer.js` `render()` chips; `index.html` button bar |
| F2 | The chips are laid out left to right in canvas px: banner 6–188, "epoch NN · anchor NN.Ns" 192–353, so **RESYNC starts at x ≈ 357 and ends at ≈ 407, past the 384 px edge**: clipped, and the rest sits under the top-right icons. DIGEST MISMATCH and the anomaly chip go the same way. During the 31.5 s A16 run the store was in resync and the operator could not see it. | `vector_renderer.js` lines with `chip(...)` |
| F3 | Rule 1 is not implemented: a scene with `v` = 2, scene badge 9 and no `anchor_age_ms` is drawn normally ("anchor ?"); no "VECTOR?" panel and no refusal POST. | `vector_renderer.js` `render()` |
| F4 | Rule 5 is not implemented (code): `/ws/state` has no `schema=2` handling, so a browser that loaded before the deploy keeps its old scripts and gets no black-tile set during VECTOR. | `web_ui.py` `ws_state` / `_admit_ws` |
| F5 | The banner is the fixed text "VECTOR — NOT CAMERA PIXELS", without "· age · loss" (code). | `vector_renderer.js` |
| F6 | No "SAFETY DETECTOR: NO PIXELS" chip anywhere, although the snapshot carries `safety_detector: "no_pixels"` (code + browser). | `vector_renderer.js`, `index.html` |
| F7 | The anchor chip shows the age of the last absolute L0 record. 2a is a NO_HORIZON stream (58 `HznNoHorizon`, 0 `HznResid` in 596 frames; the encoder sends no L0 record while no horizon is found), so the chip climbs to ~52 s on a live 2 fps stream and reads like a stale picture. True for L0; decide what the operator should read there. | store `_anchor_clk`, encoder L0 slot |
| F8 | The console's top line reads "N Hz · tile stream" during VECTOR (it counts `/ws/state` snapshots), and the "enc:" pill shows the stored operator override ("Full color" in a replay), not the received codec. | `app.js` meta line; encode pill |
| F9 | When frames stop, every shape ends outline-only with a growing age label and the scene stays: TTL counts applied frames, so nothing expires without frames. Honest, but there is no explicit link-lost cue in the vector panel. | §4.3 TTL |
| F10 | After the switch back to tiles the snapshot still carries the last `vector_scene` (the renderer ignores it because `encode_mode` is no longer vector). Harmless today; note for the old-client path. | `state_publisher.py` |

F1, F2 and F6 bear on §7.5's honesty cues directly and look like the first
renderer fixes after this desk check; F3 and F4 are fail-closed rules the
design lists and the code does not have yet.
