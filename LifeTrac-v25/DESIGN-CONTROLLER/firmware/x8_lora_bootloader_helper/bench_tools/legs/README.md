# Bench leg scripts

These scripts run the RS-13 bench procedure
([RS13_VECTOR_LEG.md](../RS13_VECTOR_LEG.md)) from any checkout. They are
parameterised copies of the RS-13.1 session scripts.

The originals stay unchanged in
[`bench-evidence/RS_13_vector_scene_2026-09-26/scripts/`](../../../../bench-evidence/RS_13_vector_scene_2026-09-26/scripts/)
as the historical record. They hardcode one PC's paths, a session scratchpad,
the serials and the password. Never edit them. Change these copies instead.

Before you start, read:
- [BENCH_QUICKSTART.md](../BENCH_QUICKSTART.md), the entry point;
- [PC_SETUP.md](../PC_SETUP.md) for the PC tools;
- [BENCH_SETUP.md](../BENCH_SETUP.md) to provision the boards;
- [DEPLOY.md](../DEPLOY.md) for the images and the base deploy;
- [BENCH_RUNBOOK.md](../BENCH_RUNBOOK.md) for the campaign rules.

## Rules that apply to every script

- **Radios only on an explicit GO from the operator, per round.**
  - `leg_prep.sh` wakes the receivers.
  - The harness transmits.
  - `leg2d_switch.sh` makes the base transmit commands.
  - Run them only inside a GO'd leg.
  - The other scripts never transmit.
- **Park last.** `leg_post.sh` parks both radios after the reports. Before you
  leave, run `power_up_guard.sh --check-only`.
- **Screen.** Never use `firefox --kiosk` or any fullscreen the operator cannot
  leave with Esc. The bench video runs in a normal window, and only the YouTube
  player is fullscreen (`youtube_window.ps1`).
- **DTS carrier.**
  - A profile-2 (DTS) leg flies only on a carrier from a receive-only
    spot-check made that same day with the tractor parked.
  - Without `DTS_CARRIER_HZ`, `leg_prep.sh` refuses the leg.
  - It also refuses 915.000 MHz, the channel of the RS-11.6 external emitter
    (RS-13.1 A19).
- **Shells.** Run the `.sh` scripts in Git Bash. Run the harness only in
  PowerShell, with an execution policy that lets local scripts run
  ([PC_SETUP.md](../PC_SETUP.md#powershell-execution-policy)).

## Setup (once per PC)

```bash
cd LifeTrac-v25/DESIGN-CONTROLLER/firmware/x8_lora_bootloader_helper/bench_tools
cp bench.env.example bench.env        # gitignored; edit serials, BASE_TRANSPORT, PC_HOST ...
bash -c '. lib/bench_env.sh && bench_show_env'   # what the scripts will use
```

`PC_HOST` is **required** for a leg: this PC's LAN IPv4 address as the
boards see it. The harness gets it as `-HostIp`, and the base rx daemon uses
it as its control broker. If it is empty, `leg_prep.sh` auto-detects this
PC's LAN IPv4 and prints it, or refuses (exit 6) when that is ambiguous;
it never lets the harness fall back to the original bench PC's address.

Other refusals: leg, scene and suffix tags must match `^[A-Za-z0-9_.-]+$`;
`leg_post.sh` exits 3 if a board is not radio-quiet when it finishes (park
with `power_up_guard.sh --stop-leg-daemons`); `leg2d_switch.sh` clears its
retained `encode_mode_override` when it exits.

Every script sources [`../lib/bench_env.sh`](../lib/bench_env.sh). It works
out the repo root from its own location, so you can call the scripts from any
directory. It loads `bench.env`, or `bench.env.example` when there is none;
the example holds the original bench's serials and addresses, so create
`bench.env` before the first run. A non-empty variable in your environment
overrides the file for one run of these `legs/` scripts, for example
`TRACTOR_SERIAL=... bash leg_prep.sh ...`. Each script prints its usage with
`-h`.

## The scripts

| script | what it does | boards | radio |
|---|---|---|---|
| `power_up_guard.sh` | Waits for the boards. Stops the tractor camera unit. Stages the probes. Reads the L072 and parks it if it is listening. `--check-only` is the end-of-session check. `--stop-leg-daemons` clears an interrupted harness. Works over adb, or over ssh for the base. | both | reads the radio and parks it; never transmits |
| `stage_boards.sh` | (Re)stages `/tmp/lifetrac_strict` (probes, injectors, `channel_survey_sniff.py`) and the harness's `/tmp/lifetrac_p0c/08_boot_user_app.cfg` (LF-clean), then the code tree. md5-verifies every file. `--probes-only` stages the probes and paho only (plus the tractor's `08_boot_user_app.cfg`). | both | file copies only |
| `push_fix_to_board.sh` | Pushes this checkout's `camera_service.py`, encoder, store and `vector_dry_run.py`, md5-checked, then runs an import smoke without any device. | one | none |
| `youtube_window.ps1` | `-Open` / `-Front` / `-Close` the bench video window (its own Firefox profile, muted). `-Url`, `-ProfileDir`, `-FirefoxExe`. | none | none |
| `scene_check.sh` | Before every camera run: window to the front, then frames from `/dev/video1` until 10.5 s after the camera's sixth frame (at most 20 s after its first). Needs ≥ 8 % of pixels changed over 10 s (0 = record only, for a still). No frame within 20 s, or fewer than 10 frames, fails with a `NO-FRAMES` reason (exit 4). | tractor camera | none |
| `step1_pass.sh` | One camera-only step-1 pass: fresh `bench_mqtt`, capture, then `camera_svc` with the profile's `LIFETRAC_REG_PROFILE`. `--from-work` runs the pushed tree. Refuses to run while a leg daemon is up. Exit 6 = camera_service's VECTOR cold-start clamp was seen (boot-log check or the pulled camera log), or no camera log from this pass with its `byte_budget` line came back: the pass is not valid at the profile's budget. | tractor | none (camera only) |
| `leg_prep.sh` | Scene check, UART-holder check, health probes, `clear_retained`, pre-brackets, detached base capture. Writes the harness command. Knows the RS-13.1 tags and the range-edge leg names. | both | wakes the receivers (RXCONT); does not transmit |
| `link_sample.sh` | During a leg: the first `link_stats` message that has frames (P7). | base broker | none |
| `leg2d_switch.sh` | Leg 2d only: the override publishes at T+60 s and T+180 s, plus the acks. | base broker | **the base transmits** (0x63 commands) |
| `leg_post.sh` | Collects the capture, post-brackets, `rs12_leg_report --capture`, `frag_gap_report`, P2/P4 and the tractor-log, then **parks** (with retries). | both | parks; never transmits |
| `leg_replay.py` | PC only: replays a base capture (seq gaps per codec run, resync episodes, last DIGEST). `py -3 leg_replay.py <leg> <image_bw500\|image_bw250> [--evidence-dir DIR \| --capture FILE]`; it reads `<DIR>/leg<leg>_base.jsonl` (default DIR: `$EVIDENCE_DIR`, which only the scripts export, so pass `--evidence-dir` from a plain shell). `leg_post.sh` runs it for you. | none | none |

## A session

1. **Power-up.** Arm the guard *before* switching the boards on:

   ```bash
   bash power_up_guard.sh --wait-power-cycle
   ```

   When the boards are already up, run it without the flag. If the base does
   not enumerate on USB, set `BASE_TRANSPORT=ssh` for this step.
2. **Stage both boards:** `bash stage_boards.sh`.
   - Re-run it after any reboot or flash, and whenever the last push is 5 days
     old: `/tmp` is tmpfs and ages at 5 days.
   - Each board should hold ≥ 19 entries (a fresh full staging holds 23,
     `channel_survey_sniff.py` included).
3. **Moving content.** In PowerShell:

   ```powershell
   .\youtube_window.ps1 -Open
   ```

   Start it at least a minute before the first check, because two pre-roll
   ads play first.
4. **DTS carrier.** This step is needed before the first profile-2 leg of the
   day.
   - Run a receive-only spot-check with the tractor parked:
     [`../../channel_survey_sniff.py`](../../channel_survey_sniff.py)
     (`stage_boards.sh` stages it; run it on the base) or
     [`../../hunt_sniff.ps1`](../../hunt_sniff.ps1), then
     `tools/survey_compare.py`.
   - Put the pick in `bench.env` as `DTS_CARRIER_HZ=<Hz>` with
     `DTS_CARRIER_DATE=<today, YYYY-MM-DD>`. `leg_prep.sh` refuses a DTS leg
     without it or on another day, and writes it into the harness command as
     `-ForceFrfHz`.
   - Record the pick in RESULTS.

## One leg: prep, harness, link sample, post, park

Example leg 2a (DTS, VECTOR at boot). Tags may carry a suffix (`2a_r5`); the
tag names every file.

```bash
# 1. prep (Git Bash). Ends by printing the harness command and writing it to
#    $BENCH_SCRATCH/leg2a_harness.ps1. Preview without touching a board:
#    add --print-harness.
bash leg_prep.sh 2a image_bw500
```

```powershell
# 2. harness (PowerShell, right away: the base capture is already listening).
#    Easiest: run the wrapper leg_prep printed:  & "<BENCH_SCRATCH>/leg2a_harness.ps1"
#    It tees the transcript and records the archive folder for leg_post.
#    By hand it is this, from firmware/x8_lora_bootloader_helper/,
#    with the values of your bench.env:
$DTS_CARRIER_HZ = <your same-day pick, in Hz>
.\run_live_radio_monitor.ps1 -TxAdbSerial <TRACTOR_SERIAL> -RxAdbSerial <BASE_SERIAL> -HostIp <PC_HOST> `
   -TxFeed camera -RegProfile 2 -ForceFrfHz $DTS_CARRIER_HZ -DurationS 300 -SynthFps 2 `
   -KfRequestDisable 1 -ReactiveFire 0 -ProbeEcho 0 -LogFragArrivals 1 -TxBatch 0 `
   -CamExtraEnv "-e LIFETRAC_ENCODE_MODE=9 -e LIFETRAC_VECTOR_DETAIL=80" -Archive
```

```bash
# 3. while frames flow (Git Bash):
bash link_sample.sh 2a
# 4. after the harness prints "[EVIDENCE] archived to ...": collect, report, park
bash leg_post.sh 2a          # or: bash leg_post.sh 2a <archive dir>
```

Before step 4, read the `=== LEG CHECKS ===` block just above the
`[EVIDENCE]` line. `TRACTOR_L072_RESET landed` is expected. `failed` or
`unknown` means the tractor's counters may carry over from before the leg;
`deltas.py` handles that (BENCH_RUNBOOK, "A leg"). `failed (... not this leg's ...)` means the
leg-start reset was never spawned (the spawn's output is quoted).
`failed (... killed by SIGHUP ... openocd never started)` points at the
hangup race (candidate (c) in BENCH_RUNBOOK, "L072 resets at leg start").
Keep that leg's archived `tractor_ocd_reset.log`. `BASE_L072_RESET unverified`
prints on every leg: score the base brackets post − pre.

**Step 5 is the park.**
- `leg_post.sh` parks both radios last. It retries a `PARK_TRANSIENT` after
  60 s, up to 3 attempts, and logs every attempt in `leg2a_park.txt`.
- If it ends with `PARK NOT CONFIRMED`, run `bash power_up_guard.sh`. It parks
  any board that is not in `0x80`.
- At the end of the session, run `bash power_up_guard.sh --check-only`.

**Not flying after the prep after all** (no GO, the harness refused, the
round was aborted)? `leg_prep.sh` has woken both receivers (RXCONT) and left
the detached base capture `rs13_cap_base` running, and nothing else will park
them. Run `bash power_up_guard.sh --stop-leg-daemons`: it stops the leftover
leg containers and parks both radios.

A few cautions about the harness command:
- Do not drop `-ForceFrfHz` on a profile-2 leg. The harness refuses such a
  leg (its default carrier is 915.000 MHz); `-AllowDefaultCarrier` is only for
  a deliberate 915 MHz control.
- Always pass `-HostIp <PC_HOST>`; `PC_HOST` is required (Setup, above).
- The harness refuses unknown parameter names at launch (it used to ignore a
  misspelling such as `-DurationSeconds`). Still copy the generated command
  rather than typing it.
- For a command-plane leg add `-InjectCmd "<injector> <args>"` (for example
  `-InjectCmd "dual_inject.py 250"`), plus `-InjectOn fault:0x0D:final` for a
  scan-FAILED leg. The harness starts the injector on the base at the trigger
  and archives `inject.log` and `inject_trigger.txt`. Do not start injectors
  by hand from the console tail. `leg_prep.sh` does not write `-InjectCmd`;
  add it to the line yourself.

### The four RS-13 legs

| leg | prep | harness, beyond the 2a line | base capture |
|---|---|---|---|
| 2a: DTS, VECTOR at boot | `leg_prep.sh 2a image_bw500` | (the line above) | `--strict`, min 500 frames |
| 2b: FHSS, VECTOR at boot | `leg_prep.sh 2b image_bw250` | `-RegProfile 1`, **no** `-ForceFrfHz` | `--strict`; at `--fps 1`, min 200 (A7) |
| 2c: DTS control, `mono_g4` at boot | `leg_prep.sh 2c image_bw500` | `-CamExtraEnv "-e LIFETRAC_ENCODE_MODE=6"` | no `--strict` |
| 2d: DTS switch leg, boots `mono_g4` | `leg_prep.sh 2d image_bw500`, then `leg2d_switch.sh 2d` right after the harness launch | same as 2c | no `--strict` |

`leg_prep.sh` picks the boot mode and the strictness from the tag (`2c*` and
`2d*` boot `mono_g4` and run without `--strict`). Override them with
`--boot-mode`, `--strict` / `--no-strict`, `--fps` and `--min-frames`.
`--duration S` sets the leg length (default 300 s; the base capture runs
S + 180 s). `--reactive-fire 1` makes the base fire a no-op PROBE per
received frame; the default 0 is the harness default, and the line always
states it.

How to fly and read them:
- For P3, interleave the control with the VECTOR legs (2c, 2a, 2c′, 2a′).
- At 1 fps, read 2b's P2 and P3 from lock.
- The pass criteria are in RS13_VECTOR_LEG.md, step 2.

### Range-edge legs (RS13_RANGE_EDGE_LEG.md)

`leg_prep.sh` knows the leg names of
[RS13_RANGE_EDGE_LEG.md](../RS13_RANGE_EDGE_LEG.md), each optionally
followed by a `_suffix` (`A2a_r2`). Since 2026-10-19 only the `A2*` tags are
planned: they are to fly on the antennas (the §8.6 command-delivery row and
the A16 confirmation), on the operator's explicit GO for the round (RS13_RANGE_EDGE_LEG.md, "Safety and GO rules"). The `leak_*`, `B*`, `C*` and
`*close` tags need the conducted attenuator path, which the bench does not
have, so they are not planned. `leg_prep.sh` still accepts them.

| tag | prep | harness | base capture |
|---|---|---|---|
| `leak_dts`, `leak_fhss` | `leg_prep.sh leak_dts` | VECTOR, `-DurationS 120 -ReactiveFire 0` | 300 s, min 200, no `--strict`; must stay empty |
| `A2c`, `A2a`, `A2c2`, `A2a2` | `leg_prep.sh A2c` | DTS, `mono_g4` (c) / VECTOR (a), `-DurationS 300 -ReactiveFire 1` | 480 s, min 500, `--strict` on VECTOR only |
| `B<step>v`, `B<step>m` | `leg_prep.sh B12v --duration 120` (coarse) or `180` (fine) | DTS, VECTOR (v) / `mono_g4` (m), `-ReactiveFire 1` | leg + 180 s, min 200 / 300, `--strict` on VECTOR only |
| `C<step>v`, `C<step>m` | `leg_prep.sh C12m --duration 120` or `180`; `C0v` / `C0m` default to 180 s | FHSS, no `-ForceFrfHz`, `-ReactiveFire 0` | the same |
| `B0close`, `C0close` | `leg_prep.sh B0close` | VECTOR, 120 s; DTS with probes / FHSS without | 300 s, min 200, `--strict` |

- The tag sets the profile (it may be left off), the boot codec, the probes
  and the length. Every line carries `-TxBatch 0 -ProbeEcho 0`, and every
  DTS line `-ForceFrfHz $DTS_CARRIER_HZ` (the DTS carrier gate applies).
- `<step>` is the step chain in dB: 0 to 66 in 3 dB steps, no leading zero.
- `leg_prep.sh` refuses:
  - a walk leg without `--duration`;
  - a near-miss tag (`A2b`, `b12v`, `B09v`, `B10v`, `B0closed`, `leak_dt`):
    any tag that starts with `leak`, `A2`, or `B`/`C` and a digit, in any
    letter case, must be one of the names above;
  - a profile or `--boot-mode` that contradicts the tag.
- It keeps `--duration`, `--reactive-fire`, `--fps` and `--strict` /
  `--no-strict` overrides, and prints each as a NOTE.
- Set `EVIDENCE_DIR` to `bench-evidence/RS_13_range_edge_<date>/legs` in
  `bench.env` first; otherwise the prep prints a NOTE.
- File names carry the `leg` prefix (`legA2c_*`, `legleak_dts_*`).
- `--print-harness` shows the plan line (profile, codec, length,
  `-ReactiveFire`, capture) and any NOTE without touching a board.
- The scoring helpers (`../rs13_rf_extract.py`, `../rs13_probe_score.py`,
  `../rs13_digest_trace.py`) are PC tools; `leg_post.sh` does not run them
  yet. RS13_RANGE_EDGE_LEG.md says which to run per leg.

## Step 1: camera-only dry run (no radio)

Three passes. Each pass runs its own scene check and recreates `bench_mqtt`
(0 retained messages):

```bash
bash stage_boards.sh tractor                   # code tree + bench_mqtt.conf on the tractor
bash step1_pass.sh bw250 moving --from-work    # the railroad video (scene check >= 8 %)
bash step1_pass.sh bw250 landscape --from-work # a still (scene check records only)
bash step1_pass.sh bw500 landscape --from-work
```

`step1_pass.sh` gives camera_svc `LIFETRAC_REG_PROFILE` 1 (bw250) or 2
(bw500), as the harness does on those legs, so a bw500 pass encodes at
243 B instead of the 203 B cold-start clamp (the 2026-10-14 bw500 passes
ran without it). The boot-log check greps camera_service's
`VECTOR budget … until link_budget` line, and the full camera log pulled at
the end is checked for it too (a slow start can log it after the +15 s
sample). If it appears, the pass prints a WARNING, still tears down and
pulls its files, then ends `== pass <tag> INVALID: …` and exits 6: the pass
did not encode at the profile's one-fragment budget, so re-run it (with a
new `--suffix`) once the cause is fixed. A pass whose camera log does not
come back, or whose log lacks camera_service's `byte_budget=<203|243> B/frame`
line, also ends INVALID (exit 6): its budget is unverified.

- `--from-work` runs the pushed tree. The files get a `_work` suffix (RS-13.1
  used `_fix`).
- Without the flag, the pass runs the image's own `/app` copy.
- For the landscape still, show a picture in the same normal window:

  ```powershell
  .\youtube_window.ps1 -Open -Url file:///C:/path/to/picture.jpg -NoPlayerFullscreen
  ```

  Maximise it by hand and frame the camera on it.
- `step1_pass.sh` refuses to start while anything holds `/dev/ttymxc3` or a
  `tx_smoke` / `rx_smoke` / `synth_pub` container runs. A tx daemon would put
  the camera frames on the air.

## Where things go

- **Evidence:** `$EVIDENCE_DIR`.
  - The default is
    `bench-evidence/RS_13_vector_scene_<UTC date>/legs/`, laid out as in
    RS13_VECTOR_LEG.md, "Evidence layout".
  - Set it explicitly for a session that crosses midnight UTC.
  - `leg_prep.sh` and `leg_post.sh` save their own transcripts there
    (`leg<X>_prep.txt`, `leg<X>_post_transcript.txt`).
  - The staging md5s go to `staging_<role>.txt`.
- **Scratch:** `$BENCH_SCRATCH`, for the harness wrappers and transcripts and
  the LF-clean copies.
- **Harness archives:** `bench-evidence/radio_monitor_<stamp>_<sha>/`, where
  the harness puts them.
- **Board captures** (`power_up_guard.sh --capture`): `$ARCHIVE_DIR`, outside
  git.

## Changes from the RS-13.1 originals

- **Paths, serials, password, images.** All paths, serials, the password and
  the image names come from `bench.env`.
- **Merged step-1 script.** `step1_pass.sh` and `step1_pass_fix.sh` are now one
  script (`--from-work`). It always recreates `bench_mqtt` and saves the
  before/after retained listing to `step1_bench_mqtt_reset.txt`. It also runs
  the scene check and pulls the pass files into the evidence folder. Since
  2026-10-18 it passes `LIFETRAC_REG_PROFILE` and exits 6 when the VECTOR
  cold-start clamp shows in the boot log.
- **`stage_boards.sh`** also stages `/tmp/lifetrac_p0c/08_boot_user_app.cfg`
  LF-clean. The harness's openocd reset of the tractor uses it. The staging
  is md5-checked. It also stages `channel_survey_sniff.py` (not with
  `--probes-only`).
- **`leg_prep.sh`**:
  - enforces the DTS carrier;
  - aborts on a radio-UART holder;
  - picks the boot mode and strictness by tag;
  - knows the range-edge tags; takes `--duration` and `--reactive-fire`; the
    base capture runs the leg + 180 s;
  - writes the harness command.
- **`scene_check.sh`** runs its window until 10.5 s after the camera's sixth
  frame, which is the frame the 1.9 / 5 / 10 s pairs are measured from, and
  at most 20 s after the first frame. A slow camera start or frames that
  trickle in no longer leave the check without a 10 s figure (2026-10-15).
  It fails with a `NO-FRAMES` reason when no frame comes within 20 s of
  opening the camera or fewer than 10 frames come, and it never pulls an
  earlier run's frame (it removes the tractor's `/tmp/scene_check.jpg`
  first). The scene file gains the line `camera: first frame X s after
  opening /dev/video1; N frames, the sixth S s and the last Y s after the
  first`.
- **`leg_post.sh`**:
  - passes `--capture` to `rs12_leg_report.py` (A14);
  - finds the archive by itself;
  - flags a DTS leg flown on 915.000 MHz;
  - retries the park.
- **`leg_replay.py`** takes `--evidence-dir` or `--capture`. It runs from any
  directory. On the RS-13.1 `2a_r4` capture it reproduces the recorded
  `leg2a_r4_p2p4.txt` seq-gap line exactly.
- **`power_up_guard.sh`** replaces `guard_both.sh`, `power_up_guard.sh` and
  `tractor_guard_capture.sh`. It adds ssh for the base, `--check-only`, the
  park retry and a refusal to probe while a leg container or a UART holder is
  present.
- **`youtube_window.ps1`** sends "f" only to a YouTube window, so a still
  page is left alone.

## Known limits

- **The harness is adb-only.** `run_live_radio_monitor.ps1` drives both boards
  over adb. With `BASE_TRANSPORT=ssh` you can stage, guard and check, but you
  cannot fly a leg.
- **The harness pushes its own copy of the reset cfg.** At every launch it
  pushes `08_boot_user_app.cfg` from the working tree. The repo's
  `.gitattributes` keeps `*.cfg` LF, so this matters only on a checkout made
  before `.gitattributes` landed: there, with `core.autocrlf=true`, the copy
  has CRLF line endings and overwrites the LF copy `stage_boards.sh` staged.
- **`--capture` is adb-only.** `power_up_guard.sh --capture` calls
  `pull_board_state.sh`, which is adb-only; it reads `BASE_SERIAL` and
  `TRACTOR_SERIAL` from `bench.env`.
- **`leg_post.sh` does not copy the harness's new archive files** (`inject.log`,
  `inject_trigger.txt`, `tractor_ocd_reset.log`) into `legs/`, and does not
  run the RS-13 scoring helpers. Copy and run them by hand. Its console
  still says the base capture runs "480 s"; it waits on the container
  (`docker wait`), so a shorter or longer capture is handled.
