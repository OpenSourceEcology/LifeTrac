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
  PowerShell.

## Setup (once per PC)

```bash
cd LifeTrac-v25/DESIGN-CONTROLLER/firmware/x8_lora_bootloader_helper/bench_tools
cp bench.env.example bench.env        # gitignored; edit serials, BASE_TRANSPORT, PC_HOST ...
bash -c '. lib/bench_env.sh && bench_show_env'   # what the scripts will use
```

Every script sources [`../lib/bench_env.sh`](../lib/bench_env.sh). It works
out the repo root from its own location, so you can call the scripts from any
directory. It loads `bench.env`, or `bench.env.example` when there is none.
A non-empty variable in your environment overrides the file for one run, for
example `TRACTOR_SERIAL=... bash leg_prep.sh ...`. Each script prints its
usage with `-h`.

## The scripts

| script | what it does | boards | radio |
|---|---|---|---|
| `power_up_guard.sh` | Waits for the boards. Stops the tractor camera unit. Stages the probes. Reads the L072 and parks it if it is listening. `--check-only` is the end-of-session check. `--stop-leg-daemons` clears an interrupted harness. Works over adb, or over ssh for the base. | both | reads the radio and parks it; never transmits |
| `stage_boards.sh` | (Re)stages `/tmp/lifetrac_strict` and the harness's `/tmp/lifetrac_p0c/08_boot_user_app.cfg` (LF-clean), then the code tree. md5-verifies every file. | both | file copies only |
| `push_fix_to_board.sh` | Pushes this checkout's `camera_service.py`, encoder, store and `vector_dry_run.py`, md5-checked, then runs an import smoke without any device. | one | none |
| `youtube_window.ps1` | `-Open` / `-Front` / `-Close` the bench video window (its own Firefox profile, muted). `-Url`, `-ProfileDir`, `-FirefoxExe`. | none | none |
| `scene_check.sh` | Before every camera run: window to the front, ~10 s of frames from `/dev/video1`. Needs ≥ 8 % of pixels changed (0 = record only, for a still). | tractor camera | none |
| `step1_pass.sh` | One camera-only step-1 pass: fresh `bench_mqtt`, capture, then `camera_svc`. `--from-work` runs the pushed tree. Refuses to run while a leg daemon is up. | tractor | none (camera only) |
| `leg_prep.sh` | Scene check, UART-holder check, health probes, `clear_retained`, pre-brackets, detached base capture. Writes the harness command. | both | wakes the receivers (RXCONT); does not transmit |
| `link_sample.sh` | During a leg: the first `link_stats` message that has frames (P7). | base broker | none |
| `leg2d_switch.sh` | Leg 2d only: the override publishes at T+60 s and T+180 s, plus the acks. | base broker | **the base transmits** (0x63 commands) |
| `leg_post.sh` | Collects the capture, post-brackets, `rs12_leg_report --capture`, `frag_gap_report`, P2/P4 and the tractor-log, then **parks** (with retries). | both | parks; never transmits |
| `leg_replay.py` | PC only: replays a base capture (seq gaps per codec run, resync episodes, last DIGEST). | none | none |

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
   - Each board should hold ≥ 19 entries.
3. **Moving content.** In PowerShell:

   ```powershell
   .\youtube_window.ps1 -Open
   ```

   Start it at least a minute before the first check, because two pre-roll
   ads play first.
4. **DTS carrier.** This step is needed before the first profile-2 leg of the
   day.
   - Run a receive-only spot-check with the tractor parked:
     [`../../channel_survey_sniff.py`](../../channel_survey_sniff.py) or
     [`../../hunt_sniff.ps1`](../../hunt_sniff.ps1), then
     `tools/survey_compare.py`.
   - Put the pick in `bench.env` as `DTS_CARRIER_HZ=<Hz>` with
     `DTS_CARRIER_DATE=<today>`.
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
   -KfRequestDisable 1 -ProbeEcho 0 -LogFragArrivals 1 -TxBatch 0 `
   -CamExtraEnv "-e LIFETRAC_ENCODE_MODE=9 -e LIFETRAC_VECTOR_DETAIL=80" -Archive
```

```bash
# 3. while frames flow (Git Bash):
bash link_sample.sh 2a
# 4. after the harness prints "[EVIDENCE] archived to ...": collect, report, park
bash leg_post.sh 2a          # or: bash leg_post.sh 2a <archive dir>
```

**Step 5 is the park.**
- `leg_post.sh` parks both radios last. It retries a `PARK_TRANSIENT` after
  60 s, up to 3 attempts, and logs every attempt in `leg2a_park.txt`.
- If it ends with `PARK NOT CONFIRMED`, run `bash power_up_guard.sh`. It parks
  any board that is not in `0x80`.
- At the end of the session, run `bash power_up_guard.sh --check-only`.

A few cautions about the harness command:
- Do not drop `-ForceFrfHz` on a profile-2 leg. Without it the harness tunes
  915.000 MHz.
- Leave out `-HostIp` when `PC_HOST` is empty.
- The current harness silently ignores a misspelled parameter (for example
  `-DurationSeconds`). Copy the generated command rather than typing it.

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

How to fly and read them:
- For P3, interleave the control with the VECTOR legs (2c, 2a, 2c′, 2a′).
- At 1 fps, read 2b's P2 and P3 from lock.
- The pass criteria are in RS13_VECTOR_LEG.md, step 2.

## Step 1: camera-only dry run (no radio)

Three passes. Each pass runs its own scene check and recreates `bench_mqtt`
(0 retained messages):

```bash
bash stage_boards.sh tractor                   # code tree + bench_mqtt.conf on the tractor
bash step1_pass.sh bw250 moving --from-work    # the railroad video (scene check >= 8 %)
bash step1_pass.sh bw250 landscape --from-work # a still (scene check records only)
bash step1_pass.sh bw500 landscape --from-work
```

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
  the scene check and pulls the pass files into the evidence folder.
- **`stage_boards.sh`** also stages `/tmp/lifetrac_p0c/08_boot_user_app.cfg`
  LF-clean. The harness's openocd reset of the tractor uses it. The staging
  is md5-checked.
- **`leg_prep.sh`**:
  - enforces the DTS carrier;
  - aborts on a radio-UART holder;
  - picks the boot mode and strictness by tag;
  - writes the harness command.
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
  pushes `08_boot_user_app.cfg` from the working tree, and with
  `core.autocrlf=true` that copy has CRLF line endings. `stage_boards.sh`
  stages an LF copy, which the harness then overwrites.
- **`--capture` is adb-only.** `power_up_guard.sh --capture` calls
  `pull_board_state.sh`, which is adb-only and has the OSE bench serials built
  in.
