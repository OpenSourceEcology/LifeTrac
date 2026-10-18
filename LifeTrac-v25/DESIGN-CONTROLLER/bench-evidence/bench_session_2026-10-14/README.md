# No-radio bench session, 2026-10-14/15, and the 2026-10-18 power cycle

**Status:** steps 1, 2, 3 and 5 done, with no radio use. Step 4, the RS-13.2
desk check, was set up and passed its setup checks, but no replay was run.
It was torn down on 2026-10-18 after the operator power-cycled both boards.
That power-up was also the first boot after the 2026-10-12 unit retirement,
and it confirmed the retirement on both boards. One result needs a re-run:
both bw500 step-1 passes encoded at the 203 B cold-start clamp, not at
243 B (see "The bw500 passes ran at 203 B"). After the power cycle both
radios were parked at 0x80. Later the same day, under the operator's GO, the
L072s were flashed with the step-5 bench image for the RS-12.15
reverse-delivery round. That round is recorded in
[`../RS_12_15_reverse_delivery_2026-10-18/`](../RS_12_15_reverse_delivery_2026-10-18/).

**Clocks.** All times are PC UTC unless marked otherwise. The base's clock
reads wall time once it is up. The tractor's clock does not. It read about
19 days 7 hours behind the PC on both days: at PC 23:54:42Z on 10-14 the
tractor's `uptime` printed 16:48:10 on 2026-09-25
([`02_power_up_guard.txt`](boot_deploy_images/02_power_up_guard.txt),
[`01_boot_snapshot_tractor.txt`](boot_deploy_images/01_boot_snapshot_tractor.txt)).
At PC 15:37:52Z on 10-18 it printed 08:30:07 on 2026-09-29
([`04_power_up_guard.txt`](power_cycle_2026-10-18/04_power_up_guard.txt),
[`03_nextboot_tractor.txt`](power_cycle_2026-10-18/03_nextboot_tractor.txt)).
Every timestamp written by the tractor (the camera logs, the tractor
snapshots) uses that clock.

## What the session was

The operator set five steps for the night of 2026-10-14, with no radio GO:

1. deploy the base tree and rebuild `lifetrac-v25:latest` with
   `deploy_base.sh` (its first real run);
2. build the tractor image `lifetrac-tractor-x8` on the base with
   `build_tractor_image.sh` and load it on the tractor;
3. the camera-only steps 0 and 1 of
   [RS13_VECTOR_LEG.md](../../firmware/x8_lora_bootloader_helper/bench_tools/RS13_VECTOR_LEG.md),
   run from the new image;
4. the RS-13.2 desk check
   ([RS13_2_DESK_CHECK.md](../../firmware/x8_lora_bootloader_helper/bench_tools/RS13_2_DESK_CHECK.md));
5. build the L072 images of PR #152 on the bench PC.

The work ran from about 2026-10-14 23:54Z to 2026-10-15 00:48Z. The
deployed tree and both images are `main` `f6e4b463` (merge of #151). The
tools ran from the session branch `bench-session-2026-10-14`: `f6e4b463`
plus the `adb_sh` fix `fb96439c`, made during step 2. The desk check stopped
after its bring-up (Step 3 of RS13_2_DESK_CHECK.md). The
boards were then left as they were until 2026-10-18, when the operator
power-cycled both at about 15:28Z.

## Boot checks, 2026-10-14

Both boards had been up since the 2026-10-10 power-up: `up 4 days, 6:55`
on the base at 23:54:03Z, and `up 4 days, 6:56` on the tractor
([`01_boot_snapshot_base.txt`](boot_deploy_images/01_boot_snapshot_base.txt),
[`01_boot_snapshot_tractor.txt`](boot_deploy_images/01_boot_snapshot_tractor.txt)).
On both boards `systemctl --failed` listed only the stock
`psplash-start.service`. `lifetrac-camera.service` was inactive. `uart_chk`
printed `__UART_CHK_DONE__` with nothing before it, so no process held
`/dev/ttymxc3`. The retire directory `20261012T212515Z` was present. The
only running LifeTrac container was the base's bench broker
`design-controller-mosquitto-1`.

`provision_bench_board.sh --check`
([base](boot_deploy_images/00_prov_check_base.txt), over ssh;
[tractor](boot_deploy_images/00_prov_check_tractor.txt), over adb) read
`ok` for "old boot units retired" on both boards. The remaining TODO lines
are older than the 10-12 retirement and are unchanged by it. On the base:
the probe image `arduino-ootb-python-devel:738bc44` is missing. On the
tractor: the NetworkManager WiFi radio flag, one stored WiFi profile and the
camera USB provisioning.

The power-up guard found no radio daemon or UART holder on either board.
Both radios read `RADIO_STATE` `0x80` (SLEEP), the tractor at 23:54:47Z and
the base at 23:54:57Z ([`02_power_up_guard.txt`](boot_deploy_images/02_power_up_guard.txt)).

## Step 1: base deploy (`deploy_base.sh`, first real run)

A dry run at 23:55:18Z printed the plan and changed nothing
([`10_deploy_dryrun.txt`](boot_deploy_images/10_deploy_dryrun.txt)). The
plan: archive `f6e4b463`, a locked build, recreate the bench broker alone,
rename the old video-test compose file, and leave out
`firmware/tractor_x8/docker-compose*.yml` and `mosquitto.video-test.conf`.

The real run went from 23:55:41Z to 23:57:16Z and exited 0
([`12_deploy_base.txt`](boot_deploy_images/12_deploy_base.txt)). It copied
the tarball (md5 `89134759b20468c0afaf14929601b43a`, checked on the board).
It renamed `docker-compose.video-test.yml` to
`docker-compose.video-test.yml.RETIRED-2026-10-14`. It saved the old tree to
`/home/fio/lifetrac_deploy/prev_tree_deployed_20261014T235548Z.tgz` and
extracted `f6e4b463` (`clean=1`). It ran
`docker build -t lifetrac-v25:latest -t lifetrac-v25:f6e4b463 --build-arg LOCKED=1`,
which kept the replaced image as `lifetrac-v25:previous` (`4623980c2dac`) and
built `0413f3c4dac3`. It then recreated the broker with
`docker compose -p design-controller up -d --no-deps mosquitto`. The broker
answered (`CONNACK accepted on 127.0.0.1:1883`). Last, the run wrote
`DEPLOYED_FROM.txt`. Its final line is "The radios were not touched." The
compose warnings that `LIFETRAC_MQTT_HOST` is not set come from the compose
file's variable substitution. The broker does not use that variable.

| | before, 23:55:32Z ([`11_base_pre_deploy.txt`](boot_deploy_images/11_base_pre_deploy.txt)) | after, 23:57:38Z ([`13_base_post_deploy.txt`](boot_deploy_images/13_base_post_deploy.txt)) |
|---|---|---|
| `DEPLOYED_FROM.txt` | `deployed-from=d3751286 2026-09-15T22:28Z` | `f6e4b4632296…`, `deployed_utc=2026-10-14T23:55:45Z`, tarball md5 `89134759…`, image `sha256:0413f3c4dac3…`, `build_locked=1` |
| `lifetrac-v25` tags | `:latest` `4623980c2dac` | `:latest` and `:f6e4b463` `0413f3c4dac3`; `:previous` `4623980c2dac` |
| video-test compose file at the top | `docker-compose.video-test.yml` (Sep 15), so armed | `docker-compose.video-test.yml.RETIRED-2026-10-14` |
| `deploy/` | present | absent |
| compose files in `firmware/tractor_x8/` | `docker-compose.video-test.v2.yml`, `docker-compose.video-test.yml`, `docker-compose.yml`, `mosquitto.video-test.conf` | none: the search for compose files there printed nothing |
| bench broker `design-controller-mosquitto-1` | image `2a23a3e427e4`, created 2026-07-26T01:14:41Z | image `0db28dca6f11` (`eclipse-mosquitto:2` on the base), created 2026-10-14T23:57:12Z, `restart=unless-stopped` |
| `systemctl --version` | `systemd 250 (250.5+)` | `systemd 250 (250.5+)` |
| `uart_chk` | (boot snapshot) nothing before the marker | nothing before the marker |
| failed units | (boot snapshot) `psplash-start.service` only | `psplash-start.service` only |

`ls firmware/tractor_x8/` after the deploy:
`Dockerfile`, `README-DEPLOY.md`, `camera_service.py`, `dry_run_w2_05.py`,
`gps_service.py`, `image_tx_daemon.py`, `imu_service.py`,
`logger_service.py`, `method_compare.py`, `params_service.py`,
`requirements.lock.txt`, `requirements.txt`, `systemd/`, `time_service.py`,
`x8_image_pipeline/`.

The May backup `docker-compose.video-test.yml.bak_20260527_152255` is still
in the deployed tree, before and after. Compose does not find a file by that
name unless it is named with `-f`, and the deploy script only renames the
exact name.

## Step 2: tractor image

`build_tractor_image.sh` started at 23:58:02Z at `f6e4b463`
([`20_build_tractor_image.txt`](boot_deploy_images/20_build_tractor_image.txt)).
On the base it ran
`docker build -t lifetrac-tractor-x8:f6e4b463 -t lifetrac-tractor-x8:latest --build-arg LOCKED=1`,
which finished at 23:59:09Z as
`sha256:b2a3c8070240cd068c6e125fe0067cb4cf026b308f602701aca35756ab43010f`.
The image check printed `numpy 2.4.6 cv2 5.0.0 encoder VectorEncoder`. The
script saved the image (271420494 B) and copied it to the PC archive, where
the md5 `69780dc885c6d1d4257b195b19bbbc92` was verified at 00:02:49Z.

The first adb call to the tractor then failed:
`build_tractor_image.sh: line 171: return: 0: numeric argument required`,
exit 2. Line 171 is `return "$rc"` in `adb_sh`. As the message of commit
`fb96439c` explains, adb from platform-tools 37.0.0 ends each line with
`\r\r\n`. `adb_sh` stripped one `\r`, so the remote status reached `return`
as `0\r`. At that point the image was built
and its PC copy verified, and nothing had been loaded on the tractor. Commit
`fb96439c` makes `adb_sh` strip every trailing CR, in
`build_tractor_image.sh` and in `deploy_base.sh` (whose adb transport has
the same code; this deploy went over ssh). The commit message records the
check on the tractor, `true` → 0 and `false` → 1.

`build_tractor_image.sh --push-only` then loaded the verified tarball
between 00:26:22Z and 00:28:27Z and exited 0
([`21_push_tractor_image.txt`](boot_deploy_images/21_push_tractor_image.txt)).
The push took 87.362 s. After the load, `lifetrac-tractor-x8:f6e4b463` and
`:latest` are `b2a3c8070240`, `:previous` is the old `:latest`
`2727dfd36f9f` (also tagged `:rs13-65869517`), and `:pre-rs13`
`9bfbbc8d06cb` is unchanged. The same numpy / cv2 line printed on the
tractor. The script's last line: "No container was restarted and the radios
were not touched."

## Step 3: camera-only steps 0 and 1

### Step 0: encoder suites in the image

`tests.test_vector_encoder`, `tests.test_vector_interop` and
`tests.test_vs1_codec_parity_sil` were run inside `b2a3c8070240`, on the
base, against the deployed `f6e4b463` tree: `Ran 47 tests in 11.316s`, `OK`
([`30_step0_encoder_suites_in_image.txt`](boot_deploy_images/30_step0_encoder_suites_in_image.txt)).

### Staging and scene

At 00:29:20Z the tractor's `/tmp/lifetrac_strict` was staged from
`fb96439c`: 15 helper files and a 37-file code tree, all md5 `ok`
([`staging_tractor.txt`](step1/staging_tractor.txt)). The passes ran
`camera_service.py` and `x8_image_pipeline` from the image's `/app`, with
`lora_proto` and `image_pipeline` taken from the staged tree (`/work`), as
RS13_VECTOR_LEG.md step 1 requires. Every boot check printed
`/app/x8_image_pipeline/encode_vector.py True`.

The moving content was the railroad video in a normal Firefox window with
only the player fullscreen. Three preflight scene checks ran first
([`preflight_moving_scene.txt`](step1/preflight_moving_scene.txt),
[`…moving2…`](step1/preflight_moving2_scene.txt),
[`…moving3…`](step1/preflight_moving3_scene.txt)). The first one is the
false FAIL described under "scene_check.sh can miss its 10 s pair". Before
each pass `bench_mqtt` was recreated on the tractor with 0 retained messages
([`step1_bench_mqtt_reset.txt`](step1/step1_bench_mqtt_reset.txt)).

### Results

All four passes: RESULT PASS, 10 of 10 checks. Capture side, from
`vector_dry_run.py capture` on the tractor's broker (the `_transcript.txt`
and `.json` of each pass):

| pass (start) | profile limit | frames | arrival | wire B min / p50 / p95 / max | epoch starts recv / applied | digest checked / mismatched | scene at the end | scene check before the pass |
|---|---|---|---|---|---|---|---|---|
| [bw250 moving](step1/step1_bw250_moving_transcript.txt) (00:33:19Z) | 203 B | 250 | 2.00 fps over 124.4 s, gap max 0.63 s | 102 / 200 / 203 / 203 | 3 / 3 | 215 / 0 | epoch 2, L1 28, L2 0, L3 5, L4 0, horizon none | [29.26 %](step1/step1_bw250_moving_scene.txt) at 9.7 s (min 8 %) |
| [bw500 moving](step1/step1_bw500_moving_transcript.txt) (00:36:10Z) | 243 B | 250 | 2.00 fps over 124.4 s, gap max 0.62 s | 46 / 201 / 203 / 203 | 24 / 24 | 194 / 0 | epoch 7, L1 23, L2 0, L3 7, L4 0, horizon none | [38.58 %](step1/step1_bw500_moving_scene.txt) at 9.7 s (min 8 %) |
| [bw250 landscape](step1/step1_bw250_landscape_transcript.txt) (00:40:07Z) | 203 B | 251 | 2.00 fps over 125.0 s, gap max 0.58 s | 119 / 198 / 203 / 203 | 3 / 3 | 215 / 0 | epoch 2, L1 25, L2 0, L3 9, L4 0, horizon resid | [1.57 %](step1/step1_bw250_landscape_scene.txt) at 9.7 s (min 0 %, record only) |
| [bw500 landscape](step1/step1_bw500_landscape_transcript.txt) (00:42:58Z) | 243 B | 251 | 2.00 fps over 124.9 s, gap max 0.58 s | 85 / 188 / 202 / 203 | 3 / 3 | 217 / 0 | epoch 2, L1 25, L2 0, L3 5, L4 0, horizon abs | [1.94 %](step1/step1_bw500_landscape_scene.txt) at 9.7 s (min 0 %, record only) |

Encoder side, from `vector_dry_run.py tractor-log` on each camera log
(`step1_tractor_log_<pass>.txt`, `step1_camera_service_<pass>.log`):

| pass | VECTOR budget in effect | encoder `ms_total` p50 / p95 / max | frame B p50 / max | frames counted | epoch starts: first / forced / horizon / safety / relabel |
|---|---|---|---|---|---|
| [bw250 moving](step1/step1_tractor_log_bw250_moving.txt) | 203 B | 241.7 / 275.1 / 349.2 ms | 200 / 203 | 251 | 1 / 0 / 0 / 2 / 0 |
| [bw500 moving](step1/step1_tractor_log_bw500_moving.txt) | **203 B** (clamped; see below) | 243.0 / 260.4 / 287.7 ms | 202 / 203 | 251 | 1 / 0 / 4 / 0 / 19 |
| [bw250 landscape](step1/step1_tractor_log_bw250_landscape.txt) | 203 B | 225.3 / 248.3 / 318.6 ms | 197 / 203 | 253 | 1 / 0 / 0 / 2 / 0 |
| [bw500 landscape](step1/step1_tractor_log_bw500_landscape.txt) | **203 B** (clamped; see below) | 224.6 / 261.5 / 317.1 ms | 192 / 203 | 253 | 1 / 0 / 0 / 2 / 0 |

"Frames counted" is the encoder's own `frames` counter on its last
`vector_stats` line. The capture's 130 s window took 250 or 251 of those
frames. The landscape scene checks ran on their own just before their
passes, which then ran without one. The `content:` field in a scene file
always names the video URL, whatever is on screen, and the saved `.jpg`
frames are not in this folder. So the files do not record which still was
shown. They do record that the scene was nearly static: 1.57 % and 1.94 %
changed, against RS13_VECTOR_LEG.md's 1.9–2.0 % reference for the
landscape still.

### The bw500 passes ran at 203 B, not 243 B

Both bw500 camera logs print `byte_budget=243 B/frame (LIFETRAC_FRAGMENT_BUDGET, phy=image_bw500)`.
One second later they print
`VECTOR budget 203 B (one fragment) until link_budget arrives (provisional 243 B)`
([moving](step1/step1_camera_service_bw500_moving.log), line 6;
[landscape](step1/step1_camera_service_bw500_landscape.log), line 6). No
frame in either pass is larger than 203 B.

The cause is the RS-13 cold-start guard in
[`camera_service.py`](../../firmware/tractor_x8/camera_service.py)
(2026-10-10). Until a `link_budget` arrives, `_vector_byte_budget()` holds
VECTOR to `min(provisional, _vector_cold_budget())`.
`_vector_cold_budget()` takes one fragment of the profile named by
`LIFETRAC_REG_PROFILE`, and the smallest image profile (image_bw250,
203 B) when that variable is not set. Step 1 runs no tx daemon and uses a
fresh broker, so no `link_budget` ever arrives.
[`legs/step1_pass.sh`](../../firmware/x8_lora_bootloader_helper/bench_tools/legs/step1_pass.sh)
passes `LIFETRAC_FRAGMENT_BUDGET=1` and `LIFETRAC_FRAGMENT_PROFILE`, but not
`LIFETRAC_REG_PROFILE`. The bw250 logs have no such line, because the clamp
equals the provisional 203 B and the line is printed only when they differ.

None of the pass's checks catch this. `one_fragment` compares against the
profile limit (243 B), and the boot-log check expects `byte_budget=243`,
which is printed. Its grep pattern (`byte_budget|link_budget: ->|…|clamp|…`)
does not match the `VECTOR budget` line. So the bw500 rows above are a
second set of 203 B measurements on a different stretch of video. They are
not a 243 B characterization. They also do not compare bw250 with bw500:
the 3 against 24 epoch starts of the two moving passes happened under the
same budget.

### scene_check.sh can miss its 10 s pair

[`legs/scene_check.sh`](../../firmware/x8_lora_bootloader_helper/bench_tools/legs/scene_check.sh)
starts its 10.5 s clock right after it opens `/dev/video1`, before the
first frame arrives. It then pairs `fr[5]` with the frame closest to 10 s
later, and passes only on a "9.x" or "10.x s apart" line. If the camera is
slow to deliver its first frame, no pair is that far apart. The check then
reports `SCENE-CHECK-FAIL: 10 s change ? %` however much the scene moved.
In [`preflight_moving_scene.txt`](step1/preflight_moving_scene.txt) the
longest pair was 7.2 s apart, with 75.21 % of pixels changed, so by the
script's own test that check failed. The FAIL line goes to the console, not
into the file. Every other check this session reached 9.7 s at most, never
10.0 s.

### After step 3

Afterwards the tractor's `camera_svc` and capture containers were gone.
`bench_mqtt` was still Up, `lifetrac-camera.service` was inactive,
`uart_chk` and the `/dev/video1` holder check printed nothing
([`39_tractor_after_step3.txt`](boot_deploy_images/39_tractor_after_step3.txt)).

## Step 4: RS-13.2 desk check (set up, not replayed)

Setup checks, against RS13_2_DESK_CHECK.md:

- **Step 0** (PC, tool at `fb96439c`): the four dry runs gave 596 / 558 / 591
  / 600 rows and 305.4 / 279.5 / 305.4 / 307.0 s per pass, as the procedure
  expects ([`step0_dry_run.txt`](desk_check/logs/step0_dry_run.txt)).
- **Step 1** (image): `lifetrac-v25:latest` is `0413f3c4dac3`. The count of
  `CODEC_VECTOR` in `web_ui.py` is 2. `vector_scene_store.py` and
  `vector_renderer.js` are present. `SAFETY DETECTOR` appears 2 times in
  the renderer and `frame_age_ms` 3 times in the store, so the image carries
  the 2026-10-14 renderer fixes. `DEPLOYED_FROM.txt` names `f6e4b463`
  ([`step1_image_check.txt`](desk_check/logs/step1_image_check.txt)).
- **Step 2** (staging `/tmp/rs13_2` on the base) left no log of its own.
- **Step 3**, first attempt: the bring-up script refused with
  `ABORT: leftover desk or leg container`. The only container it could have
  meant is `rx_smoke` (Exited (137) 10 days ago), an exited harness leftover
  that Step 3 lists as fine
  ([`step3_bringup_attempt1_guard_abort.txt`](desk_check/logs/step3_bringup_attempt1_guard_abort.txt)).
  The script is not in this folder.
- **Step 3**, second attempt
  ([`step3_bringup.txt`](desk_check/logs/step3_bringup.txt)): `uart_chk`
  empty; `0 unit files listed` for `lifetrac-base*`; only the bench broker
  Up; the video-test file only as `.RETIRED-2026-10-14`. **The bench broker
  was stopped at 2026-10-15T00:47:31Z**, and netstat then showed 1883 and
  8080 free. `config --services` printed `mosquitto`, `audit_tail`,
  `web_ui`, with no `lora_bridge`. `up mosquitto web_ui` started
  `lifetrac-desk-mosquitto-1` on `127.0.0.1:1883` and
  `lifetrac-desk-web_ui-1` on `0.0.0.0:8080`. web_ui logged "Application
  startup complete", and `uart_chk` was still empty afterwards.

Steps 4 to 7 (the console, the replays, the checklist, the findings) were
not done. The stack stayed up, unused, until the 2026-10-18 power cycle.

**Teardown, 2026-10-18**
([`step8_teardown_after_reboot.txt`](desk_check/logs/step8_teardown_after_reboot.txt)).
After the reboot `/tmp/rs13_2` was gone, together with the override file
and the PIN. The stack was therefore torn down by project name, as Step 8's
"If the base rebooted" case says, at 15:37:27Z. That removed the two
containers, the four `lifetrac-desk_*` volumes and the network, and left no
`lifetrac-desk` leftovers. 1883 and 8080 were free. **The bench broker was
started at 15:37:28Z**, Up on `127.0.0.1:1883`. It still held five retained
topics from earlier radio sessions, not from the desk check, which never
published into it: `control/link_airtime`,
`control/encode_mode_override {"mode":"mono_g4"}`,
`status/radio_profile/rx` (`"reverted: link silent"`),
`status/radio_profile/tx` and `status/encode_mode` (mono_g4). The harness
cleared `encode_mode_override`, `req_keyframe` and `radio_profile` on the
base at the start of the next radio leg
([`legV_clear_retained.txt`](../RS_12_15_reverse_delivery_2026-10-18/legs/legV_clear_retained.txt)).

**The container logs are empty.**
[`mosquitto_container.log`](desk_check/logs/mosquitto_container.log) and
[`web_ui_container.log`](desk_check/logs/web_ui_container.log) are 0 bytes.
The base's docker logging driver is `journald`
([`board_state_2026-10-04/base/60_docker.txt`](../board_state_2026-10-04/base/60_docker.txt)).
After the power cycle the journal listed only the current boot
([`03_nextboot_base.txt`](power_cycle_2026-10-18/03_nextboot_base.txt)), so
the desk containers' output from the previous boot was gone. The only
record of web_ui's start is the `logs --tail 30` in `step3_bringup.txt`.

## 2026-10-18 power cycle: the first boot after the unit retirement

The operator power-cycled both boards at about 15:28Z. At 15:37:02Z the
base read `up 9 min`, and the tractor read `up 9 min` at 08:29:19 on its own
clock ([`03_nextboot_base.txt`](power_cycle_2026-10-18/03_nextboot_base.txt),
[`03_nextboot_tractor.txt`](power_cycle_2026-10-18/03_nextboot_tractor.txt)).
No board had been rebooted since the three boot units were retired on
2026-10-12
([retirement README](../board_state_2026-10-12_unit_retirement/README.md)),
so this boot is the "next boot" that
[BENCH_BOARDS.md](../../firmware/x8_lora_bootloader_helper/bench_tools/BENCH_BOARDS.md#not-yet-observed-the-next-boot)
lists checks for.

**Base on USB.** The base came back on USB this time.
`provision_bench_board.sh` reached it over adb (`2D0A1209DABC240B`), where
on 10-14 it had used ssh
([`02_prov_check_base.txt`](power_cycle_2026-10-18/02_prov_check_base.txt)
line 1, against
[`00_prov_check_base.txt`](boot_deploy_images/00_prov_check_base.txt)).
The two outputs are otherwise identical. ssh worked as well: the power-up
guard used it. BENCH_BOARDS.md records that after the 10-10 power cycle the
base did not enumerate on USB.

**Tractor camera.** `lifetrac-camera.service` is `disabled`
(`03_nextboot_tractor.txt`), but it came up at boot anyway. Its udev rule
starts it when the camera enumerates
([`99-w2-01-c2.rules`](../../firmware/x8_lora_bootloader_helper/99-w2-01-c2.rules),
line 37). It started `tractor-mosquitto` and `tractor-camera` at 08:20:27–29
on the tractor's clock, both on `lifetrac-tractor-x8:latest`, which is
`b2a3c8070240` since step 2. At 08:28:33 tractor time (about 15:36Z) the
camera was stopped before any probe, as at every power-up. Afterwards the
unit read `failed`, the x8-* containers were the only ones Up, and
`uart_chk` was empty
([`01_tractor_camera_stop.txt`](power_cycle_2026-10-18/01_tractor_camera_stop.txt)).
Its log shows `encode_mode -> 0 (full) [boot]` and
`VECTOR budget 40 B (one fragment) until link_budget arrives (provisional None B)`.
The 40 B is the `n_fragments * 40` fallback that `camera_service` uses when
`lora_proto` does not import. The image carries no `lora_proto.py` (its
build context is `firmware/tractor_x8`), as RS13_VECTOR_LEG.md step 1 says.

The checks of BENCH_BOARDS.md "Not yet observed: the next boot", plus the
journal of the retired units:

| check | base | tractor |
|---|---|---|
| 1. `provision_bench_board.sh --check`, "old boot units retired" | `ok` ([`02_prov_check_base.txt`](power_cycle_2026-10-18/02_prov_check_base.txt)) | `ok`; "radio UART free" `ok`, because the camera was already stopped ([`02_prov_check_tractor.txt`](power_cycle_2026-10-18/02_prov_check_tractor.txt)) |
| 2. `docker ps`, no `lifetrac-vtest-*` / `tractor-*-v2` | none listed. Nothing was Up: the bench broker was `Exited (0) 3 days ago`, because it had been stopped by hand at 10-15 00:47:31Z for the desk check, and `unless-stopped` does not restart a container stopped by hand (RS13_2_DESK_CHECK.md, Step 3). The two `lifetrac-desk-*` containers were Exited (`restart: "no"`) | none listed; x8-* Up; `tractor-camera` and `tractor-mosquitto` had started and were stopped, as above |
| 3. camera unit up by udev, then stopped; `uart_chk` | n/a; `uart_chk` empty | came up, stopped; `uart_chk` empty |
| 4. `systemctl --failed` | `psplash-start.service` only | `lifetrac-camera.service` (failed once its container was stopped) and `psplash-start.service` |
| journal of the three retired units this boot | `-- No entries --` | `-- No entries --` |
| retire directory | `20261012T212515Z` | `20261012T212515Z` |

The retirement held on both boards: no retired unit ran and no unit of
theirs logged a line. The only divergence from what the 10-12 README
expected is that the base broker was not Up. It was started at 15:37:28Z
(step 4 teardown above).

**Radios.** Both L072s booted listening (RXCONT). The power-up guard read
`0x85` RXCONT on the tractor at 15:37:58Z and on the base at 15:38:34Z.
Each was parked at the first attempt (`PARK_OK`, readback `0x80`), and each
then read `0x80` SLEEP, the tractor at 15:38:23Z and the base at 15:39:00Z
([`04_power_up_guard.txt`](power_cycle_2026-10-18/04_power_up_guard.txt)).
A recheck at 15:39:26Z and 15:39:31Z read `0x80` on both
([`05_radio_recheck.txt`](power_cycle_2026-10-18/05_radio_recheck.txt)).
So both radios spent about 10 minutes in RXCONT, from boot until the park.
No leg daemon ran in that time. The tractor's boot-started camera unit,
which BENCH_BOARDS.md records as mapping `/dev/ttymxc3` at boot, ran for
about 8 of those minutes before it was stopped.

## Step 5: L072 build for #152

Log: [`50_l072_152_build.txt`](boot_deploy_images/50_l072_152_build.txt).
The build ran in a separate checkout at the #152 branch head `13f20aa1`.
The committed `firmware.bin` before the build was `589c120323c2d5e7ef9f459d7a4ba42d`.

- `mingw32-make`: exit 0, `.text` 24204 B.
- `mingw32-make check`: exit 0. Every suite passes, among them
  `fhss_clock: 34 base + F6 age/refresh + RS-12.15 leads`,
  `fhss_authority: 7 test fns`, `rx_grid_policy: 11 sequences` and
  `rx_reverse_delivery: 10 SIL sequences against the real sx1276_rx.c/sx1276_tx.c`,
  and the check section ends with "memory map invariants hold". The
  `NativeCommandError` block in the middle of the log is PowerShell
  wrapping the test's `[CASE]` lines on stderr. It is not a failure.
- `mingw32-make bench`: exit 0.
- Production `firmware.bin`: md5 `148de24e1f9acce417c46c2c69a5be6b`,
  24972 B. Bench `firmware_bench_diag.bin`: md5
  `e083ae46a64686031b39866eca42e094`, 24972 B.

Before this build the toolchain was checked on `main` `f6e4b463`. It
reproduced bench `0c1bb0a9573f813137f941dfa47177d0` and production
`589c120323c2d5e7ef9f459d7a4ba42d`. That check is recorded in the message of
commit `8a226570`, not in this log. `8a226570` (2026-10-15 00:00:59Z) put the
production bin and the new md5 pair on the #152 branch. `a3d56d5c`
(2026-10-17) added the ELF, HEX and MAP rebuilt from clean that go with it,
with the same two md5s. #152 was merged as `3c8ced60`. Nothing was flashed
in this session. On 2026-10-18 the bench image was flashed to the tractor
(15:50Z) and the base (15:52Z) for the RS-12.15 round
([`flash_tractor.txt`](../RS_12_15_reverse_delivery_2026-10-18/firmware_flash/flash_tractor.txt),
[`flash_base.txt`](../RS_12_15_reverse_delivery_2026-10-18/firmware_flash/flash_base.txt)).
The boards were left on it with both radios at `0x80`
([`session_end_radio_check.txt`](../RS_12_15_reverse_delivery_2026-10-18/legs/session_end_radio_check.txt)).

## Findings and follow-ups

1. **Step-1 bw500 is not measured at 243 B.** `legs/step1_pass.sh` should
   pass `LIFETRAC_REG_PROFILE` to `camera_svc`: 2 for bw500 (maps to
   image_bw500 in `_REG_PROFILE_LINK_PHY`), 1 for bw250. Its boot-log check
   should also show the `VECTOR budget` line and fail when the budget
   differs from the profile's one-fragment limit. Then re-run the two bw500
   passes. They are camera-only.
2. **scene_check.sh false FAIL.** Start the 10.5 s window at the first frame
   received, or accept the longest pair when it is at least about 9 s. A slow
   camera start then cannot leave the check without a "10 s" line.
3. **RS-13.2 desk check still to run.** The image on the base
   (`0413f3c4dac3`) passed Step 1's checks. Steps 2 to 8 must be done again
   from the start, because the 10-15 staging went with `/tmp`. Save `$C logs`
   before teardown and before any reboot: with the journald driver they do
   not survive a reboot. The bring-up guard that refused on an exited
   `rx_smoke` should allow exited harness leftovers, as Step 3 does.
4. **`fb96439c` is only on the session branch.** It needs to reach `main`
   with this evidence. `deploy_base.sh`'s adb path has the same fix but has
   not been exercised. Now that the base is back on adb, it could be.
5. **BENCH_BOARDS.md is out of date in three places.** "Not yet observed:
   the next boot" has now been observed (above), which also settles the open
   item "Confirm the retirement at the next boot of each board". The base's
   access line says it did not enumerate on USB after the 10-10 power cycle,
   and on 10-18 it did.
6. **Production camera unit, 40 B VECTOR cold budget.** When
   `lifetrac-camera.service` runs `b2a3c8070240` without the `/work` mount,
   it cannot import `lora_proto` and would hold VECTOR to 40 B until a
   `link_budget` arrives. This does no harm today: it boots in mode 0 and is
   stopped at every bench power-up. It matters for any production use of
   VECTOR. Either `lora_proto.py` goes into the tractor image, or
   `fragment.py` stops depending on it.
7. Low priority: the May copy `docker-compose.video-test.yml.bak_20260527_152255`
   is still in the base's deployed tree.

## Evidence index

`boot_deploy_images/`, 2026-10-14/15:

- `00_prov_check_base.txt`, `00_prov_check_tractor.txt`: `provision_bench_board.sh --check`, read-only.
- `01_boot_snapshot_base.txt`, `01_boot_snapshot_tractor.txt`: time, uptime, journal boots, `systemctl --version`, containers, camera unit, failed units, `uart_chk`, retire directory.
- `02_power_up_guard.txt`: guard in park-if-listening mode; both radios already `0x80`.
- `10_deploy_dryrun.txt`, `12_deploy_base.txt`: `deploy_base.sh` dry run and real run.
- `11_base_pre_deploy.txt`, `13_base_post_deploy.txt`: the base tree, images, broker and disk before and after.
- `20_build_tractor_image.txt`: the image build and the `return: 0` failure.
- `21_push_tractor_image.txt`: `--push-only` load on the tractor, after `fb96439c`.
- `30_step0_encoder_suites_in_image.txt`: the 47 encoder tests in `b2a3c8070240`.
- `39_tractor_after_step3.txt`: the tractor after the step-1 passes.
- `50_l072_152_build.txt`: `mingw32-make`, `check` and `bench` at `13f20aa1`, with the md5 pair.

`step1/`, 2026-10-15 00:29Z–00:45Z:

- `staging_tractor.txt`: md5 of the staged helper files and code tree.
- `preflight_moving{,2,3}_scene.txt`: the three preflight scene checks.
- `step1_bench_mqtt_reset.txt`: retained messages before and after each broker recreate.
- per pass `<bw>_<scene>`, for bw250/bw500 × moving/landscape:
  - `step1_<pass>_transcript.txt`: the console of the pass;
  - `step1_<pass>_scene.txt`: its scene check;
  - `step1_<pass>_bootcheck.txt`: the boot-log check;
  - `step1_tractor_<pass>.{jsonl,json,txt}`: the capture (frames, JSON report, summary);
  - `step1_camera_service_<pass>.log`: the full camera log;
  - `step1_tractor_log_<pass>.txt`: the encoder timing and counters.

`desk_check/logs/`, 2026-10-15 and 2026-10-18:

- `step0_dry_run.txt`, `step1_image_check.txt`: procedure Steps 0 and 1.
- `step3_bringup_attempt1_guard_abort.txt`, `step3_bringup.txt`: the bring-up; the bench broker stopped 00:47:31Z.
- `step8_teardown_after_reboot.txt`: teardown by project name on 10-18; the bench broker started 15:37:28Z; its retained topics.
- `mosquitto_container.log`, `web_ui_container.log`: empty (journald; see Step 4).

`power_cycle_2026-10-18/`:

- `01_tractor_camera_stop.txt`: the boot-started camera stopped, with its log.
- `02_prov_check_base.txt` (over adb), `02_prov_check_tractor.txt`: `provision_bench_board.sh --check`.
- `03_nextboot_base.txt`, `03_nextboot_tractor.txt`: uptime, containers, units, `uart_chk`, retire directory, journal of the retired units.
- `04_power_up_guard.txt`: RXCONT `0x85` on both, then parked to `0x80`.
- `05_radio_recheck.txt`: `0x80` on both.
