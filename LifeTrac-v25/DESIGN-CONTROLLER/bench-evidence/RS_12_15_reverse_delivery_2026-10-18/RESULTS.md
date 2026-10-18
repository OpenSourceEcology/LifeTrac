# RS-12.15 reverse delivery — on-air round, 2026-10-18

**Status: PASS.** On 2026-10-18, under the operator's GO, both L072s were
flashed with the RS-12.15 reverse-delivery fix (bench build `e083ae46`) and
four profile-1 legs were flown. Legs V (camera), W2 (synthetic feed, the
re-fly of W) and X (synthetic feed, commands only after the tractor's scan
machine had reached FAILED) meet every PASS criterion of the TODO item
([TODO.md](../../TODO.md), "RS-12.15 (reverse-delivery half)"). The tractor
heard 22/23 (V), 72/72 (W2) and 36/36 (X) of the base's commands. On the v2
firmware it heard 2/43 and 7/49 on the camera legs T and U, and 19/72 on the
synthetic leg R. No stop rule tripped. Leg W as flown started its injector
2 min 41 s late. It is reported below but not scored (see "Leg W"). Both
boards were left on `e083ae46`, and a check at 16:41Z found both radios
parked (`0x80`).

The leg letters V, W and X follow the TODO's plan. This leg V is not the
RS-1.4 leg V of 2026-09-15
([RS_1_4_auto_policy_2026-09-15](../RS_1_4_auto_policy_2026-09-15/RESULTS.md)).

All times are UTC. The PC and base clocks agree to within a few seconds; the
tractor's clock does not (see "Reading the tractor's timestamps").

## What the fix changes

This summarises the TODO item. On v2, the tractor's receiver followed the
FHSS slot clock only while its scan machine was LOCKED. Between its own
trains it listened on its last TX channel or on the scan walker's channel,
so it missed most base commands. With the fix, the receiver follows the slot
clock in two cases: when the grid is adopted and LOCKED, or when the node
holds originator authority (at least 8 of its own TXs, each less than 1 s
apart). It follows in any scan state, FAILED included. Under authority, the
final scan FAIL no longer parks the receiver. Followers are unchanged. The fix
is PR #152, merged as `3c8ced60` on 2026-10-17. In the SIL harness
`check-rx-reverse-delivery`, delivery went from 17/59 to 59/59 between trains
and from 10/33 to 33/33 in scan FAILED.

## Before the round

- Since 2026-09-15 both L072s had run the RS-12.15 v2 bench build
  `0c1bb0a9573f813137f941dfa47177d0` (the table row
  `firmware_bench_diag_0c1bb0a9.bin` in
  [RS_13_vector_scene_2026-09-26/firmware/README.md](../RS_13_vector_scene_2026-09-26/firmware/README.md)).
  That is the written record, not a readback. The L072 flash is read back
  only by the flasher's verify step, after a write.
- The image flown is `firmware_bench_diag_e083ae46_13f20aa1.bin`: 24 972 B,
  md5 `e083ae46a64686031b39866eca42e094`. It was built on 2026-10-14 from
  `13f20aa1` (then the head of the PR #152 branch), together with the
  production build
  `148de24e1f9acce417c46c2c69a5be6b` (also 24 972 B). The build log is
  [50_l072_152_build.txt](../bench_session_2026-10-14/boot_deploy_images/50_l072_152_build.txt).
  The TODO item records that the toolchain first rebuilt `0c1bb0a9` and
  `589c1203` from `main`.
  - Between `13f20aa1` and the merge `3c8ced60`, the only changes under
    `firmware/murata_l072` are the README and the committed build outputs.
    So the image matches the L072 sources on `main`.
  - A copy of the image is kept here:
    [firmware_flash/firmware_bench_diag_e083ae46.bin](firmware_flash/firmware_bench_diag_e083ae46.bin).
    Its md5 was checked on 2026-10-18.
- At about 15:28 the operator power-cycled both boards (the base reported
  9 min of uptime at 15:37:02,
  [03_nextboot_base.txt](../bench_session_2026-10-14/power_cycle_2026-10-18/03_nextboot_base.txt)).
  The tractor's radio came back up listening (RXCONT, `0x85`). The power-up
  guard parked it (PARK_OK at 15:38:02) and found the base already at `0x80`
  ([04_power_up_guard.txt](../bench_session_2026-10-14/power_cycle_2026-10-18/04_power_up_guard.txt)).
  The rest of that boot is in
  [bench_session_2026-10-14](../bench_session_2026-10-14/).

## Flash and health

Both boards were flashed with `flash_l072.sh <board> <image> --go`
(`REVIVE_MODE=reboot`, after a preflight that stops before openocd, the UART
and the watchdog).

| | tractor `2E2C1209DABC240B` (adb) | base `2D0A1209DABC240B` (ssh) |
|---|---|---|
| L072 before | `0c1bb0a9…` (record, see above) | `0c1bb0a9…` (record) |
| preflight | PREFLIGHT-OK, md5 `e083ae46…` ([preflight_tractor.txt](firmware_flash/preflight_tractor.txt)) | PREFLIGHT-OK, md5 `e083ae46…` ([preflight_base.txt](firmware_flash/preflight_base.txt)) |
| flash started | 15:50:00 ([flash_tractor.txt](firmware_flash/flash_tractor.txt)) | 15:52:15 ([flash_base.txt](firmware_flash/flash_base.txt)) |
| erase | mass erase NACKed, so the flasher erased page by page (196 pages) | same |
| write | 18.8 s | 19.0 s |
| result | `Verify OK`, `flash_rc=0`, `PIPELINE-EXIT=0`, then reboot | `Verify OK`, `flash_rc=0`, `PIPELINE-EXIT=0`, then reboot |
| log checked after the reboot | `Verify OK` / `flash_rc=0` ([tractor_after_flash.txt](firmware_flash/tractor_after_flash.txt)) | `Verify OK` / `flash_rc=0`, up 0 min at 15:54:42 ([base_after_flash.txt](firmware_flash/base_after_flash.txt)) |
| L072 after | `e083ae46a64686031b39866eca42e094` (verify readback against the image) | same |
| rs116 health | 15:56:02: STATS-OK on attempt 1, `radio_state=4`, every radio and FHSS counter 0 except `radio_dio0=1` ([session_health_tractor.txt](legs/session_health_tractor.txt)) | 15:56:00: the same, with `radio_dio0=0` ([session_health_base.txt](legs/session_health_base.txt)) |

The health probe printed `RS115-INSTRUMENTED-FIRMWARE=YES`,
`RS12-URC-COUNTERS=YES` and `RS12-10-COUNTERS=YES` on both boards. The full
v2 counter tail was present: `fhss_dec_*`, `clk_demotion_reset`/`_kept`,
`tx_first_anchor` and `tx_stream_streak_max`. After the reboot,
`tractor-camera` had come back up and was stopped again
(tractor_after_flash.txt).

Between 15:54:56 and 15:55:32, `/tmp/lifetrac_strict` was re-staged on both
boards from `bench-session-2026-10-14` at `d484a92d`: 14 tool files plus the
37-file code tree, every file `ok`. The tractor also got
`08_boot_user_app.cfg`
([staging_tractor.txt](legs/staging_tractor.txt),
[staging_base.txt](legs/staging_base.txt)).

`flash_l072.sh` printed "not a recorded build" for this image. Its md5 list
(`case "$MD5"` in
[flash_l072.sh](../../firmware/x8_lora_bootloader_helper/bench_tools/flash_l072.sh))
does not have `e083ae46` yet, and it still describes `0c1bb0a9` as the build
on both boards.

## Setup

### Harness parameters

Every leg ran `run_live_radio_monitor.ps1` from the bench checkout at
`d484a92d`. The values below are from each archive's `params.txt`.

| key | V | W | W2 | X |
|---|---|---|---|---|
| archive | [110651](../radio_monitor_20261018_110651_d484a92d/params.txt) | [111558](../radio_monitor_20261018_111558_d484a92d/params.txt) | [112639](../radio_monitor_20261018_112639_d484a92d/params.txt) | [113831](../radio_monitor_20261018_113831_d484a92d/params.txt) |
| `tx_feed` | camera | local (synthetic) | local | local |
| `duration_s` | 300 | 300 | 300 | 480 |
| `kf_request_disable` | 0 | 0 | 0 | 1 |

These keys were the same on all four legs:

- `reg_profile=1`, `force_frf_hz=0`.
- `synth_fps=2`, `synth_budget_b=3000`.
- `tx_pipeline=v3`, `tx_pipeline_depth=2`, `tx_batch=1`,
  `tx_prepare_ahead=1`, `train_gap_ms=40`.
- `idle_drain_quiet_s=1.5`, `cmd_stream_min_gap_s=1.0`.
- `aligned_pump=1`, `reactive_fire=0`, `ack_copies=2`, `rxcont_arm=1`,
  `pacing_mode=smooth`, `parity_group=0`, `airtime_budget_us=0`.
- `tx_power_dbm` empty (the default).
- `tx_serial=2E2C1209DABC240B` (tractor), `rx_serial=2D0A1209DABC240B`
  (base).

The old-firmware reference legs had the same `params.txt` apart from
`git_sha`, `timestamp` and two keys added since then (`cam_extra_env`, which
was empty, and `allow_default_carrier=False`). V was diffed against T
(`radio_monitor_20260914_171517_a75089c0`) and U
(`radio_monitor_20260915_130259_aa74cde2`), and W2 against R
(`radio_monitor_20260914_161016_6feca2c0`).

The archive folders are named in PC local time (UTC−5) at the end of each
leg: `110651` is 11:06:51 local, or 16:06:51 UTC.

### Per-leg procedure

Each leg ran the same steps:

1. `clear_retained` on the base broker (three topics:
   [legV_clear_retained.txt](legs/legV_clear_retained.txt) and the W, W2 and
   X equivalents).
2. rs115 pre-brackets on both boards.
3. The harness, plus the leg's injector on the base.
4. A radio-quiet check, then rs115 post-brackets.
5. `rs12_leg_report.py` and `frag_gap_report.py`.
6. `radio_park.py` on both boards, last.

[scripts/rs1215.sh](scripts/rs1215.sh) is the session helper used for these
steps. [scripts/deltas.py](scripts/deltas.py) scores the brackets against the
TODO's criteria (see "Counter resets" below).

The post step also tried to pull an RS-13 base capture (`rs13_cap_base`).
This round never ran one, so every `leg*_post_transcript.txt` shows
`PULL FAILED` and "seq-gap loss: no base capture". That is harmless here.

### Injectors

| leg | injector on the base | trigger | started | published to the base broker | base commands on air (`command TX opcode=` in `rx_daemon.log`) |
|---|---|---|---|---|---|
| V | `kf_inject.py 15 20` | harness console tail | 16:01:45 | 20 keyframe requests ([legV_kf_inject.txt](legs/legV_kf_inject.txt)) | 23 × `0x60` REQ_KEYFRAME, 16:01:47.529–16:06:32.583 |
| W | `dual_inject.py 250` | harness console tail | 16:14:19 | em 356, kf 45 ([legW_dual_inject.txt](legs/legW_dual_inject.txt)) | 13 × `0x60` + 14 × `0x63`, 16:14:20.181–16:15:53.601 |
| W2 | `dual_inject.py 250` | [scripts/inject_on_first_frame.sh](scripts/inject_on_first_frame.sh) (reads `rx_smoke`'s own log) | 16:21:59 | em 356, kf 45 ([legW2_inject.txt](legs/legW2_inject.txt)) | 25 × `0x60` + 47 × `0x63`, 16:22:01.184–16:26:16.236 |
| X | `dual_inject.py 120` | [scripts/legX_watch.sh](scripts/legX_watch.sh) (the final RX_SCAN_FAILED) | 16:32:14 | em 171, kf 22 ([legX_dual_inject.txt](legs/legX_dual_inject.txt)) | 36 × `0x63`, 16:32:16.957–16:34:22.679 |

- `dual_inject.py` publishes ENCODE_MODE (`0x63`, mode 0, quality cycled
  78–88) every 0.7 s and REQ_KEYFRAME (`0x60`) every 5 s
  ([dual_inject.py](../../firmware/x8_lora_bootloader_helper/bench_tools/dual_inject.py)).
- Most publishes never go on air. Each new ENCODE_MODE body replaces the one
  still pending (`cmd 0x63 superseded while pending` in `rx_daemon.log`), and
  the daemon spaces commands at least `cmd_stream_min_gap_s` = 1.0 s apart.
- In every leg, the base's `radio_tx_ok` delta equals its count of
  `command TX` lines (23, 27, 72, 36). So the base sent nothing else on air.
- The V/W trigger script is not in `scripts/`. Its output wording ("first
  published frame_id seen at") is in legV_kf_inject.txt and
  legW_dual_inject.txt.

### Scene (leg V only)

`scene_check.sh` grabs frames from the tractor's `/dev/video1` and needs at
least 8 % of pixels changed. The scene was the bench video in a normal
Firefox window with only the player fullscreen. Two checks were recorded,
and both were above the 8 % minimum. The second was taken 47 s before the
leg started.

| check | time | changed > 25 at 1.9 / 5.0 / 9.7 s | mean luma |
|---|---|---|---|
| [try 1](legs/legV_scene_try1_scene.txt) | 15:57:14 | 77.55 / 77.56 / 75.47 % | 82.3 |
| [try 2](legs/legV_scene_try2_scene.txt) | 16:00:08 | 22.63 / 29.49 / 32.23 % | 104.6 |

On leg V the camera ran the tile path: `encode_mode -> 0 (full)` and
`link_budget: -> 203 B/frame (n=1, image_bw250)`
([camera_service.log](../radio_monitor_20261018_110651_d484a92d/camera_service.log)).
All 608 published frames went to `lifetrac/v25/video/tile_delta`. Legs T and
U also published only to `tile_delta` (477 and 537 frames in their
`rx_daemon.log`).

## Results against the PASS criteria

### How each figure is measured

- **Delivery** is the tractor's `radio_rx_ok` divided by the base's
  `radio_tx_ok`, over the leg (`leg*_deltas.txt`). The tractor's
  `fhss_dec_aligned` equals its `radio_rx_ok` on every leg, and its TX log
  shows the same number of decoded commands ("tractor decoded cmds" in
  `leg*_gaps.txt`).
- **Loss** compares fragments sent with fragments decoded. Fragments sent are
  the per-frame TX events in `tx_daemon.log`; fragments decoded are the
  `frag_arrival` lines in the base's `rx_daemon.log`. This is what
  `rs12_leg_report.py` prints (`leg*_report.txt`). The counts were checked
  against the archives: V 949/943, W 1215/1086, W2 1230/1185, X 2010/1880.
- **Gaps** are from `frag_gap_report.py` (`leg*_gaps.txt`).

### Scored legs

| TODO criterion | V (camera) | W2 (synth, W re-fly) | X (post-FAILED) |
|---|---:|---:|---:|
| delivery ≥ 60 % | **22/23 = 95.7 %** | **72/72 = 100 %** | **36/36 = 100 %** |
| tractor `clk_demotion_reset` = 0 | 0 | 0 | 0 |
| tractor `tx_first_anchor` = 1 | 1 | 1 | 1 |
| tractor `fhss_dec_snapped` = 0 | 0 | 0 | 0 |
| tractor `fhss_dec_rej_epoch_drift` = 0 | 0 | 0 | 0 |
| base `tx_stream_streak_max` < 8 | 1 | 1 | 1 |
| base lock-loss gaps > 3 s = 0 | 0 | 0 | 0 |
| max fragment gap ≤ 1 s | 0.8 s | 0.5 s | 0.6 s |
| loss not worse than leg U (10.5 %) | 0.6 % (6/949) | 3.7 % (45/1230) | 6.5 % (130/2010) |
| stop rule (tractor `clk_demotion_reset` > 0 or `tx_first_anchor` > 1) | not tripped | not tripped | not tripped |
| *tractor `clk_demotion_kept`* | *20* | *72* | *0* |
| *frames published* | *608* | *78* | *122* |

Sources:
[legV_deltas.txt](legs/legV_deltas.txt),
[legW2_deltas.txt](legs/legW2_deltas.txt),
[legX_deltas.txt](legs/legX_deltas.txt);
[legV_gaps.txt](legs/legV_gaps.txt),
[legW2_gaps.txt](legs/legW2_gaps.txt),
[legX_gaps.txt](legs/legX_gaps.txt);
[legV_report.txt](legs/legV_report.txt),
[legW2_report.txt](legs/legW2_report.txt),
[legX_report.txt](legs/legX_report.txt).
The published-frame counts are the `published frame_id` lines in each
`rx_daemon.log` (`leg*_p2p4.txt`).

**The leg R loss reference does not exist in usable form.** Leg R's report
printed `-17/1204 = -1.4%`, which
[that RESULTS](../RS_12_15_clock_authority_2026-09-14/RESULTS.md) calls a
bracket artifact. The archive does not rescue it either: the per-frame TX
events in
[R's tx_daemon.log](../radio_monitor_20260914_161016_6feca2c0/tx_daemon.log)
sum to 1 200 fragments, but
[R's rx_daemon.log](../radio_monitor_20260914_161016_6feca2c0/rx_daemon.log)
has 1 235 `frag_arrival` lines, so the two logs do not cover the same window.
The synthetic legs are therefore held to leg U's 10.5 %. For context, leg L
on 09-14 (RS-12.10 firmware, the same setup as R) lost 7.8 %.

### Against the v2 firmware

R and T flew bench `5a160e4a`. U flew `0c1bb0a9`, the build the boards ran
until this round.

| leg | date | L072 | workload | tractor heard / base sent | delivery | loss | max frag gap | gaps > 3 s | published |
|---|---|---|---|---:|---:|---:|---:|---:|---:|
| R | 09-14 | `5a160e4a` | synth 2 fps / 3000 B, `dual_inject 250` | 19/72 | 26 % | none usable | 0.6 s | 0 | 78 |
| T | 09-14 | `5a160e4a` | camera, `kf_inject 15 20` | 2/43 | 4.7 % | 20.9 % | 0.8 s | 0 | 477 |
| U | 09-15 | `0c1bb0a9` | camera, `kf_inject 15 20` | 7/49 | 14 % | 10.5 % | 1.0 s | 0 | 537 |
| **V** | 10-18 | `e083ae46` | camera, `kf_inject 15 20` | **22/23** | **95.7 %** | 0.6 % | 0.8 s | 0 | 608 |
| **W2** | 10-18 | `e083ae46` | synth 2 fps / 3000 B, `dual_inject 250` | **72/72** | **100 %** | 3.7 % | 0.5 s | 0 | 78 |
| **X** | 10-18 | `e083ae46` | synth, silent until FAILED, then `dual_inject 120` | **36/36** | **100 %** | 6.5 % | 0.6 s | 0 | 122 |

Where the R, T and U figures come from:

- **Command counts:** the TODO item, and the reference brackets
  [legR_post_tractor.txt](../RS_12_15_clock_authority_2026-09-14/legs/legR_post_tractor.txt)
  and
  [legR_post_base.txt](../RS_12_15_clock_authority_2026-09-14/legs/legR_post_base.txt),
  [legT_post_tractor.txt](../RS_12_15_clock_authority_2026-09-14/legs/legT_post_tractor.txt)
  and
  [legT_post_base.txt](../RS_12_15_clock_authority_2026-09-14/legs/legT_post_base.txt),
  and
  [legU_gaps.txt](../RS_12_15_clock_authority_2026-09-14/legs/legU_gaps.txt).
- **Loss and gaps:**
  [RS_12_15_clock_authority_2026-09-14/RESULTS.md](../RS_12_15_clock_authority_2026-09-14/RESULTS.md).
  T's loss is 769 sent and 608 decoded; U's is 674 sent and 603 decoded.
- **Published frames:** the `published frame_id` lines in each archive's
  `rx_daemon.log`.

The TODO item quotes the v2 delivery as 26 / 4.7 / 14 %.

The improvement this round shows is in delivery. Loss on this bench varies
this much from run to run (see "Honest reading" point 3 in the 09-14
RESULTS), so V's 0.6 % against U's 10.5 % is not claimed as an effect of the
fix.

## Leg W: the trigger error and the re-fly (W2)

**What went wrong.** W's injector was meant to start on the first published
frame. Its trigger watched the harness console, which prints only the last
two lines of each daemon log about every 5 s
([legW_harness.txt](legs/legW_harness.txt), the `--- RX Log ---` blocks). On
the synthetic feed, each 15-fragment frame logs 15 `frag_arrival` lines and
one `published frame_id` line, so the published line was rarely in the last
two.

- The first frame was published at 16:11:38.405
  ([rx_daemon.log](../radio_monitor_20261018_111558_d484a92d/rx_daemon.log)).
- The first `published frame_id` line to appear in a console tail was the one
  at 16:14:16.522 (legW_harness.txt line 483).
- The trigger fired at 16:14:19 ([legW_dual_inject.txt](legs/legW_dual_inject.txt)),
  2 min 41 s after the first frame.

On leg V the same trigger was only about 4 s late (first frame 16:01:40.780,
first console line 16:01:44.273, trigger 16:01:45), because the camera
publishes one small frame every 0.5 s.

**What W flew instead of the plan:**

1. The command plane was silent from the start.
2. The tractor's scan machine went through its four 30 s attempts and
   reached FAILED. Its faults, `code=0x0D` with sub `0x00`, `0x01`, `0x02`
   and `0x83`, are 29.9–30.0 s apart, at tractor clock 09:02:10.455 to
   09:03:40.316
   ([tx_daemon.log](../radio_monitor_20261018_111558_d484a92d/tx_daemon.log)).
   The final fault was at about 16:12:49 PC time.
3. The injector started about 90 s after that, and the first command went
   on air at 16:14:20.181.
4. The harness ended the leg at 16:15:58, while the 250 s injector was still
   running. So commands went on air for only 93 s (16:14:20.181 to
   16:15:53.601).

Every one of those 27 commands reached the tractor (27/27,
[legW_deltas.txt](legs/legW_deltas.txt)), all of them while it was in scan
FAILED. This was an unplanned second sample of what X measures.

**W as flown against the criteria.** W passes every criterion except loss:

- delivery 100 %
- tractor `clk_demotion_reset` 0, `tx_first_anchor` 1, `fhss_dec_snapped` 0,
  `fhss_dec_rej_epoch_drift` 0
- base `tx_stream_streak_max` 1
- 0 gaps over 3 s, max fragment gap 0.6 s
  ([legW_gaps.txt](legs/legW_gaps.txt))

Its loss, 129/1215 = **10.6 %** ([legW_report.txt](legs/legW_report.txt)), is
**0.1 point above leg U's 10.5 %**. This file does not attribute that loss.
W is not the planned shape, so it is reported here and not scored.

**The re-fly.** W2 used the same parameters with a new trigger:
[scripts/inject_on_first_frame.sh](scripts/inject_on_first_frame.sh) waits
for the harness's MONITORING line and then polls `docker logs rx_smoke` for
the first `published frame_id`. It fired at 16:21:59, 1.5 s after the first
frame (16:21:57.488). The first command went on air at 16:22:01.184.

W2 also has one scan fault: `sub=0x00` at tractor clock 09:12:47.763
([tx_daemon.log](../radio_monitor_20261018_112639_d484a92d/tx_daemon.log)),
about 16:21:57 PC time. The tractor's first 30 s scan window had expired
about 4 s before the first command. There were no further faults, and
`clk_demotion_kept` was 72, one per command.

## Leg X: the scan-FAILED timeline

The plan: keyframe requests off (`kf_request_disable=1`), no injector, until
the tractor sends its final RX_SCAN_FAILED; then the injector for 120 s. The
watcher [scripts/legX_watch.sh](scripts/legX_watch.sh) polled the tractor's
`tx_smoke` log every 2 s for `code=0x0D` and started `dual_inject.py 120` on
the base when it saw a fault with bit 7 set and attempt nibble 3.

| event | tractor clock (`tx_daemon.log`) | PC / base clock |
|---|---|---|
| daemons up (tractor `TX worker ready` / base `RX worker ready`) | 09:21:03.647 | 16:30:12.776 |
| watcher sees MONITORING | — | 16:30:17 |
| `code=0x0D sub=0x00` (attempt 0) | 09:21:33.635 | seen 16:30:43 |
| first frame published at the base | — | 16:31:01.516 |
| `sub=0x01` (attempt 1) | 09:22:03.546 | seen 16:31:14 |
| `sub=0x02` (attempt 2) | 09:22:33.518 | seen 16:31:43 |
| `sub=0x83` (final, attempt 3) | 09:23:03.425 | seen 16:32:14, 117 s after MONITORING |
| `dual_inject.py 120` started | — | 16:32:14 |
| first base command on air (`0x63`) | — | 16:32:16.957 |
| last base command on air | — | 16:34:22.679 |
| injector done (`DUAL_DONE em=171 kf=22 dur=120`) | — | 16:34:16 |
| harness exit | — | 16:38:31 |

Sources: [legX_faults.txt](legs/legX_faults.txt),
[legX_dual_inject.txt](legs/legX_dual_inject.txt),
[tx_daemon.log](../radio_monitor_20261018_113831_d484a92d/tx_daemon.log),
[rx_daemon.log](../radio_monitor_20261018_113831_d484a92d/rx_daemon.log),
[legX_harness.txt](legs/legX_harness.txt).

- **The faults match the firmware.** They are 29.9–30.0 s apart on the
  tractor clock, and the first came 30.0 s after its daemon started. That
  matches `SX1276_RX_SCAN_REDESIGN_MS` (30 000 ms,
  [sx1276_rx_scan_policy.h](../../firmware/murata_l072/include/sx1276_rx_scan_policy.h))
  and `SX1276_RX_SCAN_MAX_RETRIES` 3, which gives four attempts
  ([sx1276_rx_scan_fail.h](../../firmware/murata_l072/include/sx1276_rx_scan_fail.h)).
- **The base was silent until 16:32:16.957.** Its `radio_tx_ok` delta (36)
  equals its 36 `command TX` lines, all at or after that time, so no base
  traffic could LOCK the tractor before FAILED. The TODO's "if base
  background traffic keeps LOCKing it" case did not arise.
- **The silence was about 2 min, not the planned ≥ 150 s.** The final fault
  came at 117 s after MONITORING and the injector started on it. The
  condition the leg exists for still held: the tractor was in scan FAILED
  before any command went on air.
- **All 36 commands reached the tractor while it was in FAILED** (36/36). In
  the firmware's scan policy, FAILED absorbs every event (case (16) in
  [rx_scan_policy.c](../../firmware/murata_l072/bench/host_proto/rx_scan_policy.c)).
  The tractor logged no further fault, and `clk_demotion_kept` stayed 0.
- **Only opcode `0x63` flew.** With `kf_request_disable=1` the daemon drops
  the injector's 22 keyframe requests (see Observations).
- **The link held for the whole 480 s leg:** loss 6.5 %, max fragment gap
  0.6 s, 0 gaps over 3 s over a 451 s fragment span.

## Observations

1. **Counter resets and how they are scored.**
   - **Tractor.** The harness resets the tractor's L072 at the start of each
     leg (OpenOCD SWD, BOOT0 low plus NRST, fired in the background:
     [run_live_radio_monitor.ps1](../../firmware/x8_lora_bootloader_helper/run_live_radio_monitor.ps1),
     steps 4 and 5 of the script). When the reset lands, the RAM counters
     restart at 0 and post − pre is meaningless. On W, for example, the tractor's
     `radio_tx_ok` went 955 → 1222, which reads as 267, but 1222 is the
     leg's own count.
   - **How deltas.py handles it.** Counters only grow, so
     [deltas.py](scripts/deltas.py) treats any counter that decreased as a
     reset and then uses the post value. On W, W2 and X it caught the reset
     through at least two counters. For W2 these were `radio_crc_err` 1 → 0
     and `tx_done_to_rearm_max_us`; in the same leg `host_parse_ok` went
     1368 → 1395 across 1239 TXs. The rule would miss a reset if every
     counter happened to end above its pre-leg value.
   - **Base.** The base's counters were never reset. They accumulated
     through all four legs (`radio_tx_ok` 0 → 23 → 50 → 122 → 158), so
     post − pre is the right delta there.

2. **The base's gpio163 NRST did not reset its L072.** The harness pulses
   gpio163 on the base before each leg. It sends that command over adb and
   discards the output, and its comment says "gpio163 NRST works". Yet the
   base's counters carried over from each leg's post bracket to the next
   leg's pre bracket (`legV_post_base.txt` → `legW_pre_base.txt` and so on).
   So on this boot the pulse did not reset the base L072. This was not
   investigated further.

3. **The tractor's L072 was not reset at the start of leg V either.** The
   harness logs only `swd_reset_spawned`, not whether the reset landed.
   - **V.** From pre to post bracket, `host_errors` stayed at 17152, and
     `host_irq_idle` rose by exactly as much as `host_parse_ok`
     (17155 → 18959 against 2 → 1806;
     [legV_pre_tractor.txt](legs/legV_pre_tractor.txt),
     [legV_post_tractor.txt](legs/legV_post_tractor.txt)). After a reset,
     both counters would have restarted.
   - **Scoring is unaffected:** every radio counter was 0 at V's pre bracket.
   - **Same pattern on 09-14/15.** The first leg after each flash there shows
     the same carry-over: `host_errors` 97765 / 118306 / 46836 unchanged
     across legs R, T and U (their `pre_tractor` and `post_tractor`
     files). So it looks tied to the first leg after a flash reboot, not to
     this firmware.
   - **The scan machine still started fresh on V:** it LOCKed 20 times,
     which FAILED would not allow. The likely reason is the daemon's profile
     setup at start ("Initializing regulatory profile 1 on co-processor" in
     `tx_daemon.log`): activating profile 1 calls `sx1276_rx_scan_reset()`
     ([host_cfg_profile.c](../../firmware/murata_l072/host/host_cfg_profile.c)).
     A comment in the same file already notes that the harness's inter-run
     NRST is fire-and-forget and can fail to land. Not checked further.

4. **The harness console tail cannot be used as a trigger** (see "Leg W"). It
   shows two lines per daemon every ~5 s. On a feed that logs 15 fragment
   lines per frame, the line being watched for can be absent for minutes.
   `inject_on_first_frame.sh` reads the daemon's own log instead. The bench
   runbook's step "On the first `published frame_id` line start the
   injector"
   ([BENCH_RUNBOOK.md](../../firmware/x8_lora_bootloader_helper/bench_tools/BENCH_RUNBOOK.md))
   needs that kind of trigger on the synthetic feed.

5. **The final RX_SCAN_FAILED sub byte is `0x83`, not `0x84`.** Bits 3..0
   are the index of the current attempt, counted from 0. With
   `SX1276_RX_SCAN_MAX_RETRIES` 3 there are four attempts, so the final one
   is attempt 3, with bit 7 (final) set
   ([sx1276_rx_scan_fail.h](../../firmware/murata_l072/include/sx1276_rx_scan_fail.h),
   [host_types.h](../../firmware/murata_l072/include/host_types.h)).
   `legX_watch.sh` keys on bit 7 plus nibble 3, as seen in leg W.
   - The warm bit (6) was clear in every fault this round: the tractor had
     not LOCKed since its leg-start reset.
   - The `hw_suspect` (5) and `crc_seen` (4) bits were clear too.

6. **Fault code `0x0D` has two meanings.**
   [host_types.h](../../firmware/murata_l072/include/host_types.h) defines
   both `HOST_FAULT_CODE_TX_FIFO_RB_MISMATCH` and
   `HOST_FAULT_CODE_RX_SCAN_FAILED` as `0x0DU`. The TX daemon logs either one
   as `FAULT during pipelined TX: code=0x0D`. The faults in W, W2 and X are
   scan faults, for two reasons:
   - The tractor's `tx_fifo_rb_bad` was 0 in every post bracket, and those
     brackets cover the leg after the reset. `sx1276_tx.c` increments it
     whenever it emits the FIFO fault.
   - The subs and the 30 s spacing match the scan encoding.

   A reader of a daemon log cannot tell the two apart from the code alone.

7. **Parks needed retries.** `radio_park.py` was run after the post-brackets
   and reports. After every leg, at least one board needed 2–3 attempts,
   with a 60 s wait between attempts:

   | leg | base | tractor | file |
   |---|---|---|---|
   | V | TRANSIENT, TRANSIENT, OK | FAIL, FAIL, OK | [legV_park.txt](legs/legV_park.txt) |
   | W | TRANSIENT, OK | OK | [legW_park.txt](legs/legW_park.txt) |
   | W2 | FAIL, TRANSIENT, OK | TRANSIENT, TRANSIENT, OK | [legW2_park.txt](legs/legW2_park.txt) |
   | X | TRANSIENT, TRANSIENT, OK | OK | [legX_park.txt](legs/legX_park.txt) |

   - PARK_TRANSIENT means `0x80` was read back but `0x85` came back after the
     1.5 s settle. PARK_FAIL means `0x85` both times.
   - The tractor parked on the first attempt only after W and X. Those are
     the two legs in which its scan machine had already reached FAILED.
   - This matches the runbook: a park holds once the scan machine has
     exhausted into FAILED, up to ~2 min after the daemons stop
     ([BENCH_RUNBOOK.md](../../firmware/x8_lora_bootloader_helper/bench_tools/BENCH_RUNBOOK.md)).

8. **`kf_request_disable=1` drops injected keyframe requests too.** With
   `LIFETRAC_KF_REQUEST_DISABLE=1`, the daemon's handler for
   `lifetrac/v25/cmd/req_keyframe` (`_on_req_keyframe_msg` in
   [image_rx_daemon.py](../../base_station/image_rx_daemon.py)) returns
   before queueing a command. The gate sits at the subscription on purpose,
   so every request source passes through it. So the 22 REQ_KEYFRAME
   publishes from `dual_inject.py` in X never became commands, and all 36
   sends were `0x63` ENCODE_MODE. For a delivery count
   any frame on air serves, but X covers one opcode only.

9. **`clk_demotion_kept` is 0 in FAILED, by design.** FAILED never LOCKs, so
   a received command leaves no demotion to keep. The counter was 0 on W and
   X (all commands after the final FAIL), against 20 on V and 72 on W2.
   - On V, 20 kept demotions for 22 commands fits one LOCK episode per
     command, minus two pairs that arrived less than 2 s apart: offsets
     15.0 and 15.8 s, and 134.4 and 135.5 s, in legV_gaps.txt. Each pair
     falls inside one LOCK, since `SX1276_RX_SCAN_LOCK_LOSS_MS` is 2000.
   - The counter therefore measures LOCK episodes, not deliveries. In FAILED
     the fix delivers through the tractor's own clock with no LOCK at all.

10. **There were no LOCKED_OUT refusals.** The tractor's
    `fhss_dec_rej_locked_out` was 0 on every leg; leg U had 1. This is not a
    criterion.
    - On the base, `fhss_dec_snapped` and `clk_demotion_reset` rose by 1 on
      every leg. That is the same one-per-leg pattern the base showed in
      legs R, T and U (their `leg*_report.txt` RS-12.15 lines).
    - `legW_report.txt` and `legX_report.txt` are cut at 1 024 bytes, in the
      middle of that RS-12.15 line. The base's full counters are in
      `leg*_post_base.txt` and `leg*_deltas.txt`.

## Reading the tractor's timestamps

The tractor's wall clock read 2026-09-29 throughout. At the flash it read
`Tue Sep 29 08:42:19 UTC 2026` while the PC read 2026-10-18T15:50:00Z
(flash_tractor.txt). It also stepped backwards across the flash reboot:
08:43:11 at `T3`, just before the reboot, and 08:42:41 at "up 0 min" after it
(tractor_after_flash.txt).

Within this round's single boot it ran a steady 7 h 09 min 09 s (± about 1 s)
behind PC time. Two independent measurements agree:

- The X watcher logged each fault 7 h 09 min 09.4–10.6 s after its tractor
  stamp (legX_faults.txt; it polls every 2 s).
- On W, W2 and X, the base's `RX worker ready` minus the tractor's
  `TX worker ready` is 7 h 09 min 09.1–09.2 s.

The PC times given above for tractor-side events are converted with this
offset. The smallest margin any conclusion here relies on is about 4 s (W2's
first fault against its first command), against about 1 s of uncertainty in
the offset.

## What this does not show

- **One bench and one session.** The two boards were close together, over
  the air. The base logged RSSI between −67 and −59 dBm and SNR 7–13 dB on
  every leg (`rx_rf:` lines in each `rx_daemon.log`). That is a strong link,
  not a range edge and not a field link.
- **One run per leg shape.** Loss on this bench varies from run to run, so
  the loss column is a no-regression check, not a measured improvement.
- **The 1 fps limit was not tested.** The TODO notes that authority needs a
  sustained stream: at 1 fps, or about 3.5 s after a stall of 1 s or more,
  the tractor behaves as before the fix. Every leg here streamed at 2 fps.
- **Only the bench image flew.** The production build `148de24e` (same
  sources, without the diag register-write flag) was not flown.
- **The camera leg used the tile path, not VECTOR.** It published only
  `tile_delta`, at a 203 B link budget. Legs T and U also published only
  `tile_delta`.
- **Open design questions remain.** The TODO lists them for the design
  owner: the decay window, a lead tolerance, whether the receiver should
  self-follow in FAILED, and the half-duplex collisions. Nothing here
  answers them. The behaviour in FAILED that this round measured is the
  current design, which follows in FAILED.

## Board state at the end

At 16:41:48–16:41:56 a check-only guard found the following
([session_end_radio_check.txt](legs/session_end_radio_check.txt)):

- **Tractor:** up 50 min, `lifetrac-camera.service` failed (it had been
  stopped), containers `bench_mqtt x8-devel x8-provisioning x8-webapp`,
  nothing holding `/dev/ttymxc3`. Radio `0x80` (SLEEP).
- **Base:** up 47 min, container `design-controller-mosquitto-1`, nothing
  holding `/dev/ttymxc3`. Radio `0x80` (SLEEP).

Both L072s run bench `e083ae46a64686031b39866eca42e094`. Nothing was flashed
after 15:53.

## Evidence index

`firmware_flash/`

- `firmware_bench_diag_e083ae46.bin`: the flown image (md5 `e083ae46…`,
  24 972 B).
- `preflight_tractor.txt`, `preflight_base.txt`: staging and PREFLIGHT-OK.
- `flash_tractor.txt`, `flash_base.txt`: the full flash pipeline (erase,
  write, `Verify OK`, `flash_rc=0`, `PIPELINE-EXIT=0`).
- `tractor_after_flash.txt`, `base_after_flash.txt`: the boards after the
  flash reboot, and the stamped log re-read.

`legs/`

- **Session files:**
  - `staging_tractor.txt`, `staging_base.txt`: `/tmp/lifetrac_strict`
    re-staged, with md5s.
  - `session_health_tractor.txt`, `session_health_base.txt`: rs116 health
    after the flash.
  - `session_end_radio_check.txt`: radios at the end.
- **Per leg** (`legV_`, `legW_`, `legW2_`, `legX_`):
  - `clear_retained`: the base broker cleared before the leg.
  - `pre_base`, `pre_tractor`, `post_base`, `post_tractor`: the rs115
    brackets.
  - `deltas`: `deltas.py` scoring against the TODO criteria.
  - `report`: `rs12_leg_report.py` (W and X cut at 1 024 B).
  - `gaps`: `frag_gap_report.py`.
  - `p2p4`: archive greps (published frames, TX trains, drops, tractor
    authority).
  - `park`: `radio_park.py` attempts.
  - `post_transcript`: the post step.
  - `harness` and `harness_full`: the harness console; the two copies are
    byte-identical.
- **Leg-specific files:**
  - Injector logs: `legV_kf_inject.txt`, `legW_dual_inject.txt`,
    `legW2_inject.txt`, `legX_dual_inject.txt`.
  - `legX_faults.txt`: the X watcher's fault timeline.
  - Leg V: `legV_scene_try1_scene.txt` and `legV_scene_try2_scene.txt` (scene
    checks), and `legV_tractor_log.txt` (no `vector_stats` lines: the tile
    path ran).

`scripts/`

- `rs1215.sh`: the session helper (health, brackets, clear, park, state).
- `deltas.py`: reset-aware bracket scoring.
- `inject_on_first_frame.sh`: the W2 trigger, which reads `rx_smoke`'s log.
- `legX_watch.sh`: the X fault watcher and injector start.

**Harness archives** (in `bench-evidence/`, named by local end time):

- [radio_monitor_20261018_110651_d484a92d](../radio_monitor_20261018_110651_d484a92d/) (V)
- [radio_monitor_20261018_111558_d484a92d](../radio_monitor_20261018_111558_d484a92d/) (W)
- [radio_monitor_20261018_112639_d484a92d](../radio_monitor_20261018_112639_d484a92d/) (W2)
- [radio_monitor_20261018_113831_d484a92d](../radio_monitor_20261018_113831_d484a92d/) (X)

Each holds `params.txt`, `tx_daemon.log` and `rx_daemon.log`, plus either
`camera_service.log` (V) or `synth_pub.log`.

**References:**

- [RS_12_15_clock_authority_2026-09-14/RESULTS.md](../RS_12_15_clock_authority_2026-09-14/RESULTS.md)
  and its `legs/`: legs R, T and U.
- [TODO.md](../../TODO.md): the RS-12.15 reverse-delivery item and its PASS
  criteria.
- [bench_session_2026-10-14](../bench_session_2026-10-14/): the image build
  and the 2026-10-18 power cycle.
