# Bench session 2026-10-25 — RS-12.21 image flashed, first legs R1 and R2

**Status: flashed, flown and scored 2026-10-25. GO given 2026-10-25: "proceed
with any merges and you have a GO for any radio testing" (the operator; time
in the session console, not archived). That covered the merges, the flash of
both L072s and the legs. The plan followed is
[`FHSS_SCAN_FAILED_OPTIONS.md`](../../firmware/murata_l072/FHSS_SCAN_FAILED_OPTIONS.md)
section 6.1 (steps 1-10, then R1 and R2), written before the flash. This
session has no physical access to the bench and moved nothing; no RF-setup
statement was asked for.**

**Result.** Both L072s now run the RS-12.21 bench image `0daae279` (scan
option (b) + L + d2, the firmware park command, the 0x0D split; DECISIONS.md
D-FHSS1): `Verify OK` and `flash_rc=0` on both
(`firmware_flash/flash_{tractor,base}_console.txt`). Every post-flash check of
section 6.1 passed, and every item of the R1 and R2 "Confirm" lists that
applied passed (tables below); one R1 item, a base 0x0D before the lock,
was not exercised because the base locked in its first attempt. R1: the
base acquired 11.0 s after its profile-1 activation, 599 of 609 fragments,
PROBE 204/204, and the leg-end firmware park still read `0x80` more than
2 min later with profile 1 active (the test of the park gate, T3's park
clause). R2: the tractor's scan faults came as 0x0D
`name=RX_SCAN_FAILED` 0x00, 0x01, 0x02, 0x83, 30 s apart; the injector fired
on the 0x83 line only; the tractor heard 93 of the base's 95 commands in scan
FAILED. Two procedure notes for the flash (Flash record): on the tractor the
console misses `PIPELINE-EXIT`, and the ssh session of the base's
`flash_l072.sh` hung after the reboot until it was stopped.

## What was checked before the flash (no radio)

| item | result |
|---|---|
| merges | #168 (RS-12.21 option (b) package, D-FHSS1) and #164 (bench PC requirements, Pillow 13.0.0) merged on the operator's word: main `d597b094` (`git log`) |
| flight worktree | `LifeTrac-bench-main` on branch `bench-session-2026-10-25` = main `d597b094`; `bench.env` kept (git-ignored); `EVIDENCE_DIR` = this folder's `legs/` |
| `mingw32-make check` | exit 0, 31 PASS lines, `check-radio-park`, `check-rx-scan-failed-follower` and `check-rx-reverse-delivery` among them (session console, not archived) |
| clean build | bench `build/firmware_bench_diag.bin` `0daae27978089eb97ab35c18718b62b0`, production `build/firmware.bin` `2c5e055abff95126309a62409fafc654`, 25 324 B each: the md5s section 6.1 step 2 wants (session console; the flashed copy is `firmware_flash/firmware_bench_diag_0daae279.bin`, md5 re-checked for this record) |
| harness offline tests | `test_run_live_radio_monitor.ps1`: 197 passed, 0 failed (session console) |
| radios | `power_up_guard.sh` 14:07:45Z tractor / 14:07:55Z base: both `RADIO_STATE 0x80 SLEEP`, no `/dev/ttymxc3` holders; both boards up 6 days 22 h (`legs/power_up_guard_2026-10-25.log`) |
| staging | `stage_boards.sh` 14:08:01-14:08:43Z from `d597b094`, every file `ok` (`legs/staging_{tractor,base}.txt`, first block) |
| `radio_park.py` on `e083ae46` (step 4) | `"cap_bitmap": "0x0000007F"`, `"park_cmd": "unknown_type"`, `"path": "register"`, `PARK_OK`: the fallback, as the plan expects (session console) |
| preflight (step 5) | `PREFLIGHT-OK image=/tmp/lifetrac_p0c/firmware_bench_diag.bin size=25324 md5=0daae279...` on both, and again inside each `--go` run (`firmware_flash/flash_{tractor,base}_console.txt`) |

## Flash record (section 6.1 step 6)

| board | PC time | image line | ROM / erase | write, verify | pipeline | `PIPELINE-EXIT` | `flash exit status` (printed by `flash_l072.sh`: the remote command's status) |
|---|---|---|---|---|---|---|---|
| tractor `2E2C1209DABC240B` (adb) | 14:09:40-14:10:36Z (session console) | `25324 B  md5 0daae279...`, `RS-12.21 bench build (...)` | `Bootloader version: 31`; mass erase NACK, page-by-page erase of 198 pages | 19.1 s, `Verify OK`, `flasher exit code = 0` | `flash_rc=0`, `DONE: prep=0 flash=0 revive=0; REBOOTING` | **not in the console**: adb dropped at the reboot | 0 |
| base `2D0A1209DABC240B` (ssh) | T-1 14:11:28 to T3 14:12:22 base clock (NTP) | the same | the same, 198 pages | 19.3 s, `Verify OK`, `flasher exit code = 0` | `flash_rc=0`, `DONE: prep=0 flash=0 revive=0; REBOOTING` | `PIPELINE-EXIT=0 uptime_s=598704.04` | 255 (the ssh session, stopped at about 14:21Z after it hung on the reboot; the script names that case as expected) |

Sources: `firmware_flash/flash_tractor_console.txt` (lines 112-178; its
stamps are the tractor clock, which has no time source and reads
2026-10-06) and `firmware_flash/flash_base_console.txt` (lines 107-175). The
image flashed is kept as `firmware_flash/firmware_bench_diag_0daae279.bin`
(25 324 B, md5 `0daae27978089eb97ab35c18718b62b0`). From here on both boards
run `0daae279`; the previous image, `e083ae46`, is the rollback image.

Two notes for the procedure:

- **`PIPELINE-EXIT` is a console line, not a log line.** `run_flash_bench.sh`
  prints it after the pipeline returns (`bench_tools/run_flash_bench.sh:77`);
  it is not in `/home/fio/pipeline_stamped.log`. With `REVIVE_MODE=reboot`
  the board reboots right after the `DONE` line, so whether `PIPELINE-EXIT`
  reaches the PC depends on the connection outliving the reboot: on the
  tractor adb dropped first, on the base it arrived. The success record is
  `Verify OK`, `flash_rc=0` and `DONE: prep=0 flash=0 revive=0` (the
  pipeline's own lines, stamped into `pipeline_stamped.log`). FLASH_RUNBOOK
  and the memo said `PIPELINE-EXIT=0` is in that log; both now say this.
- **The ssh session of the base's `flash_l072.sh` hung** after the board
  rebooted. It hung until it was stopped at about 14:21Z, when the board was
  already back (session console). That gave status 255 (`Connection reset
  by peer`, `client_loop: send disconnect`), after which the script
  finished: its captured console ends with `flash exit status: 255` and the
  script's "Next (FLASH_RUNBOOK section 4)" block
  (`firmware_flash/flash_base_console.txt` lines 176-185). Once the board
  answers on a new connection, stop a hung ssh session; the flash is judged
  by the lines above, not by that status.

## After the flash (section 6.1 steps 7-10; nothing transmits)

| step | check | result | evidence |
|---|---|---|---|
| 7 | tractor camera unit and container stopped, `fuser /dev/ttymxc3` empty on both, `/tmp/lifetrac_strict` re-staged | the `tractor-camera` container was running again after the reboot and was stopped; `fuser` empty on both (session console); re-staged 14:22:09-14:22:45Z, every file `ok` | `legs/staging_{tractor,base}.txt`, second block |
| 8 | `rs116_health_probe.py`: `STATS-OK`, the three `=YES` lines, `tx_fifo_rb_bad` 0, `radio_state=4` | PASS on both: `STATS-OK (attempt 1)`, `RS115-INSTRUMENTED-FIRMWARE=YES`, `RS12-URC-COUNTERS=YES`, `RS12-10-COUNTERS=YES`, `tx_fifo_rb_bad=0`, `radio_state=4`, every radio counter 0 | `firmware_flash/postflash_health_{tractor,base}.txt` |
| 9 | `radio_park.py`: `PARK_OK`, `cap_bitmap` `0x000000FF`, `path` firmware, `PARKED`, `persistent` true, both readbacks `0x80` | PASS on both, `park_tries` 1, `opmode_readback` and `opmode_after_settle` `0x80`, 14:23:09Z tractor / 14:23:14Z base (times: session console). The new image is on both chips | `firmware_flash/postflash_park_{tractor,base}.txt` |
| 10 | the park holds: a read-only reading at least 2 min later reads `0x80` | PASS: `power_up_guard.sh --check-only` 14:26:02Z tractor, 14:26:06Z base, both `RADIO_STATE 0x80 SLEEP`, no holders; 2 min 52-53 s after the parks. Both boards were on their boot profile here (no scan), so this proves the command and the image; R1's leg-end park is the test against the scan | `legs/power_up_guard_2026-10-25.log` |

## R1 — cold-start acquisition and the fault codes (T3's cold start, T5)

`leg_prep.sh 2c_r1 image_bw250 --reactive-fire 1`: `mono_g4`, profile 1,
2 fps, 300 s, `-ReactiveFire 1 -ProbeEcho 0 -TxBatch 0 -LogFragArrivals 1
-StopDrainS 4` (`legs/leg2c_r1_prep.txt`, `legs/leg2c_r1_params.txt`). Prep
14:36:33Z, tractor L072 reset tag `OCD_LEG=20261025T143725Z`, archive
`radio_monitor_20261025_094252_d597b094`.

| Confirm item (section 6.1, R1) | result | evidence |
|---|---|---|
| base `rx_daemon.log` has `L072 VER warm-up ok (attempt N) cap=0x000000FF` | **PASS**: `L072 VER warm-up ok (attempt 1) cap=0x000000FF` at 14:37:32.681 base clock | archive `rx_daemon.log:11` |
| `acquisition:` within 120 s and `confirmed acquisition: ACQUIRED`, image read from the capability line (no "image not identified" note) | **PASS**: first decoded frame 11.0 s after the base's profile-1 activation; `ACQUIRED` (base Δ`clk_demotion_reset` 1; 599 of the tractor's 609 fragments, 98.4 %); no "image not identified" line | `legs/leg2c_r1_report.txt` |
| every `code=0x0D` line `name=RX_SCAN_FAILED` with its attempt fields, none `fw=old`; no `code=0x0F`; Δ`tx_fifo_rb_bad` 0 on both | **PASS**: no 0x0D and no 0x0F line in either daemon log during the leg (the tractor logged only its boot faults 0x09 `HOST_RX_SEEN` and 0x0C `HOST_DIAG_MARK`, `tx_daemon.log:13`, `:15`; the base logged no FAULT line); no `fw=old`; `tx_fifo_rb_bad` 0 pre and post on both | archive `tx_daemon.log`, `rx_daemon.log`; `legs/leg2c_r1_{pre,post}_{base,tractor}.txt` |
| a base 0x0D, if any, comes before its first lock, about 30 s apart; finals until the lock | **not exercised**: the base locked inside its first 30 s attempt (first frame at 11.0 s) and sent no 0x0D | archive `rx_daemon.log` |
| `leg<X>_deltas.txt`: no `!! BASE NEVER LOCKED`, base Δ`clk_demotion_reset` ≥ 1, the RS-12.15 criteria as on 2026-10-24, probe delivery in line with that round (`mono_g4` 542/543) | **PASS**: no `!!` line; base Δ`clk_demotion_reset` 1; all six RS-12.15 bracket criteria PASS, `STOP RULE: not tripped`; PROBE 204/204 = 100.0 %, brackets 204/204 AGREE | `legs/leg2c_r1_deltas.txt`, `legs/leg2c_r1_probe_score.txt` |
| `leg<X>_park.txt`: `PARK_OK` `"path": "firmware"` on both at the first attempt; `--check-only` at least 2 min later reads `0x80` on both (T3's park clause) | **PASS**: `attempt 1: PARK_OK`, `park_tries` 1, `"path": "firmware"`, readbacks `0x80` on both (park step 14:45:18Z, post step done 14:45:27Z); check-only 14:47:38Z tractor, 14:47:43Z base: `0x80 SLEEP`, 2 min 11-16 s after the post step ended, profile 1 still active (no reset or activation since the leg) | `legs/leg2c_r1_park.txt`, `legs/leg2c_r1_post_transcript.txt`, `legs/power_up_guard_2026-10-25.log` |
| STOP: `!! ACQUISITION FAILED` or `!! BASE NEVER LOCKED` | **not tripped** | `legs/leg2c_r1_report.txt`, `legs/leg2c_r1_deltas.txt` |

Also on R1:

- **Frames:** 609 sent, all K = 1, 0 ABORTED, `drop_full=0 drop_stale=0`; 599
  published; the first frame heard is the tractor's #11 (the 10 before it are
  the acquisition head); seq-gap loss after it 0/599; whole-log loss 10/609 =
  1.6 % (all 10 in the head); no gap over 3 s (max 0.7 s); FIRMWARE DROP
  599 − 599 = 0 (`legs/leg2c_r1_p2p4.txt`, `legs/leg2c_r1_report.txt`,
  `legs/leg2c_r1_gaps.txt`).
- **Tractor scan:** it heard the base's first PROBE 11.9 s after its own
  profile activation (`tx_daemon.log:45`, `:99`), so it too locked inside its
  first attempt. Tractor Δ: `clk_demotion_reset` 0, `clk_demotion_kept` 1,
  `tx_first_anchor` 1, `fhss_dec_snapped` 0, `fhss_dec_rej_epoch_drift` 0,
  `tx_stream_streak_max` 609; base `tx_stream_streak_max` 1
  (`legs/leg2c_r1_deltas.txt`).
- **Level:** base median of window medians −64 dBm, SNR 10 dB (window
  minimums −74 dBm, 5 dB) (`legs/leg2c_r1_rf.txt`): the 2026-10-24 FHSS
  baseline.
- **Kit checks:** `LOG_WINDOW after_stop`, `TRACTOR_L072_RESET landed` (own
  `OCD_LEG` tag, banner, PF_4 low then high, `OCD_EXIT=0`), deploy md5 52/52
  on both plus the 2 reset files, both clock offsets numeric (tractor
  −1667466.170 s, base −0.040 s), every base command 0x69 (204 `command TX
  opcode=0x69 ... OK (on air)` lines in archive `rx_daemon.log`), every
  scorer exit 0 (`legs/leg2c_r1_harness.txt` lines 13-14 and 3777-3786,
  `legs/leg2c_r1_tractor_ocd_reset.log`, `legs/leg2c_r1_params.txt`,
  `legs/leg2c_r1_post_transcript.txt`).
- **First leg after a boot:** the flash rebooted both X8s at about 14:10Z and
  14:12Z, so R1 was the first leg after a boot. Before the bench kit of
  2026-10-19 the tractor's L072 reset missed on the first leg after each
  flash reboot (legs V; O, Q, R, S, T, U: DC/TODO.md, the tractor L072
  reset item). R1 is the first such leg since that kit (`ocd_reset.sh`), and
  the reset landed. One case.

## R2 — the injector trigger and the tractor in FAILED (T4's first half)

`leg_prep.sh 2c_r2 image_bw250 --duration 480`, `-ReactiveFire 0`, with
`-InjectCmd "dual_inject.py 120" -InjectOn fault:0x0D:final` added to the
harness line it printed (`legs/leg2c_r2_prep.txt`; `inject_cmd=`,
`inject_on=` in `legs/leg2c_r2_params.txt`). `-KfRequestDisable 1`. Prep
14:48:01Z, reset tag `OCD_LEG=20261025T144859Z`, archive
`radio_monitor_20261025_095748_d597b094`. The 2026-10-18 leg-X shape.

| Confirm item (section 6.1, R2) | result | evidence |
|---|---|---|
| the tractor's 0x0D `name=RX_SCAN_FAILED` lines for attempts 0, 1, 2, then attempt 3 final, about 30 s apart (leg X: 0x00 0x01 0x02 0x83) | **PASS**: sub 0x00, 0x01, 0x02, 0x83 (`attempt=0..3`, `final=1` on the last only), L072 uptime 33.6, 63.6, 93.6, 123.7 s; the last 119.8 s after the tractor's profile activation | archive `tx_daemon.log:247`, `:491`, `:741`, `:991` (activation `:51`) |
| `INJECT ... trigger: fault:0x0D:final` on that attempt-3 line and on no earlier one | **PASS**: `TRIGGER fault:0x0D:final (113 s after MONITORING)` at 14:51:07 on the sub 0x83 line. The three earlier lines, and the legacy `FAULT during pipelined TX` twins of 0x01, 0x02 and 0x83, did not fire it | `legs/leg2c_r2_inject_trigger.txt`, `legs/leg2c_r2_harness.txt:156`; `inject_trigger=14:51:07 fault:code=0x0D,sub=0x83` in `legs/leg2c_r2_params.txt` |
| the base's commands heard with the tractor in FAILED (leg X: 36/36) | **PASS** by the ≥ 60 % criterion: 93/95 = 97.9 %. All 95 went on air after the final fault (first base command TX 14:51:09.190 base clock, `rx_daemon.log:941`; the final at 07:39:59.447 tractor clock, about 14:51:05.6Z by the params offset). Leg X heard all 36; the 2 misses here are not analysed | `legs/leg2c_r2_deltas.txt`, `legs/leg2c_r2_gaps.txt` |
| the RS-12.15 bracket criteria as on 2026-10-18 | **PASS**: delivery 93/95 ≥ 60 %; tractor `clk_demotion_reset` 0, `tx_first_anchor` 1 (post 1), `fhss_dec_snapped` 0, `fhss_dec_rej_epoch_drift` 0; base `tx_stream_streak_max` 1; `STOP RULE: not tripped`. Also from the 2026-10-18 rule: max gap 0.9 s (≤ 1 s), whole-log loss 44/1769 = 2.5 % (leg U 10.5 %, leg X 6.5 %) | `legs/leg2c_r2_deltas.txt`, `legs/leg2c_r2_gaps.txt`, `legs/leg2c_r2_report.txt` |
| a tractor that keeps streaming holds FAILED (authority); a stall ≥ 1 s would restart its scan under (b) | observed: no later 0x0D during the leg, tractor `clk_demotion_reset` 0 and `clk_demotion_kept` 0 (post values; its L072 was reset at leg start), `tx_stream_streak_max` 1862: it held FAILED under authority to the end | archive `tx_daemon.log`; `legs/leg2c_r2_post_tractor.txt` |

Also on R2:

- **Base:** no FAULT line during the leg; first decoded frame 26.7 s after
  its profile-1 activation (the tractor's #41), inside its first attempt;
  `ACQUIRED` (base Δ`clk_demotion_reset` 1, 1725 of 1769 fragments, 97.5 %)
  (`legs/leg2c_r2_report.txt`, archive `rx_daemon.log:45`).
- **Commands on air:** all 95 were opcode 0x63 (ENCODE_MODE). The injector
  published 171 ENCODE_MODE and 22 key-frame requests (`DUAL_DONE em=171
  kf=22 dur=120`, exit 0, `legs/leg2c_r2_inject.log`); the key-frame requests
  stay at the base under `-KfRequestDisable 1`. The tractor applied 93
  (`encode_mode -> 0 (full) [lora_cmd]`, 93 lines in archive
  `camera_service.log`).
- **Frames:** 998 frames on air as 1769 fragments (K = 1: 339, K ≥ 2: 659,
  max K 3), 0 ABORTED, `drop_full=11`; 954 published: `mono_g4` seq 41-234
  (194 received, 0 lost), then webp full frames from the first applied
  ENCODE_MODE on (760 received, 15 lost); seq-gap loss 15/969 = 1.5 %, every
  gap a single frame; `crc_dumps=1` (−119 dBm, SNR −12.2 dB), `timeouts=3`
  (`legs/leg2c_r2_report.txt`, `legs/leg2c_r2_p2p4.txt`,
  `legs/leg2c_r2_rf.txt`).
- **FIRMWARE DROP 1818 − 1725 = 93** (`legs/leg2c_r2_report.txt`) equals the
  93 `ENCODE_MODE_ACK` lines in archive `rx_daemon.log`: the tractor's command
  acks, which the base's radio counts and the report's fragment count leaves
  out. Inferred from the equality; not a firmware drop.
- **Level:** −64 dBm / SNR 10 dB (window minimums −74 dBm, 3 dB)
  (`legs/leg2c_r2_rf.txt`).
- **Kit checks:** `LOG_WINDOW after_stop`, reset landed (own tag, banner,
  `OCD_EXIT=0`, `legs/leg2c_r2_tractor_ocd_reset.log`), deploy md5 52/52 on
  both plus the 2 reset files and the injector's 2 (`*_deploy_md5=` in
  params), clocks numeric (tractor −1667466.124 s, base −0.048 s),
  scorer exit codes all 0 (`rs13_probe_score` skipped: no PROBE on a
  `-ReactiveFire 0` leg) (`legs/leg2c_r2_harness.txt` lines 9-10 and
  6864-6875, `legs/leg2c_r2_params.txt`, `legs/leg2c_r2_post_transcript.txt`).
- **Park:** `attempt 1: PARK_OK`, `"path": "firmware"` on both at 14:59:46Z
  (post step done 14:59:55Z); check-only 15:00:56Z / 15:01:00Z (about 1 min
  later) and 15:02:35Z tractor / 15:02:55Z base (2 min 49 s to 3 min 9 s
  after the park step): `0x80 SLEEP` on both every time
  (`legs/leg2c_r2_park.txt`, `legs/leg2c_r2_post_transcript.txt`,
  `legs/power_up_guard_2026-10-25.log`).

## Observations outside the Confirm lists

- **The post-leg scan is visible, and the park stops it.** One post-bracket
  probe per leg caught a FAULT_URC in its settle window (the other two
  caught none: `legs/leg2c_r1_post_base.txt`,
  `legs/leg2c_r2_post_tractor.txt`), printed in the probe's raw
  form: the tractor at 14:45:13Z after R1, `code=0x0D sub=0xE3`, L072 uptime
  467.3 s (`legs/leg2c_r1_post_tractor.txt`), and the base at 14:59:38Z after
  R2, `code=0x0D sub=0xE3`, uptime 2801.0 s (`legs/leg2c_r2_post_base.txt`;
  uptime from payload bytes 20-23). 0xE3 is a valid scan sub: attempt 3,
  `final`, `warm`, `hw_suspect`. Each came 2.0-2.5 min after the last frame
  that board took part in (R1: the tractor's last TX, 07:31:36.970 tractor
  clock = about 14:42:43Z by the params offset, archive `tx_daemon.log:2953`;
  R2: the base's last fragment, 14:57:37.545 base clock, archive
  `rx_daemon.log:6885`): a node without authority scanning on under option
  (b) after the daemons stopped, as section 4 (b) of the memo says. The
  firmware parks 5 s and 8 s later then held at every `--check-only`
  reading. The R2 line is the base's own scan fault, seen by the probe; the
  base daemon logged no FAULT line during either leg.
- **Tractor `radio_tx_ok` is one more than its logged transmissions** on
  both legs: 610 against 609 `txdone_arrival` lines on R1; 1863 against 1769
  fragments plus 93 command acks on R2, if each of the 93 commands got one
  ack (the base logged 93 `ENCODE_MODE_ACK` lines)
  (`legs/leg2c_r{1,2}_post_tractor.txt`, `legs/leg2c_r{1,2}_p2p4.txt`). The
  same +1 was on every 2026-10-24 leg (that day's RESULTS, Other
  anomalies). No score uses it.
- **R2 boot fault 0x0A** (`HOST_RX_INACTIVE`, `reason=no_ingress_during_probe_window`,
  uptime 3.0 s; archive `tx_daemon.log:13`) before the tractor's 0x09 and
  0x0C. Also logged on two 2026-10-18 legs (`radio_monitor_20261018_111558_d484a92d`,
  `radio_monitor_20261018_113831_d484a92d`). A boot-time fault, not a scan
  fault; not investigated.
- **Tractor on R2:** `radio_crc_err` 0, Δ`rx_urc_lost` 2 (post values,
  `legs/leg2c_r2_post_tractor.txt`). Whether the 2 unserviced DIO0 edges are
  the 2 missed commands is not shown.

## Comparison with the legs these repeat

| | R1 (`0daae279`) | 2026-10-24 `mono_g4` legs 2c_f1-2c_f3 (`e083ae46`) | R2 (`0daae279`) | 2026-10-18 leg X (`e083ae46`) |
|---|---|---|---|---|
| base first frame | 11.0 s after activation | 75.0, 51.4, 15.2 s after RX ready (about 0.06 s after activation) | 26.7 s after activation | not recomputed |
| command delivery | PROBE 204/204 | 161/161, 179/179, 202/203 | 0x63 93/95 in FAILED | 0x63 36/36 in FAILED |
| tractor scan faults | none (locked on PROBE 1) | 0x01 on 2c_f1, 0x20 on 2c_f2, none on 2c_f3 (read as RX_SCAN_FAILED by timing) | 0x00 0x01 0x02 0x83, last 113 s after MONITORING | 0x00 0x01 0x02 0x83, last 117 s after MONITORING |
| loss | whole-log 10/609 (head only); seq-gap 0/599 | seq-gap 0/475, 0/523, 2/592 | whole-log 44/1769 = 2.5 %; seq-gap 15/969 | whole-log 130/2010 = 6.5 % |
| leg-end park | firmware path, attempt 1, held | register path, attempt 1, held after the scan failed out | firmware path, attempt 1, held | register path; the 10-18 parks took up to three attempts per board |

Sources: this file's R1 and R2 sections;
[2026-10-24 RESULTS](../bench_session_2026-10-24/RESULTS.md) "Results per
leg" and "Other anomalies";
[2026-10-18 RESULTS](../RS_12_15_reverse_delivery_2026-10-18/RESULTS.md)
(leg X: lines 254 and 403); DC/TODO.md, the RS-12.15 reverse-delivery half
(leg X) and the "Seen again for RS-12.21" park note. Two starts are not a
cold-start rate: that is T2.

## Scene notes

- **The bench Firefox profile was broken.** `youtube_window.ps1 -Open` with
  the bench profile (`%TEMP%\lifetrac-bench\ff_bench_youtube`) opened windows
  with no page. The profile folder was renamed to
  `ff_bench_youtube.bak-20261025`, and the next `-Open` created a fresh
  profile that loaded the video (session console). Why the old profile
  failed was not investigated.
- **Windows refused the focus change.** `-Front` printed "bench window is
  not in the foreground (Windows refused the focus change); 'f' NOT sent"
  until a synthetic Alt press and release around `SetForegroundWindow` (a
  scratch script of this session) brought the window to the front. After
  that both prep runs read `player fullscreen: True` with the window title
  `Front view of Colorado's STUNNING Royal Gorge Route Railroad - YouTube -
  Mozilla Firefox` (`legs/leg2c_r{1,2}_prep.txt`). That line compares the
  window's rectangle with the screen only, not its z-order, so it does not
  show that the window was in front; the scene frames and the base captures
  do (below). `youtube_window.ps1 -Front` now makes the Alt retry itself,
  sends "f" only to a window confirmed in front, and exits 6 when the
  window is still not in front, whatever the fullscreen state; its last
  line adds `in front:`. `scene_check.sh` then stops with
  `SCENE-CHECK-NOT-FRONT` (exit 5) before it opens the camera, so no frame
  is taken and `leg_prep.sh` aborts (`bench_tools/legs/README.md`, the
  script table; the profile rename is in its "A session", step 3).
- **One diagnostic frame was deleted.** A `scene_check` frame taken while
  the bench window was not in front captured a non-bench window. It was
  deleted and was never committed or kept in this folder. `-Front` then
  only printed a message and `scene_check.sh` took the frame anyway; both
  now refuse (above).
- **The scene checks that count** passed the 8 % rule: pre-R1 14:36:08Z
  58.28 % over 10 s (`legs/preR1_scene.txt`), R1 14:36:38Z 65.24 %
  (`legs/leg2c_r1_scene.txt`), R2 14:48:07Z 74.61 % (`legs/leg2c_r2_scene.txt`).
  The kept frames show YouTube content only, but not the railroad video:
  pre-R1 and R1 a sponsored product ad, R2 a talking-heads clip with a
  YouTube Premium card (`legs/preR1_scene.jpg`, `legs/leg2c_r{1,2}_scene.jpg`),
  as on 2026-10-24. The jpgs stay on the bench PC: `*.jpg` is git-ignored
  (`.gitignore:12`), so they are not committed (nor were the 2026-10-24
  ones); the `_scene.txt` files are.
- **Privacy check.** The main session looked at every scene jpg. The base
  captures `legs/leg2c_r{1,2}_base.jsonl`, which are committed, hold the
  screen as transmitted. A review rendered both so that every tile write
  landed in a saved canvas before it was overwritten: R1 806 tile writes in
  12 canvases, R2 1635 in 18, 0 left unrendered, 0 webp failures (review
  render, not archived). Every canvas shows YouTube content only (the
  player with its YouTube Premium card once the canvas is full): the
  railroad video for part of each leg, ads otherwise. The first check had
  rendered 12 frames per capture, which left 582 of R2's 1635 tile writes
  (5 of R1's 806) overwritten before any rendered frame; the full render
  closes that.
- The scene bears on frame sizes and encoder load, not on acquisition or
  command delivery, which is what these legs test.

## What was NOT tested

- **T1** (forced FAILED: the base silent for ≥ 125 s, then the stream): needs
  a harness option that delays the tractor's stream, not built. Without it,
  T3's recovery half ("LOCK a median ~35 s after the stream starts or
  sooner") is not tested on air: both legs' bases locked in their first
  attempt.
- **T2** (a cold-start set, about 10 starts of ≥ 150 s): the cold-start
  FAILED rate on air is not measured. Two starts (11.0 s, 26.7 s) are not a
  rate.
- **T4 with camera stalls** (the decay window on this image, memo section 5
  item 3); R2 is only T4's first half, with no stalls.
- the fix's late-acquisition path: no base reached a final 0x0D during a leg,
  so "a final followed by more finals 30 s apart until the lock" was not seen
  in a leg (only the post-leg finals above).
- VECTOR on this image; 1 fps or 5 fps; DTS (profile 2).
- Still open from earlier rounds: the FHSS switch leg ("2d on profile 1"), the
  web_ui leg, and the A16 confirmation (it needs a lossy link).
- The base L072 reset (`BASE_L072_RESET unverified` on both legs; its
  counters are post − pre).
- A rollback: not needed.

## Evidence limitations

- The pre-flash checks (`make check`, the clean build, the harness tests, the
  `e083ae46` park reading, the step-5 preflight), the GO wording, the flash PC
  times, the post-flash park times and the scene repairs are from the session
  console, not archived.
- The tractor console's stamps are the tractor clock (2026-10-06 before the
  flash reboot); the PC times of its flash come from the session console.
  The params clock offsets were read after each leg's daemons stopped and do
  not apply to times before the flash reboot.
- The post-leg 0x0D lines come from the bracket probe's settle window, which
  catches only what arrives while it listens; the other post-leg attempts
  went unlogged.
- R2's two missed commands are not identified.
- The scene jpgs (ads and a talking-heads clip) stay on the bench PC,
  git-ignored. What was on screen during each leg is in the committed base
  captures (`legs/leg2c_r{1,2}_base.jsonl`), which hold the transmitted
  screen: the railroad video for part of each leg, ads otherwise (from the
  privacy render, which is not archived).

## Next steps

No radio:

1. Build and test offline the harness option that delays the tractor's
   stream, for T1.
2. Write T2's set (about 10 profile-1 cold starts of ≥ 150 s at 2 fps) and its
   pass rule before flying it: first-frame time, base 0x0D finals, post
   `radio_state`.
3. Plan T4 with 2.5 s camera stalls on this image (the decay-window
   trade-off accepted in D-FHSS1).

Radio, each on its own GO:

4. T1 and T2 on this image (T3's recovery half and the cold-start rate on
   air).
5. T4 with camera stalls.
6. Still open: the FHSS switch leg, the web_ui leg, the A16 confirmation.

Rollback, only if needed and on its own GO: the flown `e083ae46` copy,
`bench-evidence/RS_12_15_reverse_delivery_2026-10-18/firmware_flash/firmware_bench_diag_e083ae46.bin`,
with `flash_l072.sh`.

## Files

- **`firmware_flash/`:** `flash_tractor_console.txt`, `flash_base_console.txt`
  (the `--go` runs, preflight included); `firmware_bench_diag_0daae279.bin`
  (the flashed image); `postflash_health_{tractor,base}.txt` (step 8);
  `postflash_park_{tractor,base}.txt` (step 9).
- **`legs/`, session:** `power_up_guard_2026-10-25.log` (14:07Z power-up
  guard; check-only at 14:26Z, 14:47Z, 15:00Z and 15:02Z);
  `staging_{tractor,base}.txt` (before and after the flash);
  `preR1_scene.txt`. The `scene.jpg` files (pre-R1 and per leg) stay on the
  bench PC: `*.jpg` is git-ignored.
- **`legs/`, per leg** (`leg2c_r1_`, `leg2c_r2_`): `prep`, `scene.txt`,
  `health_{base,tractor}`, `clear_retained`, `pre_`/`post_{base,tractor}`
  brackets, `link_stats`, the base capture `base.{jsonl,json,txt}`,
  `harness`, `params`, `report`, `p2p4`, `gaps`, `rf`, `deltas`, `replay`,
  `tractor_log`, `tractor_ocd_reset.log`, `park`, `post_transcript`; R1 adds
  `probe_score`; R2 adds `inject.log` and `inject_trigger.txt`.
- **Harness archives** (in `bench-evidence/`; `params.txt`, `rx_daemon.log`,
  `tx_daemon.log`, `camera_service.log`, `deploy_md5.txt`,
  `tractor_ocd_reset.log`; R2 adds `inject.log` and `inject_trigger.txt`):
  - R1 (2c_r1): `radio_monitor_20261025_094252_d597b094`
  - R2 (2c_r2): `radio_monitor_20261025_095748_d597b094`

## Session end

- 15:02:35Z tractor / 15:02:55Z base: `--check-only`, both `0x80 SLEEP`, no
  holders, rc 0 (`legs/power_up_guard_2026-10-25.log`). Both radios parked by
  the firmware path.
- The PC MQTT broker that the harness started was stopped, and the bench
  video window was closed (session console).
- Both L072s run bench `0daae279`. Production `2c5e055a` is on no board.
