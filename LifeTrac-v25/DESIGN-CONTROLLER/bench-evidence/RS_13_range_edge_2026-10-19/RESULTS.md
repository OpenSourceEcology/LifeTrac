# RS-13 antenna legs — VECTOR vs mono_g4 command delivery and the A16 check (2026-10-19)

**Status: flown and scored 2026-10-19. Verdict: on DTS at bench distance,
VECTOR's command delivery (§8.6) is not worse than `mono_g4`'s (433/433 vs
418/420, Fisher p = 0.242), and neither is its frame loss (P3: 1/1228 vs
1/1230, p = 1). The A16 fix was not exercised on air: the one VECTOR loss
opened no resync episode, so the check scored nothing (thin).**

Procedure: firmware/x8_lora_bootloader_helper/bench_tools/RS13_RANGE_EDGE_LEG.md
@ `1d10f57f`, flown as adapted for the antennas in PR #159 (`a95ef481`, the
status block: no attenuation equipment, operator 2026-10-19). Leg criteria:
RS13_VECTOR_LEG.md as amended 2026-10-10. The folder keeps the procedure's
name (`RS_13_range_edge_<date>`) so `leg_prep.sh` needs no override.

GO: 2026-10-19 12:47 CDT (17:47Z), operator: "Okay GO for next radio test".
Scope, per DC/TODO.md "Next radio session = the antenna legs" item (1): the
receive-only channel spot-check, then the DTS legs A2c, A2a, A2c2, A2a2 in
that order, on the antennas. A re-fly or an added leg needs a new GO. All
legs fly on the local date of the carrier pick (before 23:59 CDT).

Rules stated before the first leg:

- **P3:** pooled seq-gap loss, VECTOR (2a + 2a′) against `mono_g4` (2c + 2c′):
  not worse = VECTOR loss ≤ `mono_g4`, or a two-sided Fisher exact test on
  lost / received with p ≥ 0.05. No margin.
- **Command delivery (§8.6):** the same test on PROBE delivered / undelivered
  (`rs13_probe_score.py`, pooled by codec).
- **A16:** `rs13_digest_trace.py` on the A2a and A2a2 base captures. An
  isolated-loss episode ended by an epoch start within 25 rows, with the
  base back in step, is reported as the tool prints it ("FAIL by the
  letter"), named as not the A16 shape, and left to the design owner rather
  than re-judged. Fewer than 3 loss-opened episodes is thin, and with no
  Round B to pool from it stays thin.
- **Open sign-offs (not waived by the GO):** P8 as an absolute 350 ms bound
  (A17), the P6 return clause (A18; n/a, no switch leg), and the VECTOR_SCENE
  amendment c922efbb (the DIGEST exit from resync) await the design owner;
  these legs are judged by the amended text.
- **Desk check (RS-13.2):** not run; not a prerequisite here (web_ui is not
  in the loop).

## Software under test

| item | value |
|---|---|
| A16 fix | PR #143 (merge `cf71a587`), an ancestor of the images' `f6e4b463`; `a16_sil.py` on the merged code: A16-like bw500 0/100, bw250 0/100, recovery max 10 / 11 frames, lossfree_out_of_step_frames 0 (DC/TODO.md A16 item) |
| code guards | PR #143 (batching, cold-start budget clamp, double epoch start, switch gap) |
| tractor image | `lifetrac-tractor-x8:latest` id `b2a3c8070240`, built 2026-10-14 from `f6e4b463`; step 0 smoke 47 tests OK (2026-10-14); step 1 from `/app`, bw500 moving, 2026-10-19 at 243 B (`legs/step1_*bw500_moving*`; exit 0 `done` on the console, not in a file; the files show neither exit-6 condition: bootcheck `byte_budget=243 B`, no clamp line): encoder p50 / p95 / max 199.7 / 221.8 / 227.7 ms, 251 VECTOR frames, wire p95 242 B / max 243 B, 4 relabel epoch starts |
| base image | `lifetrac-v25:latest` id `0413f3c4dac3`, built 2026-10-14 from `f6e4b463` |
| code that runs | the harness pushes the daemons and pipelines from the flight tree `1d10f57f` each leg; those files are identical to `f6e4b463` |
| staging | 2026-10-19 18:09–18:10Z from `1d10f57f`; md5 `ok` on every line: base 15 staged files + 37 code-tree files (52 lines), tractor 16 + 37 (53 lines; the extra one is `08_boot_user_app.cfg`), incl. `channel_survey_sniff.py`; records `legs/staging_base.txt`, `legs/staging_tractor.txt` |
| dials | `LIFETRAC_VECTOR_DETAIL=80` (V0), camera 2 fps, `LIFETRAC_KEYFRAME_COPIES` 1, `LIFETRAC_WEBP_QUALITY` 55, `-TxPowerDbm` empty (14 dBm), `-TxBatch 0`, `-KfRequestDisable 1`, `-ReactiveFire 1 -ProbeEcho 0` |
| scene | the railroad video in a normal Firefox window, player fullscreen (no kiosk); preflight 57.0 % change in 10 s; scene check before every leg (`legs/<leg>_scene.{txt,jpg}`) |

## Firmware on the boards

| board | build | md5 | flashed |
|---|---|---|---|
| base 2D0A1209DABC240B | bench (register-write diag) build, PR #152 | `e083ae46a64686031b39866eca42e094` | 2026-10-18 15:52Z |
| tractor 2E2C1209DABC240B | same | `e083ae46a64686031b39866eca42e094` | 2026-10-18 15:50Z |

Session health before the spot-check (18:13:47Z / 18:13:49Z): `STATS-OK`
on both boards, `radio_state=1`, `RS115-INSTRUMENTED-FIRMWARE=YES`
(`legs/session_health_base.txt`, `legs/session_health_tractor.txt`).

## RF setup

Not flown: no attenuation equipment (2026-10-19). Both boards on their
antennas at bench distance, as for the 2026-10-18 RS-12.15 legs. No
conducted path, no pads, no shielding. R0's −65 to −75 dBm window belongs to
the conducted path: the level below is recorded only.
(`rs13_rf_extract.py` prints "R0 over 4 legs … RSSI −70.0 dBm / SNR 6.0 dB;
target −65 to −75 dBm: in range", `legs/roundA_rf_level.txt`. That verdict
does not apply here.)

Leakage check: not flown (no terminations; no attenuation equipment).

Spot-check (receive-only, on the base, profile 2, 30 s per channel):
`legs/spot_check_2026-10-19.jsonl`, 18:14:21Z to 18:40:06Z, rc 0
(`legs/spot_check_2026-10-19_times.txt`). 51 channels, 49 hot. Only two
were zero-hot: 922.5 MHz (max −77 dBm, n = 474) and 927.5 MHz (max −94 dBm,
n = 473) (`legs/spot_check_2026-10-19_compare.txt`).

- **Pick: 927.5 MHz** (`-ForceFrfHz 927500000`, the compare's RECOMMENDED
  line).
- **Floor at the pick:** 0 hot samples, max −94 dBm over its 30 s dwell. It
  was the survey's last channel.
- **Change since the 2026-08-22 survey**
  (`RS_11_6_channel_survey_2026-08-22/survey_base_day3.jsonl`): newly hot
  914.5 MHz (−91 → −43 dBm) and 916.5 MHz (−90 → −44 dBm); newly clean
  922.5 MHz (−46 → −77 dBm); 3 of 51 channels flipped.
- **Time to the first DTS leg:** about 2 min, from the survey end
  (18:40:06Z) to the A2c harness launch (18:42:11Z, first line of
  `legs/legA2c_harness.txt`).
- `FRF readback: 927.500 MHz (OK)` on both boards on every DTS leg: **yes**
  (`legs/roundA_rf_level.txt`).

## The DTS legs (@ 927.5 MHz)

Flight order: 2c (A2c), 2a (A2a), 2c′ (A2c2), 2a′ (A2a2). Harness launches
18:42:11Z, 18:51:27Z, 19:00:46Z and 19:10:29Z (first line of each
`legs/leg<X>_harness.txt`); 300 s each, profile 2 (bw500, DTS), camera feed
at 2 fps. Base capture per leg: `rs13_cap_base`, profile image_bw500, 480 s,
min-frames 500 (`--strict` on the VECTOR legs).

Harness lines (`legs/legA2c_prep.txt` and `legs/legA2a_prep.txt`; A2c2 and
A2a2 repeat them exactly):

```
A2c, A2c2:
.\run_live_radio_monitor.ps1 -TxAdbSerial 2E2C1209DABC240B -RxAdbSerial 2D0A1209DABC240B -HostIp 192.168.1.79 -TxFeed camera -RegProfile 2 -ForceFrfHz 927500000 -DurationS 300 -SynthFps 2 -KfRequestDisable 1 -ReactiveFire 1 -ProbeEcho 0 -LogFragArrivals 1 -TxBatch 0 -CamExtraEnv "-e LIFETRAC_ENCODE_MODE=6" -Archive
A2a, A2a2:
.\run_live_radio_monitor.ps1 -TxAdbSerial 2E2C1209DABC240B -RxAdbSerial 2D0A1209DABC240B -HostIp 192.168.1.79 -TxFeed camera -RegProfile 2 -ForceFrfHz 927500000 -DurationS 300 -SynthFps 2 -KfRequestDisable 1 -ReactiveFire 1 -ProbeEcho 0 -LogFragArrivals 1 -TxBatch 0 -CamExtraEnv "-e LIFETRAC_ENCODE_MODE=9 -e LIFETRAC_VECTOR_DETAIL=80" -Archive
```

Tractor reset per leg (`TRACTOR_L072_RESET` in the harness's LEG CHECKS and
`tractor_l072_reset=` in each `legs/leg<X>_params.txt`): **landed** on A2c,
A2a and A2a2 (PF4/NRST read back low then high, openocd exit 0). **Failed**
on A2c2 (SIGHUP, exit 129, openocd never started; see Anomalies). A2c2's
tractor brackets are scored with `deltas.py` as post − pre
(`legs/legA2c2_deltas.txt`). `BASE_L072_RESET unverified` on all four legs,
so base brackets are post − pre. No base counter fell during the round (base
`radio_tx_ok` 158 → 369 → 586 → 797 → 1015, `legs/legA2*_pre_base.txt` and
`legs/legA2*_post_base.txt`).

| # | criterion | 2c | 2a | 2c′ | 2a′ | pass? |
|---|---|---|---|---|---|---|
| archive | | `radio_monitor_20261019_134805_1d10f57f` | `radio_monitor_20261019_135659_1d10f57f` | `radio_monitor_20261019_140618_1d10f57f` | `radio_monitor_20261019_141603_1d10f57f` | |
| P1 | dry-run checks, loss rule (`store:`, `resync:` episodes, final `digest_ok`, `last DIGEST checked`) | n/a | 10/10 `[PASS]`, `RESULT: PASS`; orphans 0, digest 598 checked / 0 mismatched, `resync: none`, final `digest_ok True`; last DIGEST (611, ok) | n/a | 10/10 `[PASS]`, `RESULT: PASS`; orphans 0, digest 568 / 0, `resync: none`, final `digest_ok True`; last DIGEST (614, ok); 0 mismatch runs after the row-268 loss | PASS |
| P2 | `published frame_id` ÷ 300 | 612 ÷ 300 = 2.04 | 608 ÷ 300 = 2.03 | 609 ÷ 300 = 2.03 | 612 ÷ 300 = 2.04 | PASS (≥ 1.8; 2c is the baseline) |
| P3 | `capture seq gaps`; pooled VECTOR 1/1228 vs mono_g4 1/1230, Fisher p 1 | 0/616 | 0/612 | 1/614 (row 532) | 1/616 (row 268) | PASS |
| P4 | max K, K = 1 count, ABORTED | 1, 611, 0 | 1, 607, 0 | 1, 609, 0 | 1, 612, 0 | PASS (K ≥ 2: 0 on every leg) |
| P5 | gaps > 3 s | 0 (max gap 0.6 s) | 0 (0.6 s) | 0 (1.0 s) | 0 (1.0 s) | none; required on FHSS 2b only, recorded |
| P7 | link_stats `rx_codec_name` | `mono_g4` | `vector` | `mono_g4` | `vector` | PASS (one sample per leg) |
| P8 | encoder p95 vs 350 ms (amended); vs step 1 (221.8 ms) for information | n/a | 247.2 ms (p50 206.0, max 293.7); +11.5 % vs step 1 | n/a | 261.3 ms (p50 211.8, max 299.9); +17.8 % vs step 1 | PASS (amended text; A17 sign-off open); `drop_full=0 drop_stale=0` on both |
| level | `rx_rf:` median RSSI / SNR (recorded only) | −70.0 dBm / 6.0 dB (min −72 / 5.0), CRC err 2 | −70.0 / 6.0 (min −71 / 5.0), CRC err 1 | −70.0 / 6.0 (min −72 / 5.0), CRC err 2 | −70.0 / 6.0 (min −72 / 5.0), CRC err 5 | recorded only |
| cmd | PROBE delivered / sent (tractor ∩ base seqs) | 209/210 | 216/216 | 209/210 | 217/217 | see below |

Sources: P1 `legs/leg<X>_post_transcript.txt`, `legs/leg<X>_digest_trace.txt`,
`legs/leg<X>_replay.txt`; P2, P4 `legs/leg<X>_p2p4.txt`; P3
`legs/leg<X>_report.txt`, `legs/roundA_p3.txt`; P5 `legs/leg<X>_gaps.txt`;
P7 `legs/leg<X>_link_stats.txt`; P8 `legs/leg<X>_tractor_log.txt`; level
`legs/roundA_rf_level.txt`; cmd `legs/leg<X>_probe_score.txt`.

P3 detail:

- The harness's counter `loss` line reads 0/611, 0/607, 0/609 and 0/612
  (`legs/leg<X>_report.txt`). It compares the two logs' own windows
  (RS_12_15_clock_authority_2026-09-14/ERRATA.md §3), so the seq-gap figure
  is the one scored.
- Brackets, tractor Δ`radio_tx_ok` against base Δ`radio_rx_ok`: 617/617,
  613/613, 615/614, 617/616. The two legs with a seq gap are the two that
  are one short.
- A2c2's lost frame (row 532; seq 21 is missing between 19:05:34.108Z and
  19:05:35.131Z in the archive's `rx_daemon.log`) left no CRC dump in that
  gap.
- A2a2's lost frame (row 268; seq 13 is missing between 19:13:04.764Z and
  19:13:05.744Z) has two CRC dumps inside that gap: a noise-floor dump at
  19:13:04.831Z (rx_len 228, −115 dBm, SNR −11.8 dB, like the round's other
  dumps) and, at the time seq 13 was due, 19:13:05.259Z, rx_len 248,
  RSSI −69 dBm, SNR 2.8 dB. That fits a frame that arrived at the normal
  level, with SNR about 3 dB below the leg's median, and failed its CRC.

Command delivery (§8.6): VECTOR 433/433 = 100.0 % vs mono_g4 418/420 =
99.5 %, Fisher p = 0.242, **PASS** (`rs13_probe_score.py` prints "NOT WORSE
(VECTOR >= mono_g4)", `legs/roundA_probe_score_pooled.txt`).

Both `mono_g4` misses are PROBE seq 210, the last PROBE of the leg. The
tractor's log had already ended when it would have arrived:

- On A2c the base logged seq 210 on air 1.02 s after seq 209. The tractor
  log ends 0.24 s after the tractor logged seq 209.
- On A2c2 those figures are 1.51 s and 0.74 s.
- On both legs the tractor L072's leg count of `radio_rx_ok` (211) equals
  the base's Δ`radio_tx_ok` (211). So the counters show no air loss there.

Sources: the archives' `rx_daemon.log` (base) and `tx_daemon.log`
(tractor), `legs/legA2c_post_tractor.txt`, `legs/legA2c2_deltas.txt`. The
rule scores both PROBEs as undelivered, and the verdict stands as scored.
Inside the tractor's log window, every PROBE was delivered on all four legs.

A16 confirmation (bw500):

| leg | loss at row | episode (start-end, frames, s) | ended by | back in step (`rs13_digest_trace.py`, frames after the loss) | isolated? | pass? |
|---|---|---|---|---|---|---|
| A2a | – (no seq gap) | no resync episode | – | – | – | – (nothing to score) |
| A2a2 | – in the tool's row; the leg's one seq gap is at row 268 (1 frame) | no resync episode | – | never out of step: 0 DIGEST mismatch runs; row 268 and every row to the next epoch start (row 294) read `dig=ok` | yes (the leg's only loss) | – (not scored: no episode) |

`rs13_digest_trace.py` over A2a + A2a2: 0 episodes, loss-opened 0, "THIN:
fewer than 3", isolated and scored 0 (0 PASS, 0 FAIL), **NO SCORED EPISODE**
(`legs/roundA_digest_trace.txt`; per leg `legs/legA2a_digest_trace.txt`,
`legs/legA2a2_digest_trace.txt`; rows from `legs/legA2a2_base.txt`). The
reason: the A16 fix acts on a resync episode that a loss opens. A2a lost no
frame. A2a2's one lost frame came 4 rows after the epoch start at row 264
and caused no DIGEST mismatch, so no episode opened. **The A16 fix was not
exercised on air.** There is no Round B to pool from, so this stays thin.

Base `ttl_dropped` 0 on both VECTOR legs (`store:` lines). Encoder counters
per leg (the `tractor-log` `counters` line, `legs/legA2a_tractor_log.txt`,
`legs/legA2a2_tractor_log.txt`; one `camera_service` run each):

| leg | frames | epoch starts (first / forced / horizon / safety / relabel) | repair share frames | share re-sends (repeat-once) | carousel re-sends | DEL repeats | ttl_dropped |
|---|---:|---|---:|---:|---:|---:|---:|
| A2a | 607 | 15 (1 / 0 / 6 / 1 / 7) | 151 | 714 (70) | 644 | 2097 | 0 |
| A2a2 | 612 | 17 (1 / 0 / 6 / 2 / 8) | 153 | 724 (86) | 638 | 2139 | 0 |

The share frames match V0's one frame in four (⌊607 ÷ 4⌋ = 151, 612 ÷ 4 = 153).

Episodes scored: 0 (thin).

Emitter fold: not required. The rule asks for it on archives with ≥ 5 lost
frames, and the most any leg lost was 1. It was run on all four anyway
(`legs/roundA_fold_emitter.txt`). At every period it gives lost R = 0.000,
p = 1 and hot R = 0.000, p = 1. The random-frame null gives P = 1.000 on A2c2
and A2a2. With at most one loss per leg and no hot CRC dumps (the script's "hot" = RSSI
above −60 dBm with SNR below −5 dB; A2a2's −69 dBm / 2.8 dB dump is not one), the script
returns R = 0 (it needs two events). So it says nothing either way.

## Rounds B and C, the edge

Not flown: no attenuation equipment (2026-10-19). No range data comes from
this bench; it needs a distance test (top-level TODO.md, S-HW.2).

## VECTOR_SCENE.md §8.6 rows

| measure | criterion | result |
|---|---|---|
| Base command delivery during VECTOR | not worse than during `mono_g4` | DTS on the antennas: VECTOR 433/433 vs `mono_g4` 418/420, Fisher p = 0.242, not worse (PASS); both `mono_g4` misses are the leg's last PROBE, sent after the tractor log ended; FHSS: not measured here (a separate leg) |
| Fragment loss | not worse than `mono_g4` on the same workload and profile | P3 pooled seq-gap loss VECTOR 1/1228 (0.081 %) vs `mono_g4` 1/1230 (0.081 %), Fisher p = 1, not worse (PASS); DTS profile 2, bench distance, about −70 dBm |
| Range-edge leg | per 3 dB step | not flown: no attenuation equipment (2026-10-19) |

## What these legs did NOT test

- V1–V3 (the ladder) and with it the §8.6 L0/STATUS clause; self-select (D-VS6b), B4, B5
- FHSS command delivery and the FHSS switch leg (separate legs, next in DC/TODO.md)
- the mode switch (no 2d leg); web_ui in the loop; the production stack (B1, RS-9.2/9.4, RS-11.7)
- range: bench distance on the antennas, one received level; no attenuation, no distance
- the A16 fix on air: no loss opened a resync episode (A16 section)
- nothing was cut on the day: the spot-check and all four legs flew as
  scoped; `-InjectCmd` was not used (`inject_cmd=` empty in every
  `params.txt`)

## Evidence limitations

- Tractor-side receive level: not logged. The tractor's `LoRa cmd: PROBE`
  lines carry no RSSI or SNR. The level above is the base's only.
- The scene was checked before each leg, not during it.
- n per leg: 210–217 PROBEs and 612–616 frames, two legs per codec. A
  difference of two PROBEs or one frame is well inside chance (p = 0.242,
  p = 1).
- One carrier and one level (about −70 dBm, SNR 6 dB) on a link with
  margin. Two frames were lost in the whole round, so nothing here bears on
  loss behaviour or on the A16 path.
- P7 rests on one `link_stats` sample per leg, the first message with
  frames, taken at leg start.
- The base L072 was not reset (`BASE_L072_RESET unverified`), and neither
  was A2c2's tractor L072. Those brackets are post − pre.
- The tractor's logs run on the tractor clock (they read 2026-09-30).
  Cross-board timing above uses matched events (PROBE seq 209 on both logs),
  never the clocks.
- `legs/roundA_fold_emitter.txt` was regenerated in full after the round
  (26 lines, all four archives): lost 0 / 0 / 1 / 1 and hot CRC dumps 0 on
  every leg, R = 0.000 at every period.
- `rs12_leg_report.py` prints "received > sent: the TX and RX logs disagree
  on the window" on A2c (612 > 611) and A2a (609 > 607). This is the known
  window effect, not loss.

## Anomalies for the record

- **A2c2: the tractor's SWD reset never ran.**
  `legs/legA2c2_tractor_ocd_reset.log` holds only the `OCD_LEG=20261019T190055Z`
  tag, `OCD_EXIT=129`, `__OCD_LOG_RC=0` and `__OCD_RUNNING=0`, with no openocd
  banner. The harness verdict reads "failed (the nohup openocd job was killed
  by SIGHUP … openocd never started (no banner in its log) (exit 129) before
  the NRST pulse)" (`legs/legA2c2_params.txt`, `legs/legA2c2_harness.txt`).
  - This is the signature of BENCH_RUNBOOK candidate (c), the SIGHUP race,
    and the first archived log of it.
  - A2c2 was not the first leg after a boot. The tractor was up 1 day 3:28
    at 19:19:48Z (`legs/session_end_park_2026-10-19.txt`), so it last booted
    about 2026-10-18 15:51Z, at the flash. This log therefore does not show
    whether the first-leg misses are (c) too.
  - The tractor counters carried over (`radio_tx_ok` 613 → 1228), so they
    were scored with `deltas.py` (`legs/legA2c2_deltas.txt`).
  - Its RS-12.15 bracket criteria (`tx_first_anchor` [FAIL], STOP RULE
    UNDETERMINED) are FHSS clock-authority checks. They do not apply to a
    DTS leg.
  - Its cross-check reads DISAGREE because the base brackets count 211 sends
    and the base log 210. That is the log's known one-send undercount
    (RS_12_15_clock_authority_2026-09-14/ERRATA.md §2). The same +1 shows on
    the other legs too: A2c 211/210, A2a 217/216, A2a2 218/217 (base
    brackets / `base sends on air`, `legs/leg<X>_report.txt` and
    `legs/leg<X>_gaps.txt`). The tractor's 211 against 209
    decoded commands is that unlogged send plus the late PROBE seq 210
    (Command delivery, above).

- **A stale `rx_smoke` container was removed.** The survey command removed
  a stale, exited `rx_smoke` container on the base. The first line of
  `legs/spot_check_2026-10-19.jsonl` is the name that `docker rm` printed.
- **Console noise.** Each harness transcript has 25 `RemoteException` /
  `NativeCommandError` blocks (`legs/leg<X>_harness.txt`). PowerShell 5.1
  wraps adb push progress ("… bytes in 0.001s") written on stderr. Cosmetic.
- **Low scene change at leg time.** The 10 s scene change was 11.51 %
  (A2c), 17.07 % (A2a), 9.53 % (A2c2) and 18.15 % (A2a2), against 57.01 % at
  preflight. All are above the 8 % minimum. A2c2's 1.9 s pair read 4.53 %.
  The two `mono_g4` legs drew the two quieter scenes
  (`legs/leg<X>_scene.txt`, `legs/preflight_moving_scene.txt`).
- **The dry run FAILs on the `mono_g4` captures, as expected** (`all_vector`
  and the VECTOR-only checks). A2c's `one_fragment` line also counts 2
  `mono_g4` frames: both are 243 B with K = 1 (rows 490 and 572), over the
  242 B limit the dry run applies to K = 1 (epoch-start) frames. P1 is n/a on 2c, and P4 (K = 1 on every TX)
  holds. Not investigated.
- **The prep files call these legs "Round A, 0 dB step".** That is
  `leg_prep.sh`'s tag label. No step chain was used.

**Correction (2026-10-20; the bench kit fixes on branch
`bench-kit-fixes-2026-10-19`, checked on the committed evidence, no
radio).** The text above stays as written; these points correct
how parts of it read.

- **The +1 base send is a PROBE sent after the harness read the base log,
  not the log's one-send undercount of ERRATA §2.** The harness that flew
  these legs read `docker logs` of tx_smoke (tractor) and then rx_smoke
  (base) while both daemons still ran, and stopped them afterwards; the post
  brackets were read after the stop. So the tractor's log ends first, the
  base's next, and both before the brackets. On every leg the base fired one
  more PROBE after its log had been read: base brackets / `base sends on air`
  A2c 211/210, A2a 217/216, A2c2 211/210, A2a2 218/217. The 2026-10-18 legs
  flew `-ReactiveFire 0` (the base sent only injector commands) and agree
  exactly: V 23/23, W 27/27, W2 72/72, X 36/36
  (`RS_12_15_reverse_delivery_2026-10-18/legs/leg<X>_{pre,post}_base.txt`,
  `leg<X>_gaps.txt`).
- **Both `mono_g4` misses reached the tractor's radio.** On A2c and A2c2 the
  tractor L072's leg `radio_rx_ok` (211) equals the base's Δ`radio_tx_ok`
  (211): the tractor's radio received every base send, both seq-210 PROBEs
  included, after the tractor's log had been read. At the radio, `mono_g4`
  delivery was 420/420 (on A2a and A2a2 too the tractor's `radio_rx_ok`, 217
  and 218, equals the base's Δ`radio_tx_ok`). The scored figure stays 418/420, since
  the rule scores the tractor's log, and the verdict, NOT WORSE, is
  unchanged. `rs13_probe_score.py` now prints a `tail:` line per leg:
  `[210]` on A2c and A2c2, `none` on A2a and A2a2.
- **The `received > sent` under Evidence limitations (A2c 612 > 611, A2a 609 > 607), like the P3 detail's counter `loss` line, is the
  same window.** The fixed `tools/rs12_leg_report.py` prints a
  `shared-window loss` line (ERRATA §3 method): 0/611, 0/607, 1/609 (0.2 %)
  and 1/612 (0.2 %) for A2c, A2a, A2c2 and A2a2. These are the same losses
  as the seq-gap figures (0/616, 0/612, 1/614, 1/616), which stay the loss
  of record. With `--capture`, the report's received > sent warning now
  names the capture seq-gap line first. Its FIRMWARE DROP line now subtracts
  the base's own count: 5, 4, 5 and 4 (A2c and A2a printed 6 because the
  received > sent clamp had replaced the base's count with the TX count).
  The remaining 4-5 is the log window plus the URCs the L072 lost
  (`rx_urc_lost` 1, 0, 0 and 2). The tractor brackets show the same: leg
  `radio_tx_ok` 617 / 613 / 615 / 617 against 611 / 607 / 609 / 612
  fragments in the TX logs. On A2c, A2a and A2a2 the tractor L072 restarted
  at leg start, so that figure is the post value; only A2c2's 615 is a
  post − pre delta. The report now prints a log-window note under FIRMWARE
  DROP on every archive without `log_window=after_stop`.
- **The leg reports in `legs/` are cut at 1024 B.** Six reports stop at
  exactly 1024 B as written: `legs/legA2c_report.txt`, `legA2a_report.txt`,
  `legA2c2_report.txt` and `legA2a2_report.txt` here, and `legW_report.txt`
  and `legX_report.txt` in `RS_12_15_reverse_delivery_2026-10-18/legs/`.
  `leg_post.sh` piped `rs12_leg_report.py | tee | head -8`, and head's exit
  killed tee. Re-running `rs12_leg_report.py` at main `36543a35` on each
  archive, with the committed brackets (and capture for the A2* legs), gives
  a full report whose first 1024 B are byte-identical to the committed file.
  Full sizes: A2c 1165, A2a 1081, A2c2 1046, A2a2 1046, W 1185, X 1119 B.
  Only the tail of the last `RS-12.15:` line is missing: ` | streak_max
  post=1` on A2c2 and A2a2, everything from `| tx_first_anchor` on for A2a,
  and on A2c, W and X part of the `consider_remote deltas` / `demotion
  reset` fields too. The loss lines (`loss`, `capture seq gaps`) and the
  radio deltas are intact; the gaps reports were not cut. `leg_replay.py`
  also crashed (TypeError, `dr.last_ts` None) after its first two lines on the
  `mono_g4` captures A2c and A2c2, hidden by `| head -1`; the p2p4
  `seq-gap loss` rows are that first line and are correct. `leg_post.sh` now
  writes each report whole.
- **`legs/legA2c2_deltas.txt` scored a DTS leg by FHSS rules.** It came from
  RS-12.15's `scripts/deltas.py`, which applies the FHSS criteria to every
  leg; its `[FAIL] tractor tx_first_anchor post = 0`, its UNVERIFIED rows and
  `STOP RULE: UNDETERMINED` are not results (as the A2c2 bullet above says).
  The kit's `bench_tools/bracket_deltas.py` scores A2c2, and A2c, A2a and
  A2a2, as DTS: deltas, reset evidence and cross-check only, exit 0. The
  deltas and A2c2's cross-check are the same (brackets 211/211 vs logs
  210/209, DISAGREE: the window above). `deltas.py` also judged
  `tx_first_anchor` by its raw post, which would trip STOP on the second
  FHSS leg after a missed tractor reset; `bracket_deltas.py` judges the
  leg's delta. The 2026-10-18 legs are not affected: it gives V, W, W2 and X
  the same numbers, all PASS. Where this file says to score a leg with
  `deltas.py`, read `bench_tools/bracket_deltas.py <legs> <leg>`.
- **The dry run's `one_fragment` count on A2c is a tool bug, not an
  oversize frame** (the "dry run FAILs on the `mono_g4` captures" bullet
  above). `vector_dry_run.py` applied VECTOR's 1-byte epoch-start trim
  (VS1's own cap, VECTOR_SCENE.md §3.1) to every K = 1 frame of any codec. A
  `mono_g4` key frame goes out under the default copies = 1 as one 0xFE
  fragment at the full 243 B, so rows 490 and 572 fit. Fixed: a replay of
  `legs/legA2c_base.jsonl` now reads `over limit: 0` and
  `[PASS] one_fragment`; no other row of the four replays changes. RS-13.1's
  leg2d_yt (row 254) and leg2d_r4 (rows 359 and 577) carried the same false
  flag.
- **Fixed in the same kit, none flown yet.** The A2c2 reset miss
  (`OCD_EXIT=129`, no openocd banner, BENCH_RUNBOOK candidate (c)): the
  harness now pushes `bench_tools/ocd_reset.sh`, which ignores HUP before it
  forks. The log window: the harness now stops the feed, drains
  (`-StopDrainS`, default 2 s), stops tx_smoke and then rx_smoke, and reads
  the logs afterwards (`log_window=after_stop` in params.txt). The "Console
  noise" bullet: every adb push is now checked and md5-compared, and the 25
  `RemoteException` blocks are gone. The "Round A, 0 dB step" prep label:
  `leg_prep.sh` now labels the A2* tags "Round A on the antennas (no
  attenuators since 2026-10-19)". The RF setup's R0 verdict:
  `rs13_rf_extract.py --path antenna` prints the level only (RSSI −70.0 dBm
  / SNR 6.0 dB over the four legs, as above).

## Bench kit (PR #158): first real-board runs

- **`scene_check.sh`:** every scene file has the new line: "first frame
  0.5 s after opening /dev/video1", the sixth frame 0.2 s and the last
  10.7 s after the first.
- **`step1_pass.sh` with the bw500 profile:** the boot log shows
  `byte_budget=243 B/frame (LIFETRAC_FRAGMENT_BUDGET, phy=image_bw500)` and
  no `VECTOR budget` clamp line
  (`legs/step1_bw500_moving_bootcheck.txt`,
  `legs/step1_camera_service_bw500_moving.log`). The exit-6 path had nothing
  to catch.
- **`leg_prep.sh` A2* tags:** all four legs were prepped from them
  (`legs/legA2*_prep.txt`).
- **The tractor reset verdict:** three `landed`, plus one `failed` with the
  right reason and the `deltas.py` hint.
- **Parks:** `PARK_OK` on the first attempt on both boards after every leg
  (`legs/leg<X>_park.txt`).
- **End-of-round check (19:19:48–56Z):** both radios read `0x80` SLEEP, no
  `/dev/ttymxc3` holders, no capture or leg-daemon container (tractor:
  `bench_mqtt`, `x8-devel`, `x8-provisioning`, `x8-webapp`; base:
  `design-controller-mosquitto-1`) (`legs/session_end_park_2026-10-19.txt`).
- **`-InjectCmd`:** not used.

## GO / NO-GO

The deciding rows:

- Command delivery (§8.6, DTS): PASS.
- P3: PASS.
- P1, P2, P4, P7 and P8: PASS on both VECTOR legs.
- A16: not exercised. There was no failure, and no confirmation.

Nothing re-opens an earlier finding. **GO** for the next items, each on its
own explicit GO:

1. the FHSS command-delivery and switch legs ("2d on profile 1", DC/TODO.md);
2. then a live web_ui leg.

The A16 on-air confirmation stays open. It needs loss-opened resync
episodes, so it needs a lossy link. With no attenuation equipment, that
means a distance test (top-level TODO.md, S-HW.2). The tractor SWD reset
race seen on A2c2 stays open in BENCH_RUNBOOK ("L072 resets at leg start").
Until it is fixed, score any leg whose reset fails with `deltas.py`, as
here.

## Files

- **`legs/`, session:** `staging_{base,tractor}.txt` (md5 records),
  `session_health_{base,tractor}.txt`, `preflight_moving_scene.{txt,jpg}` (the .jpg frames stay local: `*.jpg` is gitignored).
  The step-1 bw500 run is in `step1_*`: bench_mqtt reset, boot check, scene,
  `camera_service` log, capture `.jsonl/.json/.txt` and `tractor-log`. The
  spot-check is `spot_check_2026-10-19.jsonl` with its `_compare.txt` and
  `_times.txt`. The end-of-round radio check is
  `session_end_park_2026-10-19.txt`.
- **`legs/`, per leg** (`legA2c_`, `legA2a_`, `legA2c2_`, `legA2a2_`): `prep`,
  `scene.{txt,jpg}`, `health_{base,tractor}`, `clear_retained`,
  `pre_`/`post_{base,tractor}` brackets, `link_stats`, the base capture
  `base.{jsonl,json,txt}` (with the dry run), `harness` (console
  transcript), `params`, `report`, `p2p4`, `gaps`, `probe_score`, `rf`,
  `tractor_log`, `tractor_ocd_reset.log`, `park` and `post_transcript`.
  The VECTOR legs add `digest_trace` and `replay`; A2c2 adds `deltas`.
- **`legs/`, round:** `roundA_probe_score_pooled.txt` (§8.6),
  `roundA_p3.txt`, `roundA_digest_trace.txt` (A16), `roundA_rf_level.txt`
  (level, FRF readback) and `roundA_fold_emitter.txt`.
- **`scripts/`:** `fold_emitter.py` and `streak_check.py`, copied from
  RS-13.1. `streak_check.py` is an FHSS-authority check, and none of its
  output is in `legs/`.
- **Harness archives** (in `bench-evidence/`; each holds `params.txt`,
  `rx_daemon.log` (base), `tx_daemon.log` (tractor), `camera_service.log`
  and `tractor_ocd_reset.log`):
  - A2c: `radio_monitor_20261019_134805_1d10f57f`
  - A2a: `radio_monitor_20261019_135659_1d10f57f`
  - A2c2: `radio_monitor_20261019_140618_1d10f57f`
  - A2a2: `radio_monitor_20261019_141603_1d10f57f`
