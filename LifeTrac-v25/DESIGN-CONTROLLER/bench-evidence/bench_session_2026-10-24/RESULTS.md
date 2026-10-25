# Bench session 2026-10-24 — FHSS command delivery, VECTOR vs mono_g4

**Status: flown and scored 2026-10-24. GO given 2026-10-24 about 14:40Z: "Okay ready for a GO? Please proceed when
ready. GO", in reply to this file's proposed scope and pass rule, which
therefore flew as written below (pass rule fixed before leg 1). The operator
made no separate RF-setup statement: leg 1's level is recorded as the FHSS
baseline, and nothing was moved on the bench by this session.**

**Verdict, by the pass rule written before leg 1: on FHSS (profile 1) at bench
distance, VECTOR's command delivery is NOT WORSE than `mono_g4`'s.** VECTOR
383/384 = 99.7 %, `mono_g4` 542/543 = 99.8 %, difference −0.08 pp, one-sided
95 % lower bound (Newcombe hybrid score) −0.99 pp, above −3 pp
(`legs/round_probe_score_pooled.txt`). Leg 6 (2b_f3, VECTOR) is in that pool,
as the rule says. It fired only 5 PROBEs: the base's cold-start FHSS scan never
locked, and the base heard 5 of the tractor's 606 frames (Anomalies). So the
VECTOR arm has 384 probes, not the ~424 per arm the power note named for
97 % / 97 %. Without leg 6,
for information only: 379/379 against 542/543, +0.18 pp, bound −0.54 pp.
Leg 4 gave the first loss-opened A16 episode scored on air: 1 PASS, thin. This
was the first flight of the #161 kit; every first-flight check passed (the
end-of-round radio check, re-read at 17:05Z, found both radios at 0x80).

## What was checked (no radio activity)

| item | result |
|---|---|
| merges since the 10-19 legs | #156 (D-PWR1 idle-reboot supervisor, 10-22): docs, `firmware/x8_recovery/` and SIL tests; the supervisor is installed on neither board. #162 (H7-4, tractor control over Method G, 10-22): `firmware/tractor_h7` (the H747 sketch, not flashed on the bench), its SIL, CI and docs. #161 (this bench kit, 10-24, `d28bc2ca`) |
| old CI failure | `LifeTrac v25 Controller CI` on #156's branch at `f6b4cd8a` (10-22 18:00Z): a supervisor SIL timing flake (`test_tractor_forced_path_giving_up_withdraws_when_nothing_was_issued`), fixed in `e0a61aa4` before #156 merged; main passed after every merge. The two preview workflows fail on push since 2026-10-18 (HTTP server / commit-and-push steps), unrelated to the radio |
| kit | main `d28bc2ca`: #161 merged on top of #156 and #162. Merged tree on this PC: base_station 1807 passed, 10 failed (the Windows-only watchdog cases, as on main), harness offline 167/167, no TODO / DECISIONS / VECTOR_SCENE citation moved (session console, not archived) |
| flight worktree | `LifeTrac-bench-main` on branch `bench-session-2026-10-24` = `d28bc2ca`; `bench.env` kept (git-ignored); board scripts LF |
| boards | both on adb, up about 6 days (no reboot since the 2026-10-18 power cycle); tractor clock reads 2026-10-05 (no time sync, known) |
| radio UART | no holder of `/dev/ttymxc3` on either board; leg daemons exited |
| brokers | base `design-controller-mosquitto-1` up; tractor `bench_mqtt` up (the harness recreates it per leg) |
| images | base `lifetrac-v25:latest` `0413f3c4dac3`, tractor `lifetrac-tractor-x8:latest` `b2a3c8070240`: unchanged since 10-19 (`docker images` on both boards at 14:23Z; session console, not archived) |
| L072 firmware | bench `e083ae46` on both since 2026-10-18 |
| staging | `stage_boards.sh` 14:26-14:27Z from `d28bc2ca`: tractor 16 + 37 files, base 15 + 37 files, all md5-verified (`legs/staging_{tractor,base}.txt`); 24 entries in `/tmp/lifetrac_strict` on each board. It ages out after 5 days |
| radios | `power_up_guard.sh` 14:27:43Z / 14:27:53Z: both `RADIO_STATE 0x80 SLEEP`, nothing to park (`power_up_guard_2026-10-24.log` in `BENCH_SCRATCH`; copy in `legs/`) |
| `bench.env` | `EVIDENCE_DIR` = `bench-evidence/bench_session_2026-10-24/legs` (new, untracked). `DTS_CARRIER_*` still reads 927.5 MHz / 2026-10-19: stale, so DTS legs are refused until a same-day spot-check (FHSS legs need none) |

## Still needs the operator (asked before the GO; answered below)

1. **The GO** for the round: which legs, their order, and the stop time.
2. **RF setup:** are both antennas, cables and board positions unchanged
   since 2026-10-19 (about −70 dBm / SNR 6 dB at 927.5 MHz then)? A photo
   for `legs/rf_setup_2026-10-24.jpg` if anything moved.
3. **The pass rule** below, before leg 1 (proposed, not yet agreed).
4. **The scene:** the railroad video in a normal Firefox window with the
   player fullscreen, never a kiosk; start it a minute early.

How each was answered: (1) the GO at about 14:40Z, for the scope and order
below; no stop time is recorded here. (2) No RF-setup statement and no photo
(status line, RF setup below). (3) The pass rule, agreed with the GO. (4) The
window opened at 14:46:29Z (session console, not archived), but leg 1's
scene check ran 29 s later, not a minute: a session error (Scene notes).

## Scope (flown as proposed)

FHSS command delivery, VECTOR against `mono_g4`, interleaved: profile 1
(`image_bw250`), `-ReactiveFire 1 -ProbeEcho 0 -TxBatch 0`, no
`-ForceFrfHz`, `-StopDrainS 4` (`leg_prep.sh`'s FHSS default), 300 s legs.
Order: mono_g4, VECTOR, mono_g4, VECTOR, mono_g4, VECTOR. No spot-check is
needed for FHSS.

Not in this scope:

- the FHSS switch leg (the P6 bound and the A17/A18 sign-offs are open);
- the web_ui leg (it needs its own plan);
- the A16 confirmation (it needs a lossy link, i.e. a distance test).

This is the first flight of the #161 kit. On leg 1, also check the first-leg
list in `legs/README.md`:

- `LOG_WINDOW after_stop` (green);
- the tractor reset landed;
- `deploy_md5` all ok;
- both clock offsets are numeric;
- every base command is 0x69;
- after the leg-end park, wait about 2 min, then `power_up_guard.sh --check-only`.

## Pass rule (written before leg 1)

| item | proposed value |
|---|---|
| metric | PROBE delivery per codec: tractor-received PROBE seqs / base PROBE sends on air (`rs13_probe_score.py`), pooled over the legs below |
| rule | VECTOR is NOT WORSE when the one-sided 95 % lower bound (Newcombe hybrid score) of VECTOR minus `mono_g4` is above −3 pp: `py -3 rs13_probe_score.py --margin 3 <the six archives>` |
| legs per codec | 3, interleaved as above |
| probes per leg | about 200 expected (a 300 s DTS leg fired about 210 on 2026-10-19; the FHSS figure is unmeasured: read it on leg 1). At 97 % / 97 % the 3 pp score bound needs about 424 per arm for 80 % power; if leg 1 fires under about 150, add a leg pair before scoring |
| `-StopDrainS` | 4 s on every leg |
| `log_window` | `after_stop` on every leg; a leg that reads `after_stop_unverified` is re-flown |
| the old rule | the two-sided Fisher line still prints, for comparison with 10-19, but is not the pass rule: it passes when the sample is too small |
| agreed | 2026-10-24 about 14:40Z, the operator's GO for this proposal; fixed before leg 1 |

## RF setup

Both boards on their antennas at bench distance, as on 2026-10-18 and
2026-10-19. No statement from the operator; this session moved nothing. FHSS
on the 50-channel wide mask (`FHSS mask: WIDE (50 channels)` in every
`rx_daemon.log`), seed (0,0), no `-ForceFrfHz` (`force_frf_hz=0` in every
`params.txt`). Received level at the base on legs 1-5: median of the window
medians −64 dBm, SNR 10 dB (window minimums −73 dBm and 5-8 dB;
`legs/leg<X>_rf.txt`). Leg 6: −62 dBm / 10 dB over its 5 frames. This is the
FHSS baseline for this bench. The tractor's receive level is not logged.

## Flight log

Scene: `youtube_window.ps1 -Open` 14:46:29Z (normal window, player fullscreen; Esc leaves it; session console).
Tags: mono_g4 legs `2c_f1`, `2c_f2`, `2c_f3`; VECTOR legs `2b_f1`, `2b_f2`, `2b_f3`; every leg
`leg_prep.sh <tag> image_bw250 --reactive-fire 1` (2 fps, 300 s, `-StopDrainS 4`).

| # | leg | codec | prep | harness archive | LOG_WINDOW | reset | park | notes |
|---|---|---|---|---|---|---|---|---|
| 1 | 2c_f1 | mono_g4 | 14:47:28Z, scene 49.6 % (the kept frame is a pharma ad, not the railroad video; corrected) | `radio_monitor_20261024_095315_d28bc2ca` | after_stop (verified) | landed | PARK_OK x2 14:56Z | PROBE 161/161; brackets AGREE; 613 sent K=1, 475 published, seq-gap 0 after lock (first frame seq 139, A7 head 138); FIRMWARE DROP 0; -64 dBm / SNR 10 dB; deploy md5 all ok (tractor 54 files = 52 + the 2 reset files, base 52; corrected); clocks numeric |
| 2 | 2b_f1 | VECTOR | 14:57:05Z, scene 62.5 % (on a cut: 1.9 s change 9.9 %; the frame is a talking-heads clip with a YouTube Premium card; corrected) | `radio_monitor_20261024_100249_d28bc2ca` | after_stop (verified) | landed | PARK_OK x2 15:05Z | PROBE 191/191; brackets AGREE; 615 sent K=1, 558 published, seq-gap 1/559 = 0.2 % (first frame seq 57, A7 head 56); FIRMWARE DROP 0; -64 dBm / SNR 10 dB; strict capture RESULT FAIL (64 orphans, 10/465 digest mismatches, all in the one join resync #0-#13 ended by DIGEST; final digest_ok True) |
| 3 | 2c_f2 | mono_g4 | 15:07:59Z, scene 58.1 % (first try 15:06Z refused at 7.9 %: a mid-roll ad; window to the front, retried; the retry's frame shows the same talking-heads clip; corrected) | `radio_monitor_20261024_101341_d28bc2ca` | after_stop (verified) | landed | PARK_OK x2 15:16Z | PROBE 179/179; brackets AGREE; 613 sent K=1, 523 published, seq-gap 0 after lock (first frame seq 91); FIRMWARE DROP (drx_ok - host URCs): 523 - 523 = 0 |
| 4 | 2b_f2 | VECTOR | 15:17:08Z, scene 57.2 % (on a cut: 1.9 s change 6.3 %; talking-heads frame; corrected) | `radio_monitor_20261024_102247_d28bc2ca` | after_stop (verified) | landed | PARK_OK x2 15:25Z | PROBE 188/188; brackets AGREE; 609 sent K=1, 552 published, seq-gap 1/553 = 0.2 % (first frame seq 57); FIRMWARE DROP (drx_ok - host URCs): 552 - 552 = 0; strict capture RESULT FAIL (46 orphans, 12/420 digest mismatches: join resync #3-#13 and a loss-opened resync #437-#441, both ended by DIGEST; final digest_ok True); **A16: 1 loss-opened episode, isolated, scored PASS (thin: < 3)** |
| 5 | 2c_f3 | mono_g4 | 15:26:15Z, scene 8.95 % (the mid-roll ad again: talking heads; the window title stayed on the railroad video) | `radio_monitor_20261024_103156_d28bc2ca` | after_stop (verified) | landed | PARK_OK x2 15:34Z | PROBE 202/203 (one mid-leg miss at the tractor radio; not a tail); brackets AGREE 202/203; 610 sent K=1, 590 published, seq-gap 2/592 = 0.3 % (first frame seq 19); FIRMWARE DROP (drx_ok - host URCs): 590 - 590 = 0 |
| 6 | 2b_f3 | VECTOR | 15:35:19Z, scene 61.05 % (on a cut: 1.9 s change 12.27 %; talking-heads frame) | `radio_monitor_20261024_104056_d28bc2ca` | after_stop (verified) | landed | PARK_OK x2 15:43Z | PROBE 4/5 (seq 3 undelivered; not a tail); brackets AGREE 4/5; 606 sent K=1, **5 published: the base's FHSS scan never locked** (Anomalies); first frame heard = the tractor's frame #264 (u8 seq 8 after one wrap), 138.0 s after the base RX worker was ready; seq-gap 226/231 = 97.8 % after it (`leg_replay`; the report's 91/96 is a tool error); FIRMWARE DROP 5 - 5 = 0; -62 dBm / SNR 10 dB (5 frames); post brackets read radio_state 4 on both boards (2 on legs 1-5); strict capture RESULT FAIL (5 frames, the join never closed, digest_ok False) |

Corrections made after the round (2026-10-24, from a re-score of the raw logs
and a skeptic's pass over it; no radio):

- **Scene cells of legs 1, 2, 3 and 4.** The frame each scene check kept is
  not the railroad video. Leg 1's (`legs/leg2c_f1_scene.jpg`) is a pharma ad.
  Legs 2-6 show a talking-heads clip with a YouTube Premium card. The flight
  log had noted an ad only for leg 3's first try and leg 5 (Scene notes).
- **Row 1, "deploy md5 52/52 both".** That read as 52 files on each board.
  The tractor checked 54 (52 plus the 2 reset files), all ok.
- **Row 6 is new.** Its first frame is the tractor's frame #264, not #8: the
  u8 frame seq had wrapped once. The leg report's "first frame heard at seq 8
  … counting from seq 1: 98/103 = 95.1 %" and its "capture seq gaps … lost
  91/96" are wrong for that reason (Anomalies, "Tool errors on leg 6"). The
  true whole-leg loss is 601/606 = 99.2 %.
- **No row said the base "held FHSS lock only briefly".** The counters say
  neither board's scan was ever LOCKED on leg 6 (Anomalies). The row above
  says "never locked".
- The title read "prep for the next radio round" and the status "in progress"
  until the round was scored.

## Results per leg

| # | leg | codec | PROBE delivered / sent | frames sent (all K = 1) / published | first frame heard: tractor frame #, s after base RX ready | seq-gap loss after it | gaps > 3 s (max gap) | FIRMWARE DROP / Δ`rx_urc_lost` | RSSI / SNR (median of window medians) | base Δ aligned / snapped / demotion reset | post `radio_state` base, tractor |
|---|---|---|---|---|---|---|---|---|---|---|---|
| 1 | 2c_f1 | mono_g4 | 161/161 | 613 / 475 | #139, 75.0 s | 0/475 | 0 (0.7 s) | 0 / 2 | −64 dBm / 10 dB | 474 / 1 / 1 | 2, 2 |
| 2 | 2b_f1 | VECTOR | 191/191 | 615 / 558 | #57, 34.4 s | 1/559 (0.2 %) | 0 (1.0 s) | 0 / 1 | −64 / 10 | 557 / 1 / 1 | 2, 2 |
| 3 | 2c_f2 | mono_g4 | 179/179 | 613 / 523 | #91, 51.4 s | 0/523 | 0 (0.7 s) | 0 / 0 | −64 / 10 | 522 / 1 / 1 | 2, 2 |
| 4 | 2b_f2 | VECTOR | 188/188 | 609 / 552 | #57, 34.4 s | 1/553 (0.2 %) | 0 (1.0 s) | 0 / 3 | −64 / 10 | 551 / 1 / 1 | 2, 2 |
| 5 | 2c_f3 | mono_g4 | 202/203 | 610 / 590 | #19, 15.2 s | 2/592 (0.3 %) | 0 (1.1 s) | 0 / 8 | −64 / 10 | 589 / 1 / 1 | 2, 2 |
| 6 | 2b_f3 | VECTOR | 4/5 | 606 / 5 | #264, 138.0 s | 226/231 (97.8 %) | 4 (68.1 s), counted from the first frame heard | 0 / 1 | −62 / 10 (5 frames) | 0 / 3 / 0 | 4, 4 |

Sources: PROBE `legs/leg<X>_probe_score.txt`; frames and K `legs/leg<X>_p2p4.txt`;
seq-gap `legs/leg<X>_p2p4.txt` and `legs/leg<X>_replay.txt` (`leg_replay`);
gaps `legs/leg<X>_gaps.txt`; FIRMWARE DROP and `rx_urc_lost` `legs/leg<X>_report.txt`;
level `legs/leg<X>_rf.txt`; counters `legs/leg<X>_deltas.txt` and
`legs/leg<X>_post_{base,tractor}.txt`. The first-frame column maps the base's
first `frag_arrival` to the tractor's `txdone_arrival` by time, with the
params.txt clock offsets (19 ms median TX-to-RX latency on leg 1). On legs
1-5 the first frame is the frame of that seq in the first u8 cycle; on leg 6
it is the second seq 8 (tractor txdone 08:29:19.662 board time = 15:37:56.51Z;
base `frag_arrival` 15:37:56.54Z, `rx_daemon.log:67`).

Also per leg:

- **Whole-log loss** (the acquisition head plus the seq-gap losses above:
  138+0, 56+1, 90+0, 56+1, 18+2 on legs 1-5): 138/613, 57/615, 90/613,
  57/609, 20/610 and 601/606, the same in the
  `shared-window loss` line (`legs/leg<X>_report.txt`). The base needs time to
  find the tractor's hop grid after its profile activation; that cost is the
  head.
- **Tractor, every leg:** a `txdone_arrival status=0` for every frame, 0
  ABORTED, `drop_full=0 drop_stale=0`, `tx_first_anchor` 1,
  `clk_demotion_reset` 0, `fhss_dec_snapped` 0, `fhss_dec_rej_epoch_drift` 0.
  The RS-12.15 bracket criteria PASS and the STOP rule is not tripped on all
  six (`legs/leg<X>_deltas.txt`).
- **Codec in `link_stats`** (one sample per leg): `mono_g4`, `vector`,
  `mono_g4`, `vector`, `mono_g4`, `vector` (`legs/leg<X>_link_stats.txt`).
- **VECTOR encoder** (for information; P8 is not in this round's rule): p95
  221.5 / 209.2 / 229.2 ms, frame bytes p50 200 / 200 / 199, max 203 B
  (`legs/leg2b_f*_tractor_log.txt`).
- **Strict VECTOR captures** print `RESULT: FAIL` on all three (`no_orphans`
  and `digest`; leg 6 also `epoch_seen` and `min_frames`). On legs 2 and 4 the
  mismatches sit in the join resync, plus leg 4's loss-opened episode, and the
  final `digest_ok` is True. Leg 6's 5 frames never left the join
  (`legs/leg2b_f*_base.txt`, `legs/leg2b_f*_replay.txt`).

Seq-gap loss after the first frame is the loss of record on an RS-13 leg, but
it is not in this round's pass rule. Pooled for information, legs 1-5 only:
VECTOR 2/1112, `mono_g4` 2/1590. Leg 6 is not a loss figure: its receiver
never followed the hop grid.

## Command delivery (the pass rule)

| leg | codec | delivered / sent | tail |
|---|---|---|---|
| 2c_f1 | mono_g4 | 161/161 | none |
| 2b_f1 | VECTOR | 191/191 | none |
| 2c_f2 | mono_g4 | 179/179 | none |
| 2b_f2 | VECTOR | 188/188 | none |
| 2c_f3 | mono_g4 | 202/203 (seq 115, mid-leg) | none |
| 2b_f3 | VECTOR | 4/5 (seq 3, mid-leg) | none |

`py -3 rs13_probe_score.py --margin 3` over the six archives, exit 0
(`legs/round_probe_score_pooled.txt`):

```
pooled FHSS VECTOR 383/384 = 99.7 %; pooled FHSS mono_g4 542/543 = 99.8 %
two-sided Fisher exact on delivered / undelivered: p = 1
margin rule, 3 pp: VECTOR minus mono_g4 delivery -0.08 pp, one-sided 95 % lower bound -0.99 pp (Newcombe hybrid score interval, method 10)
command delivery, FHSS legs (...), margin 3 pp: NOT WORSE (margin rule): the lower bound -0.99 pp is above -3 pp
```

**PASS: NOT WORSE (margin rule), as the pre-written rule gives it.** The
Fisher line (p = 1) prints for comparison only.

- **Independent check.** The re-score recounted every leg from the raw
  daemon logs and computed the bound with its own implementation (exact
  rationals for Fisher, Newcombe method 10 checked against Newcombe 1998
  Table II). It gives the same counts and −0.986 pp (two-sided 90 %
  interval −0.986 to +0.592 pp).
- **Leg 6 excluded, for information only** (`legs/round_probe_score_excl_leg6.txt`):
  VECTOR 379/379 = 100.0 % against 542/543, +0.18 pp, bound −0.54 pp, NOT
  WORSE. The rule as written includes leg 6, so this figure is not the
  verdict. Dropping a leg after seeing its data would be a choice made after
  the numbers.
- **Power.** The VECTOR arm has 384 probes against the ~424 the power note
  named. The rule's only extra-leg clause (leg 1 under ~150) did not apply:
  leg 1 fired 161. Full legs fired 161-203 probes.
- **What the metric cannot see.** With `-ReactiveFire 1` the base fires a
  PROBE only after it receives a frame. Leg 6's dead forward link therefore
  removed probes (5 instead of about 190) rather than scoring them as
  undelivered. Had the base caught no frame at all, `rs13_probe_score.py`
  would have exited 1 and withheld the pooled verdict ("Exit 1 when a leg
  fired no probe", its docstring). Whether such a leg is counted or re-flown
  is left to chance by the current rule.
- **The two misses.** Leg 5 seq 115 (base TX 15:29:37.297 base clock,
  `radio_monitor_20261024_103156_d28bc2ca/rx_daemon.log:2329`; the tractor
  logged 114 and 116 as rx#114 and rx#115, `tx_daemon.log:1703`, `:1729`)
  and leg 6 seq 3 (base TX 15:38:36.566, `rx_daemon.log:123`). Neither is a
  tail. The tractor's `radio_crc_err` is 1 on exactly these two legs and 0 on
  the four at 100 % (`legs/leg<X>_post_tractor.txt`). That fits a reception
  that reached the tractor's radio and failed its CRC. Unverified.
- Pooling treats probes as independent.

## First flight of the #161 kit

Every leg of the round was flown on it. The checks from `legs/README.md` and
DC/TODO.md ("First flown leg on the 2026-10-19 kit"):

| check | result | evidence |
|---|---|---|
| `LOG_WINDOW after_stop`, both stops confirmed, both SIGTERM shutdown lines | PASS 6/6 | `log_window=after_stop`, no `log_window_detail`, `feed_stop=camera_svc stopped`, `stop_drain_s=4` in every `params.txt`. Every transcript prints `LOG_WINDOW after_stop (feed: camera_svc stopped; 4 s drain, tx_smoke stopped, rx_smoke stopped 0.5 s later, then the logs were read; both stops ok, both daemon logs hold their SIGTERM shutdown line)`. That text comes only from the verified branch; the colour itself is not in the tee. Both daemon logs contain `signal received; shutting down` (a few exit lines follow it) |
| `TRACTOR_L072_RESET` landed, openocd banner, `OCD_EXIT=0` | PASS 6/6 | `legs/leg<X>_tractor_ocd_reset.log` (byte-identical to the archive copies): own `OCD_LEG=` tag equal to params `tractor_ocd_tag` (144746Z, 145718Z, 150812Z, 151720Z, 152629Z, 153531Z), `Open On-Chip Debugger 0.11.0-dirty`, PF_4 low 0 then high 1, `OCD_EXIT=0`. No 129 / SIGHUP: `ocd_reset.sh` held. Not a first leg after a boot (boards up about 6 days), so the first-leg-miss question stays open |
| `deploy_md5` all ok, no `RemoteException` / `NativeCommandError` | PASS 6/6 | each `deploy_md5.txt` 106 lines (tractor 54, base 52), all `ok`, pc md5 = board md5; `[DEPLOY <serial>] 12 pushes OK, md5 52/52 match` for both serials; 0 `RemoteException` / `NativeCommandError` in every transcript (the 10-19 transcripts had 25 each) |
| both clock offsets numeric | PASS 6/6 | tractor −1667317.007 to −1667316.851 s (−19.30 d), base −0.848 to −0.869 s, rtt 0.060-0.074 s; the `CLOCKS` line in LEG CHECKS matches |
| every base command 0x69 | PASS 6/6 | only `command TX opcode=0x69 copy=1/1 OK (on air)`: 161, 191, 179, 188, 203, 5; no failed attempt, no `!!` line in any `legs/leg<X>_probe_score.txt`; the tractor logged only `LoRa cmd: PROBE` |
| base brackets = base sends on air; received ≤ sent | PASS 6/6 | base Δ`radio_tx_ok` 161/191/179/188/203/5 = the on-air counts; tractor Δ`radio_rx_ok` 161/191/179/188/202/4; base Δ`radio_rx_ok` ≤ tractor txdone on every leg; every cross-check AGREE (`legs/leg<X>_deltas.txt`) |
| no PROBE tail at a leg end (`-StopDrainS 4`) | PASS 6/6 | `tail: none` on every leg: the tractor logged the base's last PROBE |
| FIRMWARE DROP falls to about the `rx_urc_lost` level | held, at 0 | FIRMWARE DROP 0 on all six; Δ`rx_urc_lost` 2/1/0/3/8/1. The 4-5 on the 10-18/19 legs was the old log window. `rx_urc_lost` counts DIO0 edges never serviced, which never reach `radio_rx_ok`, so FIRMWARE DROP cannot see them. Leg 5's 8 is above the 0-2 seen on 10-18/19; not investigated |
| leg reports whole, `scorer exit codes:` line | PASS 6/6 | `legs/leg<X>_report.txt` 1463/1368/1457/1368/1454/1347 B, complete; every `legs/leg<X>_post_transcript.txt` has `scorer exit codes:` with all codes 0 (7 scorers on `mono_g4` legs, 8 on VECTOR legs) |
| leg-end park, then `power_up_guard.sh --check-only` ≥ ~2 min after the daemons stop | PASS (re-checked 17:05Z) | `PARK_OK` on attempt 1 on both boards after every leg, readback and after-settle 0x80 (`legs/leg<X>_park.txt`). Check-only 15:44:09-18Z: both `RADIO_STATE 0x80 SLEEP`, no `/dev/ttymxc3` holders, no leg containers, rc 0 (`legs/power_up_guard_2026-10-24.log`). That is 3 min 19-24 s after leg 6's daemons stopped (15:40:54Z) but only 45-50 s after its park (15:43:28Z). Leg 6's post brackets (15:43:21Z) read `radio_state` 4 (RX_CONT) on both boards, not the usual 2, so the "scan already failed out" premise of the ~2 min rule did not hold in its usual form. From code reading no arm path runs in scan FAILED once the tractor's authority has lapsed, so 0x80 should hold. The later read-only re-check (`power_up_guard.sh --check-only`, 17:04:58-17:05:06Z, 80 min after the last park) read `0x80 SLEEP` on both boards, no holders, no leg containers (`legs/power_up_guard_2026-10-24.log`) |

Two more notes on the kit:

- `leg_prep.sh` wrote `-StopDrainS 4` on every FHSS leg, and
  `rs13_probe_score.py --margin 3` gave the pooled verdict above on its first
  real use.
- Leg 1's harness transcript (the console tee) never got an `[exited with
  code N]` line. The PC host broker (amqtt) that leg 1's harness started kept
  writing into it until 15:40:54Z, including 6 `IncompleteReadError`
  tracebacks: one at leg 1's own [NET] warm-up (14:47:45Z) and one per
  later leg's. Those console tees are in the session's task output; the
  `legs/leg<X>_harness.txt` copies end at the archive line and hold neither
  the exit lines nor the tracebacks. That is PC broker noise, not
  a harness error block. Leg 1's exit code is unverified; its archive is
  complete (`=== LIVE LORA RADIO MONITOR COMPLETE ===`). Legs 2-6 end
  `[exited with code 0]`.

## A16 (by-product; not in this round's scope)

| leg | loss at row | episode (start-end, frames, s) | ended by | back in step (`rs13_digest_trace.py`, frames after the loss) | isolated? | pass? |
|---|---|---|---|---|---|---|
| 2b_f1 | – (its one seq gap, row 75, opened no episode) | join #0-#13 (13, 6.6 s), not scored | DIGEST | – | – | – |
| 2b_f2 | 435 (1 frame) | #437-#441 (4, 2.0 s); the join #3-#13 is not scored | DIGEST | #439 (+4) | yes | PASS |
| 2b_f3 | – | join #0-end (5 frames, open), not scored | open | – | – | – |

`rs13_digest_trace.py` over the three VECTOR captures: 4 episodes, 1
loss-opened, "THIN: fewer than 3", isolated and scored 1: 1 PASS, 0 FAIL
(`legs/round_digest_trace.txt`; per leg `legs/leg2b_f*_digest_trace.txt`).
This is the first loss-opened A16 episode scored on air; 2026-10-19 had none.
In episode #437-#441 the DIGEST mismatched for rows 435-438 and read ok from
row 439, before the next epoch start (row 469). One episode is not a
confirmation. A16 stays open and thin; it still needs a lossy link.

## Scene notes

- `youtube_window.ps1 -Open` ran at 14:46:29Z and leg 1's scene check at
  14:46:58Z, 29 s later. `legs/README.md` asks for a minute or more, because
  pre-roll ads play first. Leg 1's kept frame is an ad.
- Every scene check passed the written 8 % rule over 10 s, so no leg is void:
  49.6, 62.5, 58.1, 57.2, 8.95 and 61.05 % (`legs/leg<X>_scene.txt`). Leg 3's
  first try, 15:05:52Z, read 7.88 % and was refused before any health probe
  or receiver wake (`legs/leg2c_f2_prep.txt`), so it woke no radio.
- No kept frame shows the railroad video. Leg 1: a pharma ad. Legs 2-6: a
  talking-heads clip with a YouTube Premium card (`legs/leg<X>_scene.jpg`;
  legs 1 and 4 looked at again for this record). The window title stayed
  `Front view of Colorado's STUNNING Royal Gorge Route Railroad`
  (`legs/leg<X>_prep.txt`).
- Legs 2, 4 and 6 passed on a cut: 10 s change 62.5 / 57.2 / 61.05 %, but
  1.9 s change 9.9 / 6.3 / 12.3 %. Leg 5 passed at 8.95 % (1.9 s: 11.98 %).
- What played during each leg is unverified: the check runs before the leg,
  not during it. Both codecs drew the same kind of content, interleaved. It
  bears on frame sizes and encoder load, not on FHSS acquisition.

## Anomalies for the record

### Leg 6 (2b_f3): the base never locked the FHSS grid

**What was observed.**

- The tractor sent normally: 606 `txdone_arrival status=0` lines, K = 1, 0
  ABORTED, `frames_in=606 ok=606 fail=0` (`tx_daemon.log:2545`),
  `tx_stream_streak_max` 606, `tx_first_anchor` 1, `clk_demotion_reset` 0.
  Its grid was never renumbered.
- The base heard 5 frames: the tractor's #264, #293, #346, #358 and #494 (u8
  seq 8, 37, 90, 102, 238; `rx_daemon.log:67, 93, 119, 139, 195`). Each one
  matches a tractor `txdone_arrival` within 4-31 ms on the params clock
  offsets. Level −62 to −71 dBm, SNR 10 dB on all five.
- The base radio logged nothing else except one unserviced DIO0 edge:
  Δ`radio_dio0` 10 = 5 RX_DONE + 5 TX_DONE, Δ`radio_crc_err` 0, no
  `crc_dump` (`legs/leg2b_f3_report.txt`), and Δ`rx_urc_lost` 1, time
  unknown (legs 1-5 show 0-8).
- Neither board's scan state machine was ever LOCKED. A demotion is counted
  on every LOCKED → SCANNING edge (`sx1276_rx.c:786-796`,
  `host_stats.c:214-220`). Base Δ`clk_demotion_reset` 0 (1 on each of legs
  1-5: the end-of-leg demotion); base Δ`fhss_dec_aligned` 0 (rx_ok − 1 on
  legs 1-5); Δ`fhss_dec_snapped` 3; Δ`fhss_dec_rej_epoch_drift` 2, the first
  since the base booted. Tractor `clk_demotion_kept` 0 (1-3 on legs 1-5)
  (`legs/leg2b_f3_deltas.txt`).
- Post brackets read `radio_state` 4 (RX_CONT) on both boards; 2 (STANDBY)
  on legs 1-5 (`legs/leg2b_f3_post_{base,tractor}.txt`).
- The base's PROBE TX channels were 44, 6, 6, 18, 18 (`RFCO_PERTX`,
  `rx_daemon.log:73, 99, 125, 145, 201`). A hopping receiver would not repeat
  like that.
- The tractor's own scan logged FAULT 0x0D sub 0x20 (attempt 0, hw_suspect)
  at L072 uptime 34.0 s and sub 0x02 (attempt 2) at 94.0 s
  (`tx_daemon.log:237`, `:729`; uptime from payload bytes 20-23). That is the
  30 s attempt cadence. The base's own scan faults are not logged:
  `image_rx_daemon.py` drops FAULT URCs at DEBUG (`:1337-1340`).

**Timeline** (PC UTC. PC = base + 0.869 s, PC = tractor + 1667316.851 s, from
params.txt, read after the leg, error about ±0.03 s. "(model)" = inferred from
the walker model and the code, not logged.)

| PC time | board time | event | source |
|---|---|---|---|
| 15:35:10Z / 15:35:12Z | – | pre-brackets: base `radio_state` 1, aligned 7802, snapped 9, epoch_drift 0, `clk_demotion_reset` 9 | `legs/leg2b_f3_pre_{base,tractor}.txt` |
| 15:35:31Z | – | tractor L072 NRST through openocd, landed | `legs/leg2b_f3_tractor_ocd_reset.log` |
| 15:35:37.03 | tractor 08:27:00.181 | tx_daemon opens the HostLink | `tx_daemon.log:1` |
| 15:35:37.69 | base 15:35:36.816 | rx_daemon opens the HostLink | `rx_daemon.log:1` |
| 15:35:37.86 | tractor 08:27:01.013 | tractor profile 1 active (LBT line); its scan attempt 0 starts | `tx_daemon.log:41` |
| 15:35:38.49 | base 15:35:37.624 | base `REG_PROFILE=1 OK` / LBT line. Activation resets the base scan state machine (`host_cfg_profile.c:214-223`); walker attempt 0 starts at ch 0 | `rx_daemon.log:29-31` |
| 15:35:38.55 | base 15:35:37.678 / .679 | `SX1276 already in RXCONT`; RX worker ready (the 138.0 s reference) | `rx_daemon.log:35`, `:37` |
| 15:35:43.16 | tractor 08:27:06.31 (fitted) | tractor grid slot 0: a pre-frame TX at (epoch 0, hop 0) ch 3, ToA 125.6 ms | `tx_daemon.log:47` |
| 15:35:44.98 | tractor 08:27:08.131 | first image frame's txdone (#1) | `tx_daemon.log:49` |
| 15:36:07.76 | tractor 08:27:30.912 | tractor scan attempt 0 FAIL (0x0D sub 0x20) | `tx_daemon.log:237` |
| about 15:36:08.5, 15:36:38.5, 15:37:08.5 | – | base attempts 0, 1, 2 fail (model) | – |
| 15:37:07.61 | tractor 08:28:30.758 | tractor attempt 2 FAIL (0x0D sub 0x02) | `tx_daemon.log:729` |
| about 15:37:38.5 | – | base final FAIL (+120 s): scan FAILED, which is absorbing (`sx1276_rx_scan_policy.c:153`); the modem to standby (`sx1276_rx.c:665-685`) on the walker's last channel, ch 9 (dwell 60 = index 59 mod 50). The daemon's 5 s `_ensure_rxcont` re-arms RXCONT there without a retune (`image_rx_daemon.py:1312-1316` at `d28bc2ca`) (model and code) | – |
| 15:37:56.54 | base 15:37:55.675 | catch 1: #264 (seq 8), slot (13,17) on the fitted grid, ch 9; −62 dBm / 10 dB. PROBE 1 at (13,18) ch 44 | `rx_daemon.log:67-73` |
| 15:37:56.77 | tractor 08:29:19.918 | tractor hears PROBE 1 (rx#1) | `tx_daemon.log:1135` |
| 15:38:11.11 | base 15:38:10.240 | catch 2: #293 (seq 37), (14,40) ch 44, the PROBE-1 channel. PROBE 2 at (14,41) ch 6, heard | `rx_daemon.log:93-99`; `tx_daemon.log:1257` |
| 15:38:37.43 | base 15:38:36.564 | catch 3: #346 (seq 90), (17,22) ch 6. PROBE 3 at (17,22) ch 6, **not heard** | `rx_daemon.log:119-125` |
| 15:38:43.45 | base 15:38:42.583 | catch 4: #358 (seq 102), (18,2) ch 6. PROBE 4 at (18,3) ch 18, heard | `rx_daemon.log:139-145`; `tx_daemon.log:1525` |
| 15:39:51.50 | base 15:39:50.635 | catch 5: #494 (seq 238), (24,43) ch 18, the first tractor frame on ch 18 after catch 4 (gap 68.1 s). PROBE 5 at (24,43) ch 18, heard. No later tractor frame used ch 18 | `rx_daemon.log:195-201`; `tx_daemon.log:2085` |
| 15:40:48.41 | tractor 08:32:11.556 | `frames_in=606 ok=606 fail=0`; the last 112 frames (56.0 s) went unheard | `tx_daemon.log:2545` |
| 15:40:52.28 / 15:40:54.22 | tractor 08:32:15.433 / base 15:40:53.346 | SIGTERM, tractor then base | `tx_daemon.log:2547`; `rx_daemon.log:245` |
| 15:43:21Z / 15:43:23Z | – | post brackets: `radio_state` 4 on both boards | `legs/leg2b_f3_post_{base,tractor}.txt` |
| 15:43:28Z | – | `PARK_OK` x2, 0x80 / 0x80 | `legs/leg2b_f3_park.txt` |
| 15:44:09-18Z | – | check-only: both 0x80 SLEEP, rc 0 | `legs/power_up_guard_2026-10-24.log` |
| 17:04:58-17:05:06Z | – | read-only re-check, 80 min after the park: both 0x80 SLEEP, rc 0 | `legs/power_up_guard_2026-10-24.log` |

Slots and channels: the slot of each catch is its tractor TX time on a grid
fitted from the tractor's own TX times (period about 199.6-199.7 ms). The channel
is the firmware hop permutation (FNV-1a seed, xorshift shuffle) of that slot,
re-implemented in `scripts/fhss_perm.py`; it reproduces every logged
`RFCO_PERTX` (epoch, hop) → channel pair. Which catch was SNAPPED and which
was REJECTED_EPOCH_DRIFT is not logged per frame: catches 1, 2 and 4 snapped
and 3 and 5 were rejected is one reading that fits the totals (any 3/2
split does); unverified per frame.

**Leading hypothesis (H1; high confidence in the outcome, part of the
mechanism inferred).** The base's cold-start FHSS scan used up its four
30 s attempts (`SX1276_RX_SCAN_REDESIGN_MS 30000`,
`sx1276_rx_scan_policy.h:91`; `SX1276_RX_SCAN_MAX_RETRIES 3`,
`sx1276_rx_scan_fail.h:63`; 500 ms per channel, back to ch 0 each attempt)
without a tractor frame on the channel it was dwelling on, and entered scan
FAILED at about +120 s. FAILED is absorbing: the only way out is a reset or a
profile re-activation (`sx1276_rx_scan_policy.c:153`). In FAILED a valid
frame cannot LOCK the scan, and the slot follower runs only when LOCKED with
an adopted grid, or for a node with originator authority
(`sx1276_rx_grid_policy.c:107-112`, `sx1276_rx.c:867-871`). The base is a
follower and never holds authority. So the base adopted the tractor's grid on
the snapped catches (by the reading above; unverified per frame) but never
hopped. It listened on one channel: first the walker's last (ch 9), then
each PROBE's TX channel (44, 6, 18). After each TX the receiver is re-armed
where it is, by the firmware's own post-TX re-arm (`sx1276_tx.c:156-160`;
the daemon's earlier raw RXCONT write is mirrored into the firmware's
tracked state, `host_cmd.c:570-584`) and by the daemon's `_ensure_rxcont`
(`image_rx_daemon.py:870-873` at `d28bc2ca`). Neither retunes. Every tractor frame on the parked channel in each parked
window was caught, and no other: 5 of the 231 frames from the first catch on,
about 1 in 50 channels. Evidence for it: the parked channels, the counter fingerprint
(aligned 0, snapped 3, epoch_drift 2, demotion 0, dio0 10), `radio_state` 4
on both boards at post (neither went through a LOCKED → SCANNING → FAILED →
standby cycle after the leg), and the first catch at 138.0 s, past the 120 s
budget.

What rests on inference:

- That no tractor frame reached a dwelled channel during the scan: the one
  unserviced DIO0 edge (Δ`rx_urc_lost` 1, time unknown) could have been one.
- The base's FAILED entry and its time: no base FAULT URC is logged. One
  direct anchor bounds the scan start: `SX1276 already in RXCONT` at
  +0.054 s after the base's LBT line (`rx_daemon.log:35`) shows the scan had
  begun by then.
- The walker model (`scripts/acq_sim2.py`, `scripts/grid_fit.py`): with one
  common activation offset it reproduces the first frames of legs 1-5 (#139,
  #57, #91, #57, #19, on attempts 2/1/1/1/0) and gives no catch in 120 s on
  leg 6. That leg-6 result is knife-edge. A replication found the leg 1-5
  offsets span about −5 to +75 ms, and leg 6 gives "no catch" only for −5 to
  +40 ms; above that the model catches frame #6 at +8.8 s. The model also
  ignores the base L072's tick rate: it runs on HSI16, measured 610-1400 ppm
  slow against the base host clock today, varying by leg
  (`scripts/refute/fwrate.py`). A slow base clock makes leg 6's "no catch"
  firmer. The direct evidence above does not depend on the model.

Rejected or not needed:

- **H2, a lock loss after acquisition** (the RS-12.12 / RS-12.15 class):
  rejected. Base demotion Δ0 and aligned Δ0; the first catch came at +138 s.
- **H3, a wrong grid adopted:** rejected. All five headers fit one tractor
  grid, and the tractor heard PROBE 1, sent on it.
- **H6, a time-authority fault:** rejected. Tractor counters clean on every
  leg; the base's epoch_drift rejections come from its own scheduler epoch,
  frozen in FAILED.
- **H9, base command-plane deafness (M2) or the URC path (M1):** rejected.
  The base sent 5 times; FIRMWARE DROP 0, Δ`rx_urc_lost` 1.
- **H4, RF interference or a weak link:** low; not needed. The level and SNR
  match legs 1-5, and every frame on the parked channel was received.
  Interference during the receive-only scan dwells cannot be excluded.
- **H5, stale base L072 state** (never reset between legs): low. Profile 1
  activation resets the scan, clock, authority and adoption
  (`host_cfg_profile.c:214-223`), and the pre-leg state was the same as on
  legs 2-5, which acquired.

The codec: there is no VECTOR-specific defect; the failure is below the
payload. But frame airtime decides whether a walker dwell catches a frame.
VECTOR frames are on air about 166-169 ms (median ToA per VECTOR leg) against
about 131 ms for `mono_g4`. On leg 6 the first frame on a dwelled channel
(#6, 169 ms) overran its 500 ms dwell; a `mono_g4`-size frame would have been
caught at +8.8 s for part of the calibrated offset window
(`scripts/refute/walk6c.py`). The model puts the chance of no lock in 120 s at
about 9.2 % for VECTOR legs and 7.9 % for `mono_g4` legs at 2 fps. So an
airtime contribution to this leg's failure cannot be excluded. The tractor
itself was unaffected.

How often: 1 of 27 archived profile-1 legs that log `frag_arrival` (26 from
2026-09-12 to today's legs 1-5, plus leg 6) went past 120 s. The other 26
caught their first frame about 3.5-75 s after activation (two independent
surveys, which differ slightly by their reference line; `scripts/first_frames.py`, `scripts/refute/survey.py`). Quote that
rate, not the modelled 5-10 %. The July and August profile-1 archives do not
log `frag_arrival` and were not checked.

Related, earlier:

- 2026-10-18 legs W and X (DC/TODO.md, RS-12.15 reverse-delivery half): the
  tractor's scan ran out into FAILED (faults 0x00/0x01/0x02/0x83 about 30 s
  apart). It was harmless there because an originator follows its own clock
  in FAILED. A follower has no such escape, which is what stranded the base
  here. "Whether self-follow should also run in scan FAILED" is open for the
  design owner on that row.
- RS-12.21 (DC/TODO.md) documents FAILED as absorbing (500 ms per channel,
  30 s per attempt, 3 retries, up to about 2 min). Its post-leg form is the
  park rule; leg 6 is the first time it was reached while a leg ran.
- RS-12.19: "the base's follower needs 20–30 s of frames to lock". Slow
  acquisition was known; its failure rate and its absorbing end were not.

**PROBE 3 miss** (secondary). The base labelled PROBE 3 (17,22), the slot of
the frame it had just heard; its on-air OK came 49 ms after that frame
(`rx_daemon.log:119-127`). On the fitted grid it ends at about 0.91 of the
slot, before the boundary; same-slot PROBEs ending later were delivered on
legs 1-5. The tractor shows `radio_crc_err` 1 and one DIO0 that no counter
explains (Δ`radio_dio0` 613 against 607 TX + 4 RX + 1 CRC). That points to a
corrupted reception rather than an off-channel miss. A drifting free-running
clock (H7: both L072s tick on HSI16 by design, `platform.c:143-171`, while
the FHSS clock design assumes ±2 ppm TCXOs, `sx1276_fhss_clock.h:15`) is
possible but not supported by the timing. Cause unverified.

**Effect on the round.** The pre-written rule computes NOT WORSE with leg 6
included (−0.99 pp). Leg 6 is a base-side FHSS acquisition failure, not a
measure of VECTOR command delivery, but the rule counts its 4/5 and the
record keeps it there. Whether to re-fly 2b_f3 is the operator's call; it
needs a new GO.

### Tool errors on leg 6

- `rs12_leg_report.py` and `rs13_digest_trace.py` print "first frame heard
  at seq 8: the 7 frame(s) numbered before it … counting from seq 1: 98/103
  = 95.1 %". The A7 head assumes no u8 wrap (`tools/rs12_leg_report.py:333` at `d28bc2ca`).
  Seq 8 was frame #264, and the true figure is 601/606 = 99.2 %.
- The capture line "lost 91/96 = 94.8 %" books the 102 → 238 step as a
  counter-restart "jump": with no clean +1 step there is no period, so any
  step over 128 is a jump (`tools/rs12_leg_report.py:287-294` at `d28bc2ca`). `leg_replay.py`
  and `legs/leg2b_f3_p2p4.txt` give 226/231 on the same capture.
- `frag_gap_report.py`'s "gaps>3s=4, max_gap 68.1 s" spans only the 115 s
  between the first and last frames heard. It leaves out the 131.6 s before
  the first frame and the 56.0 s after the last.

### Other anomalies

- **Tractor `radio_tx_ok` is the txdone count + 1 on every leg** (614/613,
  616/615, 614/613, 610/609, 611/610, 607/606). Each `tx_daemon.log` has an
  `RFCO_PERTX` at (epoch 0, hop 0) ch 3 with ToA 125.6 ms that matches no
  frame (`tx_daemon.log:47`). On leg 6 it falls at grid slot 0, 40 ms after
  `camera_service`'s `encode_mode -> 9 (vector) [boot]`
  (`camera_service.log:9`), and frame #1 is at slot 8. Likely the boot ack
  relayed over LoRa, which also anchors the tractor's grid. Its content is
  unverified. It does not touch any score.
- **FAULT code 0x0D is ambiguous.** `HOST_FAULT_CODE_TX_FIFO_RB_MISMATCH` and
  `HOST_FAULT_CODE_RX_SCAN_FAILED` are both 0x0D (`host_types.h:257`, `:280`).
  The tractor's 0x0D lines are read as RX_SCAN_FAILED from their timing (+30,
  +60, +90 s after activation). Per leg: leg 1 sub 0x01 at uptime 64 s; legs
  2, 3 and 4 sub 0x20 at 34.0 s; leg 5 none; leg 6 0x20 at 34.0 s and 0x02 at
  94.0 s. They track the base's acquisition time (the tractor's scan locks
  only after the base starts sending).
- **Leg 5 Δ`rx_urc_lost` 8**, above the 0-2 of 10-18/19. Not investigated.

## VECTOR_SCENE.md §8.6 rows

| measure | criterion | result |
|---|---|---|
| Base command delivery during VECTOR | not worse than during `mono_g4` | FHSS profile 1 on the antennas: VECTOR 383/384 vs `mono_g4` 542/543, one-sided 95 % lower bound −0.99 pp > −3 pp: NOT WORSE (margin rule, written before leg 1). Leg 6 contributes 4/5 from a base that never locked. DTS: 2026-10-19, NOT WORSE (Fisher p = 0.242) |
| Fragment loss | not worse than `mono_g4` on the same workload and profile | not in this round's rule. For information, seq-gap loss after the first frame, legs 1-5: VECTOR 2/1112, `mono_g4` 2/1590 |
| Range-edge leg | per 3 dB step | not flown: no attenuation equipment (2026-10-19) |

## What the round did NOT test

- the FHSS switch leg ("2d on profile 1"), the web_ui leg and the A16
  confirmation (out of scope; one A16 episode came by chance)
- DTS (profile 2): flown 2026-10-19
- range: one bench distance on the antennas, one level (−64 dBm / SNR 10 dB)
- fragment loss as a pass criterion (recorded, not scored)
- the railroad scene: the kept frames show ads and a talking-heads clip
- FHSS cold-start acquisition as such: leg 6's failure was found, not
  tested, and the base's scan state was not logged
- a first leg after a boot: both boards had been up about 6 days, so the
  tractor reset's first-leg question stays open
- the base L072 reset (`BASE_L072_RESET unverified` on all six)
- 1 fps or 5 fps; `-InjectCmd` (`inject_cmd=` empty in every `params.txt`)

## Evidence limitations

- The base's scan state is not observed. Its FAULT URCs are dropped, so the
  FAILED entry on leg 6 is inferred.
- The leg-6 mechanism leans on a walker model calibrated on legs 1-5. Its
  "no catch" depends on about 40 ms of activation timing and on the base
  L072's HSI16 rate. The parked channels and the counters do not depend on
  it.
- Tractor-side receive level: not logged (`LoRa cmd: PROBE` lines carry no
  RSSI or SNR).
- The scene was checked before each leg, not during it.
- Probes pooled as independent; 161-203 per full leg. With `-ReactiveFire 1`
  probes exist only after a base reception.
- `BASE_L072_RESET unverified` on all six: base brackets are post − pre. The
  tractor L072 reset landed on all six: its brackets are the post values.
- The tractor's logs run on the tractor clock (2026-10-05). Cross-board
  timing above uses the params.txt offsets (±0.03 s) and matched events.
- `scripts/` holds the analysis scripts as written during this session's
  diagnosis. They are not reviewed, and some read this PC's paths.

## Next steps

No radio, first:

1. **Make the base's scan state visible.** Log FAULT URCs in
   `image_rx_daemon.py` (at least 0x0D with the decoded sub-byte), and on the
   tractor outside the pipelined-TX wait too. Add an acquisition check to the
   leg report or `bracket_deltas.py`: first-frame latency after activation,
   flagged over 120 s; base Δ`clk_demotion_reset` 0 with Δaligned 0 while
   the tractor streamed; `radio_state` 4 on both boards at post. Fix the u8
   wrap in the A7 head and in the capture seq-gap count, and print the head
   and tail in `frag_gap_report.py`. SIL and unit tests only.
2. **Reproduce leg 6 in the host-cc SIL** against the real `sx1276_rx.c` /
   `sx1276_tx.c` (the `check-rx-reverse-delivery` style): a follower on
   profile 1 with no frames for 120 s goes FAILED; then valid frames: it
   stays FAILED, adopts the grid, does not follow, re-arms on the TX channel,
   and gives the SNAPPED / EPOCH_DRIFT pattern.
3. **Fix options for the design owner**, each SIL-checked before any flash:
   (a) FAILED plus an accepted header → LOCKED, a valid frame as the external
   recovery; (b) no absorbing FAILED for a receive-only follower on profile 1
   (check the 30 s regulatory narrative; RX scanning emits nothing); (c) a
   host watchdog: the rx daemon re-sends `REG_PROFILE=1` when no frame comes
   N s after start, or on a final 0x0D; (d) a higher per-attempt success (now
   about 0.5), e.g. dwell at least one 10 s epoch per channel.
4. **A pass-rule clause for a leg whose base never locks** (re-fly or count),
   written before the next round. Run the walker model over the archived
   profile-1 legs at 1, 2 and 5 fps to size the risk.
5. **Clock side item:** the L072s tick on HSI16, the FHSS clock design
   assumes ±2 ppm TCXOs. Bound the free-running horizons in the SIL (PROBE 3;
   RS-12.15's originator decay); look for `BOOT_URC` `clock_source_id` in any
   archived boot capture.
6. **Done:** the read-only re-check at 17:05Z (`power_up_guard.sh
   --check-only`) read 0x80 on both boards after leg 6's RX_CONT post
   brackets.

Radio, each on its own explicit GO:

7. Optional: re-fly 2b_f3 so the VECTOR arm has three full legs. The verdict
   stands as computed either way.
8. After the logging change is deployed: a cold-start repeat set, about 10
   short profile-1 starts at 2 fps of at least 150 s each, to see the base's
   0x0D sub-bytes and measure the FAILED rate.
9. After a SIL-validated fix (a flash GO plus a GO per round): an old/new A/B
   cold-start set (first-frame latency, FAILED count, post `radio_state`).
10. Still open from the scope: the FHSS switch leg, the web_ui leg, the A16
    confirmation (a distance test).

## GO / NO-GO

The deciding rows:

- Command delivery on FHSS, by the pre-written rule: NOT WORSE (PASS).
- First flight of the #161 kit: PASS on every check; the end-of-round
  radio state re-checked at 17:05Z: 0x80 on both boards.
- A16: 1 loss-opened episode, PASS, thin. Not a confirmation.
- Leg 6: a base-side FHSS cold-start acquisition failure. Open. It does not
  change the verdict as computed, and nothing ties it to a VECTOR defect.

Recommended next: the no-radio items 1-5 above (code, logging and SIL; the
operator's and the design owner's call). Any radio leg, a re-fly of 2b_f3
included, needs a new explicit GO. Do items 1 and 4 before the next FHSS
round: as the round stands, another acquisition failure would cost a leg
again, and one with no catch at all would withhold the pooled verdict.

## Files

- **`legs/`, session:** `staging_{base,tractor}.txt` (md5 records);
  `power_up_guard_2026-10-24.log` (a copy of the `BENCH_SCRATCH` log: the
  14:27Z power-up check, the 15:44Z end-of-round check-only and the 17:05Z
  re-check);
  `round_probe_score_pooled.txt` (the pass rule over the six archives),
  `round_probe_score_excl_leg6.txt` (for information) and
  `round_digest_trace.txt` (A16 over the three VECTOR captures), all
  generated after the round from the committed archives at `d28bc2ca`.
- **`legs/`, per leg** (`leg2c_f1_`, `leg2b_f1_`, `leg2c_f2_`, `leg2b_f2_`,
  `leg2c_f3_`, `leg2b_f3_`): `prep`, `scene.{txt,jpg}`,
  `health_{base,tractor}`, `clear_retained`, `pre_`/`post_{base,tractor}`
  brackets, `link_stats`, the base capture `base.{jsonl,json,txt}`,
  `harness` (console transcript), `params`, `report`, `p2p4`, `gaps`,
  `probe_score`, `rf`, `deltas`, `replay`, `tractor_log`,
  `tractor_ocd_reset.log`, `park` and `post_transcript`. The VECTOR legs add
  `digest_trace`.
- **`scripts/`:** the leg-6 diagnosis and re-score scripts, copied from the
  session scratchpad: `fhss_perm.py` (hop permutation), `grid_fit.py`
  (tractor grid fit; writes `grid.pkl`, not kept), `acq_sim2.py` and
  `acq_prob.py` (walker model), `parked3.py`, `first_frames.py`,
  `alldelta.py`, `period_fit.py`, `rescore.py`, `pooled_stats.py`; and
  `refute/` (the skeptic's replication: `walk6*.py`, `allegs*.py`,
  `grid*.py`, `perm.py`, `fwrate.py`, `probephase.py`, `probes6.py`,
  `survey.py`, `newc.py`).
- **Harness archives** (in `bench-evidence/`; each holds `params.txt`,
  `rx_daemon.log` (base), `tx_daemon.log` (tractor), `camera_service.log`,
  `deploy_md5.txt` and `tractor_ocd_reset.log`):
  - 2c_f1: `radio_monitor_20261024_095315_d28bc2ca`
  - 2b_f1: `radio_monitor_20261024_100249_d28bc2ca`
  - 2c_f2: `radio_monitor_20261024_101341_d28bc2ca`
  - 2b_f2: `radio_monitor_20261024_102247_d28bc2ca`
  - 2c_f3: `radio_monitor_20261024_103156_d28bc2ca`
  - 2b_f3: `radio_monitor_20261024_104056_d28bc2ca`

## Session end

- 15:44:08Z: the bench video window closed (`youtube_window.ps1 -Close`).
- 17:05Z: read-only re-check, both radios 0x80 (above).
- About 17:06Z: the PC MQTT broker that leg 1's harness started
  (`start_mqtt_broker.py`, PID 18712, listening on 0.0.0.0:1883 since
  14:47:36Z) was stopped by this session; nothing listens on 1883 now, as
  before the round. The harness starts a fresh one at the next leg, so no
  retained control topic carries over. Session console, not archived.
