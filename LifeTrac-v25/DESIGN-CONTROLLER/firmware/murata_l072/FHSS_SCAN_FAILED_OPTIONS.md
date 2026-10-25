# FHSS scan FAILED: options for a node that cannot recover

**Status:** decision memo, **adopted by the design owner (Dorkmo) on
2026-10-24**: the option (b) package of section 5 item 2 (b + L + d2, the
RS-12.21 park command in the same image, the 0x0D split of section 7), with
the decay-window trade-off of section 5 item 3 accepted
([`DECISIONS.md`](../../DECISIONS.md) D-FHSS1). Implemented on branch
`fhss-scan-recovery-2026-10-24` and built 2026-10-25 (bench `0daae279`,
production `2c5e055a`, section 9). **Flashed on both bench boards
2026-10-25** on the operator's GO (tractor then base: `Verify OK`,
`flash_rc=0`), with every post-flash check and the first legs R1 and R2
passed (section 6.1;
[`RESULTS.md`](../../bench-evidence/bench_session_2026-10-25/RESULTS.md) of
that day). T2 (ten cold starts) flew later that day and passed: 10 of 10
acquired, none stranded
([`RESULTS.md`](../../bench-evidence/bench_session_2026-10-25_t2/RESULTS.md)).
T1 and T4 with camera stalls are still to fly.
**Date:** 2026-10-24 (main `5d00dac2`).
**Decision owner:** the design owner (Dorkmo).
**Rows:** RS-12.21 and RS-12.15 in [`../../TODO.md`](../../TODO.md).
**Evidence:** leg 6 (2b_f3) of the 2026-10-24 round,
[`RESULTS.md`](../../bench-evidence/bench_session_2026-10-24/RESULTS.md)
"Leg 6 (2b_f3): the base never locked the FHSS grid" (lines 315-501).

How to read the sources. `R:N` is a line of that RESULTS.md. File:line
citations are on main `5d00dac2`. **SIL** is the new host-cc check
`check-rx-scan-failed-follower`
([`bench/host_proto/rx_scan_failed_follower.c`](bench/host_proto/rx_scan_failed_follower.c),
`make check` runs it). **SIL-E1/E1w/E2/E3** are that program's `-e` and
`-e3` evaluations, run on patched scratch copies of the firmware.
**W** and **C** are the walker-model and clock outputs in
[`bench-evidence/fhss_cold_start_analysis_2026-10-24/`](../../bench-evidence/fhss_cold_start_analysis_2026-10-24/)
(`risk_run.txt`, `risk_summary.txt`, `clock_run.txt`; the scripts are
there, and `sil_make_variant.py` regenerates every SIL option patch,
including the variants without L, `noL`, that section 4 quotes).

---

## 1. Summary

- On FHSS (profile 1) the base is a follower. Its cold-start scan has four
  30 s attempts. If no tractor frame lands in a dwell, the scan enters
  FAILED. FAILED never ends without a reset or a profile re-activation.
  In FAILED a follower never LOCKs and never hops. That is the leading
  hypothesis (H1) for leg 6. It fits every leg-6 observation. It is not yet
  observed directly, because the base's FAULT URCs are not logged.
- The SIL reproduces the mechanism against the real firmware and pins it as
  a known defect. It also found a twin on the tractor: in FAILED, one
  camera stall plus one heard command ends the tractor's self-follow for
  the rest of the session.
- How often (walker model, section 3): about 7-8 % of 2 fps legs with one
  fragment per frame end in FAILED (4.0-10.4 % by leg). 21-31 % at 1 fps.
  Under 1 % on average at 2.4 or more transmissions per second. By the same
  model a six-leg 2 fps round loses at least one leg 34-40 % of the time
  (base tick 0 to −1400 ppm). Observed: 1 of the 8 archived 2 fps
  one-fragment legs, 1 of all 27 legs. Those counts are too few to size a
  round, so the model rate is quoted beside them.
- Four fix options were prototyped in the SIL only. (a), (b) and a host
  watchdog (c) all recover leg 6 in 100/100 replays. (d), a longer dwell,
  makes FAILED rarer but does not recover from it.
- **Recommendation for the design owner (section 5; adopted 2026-10-24,
  built and flashed 2026-10-25):** option (b) with the
  loss-timer fix L and a 2 s dwell, flashed together with the RS-12.21 park
  command it needs and a new code for one of the two 0x0D faults. L and
  the 2 s dwell also change the tractor's normal RS-12.15 decay window
  (SIL-E3, 200 runs: commands received 53.2 → 59.1 %, own-grid
  renumberings +24 %); weigh that first (section 5, item 3).
- Side items: the L072 ms tick is HSI16, not a ±2 ppm TCXO. The base and
  tractor ticks differed by 2366-3210 ppm on 2026-10-24. The leg-6 PROBE
  labels fit that drift; which catches SNAPPED is inferred under H1, not
  logged (section 8.5). FAULT code 0x0D means two different faults.

## 2. The problem

### 2.1 What leg 6 showed

- The tractor sent 606 frames normally (R:319-322).
- The base heard 5 of them (R:323-326). Its first frame came 138.0 s after
  its profile activation (R:368, R:413), past the 4 x 30 s budget.
- The base was never LOCKED: Δ`clk_demotion_reset` 0, Δ`fhss_dec_aligned`
  0, Δ`fhss_dec_snapped` 3, Δ`fhss_dec_rej_epoch_drift` 2 (R:331-337).
- The five catches were on channels 9, 44, 6, 6, 18. After the first, each
  was on the channel of the base's previous PROBE (R:340-342, R:368-373).
  A hopping receiver would not do that.
- Post brackets read `radio_state` 4 (RX_CONT) on both boards; 2 on legs
  1-5 (R:338-339).
- The base's 0x0D scan faults are not in the log: `image_rx_daemon.py`
  drops every FAULT URC at DEBUG (`base_station/image_rx_daemon.py:1337-1340`).
  So the FAILED entry is inferred (R:346-347, R:416-433).

### 2.2 The mechanism in the code (H1)

1. Walker: 500 ms per channel (`include/sx1276_rx_scan_policy.h:63`), 60
   dwells per 30 s attempt (`:91`), back to channel 0 at each attempt
   (`radio/sx1276_rx.c:564-572`), 3 retries (`include/sx1276_rx_scan_fail.h:63`).
2. Each failed attempt emits FAULT 0x0D with a sub-byte
   (`radio/sx1276_rx.c:665`). The fourth is final. FAILED then absorbs
   every event (`radio/sx1276_rx_scan_policy.c:153-162`). A follower's modem
   is parked in STANDBY (`radio/sx1276_rx.c:683-685`) on the walker's last
   channel: dwell 60 is index 59, channel 9.
3. The base daemon re-arms RXCONT every 5 s by a raw register write, with
   no retune (`base_station/image_rx_daemon.py:1312-1316`). The firmware
   mirrors that write into its tracked state (`host/host_cmd.c:570-584`)
   and re-arms after each own TX on the TX channel (`radio/sx1276_tx.c:156-160`).
4. A valid frame heard in FAILED is still adopted (`radio/sx1276_rx.c:273-294`)
   but cannot LOCK. The slot follower runs only when LOCKED on an adopted
   grid, or under originator authority (`radio/sx1276_rx_grid_policy.c:107-112`,
   `radio/sx1276_rx.c:867-871`). A follower never holds authority.
5. So the base listens on one channel at a time: about 1 frame in 50.
6. Only a reset or a profile re-activation leaves FAILED
   (`host/host_cfg_profile.c:214-223`). The same comment records an earlier
   stranding of this class, on 2026-07-24 (`:217-222`).

### 2.3 What the SIL shows

`check-rx-scan-failed-follower` runs the real `sx1276_rx.c`, `sx1276_tx.c`,
scan, grid, authority and clock code against a simulated SX1276, air and
main loop. The file header lists exactly what is asserted, assumed and
seed-fitted.

- **Assumed, not observed:** no frame reaches the base for the whole
  120 s budget; 0 ppm between the boards; fixed 200 B frames. So "FAILED at
  +120 s" and the fault sub-bytes 0x20 0x21 0x22 0xA3 are H1 predictions,
  not leg-6 observations.
- **Asserted for any seed (1-1000 pass, except 558):** four 0x0D faults and
  STANDBY on channel 9; then frames are adopted but never LOCK, never
  demote and never hop; every later catch is on the previous PROBE's
  channel; never ALIGNED; a catch within one epoch of the frozen scheduler
  epoch is SNAPPED, two or more is REJECTED_EPOCH_DRIFT; the run ends in
  RX_CONT, still FAILED.
- **Seed-fitted (seed 852, a soft check):** leg 6's first two catches slot
  for slot, (13,17) on ch 9 and (14,40) on ch 44, and its 3/2 decision
  split. The split comes up in 35 of 1000 seeds.
- **Not reproduced:** leg 6's catches 3-5 and its same-slot PROBEs. Section
  8.5 shows those need clock drift, which the SIL originator model leaves
  out.
- **Tractor twin (asserted, default seed):** an originator in FAILED keeps
  following its own clock, as on 2026-10-18 legs W/X. But it adopts any
  accepted header (UNANCHORED health while nothing is adopted), only a
  LOCKED -> SCANNING demotion clears adoption
  (`radio/sx1276_rx_grid_policy.c:84-96`), and no node holds authority while
  adopted. In FAILED nothing LOCKs, so nothing demotes. One camera stall
  (authority decays 1 s after the last own TX,
  `include/sx1276_fhss_authority.h:59`) plus one command heard in it ends
  self-follow for good. SIL-E3, 50 runs, 300 s with a 2.5 s stall every
  9 s: the first adoption comes a median 12.6 s into the command phase
  and authority never returns (50/50); authority on 3.8 % of the phase's
  passes, 36.6 % of commands received. The baseline is main before
  FAILED under the same stalls (commands from 5 s, so the scan LOCKs on
  them and never fails out: RS-12.15's decay window, 50 runs): authority
  on 22.1 %, 53.2 % received. In FAILED without stalls (20 runs):
  authority on 100 %, 80.7 % received.
- The check passes on main and is written to fail when either behaviour
  changes. The PR that lands a fix replaces it with that fix's acceptance
  check.

### 2.4 Why it matters beyond one leg

- **Delivery:** a stranded base hears about 2 % of the stream (SIL-E1 on
  main: 723/36002 frames). Leg 6 heard 5 of 606 (R:323).
- **Regulatory:** the FCC notes list "synced receivers" in the
  §15.247(a)(1) row (`AI NOTES/2026-05-19_FCC_Part15_902-928_Compliance_Notes_Copilot_v1_0.md:463`).
  A follower in FAILED keeps transmitting on hop channels (leg 6: 5 PROBEs)
  while its receiver no longer follows the hop sequence. The scan policy
  header ties the 30 s FAIL to the narrative "cold-start ≤ a few seconds"
  (`include/sx1276_rx_scan_policy.h:31-35`). That narrative does not hold
  for a stranded base.
- **The plan's own threshold:** plan §7 Q6 sets a 5 s cold-start target and
  says a measured time over 30 s means an A6 redesign, "adaptive scan dwell
  or super-frame beacon slot", not a bug fix
  (`AI NOTES/2026-05-19_LoRa_FCC_50CH_FHSS_Implementation_Plan_Copilot_v1_0.md:677`).
  Measured first-frame times on the 27 archived legs: median 28.3 s, 1 leg
  at 5 s or less, 13 legs over 30 s (W, `risk_run.txt` column `t_obs`).
- **Rounds:** see section 3.

## 3. How often: the walker model over the archive

**Method** (W, `risk_run.py`). The model is the one in
`bench_session_2026-10-24/scripts/acq_sim2.py`, vectorised and run on every
archived profile-1 leg that logs `frag_arrival` (27 legs, 2026-09-12 to
2026-10-24, the same two boards on all of them: tx 2E2C1209DABC240B, rx
2D0A1209DABC240B in `params.txt`). For each leg it fits the tractor's slot
grid from its own TX times, numbers the slots from the base's PROBE labels
where they exist, and walks the base's scan over the leg's real frames. A
frame is caught when it is on the dwelled channel and lies wholly inside
the dwell. The first catch is the LOCK. The base activation is swept over
±5 s around the logged one. The base tick runs 0, 610, 1000 or 1400 ppm
slow (the 2026-10-24 measured range, `period_fit.py`). For 1, 2 and 5 fps
the model also builds one-fragment-per-frame streams on each leg's grid,
with that leg's frame sizes and the firmware's slot gate.

**Per leg, replay of the recorded stream** (W, `risk_summary.txt`):

| workload | legs | past 120 s observed | model P(no LOCK in 120 s), tick 0 / −610 / −1000 / −1400 ppm | per-leg range |
|---|---|---|---|---|
| 2 fps camera, 1 fragment per frame (~1.9 TX/s) | 8 | 1 (leg 6) | 8.1 / 7.9 / 7.4 / 6.8 % | 4.0-10.4 % |
| 1 fps camera (0.94 TX/s) | 2 | 0 | 28.0 / 28.9 / 28.8 / 27.6 % | 26.0-30.6 % |
| camera, multi-fragment frames (2.4-3.3 TX/s) | 7 | 0 | 0.9 / 0.8 / 0.7 / 0.7 % | 0.0-2.0 % |
| synthetic feed, multi-fragment (2.9-3.9 TX/s) | 10 | 0 | 0.4 / 0.3 / 0.3 / 0.3 % | 0.0-3.2 % |
| all 27 | 27 | 1 | mean 4.9 / 4.8 / 4.6 / 4.4 % | 0.0-30.6 % |

- 2026-10-24 by codec: VECTOR 9.3 / 8.5 / 7.4 / 6.5 %, `mono_g4`
  7.9 / 8.2 / 8.1 / 7.3 %. The 0 ppm figures agree with RESULTS (9.2 %
  and 7.9 %, R:460-462).
- Leg 6 itself: 9.8 / 9.2 / 9.2 / 9.4 %.

**At 1, 2 and 5 fps, one fragment per frame** (W, `risk_run.txt` part B;
mean over the 27 grids, tick 0 to −1400 ppm):

| rate | base starts before the stream (as the harness does) | stream already on (the SIL-E2 case) | SIL-E2 on main |
|---|---|---|---|
| 1 fps | 23.5-26.5 % | 23.2-27.2 % | 23.2 % (95 % 20.0-26.7 %) |
| 2 fps | 7.2-9.3 % | 5.7-8.3 % | 6.7 % (95 % 4.9-9.0 %) |
| 5 fps | 0.00 % (no leg above 0.1 %) | 0.00-0.01 % | not run |

With camera frame sizes only, 2 fps is 6.3-8.2 % and 1 fps 21.2-23.8 %.

**The empirical rate** is 1 of 27 legs, 3.7 % (exact 95 % interval
0.09-19.0 %). The model expects 1.17-1.32 such legs among the 27 and gives
P(at least one) 0.72-0.76 (W, `risk_run.txt`). For the 26 legs that
caught, the observed first-frame time sits at a model quantile with mean
0.47 and quartiles 0.27 / 0.53 / 0.69, close to uniform. The model is
consistent with the archive.

**What this means:**

- The risk is set by slot occupancy, not by codec. One fragment per frame
  at 2 fps fills 2 slots in 5. That is the current camera workload.
- Walker model: a six-leg round at 2 fps loses at least one leg 34-40 % of
  the time, 1 − (1 − p)^6 with p the 2 fps one-fragment row above, 6.8 %
  (tick −1400 ppm) to 8.1 % (0 ppm). The model rate is quoted, not the
  archive's, because the archive is too thin for it: 1 of 27 pools
  workloads whose model risk runs from 0.0 to 30.6 %, and the matching
  subset is 1 of 8 two-fps one-fragment legs (exact 95 % 0.3-52.7 %; the
  model expects 0.54-0.65 of those 8).
- The base tick in the measured range moves the risk by under 2 points. A
  slower tick lengthens each dwell slightly.
- **Limits:** an ideal channel; the activation sweep stands in for the
  unknown phase; slot numbering on the older legs is approximate
  (statistically neutral); 5 fps keeps the leg's own frame sizes.

## 4. Fix options

Prototypes live only in scratch copies made by `sil_make_variant.py`. The
shipped sources are unchanged. Numbers: SIL-E1 is a leg-6 replay (100
seeds; silent for the budget, then the stream). SIL-E2 is a cold start with
the stream on (600 seeds). SIL-E3 is the tractor (50 seeds; 300 s of
commands with a 2.5 s camera stall every 9 s), twice: before FAILED, which
is main's normal operation and RS-12.15's decay window, and in FAILED, the
tractor twin.

**L, the loss-timer anchor (needed by (a) and (d)).** A LOCK leaves the 2 s
LOCKED loss timer at the last walker step (`radio/sx1276_rx.c:814-829`). At
a 500 ms dwell that shortens a LOCK by at most 0.5 s. Out of FAILED, or at
a long dwell, the fresh LOCK demotes on the next tick. Without L, (a) LOCKs
and demotes at once (SIL leg-6 check, variant `a+noL`: locks 2, demotions
1), and d10 with the watchdog hears 61.2 % of frames instead of 99.8 %
(SIL-E1w, `d10+noL`). L is 4 lines in `scan_drive`. Alone it keeps all of
`make check` green and leaves SIL-E1 and E2 unchanged. It does change the
tractor's decay window, where every LOCK comes out of SCANNING:
`check-rx-reverse-delivery` scenario 7 ("decay window") is the one trace
hash that moves, and SIL-E3 before FAILED goes from authority on 22.1 % of
passes, 53.2 % of commands received and 344 own-grid renumberings to
20.2 %, 54.8 % and 349 (50 runs; over 200 runs 22.1 % → 21.5 %, 53.2 % →
54.3 %, 1389 → 1371). Alone that is small. With the 2 s dwell it is most of
the change in section 5, item 3.

| | (a) FAILED + accepted header → LOCK | (b) no absorbing FAILED without authority | (c) host watchdog re-sends REG_PROFILE=1 | (d) dwell ≥ 2-10 s |
|---|---|---|---|---|
| change | policy + dispatcher, ~13 lines, + L | `sx1276_rx.c`, ~19 lines | `image_rx_daemon.py` only | 1 constant + walk continues across attempts, + L |
| leg-6 replay (E1) | 100/100; median 17.2 s, p90 48.4 s; 99.8 % heard | 100/100; median 34.8 s, p90 107.9 s; 97.6 % heard | 100/100; median 31.5 s, p90 88.9 s; 99.5 % heard | 0/100 alone; b+d10 100/100, median 16.8 s, 99.9 % |
| 2750 ppm drift | 100/100, 99.9 % | 99/100, 98.2 % | not run | b+d10 100/100 |
| cold start (E2), 2 fps | 6.7 % no LOCK in 120 s; 600/600 by 300 s | 6.7 %; 599/600 by 300 s | 6.7 % (fires only after the first budget); not run past it | d2 1.3 %, d10 0.3 %, d10r 0.5 % |
| cold start (E2), 1 fps | 23.2 %; 598/600 by 300 s | 23.2 %; 584/600 by 300 s | 23.2 %, as above | d2 10.7 %, d10 8.5 %, d10r 6.0 % |
| tractor twin, FAILED + stalls (E3; main: authority never back 50/50, 3.8 %, 36.6 % received) | authority back in 50/50; 22.0 %, 53.7 % | authority back in 50/50; 22.5 %, 53.6 % | unchanged (base only) | alone unchanged (d2, d10, d10r: 3.8 %, 36.6 %); b+L+d2 back in 50/50, 13.6 %, 61.3 %; b+d10 12.9 %, 61.0 % |
| decay window, before FAILED (E3; main: authority 22.1 %, 53.2 % received, 344 own-grid renumberings) | unchanged (L in its FAILED-only form) | unchanged | unchanged | d2 (and b+L+d2) 14.4 %, 58.8 %, 403; d10, d10r, b+d10 13.0 %, 59.4 %, 424; d2 without L 21.5 %, 52.5 %, 403 |
| existing checks | `rx_scan_policy` vector 16 fails; reverse-delivery scenarios 8/9 setup asserts fail | shipped checks none; the shared sim's own FAIL invariants fail in the leg-6 run (see (b)) | firmware none | d2 none; d10/d10r scenario 6 fails |
| flash | yes | yes | no (base X8 deploy) | yes |

### (a) A valid frame recovers from FAILED

- **What:** FAILED + FRAME_VALID returns LOCK
  (`radio/sx1276_rx_scan_policy.c:153`), plus L in its FAILED-only form.
- **For:** fastest recovery. A parked receiver is an unlimited-dwell
  single-channel scan, so it catches 1 frame in 50 and then LOCKs. Keeps
  the bench park: FAILED still parks unless a frame arrives.
- **Against:**
  - It depends on the receiver being armed in FAILED. The firmware parks a
    follower in STANDBY at the final FAIL. Only the daemon's 5 s raw
    RXCONT re-arms it. A host that does not do that stays deaf. (a) needs
    "do not park a follower at the final FAIL" with it.
  - False lock: the A6a header is unauthenticated and farm/node default to
    0. A foreign same-profile transmitter heard on the parked channel LOCKs
    the base onto its grid. SCANNING has that exposure today; (a) makes it
    open-ended.
  - The tractor in FAILED now LOCKs on ALIGNED commands and demotes 2 s
    later, keeping its own clock (`clk_demotion_kept` rises). The 2026-10-18
    leg-X expectation "kept 0, no LOCK while FAILED" changes.
  - It changes the pure policy's contract (vector 16 pins "FAILED absorbs").
- **Regulatory:** restores receiver sync at the first frame heard. No
  change to emissions.

### (b) FAILED only while the node follows its own clock

- **What:** at the final FAIL, a node without originator authority
  re-enters SCANNING with fresh anchors, as a non-final FAIL does. Every
  later 30 s attempt still emits 0x0D with the final bit. `scan_tick` also
  restarts the scan of a node in FAILED whose authority has decayed. A
  streaming tractor keeps today's FAILED-with-self-follow.
- **For:** uses the normal acquisition path; the pure policy and its
  vectors stay as they are; none of the shipped checks regresses (all 10
  reverse-delivery trace hashes identical); fixes the tractor twin to
  main's pre-FAILED level; does not depend on a host re-arm.
- **Against:**
  - **Harness:** the shared simulation (`rx_reverse_delivery.c`) assumes
    a final FAIL parks. There it expects no retune and mirrors the scan
    state as FAILED, so the retune that (b) dispatches at the final FAIL
    (BEGIN_SCAN; ADVANCE_CHANNEL with d2 or d10) counts as a walker
    violation, and the mirror loses track until the next LOCK
    (`check_structure`, leg-6 run: 2 walker violations and 47 396 mirror
    mismatches under (b), 1 and 3200 under b+L+d2). The prototype also
    records only one of its two FAILED → SCANNING edges: the `scan_tick`
    restart calls `sx1276_rx_scan_counter_record`, the re-entry at the
    final FAIL does not. So (b)'s acceptance check needs the harness's
    final-FAIL expectations changed, and the fix should record both edges
    (or the mirror learn (b)'s rule).
  - **Bench park (RS-12.21):** both boards scan forever once the daemons
    stop (a stopped tractor loses authority after 1 s), re-arming RXCONT
    every dwell. `radio_park.py`'s SLEEP write never holds. The firmware
    park command in the RS-12.21 row becomes a prerequisite.
  - Power: continuous RX instead of STANDBY. Fine for the base and the
    tractor. A battery node would want the back-off the 2026-05-19 analysis
    deferred (`AI NOTES/2026-05-19_RX_Scan_FAILED_State_Analysis_Copilot_v1_0.md`
    §13.2). That analysis also says such a change "replaces the absorbing
    FAILED contract": this is that decision.
  - One 0x0D every 30 s while unacquired, so the 0x0D collision (section
    7) should be fixed first.
  - On the tractor, recovery brings back main's decay-window behaviour:
    phase restarts and renumbering by up to 16 slots (SIL-E3: 335
    renumberings in 50 runs, as on main before FAILED, 344). That is the
    open RS-12.15 "decay window" question, not new exposure. With L and d2
    added, recovery returns to that image's own decay window, which is not
    main's (section 5, item 3).
- **Regulatory:** restores receiver sync. The 30 s FAIL becomes "report
  and keep scanning (receive only)". The FCC narrative in the scan policy
  header should say so.

### (c) Host watchdog (modelled, not written)

- **What:** the rx daemon re-sends the base's own `CFG_SET REG_PROFILE=1`
  on a final RX_SCAN_FAILED fault. Activation resets the scan, clock,
  authority and adoption (`host/host_cfg_profile.c:214-223`).
- **For:** no flash.
- **Against:**
  - The daemon must first decode FAULT URCs. The 0x0D collision makes a
    naive "0x0D with bit 7" trigger fire on a TX FIFO mismatch of a base
    command (sub 0xFD). A sub-byte validity filter avoids it (section 7).
  - "No frame for N s" does not work as a trigger: a base in FAILED still
    hears a stray frame about every 25 s at 2 fps.
  - **Never on the tractor:** re-activation restarts the originator's grid
    at epoch 0 (SIL-E3w: renumbered by up to 602 slots).
  - FAILED stays absorbing between firings: windows of at least 2 min plus
    re-acquisition without receiver sync.
  - The tractor twin is untouched.

### (d) A scan that catches more often

- **What:** `SX1276_RX_SCAN_DWELL_MS` 500 → 2000 (d2) or 10000 (d10), and
  the walk continues across attempts instead of restarting at channel 0.
  With (b) it also continues across (b)'s two restarts, so a silent node's
  successive attempts cover all 50 channels (b+L+d2: 15 new channels per
  attempt; b+d10: 3). d10r keeps the restart.
- **For:** fewer cold starts end in FAILED (table). At 1 fps d10r is best
  (6.0 %), but it only ever covers channels 0-2.
- **Against:** alone it does not recover leg 6 (E1 0/100): FAILED still
  absorbs. d10 breaks reverse-delivery scenario 6, whose 6 s window is
  shorter than a 10 s dwell. A full 50-channel sweep takes 100 s at d2 and
  500 s at d10 (25 s today), which is slower to route around a jammed
  block. The 2026-05-19 analysis rejected "adaptive dwell" (§13.3); this is
  a fixed constant, and plan §7 Q6 names adaptive scan dwell as the
  redesign path once acquisition exceeds 30 s.
- **Decay window:** the dwell also runs on the tractor whenever its
  authority has decayed, so (d) changes its normal pre-FAILED behaviour
  (table, "decay window" row): with L, fewer passes under authority, more
  commands received and more own-grid renumberings. Section 5, item 3.
- **Ceiling:** a 10 s dwell already reaches the single-channel bound. An
  unaligned 10 s window holds 0, 1 or 2 occurrences of its channel (1 on
  average), so at 2 fps about 0.37 per dwell and 0.75 per 3-dwell attempt
  (SIL 0.73-0.81). More needs the transmit side (sync frames in empty
  slots: about 0.83 per dwell) and a regulatory review.
- **Regulatory:** receive side only. A receiver's dwell is not the
  §15.247 channel-occupancy limit.

## 5. Recommendation (for the design owner, Dorkmo)

This was a recommendation, not a decision. **Decision, 2026-10-24:** the
design owner adopted item 2 as written (b + L + d2, the park command and
the 0x0D split in one image) and accepted the decay-window trade-off of
item 3 (more own-grid renumbering for more commands received; closing the
decay window stays the open RS-12.15 grid-policy change). Recorded as
[`DECISIONS.md`](../../DECISIONS.md) D-FHSS1. Implemented 2026-10-25 on
branch `fhss-scan-recovery-2026-10-24`, built, and **flashed on both bench
boards 2026-10-25** (section 6.1):

- scan: (b) in `radio/sx1276_rx.c` (the pure policy is unchanged, vector 16
  still pins "FAILED absorbs" there), L in `scan_drive`, d2 as a fixed
  2000 ms `SX1276_RX_SCAN_DWELL_MS`; the acceptance check is
  `check-rx-scan-failed-follower` (name kept), and the shared simulation
  now checks (b)'s rule, the walk order and L in every reverse-delivery
  scenario. Its `-e` / `-e3` output on the real sources is byte-identical
  to the b+L+d2 prototype's, so the figures in this section and in
  section 4 stand for the built image;
- park: `HOST_TYPE_RADIO_PARK_REQ` 0x04, answered by `RADIO_PARK_URC` 0x84
  {status 0 PARKED / 1 already parked / 2 refused, RegOpMode readback};
  VER/READY capability bit 7 (`HOST_CAP_RADIO_PARK`, bitmap 0xFF; 0x7F
  before); pinned by `check-radio-park`;
- fault codes: section 7's proposal, items 1-3 (TX_FIFO_RB_MISMATCH on
  0x0F; `HOST_PROTOCOL_VER` and `HOST_WIRE_SCHEMA_VER` unchanged, the
  contract of item 5);
- hosts: `radio_park.py` (the firmware path on a bit-7 image),
  `rs12_leg_report.py` / `rs13_probe_score.py` (`--scan-image`: on this
  image a final 0x0D is a 30 s report, not FAILED), the 0x0D decode and
  the `-InjectOn fault:0x0D:final` rule of item 4 (section 7).

1. **Now, no radio, no flash.**
   - Make the base's scan visible: log FAULT URCs in `image_rx_daemon.py`
     with the decoded sub-byte, and on the tractor outside TX waits too.
     Add the acquisition check to the leg report (R:583-590).
   - Decode 0x0D with the validity filter of section 7, in the shared
     `format_fault_payload` and in `run_live_radio_monitor.ps1`
     `Test-InjectFaultLine`.
   - Write the pass-rule clause for a leg whose base never locks before
     the next FHSS round (R:603-605). At 2 fps the walker model gives a
     six-leg round a 34-40 % chance of needing it (section 3).
2. **Firmware, one flash GO:** (b) + L + d2 for the scan; the RS-12.21
   park command in the same image (b needs it on the bench); and the 0x0D
   split (section 7). L and d2 also change the tractor's decay window
   (item 3); decide on that first. Before the flash: replace the
   characterisation with (b)'s acceptance check, which needs the harness
   changes in section 4 (b).
   - The recommended scan, b+L+d2, in the SIL: leg-6 replay 100/100,
     median 17.9 s, p90 56.3 s, 99.8 % heard (2750 ppm: 100/100, 99.9 %);
     cold start 1.3 % at 2 fps (600/600 by 300 s), 10.7 % at 1 fps
     (598/600); tractor twin: authority back in 50/50.
   - Why (b) over (a): no change to the pure policy contract, none of the
     shipped checks regresses (the harness needs updating), no reliance on
     a host re-arm, and the tractor twin is covered. (a) alone is faster
     than (b) alone (median 17 s against 35 s; with d2, (b) takes 17.9 s,
     about the same as (a)) but needs the receiver kept armed in FAILED
     and changes the 2026-10-18 expectations. (a) is the fallback if the
     park command cannot be built first.
   - Why d2: it cuts 2 fps cold-start failures from 6.7 % to 1.3 % and
     1 fps from 23.2 % to 10.7 % with every existing check green. With L
     it also moves the tractor's decay window (item 3). d10 is better at
     2 fps (0.3 %) once scenario 6 is rescaled, and moves the decay window
     slightly further (table); revisit then.
   - Why not (c): a host round trip, never usable on the tractor, FAILED
     stays absorbing between firings, and the tractor twin stays. Keep it
     only as a stop-gap if the flash is far off, on the base only, keyed on
     a validated final 0x0D.
3. **RS-12.15:** the open question "should self-follow also run in FAILED"
   becomes: should FAILED exist at all for a node without authority. (b)
   answers no. The decay-window question stays open, but the recommended
   image moves the decay window it starts from. SIL-E3 before FAILED
   (commands from 5 s, 2.5 s stalls every 9 s, 300 s), 200 runs: n in
   `run_eval_tractor()` raised from 50 to 200 in a scratch copy of the
   SIL, nothing else changed, so the first 50 seeds are the `-e3` ones:

   | image | authority (share of passes) | commands received | own-grid renumberings per run (max slots) | LOCKs / demotions |
   |---|---|---|---|---|
   | main | 22.1 % | 53.2 % | 6.9 (17) | 7705 / 7580 |
   | L alone (= b+L) | 21.5 % | 54.3 % | 6.9 (16) | 7403 / 7266 |
   | d2 without L | 22.0 % | 52.4 % | 7.9 (16) | 12 905 / 12 780 |
   | b+L+d2 (recommended) | 14.4 % | 59.1 % | 8.6 (17) | 7258 / 7103 |

   Phase restarts (`tx_first_anchor` > 1) occur in every run of every
   image. Out of FAILED, b+L+d2 lands at the same level (14.3 %, 59.7 %,
   8.8 per run; 200 runs). The 50-run figures are in the section 4 table.
   - How: the 2 s dwell alone adds renumberings (+14 %) and makes many
     LOCKs demote on the next tick (LOCKs ×1.7, the d10-without-L effect).
     L removes those early demotions, and the tractor is then more often on
     an adopted grid (adopted at the end of 152 of 200 runs, 120 on main):
     5.9 points more commands received, 7.7 points less authority, 24 %
     more own-grid renumberings, each still up to 17 slots as on main.
   - Not modelled: the SIL base is an ideal follower that re-anchors on
     every tractor frame. What a renumbering costs the base's camera
     reception (the RS-12.12/14 lock-loss mechanism) is not in these
     numbers; command delivery is.
   - Whether the recommendation stands is the design owner's call: it
     trades more own-grid renumbering for more commands received in the
     decay window. If that is not acceptable before the decay window is
     closed, b+L at a 500 ms dwell stays near main's decay window (above)
     and still recovers leg 6 (E1 as (b): 100/100, median 34.8 s, 97.6 %
     heard), but keeps main's cold-start rate (2 fps 6.7 % instead of
     1.3 %). Closing the decay window itself remains the open RS-12.15
     grid-policy change.
4. **Clock (section 8):** no change needed for the fix. Record the measured
   HSI16 rates in `include/sx1276_fhss_clock.h` and size the head start and
   any leading-grid tolerance against them. Whether to clock the ms tick
   from the TCXO is a separate decision.

## 6. What needs a GO, and the radio tests

Every firmware change needs a flash GO for both L072s and new bench and
production md5s. Each radio test needs its own GO. Daemon changes need a
deploy to the X8s, which is also the operator's call.

| test | firmware | what it shows | pass |
|---|---|---|---|
| T1 forced FAILED | main, after the FAULT logging is deployed | H1 directly: start the base, keep the tractor silent ≥ 125 s, then stream at 2 fps for 180 s (needs a harness option to delay the tractor's stream; not built) | four base 0x0D (final one sub bit 7 set) logged; about 1 frame in 50 heard; Δaligned 0; Δ`clk_demotion_reset` 0; post `radio_state` 4 |
| T2 cold-start set | main | the FAILED rate and first-frame times (R:618-620) | about 10 starts of ≥ 150 s; compare with section 3 (at ~7 % per start, 10 starts see a FAILED about half the time, so T1 is the stronger test) |
| T3 fix A/B | fix | T1 and T2 on the new image | T1: LOCK a median ~35 s after the stream starts or sooner, ≥ 97 % heard afterwards (SIL-E1: b alone median 34.8 s, 97.6 %; b+L+d2 median 17.9 s, 99.8 %); 0x0D every 30 s while silent; the park command parks both boards (0x80 holds) |
| T4 tractor twin | main, then fix | a 2026-10-18 leg-X shape (scan FAILED first) with 2.5 s camera stalls and commands; on the fix, the same stalls before FAILED too (the decay window, section 5, item 3) | main: commands received drop after the first stall with a command heard in it; fix: tractor `clk_demotion_*` counters move and delivery returns to the fix image's own pre-FAILED level (SIL-E3, b+L+d2: 59.7 % out of FAILED, 59.1 % before it, main 53.2 % before it) |
| T5 fault codes | fix | the 0x0D split | scan faults arrive as 0x0D only; `tx_fifo_rb_bad` 0 and no 0x0F |

### 6.1 Flash and verify plan (the next radio day)

**Flown 2026-10-25** on the operator's GO ("proceed with any merges and you
have a GO for any radio testing"), from main `d597b094`, as written below
([`RESULTS.md`](../../bench-evidence/bench_session_2026-10-25/RESULTS.md),
`firmware_flash/` and `legs/` beside it). Steps 1-10 passed: tractor then
base flashed (`Verify OK`, `flash_rc=0`), `radio_park.py` `PARK_OK` with
`cap_bitmap` `0x000000FF` and `path` firmware on both, `0x80` still at the
check-only about 3 min later. R1 and R2 passed every Confirm item below
that applied (R1's base-0x0D item was not exercised: the base locked in its
first attempt). Both boards now run bench `0daae279`; step 11 was not
needed. T2 flew
later that day (the "Later" item at the end of this section). Not flown
yet: T1, T4 with stalls.

Written 2026-10-25 with both radios off, for the round after it. Both
boards run `e083ae46` until step 6. Each step that flashes or opens
`/dev/ttymxc3` needs the operator's GO: the flash GO covers steps 6-10
(steps 4-5 touch no radio beyond the power-up guard's read and park, the
GO rule's exception), and the rollback and each leg need their own. Commands and variables (`$BT`, `$BASE`,
`$TRAC`, `$R`, `$BI`, `$TI`) as in
[`BENCH_QUICKSTART.md`](../x8_lora_bootloader_helper/bench_tools/BENCH_QUICKSTART.md)
§3, §5 and §6; flashing in
[`FLASH_RUNBOOK.md`](../x8_lora_bootloader_helper/FLASH_RUNBOOK.md).

**Before the GO (no radio).**

1. Fly from a checkout that holds this work (the merged `main`, or this
   branch), with its `bench.env` and a new dated `EVIDENCE_DIR`. The
   harness pushes the daemons and probe helpers from the checkout it runs
   in, so the FAULT decode, the base's VER capability line and
   `radio_park.py`'s firmware path ride with it.
2. PowerShell, in `firmware/murata_l072`: `mingw32-make check` (every
   target `[PASS]`, `check-radio-park`, `check-rx-scan-failed-follower` and
   `check-rx-reverse-delivery` among them), then `mingw32-make clean`,
   `mingw32-make`, `mingw32-make bench`. Want bench
   `build/firmware_bench_diag.bin` md5 `0daae27978089eb97ab35c18718b62b0`
   and production `build/firmware.bin` `2c5e055abff95126309a62409fafc654`,
   25 324 B each (section 9). Another md5 means another toolchain or
   source: stop.
3. `powershell -NoProfile -File bench_tools\test_run_live_radio_monitor.ps1`
   (in `x8_lora_bootloader_helper`): 0 failed.

**Flash (the flash GO; a flash is a radio-on event).**

4. Power-up guard and staging as every session (`legs/power_up_guard.sh`,
   `legs/stage_boards.sh`). On `e083ae46` the new `radio_park.py` reports
   `"cap_bitmap": "0x0000007F"`, `"park_cmd": "unknown_type"` and
   `"path": "register"`: the fallback, as expected (so it read on
   2026-10-25).
5. Stage and preflight each board without `--go`:
   `bash "$BT/flash_l072.sh" tractor "$FW"`, with
   `FW=$DC/firmware/murata_l072/build/firmware_bench_diag.bin`. Want the
   `image:` line `25324 B  md5 0daae279...` followed by `RS-12.21 bench
   build (...)`, then `PREFLIGHT-OK`. The same for `base` (over ssh).
6. Flash the tractor, then the base, one at a time:
   `bash "$BT/flash_l072.sh" tractor "$FW" --go`, then `base`. Success:
   `Verify OK`, `flash_rc=0` and `DONE: prep=0 flash=0 revive=0` in
   `/home/fio/pipeline_stamped.log`; the board reboots in 12-24 s and
   `/tmp` is wiped. `PIPELINE-EXIT=0` is not in that log:
   `run_flash_bench.sh` prints it to the console after the pipeline
   returns, and on the tractor the console misses it because adb drops at
   the reboot (2026-10-25; the base's console had it). The base's
   `flash_l072.sh` can hang on its dropped ssh session after the board
   reboots: once the board answers on a new connection, stop it (its exit
   status, 255 then, does not judge the flash). Keep the consoles and a
   copy of the image in a `firmware_flash/` folder of the session, as on
   2026-10-18
   (`bench-evidence/RS_12_15_reverse_delivery_2026-10-18/firmware_flash/`).
   Never put the production bin on a bench board.

**After the flash (same GO; nothing transmits).**

7. Stop the tractor's camera unit and container, `fuser /dev/ttymxc3`
   empty, and re-stage `/tmp/lifetrac_strict` (`legs/stage_boards.sh`).
   Do not use `legs/power_up_guard.sh` here: it parks the radio, and
   steps 8 and 9 expect an unparked one. (If it was used anyway, step 8
   reads `radio_state=1` (SLEEP) and step 9 reports
   `"park_status": "ALREADY"` with both readbacks `0x80`; that is also a
   pass.)
8. `rs116_health_probe.py` on both: `STATS-OK`,
   `RS115-INSTRUMENTED-FIRMWARE=YES`, `RS12-URC-COUNTERS=YES`,
   `RS12-10-COUNTERS=YES` (the STATS layout did not change),
   `tx_fifo_rb_bad` 0, `radio_state=4`.
9. Image and park in one step: `radio_park.py` on both. VER_URC still says
   `0.0.0` / `dev`; the image shows in its capability bitmap. Want the last
   line `PARK_OK` with `"cap_bitmap": "0x000000FF"`, `"path": "firmware"`,
   `"park_status": "PARKED"`, `"persistent": true` and both readbacks
   `0x80`. `"path": "register"` or `0x0000007F` means the old image is
   still on the chip: stop.
10. The park holds: at least 2 min after step 9, read both with the
    read-only `radio_state.py` (or `legs/power_up_guard.sh --check-only`):
    `0x80`. Right after a flash the L072 is on its boot profile (fixed
    915 MHz, FHSS not initialised, no scan), so this proves the command
    and the image. The test of the gate against the scan (T3's park
    clause) is R1's leg-end park below, with profile 1 still active: on
    `e083ae46` such a park held only after the scan had failed out, and on
    this image the scan of a node without authority never stops. Anything
    but `0x80` after a firmware `PARK_OK` is an arm path the gate missed:
    stop radio work and power both boards down (tractor `systemctl
    poweroff`; base `systemctl poweroff`, then remove its power).
11. Rollback, only if needed and on its own GO: the flown `e083ae46` copy,
    `bench-evidence/RS_12_15_reverse_delivery_2026-10-18/firmware_flash/firmware_bench_diag_e083ae46.bin`,
    with `flash_l072.sh`.

**First legs (each on its own GO).** Each leg's daemon start activates
profile 1, which ends the park; `leg_post.sh` parks again at the end.

- **R1: cold-start acquisition and the fault codes (T3's cold start, T5).**
  The 2026-10-24 command-delivery shape: `legs/leg_prep.sh 2c_<tag>
  image_bw250 --reactive-fire 1` (`mono_g4`, profile 1, 2 fps, 300 s,
  `-ProbeEcho 0 -TxBatch 0 -LogFragArrivals 1 -StopDrainS 4`), then
  `legs/leg_post.sh`. Confirm:
  - the base's `rx_daemon.log` has `L072 VER warm-up ok (attempt N)
    cap=0x000000FF`;
  - `leg<X>_report.txt`: the `acquisition:` line within 120 s (2026-10-24
    legs 1-5: 15-75 s; SIL-E2 at 2 fps: 1.3 % of cold starts not locked
    by 120 s) and `confirmed acquisition: ACQUIRED`, with the image read
    from the capability line (no "image not identified" note);
  - every `code=0x0D` line on either board is `name=RX_SCAN_FAILED` with
    its attempt fields, none carries `fw=old` (a pre-split FIFO fault);
    no `code=0x0F` line and Δ`tx_fifo_rb_bad` 0 on both (the FIFO fault
    has never fired; a first 0x0F must match a `tx_fifo_rb_bad` step);
  - a base 0x0D, if any, comes before its first lock, about 30 s apart; a
    final one (attempt 3) is followed by more finals 30 s apart until the
    lock (scan option (b)), and the report reads it as a late
    acquisition, not FAILED;
  - `leg<X>_deltas.txt`: no `!! BASE NEVER LOCKED`, base
    Δ`clk_demotion_reset` ≥ 1, the RS-12.15 criteria as on 2026-10-24,
    probe delivery in line with that round (`mono_g4` 542/543);
  - `leg<X>_park.txt`: `PARK_OK` with `"path": "firmware"` on both at the
    first attempt (no 60 s retries), and `power_up_guard.sh
    --check-only` at least 2 min later reads `0x80` on both (T3's park
    clause: profile 1 is still active, so without the gate the scan would
    re-arm RXCONT every 2 s dwell);
  - STOP: `!! ACQUISITION FAILED` or `!! BASE NEVER LOCKED` on this image.
    Park, do not re-fly, and read the base's 0x0D lines first.
  - **Flown 2026-10-25 as `2c_r1`, every item that applied PASS**
    (RESULTS, R1): base
    `cap=0x000000FF`; ACQUIRED 11.0 s after activation (599 of 609
    fragments); no 0x0D or 0x0F line on either board during the leg (both
    scans locked in their first attempt, so the base-0x0D item was not
    exercised), Δ`tx_fifo_rb_bad` 0; base Δ`clk_demotion_reset` 1, the
    RS-12.15 criteria PASS, PROBE 204/204; leg-end `PARK_OK` on the
    firmware path at the first attempt on both, `0x80` at the check-only
    2 min 11-16 s later: the park gate held with profile 1 active. The
    post-bracket probe caught a final 0x0D (sub 0xE3) from the tractor 5 s
    before that park: its scan was running, as (b) predicts.
- **R2: the injector trigger and the tractor in FAILED (T4's first
  half).** The 2026-10-18 leg-X shape: profile 1, 480 s, KF requests off:
  `legs/leg_prep.sh 2c_<tag> image_bw250 --duration 480`, with
  `--reactive-fire` left at 0 (the base's PROBEs would lock the tractor's
  scan before it fails out; `leg_prep.sh` already writes
  `-KfRequestDisable 1`), and `-InjectCmd "dual_inject.py 120" -InjectOn
  fault:0x0D:final` added to the harness line it prints. Confirm: the tractor's 0x0D `name=RX_SCAN_FAILED`
  lines for attempts 0, 1 and 2, then attempt 3 final, about 30 s apart
  (leg X: 0x00 0x01 0x02 0x83); `INJECT ... trigger: fault:0x0D:final` on
  that attempt-3 line and on no earlier one; the base's commands heard with
  the tractor in FAILED (leg X: 36/36); the RS-12.15 bracket criteria as
  on 2026-10-18. A tractor that keeps streaming holds FAILED (it has
  authority). If its stream stalls for 1 s or more it restarts its scan
  under (b), so on this image a later final 0x0D and moving
  `clk_demotion_*` counters are normal. **Flown 2026-10-25 as `2c_r2`,
  every item PASS** (RESULTS, R2): tractor 0x0D `name=RX_SCAN_FAILED` sub
  0x00, 0x01, 0x02, 0x83 at L072 uptime 33.6, 63.6, 93.6, 123.7 s; the
  trigger fired on the 0x83 line only, 113 s after MONITORING; 93 of the
  base's 95 commands heard with the tractor in FAILED (leg X: 36/36); the
  RS-12.15 criteria PASS, stop rule not tripped; no later 0x0D, tractor
  `clk_demotion_*` 0 (it streamed to the end and held FAILED).
- **Later, not on the first day:** T1 (the base silent for ≥ 125 s, then
  the stream) needs a harness option that delays the tractor's stream,
  which is not built (build and test it offline first); T2 (a set of about
  10 cold starts); T4 with camera stalls. **T2 flown 2026-10-25 on the
  operator's GO, PASS**
  ([`RESULTS.md`](../../bench-evidence/bench_session_2026-10-25_t2/RESULTS.md)):
  ten 180 s FHSS legs, `mono_g4` and VECTOR alternating, `-ReactiveFire 1`;
  every start `confirmed acquisition: ACQUIRED`, first frame 11.0-62.4 s
  after activation (median 11.1 s; 0 of 10 past 120 s, so no final 0x0D
  and no (b) restart on the base); one start locked in attempt 2 after
  base 0x0D attempts 0 and 1, 30.0 s apart; every leg-end park `PARK_OK`
  on the firmware path, `0x80` on both at the check-only 2 min 26-31 s
  later. The times fall on three values (11.0-11.1, 26.4-26.9 and 62.4 s;
  all five VECTOR starts 11.1 s): the harness starts both boards the same
  way each leg, so T2 sampled a few start phases, not the random-phase
  rate SIL-E2 predicts (1.3 % past 120 s at 2 fps). Measuring that rate
  on the bench needs a random delay between the tractor reset and the
  base's activation, which no harness option does yet.

After the round: record it in the session's RESULTS.md, then update the
"on the boards" records (`murata_l072/README.md` "Expected md5s",
`FLASH_RUNBOOK.md`, `BENCH_BOARDS.md`, `BENCH_RUNBOOK.md` "Board facts",
`flash_l072.sh`'s build list) and the TODO.md RS-12.21 and RS-12.15 rows.
End the session with both radios parked by the firmware path and the
`--check-only` reading. Done 2026-10-25 (the session ended with both radios
at `0x80` at 15:02:35Z / 15:02:55Z).

## 7. Side item: FAULT code 0x0D means two faults

`HOST_FAULT_CODE_TX_FIFO_RB_MISMATCH` and `HOST_FAULT_CODE_RX_SCAN_FAILED`
are both 0x0D (`include/host_types.h:257`, `:280`).

**Emitters:**

| code | emitted at | when | sub |
|---|---|---|---|
| TX_FIFO_RB_MISMATCH | `radio/sx1276_tx.c:574` | FIFO checksum after TX_DONE differs from the load-time one | the host's tx_id: tractor fragments `idx & 0xFF` (`firmware/tractor_x8/image_tx_daemon.py:1226`, `:1329`), tractor command frames 0xFE (`:980`), base commands and PROBEs 0xFD (`base_station/image_rx_daemon.py:841`) |
| RX_SCAN_FAILED | `radio/sx1276_rx.c:665` | each failed 30 s scan attempt | bit 7 final, 6 warm, 5 hw_suspect, 4 crc_seen, bits 3-0 attempt (`include/host_types.h:261-279`; built in `radio/sx1276_rx_scan_fail.c`) |

**History.** RX_SCAN_FAILED took 0x0D on 2026-05-20 (`d4dfcb86`; the
2026-05-19 analysis §13.1 called 0x0D "the verified next free slot").
RS-11.5 took 0x0D again on 2026-08-02 (`87821c90`). The RX_SCAN_FAILED
comment still says "no code emits it yet", which may be why. `host_types.h`
checks HOST_TYPE_* values for duplicates at compile time but not fault
codes.

**What the evidence holds.** 240 `code=0x0D` lines in the committed logs
(some logs are archived twice), 16 distinct sub-bytes, all valid scan
encodings (0x00-0x02, 0x20-0x22, 0x40-0x42, 0x60-0x62, 0x83, 0xA3, 0xC3,
0xE3). All 331 `tx_fifo_rb_bad` readings in `bench-evidence/` are 0. The
FIFO fault has never been counted.

**What the hosts do with 0x0D today:**

- Base `image_rx_daemon.py:1337-1340`: drops every FAULT URC at DEBUG.
- Tractor `image_tx_daemon.py:1443-1448`: logs non-benign faults during the
  pipelined TX as `code=0x0D sub=0x.. raw=..`, undecoded; the serial path
  prints them in `wait_for_tx_done` and ignores them. Outside TX waits
  they are not logged (R:583-585).
- `format_fault_payload` (`firmware/x8_lora_bootloader_helper/method_g_stage1_probe.py:487-523`)
  decodes 0x09-0x0C only.
- `run_live_radio_monitor.ps1` `Test-InjectFaultLine` (`:326-341`):
  `-InjectOn fault:0x0D:final` tests sub bit 7. Its comment says only
  RX_SCAN_FAILED sets bit 7 because fragment indexes stay under 0x80. The
  tractor's command tx_id 0xFE also has bit 7: a FIFO mismatch on one of
  those would fire the injector.
- H7 host: stores and prints code and sub, no decode
  (`firmware/tractor_h7/murata_host/mh_runtime_health.c:48-55`,
  `mh_runtime.c:94-98`).
- SIL checks key on the symbolic names, so they follow any renumbering.

**Ambiguity.** A valid scan sub has attempt 0-3, final set exactly when
attempt is 3 (or the 0x80 null-input marker), and never hw_suspect with
crc_seen (`radio/sx1276_rx_scan_fail.c:37`). That rejects 0xFD and 0xFE. It
cannot separate a FIFO fault on fragments 0-2 (subs 0x00-0x02) from a
non-final scan attempt.

**Proposal (a wire change, so a design decision):**

1. Keep 0x0D for RX_SCAN_FAILED: it has the structured sub, every logged
   0x0D is one, and the bench trigger and the 2026-10-18 evidence use it.
2. Move TX_FIFO_RB_MISMATCH to 0x0F, the next unused code (0x0E is
   TX_DONE_EARLY). Its counter `tx_fifo_rb_bad` stays the primary signal.
3. Add a compile-time duplicate check for HOST_FAULT_CODE_*, like
   `host_type_compile_time_uniqueness_check`, and fix the stale comment.
4. Hosts first, before the flash: decode 0x0F as the FIFO fault; decode
   0x0D as RX_SCAN_FAILED when the sub is valid, else as "0x0D, ambiguous
   (old-firmware FIFO fault, tx_id = sub)". Gate on the firmware version
   (the md5 in `deploy_md5.txt`, or VER_URC) to drop the ambiguity on new
   images. Tighten `-InjectOn fault:0x0D:final` to "attempt 3 and a valid
   sub" now.
5. The FAULT_URC frame format is unchanged; only a code value moves, as in
   the 2026-05-19 §13.1 #7 contract. Both md5s change: flash GO, md5
   tables (`.github/workflows/l072-bench-md5.yml`, `README.md`).

**Adopted 2026-10-24, implemented and flashed 2026-10-25.** Items 1-3 as
proposed (`include/host_types.h`; the duplicate check covers every
HOST_FAULT_CODE_*). Item 4: the hosts name a 0x0D with a valid scan sub
RX_SCAN_FAILED on any image and keep the in-flight test only as a `basis=`
token; any other 0x0D sub is an old image's FIFO fault and gets `fw=old`;
0x0F is TX_FIFO_RB_MISMATCH. The version gate is the VER capability bit 7
(`HOST_CAP_RADIO_PARK`), which the same image added with the park command;
`-InjectOn fault:0x0D:final` fires on a valid scan sub at attempt 3 only.
Item 5: no `HOST_PROTOCOL_VER` / `HOST_WIRE_SCHEMA_VER` bump.

Moving RX_SCAN_FAILED instead would touch more host code and leave every
archived 0x0D needing version-gated decoding.

## 8. Side item: the L072 clock

### 8.1 What the design assumes and what runs

- `include/sx1276_fhss_clock.h:15-19`: "both L072s clock from ±2 ppm
  TCXOs", so drift is "≈ 0.12 ms per minute".
- `hal/platform.c:132-160`: SYSCLK is always HSI16. The HSE (TCXO) is
  probed for reporting only (W1-9d, 2026-05-12: SYSCLK on HSE doubled the
  USART1 kernel clock and broke 921 600 baud). SysTick, the ms tick, runs
  from that clock (`:166-172`).
- `BOOT_URC.clock_source_id` reports only whether HSE came ready
  (0 = HSE_OK, 1 = HSI fallback; `include/platform.h:9-11`). Either value
  means an HSI16 tick. The diag trace still prints `C:CLK_HSE_32M` for 0
  (`host/host_cmd.c:744-748`), which reads as if the tick used the TCXO.

### 8.2 BOOT_URC in the committed evidence

- 2026-05-09: five decoded lines, all `clock_source_id=1`, `radio_ok=0`
  (`T6_phase9_halt_timing_2026-05-09_191923/47..50_*_stage1_probe.txt`,
  `T6_phase9_backfill_cfg47_2026-05-09_210323/47_*`). Before the PB6
  TCXO-enable fix.
- 2026-05-11: raw payloads `000112010101` (id 1) in the `W1-9_stage2_tx_*`
  and `W1-9b_*` folders, and the trace `C:CLK_HSI_16M` in
  `T6_bringup_2026-05-11_*`.
- 2026-05-22: `000112010200` (id 0, HSE_OK) from board 2E2C1209DABC240B,
  today's tractor (`W1-10b_rx_pair_2026-05-22_201518`), and from one side
  of `W2-02_image_over_lora_2026-05-22_185126`; `000112010201` (id 1) from
  the other side.
- 2026-05-25: `000112010201` on both boards with FAULT 0x08 sub 0x01
  (CLOCK_HSE_FAILED) (`rx_pair_nrst_20260525_120455`,
  `W1-10b_tx_burst_2026-05-25_083436`).
- Nothing later. The current harness drains the boot and does not log it.
  No committed record shows the clock source of the boards as flown since
  May. It would not change the tick either way.

### 8.3 Measured rates

From the 27 legs (C, `clock_run.txt`). Negative means the base L072 is
slower.

| rate | 2026-10-24 legs | all 27 legs |
|---|---|---|
| base L072 vs base host (`fw_us`) | −610 to −1399 ppm | −1979 to +603 ppm |
| tractor L072 vs tractor host (slot period) | +1324 to +1875 ppm | −386 to +2496 ppm |
| base L072 vs tractor L072 | −2366 to −3210 ppm | −4122 to +871 ppm |

The last row regresses the base's `fw_us` at each catch on the same
frame's tractor end time, then converts with the tractor's slot period.
A host-free method, `fw_us` against the frame's slot-grid time, gives
−2406 to −3169 ppm on 2026-10-24 (−4153 to +823 over 27 legs): within
41 ppm of the first on every 2026-10-24 leg. Leg 6's five catches alone
give −2299 ppm. The sign is not fixed: on three 2026-09-14 legs the base was
the faster one. A ±2 ppm pair drifts at most 4 ppm.

### 8.4 What the drift does

Drift of a free-running copy of the other node's grid (C):

| free-run | 2026-10-24 (2366-3210 ppm) | archive max (4122 ppm) | two ±2 ppm TCXOs |
|---|---|---|---|
| 0.5 s (2 fps frame interval) | 1.2-1.6 ms | 2.1 ms | 0.002 ms |
| 1 s (authority streak gap) | 2.4-3.2 ms | 4.1 ms | 0.004 ms |
| 2 s (`LOCK_LOSS_MS` = `CLOCK_FRESH_MS`) | 4.7-6.4 ms | 8.2 ms | 0.008 ms |
| until 4 ms (12 ms head start minus its ~8 ms retune and skew estimate, `include/sx1276_fhss_clock.h:70-78`) | 1.2-1.7 s | 1.0 s | 1000 s |
| until 15 ms (TX fit guard) | 4.7-6.3 s | 3.6 s | 3750 s |
| until 100 ms (half a slot) | 31-42 s | 24 s | 25 000 s |

- **Slot follower (LOCKED):** it re-anchors on every accepted frame and
  demotes after 2 s without one, so drift is bounded at 4.7-6.4 ms. At 2 fps
  with no loss it is 1.2-1.6 ms. After 2-3 lost frames it passes the ~4 ms
  the head start leaves, so a slow base may retune after the tractor's
  preamble starts. The ±2 ppm assumption hides this; the margin is thin,
  not gone.
- **PROBE and command timing:** the base keys on its own copy. In LOCKED
  the copy is at most 2 s old, inside the 15 ms guard. In FAILED it
  free-runs between SNAPPED catches for tens of seconds (leg 6, under the
  H1 split of section 8.5: 26.3 s and 68.0 s), well past the guard.
- **RS-12.15 originator decay:** the tractor's own grid is the reference,
  so its decay itself has no drift. A follower's copy is reset at the 2 s
  demotion. But the sign can flip (above): a base faster than the tractor
  makes its copy lead, by 0.9 ms after 1 s at the archive's +871 ppm, and
  by 4 ms at a 4000 ppm rate of that sign. The open "lead tolerance" in the leading-grid
  test (SIL margin ~0.5 ms, RS-12.15 row) should be sized against that, not
  against TCXO drift.

### 8.5 Leg 6's PROBE labels fit the drift

RESULTS left PROBE 3's miss unexplained and found drift "possible but not
supported by the timing" (R:486-495). That reading used the PC-to-board
offsets (±0.03 s). This re-analysis (C) uses only base-side times: the
header slot offset 14 ms, each frame's ToA from the tractor log, and the
base's 49 ms from `frag_arrival` to the PROBE's "OK (on air)" (all five).

**Inferred, not logged.** The base logs only the decision totals: 3
SNAPPED, 2 EPOCH_DRIFT (R:331-337). Which catch got which is not logged
per frame; any 3/2 split fits the totals (R:385-388). The split below is
the one H1's frozen-epoch rule gives (section 2.3, SIL check 4): a catch
within one epoch of the scheduler epoch frozen at the last own TX or SNAP
is SNAPPED, two or more is rejected. Catch 1 (13,17) is the first, so
SNAPPED; catch 2 (14,40) is one epoch past PROBE 1 at (13,18); catch 3
(17,22) is three past PROBE 2 at (14,41); catch 4 (18,2) is one past
PROBE 3 at (17,22); catch 5 (24,43) is six past PROBE 4 at (18,3)
(slots: R:368-373).

- Under that split, PROBEs 1, 2 and 4 follow SNAPPED catches (fresh
  anchor). Each was labelled with the next slot and heard.
- PROBEs 3 and 5 follow EPOCH_DRIFT catches. The base clock was then 26.3 s
  and 68.0 s old (since catches 2 and 4). Each was labelled with the
  **same** slot as the frame just heard (observed). PROBE 3 was not heard;
  PROBE 5 was.
- With no drift, the firmware gate cannot produce that pattern for any
  common timing offset from 0 to 20 ms, even with ±3 ms per PROBE.
- With the leg's own measured rate (−2299 ppm) all five labels and
  outcomes fit for an offset of 8-14 ms. Any rate from −1125 to −2375 ppm
  fits.
- Under that fit, PROBE 3's base clock was about 60 ms behind. The base
  saw room left in the slot and keyed at about 194 ms of the tractor's
  slot. The PROBE ended about 20 ms past the tractor's boundary, where the
  tractor retunes: lost. PROBE 5's clock was about 156 ms behind; it was
  keyed near the slot head on the base's clock and ended within a few ms
  of the tractor's boundary: heard.

This rests on the inferred split above and on equal log latencies for the
RX and TX URCs up to the fitted offset. It does not explain the tractor's
one CRC error (R:490-492).

## 9. Files

- This memo.
- [`bench/host_proto/rx_scan_failed_follower.c`](bench/host_proto/rx_scan_failed_follower.c)
  and the Makefile target `check-rx-scan-failed-follower` (in `make check`).
  `-v` prints a run, `-e` / `-e3` the option evaluations.
- [`bench-evidence/fhss_cold_start_analysis_2026-10-24/`](../../bench-evidence/fhss_cold_start_analysis_2026-10-24/):
  `legload.py`, `load_all.py`, `risk_run.py`, `risk_summary.py`,
  `clock_run.py` and their `.txt` outputs; `sil_make_variant.py` for the
  option patches (a, b, L, d2, d10, d10r, combinations such as b+L+d2, and
  `noL` to leave L out). With b, d2 and d10 also continue the walk across
  b's two restarts (b+d10r and plain b restart at channel 0, as their
  retries do).
- Unchanged by the analysis of 2026-10-24: the bench image (md5
  `e083ae46a64686031b39866eca42e094`) and the production image (md5
  `148de24e1f9acce417c46c2c69a5be6b`), both rebuilt and checked that day;
  `e083ae46` stayed on both boards until the flash of section 6.1
  (2026-10-25); its flown copy is the rollback image (step 11).
- The adopted image (section 5, 2026-10-25; branch
  `fhss-scan-recovery-2026-10-24`, built, on both bench boards since
  2026-10-25): bench
  `build/firmware_bench_diag.bin` md5 `0daae27978089eb97ab35c18718b62b0`,
  production `build/firmware.bin` md5 `2c5e055abff95126309a62409fafc654`,
  25 324 B each (APP 25 324 / 188 416 B, RAM 6 748 / 17 920 B,
  `tools/check_size_budget.py` PASS), with arm-none-eabi-gcc 12.2.1 (Arm
  GNU Toolchain 12.2.MPACBTI-Rel1). The same md5s from a clean copy of the
  tree and again after `mingw32-make clean` in the worktree.
