# RS-13 range edge — VECTOR and mono_g4 over a conducted attenuator walk-down

*Procedure, 2026-10-10. Not flown. This is the next radio session after
RS-13.1, as the RS-13.1 review set it out
([RESULTS, "Remaining radio tests"](../../../bench-evidence/RS_13_vector_scene_2026-09-26/RESULTS.md#remaining-radio-tests-review-2026-10-04-after-the-round)).
It measures two rows of `VECTOR_SCENE.md` §8.6: the range-edge leg ("at each
3 dB step: VS frames received per second, level reached, command delivery,
and the same for `mono_g4`") and base command delivery during VECTOR ("not
worse than during `mono_g4`"). RS-13.1 measured neither.
[BENCH_RUNBOOK.md](BENCH_RUNBOOK.md) (prep, brackets, park, evidence) and
[RS13_VECTOR_LEG.md](RS13_VECTOR_LEG.md) apply unchanged. RS13_VECTOR_LEG.md
supplies the staging, the scene, the base capture, the leg reports and the
pass criteria P1–P8, as amended 2026-10-10. This file adds the RF path, the
steps, and what to record at each step. Results go into
`bench-evidence/RS_13_range_edge_<date>/RESULTS.md`, started from the
[template](#results-template) at the end.*

**Radios only on an explicit operator GO, given per round.** Every radio-on
item here needs that GO, including the receive-only spot-check (it wakes the
base radio). Nothing in this document is a GO.

## Safety and GO rules — read first

- **GO per round.** A GO covers one round (A, B or C below) as announced:
  its legs, its order and its stop time. A re-fly, an added leg or a changed
  plan needs a new GO. Between rounds the radios are parked.
- **Never key a radio with its antenna port open.** Whenever a board's
  daemons could run, its LoRa SMA jack carries one of three things: its
  antenna, its fixed 30 dB pad, or a 50 Ω load. The pads go onto the jacks
  first and stay there for the whole session.
- **Never join the two LoRa jacks through less than the fixed attenuation.**
  At the profile default of 14 dBm, a bare cable puts about +13 dBm into the
  other receiver. That is above the SX1276's +10 dBm absolute-maximum RF input.
- **Touch the RF hardware only with both radios parked.** This covers pads,
  step chain, cables, antennas and enclosure. Both boards must read
  `PARK_OK` and then `radio_state.py` `0x80`, with no daemon, camera unit or
  probe running on either board. A step change is a hardware change: make it
  between legs, never during one.
- **Do not rescue a step.** Do not raise TX power, change SF, BW or CR,
  re-enable keyframe requests or raise `LIFETRAC_KEYFRAME_COPIES`. The walk
  measures margin at the field settings: profile default 14 dBm on both
  boards, `-TxPowerDbm` left empty.
- **No `firefox --kiosk`**, and nothing fullscreen that the operator cannot
  leave with Esc (RS13_VECTOR_LEG.md, Scene).
- **Park last after every leg.** End every round with the read-only
  `radio_state.py` on both boards.

## What this session proves, and what it does not

Proves:

1. **At the 0 dB step (Round A, DTS).**
   - The §8.6 command-delivery row, VECTOR against `mono_g4` on interleaved
     legs.
   - The A16 fix confirmed on air at bw500.
   - P1–P5, P7 and P8 (amended) on a spot-checked carrier, not on the RS-11.6
     emitter's 915.000 MHz (RS-13.1 A19).
2. **Over the walk (Rounds B and C).** For VECTOR and `mono_g4`, at profile 2
   and profile 1, the walk records:
   - where each codec's frame delivery falls off;
   - how VECTOR's scene state (store in step, resync episodes, recovery)
     survives the same loss;
   - command delivery per step on DTS;
   - received RSSI/SNR tracking the attenuator.

Does not prove:

- **§8.6's "VS1 must keep an L0 picture and STATUS at attenuations where
  `mono_g4` delivers no complete frame".** That claim rests on the V1–V3
  degradation ladder (`VECTOR_SCENE.md` §4.5.4: smaller frames, a larger
  carousel share, repeats), which is not built. Every frame is V0 at detail 80.
  - At V0 both codecs send one fragment per frame.
  - VECTOR frames are fuller: p50 242 B against `mono_g4`'s 148 B on the
    railroad video (RS-13.1 A15), so they spend longer on air.
  - This walk is therefore the **V0 baseline**. It is expected to show
    VECTOR's frame delivery no better than `mono_g4`'s.
  - Record that criterion as "not testable at V0", and walk again when the
    ladder lands.
- **FHSS command delivery and the FHSS switch leg ("2d on profile 1").** These
  wait for the RS-12.15 reverse-delivery firmware fix. Base→tractor delivery
  on FHSS was 1/17 and 49/281 in legs H/I, so before the fix these legs would
  measure that defect, not VECTOR. The FHSS walk is image-only
  (`-ReactiveFire 0`, no base commands).
- **A distance.** A conducted path has no fading, multipath, foliage, antenna
  gain or height. The edge found here is a path loss at the bench's power. It
  serves to compare codecs and profiles; it is not a range figure.
- **The mode switch.** There is no switch leg (P6 n/a). RS-13.1 round 4 flew
  the DTS switch; the amended return clause gets its first test on the next
  2d leg flown.
- **The rest of the stack.**
  - The production stack: B1, RS-9.2/9.4, and a production-legal DTS carrier
    (RS-11.7). Both boards run the bench (register-write diag) L072 build.
  - `web_ui` in the loop.
  - Tractor self-select (D-VS6b), the loss-driven floor (B4) and AE/AWB lock
    (B5).

## Prerequisites — before asking for the GO

The RS-13.1 review put the session after the A16 fix and the code guards
(RESULTS, "Remaining radio tests"; TODO, RS-13).

| item | state needed |
|---|---|
| A16 fix | Merged. `scripts/a16_sil.py` on its SHA: A16-like **0/100 at both budgets** (bw500 243 B, bw250 203 B), recovery max ≤ ≈ 25 frames, `lossfree_out_of_step_frames` 0. Full `base_station` suite green. The summary JSON goes in the fix PR |
| code guards | Merged: codec-6 frames are never batched; the VECTOR byte budget is clamped to one fragment until `link_budget` arrives (2a_r4 started at `byte_budget=2436`); no double epoch start on VECTOR entry; no ≥ 1 s TX gap on a mode switch. Carousel counters in `vector_stats`, if landed, make the A16 confirmation direct |
| images | Tractor `lifetrac-tractor-x8:latest` and base `lifetrac-v25:latest` rebuilt from the merged SHA. The deployed base image `4623980c2dac` predates every VECTOR `web_ui`/store commit |
| step 0 / step 1 | RS13_VECTOR_LEG.md Steps 0–1 run **from the image's `/app`**, plus a **bw500 video pass**: the step-1 figures that P8 now records for information, per profile |
| staging | Fresh on both boards: under 5 days old, md5-checked against the working tree (RS13_VECTOR_LEG.md Step 0.3) |
| base services | `web_ui` and `lora_bridge` not running on the base. During VECTOR a stale-tile worker would put a `0x6C` on air at least every 10 s, and `-KfRequestDisable` does not stop it |
| RF hardware | On hand, assembled and recorded (below), all with the radios parked |

## Numbers to hold in mind

| | DTS profile 2 (`image_bw500`) | FHSS profile 1 (`image_bw250`) |
|---|---|---|
| TX power, both boards | 14 dBm (profile default) | 14 dBm |
| one-fragment VECTOR payload / full-fragment airtime | 243 B / 99.9 ms | 203 B / 169.1 ms |
| SF7 demodulation floor (SNR, `VECTOR_SCENE.md` §4.5.4) | −7.5 dB | −7.5 dB |
| sensitivity estimate: −174 dBm/Hz + 10 log₁₀ BW + NF 6 dB (assumed) − 7.5 dB | ≈ −118.5 dBm | ≈ −121.5 dBm |
| quiet-channel floor the base reports (raw RSSI, `channel_survey_sniff.py`) | ≈ −114 dBm | — |
| RS-13.1 over-air level of the tractor at the base | ≈ −64 dBm (A19) | — |

- **The 0 dB step's level.** The path plan below fixes about 80 dB of loss:
  a 30 dB pad on each jack, 20 dB in the middle, plus 1–2 dB of cables. That
  puts the base's median RSSI at the 0 dB step (**R0**, measured in Round A)
  near −67 dBm, close to RS-13.1's over-air operating point. The target is
  −65 to −75 dBm.
- **Where the edge should fall.** Expect it near step R0 − S, where S is the
  sensitivity estimate: about 51 dB (DTS) and 54 dB (FHSS) for R0 = −67 dBm.
  The step set reaches 66 dB, which leaves 12–15 dB past the expected edge.
  If R0 reads above −65 dBm, add 10 dB to the fixed middle and re-measure.
  Steps always count from the 0 dB step.
- **RSSI and SNR each work over only part of the range.** The SNR the base
  reports stops rising near +10 dB; RS-13.1 read 5–10 dB at ≈ −64 dBm. RSSI
  carries the signal there. Below SNR 0 dB the reported RSSI flattens toward
  the floor, and SNR carries the signal down to the −7.5 dB demodulation
  floor.
- **Leakage margin.** A leakage path adds a second copy of the tractor's
  signal at an arbitrary phase.
  - 10 dB below the conducted signal, it moves the received level by
    +2.4 / −3.3 dB.
  - 20 dB below, by +0.8 / −0.9 dB.
  - Near the edge, leakage must therefore sit at least 10 dB under the
    conducted level.
- **Probes.** On DTS legs the base fires one no-op PROBE per received frame,
  at most one per second. Two limits set the rate: the 0.5 s
  `LIFETRAC_PROBE_MIN_GAP_S` default and the RS-12.14 shared 1.0 s command
  gate while frames flow (`image_rx_daemon.py` `_maybe_fire_probe`). That
  gives roughly 150–300 probes per leg at the 0 dB step, and fewer where
  frames are lost.

## RF hardware

| part | spec | role |
|---|---|---|
| 2 × 30 dB SMA pad | 50 Ω, DC–≥ 1 GHz, ≥ 1 W | One screwed onto each board's LoRa SMA jack (Portenta Max Carrier, HIL_RUNBOOK §0), in place of its Taoglas TD.95.6H31 blade, for the whole session. Each PA then looks into ≥ 60 dB of return loss whatever happens downstream |
| 1 × 20 dB SMA pad | same | Fixed in the middle of the path all session (with the two jack pads: the ≈ 80 dB fixed loss) |
| step set: 3, 6, 12, 15, 30 dB SMA pads | same | Every multiple of 3 dB from 0 to 66 dB (table below). A manual step attenuator (DC–≥ 1 GHz, 0–≥ 60 dB in ≤ 3 dB steps, ≥ 1 W) may replace the set, switched under the same rule: radios parked, between legs |
| 50 Ω SMA cables, SMA barrels | low loss at 915 MHz; record type and length (an LMR-240-class cable for a run between rooms) | The path; a barrel is the 0 dB step |
| SMA bulkhead feed-through | 50 Ω | Path through the enclosure wall |
| 2 × 50 Ω SMA termination | ≥ 1 W | The leakage check |
| SMA torque wrench | 0.9 N·m | A loose SMA in the chain is both a leakage path and a loss that moves |
| shielding | a shielded enclosure for the **base** board, with its USB, Ethernet and power through filtered or ferrite-clamped entries; or, at the least, the two boards in separate rooms with the cable between them | Keeps the room, the RS-11.6 emitter and board-to-board leakage from dominating the conducted path. The tractor stays outside: its camera must see the PC screen |

**Assembly**, with the radios parked (`radio_state.py` `0x80` on both
boards, no daemon, camera unit or probe running):

1. Tractor: take off the blade, store it with its position noted, fit 30 dB
   pad A to the jack, torque it.
2. Base: the same with pad B. Put the base in the enclosure and route the
   cable through the bulkhead. Keep the base's USB on a USB 2.0 port: RS-11.6
   noted broadband USB 3.0 noise at 900 MHz.
3. Chain: pad A → cable → 20 dB fixed → step chain → bulkhead → cable → pad B,
   with the step chain at 0 dB (a barrel). Torque every joint.
4. Record pad markings or serials, cable types and lengths, the enclosure and
   its cable entries, and photos (`legs/rf_setup.txt`, `legs/rf_setup_*.jpg`).
5. Every later step change touches only the step chain. The jack pads and the
   20 dB fixed pad stay put.

**Step combinations** (nominal; a pad's tolerance is about ±0.5–1 dB and each
connector pair about 0.1 dB, so the chain can be off by up to ±2 dB at 60 dB;
the RSSI tracking rule below is the calibration, and the record carries both
the nominal step and the measured RSSI):

| step | pads | step | pads | step | pads |
|---|---|---|---|---|---|
| 0 | barrel | 24 | 3+6+15 | 48 | 3+15+30 |
| 3 | 3 | 27 | 12+15 | 51 | 6+15+30 |
| 6 | 6 | 30 | 30 | 54 | 3+6+15+30 |
| 9 | 3+6 | 33 | 3+30 | 57 | 12+15+30 |
| 12 | 12 | 36 | 6+30 | 60 | 3+12+15+30 |
| 15 | 15 | 39 | 3+6+30 | 63 | 6+12+15+30 |
| 18 | 3+15 | 42 | 12+30 | 66 | 3+6+12+15+30 |
| 21 | 6+15 | 45 | 15+30 | | |

Log every step change as a row of `legs/atten_log.csv`:
`utc, step_db, pads, next_leg, operator`. This is the W4-02 `atten_log`
idea, with a row per leg.

## Prep — every round

1. BENCH_RUNBOOK prep 1–7, and the "Every session, every run" checklist of
   RS13_VECTOR_LEG.md. That covers:
   - the staging count and md5;
   - the railroad video in a normal window, with the player's fullscreen;
   - the scene check before **every** leg (≥ 8 % of pixels changed over
     10 s);
   - `clear_retained.py` before every leg;
   - park last;
   - the P3 interleaving rule and the amended P8.

   The tractor stays outside the shielding, with its camera on the screen.
2. Health probes on both boards, keeping the whole output
   (`rs116_health_probe.py`): once per round, and again after any board
   reboot.
3. Check the RF path: walk the chain, check torque, confirm the planned step
   and log it.
4. **Receive-only spot-check: DTS rounds only, the same day, before the first
   DTS leg.**
   - Setup: tractor parked (`0x80`, no daemons); RF path as the legs will fly
     it (conducted, step chain at 0 dB).
   - Survey: run [`channel_survey_sniff.py`](../channel_survey_sniff.py) on
     the base over 902–928 MHz in 0.5 MHz steps. A dwell of about 30 s
     catches ≥ 4 ticks of the 7.08 s emitter; the full band takes ≈ 27 min.
   - Pick: `tools/survey_compare.py today.jsonl [--prev <last survey>]`. Any
     hot sample (> −75 dBm) disqualifies a channel.
   - Use: the pick goes on **every** profile-2 harness line as
     `-ForceFrfHz <spot-checked carrier>` (RS13_VECTOR_LEG.md Amendments
     2026-10-10, item 3).
   - Keep the survey (`legs/spot_check_<date>.jsonl`, plus the compare
     output), and record the pick's floor: it is the reference for the
     tracking rule. FHSS rounds take no pick (FHSS hops all 50 channels).
5. Per leg: RS13_VECTOR_LEG.md Step 2 prep and post.
   - Before: production camera stopped, no `/dev/ttymxc3` holder,
     `clear_retained.py`.
   - Base capture started before the harness; leg report run **with**
     `--capture`; `frag_gap_report.py`.
   - Pre- and post-brackets on both boards for every leg, as in RS-13.1.
   - Park last; on `PARK_TRANSIENT`, wait about 60 s and park again.
6. Copy the RS-13.1 scripts this session uses into its own `scripts/` and set
   their evidence paths. They carry RS-13.1's paths (`E=…RS_13_vector_scene_2026-09-26`)
   and `leg_prep.sh` hard-codes the capture's `--duration 480`. The scripts
   are `leg_prep.sh`, `leg_post.sh`, `scene_check.sh`, `stage_boards.sh`,
   `leg_replay.py`, `digest_trace.py`, `fold_emitter.py`, `streak_check.py`
   and `youtube_window.ps1`.

## Leg lines

Run from `firmware/x8_lora_bootloader_helper/`. Everything not named is at the
harness default: `-TxPipeline v3`, depth 2, prepare-ahead 1, smooth pacing,
`-CmdStreamMinGapS 1.0`, `-IdleDrainQuietS 1.5` and `-TxPowerDbm` empty.
Pass `-TxBatch 0` every time, because the harness default (1) would batch
frames that queue behind a stall into multi-fragment trains.

- **DTS, VECTOR** (the 2a line with probes):

  ```powershell
  .\run_live_radio_monitor.ps1 -TxFeed camera -RegProfile 2 -ForceFrfHz <spot-checked carrier> `
     -DurationS <300|180|120> -SynthFps 2 -KfRequestDisable 1 -ReactiveFire 1 -ProbeEcho 0 `
     -LogFragArrivals 1 -TxBatch 0 -CamExtraEnv "-e LIFETRAC_ENCODE_MODE=9 -e LIFETRAC_VECTOR_DETAIL=80" -Archive
  ```

- **DTS, `mono_g4`** (the 2c line with probes): the same with
  `-CamExtraEnv "-e LIFETRAC_ENCODE_MODE=6"`.
- **FHSS, VECTOR** (the 2b line): no `-ForceFrfHz` (the hop scheduler owns the
  synthesizer at profile 1) and no probes:

  ```powershell
  .\run_live_radio_monitor.ps1 -TxFeed camera -RegProfile 1 `
     -DurationS <300|180|120> -SynthFps 2 -KfRequestDisable 1 -ReactiveFire 0 -ProbeEcho 0 `
     -LogFragArrivals 1 -TxBatch 0 -CamExtraEnv "-e LIFETRAC_ENCODE_MODE=9 -e LIFETRAC_VECTOR_DETAIL=80" -Archive
  ```

- **FHSS, `mono_g4`**: the same with `-CamExtraEnv "-e LIFETRAC_ENCODE_MODE=6"`.

**Base capture** for each leg: RS13_VECTOR_LEG.md Step 2, with
`--profile image_bw500` on DTS and `image_bw250` on FHSS.

- `--duration`: the leg's length plus 180 s.
- `--min-frames`: 500 / 300 / 200 for 300 / 180 / 120 s legs.
- `--strict`: on VECTOR legs only.

At lossy steps `min_frames` and `applied_ratio` fail by design. There the
per-step numbers are the result, not the `RESULT:` line.

On every DTS leg, check the archive's daemon logs for
`forcing FRF -> <pick>` … `FRF readback: … (OK)` on both boards.

**Leg names**, also used as file prefixes in `legs/`:

| round | legs |
|---|---|
| leakage check | `leak_dts`, `leak_fhss` |
| A | `A2c`, `A2a`, `A2c2`, `A2a2` |
| B (DTS walk) | `B<step>v` (VECTOR), `B<step>m` (`mono_g4`) |
| C (FHSS walk) | `C<step>v`, `C<step>m` |
| closing check | `B0close`, `C0close` |

## Round A — leakage check and the 0 dB step (DTS)

### Leakage check

Run it first in Round A, and again whenever the RF setup is reassembled or
moved. It keys the tractor, into pad A and a termination, so it is part of
the round's GO.

1. With the radios parked, uncouple the cable from each jack pad's free end
   and fit a 50 Ω termination there. Each PA then sees its 30 dB pad and a
   load, and the base can hear the tractor only by leakage.
2. Fly `leak_dts`: the DTS VECTOR line at the pick with `-DurationS 120
   -ReactiveFire 0`, base capture running (without `--strict`; it should stay
   empty). Then fly `leak_fhss`: the FHSS VECTOR line with `-DurationS 120`.
3. **Pass:** the base decodes no frame on either leg. There must be no
   `published frame_id` line, `rx_frames_seen` must stay 0 in `link_stats`,
   and the base capture must be empty. Record any `crc_dump:` lines; their
   RSSI is the leakage level.
4. **Fail** (any frame decoded): leakage reaches the receiver at or above its
   sensitivity, so the walk would measure the leakage path. Improve the
   shielding (seams, cable entries, ferrites, separation) and repeat. There is
   no 0 dB step and no walk until it passes.
5. Restore the path with the radios parked, step chain at 0 dB, and torque it.

The check proves only that leakage is below sensitivity. The last 10 dB is
covered by the walk's tracking rule.

### The 0 dB step

Fly **2c, 2a, 2c′, 2a′** (`A2c`, `A2a`, `A2c2`, `A2a2`): 300 s each, at the
pick, step chain at 0 dB, using the DTS lines with probes. The order interleaves the two codecs, so P3 and
command delivery compare pooled legs (RS-13.1 A15).

| what | rule | source |
|---|---|---|
| P1–P5, P7, P8 | As in RS13_VECTOR_LEG.md, amended 2026-10-10: P8 is the absolute 350 ms bound at 2 fps. **P3 interleaved:** pooled seq-gap loss of 2a + 2a′ against 2c + 2c′, two-sided Fisher exact p ≥ 0.05, or a margin stated in RESULTS before the legs. These legs carry probes, so their image loss compares only with each other, not with RS-13.1's probe-free legs | RS13_VECTOR_LEG.md Step 2 |
| R0 | Per leg, the median of the base's 10-s `rx_rf:` RSSI medians, and the same for SNR. R0 is the median over the four legs and must sit in −65 to −75 dBm. If it does not, change the fixed middle pad (radios parked) before Round B and re-measure with one 120 s VECTOR leg. The 0 dB legs stand, with their level recorded | archive `rx_daemon.log` |
| command delivery (§8.6) | Per leg: the distinct PROBE seqs in the tractor's `LoRa cmd: PROBE seq=N … (rx#M)` lines that also appear in the base's `PROBE TX seq=N … (tx#K)` lines, over the base's PROBE TX count. Pool VECTOR (2a + 2a′) and `mono_g4` (2c + 2c′). **Not worse** = VECTOR ≥ `mono_g4`, or two-sided Fisher exact on delivered / undelivered p ≥ 0.05. Ignore the base's own `probe:` line: with `-ProbeEcho 0` it counts echoes and reads 0 %. Keep the score as `legs/<leg>_probe_score.txt` | archive `tx_daemon.log` (tractor), `rx_daemon.log` (base) |
| A16 confirmation (2a, 2a′, bw500) | Every resync episode opened by an **isolated** lost frame (no other loss within 25 frames after it) ends **by DIGEST** (`digest` in the dry-run `resync:` episodes line) within **≈ 25 frames of the lost frame**, counted in capture rows from the seq gap to the episode's end. That is the `a16_sil.py` acceptance figure; a lost DEL's ghost alone holds the base out of step for TTL_FRAMES = 20 applied frames. No such episode ends at an epoch start more than 25 frames after its loss. `scripts/digest_trace.py` over each episode shows the base's live count back at the encoder's DIGEST `n_live` within those frames, with no later row where the encoder counts more live shapes than the base. Record the base's `ttl_dropped`, and the encoder's TTL/carousel counters if `vector_stats` carries them. The final `digest_ok` is not `False` (`leg_replay.py`'s `last DIGEST checked` beside it) | capture summary, `leg_replay.py`, `digest_trace.py` |
| emitter fold | `scripts/fold_emitter.py` on every archive with ≥ 5 lost frames. A significant fold at 7.07–7.09 s (random-frame null p < 0.05) means the RS-11.6 emitter still reaches the link: re-check the carrier and the shielding before Round B | archive `rx_daemon.log` |

A clean carrier at 0 dB may lose only a handful of frames. If 2a and 2a′
hold fewer than 3 loss-opened episodes, the A16 confirmation is **thin**. Say
so, and pool into the same rule the Round B VECTOR legs whose seq-gap loss is
≤ 10 %. Any episode that fails the rule re-opens A16, and the round's
verdict names it. A mismatch with no loss before it fails P1, as in RS-13.1.

**Before asking for the Round B GO:** the leakage check passed, R0 is in
range, 2a and 2a′ pass P1 by the loss rule, and the A16 confirmation has not
failed.

## Rounds B and C — the walk-down

**Round B** walks DTS: profile 2, at the day's pick, with probes. **Round C**
walks FHSS: profile 1, image-only. Each round needs its own GO, and the two
may fall on different days:
- a DTS round on a later day takes a new spot-check;
- a reassembled RF setup takes a new leakage check;
- Round C opens with its own 0 dB pair (`C0v`, `C0m`, 180 s each), which
  gives FHSS's R0 and baseline.

**Step plan**, from R0 and the sensitivity estimate S (Numbers):

- **Coarse, 120 s legs:** every 9 dB from 0 up to C, the largest multiple of
  9 with R0 − C ≥ S + 12 dB, for as long as each step stays clean. Clean
  means seq-gap loss ≤ 1 % on both codecs and the RSSI tracking rule holds.
- **Fine, 180 s legs:** every 3 dB step from C + 3 to the stop. If a coarse
  step is not clean, the fine steps start 3 dB above it instead.
- **Example, R0 = −67 dBm:** C = 36 on both profiles. The coarse steps are
  9, 18, 27 and 36; the fine steps are 39, 42, 45 and onward.
- **Why coarse steps at all.** Above S + 12 dB a 3 dB step changes only the
  RSSI. The RS-13.1 link at ≈ −64 dBm ran at its interference floor, not at
  its noise floor. Flying those steps at 3 dB would add about 12 legs per
  profile. If the operator wants §8.6's "each 3 dB step" to the letter, fly
  the coarse region at 3 dB with 120 s legs, and say which plan in RESULTS
  before the round.

**Within each step:**

1. Make the step change: radios parked after the previous leg's `PARK_OK`,
   then a row in `atten_log.csv`.
2. Scene check.
3. Fly both codecs at that step, alternating the order (ABBA): VECTOR first
   on the walk's odd-numbered steps, `mono_g4` first on even ones, so a slow
   drift does not favour either codec.

**Record per leg** (one row each in the walk tables of the template):

| measure | how | source |
|---|---|---|
| step | nominal dB in the step chain, and its pads | `atten_log.csv` |
| frames/s | `published frame_id` lines ÷ leg seconds. For a `mono_g4` frame, published means every fragment arrived; confirm max K in `tx_daemon.log` (`done (pipelined): K fragments ok`, K = 1 expected at `-TxBatch 0`) | archive `rx_daemon.log`, `tx_daemon.log` |
| seq-gap loss | The `capture seq gaps` line of `rs12_leg_report.py --capture` (the loss of record, RS-13.1 A14). FHSS: from the first frame heard, with the `first frame heard at seq N` head and the `counting from seq 1` figure (RS-13.1 A7) | leg report |
| longest silence | `frag_gap_report.py` `max_gap` and `gaps>3s`, for both codecs | archive |
| epoch starts | `epoch starts: N received, M applied` (VECTOR) | capture summary |
| resync and recovery | The dry-run `resync:` episodes line: count, frames and seconds per episode, and what ended each (digest, epoch start, or still open at the end). The in-step share is 1 − resync frames ÷ frames received. Isolated-loss episodes are scored by the A16 rule at steps with loss ≤ 10 %. At heavier loss, overlapping losses legitimately stretch episodes: record them, do not score them | capture summary, `leg_replay.py` |
| store | `orphans`, `digest` checked / mismatched, `ttl_dropped`; the final `scene:` line (horizon mode, layers, `digest_ok`) | capture summary |
| level reached | The VS header's LL field: per-frame `L<n>` in the capture rows, `scene: … level`, and `tractor-log`'s `levels seen`. V0 is expected everywhere, because no ladder is built; anything else is an anomaly | capture, `tractor-log` |
| command delivery | DTS only, scored as in Round A, with the probe count beside it. Probes fire only on a received frame, so at lossy steps fewer fire, and delivery is conditional on a forward link that just worked | archive logs |
| RSSI / SNR | From the base's `rx_rf:` 10-s windows: the median of the medians, the minimum, and n. Also the `crc_dump:` count with its RSSI/SNR. The tractor's tx daemon logs no RSSI, so its receive level is inferred: the path is passive and reciprocal, and both radios transmit at 14 dBm | archive `rx_daemon.log` |
| tractor | `radio_tx_ok` (post-bracket); on FHSS, `tx_stream_streak_max` (≥ 8 = time authority, RS-13.1 A11); `drop_full` / `drop_stale` | brackets, `tx_daemon.log` |
| encoder | `vector_stats` `ms_total` p95, against the amended P8 bound | `tractor-log` on the archived `camera_service.log` |

**RSSI tracking rule.** This is the walk's validity check. If two
consecutive steps break it, stop the walk: leakage, a bad connector or the
floor has taken over, and the steps past the last one that held the rule are
void for the edge.

- While the median RSSI is ≥ 10 dB above the pick's survey floor, it must
  read within ±3 dB of R0 − step.
- Below that, the median SNR must fall by 3 ± 2 dB per 3 dB step until
  frames stop.
- Near the edge LoRa loss rises steeply, from a few percent to most frames
  within one or two steps. A slow, ragged decline over many steps points to
  leakage or a moving connector.

### Stop rules

- **End of a walk** (per profile), after the first of:
  - the first step at which **both** codecs publish < 10 % of the camera
    rate (< 0.2 frames/s over the leg);
  - the 66 dB step;
  - on FHSS, two consecutive steps on which the base never acquires (no
    frame heard): acquisition, not demodulation, then sets the limit;
    record that step.
- **Closing check** after every walk (`B0close`, `C0close`): the step chain
  back to 0 dB, then one 120 s VECTOR leg. Its median RSSI must be within
  2 dB of R0, and its loss no worse than the 0 dB legs'. If not, a connector
  moved, and the later steps are suspect: say so.
- **Abort the round, park, record, and fly nothing more without a new GO** if
  any of these happens:
  - a DTS leg without `FRF readback: … (OK)` at the pick on both boards, or
    any `FRF force-write failed`;
  - the leakage check failing;
  - the tracking rule failing on two consecutive steps;
  - a board reboot, a watchdog reset or a UART-holder conflict;
  - the scene check failing twice in a row;
  - anything that suggests an open RF port: a sudden RSSI jump, or a
    connector found loose;
  - the operator's call.
- **Time.** A round ends at its announced stop time, even mid-walk. The walk
  resumes from the last completed step under a new GO, after the 0 dB
  closing check.

### Expected effort (estimates)

| round | content | legs | time |
|---|---|---|---|
| A | spot-check (≈ 27 min), leakage check (2 × 120 s), 0 dB step (4 × 300 s) | 6 | ≈ 1.5–2 h |
| B | DTS walk: ≈ 4 coarse + ≈ 7 fine steps × 2 codecs, plus the closing check | ≈ 23 | ≈ 4 h |
| C | FHSS walk: 0 dB pair, ≈ 4 coarse + ≈ 7 fine steps × 2 codecs, plus the closing check | ≈ 25 | ≈ 4 h |

Each leg costs about 7 minutes on top of its airtime: prep and brackets,
harness start-up, the base capture's tail, reports and park. Each step change
adds 1–2 minutes. Plan one round per day unless the operator wants more.

## Evidence layout

```
bench-evidence/RS_13_range_edge_<date>/
  RESULTS.md                                   from the template below
  legs/rf_setup.txt rf_setup_*.jpg atten_log.csv
  legs/spot_check_<date>.jsonl spot_check_<date>_compare.txt
  legs/leak_dts_* leak_fhss_*                  base capture, reports, park
  legs/legA2c_* legA2a_* legA2c2_* legA2a2_*   RS13_VECTOR_LEG.md's per-leg set: scene, health,
                                               pre/post brackets, base capture (.jsonl/.json/.txt),
                                               report (--capture), gaps, park
  legs/legB<step><v|m>_* legC<step><v|m>_*     the same per walk leg
  legs/leg<X>_probe_score.txt                  DTS legs: tractor ∩ base PROBE seqs
  legs/leg<X>_rf.txt                           rx_rf + crc_dump extract
  legs/leg<X>_digest_trace.txt                 VECTOR legs with a resync episode
  legs/session_end_park_<date>.txt             radio_state.py, both boards, per round
  scripts/                                     the copied RS-13.1 scripts with this session's paths
```

The harness archives (`bench-evidence/radio_monitor_<stamp>_<sha>/`) stay where
the harness puts them; RESULTS names each one per leg. Use Windows-style
`C:/…` paths for the Python tools and `adb pull` (RS-13.1 A6).

## Traps

- **`-ProbeEcho 0` hides delivery from the base.** The base's `probe:` line
  then reads 0 % delivered, so score delivery tractor-side.
- **`-ForceFrfHz` belongs on profile-2 lines only.** The harness refuses a
  centre outside 902.25–927.75 MHz.
- **Two harness defaults are wrong for this session.** `-ReactiveFire`
  defaults to 0, so DTS legs pass `1` explicitly. `-TxBatch` defaults to 1,
  so every leg passes `0`.
- **SNR saturates and RSSI flattens.** Read RSSI in the strong region and
  SNR near the floor (Numbers).
- **Lossy steps fail the capture checks.** `min_frames` and `applied_ratio`
  fail there by design. The leg report's counter `loss` line is not the loss
  of record (A14, RS-12.17); use `capture seq gaps`.
- **Parking gets harder at the edge.** After a leg in which the base lost the
  tractor, its scan walker can keep walking for about 2 minutes
  (BENCH_RUNBOOK, `radio_park.py`), so expect `PARK_TRANSIENT`: wait and park
  again. Touch no RF hardware before `PARK_OK` and `radio_state.py` `0x80`
  on both boards.
- **The FHSS acquisition head grows as the signal weakens** (A7). Read FHSS
  loss from lock, and record the seq of the first frame heard at every step.
- **A closed enclosure warms the base over a long round.** Read the X8's
  `/sys/class/thermal/thermal_zone*/temp` at the start and end of each round.
- **The RS-13.1 traps apply unchanged**: staging rot, retained topics, the
  kiosk ban, the production camera unit stealing `/dev/ttymxc3`, and the
  `LIFETRAC_USE_LORA_BRIDGE=1` requirement (RS13_VECTOR_LEG.md, Traps).

## Results template

Copy into `bench-evidence/RS_13_range_edge_<date>/RESULTS.md` and fill every
`<placeholder>`. Keep the section order.

```markdown
# RS-13 range edge — VECTOR and mono_g4, conducted walk-down (<YYYY-MM-DD>)

**Status: <in progress | complete>. Verdict: <one sentence per round: what
passed, what failed, the edge per profile and codec>.**

Procedure: firmware/x8_lora_bootloader_helper/bench_tools/RS13_RANGE_EDGE_LEG.md
@ `<sha>`; leg criteria RS13_VECTOR_LEG.md as amended 2026-10-10. GOs: Round A
<date, time, operator's words>; Round B <…>; Round C <…>.

## Software under test

| item | value |
|---|---|
| A16 fix | PR #<n> @ `<sha>`; `a16_sil.py` on that SHA: A16-like bw500 <0>/100, bw250 <0>/100, recovery max <n> frames, lossfree_out_of_step_frames <0> |
| code guards | PR #<n> @ `<sha>` (batching, cold-start budget clamp, double epoch start, switch gap) |
| tractor image | `lifetrac-tractor-x8:latest` id `<id>`, built <date> from `<sha>`; step 0 smoke `<numpy x cv2 y encoder VectorEncoder>`; step 1 from `/app`: bw250 video p50/p95 <ms>, bw500 video p50/p95 <ms> |
| base image | `lifetrac-v25:latest` id `<id>`, built from `<sha>` |
| staging | pushed <date, time>; entry counts base <n> / tractor <n>; md5 `<…>` |
| dials | `LIFETRAC_VECTOR_DETAIL=80` (V0), camera 2 fps, `LIFETRAC_KEYFRAME_COPIES` 1, `LIFETRAC_WEBP_QUALITY` 55, `-TxPowerDbm` empty (14 dBm), `-TxBatch 0`, `-KfRequestDisable 1` |
| scene | the railroad video in a normal window (no kiosk); scene check before every leg (`legs/<leg>_scene.{txt,jpg}`) |

## Firmware on the boards

| board | build | md5 | flashed |
|---|---|---|---|
| base 2D0A1209DABC240B | <bench (register-write diag) build, PR # @ sha> | `<md5>` | <date> |
| tractor 2E2C1209DABC240B | <same> | `<md5>` | <date> |

## RF setup

| item | value |
|---|---|
| tractor jack | pad A <dB, marking / serial> |
| base jack | pad B <dB, marking / serial> |
| fixed middle | <20 dB, marking> |
| step set | 3 / 6 / 12 / 15 / 30 dB <markings>, or step attenuator <model> |
| cables, bulkhead | <type, length each> |
| shielding | <enclosure model, cable entries / separate rooms> |
| fixed loss, nominal | <dB> |
| R0 (0 dB step, base `rx_rf:` median RSSI / SNR) | DTS <dBm / dB>; FHSS <dBm / dB> |
| photos | `legs/rf_setup_*.jpg` |

Leakage check: `leak_dts` <archive>, `leak_fhss` <archive>; frames decoded
<0 required>; `crc_dump:` lines <n, RSSI>; <PASS | FAIL, what was fixed>.

Spot-check: `legs/spot_check_<date>.jsonl`; pick <Hz>; floor at the pick <dBm>;
<minutes> before the first DTS leg; `FRF readback: … (OK)` on both boards on
every DTS leg <yes / list>.

## Round A — the 0 dB step (DTS @ <pick>)

Flight order: 2c, 2a, 2c′, 2a′. Harness lines: <paste>.

| # | criterion | 2c | 2a | 2c′ | 2a′ | pass? |
|---|---|---|---|---|---|---|
| archive | | | | | | |
| P1 | dry-run checks, loss rule (`store:`, `resync:` episodes, final `digest_ok`, `last DIGEST checked`) | n/a | | n/a | | |
| P2 | `published frame_id` ÷ 300 | | | | | |
| P3 | `capture seq gaps`; pooled VECTOR <x/y> vs mono_g4 <x/y>, Fisher p <p> | | | | | |
| P4 | max K, K = 1 count, ABORTED | | | | | |
| P5 | gaps > 3 s | | | | | |
| P7 | link_stats `rx_codec_name` | | | | | |
| P8 | encoder p95 vs 350 ms (amended); vs step 1 for information | n/a | | n/a | | |
| R0 | `rx_rf:` median RSSI / SNR | | | | | |
| cmd | PROBE delivered / sent (tractor ∩ base seqs) | | | | | |

Command delivery (§8.6): VECTOR <x/y = %> vs mono_g4 <x/y = %>, Fisher p <p>,
<PASS | FAIL>.

A16 confirmation (bw500):

| leg | loss at row | episode (start-end, frames, s) | ended by | back in step (digest_trace, frames after the loss) | isolated? | pass? |
|---|---|---|---|---|---|---|

Base `ttl_dropped` <n>; encoder counters <if present>. Episodes scored <n>
(<thin: pooled with Round B legs ≤ 10 % loss>). Emitter fold: <archive: R, p>.

## Round B — DTS walk (@ <pick>)

Step plan stated before the round: <coarse 9 dB to C = <n>, then 3 dB | 3 dB throughout>.

| step dB | leg | codec | s | archive | frames/s | seq-gap loss | max gap s | epoch starts rx/applied | resync episodes (max frames, ended by) | in step % | orphans | ttl_dropped | cmd delivered/sent | RSSI med / min | SNR med / min | CRC err | level | notes |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|

Closing check `B0close`: RSSI <dBm> (R0 <dBm>), loss <%>; <OK | suspect>.

## Round C — FHSS walk

| step dB | leg | codec | s | archive | frames/s | seq-gap loss (from lock) | first frame heard (seq) | max gap s / gaps>3s | epoch starts rx/applied | resync episodes | in step % | orphans | RSSI med / min | SNR med / min | CRC err | streak_max | level | notes |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|

Closing check `C0close`: <…>.

## The edge

| profile | codec | last step with loss ≤ 1 % | first step with loss ≥ 10 % | first step < 10 % of frames | received level there (R0 − step) | last step with the tracking rule held |
|---|---|---|---|---|---|---|
| DTS | VECTOR | | | | | |
| DTS | mono_g4 | | | | | |
| FHSS | VECTOR | | | | | |
| FHSS | mono_g4 | | | | | |

## VECTOR_SCENE.md §8.6 rows

| measure | criterion | result |
|---|---|---|
| Base command delivery during VECTOR | not worse than during `mono_g4` | DTS 0 dB: <…>; per step: <…>; FHSS: not measured — waits for the RS-12.15 reverse-delivery fix |
| Fragment loss | not worse than `mono_g4` on the same workload and profile | Round A P3 <…>; per step: <…> |
| Range-edge leg | at each 3 dB step: VS frames/s, level reached, command delivery, the same for `mono_g4`; VS1 keeps an L0 picture and STATUS where `mono_g4` delivers no complete frame | per-step tables above; level <V0 throughout>; the L0/STATUS clause **not testable at V0** (no ladder built) |

## What these legs did NOT test

- V1–V3 (the ladder) and with it the §8.6 L0/STATUS clause; self-select (D-VS6b), B4, B5
- FHSS command delivery and the FHSS switch leg (RS-12.15 reverse-delivery fix first)
- the mode switch (no 2d leg); web_ui in the loop; the production stack (B1, RS-9.2/9.4, RS-11.7)
- a distance: conducted path, no fading or antenna effects
- <anything cut on the day>

## Evidence limitations

- <nominal pad values vs the tracking rule; leakage only proven below sensitivity>
- <tractor-side receive level inferred, not logged>
- <scene checked before each leg, not during it>
- <n per step; coarse-step plan if flown>

## Anomalies for the record

- <anything the criteria do not cover>

## GO / NO-GO

<per round: the deciding rows>. Next: <the ladder walk / FHSS command legs after RS-12.15 / …>.
```
