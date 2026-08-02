# RS-11.4 train-length sweep — the cumulative-loss question, re-baselined (2026-08-02)

**SHA under test:** main @ `bb21b053`. Six 300 s runs, synth local feed,
pinned operating point (v3 depth 2, DTS profile 2, smooth pacing,
TrainGapMs 40, SynthFps 2, KfRequestDisable 1, ProbeEcho 0), `-Archive`
bundles `radio_monitor_20260801_21{1446,2010,2534,3057,3622,4147}_bb21b053`.
SynthBudgetB 3000 / 1500 / 750 → 13 / 7 / 4 fragments per train, n=2 each.

## Verdict in three parts

1. **The D1 in-train gradient is GONE — the original RS-11.4 question is
   moot.** D1 (2026-07-29, token-bucket era) attributed 67 in-train losses
   in one run with indices 8–11 taking 37/67. Under smooth pacing (default
   since 2026-07-30) the in-train attributed loss collapsed to 2–8 per run
   (~0.1–0.4% of fragments) with NO index gradient at any train length.
   The cumulative mechanism was almost certainly the AirtimeBudget token
   bucket — the same family as the index-10 notch — and shipping smooth
   pacing fixed it before this sweep ran. None of the four RS-11.4
   candidate mechanisms (PA thermal, dwell, backpressure, clock drift)
   shows in-train.

2. **The dominant remaining loss is a PER-TRAIN-BOUNDARY event, and it is
   much bigger than the old floor.** Raw fragment shortfall
   (frags_tx − rx_frames) scales INVERSELY with train length — the
   opposite of cumulative-in-train:

   | budget | frags/train | trains | frags_tx | raw lost | raw loss% | reassembler timeouts | trains timed out | frames published |
   |-------:|------------:|-------:|---------:|---------:|----------:|---------:|--------:|--------:|
   | 3000 B | 13 | 190 | 2419 | 118 | 4.9% | 64 | 34% | 122 |
   | 3000 B | 13 | 191 | 2438 | 129 | 5.3% | (n=2 twin) | | 122 |
   | 1500 B |  7 | 382 | 2439 | 148 | 6.1% | 94 | 25% | 280 |
   | 1500 B |  7 | 382 | 2440 | 150 | 6.1% | (twin) | | 276 |
   |  750 B |  4 | 582 | 1891 | 179 | 9.5% | 133 | 23% | 436 |
   |  750 B |  4 | 583 | 1895 | 190 | 10.0% | (twin) | | 428 |

   `published + timeouts ≈ trains` in every run (122+64≈190, 280+94≈382,
   436+133≈569) — every train either completes or times out; there is no
   third bucket. Loss events per boundary are near-constant at ~0.20–0.25
   after subtracting the 17 periodic 0x63 radiations per run (see §4),
   while events per second double across the sweep — the event is tied to
   the boundary, not to wall clock. Each event costs ~1.3–1.8 fragments
   (raw lost / timeouts), consistent with the `post_loss` gap median of
   ~234 ms ≈ exactly two 116.9 ms fragment slots.

3. **CORRECTION (2026-08-02, same session): the "instrument gap" first
   reported here was an ANALYSIS error, not an instrument defect.**
   `lost_frag_idx` logs are per-10 s-window (the histogram is reset after
   every stats log — image_rx_daemon.py:1630); the first read took the
   LAST window as a run cumulative and concluded 2–8 attributed vs
   118–190 raw. Summing windows, the instrument attributes 68–143 per
   run ≈ 55–75% of raw — a residual of ~40–60/run (whole-train losses
   and tail losses where `frag_total` context is absent), roughly
   train-length-independent. Kept on the record per the corrections
   discipline.

   **The corrected per-index histograms answer the boundary-event
   discrimination outright — the loss is position-locked to the
   SECOND-TO-LAST fragment of the train at every train length:**

   | frags/train | penultimate idx | its share of attributed | idx-0 share |
   |------------:|----------------:|------------------------:|------------:|
   | 13 | 11 | 20/70, 22/68 (~30%) | 8/70, 3/68 |
   | 7  | 5  | 52/102, 49/101 (~50%) | 11/102, 8/101 |
   | 4  | 2  | 110/143, 99/140 (~75%) | 10/143, 13/140 |

   11–18% of ALL trains lose exactly their penultimate fragment. Index 0
   is near-baseline — the RX re-arm-at-train-start hypothesis is dead
   (again, consistent with RS-10.1). Constant-per-boundary event rate +
   position total−2 = an end-of-train mechanism.

   Localization from existing data:
   - **Not TX-side skips:** the TX `fragments ok` mix (12/13 at 27%)
     matches the synth bank's size spread (2892–3000 B → 22% need only
     12 fragments) and every fragment got TX_DONE — the L072 reports the
     penultimate fragment went on air.
   - **Not host command TX blinding:** only 17 command radiations per run
     (the unacked 0x63 retries) against 50–105 penultimate losses at the
     short-train tiers — arithmetically impossible.
   - Remaining split — base L072 demodulated-but-dropped (ring/UART) vs
     never-demodulated (PHY/firmware state at train end) — needs the
     L072's own STATS counters; that is the §6 experiment.

## 4. Secondary observations

- **17 × 0x63 (ENCODE_MODE) radiations in every run** — the retained
  encode-mode override retries unacked for the whole session because the
  synth feed has no camera_service to ack it (pending-ack machinery never
  gives up; throttled to ~1/18 s). Each TX blinds the base's RX briefly:
  a real robustness point (bounded retries needed) and a bench artifact
  to subtract from loss accounting. Constant across tiers, so it does not
  drive the inverse scaling.
- Boundary gap median grows as trains shorten: 178 ms (13/7-frag) →
  274.7 ms (4-frag) — prepare-ahead has less in-train time to build the
  next frame at short trains. Not separately investigated.
- Frame-level throughput: 0.41 / 0.93 / 1.45 frames published per second
  across the tiers — short trains publish more frames but pay a higher
  per-fragment tax; with a ~23–34% train-failure rate, the boundary event
  is now the single largest throughput lever on the table.

## 5. What this changes

- RS-11.4 CLOSED (question answered by re-baseline: mechanism was the
  bucket; already fixed).
- RS-11.5 reframed by the correction: the index-0-vs-tail discrimination
  is ALREADY answered (tail: penultimate slot, 11–18% of trains). The
  open question is the §6 split — L072-demodulated-but-dropped vs
  never-demodulated — plus bounding the 0x63 pending retries.
- The ~3.5% "loss floor" number should stop being quoted — the current
  floor is ~5% at 13-frag trains, it lives at the train tail, and the
  train-failure rate (23–34% timeouts) is the throughput cost to attack.

## 6. L072 counter split (run after the correction, same session)

Design: one 300 s run at 3000 B; STOP the rx daemon without resetting the
L072 (docker stop does not touch NRST) and read the L072's own counters
via the deployed diag probe (STATS_DUMP → STATS_URC survives in RAM). If
the L072's demodulated-frame count ≈ TX frags (~2300+) while the daemon
saw ~118 fewer → the loss is host-side (ring/UART drain). If the L072
count matches the daemon's (both ≈ TX − ~118) → the penultimate fragment
is never demodulated: a PHY/firmware-state event at train end (e.g., the
radio's end-of-train state transition clipping the tail of RXCONT).
Result recorded below when run.
