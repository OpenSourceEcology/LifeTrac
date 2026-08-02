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
   436+133≈569) — nearly every train either completes or times out; the
   remainder (4–13 trains/run) is whole-train losses plus trains
   in-flight at teardown. Loss events per boundary are near-constant at
   ~0.20–0.25 (the 17 0x63 radiations per run are confined to the first
   ~10 s — see §4 — and cannot drive the steady-state rate), while
   events per second double across the sweep — the event is tied to the
   boundary, not to wall clock. Each event costs ~1.3–1.8 fragments
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
  encode-mode override retries unacked because the synth feed has no
  camera_service to ack it. **Correction (review catch, 2026-08-02):**
  the first write-up said the machinery "never gives up, ~1/18 s" — that
  was an averaging artifact (17 ÷ 300 s without checking the
  distribution). The log is explicit: all 17 attempts land in the first
  ~10 s and then `cmd 0x63 GAVE UP after 17 attempts (10.3 s)` — the
  pending-ack machinery is already bounded, and the "add a retry bound"
  follow-up dissolves. Startup-confined, so it cannot contribute to
  steady-state loss at all (stronger exoneration than the count
  argument alone).
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

## 6. L072 counter split — the loss is ON-AIR CRC CORRUPTION

Probe-bracketed runs (`rs115_stats_probe.py`; counters are cumulative —
see §7 finding on the dead base NRST — so deltas are taken around each
run). Both runs 300 s @ 3000 B, n=1 each arm:

| leg | PrepareAhead | Δradio_rx_ok | Δradio_crc_err | Δdio0 | Δtx_ok | ring_ovf/dropped |
|-----|-------------:|-------------:|---------------:|------:|-------:|-----------------:|
| A   | 1 (default)  | +2353 | **+75** | +2445 | +17 | 0 / 0 |
| B   | 0            | +2361 | **+80** | +2458 | +17 | 0 / 0 |

`Δdio0 = Δrx_ok + Δcrc_err + Δtx_ok` reconciles EXACTLY in both legs:
every transmitted fragment reaches the base demodulator; the missing ones
arrive **corrupt (CRC fail)**. Host-side drops are zero at every counter.
The CRC delta (~75–80) matches the attributed per-run loss (76–82) — the
whole boundary event is corruption, not deafness.

Note the layering trap this exposes: the CR-4/5 coding-rate decision
rested on "zero payload CRC errors in ~19k frames" measured at the DAEMON
(rx_decode_err) — a layer that never sees radio-CRC failures (corrupt
frames produce no URC). At the radio layer the error rate is 3.2–3.4%,
past the ~1% revisit trigger in spirit. Revisit belongs on the table once
the mechanism is fixed (parity/CR would mask, not fix).

## 7. Prepare-ahead A/B — hypothesis refuted; position lock refined

The prepare-ahead SPI/FIFO-during-TX hypothesis predicted the penultimate
spike collapses with `-TxPrepareAhead 0`. It did not move:

| run | PrepareAhead | attributed | idx 11 | idx 12 |
|-----|-------------:|-----------:|-------:|-------:|
| sweep #1 | 1 | 70 | 20 | 4 |
| sweep #2 | 1 | 68 | 22 | 3 |
| rs115 #1 | 1 | 76 | 21 | 1 |
| A        | 1 | 82 | 23 | 6 |
| B        | **0** | 79 | **21** | 3 |

Position-lock refinement from the synth size mix (22–27% of frames are
12-fragment): a "last fragment of the train" mechanism would put ~73% of
the spike at index 12 — observed is the opposite. The lock is **per-frame
slot total−2** (13-frag → 11, 12-frag → 10 [moderate counts observed],
7-frag → 5, 4-frag → 2), i.e., exactly where the v3 pipeline enters its
drain phase (all fragments submitted, last one parked, nothing left to
feed).

Exonerated across the session: TX-side skips (TX_DONE + size-mix
accounting), host command TX (17/run vs 50–105 losses), RX re-arm at
train start (index 0 near-baseline), prepare-ahead (this A/B), host-side
drops (all counters zero), cumulative in-train (gone with the bucket).

Host and firmware code audit (image_tx_daemon v3 path; sx1276_tx.c) shows
clean discipline at the visible layers: FIFO writes only from IDLE,
mailbox parks in RAM, raw host RXCONT writes post-burst and
opmode-guarded. The corruption source sits below host-side evidence —
inside the fragment-turnaround (TX→STANDBY→RXCONT→TX cycling with the
mailbox drain timing) or the PA/synth behaviour around it.

## 8. Leg C — RXCONT turnaround refuted; host-side discriminator space closed

A daemon env gate (`LIFETRAC_RXCONT_ARM=0`, harness `-RxcontArm 0`,
production default unchanged) disables all RXCONT arming, so the
firmware's RS-4.12 re-arm chain never starts and the tractor radio sits
in STANDBY between fragments — no TX→RXCONT→TX cycling at all (command
downlink deliberately deaf for the run). 300 s @ 3000 B, probe-bracketed
(archive `radio_monitor_20260802_094053_86bf5d7f`):

| leg | RXCONT cycling | Δcrc_err | attributed | idx 11 |
|-----|---------------:|---------:|-----------:|-------:|
| A/B | on             | +75/+80  | 82/79      | 23/21  |
| C   | **off**        | +62      | 66         | **22** |

The spike and the CRC rate are unchanged (Δ within run-to-run spread).
The inter-fragment turnaround is NOT the mechanism.

Incidental: the idle bench accumulated +47 radio CRC errors over ~11 h
overnight (ambient noise false-demods at ~4/h) — negligible against
62–80 per 300 s run, but worth knowing the counter's noise floor.

**With leg C, every software-layer candidate is eliminated** (TX skips,
host command TX, RX re-arm at start, prepare-ahead, host-side drops,
in-train cumulative, RXCONT turnaround). What remains is physical/
silicon-level at slot total−2: the transmitter radiating corrupt bits
for a reason invisible to the C-code guards (FIFO/PLL/PA behaviour), or
deterministic self-EMI at the tractor (e.g., host UART burst timing).
Discriminating those needs an SDR capture of a corrupt slot, or firmware
instrumentation (post-TX FIFO readback CRC, IRQ-flag dumps) — a flash
session either way.

## 9. Leg D — instrumented firmware: the transmitter's DIGITAL path is clean

Experimental L072 firmware (branch `rs115-fw-discriminators`, flashed to
the tractor via SWD/AN3155, `flasher RC=0`): DJB checksum over the FIFO
at load, chunked FIFO readback + compare after TX_DONE, and an
early-TX_DONE check (>5 ms before expected ToA). Reported via three
additive STATS counters and FAULT URCs 0x0D/0x0E carrying the fragment's
tx_id. One 300 s run at the standard operating point (archive
`radio_monitor_20260802_105435_87821c90`; tractor counters zeroed by
the launch reset; the aborted leg E archived as `..._110209_...`):

    tractor:  radio_tx_ok = 2468
              tx_fifo_rb_ok = 2468   tx_fifo_rb_bad = 0
              tx_done_early = 0      FAULT lines: none
    base:     Δradio_crc_err = +110  (corruption at full rate)

**Every transmitted fragment left an intact FIFO and radiated for its
full expected duration — yet ~110 fragments still arrived corrupt.**
The instrumentation's ~1–2 ms added inter-fragment latency did not
suppress the effect. Combined with legs A–C, the transmitter's entire
digital path (host feed → mailbox → FIFO → duration) is exonerated by
direct measurement. The corruption enters between the SX1276 modulator
and the base demodulator: analog/RF at either end, or receiver silicon.

## 10. Leg E (role swap) — aborted, no data; two rig facts learned

Swapping -TxAdbSerial/-RxAdbSerial aborts before any TX: the harness's
TX container image (`arduino-ootb-python-devel`) exists only on the
tractor, and `lifetrac-v25` on the base lacks numpy for the synth
publisher. No stream, no harm. Learned: (1) the OpenOCD SWD reset path
WORKS on the base board — its L072 counters are now cleanly zeroed, a
working substitute for the dead gpio163; (2) a proper swap needs
`-TxFeed host` (PC-side publisher over the base's ethernet) with the tx
daemon in `lifetrac-v25`. The swap remains the designed discriminator:
corruption following the ROLE = systemic/protocol-timing; corruption
vanishing or changing = unit-specific analog hardware (antenna,
connector, front-end).

## 11. Leg E′ — manual role swap: the corruption FOLLOWS THE ROLE

Manual swap (tx daemon on the BASE in `lifetrac-v25`, host-feed from the
bench PC broker over ethernet; rx daemon on the TRACTOR against its
bench_mqtt; both L072 counter baselines zeroed by prior resets):

    base   (TX): radio_tx_ok = 2596 — clean
    tractor(RX): Δrx_ok = 2422, Δcrc_err = **170** (6.6%!)
                 dio0 reconciles exactly (2422 + 170 = 2592)
    per-index (rx daemon, summed windows): 0:2 1:13 2:12 3:17 4:14 5:15
                 6:12 7:16 8:15 9:15 10:7 11:18 12:4 — **UNIFORM**, no
                 penultimate lock; timeouts 124.

Both boards corrupt as receivers; both are clean as transmitters. Unit-
specific hardware defect eliminated — but the SHAPE is board-specific:
base-as-RX locks at slot total−2 (~3–4.5%), tractor-as-RX is uniform and
worse (6.6%).

## 12. Leg F — 12 dB power drop: front-end overload REFUTED

New host-side knob `LIFETRAC_TX_POWER_DBM` (CFG_SET at daemon startup;
can only lower below the ERP-clamped ceiling). Forward roles at
**2 dBm** (vs default 14):

    tractor(TX): Δtx_ok = 2613, FIFO readbacks all intact
    base   (RX): Δrx_ok = 2492, Δcrc_err = **+101 (3.9%)** — UNCHANGED

A 12 dB receiver-input reduction did not move the corruption rate.
Overload is out. And the overnight idle floor (§8: ~4 false-demod CRC
events/HOUR) rules out ambient interference: active runs take 75–170 per
300 s — three orders of magnitude above background, so it is OUR OWN
packets arriving corrupt.

## 13. Final state of the diagnosis

**~4% of image fragments arrive payload-corrupt at the receiver (header
demodulates, payload CRC fails), in BOTH link directions, at BOTH power
levels, with the transmit digital path proven intact by in-firmware
readback. Eliminated by direct measurement: TX skips, host command TX,
RX re-arm, prepare-ahead, RXCONT turnaround, host-side drops, in-train
cumulative effects, unit-specific hardware, front-end overload, ambient
interference. The corruption shape is receiver-board-specific (base:
slot-(total−2) locked; tractor: uniform).**

Remaining hypothesis space (needs physical intervention or deeper
firmware work): (a) receiver-side host/board coupling into demodulation
— the base's position lock suggests something train-aware or
117-ms-periodic on the receiving host disturbing the radio; (b)
conducted/radiated EMI from the receiving board's own X8 with
board-specific activity signatures. Next discriminators: physical
separation/orientation change (user hands required — no RF equipment on
this bench), or firmware capture of RegIrqFlags + RSSI + payload dump
for corrupt receptions (RX-side instrumentation, next flash increment).

## 14. Session verdict and residuals

**Diagnosis after legs A–E: 11–18% of trains lose per-frame slot
total−2 to CRC corruption that enters BETWEEN the transmitter's FIFO
(proven intact, full-duration radiation, leg D) and the receiver's
demodulator. Every software and digital-TX layer is eliminated by
direct measurement. Remaining: analog/RF at either end, or receiver
silicon — with the position lock still unexplained.**

Residuals for RS-11.5 follow-up:
- The role swap (leg E recipe: -TxFeed host, tx daemon in lifetrac-v25
  on the base, rx on the tractor) — corruption follows role vs
  hardware unit.
- Physical inspection: antennas/connectors/separation (boards are
  bench-adjacent at 14 dBm — front-end compression is plausible, though
  it does not explain the position lock by itself).
- Base board gpio163 NRST no longer resets the L072 (counters survived
  ~8 harness launches; harness resets silently no-op) — bench
  infrastructure fix needed; probe deltas are the workaround.
- The daemon-level rx_frames vs Δrx_ok gap (~50) is the tractor's
  DurationS+10 synth overrun transmitting after the daemon stops —
  bench artifact, not loss.
