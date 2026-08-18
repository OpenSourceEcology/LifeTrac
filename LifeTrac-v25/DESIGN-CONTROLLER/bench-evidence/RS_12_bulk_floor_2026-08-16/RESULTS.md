# RS-12 opening session — the bulk floor is a silent L072 URC drop, not RF

Issue #107. SHA `29c8dea8`. Four legs, all at the clean 902.5 MHz channel
(`-ForceFrfHz 902500000`), all otherwise the standard operating point.

| leg | archive | config | loss | idx-lock |
|---|---|---|---:|---|
| A (from RS-11.6) | `radio_monitor_20260816_200712_9fee4460` | 3000 B / 13 frags | 3.3 % | idx 11 = 35 % |
| B | `radio_monitor_20260816_204928_29c8dea8` | 3000 B / 13 frags | 6.7 % | idx 11 = 32 % |
| C | `radio_monitor_20260816_205609_29c8dea8` | 1500 B / 7 frags | 5.7 % | **idx 5 = 53 %** |
| D (stats-bracketed) | `radio_monitor_20260816_210320_29c8dea8` | 3000 B / 13 frags | 8.2 % | idx 11 = 28 % |

## 1. The losses are never CORRUPTED — and never even MISSING at the radio

Two independent instruments, one conclusion:

**(a) Corrupt-capture indices are uniform; loss indices are not.** The
crc_dump payloads carry the fragment header, so the corrupt population's
index distribution is readable: at 13-frag trains idx 11 holds 9 % of
readable corruptions (≈ 1/13 uniform) while holding 28–35 % of losses; at
7-frag trains idx 5 holds ~1/7 of corruptions and **53 %** of losses.

**(b) The L072 counter bracketing (leg D) closes the books exactly:**

```
radiated (tractor frags_ok)          2414
radio handled (Δrx_ok + Δcrc_err)    2429   → NOTHING went undemodulated
Δcrc_err vs host crc_dumps            126 = 126   (instrument closure, exact)
Δrx_ok vs host URCs received         2303 vs 2215 → 88 frames demodulated,
                                                     then LOST before the host
L072 host_dropped / queue_full / ring_ovf   all ZERO
host_parse_err                              ZERO
identity residue Δdio0−(Δrx_ok+Δcrc_err+Δtx_ok) = 9 (small; cross-traffic)
```

**The clustered bulk loss is not an RF phenomenon at all.** The SX1276
demodulates the "lost" fragments successfully (`rx_ok` counts them); they
then vanish inside the L072's URC-emission path without incrementing any
drop counter, and nothing malformed reaches the host (zero parse errors —
the URCs are not truncated, they are never sent, or are overwritten whole).

## 2. The lock follows train STRUCTURE, confirmed on the clean channel

Leg C moved the penultimate from idx 11 to idx 5 and the lock followed
(53 %, matching RS-11.4's 50 % at this train length measured two weeks
earlier, pre-RF-switch-fix, on 915.0). Elapsed-time mechanisms are dead.
The L072 does not parse fragment headers, so the phase-lock must be
induced by the *traffic pattern itself* — the prime suspect being the TX
v3 pipeline's mailbox-drain behaviour at train end (the last two fragments
are the drain; something about their timing or the L072's train-end
bookkeeping opens a window in which a pending RX URC is silently lost).
This also dissolves the old role-swap puzzle: identical firmware showed
the lock only with base-as-RX because the *trigger* is the transmitter's
end-of-train pattern, which differed between the swapped configurations.

## 3. Run-to-run variance is event-DEPTH, not event-RATE

Loss swings 3.3 → 8.2 % across four same-config legs, but the boundary
event rate is stable (~86 timed-out trains and ~100 published frames in
every 3000 B leg; publish-gap structure identical). The extra losses in
bad runs deepen already-doomed trains rather than dooming more trains.

**Correction to the RS-11.6 leg-3 record:** "the escape halves the loss
(5.9 → 3.3 %)" compared two single runs whose spread is now known to be
~2×. The escape's *confirmed* effect is on the interferer-attributable
population (hot captures 11–13 → 5–8/run and the −43 dBm class weakened
to −45…−57 leakage); its effect on total loss cannot be resolved from
n=1 vs n=1 and the honest statement is "the periodic interference
component was removed; the bulk floor dominates and varies 3–8 %."

## 4. Where this leaves the campaign

The target is now precisely: **an L072 firmware race in the RX→URC path
that silently discards a demodulated frame, triggered once per train near
the transmitter's end-of-train pattern, uncounted by every existing
counter.** Next steps, in order:

1. **Firmware instrumentation** (needs a flash — bench presence): count
   URC-emission entries vs exits and pending-buffer overwrites in the RX
   path; a single new counter (`rx_urc_lost`) placed at the suspected
   race would name the exact line. Read `tx_done_early` on the tractor in
   the same session (exists, never read across a leg).
2. **TX-pattern discriminators** (no flash): vary `TrainGapMs` and
   `TxPipelineDepth` — if the idx-lock rate moves with the transmitter's
   end-of-train timing, the trigger is confirmed TX-side-pattern.
3. The activity-modulation legs (ethernet/USB/CPU flood) are DEPRIORITIZED
   — the drop is inside the L072, not board-level EMI coupling.

## 5. Caveats

- **Log clocks are per-board and unsynchronized** (the tractor's is ~9
  days behind the base's — tx_daemon.log wall stamps will not match the
  archive folder timestamp or rx_daemon.log). Every timing analysis in
  this campaign uses per-board internal deltas only; never correlate
  wall timestamps across the two logs. (Review catch, PR #108.)

- The 88-frame firmware-drop figure is one bracketed leg; the base's dead
  NRST makes counters cumulative, so brackets are cheap to repeat — do
  n=2 next session.
- Δrx_ok includes anything else the base radio heard (its own command
  echoes etc.); the −15 over-count in "radiated vs handled" is consistent
  with that cross-traffic and does not affect the conclusion.
- The cumulative (pre-campaign) identity residue was 78 across history vs
  9 in this leg — historical residue may contain the same drop class or
  past resets; not load-bearing either way.

## 6. Addendum — the TX-pattern discriminators (legs E–F, same session)

Both no-flash levers moved the phenomenon; the drain lever moved it most.
All legs 3000 B / 902.5 MHz, counter-bracketed
(`stats_pre_E/post_E/post_F.txt`; analyzer `tools/rs12_leg_report.py`):

| leg | TX pattern | fw drops | penultimate lock | loss |
|---|---|---:|---:|---:|
| D | gap 40, depth 2 (baseline) | 88 | 28 % | 8.2 % |
| E | **gap 120**, depth 2 | 37 | 26 % | 5.0 % |
| F | gap 40, **depth 1** | 26 | **15 %** | 3.9 % |

Reading, with the run-to-run depth variance (§3) kept in mind — the lock
*share* is the robust metric, the drop *counts* are single samples:

- **Tripling the gap left the lock intact** (26 %) — boundary-relative
  timing is not the trigger. Its drops were purely single-penultimate
  events (37 drops ≈ 37 penultimate-attributed losses), suggesting gap
  width modulates event *depth*, not occurrence.
- **Removing the depth-2 drain halved the lock** (15 %, below every prior
  3000 B leg) and produced the lowest drop count of the series. The
  two-stage mailbox drain that fingerprints every train's ending is
  therefore a major component of the trigger — but not all of it: 16
  penultimate losses remain, ~2× uniform, and at depth 1 the mailbox
  still drains 1→0 at train end.
  > **SUPERSEDED 2026-08-17** by the n=3 interleaved A/B
  > (`../RS_12_depth_ab_2026-08-17/RESULTS.md`). The lock collapse
  > reproduces (37 % → 6.6 %), but the drop-count reduction does NOT:
  > 25.7 vs 24.0 drops, p=0.90. Leg F's low count was run-to-run spread.
  > The drain **aims** the drops, it does not cause them — "a major
  > component of the trigger" is withdrawn.
- Instrument closure held in every leg (Δcrc_err vs crc_dumps: 126/126,
  126/124, 94/94) — corruption and the silent drop remain fully separate
  channels throughout.

Sharpened hypothesis for the firmware instrumentation pass: the L072
RX-URC path loses a pending frame when the *transmitter's end-of-train
drain cadence* coincides with it; drain depth scales the probability,
boundary gap scales how many neighbours the event takes. The
`rx_urc_lost` counter (needs flash) remains the step that names the line;
depth-1 legs give the low-rate control.

Operational note: `-TxPipelineDepth 1` costs throughput (2262 frags
offered vs ~2420 at depth 2, −6.5 %) but cut loss to 3.9 % in this
sample — not a recommendation yet (n=1, variance §3), but worth the n=3
A/B if the firmware fix stalls.

## 7. Addendum 2 (2026-08-17 evening) — MECHANISM FOUND, host-side fix validated

Three instrumented legs (H, I, J at the day's stability channel 927.5 MHz)
with a new dual-clock instrument — per-fragment RX demod timestamps
(`frag_arrival:`, firmware µs clock) and per-fragment TX_DONE times
(`txdone_arrival:`, with `toa_us`) — closed the case. The route had two
wrong turns, both recorded:

**Wrong turn 1 (leg H): "TX compression".** Drop-train neighbours' demod
gaps read 1.16–1.36 slots instead of the expected 2.0, which looked like
the transmitter squeezing the end of the train. The real explanation was
an unexamined assumption: *the last fragment is short* (the ~36 B frame
remainder, ~20 ms ToA), and it rides the firmware's fire-on-TX_DONE ~42 ms
behind the penultimate BY DESIGN. 117+42 = 159 ms ≈ 1.36 slots — drop-train
timing is completely NORMAL. The `len=` field that settles this was in the
instrument's own log line from the start, unread for two legs.

**Wrong turn 2 (this document's §1): the "firmware drop" metric is
ack-contaminated.** Host `rx_frames` excludes command/ack frames (they are
dispatched before the counter increments) while radio `rx_ok` includes
them, so every Δrx_ok−URCs figure in §1/§6 and the depth A/B carries
roughly the leg's ack traffic (~32 frames: 16 base commands × 2 tractor
ack copies) as a constant inflation. The penultimate-loss localisation
survives this — corrupt-capture indices stay uniform while losses lock —
but the absolute drop counts do not separate "URC lost" from "ack not
counted", and the clean instrument remains the future `rx_urc_lost`
firmware counter.

### The mechanism

**The penultimate fragment is the only fragment whose successor arrives
~42 ms later instead of ~117 ms** (the short last fragment rides
TX_DONE). Its full-size 255 B URC therefore has a 2.8×-tighter deadline
through the L072's single pending-URC path than any other fragment; when
emission slips past the deadline, the successor's arrival silently
overwrites the pending URC. Every prior observation follows: the
penultimate lock and its tracking across train lengths (the ride exists at
every length), depth-1 collapsing the lock (no park → no ride → no tight
deadline), gap-width irrelevance (the ride is inside the train), channel/
power irrelevance, role-swap asymmetry (v3 park pattern only in the
tractor→base direction), and corrupt-capture uniformity (this failure
erases, it never corrupts).

Confirmed at both ends in leg I: TX_DONE(pen)→TX_DONE(last) median 42 ms
with full 99904 µs ToA on the penultimate (192 trains), and RX demod gap
for the same pair median 41.7 ms vs 117.0 ms mid-train control.

### The fix (host-side, no flash): `LIFETRAC_NO_PARK_LAST=1`

The TX daemon holds the final fragment until the pipeline drains, so the
last pair is host-paced like every other pair. Leg J (n=1, dual-
instrumented, bracketed):

| metric | leg H (ride) | leg J (fix) |
|---|---:|---:|
| last-pair RX demod gap | 41.7 ms | **114.7 ms** |
| penultimate lock | 42 % | **7 % (= uniform)** |
| raw loss | 3.3 % | **1.7 % — campaign best** |
| reassembler timeouts | 69 | **37** |
| frames published | 117 | **144** |
| offered fragments | 2377 | 2319 (−2.4 %) |

Residual: 24 % of last pairs still arrive <80 ms apart (the hold releases
on TX_DONE receipt, which can still beat the pacer), and the ack-
contaminated drop metric still reads ~36 after subtraction — the firmware
counter session remains the final confirmation. But every user-visible
endpoint moved the right way at n=1, at a third of depth-1's throughput
cost.

### Status

- `-NoParkLast 1` / `LIFETRAC_NO_PARK_LAST=1` is the recommended bench
  operating point pending n=3 confirmation.
- The flash session's brief is now confirmation + real fix, not search:
  add `rx_urc_lost` where the pending URC is overwritten, then either
  double-buffer the URC path or have firmware enforce a minimum
  inter-fire spacing on parked fragments.
