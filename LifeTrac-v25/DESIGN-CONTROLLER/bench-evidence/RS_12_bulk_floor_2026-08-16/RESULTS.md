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

- The 88-frame firmware-drop figure is one bracketed leg; the base's dead
  NRST makes counters cumulative, so brackets are cheap to repeat — do
  n=2 next session.
- Δrx_ok includes anything else the base radio heard (its own command
  echoes etc.); the −15 over-count in "radiated vs handled" is consistent
  with that cross-traffic and does not affect the conclusion.
- The cumulative (pre-campaign) identity residue was 78 across history vs
  9 in this leg — historical residue may contain the same drop class or
  past resets; not load-bearing either way.
