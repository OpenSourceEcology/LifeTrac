# RS-11.8 — second chantab-grid survey: no production channel is clean, and survey ranking does not predict link loss (2026-08-24)

**Headline, after the link test below overturned this document's first
conclusion: 50/50 chantab channels were hot this pass (08-22's lone
"clean" 904.25 went −84 → −48), so the zero-hot criterion is saturated
on the production grid — as predicted. A continuous-metric ranking does
produce an ordering, and the top edge does score quieter, BUT the
top-ranked channel (927.25) then lost 2.3× more traffic on a real link
leg than off-grid 927.5, because the ~7 s ticker is sitting on it.
Passive ranking within an emitter-occupied band is not channel
selection. No channel in the production table is currently
emitter-free.**

## Captures

- `chantab_survey_20260824.jsonl` — 50 channels, 902.75–927.25 MHz,
  30 s dwell, 0.05 s sampling, base radio, container `chantab2`.
- Compare against `RS_11_8_chantab_2026-08-22/chantab_survey_20260822.jsonl`.
- Same-day x.0/x.5 control: 927.5 MHz spot-check 0 hot / max −94 dBm
  (**clean 5/5** across all surveys to date).

## Cross-survey ranking (sum of hot counts, tiebreak worst peak)

| channel | hot sum (08-22 + 08-24) | worst peak |
|---|---:|---:|
| **927.25 MHz** | **4** (2+2) | **−56 dBm** |
| 906.25 | 4 (2+2) | −48 |
| 913.25 | 4 (2+2) | −48 |
| 921.25 | 4 (2+2) | −46 |
| 925.25 | 5 (2+3) | −56 |
| 926.75 | 5 (3+2) | −52 |
| … | | |
| 922.75 / 923.25 (worst) | 12 | −49 / −46 |

**Band median hot-sum 7.5; top-edge (≥925 MHz) median 5.0.** Four of the
five quietest-by-peak channels sit at the top edge, and 927.25 is the
only channel simultaneously in the lowest hot-count group AND the
lowest-peak group.

⚠️ **This ordering did NOT survive its link test — read the next section
before using this table for anything.** The top-edge *gradient* is real
as a statistic, but it does not mean the band-edge protection that keeps
927.5 clean extends onto the grid: it does not. 927.25 ranked first here
and still carries the ticker.

## ⚠️ RETRACTED: the survey-ranked candidate FAILED its link test

An earlier revision of this document recommended **927.25 MHz** as hail
candidate #1 on the strength of the ranking above, and argued the
table-extension question was thereby *less* urgent. **A link leg run the
same session refuted both claims and they are withdrawn.**

### The link test

`radio_monitor_20260824_121253_be989c0a` — 300 s synth leg at **927.25**,
`no_park_last=1` (archive-recorded), bracketed, identical config to the
morning's 927.5 leg A:

| | 927.5 (leg A) | **927.25 (this leg)** |
|---|---:|---:|
| loss | 1.5 % | **3.5 %** |
| crc dumps | 26 | 63 |
| timeouts | 31 | 61 |
| penultimate share | 6 % | 11 % (≈ uniform 8 %) |

Loss is **2.3× worse**, and the per-index drop profile is flat
(0:5 1:7 2:11 3:5 4:4 5:6 6:2 7:3 8:6 9:8 10:6 11:8) — interference
shaped, not the RS-12 mechanism (the hold is working: penultimate sits
at uniform).

### Why, confirmed directly

Paired 60 s spot-checks run back-to-back immediately after the leg, same
instrument, minutes apart:

| channel | hot | max | pattern |
|---|---:|---:|---|
| **927.25** | **5** | **−55 dBm** | inter-hit gaps 7.02 / 6.99 / 21.07 (=3×7.02) / 7.08 s — **the ~7 s device-A ticker is ON this channel** |
| 927.5 | 0 | −94 dBm | nothing |

So the two channels are not "similar with a gradient" — one carries the
emitter and one does not.

### What this actually teaches

1. **Passive hot-COUNT ranking at 30 s dwell is a weak predictor of link
   loss.** 927.25 scored hot=2 per dwell and ranked best of 50; it then
   lost 2.3× more traffic than a channel the same ranking could not even
   see (927.5 is off-grid). Rank ordering *within* an emitter-occupied
   population is not the same as finding a channel outside the emitter's
   footprint.
2. **No chantab channel is clean.** The best production-grid channel is
   merely *less hit* (hot-sum 4 of two dwells); 927.5 is qualitatively
   different — zero hits at a −94 dBm floor, now 5/5 surveys.
3. **The table-extension question therefore gets MORE urgent, not less**
   — the opposite of the retracted claim. If the production hop/hail set
   must live inside 902.75–927.25, it has no emitter-free channel today,
   while a legal channel 250 kHz above the table's top is repeatedly
   clean.

### Standing recommendation (replaces the retracted list)

- **Do not adopt hail-set constants from passive survey ranking alone.**
  Every candidate must pass a link leg; that is now the RS-11.8 gate.
- Candidates still worth link-testing (top-edge, quietest by survey):
  926.75 and 925.25. Test them the same way — 300 s leg + paired
  post-leg spot-checks — before any of them enters POWER_MANAGEMENT.md.
- Carry the finding that **927.5 is the only measured emitter-free
  channel** into the table-extension decision.
- Caveats retained: n=1 link leg per channel, and the bench RF
  environment varies run to run (crc dumps swung 26→70 at *fixed*
  927.5 between legs A and B), so replicate before treating the 2.3×
  as a precise ratio. The direction is corroborated by the independent
  spot-checks; the magnitude is not yet pinned.

## Method note carried forward

The zero-hot criterion should not be used on this grid while the ticker
is active. `survey_compare.py` ranks by `hot` first, so it will report
"0 of 50 zero-hot" and fall back to peak ranking — correct but easy to
misread as "no candidates" rather than "criterion saturated".

**Do NOT fold the hot-sum ranking into the tool as a picker** (an
earlier revision of this note proposed exactly that, before the link
test). This session's result is that the ranking's own top pick loses
2.3× on air, so promoting it to an automatic recommendation would
automate a wrong answer. What the tool should surface instead:

- the **hit CADENCE**, not just the count — 927.25's five hits fell on a
  clean 7.02 s grid, which identifies the emitter positively; a channel
  with the ticker's period on it is disqualified regardless of rank;
- the **noise floor** alongside the peak — 927.5 reads −94 dBm floor vs
  927.25's −55 dBm peak: an order-of-magnitude difference in kind, not
  a rank difference;
- an explicit "no clean channel found" verdict when every candidate is
  hit, rather than a ranked list that implies one is good enough.

RS-11.7 v1 should therefore present survey output as *screening*, with
the link leg as the gate.
