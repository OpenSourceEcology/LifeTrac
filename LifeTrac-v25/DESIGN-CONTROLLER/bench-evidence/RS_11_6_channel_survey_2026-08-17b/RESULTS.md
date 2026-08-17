# Third band survey — 927.5 MHz is the stability pick (3/3 clean)

SHA at sweep: `8ef0aa5d`. Trigger: possible second mains blink (both
carriers again survived on continuous uptime — this is a pure RF check).
Transcript: [`survey_base_day2b.jsonl`](survey_base_day2b.jsonl).
Analysis: `tools/survey_compare.py` (now with `--history` stability mode).

## 1. The band reshuffled again — in ~4.5 hours this time

10 of 53 channels flipped vs the morning survey (19 %), including:

- **909.0 — the morning's pick and the depth-A/B channel — went −94 → −41**
  (confirming the mid-A/B contamination the pre/post checks caught)
- 918.0 went −94 → −40; 921.0 went −91 → −41
- **924.0 lit up at −30 dBm — device B's signature level**, its first
  reappearance at that strength since the day-1 survey localized it

Flip rates are now ~20 % per survey interval at both ~14 h and ~4.5 h
spacings, so the reshuffle cadence is faster than daily and may not need
a power event at all. Either way the operational rule is unchanged and
now triply confirmed: **a channel pick is valid for hours, not days.**

## 2. The stability ranking — one channel stands alone

Across all three surveys (`--history` mode):

```
927.5 MHz   clean 3/3   worst max −75 dBm   <== clean in every survey
(every other channel: clean in at most 1 of 3)
```

927.5 is also today's absolute quietest (−96 dBm). **Standing bench
recommendation: `-ForceFrfHz 927500000`**, held not because any pin is
durable but because it is the only channel with a perfect record — and
still subject to the same-day check before any measurement leg
(`hunt_sniff.ps1`-style spot check or the sweep).

Plausible physics for why the top edge stays clean: 927.5+250 kHz grazes
the 928 MHz band edge, which hopping devices typically back away from to
keep their occupied bandwidth legal — the same reason our own 902.0 was
rejected as a center. The hopper may simply never park there. Two more
clean surveys would make that a confident claim; it is a hypothesis today.

## 3. Consequences

1. RS-12's flash-session leg should run at **927.5** with the usual
   same-day verification.
2. RS-11.7 v1 should rank by **stability across accumulated surveys**,
   not per-day quietness — `survey_compare.py --history` is that logic,
   already written and validated.
3. The depth-A/B absolute loss numbers (909.0, mid-drift) keep their
   contamination caveat; the arm comparison stands.
