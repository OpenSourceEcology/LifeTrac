# Day-2 survey (post-outage) — the interferer HOPS; a pinned channel is perishable

SHA `6d248b63`. Trigger: a thunderstorm/mains outage overnight
2026-08-16→17. Neither carrier rebooted (uptime 21:45 / 22:34, counters
continuous), so this is a pure RF-environment change.

Transcript: [`survey_base_day2.jsonl`](survey_base_day2.jsonl).
Tools: `channel_survey_sniff.py`, `tools/survey_compare.py` (new).

## 1. What the outage did to the band

Identical sweep to 2026-08-16 (902–928 MHz, 500 kHz steps, 30 s dwell):

| | 2026-08-16 | 2026-08-17 |
|---|---:|---:|
| hot channels | 46 / 53 | 43 / 53 |
| zero-hot legal centers | 5 | 8 |

**11 of 53 channels (21 %) flipped state overnight**, and the flip was
catastrophic for the pin we had chosen:

| channel | 2026-08-16 | 2026-08-17 |
|---|---|---|
| **902.5 — the pinned escape** | clean, −94 | **HOT, −44** |
| 905.5 | clean, −86 | **HOT, −43** |
| 907.0 | clean, −77 | **HOT, −43** |
| 925.5 | clean, −94 | **HOT, −44** |
| 927.5 | clean, −75 | clean, −94 |
| 909.0 | HOT, −43 | clean, −94 |
| 918.0 / 918.5 / 921.0 / 924.0 | HOT, −42…−53 | clean, −91…−94 |

**Four of the five channels that were clean yesterday are hot today.**

### Correction to the RS-11.6 record

Two earlier readings are superseded:

1. **"Standing recommendation: bench legs run `-ForceFrfHz 902500000`"**
   (leg-3 RESULTS §3.2) — withdrawn. That channel is now among the worst.
   `-ForceFrfHz` is a **per-session parameter chosen from a same-day
   survey**, never a config constant.
2. **"Device A is wideband or front-end leakage; carrier unlocalised"**
   (leg-3 RESULTS §1) — wrong branch. The device is a **frequency
   hopper**: yesterday's "−43 dBm on essentially every channel" was one
   emitter observed across a 24-minute sweep, and the outage restarted it
   into a different channel sequence. This restores the original
   utility-meter/AMI hypothesis that leg 2 had talked itself out of.

Also worth recording: an intermediate claim made during this session —
"the emitter is gone" — was made off a single 120 s sniff at 915 MHz and
was **wrong**; targeted sweeps immediately found it alive at −42…−46 dBm
on 922–925 with clean ~7.0 s intervals. One channel is never evidence
about a hopper.

## 2. Today's pick, and a stability alternative

`survey_compare.py` ranks zero-hot legal centers by quietest peak:

```
RECOMMENDED: -ForceFrfHz 909000000   (909.0 MHz, hot=0, max=-94 dBm)
runners-up:  918.0, 924.0, 927.5 (all hot=0, max -94)
```

**927.5 MHz was the only channel clean on BOTH days** (−75 → −94). If
run-to-run comparability matters more than today's absolute quietest, it
is the better long-run pin — two days is a thin basis, but it is the only
stability signal available and worth accumulating.

## 3. Re-baseline at 909.0 (leg G) — and RS-12 is untouched by any of this

Standard 300 s leg, counter-bracketed
(`../RS_12_bulk_floor_2026-08-16/stats_pre_G.txt` / `stats_post_G.txt`):

| metric | leg G @ 909.0 (day 2) | leg D @ 902.5 (day 1) |
|---|---:|---:|
| raw loss | 6.8 % | 8.2 % |
| firmware drop (Δrx_ok − URCs) | **78** | 88 |
| penultimate lock | **35 %** | 28 % |
| crc closure | 94 vs 91 | 126 vs 126 |
| identity residue | 8 | 9 |

The RS-12 mechanism reproduces on a completely different, freshly-clean
channel: the penultimate lock is intact at 35 % and ~78 demodulated
frames per leg still vanish inside the L072. **The firmware race is
channel-independent**, as the counter separation always implied — the
day-1 conclusions stand without re-work.

## 4. Operational consequences

1. **Survey before any measurement campaign.** Not hygiene — correctness.
   Had we trusted yesterday's pin, leg G would have run on a −44 dBm
   channel and shown a large "regression" that would have been hunted in
   firmware. 28 minutes unattended prevents that.
2. **RS-11.7 (operator survey in web_ui) is promoted** from
   nice-to-have to the practical prerequisite for field measurement. Its
   v0 (survey + recommend, manual apply) is exactly the workflow used
   here, and `survey_compare.py` already implements the recommendation
   logic the button needs.
3. Record `force_frf_hz` with every citation of a run — it is now a
   varying parameter (already in `params.txt`).
