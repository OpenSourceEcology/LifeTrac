# RS-11.8 — second chantab-grid survey: the binary criterion is dead, the gradient works (2026-08-24)

**Headline: 50/50 channels hot this pass (08-22's lone "clean" 904.25
went −84 → −48), so the zero-hot criterion is fully saturated on the
production grid and cannot pick a channel — as predicted. But ranking on
the CONTINUOUS metrics across both surveys does discriminate, and it
reproduces the band-edge pattern found on the x.0/x.5 grid: the
chantab's top edge is systematically quieter than the band.**

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
five quietest-by-peak channels sit at the top edge, and 927.25 — the
chantab channel adjacent to our 5/5-clean 927.5 — is the only channel
that is simultaneously in the lowest hot-count group AND the lowest-peak
group. The band-edge protection seen on the x.0/x.5 grid therefore
extends onto the production grid.

## Consequence: first evidence-based hail-set candidates

POWER_MANAGEMENT.md's rendezvous design needs 2–3 stable channels drawn
from the production table. On this evidence the candidates are, in order:

1. **927.25 MHz** — chantab idx 49, top edge, best on both metrics
2. **926.75 MHz** — top edge, hot-sum 5, peak −52
3. **925.25 MHz** — top edge, hot-sum 5, peak −56

Bounds on this recommendation, stated plainly:

- **n=2 surveys, same week.** The x.0/x.5 history needed four passes
  before 927.5's 4/4 record meant anything; this is two. Treat as
  provisional and keep accumulating (one chantab pass per survey day).
- The separation is real but not dramatic (hot-sum 4–5 vs median 7.5);
  it is a gradient, not a clean/dirty split, and it rests on a ticker
  whose channel occupancy reshuffles in hours.
- 927.25 sits at the table's upper limit, so a 500 kHz occupied
  bandwidth centred there reaches 927.5 — inside the band, but the
  **top guard is only 500 kHz**. That is legal and is exactly why the
  channel is quiet, but it should be a deliberate decision, not an
  accident.
- This does not resolve the table-extension question (adding 927.5
  itself): it makes it *less* urgent, because a table channel with
  similar protection now exists.

## Method note carried forward

The zero-hot criterion should not be used on this grid while the ticker
is active. `survey_compare.py` still ranks by `hot` first, so it will
report "0 of 50 zero-hot" and then fall back to peak ranking — correct
but easy to misread. The ranking that produced the table above (hot-sum
across surveys, tiebreak worst peak) is the one to use for chantab
picks; folding it into the tool is the obvious next step for RS-11.7 v1.
