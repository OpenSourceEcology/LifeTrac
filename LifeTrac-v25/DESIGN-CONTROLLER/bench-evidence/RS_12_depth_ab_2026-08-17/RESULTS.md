# RS-12 depth A/B — the drain AIMS the drops, it does not cause them

Issue #107. SHA `4f8a0ad4`. Six 300 s legs at 909.0 MHz, **interleaved**
(2,1,2,1,2,1), n=3 per arm, every leg counter-bracketed.
Driver: `firmware/x8_lora_bootloader_helper/run_rs12_depth_ab.ps1`.
Aggregator: `tools/rs12_ab_summary.py`.

## 1. Result

| leg | depth | loss % | penult % | fw drops | offered | goodput |
|---|---:|---:|---:|---:|---:|---:|
| 01 | 2 | 3.7 | 37 | 22 | 2414 | 2018 |
| 02 | 1 | 2.1 | 7 | 15 | 2262 | 1866 |
| 03 | 2 | 4.1 | 35 | 17 | 2414 | 1994 |
| 04 | 1 | 2.1 | 8 | 25 | 2263 | 1850 |
| 05 | 2 | 3.4 | 40 | 38 | 2415 | 2018 |
| 06 | 1 | 2.0 | 4 | 32 | 2264 | 1857 |

| metric | depth 2 | depth 1 | delta | perm p |
|---|---:|---:|---:|---:|
| **penultimate lock** | 37.1 % ±2.5 | **6.6 % ±2.0** | **−30.5** | 0.10 |
| raw loss | 3.8 % ±0.4 | **2.1 % ±0.1** | −1.7 pp | 0.10 |
| goodput | **2010 B/s ±14** | 1858 B/s ±8 | −152 | 0.10 |
| **firmware drops** | 25.7 ±11.0 | 24.0 ±8.5 | −1.7 | **0.90** |

No leg overlaps the other arm on loss, penultimate share, or goodput.
p=0.10 is the exact permutation FLOOR at n=3/arm (2 of 20 labelings), i.e.
the cleanest separation this design can produce — not "weak significance".

## 2. The mechanism, corrected

**Dropping the TX mailbox depth removes the penultimate lock but does NOT
reduce the number of silently-dropped frames** (25.7 → 24.0, p=0.90).

So the depth-2 drain is a **phase selector, not a trigger**. The same
~25 demodulated frames per leg vanish inside the L072 either way; at
depth 2 they land on the second-to-last fragment, at depth 1 they scatter.

**This corrects the 2026-08-16 addendum**, which called the drain "a major
trigger component" on the strength of a single depth-1 leg showing both a
halved lock *and* the series' lowest drop count. With n=3 the drop counts
are indistinguishable — leg F's low count was the run-to-run spread, and
the trigger conclusion drawn from it was wrong. The lock collapse is real
and reproducible; the drop reduction is not.

Why loss still improves: ~25 losses aimed at one index kill trains
outright, while ~25 scattered losses damage ~25 different trains shallowly
and many still complete. Same defect, better-distributed damage.

### Precision note on "uniform"

Depth-1 losses are **not** uniform, only no longer penultimate-locked.
Chi-square against uniform over 13 indices (representative legs):

```
depth 2: n=98  chi2(12df) = 155.9   strongly non-uniform (idx11 = 39)
depth 1: n=67  chi2(12df) =  25.6   still non-uniform (crit 21.0 at p=.05),
                                     mild excess at idx 2 (14 vs 5.2 exp)
```

An earlier phrasing of this result as "essentially uniform" overstated it.
The penultimate share does fall to ~uniform (6.6 % vs 7.7 % expected), but
a smaller, different structure remains and is unexplained.

## 3. The trade, and the recommendation

Depth 1 buys **−1.7 pp loss for −7.6 % goodput** (2010 → 1858 B/s). Real,
reproducible, and cheap to enable.

**Recommendation: do NOT adopt it yet.** It redistributes damage without
fixing the defect — the firmware still discards the same ~25 frames per
leg. Adopting it would spend 7.6 % of throughput on symptom management and
make the underlying bug *harder* to see, since the phase-lock is currently
the most legible signature we have.

**Where it IS useful: as the instrumentation condition.** The
`rx_urc_lost` counter should be exercised on a **depth-1** leg, where the
drops are spread and the confounding phase-lock is absent — a cleaner
substrate for locating the race than the depth-2 baseline.

## 4. Caveats

- **The channel drifted during the series.** Pre/post checks at 909.0
  found 4 and 3 hot samples at −41/−42 dBm, where the survey ~1 h earlier
  read hot=0 / max −94 — the hopper wandered onto our carrier mid-A/B.
  The interleaved design absorbed it (that is why it was interleaved), and
  the arm separation is unaffected, but **the absolute loss figures here
  sit on a mildly contaminated channel** and should not be compared with
  the day-1 legs.
- n=3 per arm; the permutation floor means p can never read below 0.10.
  The effect sizes (30 points of lock, 1.7 pp of loss) carry the argument,
  not the p-values.
- Firmware-drop spread is wide in both arms (±11 and ±8.5 on means of
  ~25), so the p=0.90 null is a genuine "no detectable difference", not a
  precise equality. A larger n could still reveal a small real difference.

## 5. Tooling notes

- `run_rs12_depth_ab.ps1` initially wrote **empty archive paths** into
  `legs.json`: the harness reports via `Write-Host`, which does not reach
  the pipeline, so output capture yielded nothing. The mapping was
  reconstructed from directory timestamps and then **verified leg-by-leg
  against each archive's own `params.txt`** rather than trusting order.
  The driver now detects newly-created archive directories instead.
- `rs12_ab_summary.py` uses an exact permutation test rather than a
  t-test: no distributional assumption, and it cannot manufacture
  significance the data lacks (validated — returns 0.50 on overlapping
  arms, 0.10 at maximum separation).
