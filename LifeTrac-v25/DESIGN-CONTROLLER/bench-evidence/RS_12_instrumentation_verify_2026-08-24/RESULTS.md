# Archive instrumentation verification — the hold setting is now self-evidenced (2026-08-24)

**Verdict: PASS, and stronger than a format check.** The PR #111 review
established that no archive could substantiate whether `-NoParkLast 1`
was actually in force — `params.txt` had no field and the TX daemon
never logged one, so every strict-hold claim rested on the session
transcript. The instrumentation added in PR #111 is now verified on air
by a paired leg: the recorded setting flips with the flag **and the
mechanism signature flips with it**, which is what distinguishes a
faithful record from an echo of a parameter.

## Method

Two 300 s synth legs, same channel, same session, ~6 min apart, only
`-NoParkLast` differing. Channel 927.5 MHz spot-checked clean the same
morning (60 s, 949 samples, 0 hot, max −94 dBm — **now clean 5/5** across
all surveys). Both legs bracketed on both boards (`inst_*` files here).
Boards had been quiesced overnight (radios in LoRa SLEEP, Linux up); the
L072s woke clean, `RS115-INSTRUMENTED-FIRMWARE=YES` both.

## Result

| | leg A (fix) | leg B (control) |
|---|---|---|
| archive | `radio_monitor_20260824_112905_e9f6ffd6` | `radio_monitor_20260824_113520_e9f6ffd6` |
| `params.txt` | **`no_park_last=1`** | **`no_park_last=0`** |
| TX daemon startup log | **`no_park_last=1 gap_ms=80 pipeline_depth=2`** | **`no_park_last=0 gap_ms=80 pipeline_depth=2`** |
| loss | **1.5 %** (33/2177) | **3.3 %** (78/2362) |
| penultimate (idx 11) | **6 %** (2/33) — at/below uniform | **35 %** (27/78) — 4.4× uniform |
| timeouts | 31 | 69 |
| crc closure | 26 = 26 exact | 70 = 70 exact |

Both loss figures land on the RS-12 arm values measured a week earlier
(ctrl 3.3 ±0.1, fix 1.5–1.8) — an independent reproduction on a
different day, different channel-day, and post-merge code.

**What this closes:** an archive from the instrumented build now proves
its own hold state two ways — the declared setting, and the physics that
setting produces. Future legs (starting with the motion leg) no longer
need transcript attestation.

## Tooling bug found and fixed (would have hidden this result)

`tools/rs12_leg_report.py` inferred train length from the `total` byte of
**corrupt capture headers** — the one population whose bytes are by
definition unreliable — with `most_common(1)` and no sanity bound. Leg B
had exactly ONE readable corrupt dump, whose garbage byte said 208, so
the tool reported *train length 208, penultimate idx 206 = 0 (0 %)* while
the real lock sat at 35 % on idx 11. **A false negative on the campaign's
headline metric, from a single corrupted byte.**

The first repair (modal over all TX-log trains) was also wrong and is
recorded here because it is instructive: these legs carry a **mixture**
of train lengths (leg B: 1×143, 2×45, 12×56, 13×115), so the overall
mode is 1 and "penultimate" collapses to idx −1. Short trains have no
penultimate to lock; the metric is only meaningful over long trains.

Final behaviour: train length = modal over trains of ≥3 fragments, taken
from the TX log's healthy `K fragments ok` lines, with the **full mixture
printed** so a mixed-length leg can never be mistaken for a uniform one,
and the uniform baseline printed alongside the penultimate share.

Regression-checked against historical archives — reproduces every
published number exactly: yesterday's synth leg 3 % (unchanged), RS-12
strict-hold leg L 8 % (unchanged, matches the closure record).

**Implication for earlier analyses:** any leg whose penultimate share was
computed from few readable corrupt dumps could have carried the same
distortion. The re-checks above cover the legs the RS-12 closure and the
2026-08-22 session rest on, and both are unchanged; no published figure
moves. Legs analysed with `--pre/--post` bracketing print the train-length
source line from now on, so the inference is auditable per run.
