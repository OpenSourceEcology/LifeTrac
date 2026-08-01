# RS-11.1 first measurement — the F4 gate answers NO

**Run:** `radio_monitor_20260729_224909_82f76fb2`, 240 s, DTS profile 2, v3
pipeline, 3000 B synthetic frames, `ProbeEcho 0`, `KfRequestDisable 1`,
`ReactiveFire 0` — the same pinned operating point as the C2/C2b baseline.
`frags_ok=1915`, `rx_frames=1848`, `rx_decode_err=0`.

---

## 1. Loss is NOT clustered at fragment index 0 — it rises with position

67 losses attributed across 22 stats intervals:

| frag idx | 0 | 1 | 2 | 3 | 4 | 5 | 6 | 7 | 8 | 9 | 10 | 11 |
|---|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|
| losses | **1** | 4 | 5 | 2 | 2 | 6 | 6 | 4 | 9 | 9 | **11** | 8 |

Index 0 took **1 of 67 losses (1.5%)**. Uniform loss across ~12 positions would
predict 7.7%. The distribution instead climbs monotonically toward the end of
the train: indices 8–11 account for 37 of 67 (55%), indices 0–3 for 12 (18%).

**This refutes the RXCONT re-arm-gap hypothesis.** The first fragment after a
train boundary is the *least* likely to be lost, not the most. Whatever the
receiver is doing at a train boundary, it is not missing the packet that arrives
there.

### Consequence: F4 (settable preamble) must not be flashed on this rationale

Preamble length widens the window in which a *re-arming* receiver can catch a
packet. That mechanism is now measured to not be our failure mode. RS-10.1 is
therefore closed as **not-supported**, and the firmware cycle is saved.

**Retraction.** Twice on 2026-07-29 this preamble work was called "the single
highest-value PHY experiment we are not running", on the strength of the
`rx_decode_err = 0` result — which shows only that losses are *pre-header-lock*,
not *where in the train* they occur. Pre-header-lock is consistent with several
mechanisms and I collapsed it to one without measuring. The zero-decode-error
finding stands; the preamble conclusion drawn from it does not.

### The new hypothesis, deliberately not yet chosen

Loss rising monotonically with position inside a ~1.3 s continuous train points
at something cumulative, not something positional-at-the-boundary. Candidates,
none yet tested:

- **PA thermal droop** — 13 × 100 ms of near-continuous keying; later fragments
  transmit at slightly lower output.
- **Dwell/QoS throttling** — the airtime accountant or the 95%-duty DTS budget
  gate biting later in a train.
- **Receiver-side backpressure** — the base's host pipeline falling behind as
  the train progresses, so RXCONT re-arm slips further with each fragment.
- **Intra-train clock drift** — the slot follower drifting across the train so
  later fragments land further from the expected window.

Discriminating these is cheap and should precede any firmware change: shorten
the train (fewer fragments per frame) and see whether the *per-index* rate flattens.
If loss tracks elapsed-time-since-train-start rather than index, it is thermal
or dwell; if it tracks index regardless of train length, it is pipeline depth.

---

## 2. The train boundary is now measured, for the first time cleanly

| gap class | samples | median | range across intervals |
|---|--:|--:|---|
| `seq` (contiguous, within a frame) | 1627 | **104.9 ms** | 104.9–104.9 |
| `boundary` (frame changed) | 155 | **213.9 ms** | 183.0–278.9 |
| `post_loss` (index jumped) | 65 | **210.3 ms** | 209.7–287.3 |

Three things worth keeping:

1. **Inter-fragment dead air is 5.0 ms** (104.9 − 99.904 ToA), reconfirming the
   4.87–5.01 ms already on record and again contradicting any per-fragment
   host-overhead figure in the tens of milliseconds.
2. **`post_loss` sits at ~2× ToA, exactly as predicted** — which is the direct
   demonstration of why the old aggregate histogram could not be trusted: a
   post-loss gap (210.3 ms) and a train boundary (213.9 ms) were within 4 ms of
   each other and landed in the same bucket. 65 of the 220 oversized samples in
   this run were losses masquerading as boundaries, ~30%.
3. **The boundary is ~214 ms, not ~40 ms.** `LIFETRAC_TRAIN_GAP_MS` is 40, so
   ~174 ms of the boundary is host turnaround that prepare-ahead is not hiding.
   That is the input RS-11.2's gap sweep needed, and it was previously
   unmeasurable.

---

## 3. Two blind spots, both found and closed

The instrument had a symmetric pair of holes, each invisible to naive index
arithmetic because a lost fragment at a frame *edge* looks exactly like a clean
frame transition:

- **Leading fragments** — caught during development. A new `frag_seq` first seen
  at index > 0 now attributes indices 0..idx−1 as lost. Without this the
  classifier was structurally blind to the very case F4 exists for, and would
  have reported "no index-0 loss" by construction.
- **Trailing fragments** — caught while writing up this run. If the outgoing
  frame's last seen index is below `total − 1`, the tail was lost. Fixed using
  the `total` already carried in the arrival tuple.

**Caveat on the numbers above: run D1 was collected with the trailing-fragment
hole still open,** so the per-index table undercounts the final index. Two
consequences, both worth stating plainly:

1. The 67 attributed losses reconciling against `frags_ok − rx_frames`
   (1915 − 1848 = 67) is weaker evidence than it looks. `rx_frames` also counts
   the tractor's `0x68` encode acks, so the two totals are not the same
   population and the agreement is partly coincidental.
2. **The undercount runs in the direction that strengthens the conclusion.**
   Loss rises toward the end of the train, so the missing samples are precisely
   the ones that would push indices 11–12 higher. Index 0's 1.5% share can only
   fall further against a larger tail. The "not clustered at 0" verdict is
   therefore safe; the exact shape of the tail should be re-measured on the
   fixed build before anyone reasons quantitatively about it.


---

# RS-11.4 train-length / duty-cycle sweep, 2026-07-30

Six runs, 240 s each, all else pinned to the D1 operating point.

## The valid comparison: E1 vs E2 (both saturated)

| run | frame | fragments/train | total loss | idx-0 rate | late/early ratio |
|---|---|---|---:|---:|---:|
| E1 | 3000 B @ 2 fps | 13 | 3.73% | 1.95% | **2.87x** |
| E2 | 750 B @ 2 fps | 3 | 3.49% | 3.44% | **1.07x** |

**The per-index gradient depends on train LENGTH, not on index.** Long trains
climb from ~2% early to 11.04% at index 11; short trains are flat (3.44 / 3.66 /
3.44 / 5.17%) at statistically identical total loss. A 3-fragment train loses
uniformly; a 13-fragment train loses progressively.

That rules out anything index-intrinsic (a fixed pipeline slot, a per-index
code path) and points at a mechanism that **accumulates over the ~1.3 s a long
train takes**. It also independently re-confirms the F4 closure: in the flat
short-train case index 0 is no better or worse than any other position.

## The low-duty runs cost three instrument fixes, and are only now trustworthy

E3–E6 held train length at 13 and dropped offered load to 0.4 fps (~50% duty) to
test whether idle time between trains lets the mechanism recover — the
thermal/dwell discriminator. Every one of them over-attributed loss, and chasing
that found three real defects in the classifier:

| run | attributed | actual missing | over-count | fix applied after |
|---|---:|---:|---:|---|
| E3 | 130 | ~32 | ~4x | — |
| E4 (prepare-ahead off) | 170 | ~28 | ~6x | — |
| E5 | 77 | ~60 | 1.3x | out-of-order retraction |
| E6 | 80 | ~60 | 1.3x | book each (seq,idx) at most once |

1. **Out-of-order arrival misread as loss.** The host URC queue does not always
   drain in order: 11 and 12 arrive, 9 and 10 are booked lost, then 9 and 10
   turn up. The in-order assumption was documented in the code as an assumption
   and is now measured to be false. Bookings are provisional and retracted on
   arrival; 54 retractions in E6.
2. **Retraction fired only on a backwards step.** Once a late fragment landed,
   the next one presented as a normal +1 step, so 9 was retracted and 10 was
   not. Now checked on every arrival.
3. **Double booking.** `_pending_lost` is a set but the histogram was
   incremented unconditionally, so replayed churn (9 -> 11 seen twice) inflated
   an index while only one booking was retractable.

**E4's result should not be read as evidence about prepare-ahead.** Disabling it
appeared to make things worse, but E4 predates all three fixes and sat at the 6x
over-count. That test needs re-running.

## What survives, and what is still open

**Survives:** a deterministic failure of **fragment index 10 in ~59% of long
trains at low duty** (55/93 in E6), which persisted through all three fixes and
now roughly reconciles with the byte counters — index 10 accounts for most of the
run's total loss. This is a sharp, repeatable, single-index notch, not a smooth
ramp. Smooth thermal droop does not produce that shape.

**Still open — and the sweep did NOT answer its original question.** Whether the
long-train gradient is thermal, dwell-window, or receiver backpressure remains
undetermined, because it took until E6 to get an instrument whose attribution
reconciles. The next session should:

- Re-run the duty-cycle comparison (2 fps vs 0.4 fps) on the E6 build, n=2, now
  that attribution is trustworthy.
- Re-run the prepare-ahead A/B on the E6 build.
- Chase the index-10 notch directly: it is at ~1049 ms into the train, it is
  duty-dependent (a gradient at 2 fps, a spike at 0.4 fps), and it is the single
  largest identified contributor to the loss floor.

**Method note.** Every low-duty conclusion in this section would have been wrong
without the arithmetic check "does attributed loss reconcile with
frags_tx - rx_frames". Three separate defects hid behind plausible-looking
histograms. Run that check before believing any per-index result.


---

# F-series: the index-10 notch characterised, 2026-07-30

Four duty-cycle points, all 3000 B / 13-fragment trains, all else pinned. Each
fragment is 99.904 ms on air and lands 104.9 ms apart, so a train occupies
~1.364 s.

| run | fps | frame period | **idle between trains** | notch at idx 10 | late/early |
|---|---:|---:|---:|---|---:|
| E1 | 2.0 | 0.50 s | 0 (saturated) | **no** — smooth gradient to 11.04% at idx 11 | 2.87x |
| F1 | 0.5 | 2.00 s | 0.64 s | **no** — scattered, max 8.8% | 1.39x |
| E5/E6 | 0.4 | 2.50 s | 1.14 s | **YES ~59%** (55/93), reproduced twice | 5.78x |
| F2 | 0.3 | 3.33 s | 1.97 s | **YES 55.07%** (38/69) | 8.27x |

## What this rules out

**Frame-period locking is dead.** The prediction was that a once-per-period
event would move the notch: at 0.5 fps it should have appeared near index 5. It
did not appear at all, and at 0.3 fps it returned to **exactly index 10** rather
than moving. The notch is fixed to a position in the TRAIN, not to the frame
clock.

## What the data now says

Two independent conditions, both required:

1. **Position: index 10 = 999 ms of cumulative airtime into the train.**
   Essentially exactly one second. Identical at 0.4 and 0.3 fps despite the
   frame period changing by a third.
2. **A preceding idle longer than a threshold between 0.64 s and 1.14 s.** Below
   it (0.64 s idle, and saturated) there is no notch; above it (1.14 s, 1.97 s)
   the notch is present at ~55-59% and does not grow much with more idle.

That is the signature of something that **arms or resets during a sufficiently
long idle, then trips at ~1 s of continuous airtime**. It is not a gradual
accumulation — it is a switch. Note also that the saturated case shows a smooth
gradient rather than a notch, which is consistent with the same trip point
existing but never being armed (no idle), leaving only whatever produces the
milder positional trend.

Candidates worth checking against this shape, in rough order of fit:

- A **1-second window accumulator** (duty-cycle, dwell, or QoS budget) whose
  credit is refilled by idle and exhausted by ~1 s of airtime. The DTS legal
  dwell is 400 ms over a 10 s window, which does NOT fit 1 s, so if this is the
  mechanism it is a different budget — the DTS "95% duty" gate is the obvious
  one to check.
- A **receiver state machine that demotes during idle** (LOCKED to SCANNING, or
  a scan/re-anchor path) and then mis-handles re-acquisition at a fixed offset.
- Anything on either host with a **~1 s timer** that is restarted by idle.

## Why this matters beyond the bench

The saturated case is the operating point we actually run, and it shows no
notch. But a real tractor stream is **not** saturated — a static scene produces
small frames and long idles, which is exactly the regime where the notch
appears. If it survives to the field it would bite hardest when the picture is
calm, i.e. precisely when an operator is most likely to trust it.

Next: identify the mechanism (code analysis in flight), then confirm by moving
the trip point rather than by removing the symptom.


---

# G-series: the notch mechanism identified — our own host airtime pacer

## The mechanism

`AirtimeBudget.admit()` in `firmware/tractor_x8/image_tx_daemon.py:265` is a
**blocking** gate: it spins in `stop.wait()` until a fragment's estimated
time-on-air fits a **rolling 1.0 s window** (`window_s = 1.0`, `:256`). The DTS
budget is **930_000 us** (`_PROFILE_TO_BUDGET_US[2]`, `:195`).

Per-fragment ToA at SF7/BW500/CR4-5, 255 B on air, is **99_904 us** — which
matches the measured 104.9 ms inter-fragment spacing minus the 5.0 ms dead air
bit-for-bit, so this is arithmetic rather than estimation.

930_000 / 99_904 = **9.309**, so at most 9 fragments fit a freshly-drained
window and the ~10th is refused. This is not RF loss at all: it is the tractor
pacing itself, opening a dead-air hole mid-train.

## Why it explains the idle dependency, which nothing else did

The window is **rolling over 1.0 s**, so it only drains completely when the gap
between trains exceeds one second:

| fps | idle | window at train start | observed |
|---|---:|---|---|
| 2.0 | 0 (saturated) | permanently loaded | smooth gradient, no notch |
| 0.5 | 0.64 s | still loaded | no clean notch |
| 0.4 | 1.14 s | **fully drained** | notch idx 10, ~59% |
| 0.3 | 1.97 s | **fully drained** | notch idx 10, 55% |

**The measured threshold sat between 0.64 s and 1.14 s. The window is 1.000 s.**
That is the fit that makes this mechanism rather than coincidence.

## Causal confirmation by intervention

Run G2, identical to E6 except `LIFETRAC_AIRTIME_BUDGET_US=600000`:

| | idx 10 | total loss |
|---|---:|---:|
| E6, budget 930_000 | **59.1%** | 2.27% |
| G2, budget 600_000 | **7.5%** | 4.02% |

Moving the budget destroyed the notch. That is the confirmation that matters —
the symptom tracks the knob.

## Where the simple model is WRONG, stated plainly

The prediction was `first blocked index = floor(budget/99_904) + depth - 1`,
giving index 7 at a 600_000 budget. **Observed: no notch at 7, a much weaker
peak at index 8 (15.05%).** So:

- The mechanism is confirmed, but the closed-form index formula is **not**. It
  is off by one at the second data point, and the two candidate models
  (`k-1` vs `k-2` events in the deque at admit time) each fit one budget and
  fail the other. Real timing — the rolling expiry, the `stop.wait` clamp of
  250 ms, TX_DONE arrival jitter — is not captured by the static arithmetic.
- The magnitude also collapsed (59% -> 15%). A smaller budget blocks earlier
  and more often, so the throttle spreads across indices instead of
  concentrating. That is consistent with the saturated case showing a gradient
  rather than a notch: same pacer, never a fresh window.
- Total loss **rose** at the lower budget (2.27% -> 4.02%), so the pacer's
  disruption is worse when it bites more often.

## The depth test could not be run

Predicted index 12 at `PIPELINE_DEPTH=4`. The run (G1) is **invalid**: 548
attributed losses against 2.06% actual, ~100% "loss" at indices 4-10, and
`post_loss` gaps at 104.8 ms — identical to normal spacing, where a genuine
post-loss gap is ~210 ms. With four fragments in flight the sequential
classifier cannot track arrivals. **The instrument is only sound at depth <= 2**;
that is now a known limit, not a result.

## What this means

The largest single identified contributor to the loss floor is **self-inflicted
host-side pacing**, not the radio. Three consequences worth acting on:

1. The 930 ms/s budget paces us to 93% duty, and the blocking implementation
   converts that into a *concentrated stall* rather than smooth spacing. Smooth
   pacing (spread the wait across fragments) would keep the same duty without
   the hole.
2. It bites hardest exactly where a real deployment lives — a static scene means
   small frames and long idles, which is the regime that drains the window.
   The saturated bench case is the one regime that hides it.
3. Any future measurement of "RF loss" at sub-saturation must account for this
   first. Several earlier numbers in this file are contaminated by it.

Note on method: an adversarial code-analysis workflow raised 17 candidates and
reported **zero survived**, but three independent agents had converged on this
exact mechanism with exact arithmetic. The refutation instruction ("default to
refuted when uncertain") was over-tuned and the aggregate verdict was wrong.
The candidates themselves were right; reading them individually was what
mattered.


---

# H-series: the pacer fix attempt — a regression, and a correction to my own reading

## What was tried

Replace the token bucket with **smooth pacing**: hold each fragment to the
spacing the duty target already implies (`ToA x window / budget`), so the same
duty is spread evenly instead of bursting to the budget and stalling.

## Result: it made things worse

| run | pacing | config | fragment loss |
|---|---|---|---:|
| E6 | bucket | 0.4 fps, depth 2 | 2.27% |
| H1 | **smooth, no headroom** | identical | **19.41%** |
| H2 | bucket (restored default) | identical | 2.45% |

H1 also produced new 97% notches at indices 1 and 9 — worse and differently
shaped than the single index-10 notch it was meant to remove.

**The spacing arithmetic was correct** — predicted 107.4 ms, measured `seq`
gap 107.6 ms. The defect was that pacing aimed at **100% of the budget**, which
leaves `used == budget` and puts the hard window check on a knife edge. The
backstop then fired on ordinary timing jitter, and its stall **compounded with**
the pacing delay rather than being replaced by it.

Fixed by pacing to `_PACING_HEADROOM = 0.92` of budget so the backstop stays a
backstop. **Default reverted to `bucket`** until a run confirms the fix on air —
a known-good default beats an unverified improvement.

## A correction to my own analysis of H1

I initially reported H1 as also dropping goodput 2000 -> 1204 B/s and util
81% -> 49%. **That comparison was wrong.** Offered load at 0.4 fps x 3000 B is
1200 B/s, and both H1 and H2 measured ~1204 B/s — i.e. exactly the offer. At
0.4 fps the link is **not saturated**, so goodput and util there measure what is
being asked of it, not what it can do. I had compared against E1, which ran at
2.0 fps *saturated*.

The loss regression is real and was measured at identical config. The throughput
regression was an artefact of comparing two different offered loads.

**Rule for this file going forward: never compare goodput or util across runs
with different `SynthFps` unless both are saturated.** Only the saturated runs
measure capacity.

## Also worth recording: the notch magnitude is not stable

Index 10 was 59.1% in E6 and 23.9% in H2 at identical settings. The mechanism
reproduces; the magnitude does not. Any future before/after on this notch needs
n>=2 per side, and a change of less than roughly 2x should not be called an
effect.


---

# I-series: smooth pacing verified, and shipped as the default (2026-07-30)

The headroom fix (`_PACING_HEADROOM = 0.92`) re-tested, n=2 per side at the
0.4 fps / depth-2 / 3000 B operating point, plus a saturated check.

## Low duty — the notch is gone, at no cost

| mode | run | total loss | **idx 10** | attributed vs actual |
|---|---|---:|---:|---|
| bucket | E6 | 2.27% | **59.14%** | 80 vs ~26 (3x over) |
| bucket | H2 | 2.45% | **23.91%** | — |
| **smooth** | I1 | 2.36% | **2.15%** | 27 vs 27 |
| **smooth** | I2 | 2.45% | **2.15%** | 28 vs 28 |

A **19x reduction** in the notch, far outside the >=2x bar this file set after
seeing the magnitude swing 24-59% between identical bucket runs. Total loss is
unchanged (2.36% mean both modes), so the notch was removed rather than
redistributed. Measured spacing 117.0 ms against 116.8 ms predicted.

Two secondary results worth as much as the headline:

- **Smooth is repeatable where bucket was not.** 2.15% twice, against bucket's
  24-59% swing. A mechanism that only sometimes bites is far harder to reason
  about than one that does not bite at all.
- **Attribution now reconciles EXACTLY** (27/27, 28/28), where bucket
  over-attributed ~3x. That is independent evidence the pacer stall was itself
  causing the out-of-order delivery that broke the instrument in the E-series —
  one defect, two symptoms.

## Saturated — cheaper than predicted, and it cuts loss too

| | goodput | util | total loss | late/early |
|---|---:|---:|---:|---:|
| E1 bucket, 2 fps | 2005 B/s | 79% | 3.73% | 2.87x |
| I3 smooth, 2 fps | 1952 B/s | 79% | **2.24%** | 3.45x |

Predicted cost was ~8% goodput, since smooth paces to 92% of budget. **Measured
cost is 2.7%** — because the bucket was never actually achieving its 930 ms/s:
its stalls, clamped at 250 ms, wasted most of the difference. And loss fell 40%
at saturation as well, which was not predicted at all.

## Decision

**Default flipped to `smooth`** in the daemon, the harness, and the tests. A
2.7% goodput cost buys a 19x reduction in a concentrated single-index failure at
low duty, a 40% loss reduction at saturation, and an instrument whose numbers
reconcile. `LIFETRAC_AIRTIME_PACING=bucket` and `-PacingMode bucket` restore the
old behaviour for A/B against everything recorded above.

## Caveat carried forward

`_PACING_HEADROOM = 0.92` was chosen to clear the knife edge, not tuned. The
saturated goodput cost scales roughly with it, so 0.95-0.98 may recover most of
the 2.7% while still keeping the backstop from firing. That is a cheap follow-up
sweep and it should be run before anyone treats 0.92 as considered.


---

# J-series: the headroom sweep — 0.92 stands, and goodput is the wrong metric

`_PACING_HEADROOM` was flagged as chosen-not-tuned, with the expectation that
raising it would recover most of the 2.7% saturated goodput cost. It does. It
also delivers FEWER usable frames.

Saturated (2 fps, 3000 B, depth 2, smooth):

| headroom | spacing | goodput | fragment loss | **frames published** | reasm timeouts |
|---:|---:|---:|---:|---:|---:|
| 0.92 | 116.8 ms | 1951.6 B/s | **2.24%** | **105** | 42 |
| 0.96 | 112.1 ms | 1984.8 B/s | 3.17% | 97 | 51 |
| bucket (ref) | 104.9 ms | 2005 B/s | 3.73% | — | — |

Raising headroom to 0.96 recovered 1.7% of TX goodput and cost 0.93 points of
fragment loss — and **8% of delivered frames** (105 -> 97), with 21% more
reassembly timeouts.

## The trap worth naming

**TX goodput is the wrong optimisation target for this link.** A fragment lost
mid-frame destroys the whole frame, so a small rise in fragment loss costs far
more delivered frames than the extra bytes buy. The three headroom points sit on
a monotonic curve where goodput and delivered-frames move in OPPOSITE directions:

    headroom -> 1.0   =  more bytes transmitted, fewer pictures arriving

That is also the shape that makes the pre-fix bucket look defensible on a
dashboard — 2005 B/s is the best goodput number in this entire file, and it was
produced by the configuration with the worst loss.

**Standing rule for this project: rank pacing/scheduling changes by
`frames_published`, not by `goodput`.** Goodput measures what we put on the air;
frames_published measures what the operator can actually see.

## Decision

**Keep `_PACING_HEADROOM = 0.92`.** It was chosen for the right reason (clear
the knife edge) and it turns out to also be the better operating point on the
metric that matters. 0.99 was not run — the 0.92 -> 0.96 trend already moves the
wrong way on delivered frames, and going closer to 1.0 approaches the bucket
pathology the headroom exists to prevent.

`LIFETRAC_PACING_HEADROOM` and `-PacingHeadroom` remain available for future
sweeps, e.g. if fragment size or profile changes move the trade.


---

# L-series: RS-11.2 gap sweep — the trade dissolved, and delivery hit the ceiling (2026-07-30)

Six runs, 240 s each, saturated (2 fps / 3000 B), smooth pacing, `ReactiveFire 1`
(base fires a probe at every frame completion), `ProbeEcho 0`, delivery scored
tractor-side per the standing rule. `TrainGapMs` in {15, 40, 80}, n=2 per point.

| gap ms | probes TX | probes RX | delivery | goodput | loss% | frames published |
|---:|---:|---:|---:|---:|---:|---:|
| 15 | 100 | 99 | 99.0% | 1979 B/s | 2.52 | 98 |
| 15 | 105 | 105 | 100% | 2001 B/s | 2.57 | 101 |
| 40 | 106 | 106 | 100% | 2001 B/s | 2.36 | 105 |
| 40 | 106 | 106 | 100% | 1977 B/s | 2.31 | 105 |
| 80 | 108 | 108 | 100% | 1977 B/s | 2.48 | 104 |
| 80 | 104 | 104 | 100% | 1977 B/s | 2.69 | 101 |

## Finding 1 — the RS-3.10 gap trade no longer exists

The queued A/B assumed a trade: shrink the gap, gain ~7-8% goodput, lose command
window. Measured under smooth pacing there is NO trade on either axis: goodput
is flat (1977-2001 B/s across 15-80 ms) and delivery is at ceiling everywhere.

The reason is structural. Under smooth pacing the transmitter is
**pacing-limited, not gap-limited**: train duration is set by the 116.8 ms
fragment spacing, and the designed gap is a small constant added at the
boundary. Shrinking it from 40 to 15 ms saves ~25 ms per ~1.6 s train (~1.5%),
which disappears into run-to-run noise. The 2026-07-26 estimate of +7-8% was
made against the bucket pacer, whose accidental ~44 ms host gap the designed gap
replaced 1:1; smooth pacing changed the denominator.

**Decision (RS-11.2 closed): keep `TRAIN_GAP_MS = 40`.** No axis rewards
changing it, 40 matches every run in the evidence record, and a larger designed
window is free insurance for the aligned command pump.

## Finding 2 — the headline: single-copy command delivery is now ~99.8%

Pooled across all six runs: **628 of 629 probes delivered (99.84%; 95% CI
lower bound ≈ 99.1%)**. Under the token bucket, the identical measurement on
2026-07-29 gave **91.1%** [85.8, 96.4]. Smooth pacing did not just fix the
index-10 notch — it took the in-stream command channel from "usable with two
copies" to "effectively lossless at one copy".

The mechanism is geometric. The bucket's inter-fragment dead air was 5.0 ms —
narrower than a command frame's 10.3 ms ToA, so mid-train arrivals were
impossible and everything rode the train boundary. Smooth pacing's spacing is
116.8 ms against a 99.9 ms fragment, leaving **~17 ms of listening air between
every fragment — wider than the 10.3 ms command ToA.** Every inter-fragment gap
is now a viable command slot, not just the boundary. (Probes here still fire at
frame completion; the widened gaps also stop the completion window being
clipped by URC-queue churn, which the E-series attribution work showed was
reordering deliveries under the bucket.)

At p = 0.998 single-copy, two copies give ~99.9997%. For the control plane this
is an architecture-level result:

> **The opportunistic (no-TDMA) control plane now meets a harder bar than the
> firmware slot design was invented to reach.** Firmware Batch 2 (F1 DTS slot
> clock, F2 mute gate, F3 skip/ditto contraction) should be re-evaluated as an
> *optimisation with a measured ceiling of +0.2 points*, not as a
> prerequisite for control. Batch 1 (F6-F9) is unaffected — those are latent
> correctness bugs regardless of architecture.

Caveats that keep this honest: bench-range link margin, ~0.44 probes/s (not
full command cadence — the cadence confirmation in RS-1 still stands), one
bench day, and probes fire gap-aligned rather than at random phase. The 91% ->
99.8% comparison holds all of those constant across the two pacers, so the
DELTA is solid even where the absolute number needs field confirmation.
