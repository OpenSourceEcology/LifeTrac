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
