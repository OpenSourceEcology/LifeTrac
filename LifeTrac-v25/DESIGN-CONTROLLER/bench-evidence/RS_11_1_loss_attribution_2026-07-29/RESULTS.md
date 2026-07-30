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
