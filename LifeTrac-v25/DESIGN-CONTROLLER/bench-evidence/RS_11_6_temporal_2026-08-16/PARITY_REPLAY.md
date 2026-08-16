# RS-4.1 XOR parity, replayed against the measured loss process

**Desk analysis. No bench time, no radios.**
Tool: [`tools/interferer_replay.py`](../../tools/interferer_replay.py).
Transcript: [`parity_replay_output.txt`](parity_replay_output.txt).

This drives the **real shipping code** rather than a model of it —
`lora_proto.add_parity_fragments` on the TX side and
`image_pipeline.reassemble.FragmentReassembler` on the RX side — against the
loss process measured in [`RESULTS.md`](RESULTS.md), and answers whether
RS-4.1 is worth building.

## 1. Why arrival statistics decide this, not loss rate

XOR parity repairs **exactly one loss per group**. Two losses inside one group
is a lost frame no matter how much parity was paid for. So the question is not
how much loss there is, it is **whether losses arrive alone**.

That made the loss model the whole experiment, and the first version of it was
wrong in a way worth recording. Modelling loss as independent per fragment
(IID) at 5.9 % over a 13-fragment train predicts **55 % of trains losing at
least one fragment**. RS-11.4 measured **23–34 % of trains timing out** at that
same loss rate, and described the mechanism as a per-boundary *event* of
1.3–1.8 fragments. Both say losses **cluster**: fewer trains are hit, and the
ones that are hit lose several fragments each.

Calibrating a clustered model to that measurement (events per train
`λ = −ln(1−A)`, fragments per event `L/λ`) reproduces it — the simulation
reports **33.7 % of trains affected**, inside the measured 23–34 % band. The
IID foil is retained in the tool output purely to show how much it flatters
parity.

## 2. The result, and it reverses the obvious design choice

24 trials × 250 frames, mean ± sd across trials:

| parity group | frags/train | airtime | delivery | trains hit | vs off |
|---|---:|---:|---:|---:|---:|
| off | 13 | 1.00× | 67.6 % ± 2.0 | 32.5 % | — |
| G=4 | 17 | **+31 %** | 81.3 % ± 1.4 | 39.9 % | +13.6 pts |
| G=8 | 15 | **+15 %** | 82.2 % ± 1.4 | 36.2 % | +14.6 pts |
| **G=13** | 14 | **+8 %** | **82.4 % ± 1.7** | 34.0 % | **+14.7 pts** |

**Group size does not help under clustered loss, and the coarsest setting is
the best one.** G=13 matches or slightly beats G=4 for **a quarter of the
airtime**. The three are within ~1 point of each other against a per-trial sd
of ~1.5, so the honest claim is that G=13 is *no worse* and costs far less —
not that it is dramatically better.

Under the IID foil the picture inverts completely — G=4 wins outright
(+41.3 pts vs +33.2 for G=13) and finer groups are clearly better. That is the
trap: **an IID analysis would have specified G=4 and paid 31 % airtime for
nothing.**

Two mechanisms explain it, and the second is why finer groups actively lose
ground:

1. When a burst takes 2+ fragments they land in the **same group whatever the
   group size**, so a fine-grained scheme's extra parity fragments cannot
   repair them.
2. **Every parity fragment is itself exposed to loss and adds airtime.** The
   `trains hit` column shows this directly: G=4 lengthens the train to 17
   fragments and pushes trains-with-any-loss from 32.5 % to 39.9 %, while
   G=13 only reaches 34.0 %. Fine-grained parity buys redundancy it cannot
   use and pays for it in exposure.

**Recommended configuration if RS-4.1 is built: `LIFETRAC_PARITY_GROUP=13`**
(one parity fragment per train), not the 8 that `add_parity_fragments` takes
as its default.

## 3. The interferer is the part parity fixes well

Sweeping the interferer's share of total loss:

| interferer share | off | G=4 | G=8 | G=13 |
|---|---:|---:|---:|---:|
| 0 % | 72.2 % | 79.5 % | 80.7 % | 81.3 % |
| **13 % (measured)** | 67.6 % | 80.9 % | 81.8 % | 82.0 % |
| 30 % | 61.5 % | 82.8 % | 83.0 % | 83.3 % |
| 60 % | 67.9 % | 89.2 % | 89.4 % | 89.4 % |
| 100 % | 77.0 % | 98.3 % | 98.2 % | **98.4 %** |

Parity gets *more* effective as the interferer's share rises, reaching ~98 %
delivery if the interferer were the only loss source. That is the direct
answer to the aliasing worry raised in `RESULTS.md` §9:

**There is no aliasing.** At 7.085 s against a ~1.85 s train cadence the ratio
is 3.83 — hits land in roughly one train in four, at a drifting position, and
therefore **arrive alone**, which is precisely the case XOR parity handles.
The losses parity *cannot* fix are the clustered bulk ones.

So the two mitigations target different halves of the problem, and neither
addresses the other's half.

## 4. Corrections to the record

- **CR 4/8 costs +57 % airtime, not the "~37 %" this campaign has been
  quoting.** Computed from the same time-on-air function used elsewhere:
  SF7/BW500, 255 B, CR4/5 = 99.9 ms → CR4/8 = 156.7 ms. The 37 % figure
  understates it by half again, which matters because the airtime is the
  entire cost side of that decision.
- **The 700 ms budget does not apply to image frames.** A 13-fragment image
  train is already 1521 ms. That budget is base→tractor hydraulic control
  latency. What parity and CR4/8 actually spend is *airtime*, which competes
  with the command slots control latency depends on — so the cost lands on
  **control margin**, not image latency. Any framing of these mitigations as
  "fits/doesn't fit the 700 ms budget" is a category error.

## 5. A defect this surfaced in the RX path

`FragmentReassembler._try_parity_reconstruct` **refuses to reconstruct the
last fragment of a frame** (`if missing_idx == partial.total - 1: continue`),
because XOR padding would append trailing bytes the frame parser rejects. The
exclusion is deliberate and documented in-code.

It is worth flagging against RS-11.4, which localised the dominant loss to the
**second-to-last** fragment. Second-to-last *is* reconstructable, so the
exclusion does not block the main observed failure mode — but it does mean
parity's ceiling is structurally below 100 %, and a train whose final fragment
is lost still falls back to GC-timeout plus a keyframe request.

## 6. Caveats

- **This is a simulation with a calibrated model, not a measurement.** It
  drives the real TX/RX code, so the parity *mechanics* are exercised for
  real, but the loss *arrival* process is synthetic.
- **The burst-length distribution is a choice.** Exponential with the
  measured mean reproduces the affected-train fraction, but the real
  distribution is unmeasured; a heavier tail would further reduce parity's
  value, a lighter one would raise it.
- **A bug in the first version of this simulation is recorded here rather
  than quietly fixed.** Burst placement originally clamped an over-running
  burst with `min(start + k, frags_per_train - 1)`, which piled the overflow
  onto the FINAL fragment index — exactly the index
  `_try_parity_reconstruct` refuses to rebuild (§5). That biased every parity
  configuration downward, worst for coarse groups. Bursts are now placed so
  they fit inside the train. The corrected numbers moved G=13 from 81.3 % to
  82.4 % and changed the ranking of G=13 against G=4 from "equal" to
  "equal-or-better", which is why the §2 conclusion is stated the way it is.
- **The interferer share (13 %) is the least certain input** — corrupt
  *captures* are not the same population as total losses, since a fragment can
  be lost without producing a `crc_dump` at all. Hence the sweep in §3, which
  shows the G-ranking is stable across the whole range even though the
  absolute numbers move.
- Delivery here means *all data fragments present*, not decoded image quality.

## 7. Recommendation

1. **Do not build RS-4.1 yet.** ~15 points of delivery for 8 % airtime is a
   real but modest return, and it is a return on *tolerating* the emitter.
   The idle capture that could remove the emitter outright is one 300 s leg
   away and costs nothing to try first.
2. **If it is built, use G=13**, and record why: under clustered loss finer
   groups buy nothing and cost 4× the airtime.
3. **CR 4/8 is the weaker of the two options** on cost alone — +57 % airtime
   against parity's +8 % — and its benefit is still unsized because the
   damaged-symbol count is unmeasured (see `RESULTS.md` §7). If the TX-side
   reference logging lands, size it then; otherwise parity dominates it.
