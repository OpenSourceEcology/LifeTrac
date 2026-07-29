# RS-0.12 / RS-0.13b — bench results, 2026-07-29

**Boards:** tractor 2E2C1209DABC240B, base 2D0A1209DABC240B. DTS (profile 2,
BW500). Code at `e43d07cf` (branch `control-first-tdma-plan`, PR #86).
**Instrumentation:** `CMD_OP_PROBE`/`PROBE_ECHO` reactive-fire. The
authoritative delivery number throughout is the **tractor-side**
`LoRa cmd: PROBE` count — nothing else can satisfy it, which is why it was
built that way after the 171/1 retraction.

---

## 1. Headline results

Full 2×2 (pipeline × frame size), with 95% CIs:

| # | Pipeline | Frame | Probe delivery (tractor-side) | 95% CI | Goodput |
|---|---|---|---|---|---|
| B1 | **v2** | 3000 B | **0%** (0/86) | [0, 0] | 1886 B/s @ 77% |
| B3 | **v2** | 250 B | **15%** (29/196) | [9.8, 19.8] | 1722 B/s @ 72% |
| B2 | **v3** | 250 B | **53%** (132/248) | [47.0, 59.4] | 1754 B/s @ 74% |
| **B4** | **v3** | **3000 B** | **57%** (35/61) | [45.0, 69.8] | **2005 B/s @ 79%** |
| Sweep | v3 | 250 B | 17% aggregate over 14 phase×size bins | — | 1803 B/s @ 75% |

**PIPELINE IS THE WHOLE STORY. Frame size does not matter on v3.**
B4's and B2's confidence intervals overlap heavily (45–70 vs 47–59) — they are
statistically indistinguishable — while both sit far outside v2's. The
frame-size effect seen on v2 (0% → 15%) was small frames *partially
compensating* for v2's broken RX arming, not a property of the link. On v3 it
buys nothing, **and large frames deliver 14% more goodput** (2005 vs
1754 B/s), close to the project record 2046 B/s while simultaneously carrying
57% command delivery.

**Design consequence: the "shorten trains for more boundaries" lever is
UNNECESSARY.** Keep 3000 B frames. (The *separate* loss-survival argument for
single-fragment frames — 95.7% vs 56.5% at 4.3% fragment loss — is untouched
by this and still stands on its own merits.)

**Measured RTT (base→tractor→base):** median 245–281 ms, p95 367–503 ms.
First single-clock round-trip figure the project has under load.

---

## 2. RS-0.13b bisection — what broke Run J's alignment

**The prior hypothesis was WRONG and is retracted.** `CONTROL_PLANE_DESIGN.md`
and TODO RS-0.14 both said the RS-3.10 change set (`parity_group`,
`TX_PREPARE_AHEAD`, `train_gap_ms`) destroyed Run J's 58%. It did not: those
three knobs were **off in both B2 (53%) and B3 (15%)**, which differ only in
pipeline. Two causes, both isolated:

- **Pipeline v3 vs v2 — this is the entire effect.** +57 points at 3000 B
  (0% → 57%) and +38 points at 250 B (15% → 53%). v3 is the path with
  mid-burst RX dispatch and an end-of-train RXCONT re-arm. Run J used v3;
  every run since defaulted to v2 **because that is the harness default**
  (`run_live_radio_monitor.ps1:24`, `[string]$TxPipeline = "v2"`) and no run
  passed `-TxPipeline`. The regression was a *test-harness default*, not a
  code change — which is why reading the diffs never found it. `params.txt`
  did record it, which is how it was eventually caught: evidence discipline
  paid for itself.
- **Frame size — NO effect on v3** (57% @ 3000 B vs 53% @ 250 B, CIs overlap).
  It appeared to matter on v2 only (0% → 15%) because small frames partially
  compensate for v2's broken RX arming. *Initially reported as "+15 points"
  before B4 completed the 2×2; corrected here.*

Delivery spans 0→57% while goodput spans 1722–2005 B/s, and the **best
delivery coincides with the best goodput** (v3 + 3000 B).

---

## 3. RS-0.12 phase × size sweep (872 probes, 14 bins, ~61 each)

| Phase offset | 23 B (ditto) | 38 B (control) |
|---|---|---|
| 0 ms | 30% | 34% |
| 20 ms | 31% | 26% |
| 40 ms | 16% | 19% |
| 60 ms | 19% | 15% |
| 80 ms | 2% | 8% |
| 100 ms | **0%** | 3% |
| 120 ms | 8% | 5% |

**(a) The armed window is real, and it is at frame completion.** Delivery
decays monotonically from ~32% at offset 0 to ~2% at 100 ms, with a small
uptick at 120 ms (the next boundary). This is the first direct measurement of
the window's *shape*: it is centred on the completion instant and is roughly
60–80 ms wide, not a narrow spike. Firing later is strictly worse — so the
reactive-fire-at-completion strategy is correct, and no additional delay
helps.

**(b) Sweep aggregate (17%) is LOWER than the single-phase run (53%), and the
single-phase number is the honest one for a given phase.** The sweep
interleaves deliberately-bad phases; those probes collide with the tractor's
transmissions and perturb the link for everyone. Use 53% as the achievable
opportunistic ceiling, not 32%.

**(c) SIZE DOES NOT MATTER — the ditto question is answered.**
23 B: 65/430 = **15.1%**. 38 B: 68/431 = **15.8%**. Delta **+0.7 points**,
well inside noise, and the sign even favours the *larger* frame.

Per the pre-registered decision table (`CONTROL_PLANE_DESIGN.md` §8):
> *s23 ≈ s38 → size is not the constraint → ditto is an airtime nicety;
> prefer GCM-128 (handheld parity).*

**Both consequences now settled by measurement:**
- **Ditto is worth only its airtime saving** (~5.12 ms/slot ≈ 3.5% of image
  goodput), not reliability. Keep it — it is free on the wire and the
  anti-desync contract is already built and tested — but it is not
  load-bearing and must not be a prerequisite for anything.
- **Envelope decision: GCM-128.** Since frame size buys no delivery, the
  +12-vs-+28 B question reverts to compatibility, and GCM-128 is byte-for-byte
  the shape `tractor_h7.ino:1106` already decrypts — no `boot_ctr` blocker, no
  three-tree handheld cutover. **RS-8.6's D13 preference is superseded.**

---

## 4. What this means for the architecture

**Opportunistic delivery caps around 53%. That is not enough for a control
plane, and that is precisely the argument for the firmware slot work.**

Retry math at p=0.53: 2 copies → 78%, **3 copies → 90%**, 4 → 95%, 5 → 98%.
So a usable control plane today would need 3–5 copies per command, at ≥0.5 s
spacing — i.e. 1.5–2.5 s to land one command with confidence. Two orders off a
hydraulic loop, exactly as predicted.

The TDMA design (firmware F1–F3) exists to convert this *opportunistic* window
into a *scheduled* one. The measurement supports the design in the strongest
way available: **the window physically exists, is ~60–80 ms wide, sits at a
predictable instant, and the base can already hit it half the time by guessing.
Making the tractor mute deterministically at that instant is what turns 53%
into ~100%.** Nothing here suggests the schedule is unbuildable; it says
guessing is not sufficient.

**Revised guard-budget guidance for F1–F3:** the window's ~60–80 ms usable
width is far larger than the 8–10 ms turnaround guard the design assumed, so
guard sizing is *not* the risk it was thought to be. The risk is phase
acquisition, not guard width.

---

## 5. Corrections and defects found today

1. **Base image was missing entirely.** The base board had no
   `lifetrac-v25:latest` (only mosquitto) — its image store was wiped when it
   rebooted after the 2026-07-26 layer-store purge. First B1 attempt therefore
   ran TX-only with nobody listening. Rebuilt from the deployed Dockerfile;
   clean build, no corruption recurrence.
2. **Probe echo double-count (my instrumentation bug), fixed mid-session.**
   Base reported **105.8% delivery** — impossible. The tractor's queued echo is
   radiated by `_send_command_frame` with its default `copies=2`, so each probe
   produced two echo arrivals and the base counted both. The per-(phase,size)
   grid was already pop-keyed and therefore correct; only the headline was
   inflated ~2×. Now counts unique sequences.
3. **`TxPipeline` default is a measurement hazard.** A harness default silently
   changed the code path under test across ~30 runs. `params.txt` did record
   it, which is how it was caught — evidence discipline paid for itself.

---

## 6. Not tested today, and why

**Encode-to-fit could not be verified by this harness.** The bench feeds
pre-built tile-delta frames from `publish_synthetic_frames.py` straight to
`image_tx_daemon`; `camera_service` — which owns the budget packer, the carry
fix, the age-escalation and the liveness valve — is **not in the loop at all**.
Verifying it needs `camera_service` running against the USB camera (the RS-3.3
real-camera item). The unit tests cover the logic; the on-air check is
outstanding and must not be reported as done.
