# RS-0.12 / RS-0.13b — bench results, 2026-07-29

**Boards:** tractor 2E2C1209DABC240B, base 2D0A1209DABC240B. DTS (profile 2,
BW500). Code at `e43d07cf` (branch `control-first-tdma-plan`, PR #86).
**Instrumentation:** `CMD_OP_PROBE`/`PROBE_ECHO` reactive-fire. The
authoritative delivery number throughout is the **tractor-side**
`LoRa cmd: PROBE` count — nothing else can satisfy it, which is why it was
built that way after the 171/1 retraction.

---

## 0. THE HEADLINE — the ACK was destroying the command channel

Suppressing the tractor's reply raises **forward command delivery from 56% to
91%**. Same code, same profile, same frames — the *only* difference is whether
the tractor echoes.

| Config (v3 + 3000 B, all else identical) | Forward delivery | 95% CI | Image goodput |
|---|---|---|---|
| echo ON, 2 copies (B5) | 55.6% (99/178) | [48.3, 62.9] | 1999 B/s |
| echo ON, 1 copy (A1) | 59.3% (54/91) | [49.2, 69.4] | 2014 B/s |
| **echo OFF (A2)** | **91.1% (102/112)** | **[85.8, 96.4]** | **2019 B/s** |

CIs do not overlap. **The reply transmission occupies and deafens exactly the
reverse-slot window the next command needs** — `_service_control_plane` flushes
queued replies at the top of the TX-worker loop, immediately before the next
train, which is precisely where the base is aiming.

**Two consequences, both large:**

1. **Every "delivery" number measured earlier today is depressed by our own
   measurement apparatus.** The 53–57% figures — and the whole phase sweep —
   ran with echoes on. True opportunistic forward delivery is ~91%.
2. **The architecture verdict changes.** At p = 0.911: one copy 91.1%, **two
   copies 99.2%**, three 99.9%. A usable one-way control plane may be
   achievable *opportunistically, today, with no firmware slot at all* —
   the firmware TDMA work becomes an optimization and a determinism
   guarantee rather than a precondition. (Caveat: measured with a small probe
   at ~0.5/s on a clean short-range bench link. Higher command rates mean more
   base TX, which may self-interfere; needs confirming at cadence.)

This is also the strongest possible evidence for the standing "no ack on
actuation" rule: acking does not merely cost airtime, **it cuts command
delivery by 35 points.**

## 1. Headline results

Full 2×2 (pipeline × frame size), with 95% CIs:

| # | Pipeline | Frame | Probe delivery (tractor-side) | 95% CI | Goodput |
|---|---|---|---|---|---|
| B1 | **v2** | 3000 B | **0%** (0/86) | [0, 0] | 1886 B/s @ 77% |
| B3 | **v2** | 250 B | **15%** (29/196) | [9.8, 19.8] | 1722 B/s @ 72% |
| B2 | **v3** | 250 B | **53%** (132/248) | [47.0, 59.4] | 1754 B/s @ 74% |
| **B4** | **v3** | **3000 B** | **57%** (35/61) | [45.0, 69.8] | **2005 B/s @ 79%** |
| **B5** | **v3** | **3000 B** | **56%** (99/178) | [48.3, 62.9] | **1999 B/s @ 81%** |
| Sweep | v3 | 250 B | 17% aggregate over 14 phase×size bins | — | 1803 B/s @ 75% |

**B5 (7 min) confirms B4.** Pooled v3 + 3000 B: **134/239 = 56.1%,
95% CI [49.8, 62.4]** at ~2000 B/s. This is the recommended operating point
and it now has the matrix's largest sample behind it.

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

**Measured RTT (base→tractor→base):** 245–281 ms median with 250 B frames,
but **1482 ms median with 3000 B frames.** First single-clock round-trip
figures the project has under load.

**The RTT difference is a finding, not noise: echo latency is dominated by
the tractor's TRAIN LENGTH, not by the air.** The tractor queues its echo on
`_cmd_out` and only flushes it in `_service_control_plane` at the top of the
next TX-worker iteration — so a reply waits out the train in progress
(~1.7 s for a 17-fragment 3000 B train, ~0.2 s for a 250 B one). Consequences:
- **One-way commands (drive, E-stop) are unaffected** — arrival is fast; the
  probe's *forward* delivery is what 56% measures.
- **Anything needing a round trip pays a full train period.** Ack-driven
  convergence (RS-1.5), the two-phase profile switch, and any request/response
  command inherit this. It is a strong argument for keeping actuation
  one-way + deadman rather than ack-gated, and for the reverse-slot design
  where the tractor can answer without waiting out its own train.

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

> **SUPERSEDED BY §0.** The paragraph below was written before the ack-cost
> experiment and reflects delivery measured *with the echo enabled* — i.e.
> depressed by our own instrument. Corrected verdict: opportunistic forward
> delivery is **~91%**, and **2 copies gives 99.2%**, so a one-way control
> plane looks achievable without the firmware slot. The slot becomes an
> optimization and a determinism guarantee, not a precondition. Retained
> below for the record.

~~**Opportunistic delivery caps around 53%. That is not enough for a control
plane, and that is precisely the argument for the firmware slot work.**
Retry math at p=0.53: 2 copies → 78%, 3 copies → 90%, 4 → 95%, 5 → 98%. So a
usable control plane today would need 3–5 copies per command, at ≥0.5 s
spacing — i.e. 1.5–2.5 s to land one command with confidence.~~

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

## 4a. ACK vs NACK — measured answers (2026-07-29)

**Q: does an ack waste airtime?** Two costs, and the indirect one dominates.

*Direct* — negligible at these rates, ruinous at control cadence. One echo copy
is 10.304 ms at BW500. At the ~0.4 acks/s these probes fired, that is 0.8% of
goodput (measured: 1999 vs 2019 B/s, inside noise). Scaled:

| Ack rate | ×1 copy | ×2 copies |
|---|---|---|
| 0.4/s (probes) | 0.4% wall | 0.8% wall |
| 5 Hz | 5.2% (~127 B/s) | 10.3% (~255 B/s) |
| 10 Hz | 10.3% (~255 B/s) | **20.6% (~509 B/s)** |

*Indirect* — **35 points of forward command delivery** (§0). This dwarfs the
airtime cost and is the real reason not to ack.

**Q: is the round trip reliable?** No. Base-side echo receipt is 46.1% with 2
copies, 22.2% with 1 (CIs disjoint). Backing out `P(echo home | probe
arrived)`: **83% with 2 copies, 37% with 1**. So the return path is *better*
than the forward path, yet the **combined round trip completes under half the
time**. An ack-gated retry loop therefore retransmits commands that already
landed more often than not — spending the *expensive* direction to compensate
for losses in the *cheap* one.

**Q: use NACK instead?** No, for three independent reasons:
1. **Structural.** A NACK requires knowing something *should* have arrived.
   Commands are aperiodic; a wholly-lost command leaves no gap to detect, and
   silence is indistinguishable from "nothing to say". NACK only becomes
   expressible once every slot carries something (the scheduled design).
2. **It fails unsafe.** The tractor→base path is ~83% reliable, so a NACK goes
   missing ~17% of the time — and **a lost NACK reads as success**. A lost ACK
   merely causes a redundant retry.
3. **It does not dodge the real cost.** A NACK is still a tractor→base TX in
   the reverse-slot window, so it inflicts the same −35-point wound as an ACK.

**Recommended policy (all measured, not argued):**
- **Actuation (drive, E-stop): no reply of any kind.** One-way + deadman.
  Refresh, don't confirm.
- **Configuration (encode mode, radio profile): prefer the IMPLICIT ack** —
  the frame header is self-describing, so a changed codec byte already proves
  the command landed at zero airtime and zero window damage (partly built:
  `rx_codec` in link_stats). Where an explicit ack is unavoidable, keep 2
  copies (37% → 83% return for ~10 ms) but send it **only** when a command is
  outstanding, never periodically.
- **Long term: piggyback.** Reverse-direction data rides the image stream,
  which is already running ~80% of the time. A few ack bits in the fragment or
  hop header cost nothing and — critically — add no extra TX event, so they
  avoid the window damage entirely. Needs the schema bump already planned for
  the MIC.

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

---

## 7. Coding Rate 4/5 — what it is, and why we are keeping it

### 7.1 What the setting means

LoRa's coding rate is a forward-error-correction (FEC) ratio written into
`SX1276_REG_MODEM_CONFIG1` bits 3:1 by `sx1276_set_sf_bw_cr()`
(`firmware/murata_l072/radio/sx1276.c:379`, register write at `:450`). It is
already a per-profile field — `PhyProfile.cr_den` (`base_station/lora_proto.py:142`)
— and it is already honoured by the airtime model (`cr = cr_den - 4`, `:520`).
So this is a knob we *can* turn per profile; we simply have never swept it.

`4/5` means every 4 data bits carry 1 parity bit: 1.25x expansion. In LoRa's
Hamming-style code, 4/5 and 4/6 give **detection only** — they can tell a
codeword is wrong but cannot repair it. Only 4/7 and 4/8 add enough redundancy
to **correct** bit errors, and they cost 1.75x and 2.0x the payload symbols.

Two facts about our PHY configuration matter for the decision, both confirmed
in firmware this session:

- **Explicit header mode** — `MODEM_CONFIG1` bit 0 is written `0`
  (`sx1276.c:450`). In explicit mode the LoRa *header* is always transmitted
  at CR 4/8 regardless of the payload CR. The most fragile part of the packet
  is therefore already maximally protected, and raising the payload CR does
  nothing for it.
- **Payload CRC is enabled** — `MODEM_CONFIG2` bit 2 is set (`sx1276.c:452`).
  A packet whose header locks but whose payload is corrupted is *reported*, as
  a CRC error, not silently dropped. That is what makes the next section
  conclusive rather than merely suggestive.

### 7.2 The measurement that decides it

Across all ten runs today — roughly 19,000 received frames — the counters read:

```
rx_decode_err = 0        reassembler_decode_err = 0
```

Zero. Not "low": zero, in every run, without exception.

Given explicit-header + payload-CRC, that result partitions our losses cleanly:

- A packet that arrives with a corrupted payload raises a CRC error. We have
  **none**, so no arriving packet is corrupted.
- Therefore every lost fragment failed *before* header lock — the receiver
  either was not listening, or never detected the preamble.

FEC repairs the first class. We have zero of the first class. **Raising the
coding rate would buy us nothing on this link.**

### 7.3 The simplex floor — runs C1 and C2

To find out what the residual loss actually is, two 240 s control runs stripped
the base's transmitter out of the picture (`4d33b499`, DTS profile 2, v3
pipeline, 3000 B synthetic frames):

Each condition was run **twice** so that differences could be read against the
observed run-to-run spread rather than asserted from a single sample:

| Run | keyframe reqs | base TX | frags TX | frags RX | loss | decode err | frames published | reasm timeouts |
|-----|---------------|--------:|---------:|---------:|-----:|-----------:|-----------------:|---------------:|
| C1  | on  | 86 | 1914 | 1815 | 5.17% | 0 | 81  | 70 |
| C1b | on  | 93 | 1917 | 1808 | 5.69% | 0 | 82  | 69 |
| C2  | off | 17 | 1914 | 1841 | 3.81% | 0 | 92  | 59 |
| C2b | off | 17 | 1915 | 1852 | 3.29% | 0 | 103 | 47 |

| Condition | loss (mean) | published (mean) | reasm timeouts (mean) |
|-----------|------------:|-----------------:|----------------------:|
| keyframe requests **on**  | **5.43%** | 81.5 | 69.5 |
| keyframe requests **off** | **3.55%** | 97.5 | 53.0 |
| delta | **+1.88 pts** | **−16.0 frames (−16%)** | **+16.5 (+31%)** |

Within-condition spread is 0.52 points in both conditions, so the 1.88-point
separation is roughly 3.6x the noise. Readings, all with zero decode errors:

1. **There is a ~3.5% one-way fragment loss floor with the base radio silent**
   (3.29% / 3.81%, n=2). Nothing the base does causes it, and it is not
   corruption — it is the receiver missing packets it was not ready for. This
   is the number that motivates the preamble work (F4) and rules out FEC.
2. **The self-heal keyframe request is net harmful, and the effect replicates.**
   It costs 1.88 points of fragment loss, delivers 16% *fewer* complete frames,
   and produces 31% *more* reassembly timeouts — i.e. it measurably increases
   the very condition it fires on. See §8 for why and what should replace it.
3. **Scale check on the mechanism.** The ~76 extra base transmissions cost ~36
   extra lost fragments — 0.47 fragments per base TX. Each base transmission
   deafens the base for its 10.3 ms ToA plus the host round trip (~40 ms
   total) against a ~100 ms fragment cadence, which predicts ~0.4. Measurement
   and geometry agree, so the mechanism is understood, not merely observed.

**Retracted:** an earlier draft of this section compared run A2 (2.31%, with
112 gap-aligned probes) against a single C2 and concluded that *timing* of base
TX dominates *volume*. With n=2 now in hand, A2 sits 1.2 points below the C2
mean — outside the observed spread and currently **unexplained**. The
timing-beats-volume claim may well be true (it is what the slot design assumes)
but it is not established by this data and has been withdrawn pending a
replicated probes-on/probes-off A/B.

### 7.4 Cost of the alternatives, measured against our actual failure mode

For a 255 B fragment at SF7/BW500 (`lora_time_on_air_ms`):

| Knob | Airtime | Delta | Repairs |
|------|--------:|------:|---------|
| CR 4/5, preamble 8 (current) | 99.904 ms | — | baseline |
| CR 4/6 | 118.848 ms | +19.0% | payload bit errors (we have 0) |
| CR 4/7 | 137.792 ms | +37.9% | payload bit errors (we have 0) |
| CR 4/8 | 156.736 ms | +56.9% | payload bit errors (we have 0) |
| preamble 12 | 100.928 ms | **+1.0%** | receiver re-arm window (+1.02 ms) |
| preamble 16 | 101.952 ms | **+2.0%** | receiver re-arm window (+2.05 ms) |
| preamble 24 | 104.000 ms | +4.1% | receiver re-arm window (+4.10 ms) |
| preamble 32 | 106.048 ms | +6.1% | receiver re-arm window (+6.14 ms) |

**Preamble length, not coding rate, is the knob that addresses our failure
mode, and it is roughly twenty times cheaper.** A receiver that is re-arming
RXCONT between fragments can only catch a packet if it starts listening during
the preamble; lengthening the preamble widens that catch window directly. FEC
does not widen it at all.

Preamble length is currently **not settable** — the firmware never writes
`REG_PREAMBLE_MSB/LSB` (0x20/0x21); it only reads them back in
`sx1276_airtime.c:171`. The chip default of 8 symbols is what we have been
running. Making it per-profile settable is roadmap item **F4**, which this
measurement now justifies quantitatively rather than by intuition.

### 7.5 Coding rate on the control plane — checked, also no

Small frames were worth checking separately, because LoRa quantizes payload
symbols and we already exploit that (9-12 B all cost 10.304 ms, which is what
makes DITTO's 2-byte reference free). If a control frame's symbol count had
headroom, stronger FEC on the safety-critical link might have been free:

| Body | On-air | CR 4/5 | CR 4/6 | CR 4/7 | CR 4/8 |
|-----:|-------:|-------:|-------:|-------:|-------:|
| 2 B (DITTO) | 10 B | 10.304 | 11.328 | 12.352 | 13.376 |
| 4 B (SKIP/probe) | 12 B | 10.304 | 11.328 | 12.352 | 13.376 |
| 9 B (ctrl min) | 17 B | 12.864 | 14.400 | 15.936 | 17.472 |
| 12 B (ctrl full) | 20 B | **14.144** | 15.936 | 17.728 | 19.520 |

It is not free — CR 4/8 costs +3.07 ms even on the smallest frame. The
hypothesis was wrong and is recorded as such.

One asymmetry does survive and is worth keeping in the design's back pocket:
**a DITTO at CR 4/8 (13.376 ms) is still cheaper than a FULL control frame at
CR 4/5 (14.144 ms)**. If a future field measurement shows control-frame bit
errors, the repeat frame can be armoured to maximum FEC and *still* fit inside
the slot budget already reserved for an unarmoured full frame. That is a free
option we are not exercising yet.

### 7.6 Decision, and the trigger to revisit

**Keep CR 4/5 on all three profiles.** Today's losses are deafness and
scheduling, provably not corruption, and every step up the CR ladder costs
19-57% of the image throughput that is the current priority.

This conclusion is bounded by range. The bench boards sit a few feet apart with
enormous link margin; field range will shrink SNR and bit errors will
eventually appear. The decision should be revisited on evidence, not on a
calendar, and the evidence is already instrumented:

> **Revisit trigger:** if `rx_decode_err / rx_frames` exceeds ~1% in any field
> run, corruption has become real and CR 4/6 (+19%) is the first step. Until
> that ratio leaves zero, raising CR is pure airtime loss.

`rx_decode_err` counts only packets that locked their header and failed CRC, so
it is a clean corruption signal and will not be contaminated by the deafness
losses discussed above. Range work should log RSSI/SNR alongside it — those are
published to `link_stats` on MQTT but are only written to `rx_daemon.log` at
DEBUG level, so an INFO-level periodic margin line is a small instrumentation
gap worth closing before the first range test.

---

## 8. The keyframe request should be redesigned, not deleted

### 8.1 The verdict from §7.3

The C1/C1b vs C2/C2b A/B is unambiguous and replicated: turning the self-heal
keyframe request on costs **+1.88 points of fragment loss**, delivers **16%
fewer complete frames**, and produces **31% more reassembly timeouts**. It
increases the very condition it fires on.

The mechanism is understood. Every base transmission deafens the base for its
own time-on-air plus the host round trip (~40 ms) against a ~100 ms fragment
cadence, so each request costs ~0.4 fragments — measured 0.47. Losing fragments
creates reassembly timeouts, timeouts fire requests, requests lose fragments.
The loop is bounded (`_set_pending` caps at 20 attempts / 10 s, and
`KEYFRAME_CMD_MIN_GAP_S = 10.0` gates re-pokes) so it does not run away, but it
does sit at a stable operating point that is worse than not reacting at all.

### 8.2 Why deleting it outright would be wrong

The obvious response — default `LIFETRAC_KF_REQUEST_DISABLE=1` and move on —
would trade a measured throughput gain for an unmeasured and worse image
defect, because of what the base-side canvas actually is:

> `image_pipeline/canvas.py:1` — *"Persistent tile canvas... keeps the
> most-recent encoded blob per tile."*

The canvas is persistent. When a tile update is lost in flight, the base keeps
displaying the **old** blob for that tile, indefinitely, until the encoder
happens to send that tile again. The encoder's own age-escalation
(`TILE_AGE_ESCALATE_FRAMES` in `camera_service.py`) does not help here: it
escalates on *encoder-side* age — how long since the encoder re-encoded a tile
— and the encoder has no idea which tiles the base failed to receive. In a
static scene the encoder has no reason to resend an unchanged tile, so a tile
lost once can stay visibly wrong on the operator's screen for as long as that
part of the scene holds still.

That is a worse failure than 1.88 points of fragment loss, and on a machine
where the operator is steering from that image it is the safety-relevant one.
The keyframe request is currently the **only** mechanism closing that loop. It
is load-bearing, and it must not be removed until something replaces it.

### 8.3 What is actually wrong with it

Both halves of the mechanism are mismatched to the problem:

- **Wrong trigger.** A reassembly timeout means "some fragments of one frame
  went missing." It says nothing about whether the *canvas* is stale — which
  is the condition we actually care about. Routine single-fragment loss (which
  at a 3.5% floor happens constantly) fires it needlessly.
- **Wrong response, by roughly two orders of magnitude.** A keyframe is the
  single most expensive thing this system transmits: ~13 fragments, **1.30 s
  of airtime** at a 3000 B budget. It is used to repair, typically, a handful
  of tiles.
- **Wrong schedule.** It fires when a timer expires, uncorrelated with the
  tractor's transmit pattern, so it lands wherever it lands.

### 8.4 Proposed replacement — a receiver-driven stale-tile report

Invert it. The base is the only party that knows what actually arrived, so let
the base report **canvas staleness** rather than request a retransmission:

- New opcode **`0x6C CMD_OP_TILE_STALE`**, body = `u16le base_seq_ref` +
  `u8 bitmap[(n_tiles+7)/8]`. At the 12x8 grid that is a 12 B bitmap, 15 B
  body, 23 B on-air, **15.4 ms** — versus 1298.8 ms for the keyframe it
  replaces, an **84x** reduction.
- The base computes it from state it *already keeps*: `TileState.arrived_ms`
  (`canvas.py:51`) is stamped per tile on every apply. Any tile whose
  `arrived_ms` is older than a threshold, and which the encoder has not
  refreshed, is stale. No new bookkeeping is required.
- The tractor treats the bitmap as **advisory dirty marks**, not as a
  retransmission command: those tiles enter the next encode's candidate set
  with elevated priority, exactly where `TILE_AGE_ESCALATE_FRAMES` already
  injects escalated tiles. The repair therefore rides the **next scheduled
  image frame** and consumes no additional image airtime — the encode-to-fit
  packer simply spends its existing budget on the tiles that need it.
- No frame buffering on the tractor, no retransmission state machine, no
  timing urgency. If a report is lost, the next one carries the same
  information, because staleness is a level, not an edge.
- It is control-plane traffic, so it belongs in the scheduled reverse slot
  (F2/F3) rather than the free-running pump.

This also settles where NACK-style signalling genuinely belongs. Per §4a, acks
on the command path were pure cost. Here the logic reverses: the receiver holds
information the transmitter cannot derive, the report is level-triggered rather
than edge-triggered, and it is 84x cheaper than the alternative. Feedback earns
its airtime when it carries knowledge only the receiver has.

### 8.5 What the keyframe request should still be used for

Keep it, narrowly, for genuine desynchronisation where an incremental repair is
not defined:

- stream start / decoder cold start,
- `base_seq` mismatch (the canvas already detects this — `canvas.py:1`),
- encode-mode or radio-profile change,
- a stale fraction so large that a keyframe is genuinely cheaper than the
  equivalent tile repairs (the bitmap makes this a computable comparison
  rather than a guess).

Explicitly **not** for routine fragment loss, which is what it does today.

### 8.6 Interim recommendation

Until `0x6C` exists, the honest position is that **neither setting is right**:
requests on costs 1.88 points and 16% of frames; requests off risks permanent
stale tiles. For bench throughput work keep `LIFETRAC_KF_REQUEST_DISABLE=1`
(the numbers there are what we are trying to measure, and the synthetic feed has
no operator looking at it). Do **not** ship that default to the tractor without
the replacement.

Roadmap item: **F10**, in the same batch as the reverse-slot work it depends on.
