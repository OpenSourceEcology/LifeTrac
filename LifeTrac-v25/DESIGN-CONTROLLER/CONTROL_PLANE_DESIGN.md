# LifeTrac v25 — Control Plane + Image Schedule Design

**Status:** design converged; awaiting first bench measurement (RS-0.12)
**Date:** 2026-07-27
**Supersedes the framing in:** [`SINGLE_RADIO_CONTROL_PLANE_OPTIONS.md`](SINGLE_RADIO_CONTROL_PLANE_OPTIONS.md)
(which is still the record of the option space and the rejected paths).

One radio carries everything (operator ruled out a second radio). Drive +
E-stop share the image-link radio (Murata L072 + SX1276, Method G HostLink)
with the video-like image stream. This document is the converged design across
six linked questions worked 2026-07-26/27. Every number here was computed with
the repo's own PHY model (`base_station/lora_proto.py:505` `lora_time_on_air_ms`)
and validated against the measured 2046 B/s / 82.8% baseline, or read from
code at the cited `file:line`.

> **MEASURED 2026-07-29 — results in
> [`bench-evidence/RS_0_12_phase_sweep_2026-07-29/RESULTS.md`](bench-evidence/RS_0_12_phase_sweep_2026-07-29/RESULTS.md).**
> The gating RS-0.12 run is done. Headlines, all tractor-side ground truth:
> the armed window **exists**, sits at frame completion, is **~60–80 ms wide**
> (far wider than the 8–10 ms guard this design assumed), and decays
> monotonically with offset.
>
> **Opportunistic delivery is ~91%, NOT the 53–57% first measured.** The
> earlier figures were depressed by our own instrument: the tractor's ECHO
> transmission occupies and deafens the very reverse-slot window the next
> command needs, costing **35 points** of forward delivery (91.1% echo-off
> vs 55.6% echo-on, disjoint CIs). At 91%, **two copies gives 99.2%** — so a
> one-way control plane looks achievable *opportunistically*, and the
> firmware slot becomes an optimization and a determinism guarantee rather
> than a precondition. This is also decisive for ack policy: **acking does
> not merely cost airtime, it cuts command delivery by a third.** See
> RESULTS.md §0 and §4a (ACK/NACK measured answers).
>
> **Frame size does not affect delivery** (23 B 15.1% vs 38 B 15.8%), which
> settles the envelope question in favour of GCM-128 and demotes ditto to an
> airtime-only optimization. Three things below are retracted by measurement
> and marked inline: the RS-3.10 bisection hypothesis (§8), D13's preference
> (§6), and the "53% is not enough" verdict (§7/RESULTS §4).

---

## 1. The core reframing

Prior sessions treated this as "0xFB command frames vs lora_bridge framing."
The measurements killed that framing. Three facts:

- **This is a scheduling problem, not a bandwidth/framing/crypto problem.**
  In-stream base→tractor delivery measured **0 of 386 copies** at DTS with the
  current opportunistic pump — a scheduling collision (the base fires into the
  middle of the tractor's transmission), not a link failure. Run J proved 58%
  is achievable when alignment holds.
- **The envelope is nearly free — at the boundary window.** A bare 15.4 ms
  frame, a D13 20.5 ms frame and a GCM-128 25.7 ms frame all fit the 242.6 ms
  train-boundary window ≥9× over. Under a *tight* reserved slot the envelope
  stops being free (§6).
- **Cadence is the constraint.** The usable listening window recurs 0.54×/s
  today. The whole design is about making a *reliable* window.

**Adopted direction: control-first TDMA.** Fix a control schedule (base→tractor)
as the primary, deterministic term; fit image (tractor→base) into the rest.
"Minimize control airtime" rather than "fit control into image's gaps."

---

## 2. Deadman vs control response (they are different numbers)

- **Deadman** (`CONTROL_TIMEOUT_MS=200`, `tractor_h7.ino:513`) — a *safety*
  staleness bound. Adds **zero** latency in normal operation; felt only on
  failure. Governs how far the machine travels after link loss. For radio-loss
  hazard H1 it is the **sole** line of defence (the M4 and Opta watchdogs
  detect M7 *death*, not control staleness — verified). One-loss tolerance
  requires `deadman ≥ 2 × control period`.
- **Control response** — a *performance* number felt every input: ~119 ms fixed
  (LAN sample + 20.5 ms air + 50 ms arbitration tick + Modbus + 30–50 ms spool)
  plus half a control period.

| Control rate | Mean response | Worst + 1 loss | Min deadman (1-loss) |
|---|---|---|---|
| 10 Hz | 170 ms | 389 ms | 200 ms |
| 5 Hz | 220 ms | 589 ms | 400 ms |
| 2 Hz | 370 ms | 1189 ms | 1000 ms |

Below ~3 Hz the valve executes a joystick value up to one period stale →
move-and-wait pulsing even on a perfect link. **5 Hz is the floor for
close-in maneuvering; 10 Hz is the target when image budget allows.**

Deadman policy: **do not raise it flat.** IEC 62745 §4.7.3.5 caps loss-of-signal
stop at a *recommended* 0.5 s over a *mandatory* risk assessment. The defensible
move is a **speed-gated ladder** holding uncommanded travel constant at today's
0.448 m: 200 ms @ 2.24 m/s … 750 ms @ 0.60 m/s … 1000 ms @ 0.45 m/s. Any value
> 250 ms also requires correcting the false "three independent chains" claim in
`SAFETY_CASE.md` first (see §9 defect list).

---

## 3. The superframe (DTS)

Natural quantum = one image fragment. N=1 superframe at DTS (BW500):

```
  fragment  99.904 ms   (247 B body + 8 B hop hdr = 255 B on air)
  guard A    ~5-10 ms   (tractor TX->RX + base RX->TX turnaround — UNMEASURED)
  control   15.4/20.5/25.7 ms  (bare / D13 / GCM-128)
  guard B    ~5-10 ms
  --------------------
  period    ~125-145 ms  ->  ~7-8 Hz control
```

**Cost:** the honest, authenticated, one-loss-tolerant figure is **−19% to −33%
of image goodput (mid-case −26%)** — i.e. this *is* option A4, implemented with
phase knowledge instead of probabilistic reservation. The "~4%" earlier figure
assumed a 5 ms guard, a bare frame, and no one-loss margin simultaneously; it is
withdrawn. **The guard budget is the entire design and it is unmeasured** — this
is what §8 measures.

FHSS (BW250) is DIFFERENT and must stay so: fragment 169 ms, control 2× the
bytes, and it already ships a **200 ms slot clock** (`SX1276_FHSS_SLOT_MS=200`
at `sx1276_fhss_clock.h:52`, 12 ms head-start, 15 ms guard, 50 channels). A control schedule there rides the
existing grid; a single schedule sized for FHSS costs DTS ~41%. **Rule: one
period (200 ms), two interiors.**

---

## 4. Demand-adaptive rate — and why the skip frame supersedes the state machine

The deadman only protects *motion*. When the operator commands neutral, control
staleness is harmless (the machine is already in the state a deadman trip would
force). So the control rate can be demand-driven. Two designs were costed:

- **Two-state (IDLE 2 Hz beacon / ACTIVE 10 Hz, 3.5 s dwell):** blended
  1518–1758 B/s at 10–50% driving duty (+25–45% over always-on 10 Hz). Works,
  but needs a dwell timer and stuck-state detectors, and the SF link ladder
  must be taught that idle-with-heartbeats is not "bad" (else it steps SF down
  after 15 s of idle — a real firmware bug, §9).
- **Per-slot SKIP FRAME (preferred):** in every control slot the base sends
  either the queued control frame or a **10.3 ms skip frame** (8 B hop header +
  1 B payload; 9–12 B all quantize to the same 10.304 ms, so 4 B of freight —
  skip bit, heartbeat flags, fragment-acks, budget byte — ride free). On decode
  the tractor contracts the superframe (next fragment starts early). This
  **replaces** the state machine: no dwell, no stuck detectors, and escalation
  is one superframe (~144 ms worst) vs ~530 ms.

**Skip = explicit frame, NOT CAD silence-detect.** CAD's throughput edge over
the skip frame is ~2–5% (the arbitration floor forces base TX every few slots
anyway), while CAD adds a both-frames-lost collision mode (miss the preamble →
tractor keys a fragment over the control frame → the 0/386 failure in
miniature), starves the phase estimator during idle, is trivially jammed, and
needs a firmware CAD-scan subsystem that does not exist (`sx1276_cad.c` is
single-shot, drops to STANDBY on detect, only consumer is the disabled LBT
path). The skip frame's failure mode is benign: a lost skip just makes the
tractor wait out a ~43 ms worst-case control window, +25–30 ms, no collision.

Two constraints: **a mandatory un-skippable beacon every K≈3 slots** (349 ms <
the 500 ms `SRC_BASE` heartbeat window, feeds the SF ladder); and an
**unauthenticated skip must never feed arbitration eligibility** (a replayed
skip-as-heartbeat could keep `SRC_BASE` alive with no operator present — carry
it under D13, or authenticate the mandatory beacon).

FHSS can't contract (grid welded to the hop law) → there skip *steals the slot*
(~201 B extra image payload), grid unchanged.

### 4a. The "ditto" repeat frame — three slot occupancies, not two

Skip covers "operator idle." It does **not** cover the common driving case:
the operator holds a steady stick and the next ControlFrame is *byte-identical
to the last*. Re-sending 16 bytes to say "no change" is waste. So the control
slot has **three** occupancies, selected by a 2-bit type in the frame the slot
already carries:

| Slot type | Meaning | On air (BW500, D13) | Refreshes deadman? | Superframe |
|---|---|---|---|---|
| **SKIP** | nothing to send (operator idle) | 10.30 ms | **No** | full contraction |
| **DITTO** | re-apply ControlFrame `ref_seq` | 15.42 ms | **Yes** (if ref matches) | partial contraction |
| **FULL** | new ControlFrame | 20.54 ms | Yes | none |

**Why skip and ditto must stay semantically distinct even though the wire cost
is similar:** skip deliberately lets control go stale — safe, because the
operator is commanding neutral and staleness converges to exactly the state the
deadman would force. Ditto deliberately *keeps control alive* at a non-neutral
value. Same slot, opposite deadman intent. That is the whole reason it is two
bits rather than one.

**THE SAFETY HAZARD, and the design that removes it.** A naive "repeat last"
ditto is dangerous, and in a way that defeats the deadman:

> Base sends FULL "stop" (a transition). The frame is lost. The operator is
> still holding stop, so the next slot is — from the base's point of view — "no
> change", and it sends a ditto. The tractor never received "stop"; it repeats
> its last-applied command, **"forward"**. Every subsequent ditto refreshes the
> deadman, so the machine drives on indefinitely while the base believes it
> commanded a stop.

That converts a single lost frame into unbounded stale motion — strictly worse
than plain loss, which the deadman handles. The fix is to make the ditto
*referential* rather than relative: it carries the **u16 sequence number of the
ControlFrame it is repeating**, and the receiver honors it **only** if that
matches its own last-applied frame. Any mismatch, any malformed ditto, or a
receiver that has applied nothing yet → **ignore, do not refresh freshness** →
control goes stale → 200 ms deadman → neutral. A desync degrades to the
ordinary loss path, which is already safe.

This is implemented and unit-tested now (`lora_proto.ditto_applies()` is a pure
function precisely so the H7 implementation can reuse the contract verbatim):
`CMD_OP_CTRL_DITTO = 0x6B`, args `u16le ref_seq`.

**The reference byte is free.** LoRa symbol quantization means 9–12 B of on-air
payload all cost 10.304 ms at BW500, so a ditto with its 2-byte ref is the same
airtime as a bare skip. Belt-and-braces: bound ditto runs (force a FULL frame
every K dittos) so any undetected divergence self-heals within K slots.

**Honest airtime accounting.** Authenticated, a ditto saves **exactly one
quantization step**: 5.12 ms at BW500 (D13 ditto 15.42 vs control 20.54),
10.24 ms at BW250. At 8 Hz with a 70% ditto rate that is ~29 ms/s ≈ **~71 B/s
of image goodput (~3.5%)** — real but modest. Unauthenticated it would save
10.24 ms, but an unauthenticated ditto is a replay weapon (capture one, keep the
machine moving) and is rejected for the same reason as bare 0xFB actuation.

**The bigger prize is delivery probability, and it is unmeasured.** A 15.4 ms
frame fits a marginal reverse window that a 20.5 ms frame misses. If the window
turns out tight, ditto's value is mostly *reliability*, not airtime — so
tomorrow's probe sweeps **size as well as phase** (§8 run 2) to measure
P(delivery | phase, size) directly.

**Ditto hit rate depends on quantization, not on wishful thinking.** Axes are
integers in −127…127 with `AXIS_DEADBAND = 13`, so neutral (→ exactly 0),
pinned full-travel, and held detents produce byte-identical frames — precisely
the sustained-driving cases. Mid-range analog jitter will not ditto, and should
not: an epsilon ("near enough") would trade control fidelity for airtime and is
left as a tunable, defaulting off.

---

## 5. Encode-to-fit (implemented tonight)

Because the schedule makes single-fragment frames the natural unit, the encoder
targets the transport quantum exactly. **Mechanism: greedy tile-packing, never
iterative re-encode** (WebP size is non-monotonic in quality; a per-frame Q
search only converges statistically and its multi-pass cost can blow the frame
period on the A53s). Quality adapts *across* frames via a slow servo (design,
not yet built). Landed this session:

- **Budget exactness** — the 6 B header + changed-bitmap are now charged inside
  the cap, and the oversized-first-tile bypass is removed. A "243 B" frame is
  now truly ≤ 243 B on the wire = one fragment. (Was 261 B = 2 fragments, a
  runt every budget-full frame — the exact sawtooth the schedule exists to kill.)
- **Greedy continue** — on overflow, scan up to `OVERFLOW_SCAN_LIMIT` more tiles
  for a smaller fit instead of abandoning the budget.
- **Carry fix** — a changed tile dropped by the budget has its OLD pixels
  spliced back into the diff base so it re-flags next frame (was silently
  forgotten until the sweep reached it, ~48 frames later).
- **Age-escalation** — a tile older than `TILE_AGE_ESCALATE_FRAMES` jumps to the
  front, oldest first → rolling refresh provably converges under sustained
  motion.
- **Live profile budget** — `image_tx_daemon` publishes `{n_fragments, profile}`
  retained on `tractor/link_budget`; `camera_service` feeds it into the existing
  `LinkBudget.update` seam. The encoder finally sizes to the live radio quantum
  (243 DTS / 203 FHSS) instead of a boot-frozen env default. Also fixed the
  batcher's hardcoded 243 under FHSS.
- **Quality-only ≠ keyframe** — a quality change no longer forces a full-canvas
  keyframe (~1–2 s of air). Quality is not on the wire; tiles are self-describing.

**Codec policy (verified crossover):** at 243 B, mono_g4 carries 45 tiles,
y_only 11, full color 8. Below ~500 B only mono_g4 buys coverage; y_only owns
the 1–2 KB band; full color pays only at ≥2.6 KB. Single-fragment frames also
survive the measured 4.3% loss at **95.7% vs 56.5%** for 13-fragment frames.
→ **Drive on mono_g4/rawstream single-fragment frames; park on full WebP + parity.**

---

## 6. Envelope

Under the opportunistic design the envelope was free, so GCM-128 (handheld
parity, already decrypted by `tractor_h7.ino:1106`) was the pick. **Under the
tight slot the envelope is no longer free** — at N=1 DTS, GCM-128 costs ~7.6% of
image goodput vs bare, D13 ~3.9%. This reopens the choice:

- **If the skip-frame contraction design wins** (§4), the control slot is small
  and frequent, so D13 GCM-64 (+12 B) saves real image goodput over GCM-128.
- **But** D13's implicit nonce makes RS-8.3 `boot_ctr` a hard blocker and forces
  an atomic three-tree handheld cutover; GCM-128 sidesteps both.

**SETTLED 2026-07-29 — GCM-128.** The sweep measured frame size to have **no
effect on delivery** (23 B 15.1% vs 38 B 15.8%, +0.7 points, inside noise and
favouring the *larger* frame). Since bytes buy no reliability, the choice
reverts to compatibility, and GCM-128 is byte-for-byte the shape
`tractor_h7.ino:1106` already decrypts — no `boot_ctr` blocker, no atomic
three-tree handheld cutover. **This supersedes RS-8.6's D13 preference.**
Either way: no actuation opcode goes on the link without AEAD + replay windows
+ `boot_ctr` closed first (a forged 0xFB `RADIO_PROFILE_ACK` already retunes
the base PHY).

---

## 7. What still gates a shippable drive plane (unchanged from the options doc)

- **Route B** — the H7's control/arbitration/E-stop-latch/Modbus half is
  compiled out under the Method G build (the only build that reaches the radio).
  Every delivered-drive design needs it re-homed onto the HostLink RX path.
- **SRC_BASE arbitration** — the base never emits `FT_HEARTBEAT`, so it can
  never win arbitration. The skip/beacon frame supplies it for free (§4).
- **E-stop by absence** — the safety-critical stop must not depend on a
  delivered packet on a link measured at ~0% in-stream delivery. Motion-arrest
  = deadman; engine-kill = slow retried latch.

---

## 8. Tomorrow's bench plan (ordered)

All runs DTS BW500 unless noted. Boards + serials per the bench memory. The code
for every run below shipped tonight.

1. ~~**Run-J bisection (RS-0.13b).** Toggle one RS-3.10 knob per run;
   whichever restores delivery identifies what broke Run J.~~
   **DONE 2026-07-29 — and the hypothesis was WRONG.** Neither
   `parity_group`, `TX_PREPARE_AHEAD` nor `train_gap_ms` was responsible;
   all three were off in both the 53% run and the 15% run. The full 2×2 shows
   **pipeline v3 vs v2 is the ENTIRE effect**: +57 points at 3000 B (0%→57%),
   +38 at 250 B (15%→53%). Run J used v3; every run since defaulted to v2
   because that is the *harness* default (`run_live_radio_monitor.ps1:24`) —
   a test-fixture regression no code diff could reveal. **Frame size has no
   effect on v3** (57% vs 53%, overlapping CIs); it only appeared to matter on
   v2, where small frames partially compensated for v2's broken RX arming.
   **Consequences: (a) harness default flipped to v3; (b) KEEP 3000 B frames —
   they give the best delivery AND the best goodput (2005 B/s, near the 2046
   record), so the "shorten trains for more boundaries" lever in §4 is
   unnecessary.**
2. **Reactive-fire delivery + phase×size sweep (RS-0.12) — the decisive run.**
   `-ReactiveFire 1 -ProbePhaseSweepMs "0,20,40,60,80,100,120" -ProbeSizesB "23,38"`.
   The base fires a no-op PROBE at each fragment-RX-complete; the tractor
   echoes it. Sizes 23 B and 38 B are a **D13 ditto** and a **D13 control
   frame** — the same two frames the real control plane will send — so one run
   yields both the phase answer and the ditto answer. Read `probe_grid:` (the
   cumulative per-bin `pN/sM:echo/att(P%)` table), `probe: … rtt_med=…`, and
   the new `air_gap_hist(ms)` from `rx_daemon.log`.
   **What it decides:** the guard budget (the number the whole superframe cost
   rests on); P(delivery | phase) — best-bin high → §3's DTS schedule is
   buildable, never above ~60% → fall back to stream-off (options-doc A5); and
   P(delivery | size) — if the 23 B bin materially beats the 38 B bin, ditto's
   value is reliability rather than the modest ~3.5% airtime saving, which
   promotes it from nice-to-have to load-bearing.
   Suggested duration ≥10 min so each of the 14 (phase × size) bins gets ≥30
   samples at the 0.5 s min gap.
3. **Encode-to-fit verify.** `LIFETRAC_FRAGMENT_BUDGET=1` (single fragment).
   Confirm every frame is one fragment at both profiles (no runt sawtooth),
   goodput at 243 B vs today's 3000 B, and the mono_g4/y_only tiles-per-frame
   match §5. Then a quality sweep 40→80 confirming no keyframe storm.
4. **Only after 1–3:** decide envelope (§6) — now informed by the size axis,
   since a tight window makes the +12 vs +28 B difference a *delivery* question
   and not just an airtime one — then start Route B.

**Reading the results — decision table:**

| Observation | Conclusion | Next action |
|---|---|---|
| Bisection restores in-stream delivery | the RS-3.10 knob that broke Run J is identified | pin it; re-baseline goodput |
| `probe_grid` best phase bin ≥ ~90% | the armed window is real and hittable | build the DTS slot clock (firmware F1–F3) |
| Best bin 60–90% | window exists but is marginal | widen guards / lengthen control preamble (F4), re-measure |
| Best bin ≤ ~60% at every phase | in-stream delivered drive is not solvable here | adopt A5 stream-off doctrine; ditto/skip become moot |
| s23 bin ≫ s38 bin | frame size dominates delivery | ditto is load-bearing; prefer D13 over GCM-128 |
| s23 ≈ s38 | size is not the constraint | ditto is an airtime nicety; prefer GCM-128 (handheld parity) |
| `air_gap_hist` shows a clean boundary mode | contraction has room to work | proceed to skip-frame firmware |

The old RS-0.9 air-test queue (encode mode+quality smoke, convergence re-check)
still applies and folds into runs 2–3.

---

## 9. Defect list surfaced by this analysis (tracked in TODO)

Verified in code; several fixed tonight, the rest queued:

- **[safety] `SAFETY_CASE.md` "three independent chains" is false for H1** — the
  M4 and Opta watchdogs detect M7 death, not control staleness; on radio loss
  `CONTROL_TIMEOUT_MS` is the sole defence. The false claim is repeated in
  `TRACTOR_NODE.md`, `TODO.md`, and the comment above `pick_active_source()`.
  Two H1 citations are also dangling. **Fix before any deadman change.**
- **[safety] `/api/estop` does not gate the control publisher** — after a
  software E-stop, `ws_control` keeps forwarding 20 Hz joystick frames
  (`web_ui.py:874` gates on `active_source` only). *(fixed earlier session)*
- **[control] SRC_BASE can never win arbitration** — base never emits
  `FT_HEARTBEAT`. *(queued — folds into the skip/beacon frame)*
- **[firmware] SF link ladder steps down after 15 s idle** — counts any 5 s
  window without fresh heartbeat+control as bad. *(queued)*
- **[firmware] RXCONT re-arm** — RS-4.12 shipped and works (`sx1276_modes_sync_external`);
  the "deaf inter-fragment gaps" claim in earlier notes is STALE.
- **[encoder] budget header spill / frozen profile / hardcoded 243 / forced
  keyframe on quality change** — *all fixed tonight (§5).*
- **[base] keyframe-ack is a contaminated delivery signal** — a keyframe
  arriving does not prove our request landed (source of the retracted 171/1).
  *(fixed tonight: logged UNVERIFIED; CMD_OP_PROBE is the honest signal.)*
- **[base] gap-tolerant merge floods keyframe requests at high frame rate** —
  needs debounce before N=1 ships. *(queued)*
- **[encoder] ROI planner bypasses `WEBP_QUALITY`** — the future quality servo
  must scale the ROI pair too. *(queued with the servo)*

---

## 10. L072 firmware roadmap — what the custom firmware needs for this plan

The host daemons can *measure* the schedule (that is tonight's instrumentation),
but they cannot *guarantee* it: train pacing lives in non-realtime Python, the
L072 TX ring is strict FIFO with no abort, and with `PIPELINE_DEPTH` fragments
parked in the mailbox the host's key-up uncertainty is 200–400 ms. **The
schedule must ultimately live in the L072 firmware.** The good news: the
v25.0.7 slot-clock work already built most of the hard parts, and the required
hook exists — `sx1276_tx_slot_wait_us()` is a pure advisory consulted at BOTH
TX admission points (`host_cmd.c:492` park, `:923` drain), so a firmware mute
gate re-evaluates already-parked fragments and nothing can outrun it.

### Batch 1 — no-regrets fixes (land after tomorrow's runs, before TDMA)

| # | Change | Why | Where |
|---|---|---|---|
| F6 | **Fix the epoch-drift lock-out** | `scan_feed_frame(true)` fires even on `REJECTED_EPOCH_DRIFT`, so the scan SM stays LOCKED, the 2000 ms loss demotion never runs, and the clock reset that would recover is never reached — a **permanent, unrecoverable desync** that exists today, independent of TDMA. | `sx1276_rx.c:214`, `sx1276_fhss.h:141` |
| F7 | **Fix the one-sided phase bias** | `slot_offset_ms` is sampled *before* PLL settle + FIFO burst (`sx1276_tx.c:206/224`), so TX always keys later than advertised; the RX anchor then truncates ToA µs→ms (`sx1276_rx.c:199-201`, −0.904 ms on a full fragment). Both errors push the same way and silently eat guard margin. Sample at actual key-up; round, don't truncate. | `sx1276_tx.c`, `sx1276_rx.c` |
| F8 | **Expose phase telemetry in RX_FRAME_URC** | The firmware strips the 8 B hop header before delivery (`sx1276_rx.c:389-402`) and `RX_FRAME_URC` carries no epoch/hop_idx/slot_offset — slot alignment is currently **unverifiable from either host**. Append the header fields to the URC (additive, host parsers ignore extra bytes). | `host_cmd.c:955-976` |
| F9 | **Make opmode-sync first-class** | RS-4.12's `sx1276_modes_sync_external()` — the fix that makes host RXCONT arming visible to firmware — is reachable only through `HOST_ALLOW_REG_WRITE_DIAG=1` (`config.h:61`), a "diagnostic" flag whose own comment says keep conservative in production. Turning it off silently re-breaks RX arming. Promote to a dedicated host op or always-on path. | `config.h`, `host_cmd.c:594` |

### Batch 2 — the TDMA schedule itself (gated on the RS-0.12 measurement)

| # | Change | Why | Where |
|---|---|---|---|
| F1 | **DTS slot clock** | The slot machinery is FHSS-only: `sx1276_tx.c:187-193` zeroes hop/epoch/slot_offset for DTS and `sx1276_tx_slot_wait_us()` short-circuits for non-FHSS profiles (`:510`). DTS needs a *virtual* grid (same clock TU, no hopping) so both ends share slot phase — the header fields already exist to carry it. | `sx1276_tx.c`, `sx1276_fhss_clock.*` |
| F2 | **Control-window mute gate** | Extend the slot-wait advisory into "never key up inside the control window." Because it is consulted at park AND drain, parked fragments respect it with no TX-abort needed. | `host_cmd.c:492/:923` |
| F3 | **Skip/ditto handling + contraction** | Tractor: decode the slot type and contract by the frame's actual length (skip = full contraction, ditto = partial, full = none). Base: IRQ-driven auto-TX of a pre-armed control/ditto/skip frame at fragment-RX-done + PLL settle (tightens the listen window ~10 → ~5 ms). Ditto adds the H7-side rule: honor only on exact `ref_seq` match (`lora_proto.ditto_applies()` is the reference implementation), else let the deadman run; and force a FULL frame every K dittos. | `sx1276_rx.c`, `sx1276_tx.c`, `host_cmd.c`, `tractor_h7.ino` |
| F4 | **Per-frame-type preamble** | Control frames get a longer preamble (12–16 symbols, ~+1–2 ms) for detection margin. `RegPreambleMsb/Lsb` are never written today (POR default 8); the airtime guard already reads the registers back (`sx1276_airtime.c:171`), so the budget check auto-honors it. | `sx1276_tx.c` |
| F5 | **P0 reserved slot in the host TX ring** | `HOST_TXQ_P0_RESERVED` is defined in `config.h:58` and referenced by **zero lines of C** — the designed priority reservation was never implemented. A control lane in the ring lets an urgent frame (engine-kill latch) jump parked image fragments. | `host_cmd.c` |

**Recommendation on sequencing:** do NOT flash new firmware before tomorrow —
the bench plan (§8) was deliberately designed to run on the current build
(tractor-side logging sidesteps the missing F8 telemetry). Run the measurement
first; land Batch 1 immediately after (F6/F7 are latent bugs regardless of
architecture); land Batch 2 only once the phase sweep confirms the slot design
and sets the real guard numbers that F1–F3 must implement.

## 11. Provenance

Six analysis workflows (2026-07-26/27), each with an adversarial critic, every
load-bearing claim verified against the code by the author before inclusion.
Underlying measurements:
[`bench-evidence/RS_throughput_rebaseline_2026-07-25/CONTROL_PLANE_REANALYSIS_2026-07-27.md`](bench-evidence/RS_throughput_rebaseline_2026-07-25/CONTROL_PLANE_REANALYSIS_2026-07-27.md).
Numbers marked as costs/latencies for future schedules are **derived estimates**
from the measured geometry, not measurements — the RS-0.12 run converts them.
