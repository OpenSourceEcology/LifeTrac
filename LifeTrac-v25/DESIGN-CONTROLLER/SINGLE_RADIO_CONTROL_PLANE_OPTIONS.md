# Single-Radio Control Plane — Options Analysis

**Status:** decision document, no option adopted yet
**Date:** 2026-07-27
**Decision owner:** operator (james@senaxinc.com)
**Context:** the operator has ruled out a second radio. Hydraulic drive and
E-stop must therefore share the image-link radio (Murata CMWX1ZZABZ =
STM32L072 + SX1276, Method G HostLink) with the video-like image stream.
Image throughput is the standing priority; base→tractor control must be
maintained.

> **Read this first.** Two beliefs this project held until 2026-07-27 were
> wrong, and they invalidate most prior reasoning about this decision:
>
> 1. **The "171/1 command convergence" result is retracted** (see TODO.md
>    RS-2.2). It counted convergences, not deliveries. Ground truth: in that
>    same 30-minute soak the tractor received **26 commands total, 6
>    in-stream, all six inside a 3.19 s firmware-fault transmitter stall.**
> 2. **In-stream base→tractor command delivery is ~0** with the current
>    stack: **0 of 386 copies** at DTS with the reverse slot enabled, 1 of
>    701 at FHSS.
>
> Full evidence:
> [`bench-evidence/RS_throughput_rebaseline_2026-07-25/CONTROL_PLANE_REANALYSIS_2026-07-27.md`](bench-evidence/RS_throughput_rebaseline_2026-07-25/CONTROL_PLANE_REANALYSIS_2026-07-27.md)

---

## 1. What the measurements actually constrain

Every number below is measured from `bench-evidence/radio_monitor_20260726_*`
or read from code, not estimated.

| Fact | Value | Consequence |
|---|---|---|
| Inter-fragment dead air (DTS) | **4.98 ms** | Nothing fits. Not the 10.3 ms frame actually flown, not a 15.4 ms drive frame. |
| …and it is **deaf by construction** | `sx1276_tx.c:259` | Re-arms RX only when the *firmware-tracked* state is RX_CONT; the host arms via a raw register write the firmware never sees, so the modem parks in STANDBY after every fragment. |
| Train-boundary window (armed) | **242.6 ms every 1850 ms** (record); 92.3 ms every 449 ms (soak) | The **only** listening window today. Restored once per train at `image_tx_daemon.py:1073`. |
| Usable window cadence | **0.54 /s** | This is the binding constraint. |
| Envelope cost at that window | bare 15.4 ms / D13 20.5 ms / GCM-128 25.7 ms | All fit **≥9× over**. Crypto choice buys **zero** extra delivery opportunities. |
| Spare airtime | 17–24% of wall clock already dead | 10 Hz × 15.4 ms = 15.4%. Airtime is **not** the constraint. |
| Best in-stream delivery ever achieved | **58%** (Run J, 14/24 copies, same ~75% util) | Delivery is achievable; the current stack lost it. |

**The three conclusions that reframe the whole decision:**

1. **This is a scheduling problem, not a bandwidth, framing, or crypto
   problem.** The record run's commands land uniformly across the train
   period (offset p10=176 / med=1225 / p90=1570 ms in an 1869 ms period whose
   armed window is the first ~243 ms) — the base fires squarely into the
   middle of the tractor's transmission.
2. **Frame size is irrelevant.** Pick the envelope on security and
   compatibility grounds alone.
3. **A working configuration exists in the archive.** Run J did 58% with
   `parity_group`, `TX_PREPARE_AHEAD` and `train_gap_ms` all *off*. Something
   in the RS-3.10 change set destroyed a working alignment, and **no run
   varies those three knobs one at a time.**

---

## 2. Options by layer

A complete design picks one option per layer. Verdicts: **valid** (sound,
buildable), **plausible** (sound but conditional/secondary), **rejected**
(killed by verified evidence).

### 2.1 Scheduling — how to create a reliable listening window

| Option | Verdict | Summary |
|---|---|---|
| **S1. Reserved periodic armed window** (self-clocked TDMA) | **valid** | Tractor hard-mutes TX on a known grid; base phase-locks and fires into it. **Zero extra airtime to share the grid** — `slot_offset_ms` and `epoch` already ride on every frame (`lora_pkt_hdr.h:52,55`, verified). Min window ≈ 15.4 ms frame + guard ≈ 25–30 ms. |
| **S2. Fix the RXCONT re-arm bug** | plausible | Makes inter-fragment gaps genuinely armed. But they are only 4.98 ms, so this alone delivers nothing — it must be paired with *lengthening* them. Touches L072 firmware (higher risk). |
| **S3. Shorten trains / cap frame size** | plausible | More boundaries per second with **no firmware change**. Costs ~50% goodput to reach 5 windows/s (per-train overhead multiplies, fighting RS-3.1 batching). Use for a 500–700 ms target, not to chase the 200 ms deadman. |
| **S4. Recover Run J's alignment** | **valid** | Turn the three RS-3.10 knobs off, one at a time. Zero code. Highest value per unit cost. |

### 2.2 Allocation — who gets the channel, when

| Option | Verdict | Summary |
|---|---|---|
| **L1. Best-effort opportunistic** | plausible | No reservation; stutter-to-neutral under loss. ~1.35 s per one-way command even at Run J's 58% — **disqualified for closed-loop drive and for E-stop**; discrete commands only. |
| **L2. DRIVE / LOOK mode switch** | **valid** | Operator toggle or joystick auto-detect shrinks the encode in DRIVE (raising window cadence ~4×), full image in LOOK. **0% cost when parked**, ~50–70% while actively driving. |
| **L3. Always-on reserved slice** | **valid** | Cap the airtime budget below saturation so armed idle always exists. ~24% permanent image cost (2046 → ~1550 B/s) to refresh at ≥5 Hz. The honest cost of never stuttering. |
| **L4. Control-first self-throttling** | **valid** | Tractor serves the control plane before starting each train; image yields exactly what control consumes. ~0% idle cost, ~15–24% under active control. |

### 2.3 Safety timing — what "good enough" means

| Option | Verdict | Summary |
|---|---|---|
| **F1. E-stop by absence** | **valid — non-negotiable floor** | 200 ms control staleness (`tractor_h7.ino:513`) zeroes all valves when frames stop arriving. Airtime-free, unforgeable, delivery-independent. **The safety-critical stop needs no delivered packet.** Relaxes every other layer's cadence target from 200 ms to the 500–700 ms operator budget (~3× cheaper). |
| **F2. Absence + slow-retried engine-kill latch** | **valid** | Motion stops by absence; the latching engine-kill is a separate slow retry across many windows, off the timing-critical path. |
| **F3. `/api/estop` must gate the control publisher** | **valid — defect fix** | **Verified defect:** `_base_controls_allowed()` (`web_ui.py:874`) gates only on `active_source`, never on E-stop state, so after a software E-stop `ws_control` keeps forwarding 20 Hz joystick frames. A lost E-stop packet leaves the machine moving *and* the base actively contradicting it. |
| **F4. Ramp-to-neutral (preferred) vs relaxing the timeout** | plausible | The neutral path is a **stutter, not a latch** — `apply_control(-1)` keeps ticking `REG_WATCHDOG_CTR` so the Opta never trips. Prefer a ~100–150 ms ramp (keeps 200 ms) over relaxing `CONTROL_TIMEOUT_MS`, which breaks the SAFETY_CASE §4 three-chain-agreement invariant. |
| **F5. Link-loss alarm + 1 Hz alive/E-stop beacon** | **valid** | One radio means image and control die in the *same instant*, so the operator cannot see whether the machine stopped. ~1.5% airtime buys back situational awareness. |
| **F6. Minimum auth before any actuation opcode** | **valid** | AEAD + replay windows both ends, RS-8.3 `boot_ctr` closed first. A forged 0xFB `RADIO_PROFILE_ACK` already retunes the base PHY today (`image_rx_daemon.py:604`). |

### 2.4 Envelope

| Option | Verdict | Why |
|---|---|---|
| **E1. Bare 0xFB plaintext** | **rejected** | `parse_command_frame` validates a magic byte + opcode membership, then executes. Any transmitter could move the machine or suppress an E-stop. |
| **E2. D13 GCM-64 (+12 B)** | plausible→**rejected as the choice** | The byte saving is **airtime-free** at the boundary window, while the implicit nonce makes RS-8.3 `boot_ctr` a hard blocker and forces an atomic three-tree handheld cutover. Choosing D13 to avoid a cross-tree cutover is self-contradictory. |
| **E3. GCM-128 explicit nonce (+28 B)** | **valid — settle here** | Byte-for-byte the shape `tractor_h7.ino:1106` already decrypts (`len < 12 + 16`). Reuses shipped `lp_decrypt`/replay, needs no handheld cutover, and its self-defending explicit nonce sidesteps the `boot_ctr` hazard. |

### 2.5 Integration — air to valves (the dominant shared work item)

| Option | Verdict | Why |
|---|---|---|
| **I1. Route A: X8 Python → H7 IPC → Modbus** | **rejected** | **Verified:** the entire Modbus master + `apply_control` + `pick_active_source` block is inside `#if !LIFETRAC_METHOD_G_HOST_BUILD` (`tractor_h7.ino:136`–`1802`). Under Method G — the only build that reaches the radio — `loop()` calls `mh_runtime_loop()` and returns. There is no Modbus master for a dispatcher to reach. |
| **I2. Route B: merge the control half into the Method G build** | **valid — but large** | Re-home the existing, tested control/arbitration/E-stop-latch/Modbus logic onto the HostLink RX path. Must also restore the M4 seqlock `alive_tick_ms` feed and Opta watchdog tick that today live only in the compiled-out loop. **Every delivered-drive architecture depends on this.** |
| **I3. Fix SRC_BASE arbitration** | **valid** | **Verified:** `pick_active_source` (`tractor_h7.ino:585`) requires a fresh heartbeat **and** a fresh control frame, but `web_ui.py:36` imports only `pack_control` and never calls `pack_heartbeat` (which exists at `lora_proto.py:266`). **Any base-originated drive is silently inert today.** |
| **I4. Debuggability floor** | **valid** | `_dispatch_command` silently `return`s on any non-0xFB frame with no log or counter — the failure mode that shipped a dead bridge for 68 minutes unnoticed. Add per-magic RX and MAC-fail counters plus a fault on undeliverable ControlFrame-shaped traffic. |

---

## 3. Coherent architectures

| # | Name | Image cost | Control latency | Effort | Best for |
|---|---|---|---|---|---|
| **A0** | Image-only, hydraulics deferred | 0% | none delivered | small | Proving the image link in the field now |
| **A1** | Best-effort discrete drive | ~0% reserved | ~1.35 s/command | large | Coarse discrete teleop; max image |
| **A2** | On-demand DRIVE/LOOK reservation | 0% parked, 50–70% driving | ~450 ms in DRIVE | large | Distinct observe-vs-maneuver phases |
| **A3** | Self-throttling reserved TDMA | ~0% idle, 15–24% active | ~450–600 ms | large + firmware | Continuous drive w/ live video, intermittent commands |
| **A4** | Hard cadence-to-deadman slice | **~24% permanent** | **<200 ms, no stutter** | largest | Smooth close-in work near people |
| **A5** | Stream-off reliable-control doctrine | 0% streaming | <100 ms stream-off | medium-large | Deliberate maneuvers with video paused |

**A0** — ship the single-radio image link now with no delivered drive; wire
the absence floor and `/api/estop` gate so the safety story is ready the
moment actuators appear.

**A1** — `S4 + L1 + F1/F2 + E3 + I2`. Opportunistic discrete commands, safe by
absence, stutters under loss.

**A2** — `S1 + L2 + F1/F2/F4 + E3 + I2`. Shrink the encode while driving to
raise window cadence; full video returns when parked.

**A3** — `S1 + S2 + L4 + F1/F2/F4 + E3 + I2`. A genuinely armed known-phase
window plus control-first serving. Adds the RXCONT firmware fix.

**A4** — `S1 + S2 + S3 + L3 + F1/F2 + E3 + I2`. The only architecture that
meets the 200 ms deadman with *delivered* refresh, and it pays ~24% image for
it permanently.

**A5** — pause the stream to get continuous-listen control; best-effort while
streaming. The honest contract over half-duplex physics, and the **mandated
fallback** if in-stream delivery proves unsolvable.

### Hard dependencies and impossible combinations

- **Every** delivered-drive architecture (A1–A5) is gated behind **I2
  (Route B)**. Route A composes with nothing.
- **No actuation opcode** may go on the link without F6, and F6 needs RS-8.3
  `boot_ctr` closed first.
- **E-stop by absence is real only after** Route B lands the 200 ms deadman
  into the Method G build **and** `/api/estop` gates the base publisher.
- **Any armed-gap or duty-ceiling scheme is a no-op without the RXCONT fix** —
  the 4.98 ms gaps are deaf by construction. A0–A2 rely on the once-per-train
  boundary (armed today); A3–A4 need the firmware fix.
- **"No reservation + no stutter + 200 ms refresh" does not exist.** A4 pays
  the ~24% tax, A1–A3 accept stutter, A5 trades video.
- **Relaxing `CONTROL_TIMEOUT_MS`** is a safety-case renegotiation, not a
  tuning knob — it breaks the SAFETY_CASE §4 three-chain agreement and the H1
  <250 ms test. Prefer the ramp-down (F4).

---

## 4. No-regrets work — correct under every surviving architecture

Do these first, in this order. None of them commits to an architecture.

1. **Fix the contaminated `REQ_KEYFRAME` observable.** `image_rx_daemon.py:1064`
   clears a pending request on *any* `frame_kind == 1`, so routine keyframes
   false-ack requests that never arrived. **Prerequisite to trusting any
   further delivery measurement** — this is what produced the retracted 171/1.
2. **Run-J bisection A/B.** Toggle exactly one of `parity_group` /
   `TX_PREPARE_AHEAD` / `train_gap_ms` per run, scored **tractor-side** on
   `LoRa cmd:` truth. Zero code, ~15 min, may recover a known-working 58%.
3. **Fix the `/api/estop` publisher gate** (F3). Harmless today; a hard
   prerequisite the moment any drive exists.
4. **Debuggability floor** (I4). Counters and faults instead of silent drops.
5. **Fix SRC_BASE arbitration** (I3) — base must emit heartbeat + control.
6. **Close RS-8.3 `boot_ctr`** and stand up the minimum auth posture (F6).
7. **Settle the envelope as GCM-128** (E3). Correct under every architecture.

---

## 5. The decisive measurement

**RS-0.12 phase-swept delivery run.** At DTS saturation, transmit a real 26 B
control frame at a *commanded* offset from each train boundary — sweeping the
offset in ~20 ms steps across the full train period — and count tractor-side
receptions per bin. Instrument the tractor to log a monotonic timestamp when
RXCONT is actually re-armed (`image_tx_daemon.py:1073`) and when the next
train's first fragment is submitted; that pair *is* the armed window, measured
rather than inferred. Stream it back so both clocks tie together (cross-board
skew is demonstrably ≥50 ms).

One five-minute run yields **P(delivery | phase)** and settles the decision:

- **Best-bin P comfortably high** → A3/A4 buildable; reserved slice and the
  200 ms deadman are achievable for ~24% image.
- **Best-bin P never exceeds ~60%** → in-stream delivered drive at hydraulic
  cadence is unsolvable; **A5 (stream-off doctrine) is the answer.**

Two cheap add-ons make it conclusive: record `LIFETRAC_ALIGNED_PUMP` and the
three RS-3.10 knobs in `params.txt`, and log a raw gap histogram instead of
median/p95 (`image_rx_daemon.py:1139-1145`, two lines).

---

## 6. Recommendation

**No defensible commitment to a delivered-drive architecture exists today.**

Ship **A0** now — the single-radio image link works and is at its measured
best. Do the **no-regrets work** in §4, which is correct regardless and
includes two verified defects (`/api/estop` gate, SRC_BASE arbitration) that
would otherwise make any future drive silently inert. Then run the
**phase-swept measurement** and let its one number choose between A3/A4 and
A5.

Three things are already settled and need no further analysis:

- **Envelope: GCM-128.** Cadence, not frame size, is the constraint.
- **E-stop: by absence,** with engine-kill as a slow retried latch. The
  safety-critical stop must never depend on a delivered packet on a link
  measured at ~0% in-stream delivery.
- **Route B is unavoidable** for any delivered drive, and it is larger than
  it first appears — it must also restore the M4 seqlock feed and Opta
  watchdog tick that live only in the compiled-out loop.

---

## 7. Provenance

Generated from a 7-agent analysis (5 enumeration lenses → consolidation →
adversarial kill-test), with every load-bearing claim independently verified
against the code by the author before inclusion. Underlying measurements:
[`CONTROL_PLANE_REANALYSIS_2026-07-27.md`](bench-evidence/RS_throughput_rebaseline_2026-07-25/CONTROL_PLANE_REANALYSIS_2026-07-27.md).
Related TODO items: RS-0.12 (measurement), RS-8.3/8.6 (crypto), RS-9.1/9.2/9.5
(single-radio ownership, debuggability).

Claims in this document marked "verified" were checked directly at the cited
`file:line`. Claims about *future* behavior (image cost percentages, achieved
latency per architecture) are **derived estimates** from the measured geometry,
not measurements — the phase-sweep is what converts them into facts.
