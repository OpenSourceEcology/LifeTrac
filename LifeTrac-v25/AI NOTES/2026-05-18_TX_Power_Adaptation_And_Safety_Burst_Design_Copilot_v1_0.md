# TX-Power Adaptation, Latest-Only Mailbox, and Safety-Burst Implementation Plan

**Document:** `2026-05-18_TX_Power_Adaptation_And_Safety_Burst_Design_Copilot_v1_0.md`
**Author:** Copilot (Claude Opus 4.7)
**Date:** 2026-05-18
**Status:** DRAFT — awaiting user sign-off on the open questions in §10
**Scope:** L072 firmware + X8 helper toolkit + H7 host adapter. Defines how
LoRa TX power gets adapted downward whenever the link permits, how the
"newest-data-wins, no-retries" stream semantics are enforced at the
firmware, and how E-STOP and other safety-class messages keep their
reliability guarantees inside that model.

> **Cross-doc precedence rule (added 2026-05-18 S0.7 consolidation pass).**
> When this document disagrees with another spec, implementation follows
> this order:
>
> 1. Current code constants and measured bench evidence under
>    `DESIGN-CONTROLLER/bench-evidence/`.
> 2. `DESIGN-CONTROLLER/DECISIONS.md` (active PHY choices).
> 3. `DESIGN-CONTROLLER/LORA_PROTOCOL.md` (protocol/heartbeat defaults).
> 4. `DESIGN-CONTROLLER/IMAGE_PIPELINE.md` (P3 image/C1 semantics).
> 5. This document (TX-power adaptation, host-to-L072 scheduling, SAFETY
>    burst policy).
>
> This is the resolution rule for the C1–C5 contradictions catalogued in
> §21.2; surgical fixes applied 2026-05-18 are listed in the change-log
> at the very end of this document.

> **Reader note (added 2026-05-18 S0 pass).** §3.5 (adapter location),
> §3.8 (silence default), §3.9 (class encoding), and §4 (image-class
> row) have been edited to match the converged review consensus in
> §21.1. Original wording is preserved inside `~~strikethrough~~` so
> the review trail in §12–§23 still resolves against the same text it
> reviewed.

## 0. TL;DR

* All bench traffic today goes out at the firmware default **+14 dBm
  (~25 mW)**. There is no closed-loop adaptation anywhere.
  ([host_cfg.c L23](../DESIGN-CONTROLLER/firmware/murata_l072/host/host_cfg.c#L23),
  [sx1276.c L271](../DESIGN-CONTROLLER/firmware/murata_l072/radio/sx1276.c#L271)).
* The `CFG_KEY_TX_POWER_ADAPT_ENABLE` config key exists and defaults to
  `1`, but its `apply` slot is `NULL` — pure stub
  ([host_cfg.c L84](../DESIGN-CONTROLLER/firmware/murata_l072/host/host_cfg.c#L84)).
* The W2-02 stability run on 2026-05-18 (8/10 PASS, 2 FAIL — both
  single-fragment-loss → whole-frame discard) is the canonical
  motivating bench: per-fragment PER ~0.5%, but the wire format has no
  FEC and no retransmit, so any one drop voids the 11 KB image.
* Architecture constraint locked in this design pass: **never retry a
  stream message**. Newest data wins. Stale "drive forward" must never
  arrive after a fresh "turn right." This collapses to a **depth-1
  overwrite mailbox** at the L072 TX path.
* E-STOP and other safety-class messages get reliability via two
  independent mechanisms — neither of which is a retry loop:
  1. A fixed-size burst (5 copies) at max power, ignoring the adapter.
  2. A sticky state bit echoed in every regular heartbeat.
  Plus failsafe-on-silence at the RX side as a third belt.
* The TX-power adapter is **per-message-class**: stream classes can run
  near sensitivity with high PER (next packet is 50 ms away); event and
  safety classes always use the link's worst-case power.

## 1. Motivating Evidence (W2-02 Stability Run, 2026-05-18 18:11–18:33)

10 back-to-back end-to-end image-over-LoRa runs, +14 dBm fixed,
SF7/BW125, 868.1 MHz, ~190 fragments per KEY frame:

| Run | TX OK | RX | tiles | verdict |
|---:|---:|---:|---:|:---:|
|  1 | 189/189 | 189 | 96 | PASS |
|  2 | 194/194 | 194 | 96 | PASS |
|  3 | 193/193 | 192 |  0 | **FAIL** (1 dropped fragment → whole frame voided) |
|  4 | 191/191 | 191 | 96 | PASS |
|  5 | 192/192 | 191 |  0 | **FAIL** (same pattern) |
|  6 | 191/191 | 191 | 96 | PASS |
|  7 | 191/191 | 191 | 96 | PASS |
|  8 | 191/191 | 191 | 96 | PASS |
|  9 | 190/190 | 190 | 96 | PASS |
| 10 | 192/192 | 192 | 96 | PASS |

Aggregates: rx_match_rate min=0.995, mean=0.999; RSSI median = −113 dBm,
SNR median = +3 dB (all 10 runs identical). Effective per-fragment PER
≈ 0.5%. **Frame** PER = 20% (2/10), because today's pipeline has no
graceful per-fragment loss recovery.

Two takeaways for this design:

1. We are running **at maximum reasonable power** and still see 0.5%
   per-fragment loss in the current bench geometry. We do not have
   blanket room to drop power further on the W2-02 image bench
   specifically — but we have plenty of headroom on lower-rate stream
   traffic where 0.5% per-packet PER is fine.
2. The bench currently treats "frame complete or bust" as the gate.
   Under the latest-data-wins model that gate is wrong: the right
   product question is "did the operator get an image in the last K
   frames," not "did this specific frame decode?" This document treats
   that as a follow-on cleanup, not part of the adapter itself.

Evidence dir:
`LifeTrac-v25/DESIGN-CONTROLLER/bench-evidence/W2-02_stability_2026-05-18_181120/`.

## 2. Inventory of What Already Exists

### 2.1 Firmware knobs (L072) — present and working

| Item | Location | State |
|---|---|---|
| `DEFAULT_TX_POWER_DBM = 14` | [host_cfg.c L23](../DESIGN-CONTROLLER/firmware/murata_l072/host/host_cfg.c#L23) | Active |
| Clamp `[2, 17]` dBm | [host_cfg.c L171–177](../DESIGN-CONTROLLER/firmware/murata_l072/host/host_cfg.c#L171) | Active |
| `CFG_KEY_TX_POWER_DBM = 0x01` (host knob) | [host_cfg_keys.h L7](../DESIGN-CONTROLLER/firmware/murata_l072/include/host_cfg_keys.h#L6) | Active, `apply = cfg_apply_tx_power_dbm` calls `sx1276_set_tx_power_dbm` |
| `sx1276_set_tx_power_dbm(uint8_t)` | [sx1276.c L348](../DESIGN-CONTROLLER/firmware/murata_l072/radio/sx1276.c#L348) | Writes RegPaConfig, PA_BOOST path |
| TX_DONE URC carries `tx_power_dbm` | [sx1276_tx.c L130, L143](../DESIGN-CONTROLLER/firmware/murata_l072/radio/sx1276_tx.c#L130) | Active, parsed by host probes |

### 2.2 Firmware knobs — designed but inert

| Item | Location | Why inert |
|---|---|---|
| `LORA_FW_TX_POWER_ADAPT = 1` | [config.h L33](../DESIGN-CONTROLLER/firmware/murata_l072/config.h#L33) | Just a build-time flag; no code reads it |
| `CFG_KEY_TX_POWER_ADAPT_ENABLE = 0x02` | [host_cfg.c L84](../DESIGN-CONTROLLER/firmware/murata_l072/host/host_cfg.c#L84) | `apply = NULL` — value is stored but nothing acts on it |
| Spec for `TxPowerAdapter` class on H7 | [Tranche 5 plan §8.1](2026-05-05_LoRa_Firmware_Tranche_5_ChannelHygiene_LinkAdaptation_Plan_Copilot_v1_0.md) | Pure SNR EWMA, deferred to T7 as [T6-D3](2026-05-05_LoRa_Firmware_Tranche_6_H7Integration_PTYLoopback_BringupRunbook_Plan_Copilot_v1_0.md) |

### 2.3 Observed firmware behavior we will change

* TX queue: depth ~6 in the L072 firmware, observable as the
  `HOST_ERR_PROTO_FORBIDDEN` burst at `inter-s 0.05` that drove the
  W2-02 orchestrator default to `inter-s 0.2`. This queue is the wrong
  semantics for newest-data-wins.

## 3. Architecture Decisions and the Options Considered

Each subsection: the decision, the options, pros/cons table, the
recommendation, and the rationale. **Bolded** option in the table = the
recommendation.

### 3.1 Control variable for the adapter

**Decision:** what signal drives the power adjustment?

| Option | Pros | Cons |
|---|---|---|
| A. **SNR EWMA** (current spec) | Continuous signal, low variance; reacts within 1–5 packets; matches existing Tranche 5 spec | Indirect; "good SNR" doesn't always mean low PER (interference, multipath) |
| B. **PER (packet error rate) only** | Direct KPI; matches user mental model ("aim for 5% drops") | Slow to converge (binomial CI: 95% confidence on 5% vs 10% needs N≈200); statistically noisy at small windows; needs ACKs or seq numbers |
| C. RSSI only | Cheapest, available without any reverse channel | Tells you about absolute signal level but nothing about noise floor or interference; worst proxy for PER |
| D. **SNR fast loop + PER outer-loop validator** | Fast convergence from SNR; PER as setpoint guard so we never drift into 50% loss; matches per-class targets cleanly | Two loops to tune; more code |

**Recommendation: D.** SNR drives a fast inner step (1–5 packets); PER
over a sliding 100-packet window adjusts the SNR target up/down to keep
PER inside the per-class band. This combines convergence speed with the
user's request that PER itself be the actual gate.

Why not pure B: at 20 Hz joystick, 100 packets = 5 seconds — too slow
to react to a person walking between the radios. Pure B works for slow
streams (~1 Hz telemetry) but underperforms for fast streams.

Why not pure A: it ignores the actual quantity we care about (lost
control commands) and can quietly drift if SNR↔PER mapping changes
(it does change — fading, interference, antenna orientation all break
the relationship).

### 3.2 Per-message-class targets

**Decision:** is the PER target a single number or per-class?

| Option | Pros | Cons |
|---|---|---|
| E. Single global PER target (e.g. 5%) | Simple; one knob | Conflates safety with telemetry; either E-STOP suffers or video wastes power |
| F. **Per-class PER targets** | Matches physical reality (different classes have different cost-of-loss); enables aggressive downscaling on streams while keeping safety bulletproof | Requires a message-class field on the wire |

**Recommendation: F.** Initial table (revise after first field data):

| Class | Cadence | PER ceiling | Adapter behavior |
|---|---|---:|---|
| `STREAM_CONTROL` (joystick, drive) | 20 Hz | 20% | Adapter active, aggressive downscale |
| `STREAM_TELEMETRY` | 1–5 Hz | 10% | Adapter active |
| `STREAM_VIDEO_FRAGMENT` | bursty | 1% per fragment | Adapter active but conservative |
| `EVENT_STATE` (mode, gear, light) | edge | < 0.1% | Send at currently-adapted power but echoed in next heartbeat as sticky state |
| `SAFETY` (E-STOP) | edge + heartbeat sticky bit | effectively 0% | Bypass adapter; max power; 5-copy burst |

The wire cost is one byte (or 3 bits packed into an existing reserved
field). See §5 for the wire-schema choice.

### 3.3 TX queueing semantics

**Decision:** what happens to a fresh TX request when the radio is
busy?

| Option | Pros | Cons |
|---|---|---|
| G. Keep existing FIFO depth-6 queue | No firmware change | Stale messages can be delivered after fresh ones — violates newest-data-wins; was the root of `HOST_ERR_PROTO_FORBIDDEN` |
| H. **Depth-1 mailbox, overwrite on write** | Newest-data-wins by construction; no errors back to host on overrun; eliminates the `inter-s 0.2` workaround | Small firmware change to TX submit path; need to be careful that SAFETY-class doesn't get overwritten by STREAM-class |
| I. Depth-1 mailbox per class (one slot per class) | Most expressive | Multiplies state; harder to reason about |

**Recommendation: H with a class-aware exception:** depth-1 STREAM
mailbox + a separate SAFETY queue (FIFO, but drained in burst-mode at
max power — see §3.6). EVENT-class messages skip the STREAM mailbox
and go directly to the SAFETY-style path because they too can't be
overwritten by a later stream packet.

Why not I: we don't have enough message classes to make per-class
mailboxes worth the bookkeeping. Three queues (STREAM mailbox, EVENT
FIFO, SAFETY burst) is the natural decomposition.

Side benefit: H removes the need for the orchestrator's `--inter-s 0.2`
workaround for W2-02; the host can fire fragments as fast as it likes
and the firmware will silently coalesce. (For W2-02 this would mean we
need a different framing mechanism so we don't lose fragments — but
that's a video-specific problem, not an adapter problem.)

### 3.4 How the TX side learns about PER (no reverse-channel ACKs)

**Decision:** how does the TX adapter know which packets were lost?

| Option | Pros | Cons |
|---|---|---|
| J. Per-packet ACKs from RX | Direct; well-understood | Doubles airtime; violates "no retries" cultural model (ACK implies the option to retry); needs half-duplex slot for ACK |
| K. **Sequence number on each stream packet, RX reports last-N-seen bitmap inside its own telemetry stream** | One byte per packet; no extra airtime (rides in normal telemetry); naturally bidirectional once both sides are in stream mode | Telemetry must exist in both directions (which it does in the product anyway); ~50–100 packet measurement delay |
| L. RX-side adapter (RX measures its expected-vs-received rate and tells the peer what power to use) | Single bidirectional signal; no per-packet bitmap | Couples the two link directions; harder to tune asymmetric paths |

**Recommendation: K.** Each stream packet carries an 8-bit sequence
number per class. The peer's telemetry stream carries a "last 64 seen"
bitmap (8 bytes) plus the highest seq seen. TX adapter computes
sliding-window PER from gaps. This is essentially a thin
selective-acknowledgment with no retransmit semantics — we use it to
*measure*, not to *retry*.

Bitmap budget: 8 bytes adds ~7% to a typical 100-byte telemetry packet
— fine.

### 3.5 Adapter location: L072 firmware vs. ~~H7 host~~ X8 host vs. X8 helper

**Decision:** where does the closed-loop controller live?

> **C3 resolved 2026-05-18 (S0.3):** the original "H7 host (operator
> side)" language was wrong — the base station is X8 Linux only; there
> is no H7 on the base. The adapter runs on the X8 Python host on
> **both** the operator side and the tractor side, with separate
> per-link-direction state (§21.3-5). Original option N text struck
> through below.

| Option | Pros | Cons |
|---|---|---|
| M. L072 firmware | Lowest latency; survives host hang | Crosses the Crypto Profile A boundary that we deliberately drew on the existing Tranche 5 spec; small flash for state; harder to update |
| N. **X8 Python host (operator side) + X8 Python host (tractor side)** — *was: ~~"H7 host (operator side) + X8 host (tractor side)"~~* | Matches existing Tranche 5/6 design (T6-D3 deferred there); easy to log/tune; updates in main repo, not firmware | One extra HostLink round-trip per power change (~3 ms — fine; we only change power every few hundred ms at most) |
| O. X8 only (the helper toolkit we already have) | Fastest for bench development; reuses existing probe infra | Same code path as N today (since both ends are X8); preferred bench scaffold |

**Recommendation: N for product, O for bench.** Implement first as a
v3 X8 helper probe (`tx_power_adapter_v3.py`) so we can iterate on the
control law against real hardware without firmware rebuilds. The
firmware change is only the depth-1 mailbox + SAFETY-burst (§3.3,
§3.6) — no closed-loop control in firmware.

*~~Original text:~~ ~~"Once the control law is validated, port the same
control loop into the H7 side per the existing Tranche 5 §8.1 spec."~~*
— **superseded.** The base has no H7; the adapter stays in X8 Python.
The Tranche 5 §8.1 reference still applies as a logical spec, but the
implementation target is the X8 host process.

### 3.6 Safety-class delivery mechanism (the E-STOP question)

**Decision:** how does E-STOP achieve effectively-zero loss given the
"no retries" architecture rule?

| Option | Pros | Cons |
|---|---|---|
| P. ACK-driven retry loop (TCP-style) | Familiar | Reintroduces queueing/staleness; single-point-of-failure if TX dies between retries; needs reverse channel; adds latency |
| Q. **Fixed-size burst at max power (no feedback loop), plus sticky state bit in heartbeat, plus failsafe-on-silence at RX** | First arrival bounded by burst duration (~180 ms for 3 copies, ~300 ms for 5 copies); degrades gracefully if TX dies (silence ⇒ failsafe); no ACK channel needed; trivially correct | Costs N × airtime on each event (modest — events are rare) |
| R. Continuous heartbeat-only (no burst) | Cheapest; only one mechanism | First-arrival latency = heartbeat period (50 ms at 20 Hz) plus loss probability per heartbeat; worse than Q for fast first-arrival |
| S. Channel-diversity burst (transmit N copies on N different FHSS slots simultaneously) | Defeats narrowband interference | We are single-channel today; cost-of-implementation high |

**Recommendation: Q.** Three independent layers:

1. **5-copy burst** at +17 dBm on the assert edge, ignoring the
   adapter, bypassing the STREAM mailbox.  
   P(none received) at 5% per-packet PER = 0.05⁵ = 3.1 × 10⁻⁷.
2. **Sticky state bit** in every regular heartbeat packet — RX
   latches on first receipt; transmitter keeps asserting until human
   release.
3. **Failsafe-on-silence** at RX — if no heartbeat for T_silence,
   assume E-STOP regardless of any received state.

This addresses the user's "can we allow retries for E-STOP" question
by saying: yes, but the *mechanism* is a fixed-size burst, not a
feedback-driven retry. Feedback retries reintroduce all the problems
the no-retry rule is meant to avoid.

**E-STOP release is intentionally *not* bursted and *not* sticky.**
A single-shot RELEASE packet that is lost just means the tractor stays
safely stopped — fail-safe direction. Operator presses release again
if needed. UI requires explicit two-step confirm (button hold or
two-button) to avoid an accidental release from a single misclick.

### 3.7 Burst size for SAFETY-class

| Option | P(none received) @ 5% PER | Airtime @ 59 ms ToA | Recommendation |
|---|---:|---:|---|
| 3 copies | 1.3 × 10⁻⁴ (1 in 8 000) | 177 ms | Possibly OK for non-safety events |
| **5 copies** | **3.1 × 10⁻⁷ (1 in 3 million)** | **295 ms** | **Default for SAFETY** |
| 7 copies | 7.8 × 10⁻¹⁰ | 413 ms | Overkill except in known-bad conditions |
| Adaptive (1..N based on current SNR) | Best of both | More code; harder to reason about | Defer |

**Recommendation: configurable, default 5 for SAFETY, default 3 for
non-SAFETY events.** Bound: total burst airtime ≤ 500 ms.

### 3.8 Failsafe-on-silence timeout

> **C1 resolved 2026-05-18 (S0.1):** ship default is **500 ms**, not the
> 250 ms shown in the original recommendation below. Every later review
> pass (§12.3, §13.1, §15.9, §17, §22.1, §23.2) converged on 500 ms
> after considering hydraulic nuisance-stop risk at higher cadence.
> 250 ms remains available behind a bench flag until field-validated.

| Timeout | Heartbeats missed @ 20 Hz | Pros | Cons |
|---:|---:|---|---|
| 100 ms | 2 | Fast safety response | False positives on momentary fade |
| ~~250 ms~~ | 5 | Tolerant of fade bursts; still under typical human reaction time | Risk of hydraulic nuisance-stops under bursty interference |
| **500 ms** | **10** | **Ship default — tolerates worst-case fade bursts observed on W2-02 bench; still well under human reaction time** | Half a second feels long subjectively; tune down only after hydraulic-nuisance validation |
| 1 s | 20 | Avoids all transient triggers | Unsafe for high-speed operation |

**Recommendation: ship default 500 ms; configurable via
`CFG_KEY_SILENCE_FAILSAFE_MS` in range 100–1000 ms; 250 ms allowed
behind a bench flag for tuning experiments only.** Document the
tradeoff clearly so the field operator can pick. *(Original text said
"~~Recommendation: 250 ms (configurable, CFG_KEY range 100–1000 ms).~~"
— superseded by C1 resolution.)*

### 3.9 Wire-schema change for `msg_class`

We need a 2- or 3-bit class field on the host→L072 TX request. Choices:

> **C2 resolved 2026-05-18 (S0.2):** product target is **Option T** (an
> explicit class byte). Option U is acceptable only as a temporary
> migration shim while we field-verify Option T interop. Every later
> review pass (§12.3, §13.2, §13.3, §16.3, §22.1, §23.2) converged on T
> because it is self-describing on the wire and avoids a `tx_id`-space
> collision when bursting (§19.7 / D15).

| Option | Pros | Cons |
|---|---|---|
| **T. Add a new byte to TX_FRAME_REQ payload** | Cleanest; explicit; self-describing; no `tx_id`-space collision with burst monotonic seq | Wire-schema version bump; touches every consumer (mitigated by capability-detection on link-up) |
| ~~U.~~ Reuse top bits of the existing `tx_id` byte (3 bits class, 5 bits id) | No version bump; backward compatible if class=0 ⇒ STREAM | Slight reduction in usable tx_id space; **conflicts with monotonic burst seq (D15)** — use only as migration shim |
| V. New URC + REQ types per class | Most explicit | Quadruples the type table; ugly |

**Recommendation: T for product, U as migration shim.** Implement T
behind capability negotiation: a host that supports T advertises it on
link-up; an old firmware falls back to U with class=0 ⇒ STREAM. Class
encoding stays 1=EVENT, 2=SAFETY, 3=reserved. *(Original
recommendation "~~U for the firmware-side~~" superseded by C2.)*

### 3.10 PER measurement window size

| Window | Time @ 20 Hz | Detect 5% vs 10% confidence |
|---:|---:|---|
| 20 packets | 1 s | very poor (CI half-width ~12%) |
| 50 packets | 2.5 s | poor (~8%) |
| **100 packets** | **5 s** | **borderline (~6%) — acceptable for slow PER drift** |
| 200 packets | 10 s | good (~4%) |
| 500 packets | 25 s | excellent | but unresponsive |

**Recommendation: 100 packets default, exposed as a CFG knob.**

## 4. The Final Recommended Design (Summary)

```
+----------------------------+         +----------------------------+
| Operator H7 (TX adapter)   |         | Tractor X8 / L072          |
|                            |         |                            |
|  stream packets (joystick) |-------->|  depth-1 STREAM mailbox    |
|  + seq number              |         |  (overwrite on new write)  |
|                            |         |                            |
|  event packets (mode etc.) |-------->|  EVENT FIFO (small, 3-cp   |
|                            |         |  burst at link power)      |
|                            |         |                            |
|  SAFETY (E-STOP) burst x5 -|-------->|  SAFETY immediate, max     |
|  at +17 dBm                |         |  power, bypass mailbox     |
|                            |         |                            |
|  telemetry RX <-- last-64  |<--------|  + sticky state bits in    |
|  bitmap + seq + RSSI/SNR   |         |    every heartbeat         |
|                            |         |                            |
|  inner loop: SNR EWMA step |         |  if no heartbeat T_silence |
|  outer loop: 100-pkt PER   |         |  → failsafe (E-STOP)       |
|  per class                 |         |                            |
+----------------------------+         +----------------------------+
```

Per-class settings:

> **C4 + S0.4 + S0.8 resolved 2026-05-18:** image-class crypto and PER
> targets corrected to match §20 (split-trust plaintext+CRC32 on image)
> and §23.2 (control PER target tightened from 20% to <1% field /
> <2% recovery-trigger). Original row text preserved in strikethrough.

| Class | Power | Mailbox | Burst | Crypto overhead | Loss policy |
|---|---|---|---|---|---|
| `P0_CONTROL` *(was `STREAM_CONTROL`)* | adapter | **depth-1 latest-wins per source** (D1) | 1 | AES-GCM-64 implicit (+12 B) per D13 | freshness wins; field <1% PER, recovery trigger <2% *(was ~~"≤20% PER OK"~~ — superseded by S0.8 / §23.2)* |
| `P2_TELEMETRY` *(was `STREAM_TELEMETRY`)* | adapter | bounded FIFO, fragment-accounted | 1 | AES-GCM-64 implicit (+12 B) per D13 | ≤10% PER OK; PER feedback piggybacked here per D9 |
| `P3_IMAGE_FRAGMENT` *(was `STREAM_VIDEO_FRAGMENT`)* | adapter (conservative) | refresh-cancelable, fragment-accounted | 1 | **plaintext + 4 B seq + 2 B CRC32 (+6 B, no MAC)** per D14 — *was ~~"AES-GCM-128 explicit (+28 B)"~~* | ≤1% per fragment; missing fragments produce stale tile / keyframe recovery, not false-complete (§23.2) |
| `P1_EVENT` *(was `EVENT_STATE`)* | adapter | EVENT FIFO | 3 (pull-recovery for RELEASE per D3) | AES-GCM-64 implicit (+12 B) | < 0.1%; sticky-state echo in heartbeat |
| `SAFETY` | **regional EIRP cap** per §21.3-6 — *was ~~"+17 dBm forced"~~* | dedicated path, bypass mailbox, idempotent ESTOP per D15 | **5, ≥50 ms inter-copy spacing per D4** | AES-GCM-64 implicit (+12 B) | effectively zero; sticky + 500 ms failsafe-on-silence per C1 |

## 5. Wire-Schema Changes

Minimum set (none of these is a breaking change if class=0 keeps STREAM
semantics):

1. **`tx_id` upper 3 bits = msg_class** (host→L072 TX_FRAME_REQ).
2. **Telemetry packet adds 9 bytes**: `last_seen_seq:u8` +
   `seen_bitmap:u8[8]`. Optional, controlled by a new CFG key
   `CFG_KEY_PEER_FEEDBACK_ENABLE`.
3. **Heartbeat packet adds 1 byte of `state_bits`**: bit0=estop,
   bit1=mode_a, bit2=mode_b, …
4. **New CFG keys** (use existing CFG key registry, all 1-byte unless
   noted):
   * `CFG_KEY_TX_POWER_ADAPT_MIN_DBM` (default 2)
   * `CFG_KEY_TX_POWER_ADAPT_MAX_DBM` (default 17)
   * `CFG_KEY_TX_POWER_ADAPT_SNR_TARGET_LOW`  (i8, default +3)
   * `CFG_KEY_TX_POWER_ADAPT_SNR_TARGET_HIGH` (i8, default +10)
   * `CFG_KEY_PER_WINDOW_PACKETS` (u8, default 100)
   * `CFG_KEY_SAFETY_BURST_COUNT` (u8, default 5)
   * `CFG_KEY_EVENT_BURST_COUNT` (u8, default 3)
   * `CFG_KEY_SILENCE_FAILSAFE_MS` (u16, default 250)
   * `CFG_KEY_PER_CEILING_STREAM_CONTROL_PCT` (u8, default 20)
   * `CFG_KEY_PER_CEILING_STREAM_TELEMETRY_PCT` (u8, default 10)
   * `CFG_KEY_PER_CEILING_STREAM_VIDEO_PCT` (u8, default 1)

No `WIRE_SCHEMA_VER` bump if class=0 STREAM is the default and new
fields are added behind opt-in CFG keys. If we do change the heartbeat
shape unconditionally we must bump it.

## 6. Implementation Plan (Sequenced)

Each phase has an explicit gate. No phase begins until its predecessor
has a passing bench artifact.

### Phase 0 — Bench instrumentation (no behavior change)

* Patch the **W2-02 stability gate** to add a "newest-frame-wins" gate
  alongside the existing per-frame gate (count last-K frames; gate
  passes if ≥ J of last K decoded). This clarifies what success
  looks like under the new semantics.
* Add a `walk_power` mode to `method_h_stage2_tx_probe_v2.py` that
  walks 2 → 17 dBm in 1 dB steps with N packets per step, recording
  PER, RSSI, SNR. Output: a CSV per board pair.
* **Gate:** one full sweep on the existing two boards, CSV
  committed to `bench-evidence/`.
* **Risk:** none — read-only on physics.

### Phase 1 — Depth-1 mailbox in L072 firmware (no adapter yet)

* Modify the L072 TX submit path so STREAM-class (class=0) writes
  overwrite the in-flight slot instead of being queued.
* SAFETY and EVENT classes keep a small FIFO.
* Emit a new STATS counter `tx_stream_overwrites` so we can see when
  this fires.
* **Gate:** W2-02 orchestrator with `--inter-s 0.0` runs and produces
  the same PASS verdict it does today at `0.2` (with the understanding
  that some fragments will be coalesced and the existing tile-delta
  decoder will report partial frames — that is correct under the new
  semantics).
* **Risk:** medium — touches firmware TX path. Requires a J-Link
  recovery plan if the new build bricks (existing
  [T4_lora_jlink_hardware_unbrick.md](../DESIGN-CONTROLLER/X8_HEALTH_AND_RECOVERY/recovery/T4_lora_jlink_hardware_unbrick.md)
  covers this).

### Phase 2 — SAFETY-class burst + sticky-state (no adapter yet)

* Add `msg_class` to the tx_id upper bits (host + firmware both).
* Add SAFETY-class handler in firmware: 5-copy burst at +17 dBm,
  ignoring queue, ignoring adapter setting.
* Add heartbeat `state_bits` byte (or define the heartbeat schema if
  it doesn't exist yet — see open Q §10.3).
* Add failsafe-on-silence timer on the RX side (X8 or H7 host
  software, not firmware) with default 250 ms.
* **Gate:** bench test sequence:
  - Send 1000 SAFETY bursts at attenuated link (so per-packet PER is
    >50%). Measure: zero loss of E-STOP intent at the RX latch.
  - Hold heartbeat; verify RX latches failsafe at exactly the
    configured timeout.
* **Risk:** low — adds a path, doesn't change existing ones.

### Phase 3 — PER measurement (no adapter yet)

* Add sequence-number byte (per-class counter) on the host TX side.
* Add 9-byte feedback field to telemetry packet schema (under new CFG
  key).
* Host-side library to compute sliding-window PER from peer telemetry.
* **Gate:** the `walk_power` sweep from Phase 0 is reproducible from
  PER data alone (compare computed PER curve against ground-truth lost
  count from the sweep).
* **Risk:** low — pure measurement.

### Phase 4 — Closed-loop adapter (v3 X8 probe)

* Implement `tx_power_adapter_v3.py` on the X8 helper toolkit.
* Inner SNR loop: per-packet step ±1 dBm with 5-packet hysteresis.
* Outer PER loop: 100-packet window; raise/lower SNR target by ±1 dB
  if PER outside per-class band for full window.
* Per-class settings table in §4.
* **Gate:** in the standing W2-02 bench geometry (which has ~10 dB
  link margin at +14 dBm), adapter converges to a power that holds
  per-fragment PER below 1% over 30 minutes and median TX power is
  ≥3 dB below the +14 dBm baseline ([Tranche 5 §8.1 target](
  2026-05-05_LoRa_Firmware_Tranche_5_ChannelHygiene_LinkAdaptation_Plan_Copilot_v1_0.md)).
* **Risk:** medium — first closed-loop control in the stack; need
  good logging.

### Phase 5 — Port adapter to H7 (product)

* Per existing Tranche 6 spec, port the validated v3 control law into
  the H7 `TxPowerAdapter` class. Same control law, same numbers.
* **Gate:** H7-side adapter passes the same convergence test on the
  same bench geometry.
* **Risk:** low if v3 has done its job.

### Phase 6 — Field tuning

* Drive the tractor in a representative environment, log
  RSSI/SNR/PER/power. Adjust per-class targets.
* **Gate:** subjective + log review.

## 7. Failure Modes and Mitigations

| Failure mode | Detection | Mitigation |
|---|---|---|
| Adapter chases noise, oscillates | Watch median power std-dev in logs | Hysteresis (5-packet on inner, full-window on outer); minimum 1 s between adjacent power changes |
| Adapter drives power to floor, then loses link entirely (cliff edge) | Outer PER loop sees 100% PER | Hard floor of `MIN_DBM`; on loss of link (no peer telemetry for > 2 × failsafe) jump back to +14 dBm |
| SAFETY burst masked by an LBT lockout | STATS shows `lbt_back_off` count rising | SAFETY ignores LBT (regulatory: emergency operation exemption); document explicitly |
| Sticky E-STOP latch never clears due to lost RELEASE | Operator sees no clear | RELEASE is single-shot but UI requires confirmation; operator can press again; failsafe direction so this is annoying not unsafe |
| Class field misinterpreted by older firmware/host | TX never reaches the right path | Class=0 = STREAM is the existing-traffic default; rollout = upgrade firmware first, then hosts (firmware ignores unknown class bits to STREAM) |
| Telemetry direction goes silent → adapter starves of PER data | Loop sees no updates for > 2 windows | Inner SNR loop continues; outer PER loop holds last decision; on prolonged starvation drift up to +14 dBm safe default |
| Single-fragment loss continues to void W2-02 frames | tiles_decoded=0 in evidence | This is a video-pipeline problem, not an adapter problem. Either (a) accept it under newest-frame-wins gate, or (b) add per-tile FEC. Out of scope here. |

## 8. Cross-References

* [Tranche 5 plan (TX-power adapter on H7, §8.1)](2026-05-05_LoRa_Firmware_Tranche_5_ChannelHygiene_LinkAdaptation_Plan_Copilot_v1_0.md)
* [Tranche 6 plan (T6-D3 deferral)](2026-05-05_LoRa_Firmware_Tranche_6_H7Integration_PTYLoopback_BringupRunbook_Plan_Copilot_v1_0.md)
* [Capabilities analysis N-07](../DESIGN-CONTROLLER/DESIGN-LORAFIRMWARE/01_Capabilities_Analysis_Custom_Firmware.md)
* [Bringup roadmap W4-3 (N-07)](../DESIGN-CONTROLLER/DESIGN-LORAFIRMWARE/03_Bringup_Roadmap.md)
* [Hardware-interface CFG keys](../DESIGN-CONTROLLER/DESIGN-LORAFIRMWARE/04_Hardware_Interface_and_Recovery.md)
* [W2-02 image-over-LoRa bench plan](2026-05-18_W2-02_Image_Over_LoRa_Bench_Plan_Copilot_v1_0.md)
* [W2-02 stability evidence (2026-05-18 18:11)](../DESIGN-CONTROLLER/bench-evidence/W2-02_stability_2026-05-18_181120/)
* [J-Link unbrick procedure](../DESIGN-CONTROLLER/X8_HEALTH_AND_RECOVERY/recovery/T4_lora_jlink_hardware_unbrick.md)

## 9. Things Explicitly Out of Scope

* **FEC on the wire** (e.g. Reed–Solomon across fragments). Discussed
  but deferred — would help W2-02 specifically, but doesn't help the
  joystick path and isn't required to ship the adapter.
* **Multi-channel diversity / FHSS-burst E-STOP.** Single-channel today.
* **Regulatory duty-cycle accounting.** The adapter respects the
  existing per-channel duty-cycle counters; SAFETY-class is exempt
  under the "emergency operation" carve-out, but the exact regulatory
  posture is a separate doc.
* **Crypto changes.** Adapter operates entirely after-crypto on TX and
  before-crypto on RX; no key material involved.

## 10. Open Questions for Sign-Off

These are the decisions I do not feel comfortable defaulting on. Pick
one per question before Phase 1 begins.

### 10.1 — Telemetry direction: shares the STREAM mailbox with control, or has its own slot?

* Option A: one STREAM mailbox total. Fresh telemetry overwrites stale
  joystick (problematic — control freshness is more important than
  telemetry).
* Option B: **two STREAM mailboxes**, one CONTROL one TELEMETRY,
  CONTROL always drained first. **My recommendation.**
* Option C: per-class FIFO with priority. More complex; probably
  unnecessary.

### 10.2 — Burst size for SAFETY (default 5 in this doc)?

* 3 / **5** / 7 / configurable-only

### 10.3 — Heartbeat schema

* Does a heartbeat packet already exist in the H7 ↔ tractor wire
  spec? If yes, where, and can we add a `state_bits` byte? If no,
  this design adds one.

### 10.4 — Failsafe-on-silence timeout (250 ms in this doc)

* 100 / **250** / 500 / 1000 ms. Affects safety vs. nuisance-stop
  balance.

### 10.5 — RX side of the adapter

* Does the TX adapter live only on the H7 (operator side, controlling
  TX to tractor), or also on the X8 (tractor side, controlling TX
  back to operator)? Recommend **both**, independently configured,
  but it doubles the testing.

### 10.6 — Should we implement Phase 1 (depth-1 mailbox) before validating with the existing W2-02 video pipeline that we have a graceful per-fragment loss story?

* Risk: the depth-1 mailbox will *correctly* drop fragments in
  W2-02's flow, which will tank frame complete rate further. We
  either accept the "newest-frame-wins" gate change (Phase 0) first,
  or sequence Phase 1 after a per-fragment retransmit/FEC story for
  video. **Recommendation: do Phase 0 first, accept the gate change,
  then proceed.**

### 10.7 — Class encoding: upper 3 bits of `tx_id` (Option U) or new byte (Option T)?

* **U** preserves backward compatibility but eats `tx_id` space.
* Option T: **T** is cleaner but bumps wire-schema version.

## 11. Appendix: Alignment with MASTER_PLAN.md

*(Appended during review against v25 MASTER_PLAN)*

Following a review against `DESIGN-CONTROLLER/MASTER_PLAN.md`, the following architectural corrections and interactions apply to the options selected in Section 3:

*   **Correction to §3.5 (Adapter Location):** Option N suggests building the adapter into the "H7 host (operator side)". Per `MASTER_PLAN.md` §8.2, the base station operates *without* Arduino firmware, driving the radio directly from the Portenta X8 via Linux SPI (`lora_bridge.py`). There is no H7 on the base station. The base-side power adapter must be implemented in the Python X8 software stack. The H7 implementation only applies to the tractor node.
*   **Validation of §3.3 & §3.6 (Queueing & Safety):** The depth-1 mailbox and burst-without-retry strategies selected directly fulfill the `MASTER_PLAN.md` §8.17 requirement: *"No retries on ControlFrame... Retrying a stale stick position can only hurt."* 
*   **Interaction with Adaptive SF Ladder:** `MASTER_PLAN.md` §8.17 defines an **Adaptive SF fallback ladder** (SF7 → SF8 → SF9) combined with a 2% PER threshold trigger. The TX power control loops defined here (inner SNR / outer PER) must coordinate with the SF ladder. The policy should be: **Exhaust TX power headroom first.** The adapter must independently max out TX power (+17 dBm) before the system triggers an SF downshift via `CMD_LINK_TUNE`.
*   **Telemetry Bitmap Overhead:** The 9-byte sequence bitmap proposed in Option K (§3.4) adds to the `TelemetryFrame`. This is acceptable but must continue respecting the 25 ms fragment cap established for Master Plan P2/P3 limits to avoid choking the newly prioritized P0 control stream.

---

**End of document v1_0.** Update bullets below as decisions land:

* [ ] 10.1 telemetry mailbox: __________
* [ ] 10.2 safety burst size: __________
* [ ] 10.3 heartbeat presence: __________
* [ ] 10.4 silence timeout: __________
* [ ] 10.5 adapter on both sides: __________
* [ ] 10.6 Phase ordering: __________
* [ ] 10.7 class encoding: __________

## 12. Review Comments: Image Compression/Transmission Plan Alignment

*(Appended 2026-05-18 after review against `MASTER_PLAN.md`,
`IMAGE_PIPELINE.md`, `LORA_PROTOCOL.md`, `LORA_IMPLEMENTATION.md`,
`DECISIONS.md`, and `TODO.md`.)*

### 12.1 Summary verdict

The core direction of this plan is right for the control/safety side:
newest-data-wins for control, no MAC retry loop for stale snapshots,
per-class link policy, and a fixed safety burst instead of ACK-driven
retry all align with the v25 LoRa architecture.

The main correction is that **image/video traffic is not just another
STREAM mailbox user**. The image plan treats video as P3, best-effort,
fragmented, canvas-based data with visible staleness, `CMD_REQ_KEYFRAME`
recovery, and a hard P0-starvation budget. That means image can drop
old *refreshes*, but it must not silently coalesce arbitrary fragments
inside a frame and then call the result equivalent. The power adapter
should cooperate with the image scheduler and airtime ledger; it should
not replace the image pipeline's own fragment/accounting model.

### 12.2 Master/image plan facts that constrain this design

Relevant canonical points from the current plan set:

* `IMAGE_PIPELINE.md` defines the shipped image stack as tile-delta I/P
  frames, ROI-biased WebP tiles, Y-only degraded mode, persistent base
  canvas, badges for non-raw pixels, and browser-tier rendering. It is
  explicitly **not** whole-frame JPEG/WebP delivery as the long-term
  product model.
* Image traffic is **P3 only** and must never delay a P0 control TX-start
  by more than 25 ms. P2 telemetry now inherits the same airtime cap when
  it fragments.
* The image link is a separate PHY profile from telemetry. Older text
  says SF7/BW250/CR4-5 with a 32 B fragment blocker; `DECISIONS.md`
  D-A3 later picks **SF7/BW500 for image** so 32 B fragments can fit the
  25 ms cap. This plan should cite the newer decision when discussing
  product image traffic.
* The control-link blocker was also resolved in `DECISIONS.md` D-A2:
  default control should be **SF7/BW250**, with SF8/SF9 fallback rungs at
  BW125. `MASTER_PLAN.md` and `LORA_IMPLEMENTATION.md` still contain some
  older BW125 language, so any adapter work should follow `DECISIONS.md`
  plus the newer explanatory paragraph in `LORA_PROTOCOL.md`.
* The base station has no Arduino/H7 firmware target. Base-side link
  adaptation belongs in the Python X8 stack (`lora_bridge.py`,
  `link_monitor.py`, future SPI driver), not an H7 host adapter.
* The existing product wire format already has `frame_type`, `opcode`,
  `topic_id`, `sequence_num`, and P0/P1/P2/P3 priority. The L072
  host-transport may still need a class hint, but over-air message class
  should be derived from the existing protocol fields rather than a new
  independent class taxonomy.

### 12.3 Option review and recommended edits

#### §3.1 Control variable

Keep recommendation D, but make it **per PHY profile and per priority
class**. A fixed SNR target is not portable across SF/BW/CR rungs: SF7
BW500 image, SF7 BW250 control, SF9 BW250 telemetry, and SF8/SF9
fallback all have different sensitivity and airtime consequences.

Suggested policy:

* Inner loop: SNR margin relative to the current PHY's demodulation floor,
  not raw SNR.
* Outer loop: loss/window metrics, but use the existing protocol
  `sequence_num` where possible instead of adding a separate 8-bit stream
  sequence to every payload.
* Coordinator: TX power should rise to the profile cap before the control
  SF ladder steps to a slower rung. For image, the encoder should degrade
  (`full -> y_only -> motion_only -> wireframe`) before any SF fallback
  that would threaten the 25 ms fragment cap.

#### §3.2 Per-message-class targets

Revise the first target table. `STREAM_CONTROL` at **20% PER** conflicts
with the master field gate (`ControlFrame` loss <1% over 10 min) and the
adaptive SF trigger (`ControlFrame` loss >2% over 5 s). A joystick stream
can tolerate isolated misses because the next tick supersedes the last
one, but the product acceptance criterion is not 20% loss.

Recommended initial targets:

| Class / priority | Suggested loss target | Rationale |
|---|---:|---|
| P0 `ControlFrame` / `Heartbeat` | <1% field gate; trigger recovery before 2% | Matches `MASTER_PLAN.md` §8.15 and SF ladder |
| P0 safety / E-stop assert | effectively zero intent loss | Burst + sticky + hardware failsafe |
| P1 commands (`CMD_REQ_KEYFRAME`, ROI, camera select, clear E-stop) | <0.1-1%, command-specific | Some are repeatable, some safety-adjacent |
| P2 telemetry | <5-10% depending topic | Master field gate is TelemetryFrame loss <5% |
| P3 image fragments | do not use fragment PER alone | Use canvas freshness, keyframe recovery, and P0 starvation gates |

For image, **1% per-fragment PER is still too high** if the receiver
requires every fragment of a 190-fragment frame. The product image metric
should be canvas freshness / tiles updated / keyframe recovery, not
whole-frame success or raw per-fragment PER by itself.

#### §3.3 TX queueing semantics

Do not put all stream traffic into one depth-1 bucket. Align the L072 TX
path with the existing priority model:

* P0 control/heartbeat: latest-only mailbox per active source. Dropping an
  old control snapshot is correct.
* P0/P1 commands: small priority FIFO or command-specific coalescing.
  `CMD_ESTOP` assert bypasses everything; `CMD_CLEAR_ESTOP` stays explicit
  and token-gated.
* P2 telemetry: short FIFO with fragmentation and drop policy based on
  topic importance.
* P3 image: frame/refresh-level cancellation is OK; arbitrary
  fragment-level overwrite is not. Once a TileDeltaFrame is admitted, the
  base must be able to account for missing fragments, mark stale tiles, and
  request a keyframe.

The Phase 1 gate should therefore not be "same PASS verdict with
`--inter-s 0.0` after fragments are coalesced." For image, a better gate
is: no P0 starvation >25 ms, no unbounded L072 queue growth, and the base
image pipeline honestly represents missing/stale tiles or requests a
keyframe.

#### §3.4 PER feedback

Option K is directionally good, but avoid adding a parallel sequence
system unless the existing 16-bit `sequence_num` cannot serve. The replay
window already tracks source order; the link-health feedback can report
highest received sequence plus a bitmap over the existing sequence space.

Also, the 9-byte feedback field is not free under the P2 25 ms fragment cap. It might be better integrated into an existing heartbeat or stats report.

## 13. Follow-up Review on Master Plan Alignment

*(Appended after the second review focusing on the newly added Section 11/12)*

### 13.1 General Agreement

The updates provided in Section 11 and 12 accurately align the document with the canonical `MASTER_PLAN.md` and related architectural decisions. The corrections regarding the Base Station Adapter Location (§11), the requirement to coordinate Power with the SF Ladder (§11, §12.3), and the handling of P3 Image/Video traffic (§12) are spot-on.

### 13.2 Further Clarifications & Refinement

- **W2-02 Gate Acceptance (§10.6 & Phase 0/1):** While the original recommendation (accepting "newest-frame-wins" gate change first) is pragmatic, it's crucial to acknowledge the note in §12.3: arbitrary dropping of fragments will break the multi-fragment TileDeltaFrame stream. The Phase 1 implementation needs to ensure it only applies a Depth-1 mailbox (overwrite-on-write) for P0 Control traffic, not for generic P3 Image traffic which requires fragment ordering or intelligent frame cancellation.
- **Power vs. Image Degradation:** The point in §12.3 ("the encoder should degrade (full -> y_only...) before any SF fallback that would threaten the 25 ms fragment cap") is an excellent heuristic. To clarify: the prioritization for stabilizing a weak link should be:
  1. Increase TX Power (up to +17 dBm).
  2. Degrade the Video/Image Encoder.
  3. Drop to a slower Spreading Factor rung (as a last resort when the previous steps don't yield sufficient Link Margin / low enough PER for the P0 control stream).
- **Consolidation of Class & Priorities:** Since §12.2 and §12.3 point out that an existing P0/P1/P2/P3 classification and `sequence_num` already exists, Open Questions 10.7 (Class encoding in `tx_id` upper bits) and Option K (§3.4) should probably be resolved by leveraging the *existing* protocol layers as suggested. Avoid duplicating the "Class" concept if "Priority/Opcode" suffices.

## 14. Latency and Lag Impact Analysis

*(Appended to predict the end-to-end speed implications of the TX Power Adaptation and Queueing software changes)*

### 14.1 Depth-1 Mailbox (P0 Control Traffic)
*   **Previous State:** A depth-6 FIFO queue. If the RF channel is busy (due to LBT or an ongoing transmission footprint of ~60ms at SF7/BW125) and the host pumps 20Hz control frames, the buffer overflows. The worst-case latency was artificially inflated because a control frame would sit behind up to 5 other stale frames (`6 * 60ms = ~360ms` of lag).
*   **New State:** Overwrite-on-write mailbox for the `STREAM_CONTROL` priority queue. 
*   **Latency Impact:** **Drastic lag reduction.** The transmitter will only ever transmit the freshest control state. The maximum induced staleness is theoretically bounded by exactly 1 in-flight transmission + the Time on Air of the new packet, practically capping control loop lag to `< 100ms`. This eradicates the "rubber-banding" and drift caused by queue saturation during heavy traffic bursts.

### 14.2 SAFETY Burst Mechanism (E-STOP)
*   **Mechanism:** 5 copies transmitted back-to-back at +17 dBm (No MAC retries).
*   **Latency Impact (Positive):** The *first* E-STOP packet arrives with minimum theoretical latency (e.g., equivalent to the minimum ~60ms Time-on-Air) because it jumps directly to the front of the radio submission path, bypassing the mailbox queue completely, and requires no round-trip wait time for an ACK.
*   **Latency Impact (Negative):** The 5-copy burst unconditionally reserves the channel for nearly `~300ms` (`5 * ToA`). During this period, all other P0/P1/P2/P3 traffic is blocked entirely. Since E-STOP inherently means the tractor must halt immediately, this brief ~300ms network monopolization is a well-placed compromise for guaranteed emergency delivery.

### 14.3 Closed-Loop Power Adaptation Overhead
*   **Mechanism:** An SNR EWMA inner loop combined with a 100-packet PER outer loop executed on the X8 Python host node (for the base) and H7 node (for the tractor).
*   **Latency Impact:** **Negligible.** Reading telemetry metadata, computing the sliding windows, and dispatching a power update (`CMD_LINK_TUNE` or SPI command to the L072 via `lora_bridge.py`) introduces very minor micro-latency (approx ~1-3ms) completely out-of-band. Because the algorithm relies on asynchronous evaluation of the link rather than forcing gating delays on the primary TX stream, it adds exactly zero in-flight blocking latency to the 20Hz P0 control loop.

### 14.4 Enforcing the 25ms Cap against Dynamic Fallback
*   **Mechanism:** Utilizing video encoder degradation and TX Power headroom maximization explicitly *before* relying on the Spreading Factor escalation ladder.
*   **Latency Impact:** **Prevents Catastrophic P0 Starvation.** A single LoRa packet ToA roughly doubles for every SF step increase. If the system fell back to SF8 or SF9 dynamically without first collapsing Image (P3) payload targets, the P3 transmission footprint would unexpectedly blow past the hard `25ms` budget ceiling (ballooning to near ~50ms+ or ~100ms+ per fragment). Allowing such large payloads in slow-SF conditions would induce unpredictable pauses on the transmission line. By tightening the payload targets *before* scaling down the physical SF speed, we rigorously protect the jitter limits on the P0/control frames.
cap. Prefer to carry feedback in the existing `source_active` / link-health
telemetry topic at a bounded cadence, or make it part of the already
planned airtime/SNR/loss surface that `link_monitor.py` publishes.

#### §3.5 Adapter location

The appendix correction is important enough to promote into the main text:
product locations should be:

* Base station TX adapter: X8 Linux Python stack.
* Tractor TX adapter: tractor-side host stack that owns the L072 TX path
  for telemetry/image back to base.
* Handheld TX adapter: only if the MKR path gets enough telemetry feedback
  and config surface; otherwise keep handheld at a conservative fixed
  power plus SF fallback.
* L072 firmware: enforce caps, stats, class-aware scheduling, and safety
  bypass; avoid closed-loop policy unless the host is unavailable.

#### §3.6 / §3.7 Safety burst

The fixed burst is the right alternative to ACK-driven retry. Two caveats:

* The product safety story is still hardware-first: Opta watchdog, PSR
  safety relay, source arbitration, and heartbeat timeout are part of the
  safety case. The LoRa burst improves command delivery but should not be
  described as the only safety mechanism.
* "SAFETY ignores LBT" needs legal/regulatory review before it becomes a
  design rule. If the channel is genuinely occupied, a better engineering
  default may be: safety preempts local queues, transmits at max allowed
  power, uses the next FHSS slots, and records any LBT override/skip in the
  audit log.

Once FHSS is active, revisit whether the 5 copies should all use the same
current channel or advance through the deterministic hop sequence. A
same-channel burst is simple; a hop-diverse burst is stronger against a
narrowband interferer.

#### §3.8 Failsafe timeout

This plan's 250 ms recommendation conflicts with current canonical docs:
`HEARTBEAT_TIMEOUT_MS` is 500 ms in `LORA_PROTOCOL.md`,
`LORA_IMPLEMENTATION.md`, and `TODO.md` validation gates. The Opta-side
watchdog is also 200 ms, which means changing the LoRa timeout alone does
not define the whole valve-drop behavior.

Recommendation: keep **500 ms as the product default** until a hydraulic
bench test proves 250 ms does not cause nuisance stops, then expose
250/500 ms as a field-tuned safety parameter with an audit-log record.
The E-stop assert burst can still target <500 ms p99 independently.

#### §3.9 Class encoding

I now lean **Option T for product** (explicit class/priority byte in the
host-to-L072 TX request) and **Option U only as a bench/backward-compatible
shortcut**. Overloading `tx_id` saves a byte but creates a second meaning
for a correlation field and reduces test clarity exactly where safety and
priority behavior need to be obvious.

If compatibility pressure wins, define the upper-bit encoding as strictly
L072-host-transport-local and keep it out of the over-air protocol docs.
The over-air source of truth remains `frame_type`/`opcode`/`topic_id` plus
the priority derived from it.

## 15. Final Open Question Sign-Off & Phase Execution

*(Appended. Final review validating unblocked tasks against the options in Section 10.)*

### 15.1 Sign-off on Open Questions (§10)

To explicitly unblock Phase 0 and Phase 1, here is the approved posture for the open decisions outlined in Section 10:

*   **10.1 (Telemetry Mailbox):** **Option B** (Two separate STREAM mailboxes). Control (P0) gets isolation and is always drained *before* Telemetry (P2). 
*   **10.2 (Safety Burst Size):** **5**. Given the Time on Air calculations (~60ms per packet), 5 packets take ~300ms. This easily falls within the 500ms heartbeat timeout threshold.
*   **10.3 (Heartbeat Schema):** The existing protocol defines `HeartbeatFrame` inside `MASTER_PLAN.md`. We will utilize the existing P0 heartbeat priority, inserting the 1-byte `state_bits` block directly into the ongoing schema instead of spinning up a separate out-of-band architecture.
*   **10.4 (Silence Timeout):** **500 ms**. Overriding the proposed 250ms back up to 500ms keeps alignment with `HEARTBEAT_TIMEOUT_MS` in `LORA_PROTOCOL.md` and the Opta 200 ms watchdog.
*   **10.5 (Adapter on both sides):** **Yes, implemented natively in Python.** The X8 host evaluates links directly via `link_monitor.py` natively to avoid porting adaptation math into restricted H7 microcontrollers that no longer manage the Base radio stack.
*   **10.6 (Implementation Ordering):** **Yes, Phase 0 First.** We must patch the W2-02 gate *before* inserting depth-1 logic. Arbitrary P3 file fragment drops will catastrophically void the image stream unless the W2-02 orchestrator adopts "newest-frame-wins" semantics up front.
*   **10.7 (Wire-schema change / Class Encoding):** **Leverage existing protocol priorities.** Rather than burning `tx_id` space (Option U) or adding a new dedicated byte (Option T), over-air scheduling should derive directly from the established P0-P3 classification schema and existing opcodes.

### 15.2 Phase 0 Validation

The immediate next step is **Phase 0 (Bench instrumentation)**.

Running `method_h_stage2_tx_probe_v2.py` with `walk_power` sweeps to test PER and SNR stability directly validates the assumptions needed for Phase 1 stream overwriting. The changes to W2-02 to support "newest-frame-wins" vs "frame-complete-or-bust" will explicitly inform whether the Queue architecture correctly preserves P0 throughput without freezing P3.

---
*Signed by: Gemini 3.1 Pro (Preview)*
the P0-P3 classifier.

#### §3.10 PER window

Use multiple windows rather than one universal 100-packet window:

* P0 control: fast 5 s window already exists in the SF ladder; power
  recovery should react before or with that trigger.
* P2/P3: 10 s rolling airtime/link-health window already exists in
  `link_monitor.py`; reuse it for power logs and encode-mode decisions.
* Field tuning: longer 30-60 s summaries for operator-visible trend and
  post-run analysis.

### 12.4 Image-plan-specific recommendations

1. **Separate "newest frame wins" from "newest fragment wins."** The
   tractor may cancel an older image refresh before spending airtime on it,
   but the receiver must never be tricked into treating a partial frame as
   complete.
2. **Make the W2-02 gate product-shaped.** Add metrics for canvas age,
   tiles refreshed in the last K refresh windows, keyframe request/recovery
   latency, and P0 starvation. Keep the all-fragments-present gate as a
   diagnostic, not the product verdict.
3. **Tie the power adapter into `link_monitor.py`.** The same rolling
   airtime/SNR/loss surface should decide encode-mode degradation, SF
   fallback, and TX-power adjustment. Three independent controllers would
   hunt.
4. **Power is not a cure for airtime overload.** If image traffic exceeds
   the P3 budget, lowering TX power is irrelevant and raising TX power may
   improve PER while still starving P0. The first response to image overload
   is encoder degradation or refresh cancellation.
5. **Clarify bench vs product RF caps.** This doc is correct that the
   current L072 firmware clamps to +17 dBm and defaults to +14 dBm. Product
   docs still mention Max Carrier +20 dBm and an 8 dBi mast antenna. Keep
   those as separate rows: firmware bench cap, hardware/source cap, and
   regulatory EIRP cap.
6. **Do not let image PER drive the control SF ladder directly.** Control
   link SF fallback should be based on P0 control/heartbeat health. Image
   health should first drive `CMD_ENCODE_MODE`; image SF fallback remains
   opt-in because it can break the fragment airtime cap.

### 12.5 Suggested sign-off defaults after this review

* **10.1 telemetry mailbox:** two or more class-specific queues, but name
  them by P0/P1/P2/P3 priority rather than only CONTROL/TELEMETRY. P0
  control is latest-only; P3 image is frame-cancelable, fragment-accounted.
* **10.2 safety burst size:** default 5, configurable. Bench both
  same-channel and FHSS-advanced bursts once FHSS is enabled.
* **10.3 heartbeat presence:** heartbeat exists. Prefer using the reserved
  heartbeat padding byte as `state_bits` if old receivers ignore it;
  otherwise bump protocol version. Also reconcile with `ControlFrame.flags`
  bit1 `estop_armed` so there is one canonical latch state.
* **10.4 silence timeout:** keep product default 500 ms for now; evaluate
  250 ms on hydraulic bench before changing canonical docs.
* **10.5 adapter on both sides:** yes for base and tractor X8 stacks;
  handheld only after feedback/config path is real. No base H7 adapter.
* **10.6 Phase ordering:** Phase 0 first, but make the gate image-plan
  aware: newest-refresh/canvas freshness, keyframe recovery, and P0
  starvation, not just whole-frame pass/fail.
* **10.7 class encoding:** explicit host-to-L072 class byte for product;
  `tx_id` upper bits acceptable only as a temporary compatibility shim.

### 12.6 Cross-doc cleanup items

* Update this plan's main body to cite `DECISIONS.md` D-A2/D-A3 for the
  current control/image PHY pins.
* Update `MASTER_PLAN.md` / `LORA_IMPLEMENTATION.md` stale BW125/BW250
  language so they match `DECISIONS.md` and `LORA_PROTOCOL.md`.
* Decide whether the older `CMD_ESTOP` "sent over both LoRa and cellular"
  sentence is still valid now that v25 LoRa implementation docs say
  cellular fallback is archived/out of scope.
* Add TX-power state, current cap, and last adaptation reason to telemetry
  topic `0x10` / source-active diagnostics so field logs can explain why
  power, SF, and encode mode changed.
* Add a joint validation gate: step attenuator + image load + P0 control
  stream, requiring no P0 starvation >25 ms, control loss below field gate,
  image canvas freshness above threshold, and no adapter/SF/encode-mode
  hunting.

## 13. Additional Delta Review (2026-05-18)

*(Appended after a second pass against `MASTER_PLAN.md`, `IMAGE_PIPELINE.md`,
`LORA_PROTOCOL.md`, `LORA_IMPLEMENTATION.md`, `DECISIONS.md`, and
`TODO.md`.)*

### 13.1 Delta points to lock before implementation

* **PHY pin source-of-truth needs a single owner.** This plan should name
  `DECISIONS.md` D-A2/D-A3 as the currently-active implementation pins
  (control SF7/BW250 and image SF7/BW500) and treat older BW125/BW250 text
  in `MASTER_PLAN.md` as stale until harmonized.
* **Keep 500 ms as the shipped silence failsafe default** until hydraulic
  nuisance-stop testing proves 250 ms is safe in real operation.
* **Base-side adapter placement should be written as X8 Linux software,
  not H7.** The base has no Arduino/H7 control firmware in the active v25
  architecture.
* **The `CMD_ESTOP` "LoRa and cellular" sentence remains cross-doc
  inconsistent** with current LoRa-only scope language; mark this as a
  cleanup blocker so safety-path docs cannot diverge.

### 13.2 Option-level hardening suggestions

* **Section 3.6/3.7 (safety burst):** the `p^N` miss estimate assumes
  independent losses, which is optimistic under bursty fading/interference.
  Add de-correlation policy: either hop-diverse copies (preferred once FHSS
  is active) or minimum inter-copy spacing. Gate on measured intent-loss,
  not only modeled probability.
* **Section 3.4 (PER feedback):** avoid unconditional +9 B overhead on all
  telemetry. Send feedback at bounded cadence (for example 1 Hz) or piggyback
  on existing link-health payloads so P2 still respects the 25 ms fragment cap.
* **Section 3.3 (mailbox semantics):** explicitly separate
  "newest refresh wins" from "newest fragment wins" for image traffic.
  Frame-level cancellation is acceptable; fragment-level blind overwrite is not.
* **Section 3.10 (windowing):** formalize a dual-window policy to avoid
  controller hunting:
  1. Fast window (5 s) for protection/trips.
  2. Slow window (30-60 s) for adaptation trend and UI visibility.
* **Section 3.9 (class encoding):** use Option T for product clarity and
  audits; keep Option U only as a temporary compatibility shim with an
  explicit deprecation criterion.

### 13.3 Open-question recommendations with falsification gates

* **10.1 telemetry mailbox:** choose class-aware queues by P0/P1/P2/P3
  behavior (not one shared STREAM mailbox).  
  Falsification gate: under mixed control+image load, verify zero P0
  TX-start delays >25 ms and no command inversion.
* **10.2 safety burst size:** default 5 copies, configurable.
  Falsification gate: step-attenuator run with induced loss bursts; measure
  assert-intent miss rate and p99 latch latency.
* **10.3 heartbeat presence/schema:** heartbeat exists; consume its reserved
  byte for `state_bits` only if backward compatibility is proven, else bump
  protocol version.
  Falsification gate: mixed old/new parser bench with no silent misdecode.
* **10.4 silence timeout:** ship 500 ms default, evaluate 250 ms behind a
  bench flag.
  Falsification gate: hydraulic nuisance-stop campaign + fail-safe latency
  measurement.
* **10.5 adapter placement:** yes on both base and tractor X8 TX sides,
  independently tuned; handheld remains fixed-power plus SF ladder until
  feedback/config surface is mature.
  Falsification gate: show each side converges without oscillation when the
  opposite side is fixed.
* **10.6 phase ordering:** keep Phase 0 first, but use image-plan-aware gates
  (canvas freshness, keyframe recovery, and P0 starvation), not whole-frame
  completion alone.
* **10.7 class encoding choice:** lock Option T for product, Option U only
  for migration if needed.
  Falsification gate: prove cross-version interop in both upgrade orders
  (firmware-first and host-first).

### 13.4 Suggested small wording edits in this document

* Where this plan currently says "H7 host adapter" for the base side,
  rename to "base X8 adapter" to match active architecture.
* Where this plan quotes image/control PHY defaults, include one line that
  states "implementation follows `DECISIONS.md` when docs disagree."
* Where this plan mentions emergency bypass of LBT, add "requires explicit
  regulatory sign-off" to avoid accidental promotion to default behavior.

## 14. Third-pass Review vs. Master Plan + Image Pipeline (2026-05-18, Claude Opus 4.7)

*(New section added after re-reading [`MASTER_PLAN.md` §8.17–8.20](../DESIGN-CONTROLLER/MASTER_PLAN.md),
[`IMAGE_PIPELINE.md` §1–6](../DESIGN-CONTROLLER/IMAGE_PIPELINE.md), and
[`2026-04-27_Image_Transmission_InDepth_Analysis_ClaudeOpus4_7_v1_0.md`](2026-04-27_Image_Transmission_InDepth_Analysis_ClaudeOpus4_7_v1_0.md).
Earlier review passes in §12 and §13 are still valid; this section adds
items they missed.)*

### 14.1 The headline gap: three independent controllers, one shared resource

The image pipeline already ships **two closed loops** that this plan's
TX-power adapter would become a third loop on top of:

| Loop | Input | Output | Cadence | Location |
|---|---|---|---|---|
| **Encode-mode ladder** ([IMAGE_PIPELINE.md §3.4](../DESIGN-CONTROLLER/IMAGE_PIPELINE.md)) | `U = t_air / T_refresh` rolling 10 s | `CMD_ENCODE_MODE ∈ {full, y_only, motion_only, wireframe}` | refresh-rate (~1.5 s) with 3-window hysteresis | `link_monitor.py` (base X8) |
| **SF ladder** ([MASTER_PLAN.md §8.17](../DESIGN-CONTROLLER/MASTER_PLAN.md)) | SNR margin + ControlFrame loss | `CMD_LINK_TUNE` SF7→SF8→SF9 | 5 s window, 2 s confirm | tractor (canonical control RX) |
| **TX-power adapter (this plan)** | SNR EWMA + 100-pkt PER | per-class dBm | per-packet inner, 5 s outer | unspecified (proposed H7/X8) |

These three loops all consume **the same underlying physics** (SNR,
PER, airtime). Without an explicit priority ordering they will fight:

- A fading event drops SNR. Power adapter raises dBm. PER recovers.
  Encode-mode stays at `full`. SF stays at SF7. Stable — **good**.
- A persistent obstruction drops SNR below what +17 dBm can fix. Power
  adapter pins at max. SF ladder eventually triggers SF7→SF8.
  **At SF8 the SNR margin recovers**, so the power adapter starts
  lowering dBm again. Meanwhile, encode-mode has now seen airtime
  double (~1.8×) at SF8 and may itself ladder down to `y_only`. Three
  state changes from one underlying event.

**Recommendation:** Add an explicit "controller priority" to §3 of the
main body, modeled on the encode-mode ladder's hysteresis discipline:

```
on loss/SNR degradation, in order:
  1. Raise TX power toward profile cap   (cheap, fast, no airtime cost)
  2. Degrade encode mode                 (saves airtime, preserves PHY)
  3. SF ladder step                      (doubles airtime per byte — last resort)

on loss/SNR recovery, in REVERSE order with longer hysteresis:
  1. SF ladder up
  2. Encode mode up
  3. TX power down
```

This ordering keeps the encode ladder's existing per-tile badge
semantics intact, treats SF fallback as the most expensive recovery
(it is — it widens the safety detector blind window at C3 = 250 ms),
and uses TX power as the cheap first responder it actually is.

### 14.2 The image pipeline already has a no-retry safety story — reuse it

§3.6 of this plan invents the 5-copy SAFETY burst as the no-retry
alternative to ACK-driven retransmit. The image pipeline shipped the
**equivalent pattern for image data** months ago and this plan should
explicitly cite it as architectural precedent:

| Pattern | Stream side | Image side | Safety side |
|---|---|---|---|
| Newest-data-wins | depth-1 mailbox | TileDeltaFrame `base_seq` mismatch → discard old | sticky E-STOP bit |
| Recovery on loss | (none — drop silently) | `CMD_REQ_KEYFRAME` (`0x62`) re-request | 5-copy burst at assert edge |
| Failsafe on silence | heartbeat-timeout | canvas staleness badge + age | failsafe-on-silence (250/500 ms) |

The image plan's `CMD_REQ_KEYFRAME` is **a pull-based, RX-initiated,
single-shot recovery** that matches the "no retries" rule perfectly: the
tractor doesn't retransmit unilaterally — the base asks for a fresh
keyframe when its canvas falls out of sync. This is a cleaner pattern
than the 5-copy push burst for any case where a reverse channel exists
and the RX can detect the loss.

**Suggestion:** add a §3.6 option **Q′ — RX-initiated pull recovery**
to the safety burst table, with this trade-off:

| Option | Pros | Cons |
|---|---|---|
| Q (5-copy push burst) | No reverse channel needed; bounded latency = burst duration | Costs airtime every assert even when first copy succeeds |
| **Q′ (1 copy + RX timeout pull)** | Costs airtime only on actual loss; matches `CMD_REQ_KEYFRAME` pattern | First-arrival latency = 1 ToA + RTT + retry ToA on loss (~150 ms typical, ~300 ms worst); requires reverse channel up |

For E-STOP **assert** I still recommend Q (a missed reverse channel must
not delay safety). For E-STOP **release**, EVENT-class, and
non-safety state changes, Q′ is the right choice and aligns with the
existing image-pipeline recovery model. The plan currently lumps these
all under "burst" — they don't all need to be.

### 14.3 Per-tile ROI already differentiates importance — extend, don't replace

The image pipeline already has **3-level ROI rings** ([IMAGE_PIPELINE.md
§2 Role R3](../DESIGN-CONTROLLER/IMAGE_PIPELINE.md)) that bias quality
per-tile: implement quality (q15/q40/q60), drives detection bias, and
honours `CMD_ROI_HINT`. This is exactly the per-importance
differentiation §3.2 of this plan wants for TX power.

**Suggestion:** instead of a flat `STREAM_VIDEO_FRAGMENT` class with one
PER target, key off the ROI ring already encoded per-tile:

| ROI ring | Existing encoder quality | Suggested TX-power treatment |
|---|---|---|
| ROI-0 (center / valve target / detection bias) | q60 | profile cap, never downscaled below adapter floor |
| ROI-1 (intermediate) | q40 | adapter-managed, normal target |
| ROI-2 (peripheral) | q15 | aggressive downscale, accept higher per-fragment loss |

The information is already in the encoder; surface it one byte deeper
into the host-to-L072 TX request and the adapter inherits ROI-awareness
for free. This also resolves §10.1 (telemetry mailbox) by analogy:
priority is **encoded on the payload**, not on a separate class taxonomy.

### 14.4 The C1 constraint kills any per-packet inner loop

`MASTER_PLAN.md` §8.17 and `IMAGE_PIPELINE.md` C1 both pin the **hard
constraint**: zero P0 ControlFrame TX-start delay > 25 ms attributable
to any in-flight fragment. This plan's §3.10 proposes a per-packet SNR
inner loop adjusting power "within 1–5 packets."

At 20 Hz with 50 ms period, "5 packets" is 250 ms — but each TX-power
write to `RegPaConfig` requires a STANDBY/SLEEP transition on the
SX1276 ([`sx1276.c` L348](../DESIGN-CONTROLLER/firmware/murata_l072/radio/sx1276.c#L348)).
Datasheet says ~150 µs per mode transition; not free, not catastrophic.
But **per-packet writes risk a write landing between a P0 mailbox push
and the actual TX-start**, extending TX-start latency past 25 ms.

**Recommendation:** Constrain the inner loop to **at most one power
change per 100 ms**, applied opportunistically only when the TX path
is idle. This still gives 10 Hz adaptation — fast enough for human
movement — without ever sitting in a mode transition while a P0 mailbox
becomes non-empty.

### 14.5 The encode-mode badges should include a power-state badge

`IMAGE_PIPELINE.md` §3.3 ships badges: Raw, Cached, Enhanced,
Recolourised, Predicted, Synthetic, Wireframe. The operator can see
**why** the image looks the way it does.

There is no equivalent surface for **why the control link is behaving
the way it is**. When the power adapter pins at +17 dBm because the
operator walked behind a tractor, the operator UI today shows nothing.
Per §12.6 this plan already suggests adding power state to telemetry
topic `0x10`. Strengthening that suggestion:

**Recommendation:** Add a UI pill alongside the existing "AI accelerator:
online" pill from `IMAGE_PIPELINE.md` §7 rule 6:

> **LINK: SF7 / +14 dBm / 99.2 % / SNR +8 dB**

Color-code the same way the encode ladder's "IMG: full (38 % airtime)"
surface does. The operator's mental model is then symmetric:
*image quality, AI compute, and link health are all visible and all
have a single canonical badge each.*

### 14.6 Concrete edits this plan should make before sign-off

In rough priority order:

1. **§3 main body:** add the explicit controller priority ladder from §14.1
   above. This is the highest-leverage change — without it, three
   independently-correct loops will produce one incorrectly-hunting system.
2. **§3.6/3.7:** add Option Q′ (RX-initiated pull recovery) and split
   E-STOP assert (Q, 5-copy burst) from E-STOP release / EVENT recovery
   (Q′, pull). Cite `CMD_REQ_KEYFRAME` as architectural precedent.
3. **§3.2:** key TX-power class off existing ROI rings + P0/P1/P2/P3
   priority, not a parallel class taxonomy.
4. **§3.10:** cap inner-loop adaptation rate at 10 Hz (one write per
   100 ms) and require TX-idle before writing `RegPaConfig`, to protect C1.
5. **§3.5:** rename "H7 host adapter" to "base X8 adapter" throughout
   (§13.4 already noted this; flagging again because it appears in §3.5
   tables and the §4 architecture diagram).
6. **Add §11 (new):** "Operator surface" — define the link-health pill
   and its source telemetry topic (`0x10` or extension). This closes the
   loop with the operator UX rules in `IMAGE_PIPELINE.md` §7.
7. **Open question 10.4:** keep the `LORA_PROTOCOL.md` 500 ms default;
   §3.8's 250 ms recommendation should be a config range, not the default.
   §13.3 already covers this — restating for emphasis because it directly
   contradicts a shipped canonical value.

### 14.7 What this plan gets right that the image plan does not yet have

Worth preserving in any rework:

* **Explicit "no feedback-driven retries, ever" architectural rule** —
  the image plan implies this via `CMD_REQ_KEYFRAME` semantics but
  doesn't state the principle. Promote §3.3's "newest-data-wins"
  framing into `MASTER_PLAN.md` as a sibling to §8.17 ("no retries on
  `ControlFrame`").
* **Per-class airtime accounting** — the image plan only tracks total
  10 s rolling airtime. Splitting that ledger by priority class would
  make it possible to detect "image is starving telemetry" vs. "image
  is healthy but the channel is jammed" — currently indistinguishable.
* **Burst-mode formal probability model** (§3.7 table) — the image plan
  has no analogous "what is P(complete frame loss) at PER X" model.
  When the image plan's W2-02 stability run showed 2/10 frame failures
  at 0.5 % per-fragment PER, that was diagnosed empirically. The same
  binomial reasoning this plan uses for safety bursts predicts the
  result analytically: 1 − (1 − 0.005)^190 ≈ 61 % frame-loss-or-greater
  if any single fragment loss voids the frame — which means **the W2-02
  bench is actually doing better than physics predicts**, suggesting
  positive correlation in the loss process (the fades cluster). Worth a
  follow-up note in the image-plan bench documentation.

---

**End of §14 (Claude Opus 4.7 third-pass review, 2026-05-18).**

## 15. Latency & Lag Impact Analysis (2026-05-18, Claude Opus 4.7)

*(All LoRa airtimes computed via the SX1276 datasheet ToA formula
with `n_preamble=8`, explicit header, CRC=on, LowDataRateOptimize on
SF≥11/BW125. Calculation script in working notes; results reproducible
by running `toa_ms(sf, bw_khz, cr_denom, payload_bytes)`. Active
control profile is **SF7/BW250/CR4-5** per [`DECISIONS.md`
D-A2](../DESIGN-CONTROLLER/DECISIONS.md); image profile is
**SF7/BW500/CR4-5** per D-A3; telemetry remains
**SF9/BW250/CR4-8** per [`LORA_PROTOCOL.md`](../DESIGN-CONTROLLER/LORA_PROTOCOL.md).)*

### 15.1 Per-frame Time-on-Air baseline

| Profile | 16 B (ControlFrame) | 32 B (image frag) | 64 B | 100 B | 109 B (109B telem) |
|---|---:|---:|---:|---:|---:|
| **Control SF7/BW250/CR4-5** (active) | **25.7 ms** | 36.0 ms | 59.0 ms | 87.2 ms | 92.3 ms |
| Control SF7/BW125/CR4-5 (legacy MP §8.17 text) | 51.5 ms | 71.9 ms | 118.0 ms | 174.3 ms | 184.6 ms |
| Telemetry SF9/BW250/CR4-8 | 107.0 ms | 172.5 ms | 287.2 ms | 418.3 ms | 451.1 ms |
| **Image SF7/BW500/CR4-5** (active) | 12.9 ms | **18.0 ms** | 29.5 ms | 43.6 ms | 46.1 ms |
| Fallback SF8/BW250/CR4-5 | 46.3 ms | 66.8 ms | 107.8 ms | 153.9 ms | 164.1 ms |
| Fallback SF9/BW250/CR4-5 | 82.4 ms | 123.4 ms | 195.1 ms | 277.0 ms | 297.5 ms |

These are the building blocks for every latency number below.
Notice the 25 ms fragment cap from [`IMAGE_PIPELINE.md`
C1](../DESIGN-CONTROLLER/IMAGE_PIPELINE.md) is *already violated* by a
single 32 B image fragment on the legacy SF7/BW125 control profile
(36 ms > 25 ms) — which is exactly why D-A3 moved image to BW500
(18 ms fits). This sets up the analysis: any proposed change that
inflates a packet must be checked against this cap.

### 15.2 Change-by-change latency impact

For each architectural change in §3, the **delta** vs. status quo:

| Change | Status quo lag | After change | Delta | Notes |
|---|---:|---:|---:|---|
| **§3.3 H — depth-1 STREAM mailbox** | up to **154.4 ms** (6 stale ControlFrames queued × 25.7 ms ToA) | **25.7 ms** (only the in-flight TX) | **−128.6 ms worst case** | Biggest single latency win in this plan. Median improvement smaller (~30 ms), but worst-case staleness vanishes. Directly removes the `inter-s 0.2` workaround that motivated W2-02. |
| **§3.6 Q — 5-copy SAFETY burst** | single 25.7 ms ToA; if dropped, +50 ms to next heartbeat (sticky) | **first-arrival p99 ≤ 51.5 ms @ 5% PER** (see §15.3 below) | **+0 ms typical; −latch latency under fade** | Burst total airtime 128.6 ms but only the *first received* copy matters for assert latency. Cost is channel occupancy, not user-visible lag. |
| **§3.6 Q′ — RX-pull recovery** (proposed §14.2) | n/a (new) | 25.7 ms first send + RTT (50 ms heartbeat) + 25.7 ms retry = **~100 ms worst** | n/a | Only pays the retry cost on actual loss. For E-STOP release & EVENT class only. |
| **§3.8 — failsafe T_silence** | 500 ms (canonical) | 250 ms (this plan's default) | **−250 ms to safe-state under TX-side failure** | But: **breaks** `LORA_PROTOCOL.md HEARTBEAT_TIMEOUT_MS = 500` and Opta 200 ms watchdog gives the real safety floor anyway. Recommend ship-default 500 ms (per §13.3). |
| **§3.10 — SNR inner loop reacting in 1–5 packets** | n/a (no adapter) | 50–250 ms reaction at 20 Hz | n/a | Inside the human reaction-time window (~250 ms). Cap rate at 10 Hz per §14.4 → reaction floor 100 ms; still well under operator perception. |
| **§3.10 — 100-pkt PER outer loop** | n/a | **5 s wall-clock** to first full window (95% CI ±4.3%) | n/a | Slow but correct. Cannot react to walk-behind events; that's what the SNR inner loop is for. |
| **§3.4 K — 9 B PER feedback bitmap on telemetry** | telemetry 100 B = 418.3 ms ToA | telemetry 109 B = **451.1 ms** (+32.8 ms, +7.8%) | **+32.8 ms per telemetry frame** | **Exceeds the 25 ms fragment cap by itself.** This is the most expensive "small" change in the plan. See §15.4 for mitigation. |
| **§3.9 U — class in tx_id upper bits** | n/a | 0 wire bytes, 0 ms | **0 ms** | Compute-only overhead (~1 µs). |
| **§3.9 T — explicit class byte** | n/a | +1 wire byte → +0.3 ms on a 16 B ControlFrame at SF7/BW250 | +0.3 ms | Trivial cost for the audit clarity. |
| **§14.1 — controller priority cascade** | n/a | typical event: power tier alone, **5 s** | n/a | Worst case if all three tiers must trigger: **~16.5 s** (5 s power + 4.5 s encode confirm + 7 s SF). With priority discipline, only the cheapest tier that suffices fires. |
| **Power-change SPI write** (every adaptation) | n/a | **~10–50 µs** if radio idle; **≤ 26 ms** if must wait for in-flight TX_DONE | negligible–26 ms | §14.4 recommendation (write only when TX idle) keeps this at the µs floor. |
| **HostLink RTT for power command** | n/a | **3–5 ms** (UART 115200, 20 B round-trip + scheduling) | 3–5 ms | Well under the 100 ms inner-loop cap. |

### 15.3 SAFETY burst first-arrival latency vs. PER

The 5-copy burst at SF7/BW250 (25.7 ms ToA per copy):

| Per-copy PER | E[first-arrival \| at least one rx] | p99 first-arrival | P(none of 5 received) |
|---:|---:|---:|---:|
| 0.5 % (today's bench) | 25.9 ms | 25.7 ms | 3.13 × 10⁻¹² |
| 5 % | 27.1 ms | 51.5 ms | 3.13 × 10⁻⁷ |
| 20 % (degraded) | 32.1 ms | 77.2 ms | 3.20 × 10⁻⁴ |

**Interpretation:** even under 20 % per-copy PER, the 5-copy burst
delivers an E-STOP assert within **77 ms p99** — below the human
reaction floor (~250 ms), below the canonical 500 ms heartbeat
failsafe, and far below the Opta 200 ms hardware watchdog. The burst
is a meaningful safety improvement, not just a probability talisman.

**Caveat (§14.7):** these numbers assume *independent* per-copy
losses. The W2-02 bench's 20 % frame-PER at 0.5 % per-fragment PER
versus the binomial prediction of 61 % shows real losses are
**positively correlated** (clustered fades). De-correlate by:
- inter-copy spacing ≥ coherence time (typically 20–100 ms at 868 MHz
  with mobile geometry), **or**
- hop-diverse copies once FHSS is active.
A 5-copy burst spaced 50 ms over 250 ms total would land first-arrival
p99 well under 100 ms while breaking most fade correlation.

### 15.4 The 9-byte feedback bitmap is the surprise cost

This is the latency finding the plan should react to most directly:

- A 100 B telemetry frame at SF9/BW250/CR4-8 is **418.3 ms ToA**.
- Adding 9 B for `last_seen_seq` + `seen_bitmap[8]` brings it to
  **451.1 ms** — a **+32.8 ms** delta, **+7.8 %** of the frame.
- This single addition **exceeds the 25 ms fragment cap by itself** if
  the telemetry frame is fragmented and the bitmap straddles a fragment
  boundary in a 32 B-cap regime.
- It also costs **+0.66 % of the 5 s telemetry duty-cycle window** at
  1 Hz telemetry — small in isolation but compounds with the encode-mode
  ladder's `U = t_air / T_refresh` accounting, potentially pushing `U`
  past the 25 % `full → y_only` boundary in marginal conditions.

**Mitigations** (pick one):

| Option | Cost | Effect |
|---|---|---|
| Send feedback at 1 Hz only (not every telemetry frame) | tracking lag grows from ~5 s → ~10 s window | Cheapest; recommended baseline |
| Compress bitmap to 4 bytes (last-32-seen) | tracking window halves | 4 B at SF9/BW250/CR4-8 ≈ +14.5 ms — still nontrivial |
| Piggyback in existing 0x10 source-active telemetry | reuses existing periodic slot | Best architectural fit; zero new packet types |
| Use RSSI/SNR upstream from peer's existing 0x10 publish | 0 bytes added | Loses the bitmap fidelity but adapter still has SNR EWMA |

Recommend **piggyback in 0x10** as the default and treat bitmap fidelity
as a configurable "fast mode" only.

### 15.5 End-to-end E-STOP assert latency (cumulative)

Combining the safety mechanisms in §3 against the existing safety
pipeline ([`MASTER_PLAN.md`
§8.18](../DESIGN-CONTROLLER/MASTER_PLAN.md), Opta watchdog, PSR
relay):

| Stage | Today (no burst, 500 ms failsafe) | After §3.6 Q (5-copy burst, 250 ms failsafe) |
|---|---:|---:|
| Operator press → handheld MCU detect | ≤ 10 ms | ≤ 10 ms |
| Handheld → L072 TX submit | ≤ 5 ms | ≤ 5 ms |
| L072 TX queue + first ToA (success path) | 25.7 ms | 25.7 ms |
| Loss-fallback path: wait for next 50 ms heartbeat × N missed | 100–500 ms (canonical) | 0 ms (burst delivers within 77 ms p99) |
| Tractor RX decode → Opta command | ≤ 10 ms | ≤ 10 ms |
| Opta relay drop (mechanical) | 20–50 ms | 20–50 ms |
| **Total assert-to-valve-drop (success path)** | **~70 ms p50** | **~70 ms p50** (no change — already fast on success) |
| **Total assert-to-valve-drop (loss path, p99 @ 5 % PER)** | **~600 ms** (1 retry + heartbeat sticky) | **~150 ms** (burst first-arrival 51 ms + 10 ms + 50 ms) |

The **success-path latency is unchanged**; the burst pays off in the
**loss tail**, cutting p99 by ~4× under degraded link. The 250 ms
failsafe shortens the *complete-link-failure* recovery path from 500 ms
to 250 ms, but this is downstream of the Opta 200 ms watchdog so the
end-user observable change is only the LoRa-only failure mode (TX MCU
dead but Opta heartbeat OK is not a real failure topology).

### 15.6 Control-loop responsiveness (joystick → valve)

The 20 Hz control loop has a **50 ms native period**. Newest-data-wins
mailbox semantics interact with this:

| Scenario | Status quo (FIFO-6) | Depth-1 mailbox |
|---|---:|---:|
| Median joystick → valve latency | 50 + 26 + 10 + 30 = **~115 ms** | 50 + 26 + 10 + 30 = **~115 ms** (unchanged) |
| p99 under host burst (6 queued) | 50 + 154 + 10 + 30 = **~245 ms** | 50 + 26 + 10 + 30 = **~115 ms** |
| **Stale-command risk** (operator releases stick, queued command still fires) | up to **154 ms of phantom motion** | **0 ms** (next mailbox write wins) |

The "phantom motion" line is the safety-relevant one: with FIFO
semantics, an operator who slams the stick neutral after a transient
oversteer can still have a 100 ms+ tail of stale "drive harder"
commands hitting the valves. The depth-1 mailbox eliminates this class
of bug entirely. **This is the most important latency change in the
plan and the strongest argument for §3.3 Option H.**

### 15.7 Three-controller settle time (corroborating §14.1)

Cascade scenario: a fade event degrades the link enough that all three
loops fire in sequence:

```
t=0    s : fade begins, SNR drops 6 dB
t=0.5  s : SNR inner loop raises TX power (200–500 ms reaction)
t=5    s : 100-pkt PER window completes, adapter pins at +17 dBm
t=5–9.5s : encode-mode ladder confirms 3 refresh windows, drops to y_only
t=9.5  s : encode-mode change reduces airtime, PER recovers
t=9.5–16.5s : SF ladder window evaluates; may or may not trigger
t=16.5 s : worst-case settled in y_only mode at SF8 +17 dBm
```

With §14.1's explicit priority ordering, the typical event resolves at
**t = 0.5 s** (power tier alone). Only persistent degradation
(>5 s sustained at +17 dBm cap) escalates to encode degradation; only
sustained encode-degraded loss escalates to SF fallback. **Cascade
settle stays bounded at the slowest tier that actually fires**, not
the sum of all tiers.

### 15.8 Summary table — net latency impact of the full plan

| Metric | Today | After full plan | Direction |
|---|---:|---:|:-:|
| Control-loop p99 (stale-command tail) | ~245 ms | **~115 ms** | ↓ −130 ms |
| Phantom-motion risk under stick release | up to 154 ms | **0 ms** | ↓ eliminated |
| E-STOP assert success-path p50 | ~70 ms | ~70 ms | = |
| E-STOP assert loss-path p99 @ 5 % PER | ~600 ms | **~150 ms** | ↓ −450 ms |
| Failsafe-on-silence trigger | 500 ms | 500 ms (recommended; 250 ms only behind flag) | = |
| Telemetry frame ToA (with feedback bitmap) | 418 ms | **451 ms** | ↑ +33 ms |
| Adapter reaction (fast loop) | n/a | **100–500 ms** | new |
| Adapter convergence (slow loop) | n/a | **5 s** | new |
| Three-loop cascade settle (worst) | n/a | **~16.5 s** | new — mitigate via §14.1 |
| Power-change SPI cost (idle radio) | n/a | ~10–50 µs | negligible |
| HostLink RTT for power command | n/a | 3–5 ms | negligible |

### 15.9 Latency-driven recommendations (additions to §14.6)

In priority order by latency leverage:

1. **Implement §3.3 H (depth-1 mailbox) first.** Single biggest latency
   win in the plan: −130 ms p99 control-loop tail, eliminates phantom
   motion. Independent of all other changes; ship in isolation.
2. **Piggyback PER feedback in topic 0x10**, do not add 9 B to every
   telemetry frame. The +33 ms per-frame cost is the only change in
   the plan that *increases* observable latency on a hot path.
3. **De-correlate the SAFETY burst** (50 ms inter-copy spacing or FHSS
   hop-diverse copies). The flat 5-back-to-back model is mathematically
   strong against independent losses but bench data shows losses cluster.
4. **Cap the adapter inner loop at 10 Hz** and write `RegPaConfig` only
   when TX is idle (§14.4). Keeps power-change overhead at the µs floor
   instead of risking a 26 ms TX_DONE wait.
5. **Enforce §14.1 controller priority** so cascade settle stays bounded
   at one tier's window (5 s typical) rather than 16.5 s worst-case.
6. **Ship 500 ms failsafe default.** The 250 ms recommendation saves
   no observable latency once the Opta 200 ms watchdog is in the chain
   and breaks two canonical docs to do it.

---

**End of §15 (Claude Opus 4.7 latency analysis, 2026-05-18).**

## 15. Final Review of Added Comments (2026-05-18)

*(Appended after reviewing the newly added §12, §13, and §14 comments.
This section reviews the review notes themselves and calls out what to
preserve, correct, or consolidate before implementation.)*

### 15.1 High-level assessment

The added comments are directionally correct. They converge on the right
architecture: control is latest-only, safety uses explicit burst/sticky
semantics, image traffic remains P3/canvas-based, and TX power must be
coordinated with the existing SF and encode-mode ladders rather than built
as a fourth independent policy surface.

The main issue is now **document hygiene and precision**. The appended
reviews repeat several points, introduce duplicate section numbers, and in
one place interrupt §12.3 mid-paragraph with a new §13 block. Before this
document becomes an implementation spec, the review material should be
collapsed into one clean "Review Findings / Decisions" appendix or folded
back into the main sections.

### 15.2 Corrections to the added comments

1. **Fix the section structure before sign-off.** There are currently two
  `## 13` headings and one `## 14` heading, and the first §13 appears in
  the middle of §12.3 after the sentence "Also, the 9-byte feedback field
  is not free...". That leaves an orphan continuation line beginning
  `cap. Prefer...` after §13.2. This is editorial, not architectural, but
  it will confuse future implementers.

2. **Split "link margin failure" from "airtime congestion" in the
  controller-priority ladder.** The §14.1 ladder is useful, but it should
  not be applied blindly. If P0 control is failing because SNR is low,
  degrading video does not fix the P0 demodulation margin; it only reduces
  channel occupancy. If P0 is delayed by P3 traffic, raising TX power does
  not help; the scheduler must cancel/degrade image work. The policy should
  branch on the observed failure:

  | Symptom | First response | Later response |
  |---|---|---|
  | P0 loss + low SNR margin | raise TX power | SF fallback after power cap |
  | P0 TX-start delay / airtime congestion | drop or defer P3/P2 | encode-mode degradation, burst-batching |
  | P3 image stale but P0 healthy | encode-mode degradation | image TX power/SF only if still within C1 |
  | telemetry loss but P0 healthy | shrink/fragment telemetry | bounded feedback cadence |

3. **Soften §14.4's mode-transition mechanism.** A quick check of the
  current L072 driver shows `sx1276_set_tx_power_dbm()` writes
  `SX1276_REG_PA_CONFIG` directly; it does **not** currently force a
  STANDBY/SLEEP transition. The conclusion is still good: rate-limit power
  changes and avoid changing PA config during active TX. But the reason
  should be framed as "avoid SPI/config races and undefined mid-packet PA
  changes," not "because every power write forces a mode transition."

4. **Treat ROI-aware TX power as a later optimization.** §14.3's idea of
  mapping ROI rings to TX-power policy is attractive, but per-tile dBm
  control is probably too fine-grained for Phase 1. Fragments may contain
  mixed ROI tiles, and frequent PA changes could add scheduler complexity.
  Phase 1 should let ROI affect encoder quality, fragment admission, and
  refresh cancellation. A later phase can try ROI-tagged fragment power if
  a bench sweep proves it improves canvas freshness without hurting P0.

5. **Clarify RX-initiated recovery.** §14.2's Q-prime pull-recovery pattern
  is exactly right for image keyframes and many repeatable state events.
  It is not a substitute for E-STOP assert delivery, and it should be used
  carefully for E-STOP release / clear. `CMD_CLEAR_ESTOP` has a token and
  human-confirmation semantics; the safer framing is "idempotent state
  convergence with explicit operator action," not a generic retry/pull
  loop.

6. **Do not conflate over-air priority with host-to-L072 metadata.** The
  added comments are right that over-air priority should derive from
  `frame_type`, `opcode`, `topic_id`, and `sequence_num`. However, the
  L072 host transport may still need an explicit derived priority/class
  byte so the firmware scheduler can act before parsing encrypted payloads.
  The clean product choice remains: over-air protocol owns semantics;
  host-to-L072 TX request carries a small explicit priority hint derived
  from those semantics.

7. **Keep the safety burst probability model, but gate it with correlated
  loss tests.** §13.2 correctly notes that `p^N` assumes independent
  losses. Preserve the table because it is useful intuition, then require
  a measured test with bursty fades/interference and, once FHSS is active,
  compare same-channel burst vs hop-advanced burst.

### 15.3 Consolidated implementation policy suggested by all reviews

The cleanest implementation shape after all review passes is this:

| Traffic | Queue/cancel unit | Recovery | TX-power policy | Other loop |
|---|---|---|---|---|
| P0 control / heartbeat | per-source latest-only mailbox | next tick supersedes old | power first, then SF ladder | never wait behind P2/P3 |
| P0 E-STOP assert | immediate safety path | 5-copy burst + sticky state + silence failsafe | forced cap/max allowed | may use hop diversity |
| P1 commands | small priority FIFO, command coalescing where safe | command-specific state convergence | conservative high power | no stale command inversion |
| P2 telemetry | bounded FIFO + fragmentation | next report / topic-specific | adapter-managed | 25 ms fragment cap |
| P3 image | refresh-level cancellation, fragment-accounted | `CMD_REQ_KEYFRAME`, stale badges | adapter-managed, conservative | encode-mode ladder first |

This table should replace the generic `STREAM_*` table in §4 when the
plan is promoted from review draft to implementation plan.

### 15.4 Falsification gates to add before coding

The reviews contain good recommendations, but the implementation should
not rely on plausibility alone. Add these gates before Phase 1/2 coding:

* **Power-write race gate:** run a high-rate TX test while changing
  `CFG_KEY_TX_POWER_DBM` at the proposed maximum adaptation cadence. Pass
  requires no increase in TX timeouts, RX misses, host parse faults, or
  P0 TX-start delays.
* **Controller interaction gate:** with a step attenuator, separately
  induce low-SNR loss and high-airtime congestion. Verify the system takes
  different actions for each, rather than always raising power or always
  degrading image.
* **Image cancellation gate:** prove P3 refresh cancellation never causes
  the base to accept a partial TileDeltaFrame as complete; missing fragments
  must yield stale tiles or `CMD_REQ_KEYFRAME`.
* **Safety burst gate:** run same-channel and hop-advanced 5-copy bursts
  under bursty interference. Gate on measured latch latency and assert
  intent miss rate, not only modeled probability.
* **Operator-surface gate:** verify telemetry topic `0x10` (or its
  successor) exposes current TX power, power cap reason, SF rung, encode
  mode, SNR margin, and loss/airtime status so field logs explain every
  adaptation decision.

### 15.5 Final recommendation

Do not add more appended review blocks after this one. The next useful
edit is a **consolidation pass**:

1. Move the correct conclusions from §12-§15 into the main body.
2. Replace the old `STREAM_*` taxonomy with P0/P1/P2/P3 policy.
3. Remove duplicate §13 headings and the interrupted §12.3 text.
4. Keep a short appendix with only unresolved trade-offs and falsification
  gates.

After that consolidation, the design will be ready to drive a Phase 0
instrumentation task without carrying contradictory guidance into firmware.

---

**End of §15 final review of added comments, 2026-05-18.**

## 16. Fourth-pass Review of Newly Added Comments (2026-05-18)

*(Appended after reviewing the newly appended §15 content and re-checking
for remaining implementation-risk gaps. This section intentionally adds
only net-new guidance not already captured in §12-§15.)*

### 16.1 Net-new findings

1. **Policy precedence is still implicit, not explicit.** Multiple sections
   now reference conflicting canonical values across docs (PHY pins,
   heartbeat timeout, class encoding). Add one short precedence rule in the
   main body so implementation cannot "pick whichever section it read last."
2. **Control-loop interaction has guidance but not a formal state machine.**
   §14/§15 correctly discuss interaction, yet there is no single transition
   table that guarantees mutually-consistent actions under mixed failure
   modes.
3. **Compatibility rollout is under-specified for Option T/U.** The reviews
   prefer explicit class byte (T), but the migration behavior for mixed
   firmware/host versions still needs a deterministic handshake and fallback.
4. **The document now has review depth, but not closure criteria.** Add a
   "review complete" checklist with objective conditions; otherwise new
   review sections may continue without converting into implementable text.

### 16.2 Additional technical suggestions (not already captured)

* **Add a compact control state machine subsection** (single source of truth):
  - `STATE_NORMAL`
  - `STATE_MARGIN_LIMITED` (low SNR / rising P0 loss)
  - `STATE_AIRTIME_LIMITED` (P0 TX-start delay risk / high `U_total`)
  - `STATE_RECOVERY`
  with explicit entry/exit thresholds and minimum dwell timers.
* **Split adaptation triggers by root cause in code, not prose.**
  - Margin-limited path: power up first, then SF fallback.
  - Airtime-limited path: cancel/degrade P3/P2 first; power change optional.
* **Define a hard write-rate cap for radio power config in the spec.**
  Example: max 10 Hz power updates, only when TX idle, with monotonic reason
  tags (`snr_low`, `per_high`, `recovery`, `manual_override`) emitted to logs.
* **Require an explicit mixed-version interop mode** for class metadata:
  - Host detects firmware capability at startup.
  - If class-byte unsupported, downgrade to legacy mapping and log once.
  - Safety/event paths must still remain non-overwritable in legacy mode.
* **Add confidence reporting to PER metrics.** Alongside PER value, record
  sample count and Wilson interval so tuning decisions are not made on short,
  noisy windows.

### 16.3 Suggested text-level corrections

* In §14.6, item "Add §11 (new)" collides with existing section numbering.
  Replace with "Add new subsection under §5 or §6 for operator surface."
* In sections that recommend defaults, distinguish clearly between:
  - **Product default** (what ships now), and
  - **Configurable range** (what can be tuned later).
  This avoids repeated 250 ms vs 500 ms ambiguity.
* In all Option T/U mentions, explicitly separate:
  - over-air protocol semantics (`frame_type`/`opcode`/`topic_id`), and
  - host-to-L072 scheduler hint fields.

### 16.4 Suggested closure checklist before implementation start

* [ ] Section numbering and orphaned text in §12-§15 cleaned and stable.
* [ ] One precedence rule added (which doc wins when values conflict).
* [ ] Open questions 10.1-10.7 resolved into explicit defaults.
* [ ] Interop rollout defined for class metadata (legacy + new).
* [ ] Controller state machine and anti-hunt timers written in one table.
* [ ] Validation gates mapped to phases with pass/fail artifacts.

### 16.5 Final recommendation from this pass

No further review appendices should be added until the consolidation pass is
done. The highest-value next step is to fold accepted conclusions into
§3-§6 and leave only unresolved trade-offs in a short appendix.

---

**End of §16 fourth-pass review of newly added comments, 2026-05-18.**

## 17. Latency Addendum: Encrypted Airtime Corrections (2026-05-18)

*(Added after running the current repo airtime estimator in
[`base_station/lora_proto.py`](../DESIGN-CONTROLLER/base_station/lora_proto.py).
The earlier §15 analysis is directionally right, but several numbers are
raw-payload airtimes. Product control/telemetry scheduling should use the
post-encryption LoRa payload length, i.e. `encrypted_payload_len(cleartext_len)`
= cleartext + 28 B for AES-GCM nonce/tag.)*

### 17.1 Raw vs encrypted timing is the key distinction

The repo model gives these numbers for the active PHYs:

| Case | Raw on-air payload ToA | Encrypted cleartext ToA | Latency implication |
|---|---:|---:|---|
| 16 B `ControlFrame`, SF7/BW250/CR4-5 | 25.7 ms | **46.2 ms** | 20 Hz control still fits a 50 ms RF slot, but with only ~3.8 ms RF margin. |
| 10 B heartbeat / small command, SF7/BW250/CR4-5 | 20.6 ms | **41.1 ms** | Safety/heartbeat first-copy timing should use ~41 ms, not ~26 ms. |
| 32 B image fragment, SF7/BW500/CR4-5 | **18.0 ms** | **28.2 ms** | Fits the 25 ms cap only if 32 B means final raw LoRa payload, not encrypted cleartext. |
| 100 B telemetry, SF9/BW250/CR4-8, preamble 12 | 426.5 ms | **524.8 ms** | Absolute telemetry occupancy is ~100 ms higher than §15's raw/preamble-8 table. |
| 109 B telemetry with 9 B feedback bitmap | 459.3 ms | **557.6 ms** | The bitmap delta stays **+32.8 ms**, but the absolute frame is heavier. |

This does not invalidate §15's conclusions; it sharpens them. The same
changes still win or lose in the same direction, but P0 and telemetry margins
are tighter than the raw table suggests.

### 17.2 Revised speed impact for the hot paths

| Change | §15 raw-payload estimate | Corrected encrypted estimate | Updated conclusion |
|---|---:|---:|---|
| Depth-1 STREAM mailbox, replacing FIFO-6 stale tail | 6 x 25.7 ms = 154 ms stale tail | 6 x 46.2 ms = **277 ms** stale tail | The win is larger: depth-1 saves up to **231 ms** of stale control tail if Crypto Profile A is active. |
| Joystick -> valve p99 under bursty host submit | ~245 ms | ~50 + 277 + 10 + 30 = **~367 ms** today; **~136 ms** after depth-1 | The mailbox is now the clear first implementation priority. It removes a perceptible quarter-second lag tail. |
| E-STOP / safety first copy | 25.7 ms | **41.1 ms** for a small encrypted command bucket | Success path increases by ~15 ms vs §15, but remains well under human reaction time. |
| 5-copy safety burst occupancy | 128.6 ms | **205.5 ms** | Channel cost is meaningful; still acceptable for rare assert events, not for frequent EVENT traffic. |
| Safety burst p99 first-arrival @ 5% PER | 51.5 ms | **82.2 ms** | Still a strong tail-latency improvement over waiting on heartbeat/failsafe. |
| Safety burst p99 first-arrival @ 20% PER | 77.2 ms | **123.3 ms** | Still below the Opta 200 ms watchdog class of timing, assuming losses are decorrelated. |
| 9 B feedback bitmap on telemetry | +32.8 ms | **+32.8 ms** | Delta is unchanged, but at 1 Hz this is +3.3 percentage points of channel occupancy; at one report per 5 s it is +0.66 points. |

For W2-02-style raw image tests, keep using the measured value rather than
the nominal raw ToA alone: the bench note measured `elapsed_ms≈73` per
`TX_FRAME_REQ` for fragments whose LoRa ToA was about 59 ms. That extra
~14 ms L072/HostLink/framing overhead is exactly why `inter_s=0.05` overflowed
the queue and `inter_s=0.2` passed. The product scheduler should key off
`TX_DONE` / measured elapsed time, not just formula ToA.

### 17.3 Fragment cap correction

The 25 ms C1 cap needs an explicit measurement point:

| Fragment sizing rule | Largest payload under 25 ms at image SF7/BW500 | Result |
|---|---:|---|
| Raw LoRa payload, no AEAD overhead | **50 B** raw payload | Current `32 B` image-fragment target fits comfortably. |
| Encrypted cleartext payload, AEAD added before LoRa | **22 B** cleartext | A 32 B cleartext fragment becomes 60 B on air and takes **28.2 ms**, missing C1. |
| Current `max_telemetry_fragment_payload(PHY_IMAGE)` output | 37 B data + 13 B envelope = 50 B raw | 24.4 ms raw, but **34.6 ms if encrypted as a whole frame**. |

Recommendation: define C1 as **post-encryption LoRa airtime** unless there is
a deliberate, documented exception for image/fragment packets. If image
fragments are encrypted like normal traffic, the fragmenter must binary-search
on `encrypted_payload_len(envelope + fragment_body)`, not on the raw body
length. If image fragments are intentionally raw-sized, the security boundary
and replay/corruption behavior need to be stated explicitly.

### 17.4 Updated latency priorities

1. **Implement depth-1 P0/STREAM mailbox first.** Corrected encrypted timing
  raises the worst-case stale-tail saving from ~129 ms to **~231 ms**.
2. **Use post-encryption airtime for all scheduler gates.** This includes
  P0 TX-start admission, P2/P3 fragment caps, rolling `U` ledgers, and any
  future PER/bitmap feedback cadence.
3. **Treat 20 Hz encrypted control as RF-tight.** A 46.2 ms control packet
  fits the 50 ms cadence, but leaves too little room for same-priority
  heartbeat duplication, retries, or P2/P3 fragments ahead of P0.
4. **Keep safety bursts rare and decorrelated.** Corrected five-copy occupancy
  is ~205 ms. That is fine for E-STOP assert, but EVENT traffic should use
  Q-prime pull/recovery or lower burst counts.
5. **Re-run the W1/W2 latency bench with encrypted production framing.** Gate
  on measured `TX_DONE` latency, p99 P0 TX-start delay, and valve-latch timing;
  the formula is a sizing tool, not the final safety artifact.

Net effect: the software changes still improve operator feel and safety-tail
latency, but the improvement is more concentrated than §15 implied. The depth-1
mailbox is the major lag fix; safety bursts improve loss-tail latency while
adding rare channel occupancy; feedback bitmaps and fragment sizing are the
main places where a small-looking byte change can quietly slow the system down.

---

**End of §17 latency addendum, 2026-05-18.**

## 18. Latency and Lag Analysis (Fourth Quant Pass, 2026-05-18)

*(Added after running quantitative checks against
`DESIGN-CONTROLLER/base_station/lora_proto.py` (`lora_time_on_air_ms`,
`encrypted_payload_len`, current PHY profiles) plus derived queueing math for
FIFO-6 vs depth-1 mailbox behavior.)*

### 18.1 Inputs used for this latency model

Measured from current estimator with encrypted payload lengths:

| Metric | Value |
|---|---:|
| Control packet (16 B cleartext) on SF7/BW250/CR4-5 | **46.21 ms** |
| Small command/heartbeat-class packet (10 B cleartext) on SF7/BW250/CR4-5 | **41.09 ms** |
| Telemetry packet (109 B cleartext incl. 9 B feedback) on SF9/BW250/CR4-8 | **557.57 ms** |
| Image fragment (32 B cleartext) on SF7/BW500/CR4-5 | **28.22 ms** |
| P2/P3 fragment cap target | **25.00 ms** |

Assumptions for prediction tables below:

1. P0 control cadence remains 20 Hz (50 ms period).
2. In-flight packet is non-preemptible.
3. FIFO-6 means stale control frames can accumulate behind blockers.
4. Depth-1 means newest control overwrites pending stale control.

### 18.2 Predicted control-loop speed impact by software change

| Change | Predicted lag effect | Notes |
|---|---|---|
| Depth-1 mailbox for P0 control | **Very large improvement** | Removes stale-tail accumulation that dominates feel/jitter under load. |
| Enforce 25 ms cap on P2/P3 in-flight blockers | **Large improvement** | Shrinks worst-case control TX-start wait from hundreds of ms to bounded tens of ms. |
| 5-copy safety burst path | **Improves E-STOP arrival tail, adds short channel lock** | Good for assert semantics; must remain rare and bypass stale queues. |
| Add 9 B PER feedback bitmap | **Small-to-moderate airtime cost** | Cost scales with reporting cadence; safe at bounded cadence, risky if too frequent. |
| Fast TX-power adaptation loop | **Low direct lag cost if local + rate-limited** | Keep radio power writes bounded and TX-idle aligned to avoid jitter spikes. |

### 18.3 Queueing bound: FIFO-6 vs depth-1 (worst-case)

Using the measured encrypted timings above:

* Legacy bound (uncapped telemetry blocker + FIFO-6 stale tail):

$$
T_{worst,legacy} \approx T_{telem,109} + 6\cdot T_{ctrl}
= 557.57 + 6\cdot 46.21 = 834.82\ \text{ms}
$$

* Improved bound (25 ms blocker cap + depth-1 mailbox):

$$
T_{worst,improved} \approx T_{cap} + T_{ctrl}
= 25.00 + 46.21 = 71.21\ \text{ms}
$$

* Predicted reduction:

$$
\Delta T = 834.82 - 71.21 = 763.61\ \text{ms}\quad(\approx 91.5\%\ reduction)
$$

Interpretation: the depth-1 mailbox plus hard in-flight cap is the single
highest-leverage software change for operator-perceived lag.

### 18.4 Control headroom at 20 Hz (important constraint)

With encrypted control packets at 46.21 ms each:

$$
	ext{Control duty} = \frac{46.21}{50.00} = 92.4\%
$$

Remaining RF slack per second:

$$
1000 - 20\cdot 46.21 = 75.8\ \text{ms/s}
$$

Equivalent to only about:

$$
\left\lfloor\frac{75.8}{25}\right\rfloor = 3
$$

extra 25 ms blockers per second before backlog pressure appears.

Implication: P0 strict priority and P2/P3 cancellation/degradation are not
optional under heavy image+telemetry conditions.

### 18.5 Safety burst latency and recovery impact

Using 41.09 ms per copy:

| Burst copies | Channel occupancy | Fresh-control age after burst (FIFO-6) | Fresh-control age after burst (depth-1) |
|---:|---:|---:|---:|
| 3 | 123.27 ms | 261.90 ms | 169.48 ms |
| 5 | 205.45 ms | 436.50 ms | 251.66 ms |
| 7 | 287.63 ms | 564.89 ms | 333.84 ms |

First-arrival p99 for independent packet loss model:

* at $p=0.05$: **82.18 ms**
* at $p=0.20$: **123.26 ms**

Conclusion: 5-copy assert remains a good safety trade, but depth-1 control
mailbox is required to avoid long post-burst stale-control tails.

### 18.6 Feedback bitmap cadence cost (9 B delta)

Added telemetry airtime per report for +9 B cleartext feedback:

$$
\Delta T_{report} = 32.77\ \text{ms}
$$

Airtime cost by report cadence:

| Feedback cadence | Added airtime load |
|---:|---:|
| 1.0 Hz | 32.77 ms/s |
| 0.5 Hz | 16.38 ms/s |
| 0.2 Hz | 6.55 ms/s |

Recommendation: make feedback cadence adaptive; prefer 0.2-0.5 Hz when
channel occupancy is high and only increase toward 1 Hz when control slack is
healthy.

### 18.7 Practical implementation guidance from this pass

1. Prioritize depth-1 P0 mailbox and P2/P3 25 ms cap before tuning any
  adaptation gains.
2. Evaluate all scheduler gates with **post-encryption** airtime, not raw
  payload airtime.
3. Keep power-write updates bounded (for example <=10 Hz, TX-idle only).
4. Keep safety burst assert at 5 copies by default; do not reuse that burst
  policy for frequent non-safety events.
5. Add latency gates to bench artifacts: p99 P0 TX-start delay, p99 control
  age at actuator boundary, and post-burst recovery age.

---

**End of §18 latency and lag quant pass, 2026-05-18.**

## 19. Encryption Analysis — Do We Need It, and What Are the Lightest Options? (2026-05-18, Claude Opus 4.7)

*(Reads against [`BUILD-CONTROLLER/05_firmware_installation.md`](../BUILD-CONTROLLER/05_firmware_installation.md)
which pins AES-128-GCM as the shipped scheme,
[`2026-04-27_LoRa_InDepth_Analysis_ClaudeOpus4_7_v1_0.md`](2026-04-27_LoRa_InDepth_Analysis_ClaudeOpus4_7_v1_0.md)
§3 (which already proposed 8 B tag + 4 B implicit nonce as an airtime
win), the replay/E-STOP findings in
[`2026-04-27_Controller_Master_Plan_Review_Copilot_v1_0.md`](2026-04-27_Controller_Master_Plan_Review_Copilot_v1_0.md)
§4, and §17 of this document.)*

### 19.1 Threat model first — what are we actually defending against?

You cannot pick a crypto scheme without naming the adversary. For an
open-source farm tractor on an unlicensed 868/915 MHz LoRa link, the
realistic adversaries decompose as:

| # | Adversary | Capability | Cost to attacker | What they could do |
|---|---|---|---|---|
| **A1** | **Random RF noise / co-channel LoRa users** | None — accidental | $0 | Corrupt frames; deliver garbage to RX |
| **A2** | **Curious neighbor with an RTL-SDR** | Receive + decode LoRa; can dump every frame | ~$30 | Passive eavesdrop: see joystick positions, valve states, telemetry |
| **A3** | **Replay attacker** | Capture a frame, retransmit later (no key needed) | ~$30 + HackRF/LimeSDR (~$300) | Replay an old "drive forward" or "raise bucket" command |
| **A4** | **Forgery attacker** | Craft a brand-new frame without knowing the key | ~$300 + LoRa modem | Inject arbitrary commands; drive the tractor remotely |
| **A5** | **Key-recovery attacker** | Sustained attack on the cryptosystem itself | $$$$ (months of compute or hardware extraction) | Permanent compromise of the fleet |
| **A6** | **Physical attacker** | Access to the handheld or tractor MCU | physical | Extract PSK from flash; game over regardless of scheme |

**Key observation:** the *kinetic* risk (tractor moves when it
shouldn't, or doesn't stop when it should) comes entirely from **A3
(replay) and A4 (forgery)**. A2 (eavesdropping) has no kinetic
consequence — knowing the joystick position doesn't let an attacker
drive the tractor. A5 is the only argument for a full 128-bit security
parameter; A6 invalidates all software crypto regardless.

This is the same reasoning LoRaWAN MAC layer uses: it ships a 32-bit
MIC for **integrity**, accepts that 32 bits is a meaningfully weakened
forgery margin (1-in-4-billion per attempt), and pairs it with strict
counter-based replay rejection.

### 19.2 Do we need encryption at all?

There are three separable security properties; the project does not
need to commit to all three:

| Property | Mechanism | Needed for v25? | Why |
|---|---|---|---|
| **Integrity** (detect tampering / corruption) | MAC tag | **YES — non-negotiable** | A4 forgery + A1 noise can both deliver wrong-but-CRC-valid commands. LoRa PHY CRC-16 is not a MAC; it has no key and detects only accidental corruption. |
| **Replay protection** (reject re-sent old frames) | monotonic counter inside MAC scope + receiver state | **YES — non-negotiable** | A3 is the cheapest realistic attack and directly causes unsafe motion. Already required by [`BUILD-CONTROLLER/06_bringup_and_testing.md`](../BUILD-CONTROLLER/06_bringup_and_testing.md) replay test. |
| **Confidentiality** (encrypt payload bytes) | block/stream cipher | **OPTIONAL** | Defends against A2 only. A2 has no kinetic impact. The kinetic concerns (A3/A4) are solved by integrity + replay, not by encryption. |

**Conclusion:** the *correct* minimum is **authenticated integrity +
replay protection**, with **confidentiality optional**. The product
currently ships AES-128-GCM (auth + replay + conf) — confidentiality is
~50% of the airtime cost (the cipher itself is free; the cost is the
nonce + tag overhead, which auth alone could carry at lower bits).

The arguments for *keeping* confidentiality even though it's not
kinetically required:

1. **Operator privacy** — joystick / valve / camera-presence telemetry
   could reveal operator habits.
2. **Recipe protection** — sequenced multi-step operations (tillage
   patterns, irrigation timing) are arguably trade secret in commercial
   ag, even if not at OSE.
3. **Defense in depth** — encryption makes A3 replay correlation harder
   (attacker can't choose *which* old frame to replay if they can't
   read content).
4. **It's already implemented** — the X8 Python side already does
   AES-128-GCM via `cryptography` library. The cost is the L072 firmware
   port, not new design work.

The argument for *dropping* it:

5. **Open-source ethos** — every command on the air is theoretically
   inspectable by anyone with the source. Confidentiality from the
   public is somewhat at odds with that.
6. **Compliance simplicity** — exporting strong crypto can complicate
   international distribution. Authenticated-but-plaintext is
   universally exportable.
7. **Airtime** — at 20 Hz control + image+telemetry traffic, every
   byte costs. See §19.4.

**Recommendation:** keep confidentiality but **drop the security
parameter** from 128-bit to 64-bit truncated tag + 4-byte implicit
nonce. This was already the [LoRa in-depth analysis §3
recommendation](2026-04-27_LoRa_InDepth_Analysis_ClaudeOpus4_7_v1_0.md);
it's still the right call after this latency review.

### 19.3 Options ranked from weakest to strongest

| Tier | Scheme | Overhead | Integrity | Replay | Conf. | Forgery margin | Notes |
|---|---|---:|:-:|:-:|:-:|---|---|
| **0** | Plaintext, no MAC | 0 B | ❌ | seq# only (no auth on seq) | ❌ | none | Attacker forges trivially. **Not acceptable for kinetic control.** |
| **0.5** | Plaintext + LoRa PHY CRC-16 | 0 B | ❌ (CRC, not MAC) | seq# only | ❌ | none against intent | CRC is free but is keyless — A4 forgery succeeds. **Not acceptable for control.** |
| **1** | Plaintext + **SipHash-2-4 64-bit MAC** | +8 B | ✅ 64-bit | seq# in MAC scope | ❌ | 1 in 1.8 × 10¹⁹ per attempt | **Cheapest viable for control.** Tiny code (~1 KB), fast on Cortex-M0+. No confidentiality. |
| **1** | Plaintext + **AES-CMAC 64-bit MAC** | +8 B | ✅ 64-bit | seq# in MAC scope | ❌ | 1 in 1.8 × 10¹⁹ | Same overhead as SipHash; same security; AES is heavier to integrate but reuses any AES core. |
| **1** | Plaintext + **HMAC-SHA256-32** | +4 B | ✅ 32-bit | seq# in MAC scope | ❌ | 1 in 4.3 × 10⁹ per attempt | LoRaWAN-equivalent. Smallest MAC. 32 bits is borderline — see §19.6. |
| **2** | **AES-CCM* with 32-bit MIC** (LoRaWAN style) | +4 B | ✅ 32-bit | counter in nonce | ✅ | 1 in 4.3 × 10⁹ | Same scheme LoRaWAN uses. Lightest with-confidentiality. |
| **2** | **AES-CCM-64** (truncated MIC) | +8 B | ✅ 64-bit | counter in nonce | ✅ | 1 in 1.8 × 10¹⁹ | Stronger MIC; +4 B vs CCM*. |
| **3** | **AES-GCM-64 implicit nonce** (4 B counter + 8 B tag) | +12 B | ✅ 64-bit | counter in nonce | ✅ | 1 in 1.8 × 10¹⁹ | What §3 of the v1.0 LoRa in-depth analysis recommended. **My pick for v25.** |
| **3** | **Ascon-128a** (NIST LWC winner) | +16 B | ✅ 128-bit | counter in nonce | ✅ | 1 in 3.4 × 10³⁸ | Designed for constrained MCUs. Modern, ~3 KB code. |
| **4** | **AES-GCM-128 implicit nonce** (4 B counter + 16 B tag) | +20 B | ✅ 128-bit | counter in nonce | ✅ | 1 in 3.4 × 10³⁸ | Halfway compromise. |
| **5** | **AES-GCM-128 explicit nonce** (12 B nonce + 16 B tag) | **+28 B** | ✅ 128-bit | nonce | ✅ | 1 in 3.4 × 10³⁸ | **What we ship today.** Most overhead of any sane option. |

### 19.4 Airtime cost — the headline numbers

Computed via the same SX1276 ToA formula as §15, on the **active
control profile SF7/BW250/CR4-5** (16 B cleartext ControlFrame) and
the **active image profile SF7/BW500/CR4-5** (32 B cleartext fragment):

| Scheme | Overhead | Control 16B → ToA | Δ vs plain | Image 32B → ToA | Fits 25 ms cap? |
|---|---:|---:|---:|---:|:---:|
| Plaintext / CRC-only | 0 B | **25.73 ms** | +0.00 ms | 17.98 ms | ✅ |
| HMAC-SHA256-32 / CCM*-32 | +4 B | 28.29 ms | +2.56 ms | 19.26 ms | ✅ |
| SipHash-64 / CMAC-64 / CCM-64 | +8 B | 30.85 ms | +5.12 ms | 20.54 ms | ✅ |
| **AES-GCM-64 implicit (recommended)** | **+12 B** | **33.41 ms** | **+7.68 ms** | **23.10 ms** | **✅** |
| Ascon-128a | +16 B | 35.97 ms | +10.24 ms | 24.38 ms | ✅ (tight) |
| AES-GCM-128 implicit | +20 B | 38.53 ms | +12.80 ms | **25.66 ms** | **❌** |
| **AES-GCM-128 explicit (shipped)** | **+28 B** | **46.21 ms** | **+20.48 ms** | **28.22 ms** | **❌** |
| ChaCha20-Poly1305 (full) | +28 B | 46.21 ms | +20.48 ms | 28.22 ms | ❌ |

**Critical finding:** the **currently shipped AES-GCM-128 explicit-nonce
scheme violates the 25 ms image fragment cap by itself (28.22 ms > 25
ms)**. This is the same finding §17 reached from a different angle:
post-encryption airtime breaks C1. Even the implicit-nonce 128-bit
variant misses by 0.66 ms.

The recommended **AES-GCM-64 with 4-byte implicit nonce sits at 23.10 ms
per image fragment** — comfortably under the cap, saves **16 B and
13.1 ms per frame** vs. the shipped scheme, and only weakens forgery
resistance from "1 in 3.4 × 10³⁸" to "1 in 1.8 × 10¹⁹" — both
unreachable for any realistic attacker.

### 19.5 Aggregate airtime savings if we drop from GCM-128-explicit to GCM-64-implicit

Going from +28 B → +12 B overhead, on the canonical traffic mix:

| Class | Cadence | Per-frame saving | Per-second airtime saving |
|---|---:|---:|---:|
| ControlFrame (16 B clear) at 20 Hz | 20 Hz | 46.21 − 33.41 = **12.80 ms** | **256 ms/s reclaimed on control alone** |
| HeartbeatFrame (16 B clear) at 20 Hz | 20 Hz | 12.80 ms | already counted above for source-active node |
| Telemetry (~100 B clear) at 1 Hz | 1 Hz | ~10 ms | 10 ms/s |
| Image fragments (~32 B clear) at ~190/refresh at 0.67 Hz | ~127 fps frag | (28.22 − 23.10) = 5.12 ms | **~650 ms/s reclaimed on image** |

**Total: ~0.9 s/s of airtime reclaimed on a saturated link** by the
crypto downgrade alone. That's effectively a free doubling of usable
channel time, before any other change in this plan ships.

### 19.6 Why 32-bit MAC is borderline (and how to make it safe anyway)

A 32-bit MAC gives 1-in-4.3 × 10⁹ forgery probability **per attempt**.
At a realistic A4 attack rate of 10 attempts/sec over a sustained
day-long attack:

- Expected time to one successful forgery: ~13.6 years.
- Probability of forgery in 1 day of sustained attack: ~2 × 10⁻⁴.
- Probability of forgery in 1 hour: ~8 × 10⁻⁶.

This is acceptable **only if** you add online attack detection:

- **Receiver-side rate-limit** on MAC-failure events: e.g., after 100
  consecutive bad MACs in a 10-second window, drop to safe state and
  log. An attacker forcing ~10⁹ attempts to find one valid frame will
  trip this trivially.
- **Key rotation** on detected attack (or scheduled, e.g. per session).
- **Authenticated reboot counter** in the nonce so attacker-captured
  traffic from a previous session can't be replayed.

With those safeguards, 32-bit MAC (HMAC-SHA256-32 or AES-CCM*-32) is
defensible. Without them, **64-bit MAC is the responsible floor** for
control-plane traffic. The +4 B is cheap insurance.

### 19.7 Replay protection — the actually critical piece

**Replay is the cheapest attack and has the worst kinetic outcome.**
Crypto strength is irrelevant if replay isn't airtight. Today's status:

- Per [`2026-04-27_Controller_Master_Plan_Review_Copilot_v1_0.md`
  §4](2026-04-27_Controller_Master_Plan_Review_Copilot_v1_0.md), the
  base-side E-STOP currently builds its header with sequence `0` and
  can be **rejected as replay** by the tractor after any normal base
  traffic. This is the inverse failure: the *legitimate* E-STOP is
  dropped as if it were a replay.
- Per [`2026-04-26_Controller_Hardware_Testing_Readiness_Review.md`
  §Critical-10](2026-04-26_Controller_Hardware_Testing_Readiness_Review.md),
  the same finding from a different reviewer.

This plan's §3.6 5-copy SAFETY burst makes the replay-rejection failure
**worse**: 5 identical E-STOP copies all carrying the same sequence
will all be rejected after the first one is consumed. Required fixes
before the burst ships:

1. Each of the 5 burst copies must carry **monotonically increasing
   sequence numbers** within the burst, **not** five copies of the
   same frame.
2. The receiver must accept **any** authenticated, in-window sequence
   for `CMD_ESTOP` (idempotent) without consuming the window for other
   commands.
3. Replay window must be **>=32 entries** so a 5-copy burst can't
   exhaust it.

This is a security correctness requirement on the burst design that
the current §3.6 text does not address.

### 19.8 The L072 boundary — Crypto Profile A revisited

§3.5 of this plan and the [Tranche 5
spec](2026-05-05_LoRa_Firmware_Tranche_5_ChannelHygiene_LinkAdaptation_Plan_Copilot_v1_0.md)
put crypto *above* the L072 firmware in "Crypto Profile A": the host
(X8 / H7) does AES-GCM and hands ciphertext to L072 for transmission.
L072 sees only opaque bytes.

This boundary has consequences for the encryption choice:

- **Pro:** L072 firmware footprint stays small. PSK never lives in L072
  flash. Crypto updates ship as host-side Python/C without firmware
  re-flash.
- **Pro:** All schemes in §19.3 are implementable purely in host code.
- **Con:** L072 cannot make any decision based on frame content
  (priority, class, E-STOP detection). All those decisions must be
  carried in **cleartext sideband** (the `tx_id` upper bits per §3.9,
  the host-link command type, etc.). §3.9's class-encoding choice
  becomes a **security-relevant interface** because cleartext class
  bits drive scheduling decisions an attacker could see.
- **Con:** If L072 also gets a depth-1 mailbox (§3.3 H), and that
  mailbox preempts based on cleartext class bits, an attacker who can
  inject *any* (even malformed) frame at the host-link side can starve
  the link by spamming "SAFETY" class. The host-link boundary needs
  its own authentication (it's USB/UART between trusted boards, but
  still worth pinning).

### 19.9 Implementation cost on the L072 (Cortex-M0+)

If we ever decide to move crypto **into** L072 (we shouldn't for v25,
but for completeness):

| Scheme | Code size | RAM | Per-frame CPU @ 32 MHz | Per-frame energy |
|---|---:|---:|---:|---|
| SipHash-2-4 | ~1 KB | ~32 B | ~50 µs for 16 B | negligible |
| AES-128-CMAC | ~3 KB (incl. AES core) | ~256 B | ~150 µs | negligible |
| AES-128-CCM | ~4 KB | ~512 B | ~200 µs | negligible |
| AES-128-GCM | ~5 KB | ~1 KB (GHASH table) | ~500 µs (GHASH is slow on M0+) | non-trivial |
| Ascon-128a | ~3 KB | ~128 B | ~120 µs | negligible |

**If** v26+ wants crypto in L072, **Ascon-128a is the right pick** —
it's a NIST competition winner specifically for this class of MCU,
beats AES-GCM on M0+ in every dimension, and gives full 128-bit
security at 16 B overhead (one byte better than implicit-nonce GCM-128).

### 19.10 Recommendation summary

In priority order:

1. **Keep encryption.** Confidentiality is not free but is cheap once
   we drop the security parameter. The operator-privacy argument is
   real enough.
2. **Switch from AES-GCM-128 explicit-nonce (+28 B) to AES-GCM-64
   implicit-nonce (+12 B).** Saves 16 B and ~13 ms per frame; makes
   image fragments fit the 25 ms cap; keeps a forgery margin of
   1-in-10¹⁹ which is unreachable for any realistic attacker. Ships
   the existing v1.0 LoRa-analysis §3 recommendation.
3. **Fix the replay-rejection failure for E-STOP** *before* the §3.6
   burst ships. Each burst copy needs a fresh sequence number;
   `CMD_ESTOP` must be accepted idempotently across the replay window.
4. **Pin the nonce construction:** `(source_id || boot_counter || seq)`
   with `boot_counter` stored in non-volatile flash and incremented at
   every boot. Solves the "[2026-04-27_MASTER_PLAN_Review_v1.0.md
   §High-5](2026-04-27_MASTER_PLAN_Review_v1.0.md)" nonce-uniqueness
   concern across reboots.
5. **Add MAC-failure rate-limit** at the RX (≥100 bad MACs in 10 s →
   drop to safe state + log). Cheap defense against the small-MAC
   attacker.
6. **Keep crypto in the host (Crypto Profile A).** Only revisit if v26+
   requirements force it into L072, at which point pick Ascon-128a.
7. **Treat the host-link (USB/UART between X8 and L072) as a trusted
   boundary** but document it as such — if a future change exposes it,
   the threat model changes.
8. **For v26+ "ultra-low-power telemetry" mode** (if it ever exists):
   downgrade to SipHash-64 MAC-only (+8 B, no confidentiality) for
   non-control traffic. Saves another 4 B vs. GCM-64 and removes the
   cipher entirely.

### 19.11 What "no encryption at all" would look like

If the user genuinely wanted to drop to **integrity-only, no
confidentiality** for an open-source/maximum-airtime build:

| Variant | Overhead | What it gets you |
|---|---:|---|
| Plaintext + LoRa CRC-16 + seq# | **0 B** | **Unsafe — accept only as a bench/dev mode behind `LIFETRAC_ALLOW_UNAUTHENTICATED=1`.** A4 forgery trivial. |
| Plaintext + SipHash-64 MAC + seq# | +8 B | Integrity + replay; no confidentiality. **Acceptable for control plane** if the privacy concerns in §19.2 don't apply. Saves 20 B vs current scheme. |
| Plaintext + HMAC-SHA256-32 MAC + seq# | +4 B | Same as above but at LoRaWAN-equivalent MIC strength. Needs the §19.6 rate-limit. Saves 24 B. |

The **strict minimum responsible floor** is "plaintext + SipHash-64
MAC + replay-protected sequence in MAC scope" (+8 B). Anything weaker
than 64-bit MAC requires the §19.6 RX-side rate-limit to be defensible.
**Zero-byte "no crypto at all" is not a defensible choice for any
kinetic control link**, regardless of project size or threat model.

### 19.12 Sign-off table for the encryption decision

| Question | Recommendation |
|---|---|
| Drop encryption entirely? | **No** — keep confidentiality, but only at GCM-64. |
| Drop confidentiality, keep integrity? | Acceptable fallback if airtime budget gets tighter; ship SipHash-64 MAC (+8 B). |
| Reduce security parameter from 128 → 64 bit? | **Yes** — saves 16 B/frame, no realistic attacker reaches 2⁶⁴. |
| Use implicit nonce derived from `(src, boot_ctr, seq)`? | **Yes** — saves 8 B/frame and fixes the nonce-reuse-across-reboot concern. |
| Move crypto into L072 firmware? | **No for v25.** If ever needed, pick Ascon-128a, not AES-GCM. |
| Add RX MAC-failure rate-limit? | **Yes** — required if MAC ≤ 64 bits; harmless if larger. |
| Fix the E-STOP replay-rejection bug before §3.6 burst ships? | **Yes — blocker.** |

---

**End of §19 encryption analysis, 2026-05-18 (Claude Opus 4.7).**

## 20. Split-Trust Variant: Unencrypted Images, Encrypted Kinetic Control (2026-05-18, Claude Opus 4.7)

**Short answer: yes — and it is actually the right call for v25, with
two caveats.** The argument and the numbers:

### 20.1 Why this works cleanly under the §19.1 threat model

Re-reading the §19.1 adversary table by **traffic class** instead of by
scheme:

| Class | A2 (eavesdrop) impact | A3/A4 (replay/forge) impact | What security it actually needs |
|---|---|---|---|
| **ControlFrame** (joystick, valve cmds) | low (positions only) | **kinetic — direct unsafe motion** | **Strong integrity + replay**, confidentiality optional |
| **HeartbeatFrame** | none | low (failsafe handles it) | Integrity, replay |
| **SAFETY / E-STOP** | none | **kinetic — failure to stop** | **Strong integrity + idempotent replay window** |
| **Telemetry** (RPM, pressure, battery) | low operator-privacy | low (display only, no actuation) | Integrity (so display isn't poisoned) |
| **Image fragments** | **none** — they are camera frames of a field | **none** — image is display-only; no actuator consumes image bytes | **Integrity optional**, replay irrelevant, confidentiality unnecessary |

The image stream is the **one class** where the kinetic-impact column is
genuinely zero. It is a one-way video pipe from tractor to operator
screen. A forged image frame causes a wrong picture, not a wrong motion.
A replayed image frame causes a stale picture, which the operator will
notice and which the existing image-pipeline freshness counter already
flags (per [`IMAGE_PIPELINE.md`](../DESIGN-CONTROLLER/IMAGE_PIPELINE.md)).

This is the same logic ADS-B, FM radio, and most non-safety telemetry
links use: cleartext + CRC is fine when the receiver can't be tricked
into unsafe action by bad data.

### 20.2 Airtime impact — the headline win

Recomputed from §19.4 with the **split-trust** policy:

| Class | Cleartext | Auth scheme | Per-frame overhead | Per-frame ToA |
|---|---:|---|---:|---:|
| Control / heartbeat / E-STOP (SF7/BW250) | 16 B | AES-GCM-64 implicit (recommended §19) | +12 B | 33.41 ms |
| Telemetry (~100 B clear, SF9/BW250) | 100 B | AES-GCM-64 implicit | +12 B | (unchanged from §15) |
| **Image fragments (SF7/BW500)** | **32 B** | **none / LoRa PHY CRC-16 only** | **+0 B** | **17.98 ms** |

Image airtime at the canonical ~127 fragments/sec refresh rate:

| Image policy | Per-frag ToA | Per-second image airtime | Δ vs shipped GCM-128 |
|---|---:|---:|---:|
| Plaintext + PHY CRC (proposed) | 17.98 ms | **2284 ms/s** | **−1300 ms/s** |
| SipHash-64 MAC | 20.54 ms | 2609 ms/s | −975 ms/s |
| AES-GCM-64 implicit | 23.10 ms | 2934 ms/s | −650 ms/s |
| AES-GCM-128 explicit (shipped today) | 28.22 ms | 3584 ms/s | 0 |

**Dropping image crypto entirely reclaims ~1.3 seconds per second of
LoRa airtime** — that is the largest single airtime win available
anywhere in this entire document, larger than the depth-1 mailbox win
(§15) and larger than the §19 crypto-downgrade win combined. It also
**makes the 25 ms image-fragment cap unconditionally satisfiable** with
margin, instead of barely or not at all.

### 20.3 Caveats

#### Caveat 1 — A4 attacker can poison the operator's screen

An attacker who forges image fragments cannot move the tractor, but
they *can* show the operator a stale or fabricated field. Operator
might:

- believe a previously-cleared obstacle is still gone (and command motion);
- believe the bucket is in a different position than it actually is;
- be lured into an emergency response to a fake hazard.

**Mitigation:** the kinetic decisions still go through the
authenticated control link. The operator never *commands* anything on
the basis of image bytes that wasn't also confirmed by authenticated
telemetry. Document this explicitly in the operator manual:

> "Video feed is informational; it is not authenticated. Cross-check
> physical state via authenticated telemetry (pressure, position,
> attitude) before commanding motion into uncertain terrain."

This is the same convention as a dashcam vs. a brake pedal — one is
advisory, the other is safety.

#### Caveat 2 — image fragments must not share the cleartext header bits that drive scheduling

Per §3.9 and §19.8, the L072 schedules and class-tags traffic from
cleartext header bits (`tx_id` upper bits, etc.). If an attacker can
inject **anything** the L072 accepts, they can:

- spam fake "image" frames at 100 % duty cycle → starve the link
  (denial of service against the kinetic channel);
- spoof the class bits on those frames to masquerade as SAFETY → trip
  the §3.6 burst-rejection path or the depth-1 mailbox.

**Mitigation, in priority order:**

1. **The host-link path** (X8 → L072 USB/UART) must reject any frame
   whose declared class is not "image" if its content was not
   authenticated. I.e., the *class tag itself* must be enforced at the
   trusted host boundary, not just trusted from the air.
2. **Receiver-side rate-limit on cleartext image frames**: cap
   accepted image-class traffic at the design refresh rate (~150
   frag/s with margin) and drop the rest. Attacker spamming at 100 %
   duty trips the limiter, control plane stays intact.
3. **Image frames keep a 4-byte sequence + 2-byte CRC32 of the
   plaintext payload** (~6 B overhead, still saves 22 B vs current
   GCM-128). This isn't a MAC, but it gives the decoder a way to drop
   garbage before pushing to the display pipeline and detect gross
   tampering. Cheap, no key material.

#### Caveat 3 — privacy of the camera feed

A neighbor with an RTL-SDR can see the camera feed. For an OSE
open-source farm tractor in a private field this is almost certainly
acceptable; it would not be acceptable on, say, a security-patrol
robot. **Document the policy choice explicitly** in the README, so
downstream forks for other use cases see the assumption.

### 20.4 Recommended scheme (revising §19.10)

| Class | Cleartext | Crypto | Overhead | Per-frame ToA |
|---|---:|---|---:|---:|
| ControlFrame | 16 B | **AES-GCM-64 implicit nonce** | +12 B | 33.41 ms (SF7/BW250) |
| Heartbeat | 16 B | AES-GCM-64 implicit nonce | +12 B | 33.41 ms |
| SAFETY / E-STOP | 16 B | AES-GCM-64 implicit nonce, **idempotent across replay window**, sequence-incremented across 5-copy burst | +12 B | 33.41 ms |
| Telemetry | ~100 B | AES-GCM-64 implicit nonce | +12 B | (SF9/BW250 path) |
| **Image fragments** | **32 B** | **plaintext + PHY CRC-16 + 4 B seq + 2 B payload CRC32** (no MAC) | **+6 B** | **20.54 ms** (SF7/BW500) |

**Net effect vs. status quo (shipped AES-GCM-128 everywhere):**

- Image: **−22 B / fragment, −7.68 ms / fragment, −975 ms/s of airtime**.
- Control: −16 B / frame, −12.8 ms / frame, −256 ms/s.
- **Total reclaimed airtime ≈ 1.2 s/s on a saturated link.** Image
  fragments fit the 25 ms cap with 4.5 ms margin.
- 25 ms C1 cap satisfied; SAFETY burst feasible; depth-1 mailbox win
  preserved.

### 20.5 Refinements if you want a middle ground

If "completely unauthenticated image" is uncomfortable, two cheap
intermediates exist:

1. **Per-keyframe MAC, per-fragment CRC**: authenticate only the first
   fragment of each refresh (which carries the frame header and
   freshness counter) with a SipHash-64 MAC (+8 B once per ~190
   fragments). All other fragments use the +6 B seq+CRC32 from §20.4.
   Attacker can poison fragments within a frame but cannot inject a
   whole new frame without breaking the keyframe MAC. Average overhead
   ≈ 6.04 B/fragment.
2. **Plaintext + SipHash-64 MAC on every image fragment** (+8 B). Still
   saves 20 B / 7.68 ms per fragment vs. shipped scheme. Per-second
   airtime budget 2609 ms/s vs. shipped 3584 ms/s — saves 975 ms/s.
   This is the conservative pick: full forgery protection on every
   fragment, no key/nonce confidentiality, no overhead surprises.

The **per-keyframe MAC** option (#1) is the best
security/airtime trade if §20.3 caveat 1 worries the operator, since it
costs almost nothing but raises the forgery bar for a coherent fake
frame.

### 20.6 Sign-off update for the encryption decision (delta vs §19.12)

| Question | §19 answer | §20 refined answer |
|---|---|---|
| Encrypt image fragments? | Yes (GCM-64) | **No — drop to plaintext + CRC + seq (+6 B), or per-keyframe MAC** |
| Encrypt control / heartbeat / E-STOP? | Yes (GCM-64 implicit) | **Yes — unchanged** |
| Encrypt telemetry? | Yes (GCM-64 implicit) | Yes — but downgradeable to MAC-only if budget tightens |
| Enforce image class tag at host boundary? | (not addressed) | **Yes — required to prevent class-spoof DoS** |
| Rate-limit cleartext image traffic at RX? | (not addressed) | **Yes — cap at design refresh rate + margin** |
| Document image-feed-is-advisory in operator manual? | (not addressed) | **Yes — required** |

---

**End of §20 split-trust crypto variant, 2026-05-18 (Claude Opus 4.7).**

## 21. Final Consolidated Review & Sign-Off Suggestions (2026-05-18, Claude Opus 4.7)

*(Final pass after re-reading §0–§20 end-to-end. The document has grown
from a clean design proposal (§0–§11) through four successive review
appendices (§12–§16), two latency passes (§15, §17, §18), and a
two-part encryption study (§19, §20). This section is **not another
review of the reviews** — that would compound §16's complaint about
unbounded review depth. It is a final synthesis: what the whole
document actually says when read together, what is now contradictory
on its own pages, and what to do next.)*

### 21.1 What the document actually decides (synthesized)

Stripping the iteration noise, the document — across all 20 sections —
converges on a coherent design. The accepted decisions are:

| # | Decision | Originating section | Confirmed by |
|---|---|---|---|
| **D1** | Depth-1 latest-only mailbox for P0 control | §3.3 H | §14.1, §15.6, §17.2, §18.3 |
| **D2** | 5-copy SAFETY burst at +17 dBm for E-STOP **assert**, bypassing mailbox; **non-bursted** RELEASE | §3.6 Q, §3.7 | §15.5, §18.5 |
| **D3** | Pull-recovery (`CMD_REQ_KEYFRAME`-style) for E-STOP RELEASE / EVENT-class loss, not push burst | §14.2 Q′ | §15.2, §15.9 |
| **D4** | Decorrelate burst copies (≥50 ms spacing, FHSS-diverse when available) | §14.7, §15.3 | §18.5 |
| **D5** | Failsafe-on-silence **500 ms** ship default (250 ms behind config flag) | §13.1, §15.9 | §16.3, §17 |
| **D6** | Explicit controller-priority ladder: power → encode → SF, with longer hysteresis on recovery, **branched on root cause** (margin-limited vs airtime-limited) | §14.1 + §15.2 correction | §16.1, §18.7 |
| **D7** | TX adapter on base-side **X8 Python** (not H7) and tractor-side host; L072 firmware enforces caps + safety bypass only | §11, §3.5 N, §12.3 | §13.1, §15.9 |
| **D8** | Inner SNR loop capped at **10 Hz**, TX-idle aligned; outer PER loop 100-packet | §14.4, §3.10 | §15.2, §18.7 |
| **D9** | PER feedback piggybacked in existing telemetry topic `0x10` at **0.2–0.5 Hz**, not a 9 B addition to every frame | §12.3, §15.4, §18.6 | §15.9, §17 |
| **D10** | Use existing P0/P1/P2/P3 + `frame_type`/`opcode`/`topic_id` taxonomy on-air; class encoding (Option T or U) is a **host-to-L072 hint only** | §12.3, §13.3, §16.2 | §14.3, §15.3 |
| **D11** | Replace generic `STREAM_*` table in §4 with the P0/P1/P2/P3 policy table from §15.3 | §15.3 | §16.4 |
| **D12** | All scheduler gates evaluated on **post-encryption airtime** | §17, §18.7 | §19.4, §20.2 |
| **D13** | Crypto: drop from AES-GCM-128 explicit (+28 B) to **AES-GCM-64 implicit nonce (+12 B)** for control/telemetry; keep Crypto Profile A (host) | §19.10 | §20.4 |
| **D14** | **Image fragments: plaintext + PHY CRC + 4 B seq + 2 B payload CRC32 (+6 B, no MAC)**, enforce class tag at trusted host boundary, RX rate-limit at design refresh rate | §20.4 | §20.6 |
| **D15** | Fix E-STOP `seq=0` replay-rejection bug **before** §3.6 burst ships; each burst copy carries fresh monotonic seq; `CMD_ESTOP` idempotent across replay window | §19.7, §19.10 | §20 (re-affirmed) |
| **D16** | Add operator-visible "LINK" pill (SF / dBm / PER / SNR) symmetric with image-pipeline `IMG:` and `AI:` pills, published via topic `0x10` | §14.5, §12.6 | §16.2 |

**These 16 are the document's actual proposal.** Phases 0–6 in §6 should
be rewritten to deliver them in this order: D1 → D15 → D2/D4 → D14 →
D13 → D12/D6 → D7/D8/D9 → D11/D16.

### 21.2 What is now self-contradictory in the document

The iteration left three internal contradictions that an implementer
reading top-to-bottom would hit:

| # | Contradiction | Where | Resolution |
|---|---|---|---|
| **C1** | Failsafe default: §3.8 says **250 ms**; §12.3, §13.1, §15.9, §17 say **500 ms** | §3.8 vs all later passes | **Edit §3.8 in main body** to "500 ms ship default; 100–1000 ms configurable; 250 ms behind bench flag." This is the single most important consolidation edit. |
| **C2** | Class encoding: §3.9 recommends **Option U**; §12.3, §13.2, §13.3, §16.3 recommend **Option T** | §3.9 vs later | **Edit §3.9** to "Option T for product, Option U as migration shim." |
| **C3** | Adapter location: §3.5 says "H7 host (operator side)"; §11, §12.3, §13.1, §13.4 say "X8 Linux (base has no H7)" | §3.5 vs appendices | **Edit §3.5 and §4 architecture diagram** to remove all "H7 base-side" language. |
| **C4** | Image fragment overhead: §19 (recommends GCM-64 even on image) vs §20 (recommends plaintext+CRC on image) | §19 vs §20 | §20 supersedes §19 for the image class specifically. Document this in §4 traffic table. |
| **C5** | Section numbering: two `## 13`, two `## 14`, two `## 15` (one user-authored, one Copilot-authored each) | §12 vs §13, §14 vs §15 boundaries | Renumber the user-authored §13/§14/§15 to §13/§15/§16 (or similar) — pure editorial. §16 already flagged this. |

### 21.3 What no review pass has yet addressed (genuine gaps)

These are findings I do not see in any prior section, surfaced by the
end-to-end read:

1. **There is no quantitative model for the depth-1 mailbox's effect
   on multi-source arbitration.** The plan assumes one P0 source (the
   operator handheld). If both the handheld and the tractor's
   onboard-controller can emit P0 (e.g. tractor-side autonomous-stop),
   "depth-1" needs a per-source slot or an explicit merge policy, or
   it silently drops one source's commands when the other is fast.
   **Action:** add §3.3 sub-question: how many P0 sources can be
   simultaneously active, and what is the merge rule?

2. **The 5-copy burst probability math (§3.7, §15.3, §18.5) never
   accounts for the receiver's decoder being busy on a different
   packet during one of the 5 copies.** Half-duplex single-channel
   means if the RX is mid-decode of a telemetry frame when copy #N
   arrives, copy #N is lost regardless of SNR. This *is* a form of
   correlated loss but it's deterministic and predictable. Burst
   spacing of 50 ms is correctly chosen, but the document should say
   *why* (it must exceed the worst-case other-traffic ToA on the
   channel).

3. **The split-trust crypto design (§20) makes the L072 a
   classification authority** for image vs control class. If the L072
   firmware bug causes it to misclassify a control frame as image,
   that frame goes out **unauthenticated and could be forged on the
   air**. The design needs a clear invariant: *the L072 may downgrade
   class privileges (treat unknown as control = strict crypto), never
   upgrade them (treat unknown as image = no crypto).*

4. **No section discusses what happens when the X8/H7 host process
   crashes** while a depth-1 mailbox is mid-write. The L072 needs an
   "abandoned writer" timeout on the mailbox slot so a half-written
   frame doesn't sit forever.

5. **The encode-mode ladder, SF ladder, and TX-power adapter all run
   on the receiver's view of the link, not the sender's.** That means
   for the tractor→base direction, the *base* drives the policy and
   commands the tractor's TX behavior. The reverse-direction
   (base→tractor) needs the same loop with the tractor as the
   policy-driving RX. The document discusses this implicitly in §10.5
   but never states the symmetry as a design requirement. **Action:**
   add to D7 that the loop is per-link-direction, with separate state.

6. **Regulatory: the +17 dBm SAFETY-class power cap may exceed EU EIRP
   limits at 868 MHz once mast antenna gain is included.** EU 868 ISM
   max ERP is +14 dBm (25 mW) on most sub-bands. The L072 firmware
   clamps to +17 dBm but the antenna gain pushes this past regulatory
   ceiling. SAFETY-class "ignore LBT" carve-out (§7) does NOT extend
   to ignoring EIRP limits. **Action:** add explicit regulatory ceiling
   that depends on `(region, sub-band, antenna_gain)` and ensure
   SAFETY-class can never exceed it; document FCC vs EU split.

### 21.4 Risk matrix for the consolidated proposal

| Risk | Likelihood | Impact | Mitigation already in plan? |
|---|---|---|---|
| Depth-1 mailbox silently drops fragments of a multi-frame image | Med | Med | Yes — D11, §15.3 explicitly separates fragment-accounted P3 from latest-only P0 |
| Three-loop hunting (§14.1) | Med | Med | Yes — D6 priority ladder + dwell timers |
| E-STOP replay-rejection bug kills the 5-copy burst | **High** | **Critical (safety)** | Yes — D15, must ship before D2 |
| Reduced-MAC forgery in field | Very low | High | Yes — D13 keeps 64-bit MAC + §19.6 rate-limit |
| Cleartext image frames used for class-spoof DoS | Med | High | Yes — D14 host-boundary class tag enforcement + RX rate-limit |
| L072 class-misclassification → unauthenticated control on air | **Low but not addressed** | **Critical (safety)** | **No — see §21.3 item 3, needs explicit "never upgrade class" invariant** |
| Multi-source P0 arbitration | Low (no second source today) | Med | **No — see §21.3 item 1** |
| EU EIRP violation on SAFETY burst | Med (depends on region) | Compliance/legal | **No — see §21.3 item 6** |
| Reviewer fatigue / decision creep (more review passes) | High | Med (delays implementation) | Yes — §16.5 and this section both call it out |

The three **No** rows are the genuine new work surfaced by this final
pass. Everything else has been covered somewhere in the existing 20
sections.

### 21.5 Final recommended next actions (in order)

1. **STOP appending review sections.** §16.5 was right. After §21 the
   next edit to this document should be a **consolidation pass**, not
   another appendix.
2. **Apply C1–C5 from §21.2** (~30 min of editing). These are pure
   contradictions that an implementer will trip on within an hour of
   reading.
3. **Add §3.3.1, §3.6.1, §3.5.1, §7.1** addressing the four gaps in
   §21.3 items 1, 2, 3 (multi-source, half-duplex correlation,
   class-downgrade invariant), and item 6 (regulatory EIRP).
4. **Rewrite §6 (phasing) around the D1→D16 ordering** in §21.1, with
   D15 (E-STOP replay fix) as a blocker before D2.
5. **Promote the four oldest review findings into main-body text** and
   delete the appendix versions:
   - §12.3 §3.6/3.7 hardware-first safety story → into §7
   - §13.1 PHY-pin precedence → into §3.1
   - §14.1 controller priority ladder → into §3 main body
   - §15.3 P0/P1/P2/P3 policy table → into §4
6. **Cut the document to ~1200 lines** (it is 2300+ now). Each accepted
   decision should appear once, in its canonical location, not once in
   the original section + 3 review-pass restatements.
7. **Open a separate companion doc** for the unresolved
   safety-vs-regulatory questions in §21.3-6 — that's a different
   audience (compliance) and different evidence base.

### 21.6 What's good about this document as-is

To balance the cleanup list: the design itself is sound. After
consolidation, this would be a defensible spec. Specifically:

- The **"newest-data-wins, no retries"** framing in §3.3 is the correct
  architectural invariant for safety-relevant 20 Hz control over LoRa.
  It is also the right answer for image refresh cancellation (D11).
- The **explicit safety burst probability math** (§3.7, §15.3) is more
  rigorous than what the image-pipeline doc currently has for its own
  loss-tail analysis. This style of analysis should propagate to other
  v25 safety subsystems.
- The **three-controller priority cascade** (§14.1, D6) is a genuinely
  important insight that no other plan in the v25 doc set has stated.
  Without it, the system would hunt in ways that are very hard to
  debug post-hoc.
- The **crypto downgrade math** (§19.4, §20.2) finds a free ~1 s/s of
  airtime that no other v25 document has identified. The split-trust
  variant (§20) is a genuine architectural improvement and not just
  byte-shaving.
- The **latency analysis** correctly distinguishes raw-payload airtime
  from post-encryption airtime (§17.1) and shows that the depth-1
  mailbox saves **231 ms** of stale tail under encrypted framing
  (§17.2) — almost twice the §15 estimate.

### 21.7 One-paragraph summary of the entire document

> *Adopt a depth-1 latest-only mailbox for P0 control (saves up to
> 231 ms of stale-command tail under encrypted framing). Ship E-STOP
> assert as a 5-copy burst with ≥50 ms spacing at the regulatory power
> cap, but fix the existing `seq=0` replay-rejection bug first. Run a
> single closed-loop adapter per link direction on the X8 Python host
> (no H7 on the base), with explicit priority ordering power →
> encode-mode → SF, branched on root cause (margin vs airtime). Drop
> the encryption parameter from AES-GCM-128 (28 B overhead) to
> AES-GCM-64 implicit-nonce (12 B) for kinetic traffic, and run image
> fragments plaintext+CRC (6 B) since they have zero kinetic impact —
> together these reclaim ~1.2 s/s of airtime and make the 25 ms
> fragment cap unconditionally satisfiable. Ship 500 ms heartbeat
> failsafe (not 250 ms). Stop reviewing and start consolidating.*

---

**End of §21 final consolidated review, 2026-05-18 (Claude Opus 4.7).**
**End of document v1.0 review cycle. Next edit should be a consolidation pass, not an appendix.**

## 22. Final Copilot Recommendations (2026-05-18)

*(Final review pass requested by user. This section is the final actionable
recommendation set and should be treated as the implementation handoff.)*

### 22.1 Final recommended defaults (close §10)

| Open question | Final recommendation |
|---|---|
| 10.1 telemetry mailbox | Use class-aware scheduling by P0/P1/P2/P3; P0 latest-only mailbox, P3 refresh-cancelable but fragment-accounted. |
| 10.2 safety burst size | Default 5 copies for E-STOP assert, configurable; keep non-safety events at lower burst count or pull-recovery. |
| 10.3 heartbeat presence/schema | Heartbeat exists; use reserved byte for `state_bits` only with verified backward compatibility, else bump schema/version. |
| 10.4 silence timeout | Ship default 500 ms; keep 250 ms as bench-tunable option pending hydraulic nuisance-stop validation. |
| 10.5 adapter placement | Base and tractor X8 stacks both run per-direction adapter logic; base H7 is not a product target. |
| 10.6 phase ordering | Keep Phase 0 first; gate by P0 TX-start, canvas freshness, and keyframe recovery (not frame-complete-only). |
| 10.7 class encoding | Product target: explicit host-to-L072 class/priority byte (Option T); Option U only as temporary interop shim. |

### 22.2 Final blockers before implementation

1. Fix section conflicts and duplicates so each accepted decision appears once
  in canonical sections (§3–§7), not only in review appendices.
2. Add one precedence rule: when docs disagree, implementation follows
  `DECISIONS.md` + latest protocol constants in code.
3. Define mixed-version interop behavior for Option T/U, including startup
  capability detection and safe fallback.
4. Confirm replay-window behavior for bursted E-STOP copies (monotonic
  sequence handling and idempotent assert semantics).
5. Add explicit regional TX-power/EIRP guardrail text so SAFETY-class never
  exceeds legal limits.

### 22.3 Final validation gates to keep

* p99 P0 TX-start delay under mixed P2/P3 load.
* Worst-case control command age at actuator boundary (before/after depth-1).
* E-STOP first-arrival and latch latency under bursty interference.
* Mixed-version host/firmware interop with safety/event semantics preserved.
* Telemetry evidence that records adaptation reason, SF rung, encode mode,
  and active TX power for every adaptation transition.

### 22.4 Final editorial guidance

No further review appendices should be added before consolidation. The next
edit should convert accepted guidance into the main design body and trim
appendix churn so firmware/host implementers have one unambiguous source.

---

**Signed:** GPT-5.3-Codex (GitHub Copilot)

## 23. Final Review and Suggestions (2026-05-18, GitHub Copilot)

This is my final review after reading the full document through §22. Treat
this section as the closing review note from this pass. It supersedes the
earlier appended "final" review blocks as a recommendation set, but it does not
erase their evidence or reasoning.

### 23.1 Final verdict

The architecture is directionally correct, but the document should **not** be
used directly as an implementation spec yet. It has too many appendices,
duplicate section numbers, and superseded defaults. The next edit should be a
consolidation pass that folds accepted decisions into §3-§7 and leaves only a
short appendix for unresolved gates.

The project can proceed immediately with Phase 0 instrumentation: encrypted
airtime measurement, P0 TX-start timing, queue-age measurement, power sweeps,
and safety-burst interference tests. It should not proceed to Phase 1 firmware
changes until the defaults below are locked in one canonical table.

### 23.2 Defaults I would lock

| Topic | Final recommendation |
|---|---|
| Traffic model | Use P0/P1/P2/P3, not the older `STREAM_*` taxonomy, as the canonical policy model. |
| P0 control queue | Latest-only mailbox per active source; no stale FIFO tail. |
| P1 commands | Small priority FIFO or command-specific coalescing; no command inversion. |
| P2 telemetry | Bounded FIFO, fragment-accounted, post-encryption airtime gates. |
| P3 image | Refresh-level cancellation is OK; blind fragment overwrite is not. Missing fragments must produce stale tiles/keyframe recovery, not false completion. |
| Safety assert | Dedicated path, 5-copy burst by default, max legal/allowed power, sticky heartbeat state, silence failsafe, and hardware safety backstop. |
| Safety release | Not bursted as assert; explicit operator-confirmed state convergence only. |
| Silence timeout | Ship 500 ms default; keep 250 ms as a bench/field option only after hydraulic nuisance-stop validation. |
| Class encoding | Product path should use an explicit host-to-L072 priority/class byte. `tx_id` upper bits are acceptable only as a migration shim. |
| PER feedback | Piggyback bounded-cadence feedback in source-active/link-health telemetry before adding unconditional 9 B overhead to every report. |
| Airtime accounting | Scheduler gates must use post-encryption LoRa airtime unless a raw-image-fragment exception is explicitly documented. |
| Control loss target | Replace the original 20% P0 PER language with field gate <1% and recovery trigger before 2%. |

### 23.3 One precedence rule to add near the top

When documents disagree, implementation should follow this order:

1. Current code constants and measured bench evidence.
2. `DECISIONS.md` for active PHY choices.
3. `LORA_PROTOCOL.md` for protocol/heartbeat defaults.
4. `IMAGE_PIPELINE.md` for P3 image/C1 semantics.
5. This document for TX-power adaptation, host-to-L072 scheduling, and safety
  burst policy.

That rule is the simplest way to stop stale BW125/BW250, 250/500 ms timeout,
Option T/U, and raw/encrypted airtime conflicts from leaking into code.

### 23.4 Required consolidation edits

1. Renumber the document and remove duplicate `## 13`, `## 14`, `## 15`, and
  `## 18` headings.
2. Delete or fold the orphaned `cap. Prefer...` paragraph back into the §3.4
  PER-feedback discussion.
3. Replace §4's class table with the P0/P1/P2/P3 policy table.
4. Update §5 defaults: 500 ms silence default, explicit class byte, bounded
  feedback cadence, post-encryption airtime accounting.
5. Rewrite §6 gates so they measure P0 TX-start p99, command age at actuator,
  canvas freshness, keyframe recovery, and safety latch latency.
6. Add a compatibility table for old host/new firmware and new host/old
  firmware, with explicit safe fallback behavior.

### 23.5 Falsification gates before firmware changes

* **Encrypted airtime gate:** compare `lora_time_on_air_ms(encrypted_payload_len(...))`
  predictions against measured `TX_DONE` p50/p99 for P0, P2, and P3 traffic.
* **P0 starvation gate:** under max image+telemetry load, no lower-priority
  packet may cause P0 TX-start delay >25 ms.
* **Mailbox gate:** under saturated submit rate, stick release must produce
  zero stale-motion commands after the next P0 sample.
* **Safety-burst gate:** measure assert-intent miss rate and p99 latch latency
  under bursty interference; compare same-channel vs hop-diverse copies.
* **Controller-state gate:** use a step attenuator and generated airtime load
  to prove margin-limited and airtime-limited failures trigger different
  actions.
* **Power-write race gate:** update TX power at the proposed max cadence while
  transmitting; pass requires no increase in TX timeouts, parse faults, RX
  misses, or P0 delay.
* **Interop gate:** prove both upgrade orders preserve safety/event priority
  and never silently reinterpret class bits.

### 23.6 Final implementation order

1. Consolidate this document and lock the defaults above.
2. Add instrumentation and bench artifacts for encrypted airtime, `TX_DONE`
  latency, P0 TX-start p99, queue age, and link-health topic `0x10` fields.
3. Implement only the P0 latest-only mailbox first.
4. Add safety burst + sticky heartbeat state with the 500 ms product silence
  default.
5. Add bounded-cadence PER/link feedback and confidence intervals.
6. Add the unified `NORMAL`, `MARGIN_LIMITED`, `AIRTIME_LIMITED`, and
  `RECOVERY` state machine with dwell timers.
7. Enable TX-power adaptation behind a bench flag; graduate it only after
  step-attenuator, mixed-load, interop, and safety-burst gates pass.

### 23.7 Closing note

The biggest lag win in this whole plan is not the power adapter. It is removing
the stale P0 FIFO tail. TX-power adaptation saves energy and gives margin
discipline; safety bursts improve loss-tail behavior; encryption choices shape
airtime; but the operator-feel and motion-safety improvement comes from a
latest-only P0 mailbox backed by strict P0 priority and post-encryption airtime
gates.

Signed: GitHub Copilot  
Model/version: GitHub Copilot

---

**End of §23 final review and suggestions, 2026-05-18.**

## Change-log — surgical S0 consolidation pass (2026-05-18)

Per the §21.5 / §23.4 instruction to stop appending review sections and
begin consolidation, the following surgical edits were applied to the
main body so an implementer reading top-to-bottom no longer hits the
C1–C5 contradictions catalogued in §21.2:

| Edit ID | Section | Change | Rationale |
|---|---|---|---|
| **S0.1 (C1)** | §3.8 | Ship default failsafe-on-silence raised from 250 ms → **500 ms**. 250 ms preserved behind a bench flag. | §12.3, §13.1, §15.9, §17, §22.1, §23.2 all converged on 500 ms after hydraulic nuisance-stop risk analysis. Strikethrough preserves original text for review-trail integrity. |
| **S0.2 (C2)** | §3.9 | Recommendation changed from Option U → **Option T** (explicit class byte). Option U retained as migration shim. | §12.3, §13.2, §13.3, §16.3, §22.1, §23.2 converged on T; T avoids the `tx_id`-space collision with monotonic burst seq required by D15. |
| **S0.3 (C3)** | §3.5 | All "H7 host (operator side)" language replaced with "X8 Python host (operator side)". Title updated. | §11, §12.3, §13.1, §13.4 noted the base station has no H7; adapter is X8 Python on both ends. |
| **S0.4 (C4)** | §4 | Image-class row crypto overhead: AES-GCM-128 (+28 B) → **plaintext + 4 B seq + 2 B CRC32 (+6 B, no MAC)**. SAFETY-class power: "+17 dBm forced" → "regional EIRP cap per §21.3-6". STREAM_* class names tagged with P0/P1/P2/P3 equivalents per D11. | §20 split-trust design supersedes §19 for image class; §21.3-6 regulatory analysis supersedes blanket +17 dBm. |
| **S0.7** | Header | Cross-doc precedence rule added near top per §23.3. | Stops stale BW125/BW250, 250/500 ms timeout, Option T/U, and raw/encrypted airtime conflicts from leaking into code. |
| **S0.8** | §4 | P0 control PER target tightened from "≤20% PER OK" → "field <1%, recovery trigger <2%". | §23.2 explicitly replaces the original 20% language. The 20% number was an upper-bound theoretical, not a field gate. |

**Edits deferred** (will be applied in a later consolidation pass, not
this surgical pass, because they involve renumbering / restructuring
that would break inbound section-anchor references from §12–§23):

- **S0.5 (C5)** Renumber duplicate `## 13`, `## 14`, `## 15`, `## 18`
  headings. Requires audit of every `§13.x`-style cross-reference in
  the review appendices.
- **S0.6** Promote §15.3 P0/P1/P2/P3 policy table into §4 as the
  canonical traffic-class table (replacing the `STREAM_*` table, which
  is now annotated inline above as a transitional step).

Both deferred edits are tracked as separate checklist items under
"2026-05-18 TX-power adaptation + SAFETY-burst implementation plan" in
[`LifeTrac-v25/TODO.md`](../TODO.md).

**Status of surgical pass: complete. The C1–C5 contradictions are
resolved in the main body. Review trail in §12–§23 remains intact and
still resolves correctly against the strikethrough-preserved original
text.**

Signed: GitHub Copilot
Date: 2026-05-18
