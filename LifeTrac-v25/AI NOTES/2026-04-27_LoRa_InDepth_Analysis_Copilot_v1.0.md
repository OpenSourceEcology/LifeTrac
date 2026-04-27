# LifeTrac v25 LoRa In-Depth Analysis - Copilot v1.0

Date: 2026-04-27
Author: GitHub Copilot
Scope: Timing realism, half-duplex management, payload optimization, and image/video data strategy for LoRa transport plus base-station enhancement.
Primary references:
- [DESIGN-CONTROLLER/MASTER_PLAN.md](../DESIGN-CONTROLLER/MASTER_PLAN.md)
- [DESIGN-CONTROLLER/LORA_PROTOCOL.md](../DESIGN-CONTROLLER/LORA_PROTOCOL.md)
- [AI NOTES/2026-04-26_LoRa_QoS_Bandwidth_Management.md](2026-04-26_LoRa_QoS_Bandwidth_Management.md)
- [DESIGN-CONTROLLER/VIDEO_OPTIONS.md](../DESIGN-CONTROLLER/VIDEO_OPTIONS.md)

---

## 1. Executive Summary

The LoRa architecture direction is correct, but the current timing assumptions are not physically consistent with the pinned PHY settings. The key issue is airtime oversubscription on a single half-duplex SX1276 channel.

Most important conclusions:

1. At SF7/BW125 with the current encrypted frame shape, 20 Hz control plus 20 Hz heartbeat is not feasible.
2. Half-duplex must be treated as a scheduling problem, not just a retry/backoff problem.
3. Best near-term architecture is an asymmetric deterministic grant-window model (base-driven control windows, tractor uplink windows).
4. Payload optimization should focus first on removing redundant traffic and shrinking fixed overhead.
5. LoRa image transport should be treated as low-rate situational thumbnails, not live video.
6. Visual quality should be recovered at the base station via enhancement and inference, with clear confidence/staleness indicators.

---

## 2. Inputs and Baseline Assumptions

From current canonical docs:

- Control path: SF7/BW125/CR4/5 (MASTER_PLAN 8.17).
- Control cadence: 20 Hz ControlFrame and 20 Hz HeartbeatFrame (MASTER_PLAN 8.12).
- Telemetry path currently described as separate profile (LORA_PROTOCOL).
- Security: AES-128-GCM with explicit nonce + auth tag on every frame.
- Radio: SX1276 class, single radio, single active PHY at any instant, half-duplex TX/RX.

---

## 3. Timing Analysis

### 3.1 Airtime formula

LoRa airtime is dominated by symbol time and coding overhead. A practical formulation is:

$$T_s = \frac{2^{SF}}{BW}$$

$$T_{air} = T_{preamble} + T_{payload}$$

with payload symbols computed from payload bytes, SF, coding rate, and header/CRC mode (Semtech SX1276 method).

### 3.2 Corrected control-frame airtime (current frame shape)

Using the current encrypted on-air control payload envelope (roughly mid-40-byte class after nonce/tag/framing), CR4/5, explicit header:

| PHY | Approx control frame airtime |
|---|---:|
| SF7 / BW125 | about 90-95 ms |
| SF8 / BW125 | about 160-170 ms |
| SF9 / BW125 | about 285-310 ms |
| SF7 / BW250 | about 45-48 ms |
| SF7 / BW500 | about 22-24 ms |

Implication: the currently pinned 20 Hz cadence (50 ms period) cannot hold at SF7/BW125 once full envelope overhead is included.

### 3.3 Oversubscription check

At current plan values:

- Control: about 92 ms x 20 Hz = about 1840 ms airtime per second.
- Heartbeat: about 80 ms x 20 Hz = about 1600 ms airtime per second.
- Telemetry adds additional load.

Total demand exceeds 1000 ms/s available channel time by a large factor. This creates chronic queueing, drops, and timeout artifacts even before retries/backoff.

### 3.4 Timing corrective options

| Option | Control PHY | Control rate | Practical status |
|---|---|---:|---|
| A | SF7/BW125 | 20 Hz | not feasible with current frame overhead |
| B | SF7/BW250 | 10 Hz | feasible, good v25 balance |
| C | SF7/BW500 | 20 Hz | feasible, shorter range |
| D | SF9/BW125 | 5 Hz | feasible, long range but sluggish control |

Recommended v25 baseline: **Option B** (SF7/BW250 at 10 Hz control), then validate field range against the v25 gate.

---

## 4. Half-Duplex Management

### 4.1 Core problem

SX1276 cannot receive while transmitting. It also cannot receive two PHY profiles simultaneously. Any design that assumes concurrent downlink control and uplink telemetry/video without scheduling will see collisions or starvation.

### 4.2 MAC architecture options

#### Option 1: Priority CSMA only

- Pros: simple, no time sync.
- Cons: nondeterministic latency under load, hidden-node risk with handheld/base/tractor interactions.

#### Option 2: Deterministic grant windows (recommended for v25)

- Base emits control frame every fixed period.
- Control frame includes uplink grant duration or uplink slot index.
- Tractor transmits telemetry/image chunks only inside grant window.
- If grant lost, tractor falls back to minimal opportunistic mode and re-syncs on next valid control frame.

Pros:
- Deterministic upper bound on control latency.
- Predictable half-duplex behavior with a single radio.
- Easy to implement with existing architecture.

Cons:
- Slightly more control-frame logic.
- Tractor-initiated bursts are aligned to grant windows.

#### Option 3: Full TDMA superframe

- Best determinism, clean scaling.
- Requires stronger timebase discipline and tighter implementation rigor.
- Better candidate for v26+.

### 4.3 Recommended v25 superframe sketch (100 ms cycle)

- 0-40 ms: base -> tractor control slot (P0).
- 40-45 ms: guard/CAD.
- 45-85 ms: tractor -> base telemetry/image chunk slot.
- 85-100 ms: reserve/command/recovery slot.

This aligns naturally with 10 Hz control and provides explicit uplink opportunities without blind contention.

### 4.4 Adaptive SF handling under half-duplex

Current dual-transmit link-tune behavior (send tune frame at old and new SF back-to-back) inflates airtime at the worst possible time.

Safer method:

1. Announce target SF/BW in several consecutive frames at current PHY.
2. Include `switch_at_sequence = N`.
3. Both sides switch at N.
4. Revert only after a clean timeout policy with hysteresis.

Avoid dual-PHY back-to-back transmissions in degraded conditions.

---

## 5. Payload Optimization

### 5.1 Priority before compression

First optimization should remove redundant traffic:

1. Carry heartbeat/liveness inside control stream whenever control is active.
2. Use standalone heartbeat only during idle/no-control periods at low rate.
3. Aggregate low-priority telemetry rather than many small frames.

This often saves more airtime than micro-optimizing fields.

### 5.2 QoS classing (practical)

| Class | Typical content | Rule |
|---|---|---|
| P0 | control, estop, safety command | always first, no retry storm |
| P1 | core state telemetry | send at bounded rate, drop oldest on pressure |
| P2 | bulk diagnostics/log slices | opportunistic, aggressive drop under load |
| P3 | image chunks | strictly best-effort, zero ability to starve P0/P1 |

### 5.3 Frame-level byte savings

Potential optimizations, in order of implementation risk:

1. **Field packing**:
- Compact joystick/control fields to practical precision.
- Collapse sparse flags.

2. **Telemetry delta encoding**:
- Send deltas for slow-changing values (pressure, battery, temperatures).
- Send periodic full snapshots for resync.

3. **Topic aggregation**:
- Combine multiple low-rate sensor points into a single telemetry payload.

4. **Security envelope optimization (advanced)**:
- Consider implicit nonce reconstruction and shorter auth tag only after threat-model review.
- These can recover significant bytes but must be validated carefully.

### 5.4 Suggested payload policy for v25

- Control frame target: keep compact and fixed-size.
- Telemetry frame target: 24-48 bytes typical.
- Image chunk payload target: 40-80 bytes per chunk to bound head-of-line blocking.

---

## 6. Image/Video Over LoRa: What Is Realistic

### 6.1 Reality check

LoRa should carry:

- Safety-critical commands and control snapshots.
- Core telemetry.
- Slow situational imagery.

LoRa should not be treated as a primary live video transport.

### 6.2 Practical transmission modes

| Mode | Data sent | Typical payload budget | Use case |
|---|---|---:|---|
| M0 | metadata only (detections/events) | 20-120 B/event | best safety bandwidth efficiency |
| M1 | micro-thumbnail (grayscale) | 300-1200 B/image | proof-of-view every few seconds |
| M2 | ROI thumbnail | 150-600 B/update | focus only on implement/hazard region |
| M3 | keyframe + sparse deltas | keyframe 1-2 KB, deltas 80-300 B | static scenes with occasional motion |

Recommended default: M1 plus event metadata, with M2 when operator selects implement monitor mode.

### 6.3 Tractor-side shrink pipeline

1. Capture from UVC stream.
2. Downscale hard (for example 96x64 or 128x72).
3. Optional grayscale for low-link mode.
4. Compress (JPEG/WebP at aggressive quality).
5. Chunk and enqueue as P3 frames.
6. Gate on link state and queue pressure.

### 6.4 Adaptive image ladder

- Good link: 128x72 color, moderate compression, faster refresh.
- Marginal link: 96x64, heavier compression, slower refresh.
- Poor link: grayscale ROI only, low refresh.
- Critical load: image off, metadata-only.

---

## 7. Base-Station Manipulation and Enhancement Strategy

The base station should reconstruct operator-usable visuals from very small transmitted payloads.

### 7.1 Decode and reassembly

- Validate chunk continuity and timestamp.
- Reassemble full thumbnail object.
- Expose staleness and completion percent to UI.

### 7.2 Enhancement stack (base side)

1. **Denoise/deblocking**: reduce compression artifacts.
2. **Super-resolution**: upsample low-res images to panel-friendly resolution.
3. **Temporal stabilization**: smooth frame-to-frame jitter.
4. **Contrast/edge enhancement**: improve implement and obstacle visibility.
5. **Object overlays**: run lightweight detection and annotate results.

### 7.3 Safety UX rules

- Always display frame age (stale timer).
- Visually mark enhanced/synthesized frames.
- Keep last-known-good frame but indicate no fresh data.
- Never hide packet-loss state from operator.

This prevents over-trust in AI-enhanced imagery.

---

## 8. Recommended v25 Implementation Plan

### Phase 0: Timing correction and profile freeze

1. Correct airtime numbers in protocol/plan docs.
2. Freeze one viable control profile (recommended SF7/BW250 @ 10 Hz).
3. Remove 20 Hz standalone heartbeat while control is active.

### Phase 1: Half-duplex determinism

1. Implement grant-window scheduler.
2. Add queue classes P0-P3.
3. Add measurable latency instrumentation.

### Phase 2: Payload reduction

1. Field packing and telemetry aggregation.
2. Delta telemetry where appropriate.
3. Optional advanced crypto-envelope optimization only after review.

### Phase 3: Image pipeline

1. Tractor micro-thumbnail/ROI chunking.
2. Base reassembly and enhancement pipeline.
3. UI confidence/staleness indicators.

---

## 9. Validation Metrics

Use these metrics to decide if the LoRa stack is ready for hydraulic testing:

1. P99 control-frame inter-arrival at tractor within target profile bound.
2. P99 base estop command latency below gate threshold.
3. Zero P3-induced P0 starvation over 30-minute stress test.
4. Stable operation through induced packet-loss scenarios.
5. Image pipeline never consumes reserved P0/P1 airtime budget.

---

## 10. Key Decisions Requiring Explicit Sign-Off

1. Which control profile is canonical for v25 field tests.
2. Whether to adopt grant-window scheduling now or after first bench round.
3. Whether advanced security-envelope byte reductions are acceptable for v25.
4. Target role of LoRa imagery: proof-of-view only vs enhanced situational feed.

---

## 11. Bottom Line

The LoRa plan can be made robust, but only if timing realism drives protocol choices.

The minimum successful recipe is:

1. Pick a feasible control profile.
2. Enforce deterministic half-duplex scheduling.
3. Protect P0 traffic with strict airtime governance.
4. Treat imagery as sparse data for base-side enhancement, not as native video.

With that foundation, v25 can deliver reliable long-range control and useful visual awareness without pretending LoRa can carry conventional video.
