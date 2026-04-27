# LifeTrac v25 — LoRa System Implementation Plan

> **Status (2026-04-27):** Canonical implementation plan for the v25 LoRa stack across handheld, tractor, and base. Mirrors the structure of [IMAGE_PIPELINE.md](IMAGE_PIPELINE.md). **This document is the source of truth for *what to build* and in what order; [LORA_PROTOCOL.md](LORA_PROTOCOL.md) is the source of truth for the wire bytes; [MASTER_PLAN.md §8.17](MASTER_PLAN.md) is the source of truth for the PHY policy pins.**
>
> **Headline:** Three-source priority-arbitrated point-to-point LoRa with adaptive SF, three concurrent PHY profiles (control / telemetry / image) on one SX1276 per node, AES-128-GCM authenticated payloads, FCC §15.247 compliance via 8-channel FHSS, and a strict 25 ms-per-fragment airtime cap protecting P0 ControlFrame TX-start. **Total airtime budget on the shared 915 MHz channel sequence: ≤ 60 % steady-state across all three sources combined; P0/P1 always preempt.**

---

## 1. Scope and constraints

### 1.1 What this plan covers

- The full handheld ↔ base ↔ tractor LoRa link: PHY, MAC (CSMA + FHSS), framing (KISS+CRC+AES-GCM), protocol, arbitration, security, observability.
- Bring-up, validation gates, and 10-engineer-week build order for everything that lives below the application/topic layer.
- Cross-cutting: priority queue (P0–P3), adaptive SF ladder, image-link PHY split, telemetry fragmentation, FHSS hop sequence, key provisioning + rotation.

### 1.2 What this plan does NOT cover

- **Image-pipeline content** (tile-delta encoding, ROI policy, browser tier) — see [IMAGE_PIPELINE.md](IMAGE_PIPELINE.md). This plan covers only how its bytes get on the air.
- **MQTT bridging on the LAN side** of the base station — see [BASE_STATION.md](BASE_STATION.md).
- **Hydraulic / valve / Modbus** logic — see [TRACTOR_NODE.md](TRACTOR_NODE.md) and [Phase 4.5](TODO.md).
- **Cellular fallback** — archived per [TODO.md scope note](TODO.md). LoRa-only for v25.
- **LoRaWAN** — explicitly rejected per [LORA_PROTOCOL.md § Why not LoRaWAN](LORA_PROTOCOL.md#why-not-lorawan).

### 1.3 Hard constraints (do not violate)

| # | Constraint | Source |
|---|---|---|
| L1 | **Zero P0 TX-start delay > 25 ms** attributable to any in-flight P2/P3 fragment. Same constraint as image C1, applied to the whole stack. | [MASTER_PLAN.md §8.17](MASTER_PLAN.md), [IMAGE_PIPELINE.md C1](IMAGE_PIPELINE.md) |
| L2 | **No ACK/retry on `ControlFrame`** — the next frame in 50 ms *is* the retry. | [LORA_PROTOCOL.md](LORA_PROTOCOL.md) |
| L3 | **Failsafe within 500 ms** of last valid heartbeat → `SOURCE_NONE` → valves neutral. | [LORA_PROTOCOL.md § Multi-source arbitration](LORA_PROTOCOL.md#multi-source-arbitration) |
| L4 | **AES-128-GCM authenticated encryption** on every frame; replay-rejected via per-source monotonic sequence + nonce. | [LORA_PROTOCOL.md](LORA_PROTOCOL.md) |
| L5 | **FCC §15.247 compliance via FHSS**, not via widening to BW ≥ 500 kHz. 8 channels across 902–928 MHz. | [MASTER_PLAN.md §8.17 FHSS bullet](MASTER_PLAN.md), [IMAGE_PIPELINE.md §13.1 Revisit-5](IMAGE_PIPELINE.md) |
| L6 | **Failsafe is a hardware backstop, not just software** — Phoenix Contact PSR safety relay + engine-kill relay drop power even if firmware hangs. | [TRACTOR_NODE.md](TRACTOR_NODE.md), [HARDWARE_BOM.md](HARDWARE_BOM.md) |
| L7 | **Same firmware across MKR WAN 1310 and Portenta** (only pin map changes). | [LORA_PROTOCOL.md](LORA_PROTOCOL.md), [ARCHITECTURE.md](ARCHITECTURE.md) |
| L8 | **Bench-measure, do not assume**, the cost of `CMD_LINK_TUNE` retunes (R-7), TX→RX turnaround, and CSMA backoff. Numbers feed the airtime ledger. | [IMAGE_PIPELINE.md §13.3 R-7](IMAGE_PIPELINE.md) |

---

## 2. The shipped stack

| Layer | Pick | Phase |
|---|---|:-:|
| **Modem** | Semtech SX1276 via Murata SiP on MKR WAN 1310 (handheld) and Portenta Max Carrier (tractor + base) | 0 |
| **PHY** | Three coexisting profiles on one radio per node, retuned per-frame: Control SF7/BW125/CR4-5, Telemetry SF9/BW250/CR4-8, Image SF7/BW250/CR4-5 | 1 |
| **MAC** | CSMA (RSSI busy-detect + random backoff) + 8-channel FHSS, deterministic hop sequence seeded by AES key ID | 1 |
| **Framing** | KISS (FEND 0xC0 / FESC 0xDB byte-stuffing) + CRC-16/CCITT trailer | 2 |
| **Crypto** | AES-128-GCM via MbedTLS (Arduino core for both SAMD21 and STM32H7) — 12 B nonce + 16 B tag | 2 |
| **Replay defence** | Per-source monotonic `sequence_num` + per-source last-seen window | 2 |
| **Frame types** | `ControlFrame` (0x10), `TelemetryFrame` (0x20), `Command` (0x30), `Heartbeat` (0x40) | 2 |
| **Priority queue** | P0 (control + commands + person alert) / P1 (key cmds, ROI hint) / P2 (telemetry) / P3 (image fragments) — see [§4](#4-priority-and-airtime) | 3 |
| **Arbitration** | Tractor `pick_active_source()` on heartbeat staleness + take-control latch | 4 |
| **Adaptive SF ladder** | SF7 → SF8 → SF9 step-down on bad-window count; 30 s clean-link before step-up; 3-window hysteresis (R-8) | 5 |
| **Auto-fallback ladder (image)** | `link_monitor.py` + `CMD_ENCODE_MODE` per [IMAGE_PIPELINE.md §3.4](IMAGE_PIPELINE.md) — driven by airtime % `U` | 5 |
| **Observability** | Telemetry topic `0x10` + per-source RSSI/SNR/loss-rate + audit log of every SF/encode-mode transition | 5 |

**Hardware delta vs. v25 baseline:** $0 — radios already on the BOM. **Coral required:** No.

---

## 3. PHY policy and the three-profile split

The single SX1276 per node hosts **three coexisting PHY profiles** that the radio retunes between on a per-frame basis. The retune mechanism is the existing `CMD_LINK_TUNE` opcode (`0x21`) repurposed to also signal the per-frame PHY swap.

| Profile | SF | BW | CR | Used for | Airtime per typical frame |
|---|---|---|---|---|---|
| **Control** | 7 (adaptive 7→8→9) | 125 kHz | 4/5 | `ControlFrame`, `Heartbeat`, P0 commands, `CMD_PERSON_APPEARED` | 44 B encrypted `ControlFrame` → ~92 ms @ SF7, ~164 ms @ SF8, ~288 ms @ SF9. **Blocker:** does not fit 20 Hz cadence. |
| **Telemetry** | 9 | 250 kHz | 4/8 | `TelemetryFrame` topics `0x01`–`0x10`, `0x26` detections, `0x27` audio events | 64 B → ~250 ms (fragmented to ≤25 ms cap per R-6) |
| **Image** | 7 (opt-in 8) | 250 kHz | 4/5 | `TileDeltaFrame` (`0x25`), motion vectors (`0x28`), wireframe (`0x29`) | 32 B fragment → ~36 ms @ SF7/BW250. **Blocker:** misses 25 ms fragment cap; ≤15 B is the current estimator limit. |

**Why split.** Telemetry default of SF9/BW250/CR4-8 makes a 32 B image fragment ~120 ms on-air, violating L1. Image therefore gets its own SF7 profile retuned per-frame. SF8 image is opt-in only because it violates L1. See [LORA_PROTOCOL.md § PHY parameters](LORA_PROTOCOL.md#phy-parameters-defaults) and [IMAGE_PIPELINE.md §13.1 Revisit-1](IMAGE_PIPELINE.md).

**Why CR4-5 for image.** Application-layer recovery (canvas + `CMD_REQ_KEYFRAME` + auto-fallback ladder) substitutes for FEC. CR4-8 was a ~37 % airtime tax for redundancy already provided. Image now CR4-5; CR4-8 retained for `0x26` and `0x27` where loss would silently hide a safety alert. See [IMAGE_PIPELINE.md §13.1 Revisit-2](IMAGE_PIPELINE.md).

**Control cadence blocker (2026-04-27 implementation review).** The SF7/BW125 control profile and the 20 Hz no-retry `ControlFrame` cadence are internally inconsistent with AES-GCM overhead. The shared Python estimator reports ~92 ms for the encrypted 44-byte control payload at SF7/BW125, before KISS expansion and CSMA. Week-1 bring-up must decide whether to widen the control profile, reduce cadence, reduce framing/security overhead, or move control to a dedicated radio/channel before any moving-hydraulics test.

**Image fragment cap blocker (2026-04-27 implementation review).** The same estimator reports ~36 ms for a 32 B SF7/BW250 image fragment. The documented 25 ms cap therefore requires smaller fragments (roughly ≤15 B cleartext at the current profile), a wider image profile, or a revised cap. The image pipeline should not be marked ready until this is resolved and bench-measured.

### 3.1 Adaptive control-link SF ladder

Per [MASTER_PLAN.md §8.17](MASTER_PLAN.md):

- **Step-down (good→bad):** require N=3 consecutive bad 5 s windows (R-8) of SNR margin < threshold or loss > threshold before SF7→SF8→SF9 step. Each transition announced via `CMD_LINK_TUNE` sent twice back-to-back (once at old SF/BW, once at new), then both ends switch. If no Heartbeat at the new SF within 500 ms → revert + increment fail counter.
- **Step-up (bad→good):** require ≥ 30 s of clean link AND N=3 consecutive good 5 s windows (R-8) before SF9→SF8→SF7 step. Same `CMD_LINK_TUNE` handshake.
- **Telemetry SF is independent** of control SF — telemetry stays on its own profile and does not retune with the control ladder.
- **Image SF auto-fallback ladder is also independent** — driven by airtime % `U` per [IMAGE_PIPELINE.md §3.4](IMAGE_PIPELINE.md), with its own 3-window hysteresis.

### 3.2 FHSS hop sequence

Per [MASTER_PLAN.md §8.17 FHSS bullet](MASTER_PLAN.md):

- 8 channels evenly spaced across 902–928 MHz at 3.25 MHz spacing (satisfies §15.247(a)(1)(iii) ≥ 6 × 20 dB-bandwidth requirement for the widest 250 kHz profile).
- Deterministic shared pseudo-random sequence seeded by the **AES key ID** (so all three nodes hop identically without exchanging hop tables on the air).
- Per-channel dwell ≈ 1.5 s per ~12 s sequence cycle = ~12.5 % per channel. Well under the 400 ms-per-channel-per-20 s §15.247(a)(1)(i) limit.
- **CSMA still applies:** if the next-hop channel reads busy at RSSI > -90 dBm threshold, skip to the channel after it and log the skip for the audit ledger.
- Hop sequence implementation lives in `firmware/common/lora_proto.cpp` and is shared by all three nodes.

---

## 4. Priority and airtime

Three concurrent traffic classes share one half-duplex channel. The priority queue is the only thing keeping `ControlFrame` cadence honest under image load.

| Class | Sources | Examples | Preempts | Cap |
|---|---|---|---|---|
| **P0** | handheld, base, tractor | `ControlFrame`, `Heartbeat`, `CMD_ESTOP`, `CMD_LINK_TUNE`, `CMD_PERSON_APPEARED` | All | None |
| **P1** | handheld, base | `CMD_CLEAR_ESTOP`, `CMD_ROI_HINT`, `CMD_REQ_KEYFRAME`, `CMD_CAMERA_SELECT` | P2, P3 | None |
| **P2** | tractor, base | `TelemetryFrame` topics `0x01`–`0x10`, `0x26`, `0x27` | P3 | **25 ms-per-fragment airtime cap (R-6)** — oversized topics fragment using the `TileDeltaFrame` scheme |
| **P3** | tractor | `TileDeltaFrame` (`0x25`), motion vectors (`0x28`), wireframe (`0x29`) | — | **25 ms-per-fragment airtime cap (L1)** |

**Burst-batching (R-7 mitigation).** If `CMD_LINK_TUNE` retune cost is bench-measured > 5 ms, image fragments batch so the radio retunes at most twice per refresh window (once into image PHY, once back to telemetry PHY) instead of per fragment. Plan the code path conservatively in `lora_proto.cpp` until measured.

**Airtime ledger.** `link_monitor.py` keeps a rolling 10 s sum of on-air ms per profile and exposes:

- `U_image = t_air_image / T_refresh` (drives the image auto-fallback ladder per [IMAGE_PIPELINE.md §3.4](IMAGE_PIPELINE.md)).
- `U_telemetry = t_air_telemetry / 10 s` (sanity-check; alarm if > 30 %).
- `U_total = (t_air_control + t_air_telemetry + t_air_image) / 10 s` (alarm if > 60 % — leaves headroom for P0 bursts).

All three values published on telemetry topic `0x10` (`control/source_active`) so the operator UI can show airtime utilisation alongside RSSI/SNR.

---

## 5. Multi-source arbitration

Tractor maintains `SourceState[3]` (HANDHELD / BASE / AUTONOMY) per [LORA_PROTOCOL.md § Multi-source arbitration](LORA_PROTOCOL.md#multi-source-arbitration). Build requirements:

- `pick_active_source()` runs every loop iteration on the M7 (50 Hz). Selects the highest-priority source whose `last_heartbeat_ms < HEARTBEAT_TIMEOUT_MS` (500 ms) and whose `last_sequence` is monotonic. If none qualify → `SOURCE_NONE` → valves neutral within next M7 tick (≤ 20 ms).
- **TAKE CONTROL latch:** when a source asserts `take_control_held` flag in its Heartbeat, it gets P0 priority over other sources for 30 s after the button releases. Latch persists across heartbeat misses but not across `SOURCE_NONE`.
- **Replay defence:** per-source last-seen `sequence_num` window of 64. Frames with sequence ≤ window-low or ≥ window-high + 256 rejected. Window slides on valid frames.
- **Source-active publish:** topic `0x10` carries current `active_source`, last RSSI/SNR per source, and current SF rung. Published at 1 Hz + on every change.
- **Audit log:** every source transition (incl. `SOURCE_NONE` entries and exits) appended to the §8.10 black-box logger with wall-clock + reason.

---

## 6. Security model

| Layer | Mechanism | Implementation file |
|---|---|---|
| Confidentiality | AES-128-GCM, 12 B nonce, 16 B tag | `firmware/common/crypto.cpp` |
| Authenticity | GCM tag verification (rejects bit flips) | `firmware/common/crypto.cpp` |
| Replay defence | Per-source monotonic `sequence_num` + 64-frame window | `firmware/common/lora_proto.cpp` |
| Nonce uniqueness | `source_id (1 B) ‖ sequence_num (2 B) ‖ epoch_s_low24 (3 B) ‖ random (6 B)` = 12 B | `firmware/common/crypto.cpp` |
| Key provisioning | `tools/provision.py` writes pre-shared 16 B key + 4 B key ID via USB-CDC at first-boot | `tools/provision.py` |
| Key rotation | Documented manual procedure: provision new key, both sides switch on next power cycle, old key zeroised; **no over-the-air key exchange in v25** | [`KEY_ROTATION.md`](KEY_ROTATION.md) (NEW, see §11) |
| FCC ID transparency | AES key ID (4 B) seeds the FHSS sequence — same key ID = same hop pattern = same network = audit-traceable | `firmware/common/lora_proto.cpp` |

**Threat model (v25):** physical access to handheld/base = trusted operator (per [MASTER_PLAN.md §8.5](MASTER_PLAN.md)). LoRa air-interface defends against passive eavesdropping, spoofing, replay, and tampering — not against an attacker who has stolen a node and extracted the key.

---

## 7. Observability

Mandatory in Phase 2; not deferable.

- **`link_monitor.py`** (base, Linux) — rolling airtime ledger per profile; computes `U_image`, `U_telemetry`, `U_total`; emits `CMD_ENCODE_MODE` per the auto-fallback ladder; surfaces all three `U` values on telemetry topic `0x10`.
- **Per-source link health** — RSSI, SNR, loss rate, last-heard wall-clock, current SF rung — published on topic `0x10` at 1 Hz.
- **Audit log** (every transition, never sampled): SF ladder transitions, encode-mode transitions, FHSS skip-busy events, source arbitration changes, replay rejections, GCM-tag rejections, CSMA backoff events. Appended to the [§8.10 black-box logger](MASTER_PLAN.md).
- **Operator UI surface** — airtime % bar (green < 40 %, yellow 40–60 %, red > 60 %); current SF rung pill; FHSS hop indicator (channel 1–8); two-source-active banner if both handheld and base are within heartbeat timeout.
- **Bench instrumentation** — RTT measurement harness in `tools/lora_rtt.py` (handheld → tractor → base round-trip via timestamp echo). Run nightly during build weeks 1–10.

---

## 8. Build order (10-engineer-week plan)

Mirrors the IMAGE_PIPELINE.md week numbering so the two plans interleave correctly. Phase 1 of this plan must complete by the end of week 1 because IMAGE_PIPELINE.md week-1 deliverables depend on it.

> **Status note (2026-04-27):** ✅ marks items already shipped in the workspace at the time this plan was written. The build order is retained so a future contributor can see the original sequence; ✅ entries can be skipped during fresh bring-up.

| Week | Deliverable | Status | Phase |
|---|---|---|:-:|
| 1 | **Bench bring-up** ([Phase 1](TODO.md#phase-1--bench-bring-up)) — all three nodes hear each other at SF7/BW125/CR4-5/915.0/sync 0x12. Range smoke test at 1 m. | open | 0 |
| 1 | **R-7 retune-cost bench measurement.** Measure `setFrequency` + `setSpreadingFactor` + `setBandwidth` + `setCodingRate` cost on SX1276. If > 5 ms, enable burst-batching code path. | partial — [`firmware/bench/lora_retune_bench/`](firmware/bench/lora_retune_bench/) sketch exists; not yet bench-run | 0 |
| 1–2 | **Common firmware** ([Phase 2](TODO.md#phase-2--common-firmware-shared-by-all-three-nodes)) — KISS framer, CRC-16, frame structs, `lora_proto_encode/decode`, AES-GCM wrapper, nonce generator, replay window, unit tests. | ✅ [`firmware/common/lora_proto/`](firmware/common/lora_proto/) (`lora_proto.h`, `lora_proto.c`, `crypto_stub.c`) + Python mirror [`base_station/lora_proto.py`](base_station/lora_proto.py) + tests in [`base_station/tests/`](base_station/tests/) | 1 |
| 2 | **`provision.py`** key provisioning utility. Document key rotation in `KEY_ROTATION.md`. | ✅ tool at [`tools/provision.py`](tools/provision.py); doc still pending L-O2 | 1 |
| 2 | **FHSS hop sequence** in `lora_proto.cpp` — 8 channels, AES-key-ID-seeded sequence, CSMA skip-busy. | ✅ hop sequence in `lp_fhss_channel_index` / `lp_fhss_channel_hz` (C) and `fhss_channel_index` / `fhss_channel_hz` (Python). CSMA skip-busy helper ✅ — `lp_csma_pick_hop()` (C) and `pick_csma_hop()` (Python), default –90 dBm / 4 skips, unit-tested. Caller wiring into firmware TX path + audit-log hook still open. | 1 |
| 2–3 | **Priority queue** in `lora_proto.cpp` — P0/P1/P2/P3 classes, preemption rules, 25 ms-per-fragment cap (covers L1 and R-6). | partial — base-station heap-based priority TX queue ✅ in [`base_station/lora_bridge.py`](base_station/lora_bridge.py) (single `_tx_worker`, classification via `classify_priority()` in [`base_station/lora_proto.py`](base_station/lora_proto.py), unit-tested). Firmware-side queue (handheld + tractor) and the 25 ms cap enforcement still open. | 1 |
| 3 | **Telemetry fragmentation (R-6)** — extend `TileDeltaFrame` fragment scheme to oversized `TelemetryFrame`; base reassembly in `lora_bridge.py` before MQTT publish. | open | 1 |
| 3 | **Handheld firmware** ([Phase 3](TODO.md#phase-3--handheld-firmware)) — 50 Hz `ControlFrame` TX, OLED status, TAKE CONTROL latch, E-STOP, LiPo charging. | partial — [`firmware/handheld_mkr/handheld.ino`](firmware/handheld_mkr/handheld.ino) skeleton exists | 1 |
| 3–4 | **Tractor M7 firmware** ([Phase 4](TODO.md#phase-4--tractor-firmware)) — `pick_active_source()`, source-state tracking, failsafe at 500 ms, valve neutral handoff. | partial — [`firmware/tractor_h7/tractor_m7.ino`](firmware/tractor_h7/tractor_m7.ino) has `pick_active_source()` + `HEARTBEAT_TIMEOUT_MS = 500`; adaptive SF ladder still TODO at line 263 | 1 |
| 4 | **Bench gate B1** — three-source arbitration test on the bench (see §9). **Image-pipeline week-1 dependencies satisfied.** | open | 1 |
| 4–5 | **`CMD_LINK_TUNE` adaptive SF ladder** with R-8 hysteresis (3 windows in either direction; 30 s clean for step-up). | ✅ — state machine in [`firmware/tractor_h7/tractor_m7.ino`](firmware/tractor_h7/tractor_m7.ino) `poll_link_ladder()`: 5 s windows on the active source's HB count (bad < 80 %, good ≥ 96 %), 3 bad → SF↑ (`LTR_HIGH_LOSS`), 6 good → SF↓ (`LTR_RECOVERY_DOWN`); twice-back-to-back `CMD_LINK_TUNE` send + 500 ms revert deadline + fail counter. | 1 |
| 5 | **`link_monitor.py`** — rolling airtime ledger, `CMD_ENCODE_MODE` emitter, telemetry `0x10` publisher. **Image-pipeline week-5 dependency satisfied.** | ✅ — [`base_station/link_monitor.py`](base_station/link_monitor.py) (`RollingAirtimeLedger` + `EncodeModeController`, 3-window hysteresis) is now wired into [`lora_bridge.py`](base_station/lora_bridge.py): every TX/RX records airtime via `attribute_phy()`; a 1 Hz worker emits `CMD_ENCODE_MODE` on rung change and publishes the `(U_image, U_telemetry, U_total)` triple as retained JSON on `lifetrac/v25/control/source_active`; alarm logs fire on `U_telemetry > 30 %` and `U_total > 60 %`. | 1 |
| 5–6 | **Base station LoRa bridge** ([Phase 5](TODO.md#phase-5--base-station-firmware-linux-side-runs-in-docker-on-x8)) — `lora_bridge.py` SPI + decode + decrypt + MQTT republish + telemetry forward. Port `lora_proto.cpp` → `lora_proto.py` and `crypto.cpp` → `crypto.py`. | partial — [`base_station/lora_bridge.py`](base_station/lora_bridge.py) over serial works; SPI driver pending; airtime ledger not yet integrated | 1 |
| 6 | **FHSS spectrum-analyser bench-verification** — confirm per-channel dwell ≤ 12.5 %, all 8 channels active, no out-of-band emissions. | open | 1 |
| 6–7 | **Mast antenna installation** ([Phase 6](TODO.md#phase-6--mast-antenna-installation-base-station)). VNA SWR check < 2:1. | open | 2 |
| 7–9 | **Integration testing** ([Phase 7](TODO.md#phase-7--integration--end-to-end-testing)) — single-source, two-source, handover, TAKE CONTROL, failsafe, replay, tamper, latency. | open | 2 |
| 9–10 | **Field testing** ([Phase 8](TODO.md#phase-8--field-testing)) — 1/5/10/15 km range, foliage attenuation, vibration, IP rating, 24 h soak, 7 d deployment. | open | 2 |
| Continuous | **`tools/lora_rtt.py` nightly run.** Audit-log diff per night; flag regressions. | open | — |

### 8.1 Cuts if the schedule shrinks (in order)

1. **First out:** spectrum-analyser FHSS verification (defer to Phase 9 FCC verification — not a build blocker if the duty math holds).
2. **Next:** field-test 15 km / 10 km range tests (keep 1 km / 5 km as floor).
3. **Next:** 7-day deployment soak (keep 24 h soak).
4. **Defend with both arms:** L1 25 ms cap, L3 500 ms failsafe, L4 AES-GCM, L5 FHSS, three-source arbitration, adaptive SF ladder. These are the spine.

---

## 9. Validation gates (must all pass before field test)

| # | Gate | Target |
|---|---|---|
| L-V1 | **L1 P0 starvation under load** — 30-min mixed-mode stress (control + telemetry + image), zero P0 TX-start delays > 25 ms. | Same as image V1 — joint pass. |
| L-V2 | **L3 failsafe** — power-off active source mid-operation, tractor reaches valve-neutral within 500 ms p99. | Repeat for HANDHELD, BASE, AUTONOMY. |
| L-V3 | **Three-source arbitration** — handheld + base both heartbeating, tractor follows handheld; handheld released, follows base after 30 s latch + 500 ms timeout. | Bench. |
| L-V4 | **TAKE CONTROL preemption** — base controlling, handheld TAKE CONTROL button → tractor switches within next M7 tick (≤ 20 ms). | Bench. |
| L-V5 | **Replay rejection** — capture a frame, retransmit later, tractor rejects + logs. | Bench. |
| L-V6 | **Tamper rejection** — flip a bit in a captured frame, retransmit, GCM tag fails, tractor rejects + logs. | Bench. |
| L-V7 | **Adaptive SF ladder** — attenuate the link in 3 dB steps, verify SF7→SF8→SF9 step-down with 3-window hysteresis (no hunting at boundaries). Restore link, verify 30 s clean before step-up. | Bench with step attenuator. |
| L-V8 | **R-6 telemetry fragmentation** — publish a 120 B telemetry payload, confirm fragmentation + base reassembly + MQTT republish identical to source bytes. | Bench. |
| L-V9 | **R-7 retune cost** — bench-measured retune cost recorded; if > 5 ms, burst-batching code path active and validated under image load. | Bench. |
| L-V10 | **L5 FHSS compliance** — spectrum analyser confirms 8 channels active, ≤ 12.5 % per-channel dwell over 60 s window, no out-of-band emissions, EIRP ≤ +36 dBm. | Pre-Phase-9 FCC verification. |
| L-V11 | **Latency** — handheld → tractor valve ≤ 150 ms p99; base → tractor valve ≤ 250 ms p99. | Bench, then field. |
| L-V12 | **Field range** — base mast → tractor at 1 km LoS minimum; 5 km nice-to-have. Handheld → tractor 500 m minimum. | Field. |

---

## 10. Open scope decisions (need a human stakeholder)

| # | Decision | Deadline | Default if undecided |
|---|---|---|---|
| L-O1 | **Region** — US 915 MHz only for v25, or also EU 868 MHz? Affects FHSS channel plan + per-region key-ID prefix. | Before week 1 (Phase 0 procurement). | US 915 MHz only; EU port deferred to v25.5. |
| L-O2 | **Key rotation cadence** — annual / on-incident-only / never. Affects whether `provision.py` ships with operator-runnable docs or stays a workshop tool. | Before week 2. | On-incident-only; documented but not scheduled. |
| L-O3 | **Two-radio split (Revisit-4)** — adopt the v26 escape hatch in v25 if the L-V1 / image-V1 joint gate fails in field test, or hold the line and downshift the encoder more aggressively? | Only if a field-test gate fails. | Hold the line; revisit only on documented failure. See [MASTER_PLAN.md §8.17.1](MASTER_PLAN.md#8171-two-radio-split--documented-not-adopted-for-v25). |

---

## 11. Cross-document propagation

Files this plan touches; updates needed when this plan changes:

- [LORA_PROTOCOL.md](LORA_PROTOCOL.md) — wire bytes only; this plan refers, does not duplicate. Reservations from IMAGE_PIPELINE.md §3.2 (`0x28`/`0x29`/`0x2A`/`0x63`/badge enum) **✅ already cascaded** — see [LORA_PROTOCOL.md topic table](LORA_PROTOCOL.md#telemetryframe-variable-9128-bytes), [opcode table](LORA_PROTOCOL.md#command-frame-opcodes), and the badge enum table at line 270.
- [MASTER_PLAN.md §8.17 / §8.17.1](MASTER_PLAN.md) — PHY policy pins, FHSS, R-6/R-7/R-8 bullets, two-radio escape hatch.
- [TRACTOR_NODE.md](TRACTOR_NODE.md) — M7 `pick_active_source()`, source-state struct, failsafe path.
- [BASE_STATION.md](BASE_STATION.md) — `lora_bridge.py`, `link_monitor.py`, MQTT topic mapping, audit-log persistence.
- [HANDHELD_REMOTE.md](HANDHELD_REMOTE.md) — TAKE CONTROL latch, OLED RSSI/SF rung display.
- [TODO.md](TODO.md) — Phase 1 bench, Phase 2 common firmware, Phase 3 handheld, Phase 4 tractor M7, Phase 5 base bridge — all already enumerated. Phase 2.LoRa block consolidates the implementation-plan-level tasks.
- **`KEY_ROTATION.md`** (NEW, week 2) — operator-facing key provisioning + rotation procedure. Companion to existing [`tools/provision.py`](tools/provision.py). Pending L-O2 decision.

---

## 12. Where to read further

| Topic | File |
|---|---|
| Wire bytes (frame layouts, opcodes, topic IDs) | [LORA_PROTOCOL.md](LORA_PROTOCOL.md) |
| PHY policy pins (SF, BW, CR, FHSS, airtime caps) | [MASTER_PLAN.md §8.17](MASTER_PLAN.md) |
| Two-radio v26 escape hatch | [MASTER_PLAN.md §8.17.1](MASTER_PLAN.md#8171-two-radio-split--documented-not-adopted-for-v25) |
| Image pipeline (consumer of P3 airtime) | [IMAGE_PIPELINE.md](IMAGE_PIPELINE.md) |
| Image-pipeline LoRa revisit history (Revisit-1/2/3/5/R-6/R-8) | [IMAGE_PIPELINE.md §13](IMAGE_PIPELINE.md) |
| QoS / bandwidth-management background | [AI NOTES/2026-04-26_LoRa_QoS_Bandwidth_Management.md](../AI%20NOTES/2026-04-26_LoRa_QoS_Bandwidth_Management.md) |
| In-depth LoRa analysis (v1.1, source of truth for *why*) | [AI NOTES/2026-04-27_LoRa_InDepth_Analysis_ClaudeOpus4_7_v1_1.md](../AI%20NOTES/2026-04-27_LoRa_InDepth_Analysis_ClaudeOpus4_7_v1_1.md) |
| Image transmission analysis (drove the L1 cap) | [AI NOTES/2026-04-27_Image_Transmission_InDepth_Analysis_ClaudeOpus4_7_v1_0.md](../AI%20NOTES/2026-04-27_Image_Transmission_InDepth_Analysis_ClaudeOpus4_7_v1_0.md) |
