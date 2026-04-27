# LifeTrac v25 LoRa In-Depth Analysis - Copilot v1.1

Date: 2026-04-27
Author: GitHub Copilot
Document version: Copilot v1.1
Status: Design analysis and recommendation note. No code changes are made by this document.

Primary references:
- [DESIGN-CONTROLLER/MASTER_PLAN.md](../DESIGN-CONTROLLER/MASTER_PLAN.md)
- [DESIGN-CONTROLLER/LORA_PROTOCOL.md](../DESIGN-CONTROLLER/LORA_PROTOCOL.md)
- [AI NOTES/2026-04-26_LoRa_QoS_Bandwidth_Management.md](2026-04-26_LoRa_QoS_Bandwidth_Management.md)
- [DESIGN-CONTROLLER/VIDEO_OPTIONS.md](../DESIGN-CONTROLLER/VIDEO_OPTIONS.md)
- [DESIGN-CONTROLLER/RESEARCH-CONTROLLER/VIDEO_COMPRESSION/README.md](../DESIGN-CONTROLLER/RESEARCH-CONTROLLER/VIDEO_COMPRESSION/README.md)
- Active draft implementation under [DESIGN-CONTROLLER/firmware/](../DESIGN-CONTROLLER/firmware/) and [DESIGN-CONTROLLER/base_station/](../DESIGN-CONTROLLER/base_station/)

This is a successor note to [2026-04-27_LoRa_InDepth_Analysis_Copilot_v1.0.md](2026-04-27_LoRa_InDepth_Analysis_Copilot_v1.0.md). It keeps the same core conclusion, but adds a more explicit timing budget, a v25 half-duplex operating model, and a broader set of image/video compression and base-side enhancement options.

---

## 1. Executive verdict

The LoRa direction is still sound for long-range tractor control, but the current v25 LoRa plan has one major physics problem: the pinned SF7/BW125 control profile is being combined with old BW500 timing expectations and with a 20 Hz control plus 20 Hz heartbeat cadence. On a single SX1276-class half-duplex radio, that combination cannot fit on the channel.

The situation is fixable. The strongest v25 path is:

1. Stop treating LoRa as a free-running bidirectional pipe. Treat it as a scheduled half-duplex resource.
2. Pick one first-build PHY/rate pair that has real airtime margin. Recommended: SF7/BW250/CR4/5, 10 Hz control, single PHY both directions for first light.
3. Merge liveness into the ControlFrame while active control is flowing. A standalone 20 Hz HeartbeatFrame is the easiest airtime to recover.
4. Add a strict P0/P1/P2/P3 scheduler with hard P0 priority and bounded P3 fragment size.
5. Keep LoRa imagery as low-rate situational awareness: thumbnails, ROI tiles, metadata, or semantic scene data. Do not call it live video.
6. Do enhancement at the base station: reassembly, deblocking, super-resolution, temporal smoothing, object overlays, and synthetic situational views. Mark enhanced or synthesized views clearly.

If those changes are made, LoRa can provide robust long-range control, core telemetry, E-stop command delivery, and useful fallback visual awareness. Without them, the system risks chronic self-interference, stale controls, and seconds-long P0 latency whenever telemetry or image data is active.

---

## 2. What is inconsistent today

### 2.1 The docs and code are using different PHY assumptions

Canonical policy in the master plan is SF7/BW125/CR4/5 for control, with adaptive SF7 -> SF8 -> SF9 fallback. The active firmware still initializes both handheld and tractor at BW500:

- `handheld_mkr/handheld.ino`: `radio.begin(915.0, 500.0, 7, 5, 0x12, 14)`
- `tractor_h7/tractor_m7.ino`: `radio.begin(915.0, 500.0, 7, 5, 0x12, 20)`

That is not a small documentation mismatch. BW500 has one quarter the symbol time of BW125. A timing budget that works at BW500 can be broken by roughly 4x when moved to BW125.

### 2.2 The protocol retains a separate high-rate heartbeat

The current ControlFrame already carries a `heartbeat_ctr` byte and source flags. The handheld still sends both:

- ControlFrame at 20 Hz
- HeartbeatFrame at 20 Hz

At BW125, the separate heartbeat costs almost as much airtime as control itself. It should become a low-rate idle/liveness frame, not a second 20 Hz stream while control is active.

### 2.3 The telemetry PHY is described as separate

The protocol describes control at SF7/BW125 and telemetry at SF9/BW250. With one SX1276, the system cannot listen to both at once. Separate PHYs are possible only if there is an explicit retune schedule. Without a schedule, the radio will miss packets while sitting on the other PHY.

For v25, the simplest safe rule is: one active PHY profile at a time, both directions, unless a later TDMA design explicitly schedules retune windows.

### 2.4 KISS is being used as if the RF path were a byte stream

KISS is useful on UART links because UART is a byte stream and needs frame boundaries. LoRa packets are already length-delimited at the radio API. If Linux talks to a dumb H747 modem over serial, KISS belongs on the Linux-to-H747 serial boundary. It does not necessarily need to be transmitted over RF.

Keeping KISS over RF is not fatal, but it costs at least 2 bytes per packet plus occasional byte-stuffing. More importantly, it encourages thinking of the radio as a stream rather than short scheduled packets.

### 2.5 Base radio ownership affects timing jitter

The master plan says base Linux drives the SX1276 directly over SPI. The active `lora_bridge.py` still says the H747 runs a `base.ino` radio wrapper and pipes KISS over UART. Either architecture can work, but half-duplex scheduling depends on knowing where timing decisions live.

Recommendation: for v25, choose one:

- Direct SPI from Linux: easier service deployment, but radio timing can suffer from Linux scheduling jitter unless the driver uses interrupts and short critical sections.
- H747 modem: better deterministic radio timing, but it adds a firmware target and must expose scheduler primitives to Linux.

For safety-critical timing, the tractor H747 remains the important side. The base can tolerate more jitter if the tractor enforces P0 priority and uplink grants.

---

## 3. Timing analysis

### 3.1 Airtime assumptions used here

The active frame envelope is approximately:

| Frame | Cleartext bytes | Security overhead | KISS min overhead | LoRa PHY payload estimate |
|---|---:|---:|---:|---:|
| ControlFrame | 16 | 12-byte nonce + 16-byte GCM tag | 2 | about 46 B |
| HeartbeatFrame | 10 | 12-byte nonce + 16-byte GCM tag | 2 | about 40 B |
| Compact telemetry, 12 B payload | 21 | 12-byte nonce + 16-byte GCM tag | 2 | about 51 B |
| Larger telemetry, 64 B payload | 73 | 12-byte nonce + 16-byte GCM tag | 2 | about 103 B |

The exact number varies with byte-stuffing and RadioLib settings, but these estimates are close enough to make the planning conclusion obvious.

### 3.2 Corrected airtime for current control and heartbeat frames

Approximate LoRa airtime for the current encrypted/KISSed packets, explicit header, PHY CRC on, preamble 8, CR4/5:

| PHY | 46 B ControlFrame | 40 B HeartbeatFrame |
|---|---:|---:|
| SF7/BW125 | about 92 ms | about 82 ms |
| SF8/BW125 | about 164 ms | about 144 ms |
| SF9/BW125 | about 288 ms | about 247 ms |
| SF7/BW250 | about 46 ms | about 41 ms |
| SF7/BW500 | about 23 ms | about 21 ms |

This is the core timing issue. The master plan's SF7/BW125 text repeats old BW500-class expectations. A 20 Hz loop has a 50 ms period. A single current-form ControlFrame at SF7/BW125 consumes about 92 ms before the heartbeat even exists.

### 3.3 Oversubscription at the canonical written cadence

Current written rates:

- ControlFrame: 20 Hz
- HeartbeatFrame: 20 Hz
- TelemetryFrame: 2 Hz nominal, 10 Hz burst on event

At SF7/BW125 for P0:

| Stream | Cadence | Airtime each | Airtime demand per second |
|---|---:|---:|---:|
| ControlFrame | 20 Hz | about 92 ms | about 1840 ms/s |
| HeartbeatFrame | 20 Hz | about 82 ms | about 1640 ms/s |
| Compact telemetry at SF9/BW250/CR4/8 | 2 Hz | about 246 ms | about 492 ms/s |
| Total before video/retries/backoff | | | about 3970 ms/s |
| Channel capacity | | | 1000 ms/s |

That is about 4x oversubscribed before image data, retries, CAD, random backoff, or retune cost. It is not a tuning problem; it is an impossible schedule.

### 3.4 Why the active BW500 code can look plausible on a bench

At the active firmware's BW500 setting:

| Stream | Cadence | Airtime each | Airtime demand per second |
|---|---:|---:|---:|
| ControlFrame | 20 Hz | about 23 ms | about 460 ms/s |
| HeartbeatFrame | 20 Hz | about 21 ms | about 420 ms/s |
| Compact telemetry, if same SF7/BW500 | 1 Hz | about 26 ms | about 26 ms/s |
| Total before extras | | | about 906 ms/s |

That is still crowded, but it can appear to work during short tests. This explains the drift: the code inherited a BW500 timing world, while the master plan moved to BW125 for range.

### 3.5 Viable v25 PHY/rate profiles

Do not try to make one profile cover every range and responsiveness need on day one. Pick one for first hardware validation, then add modes after it is measured.

| Profile | Control PHY | Control rate | Expected use | Feasibility |
|---|---|---:|---|---|
| A: Long range failsafe | SF8 or SF9/BW125 | 2-5 Hz | degraded link, slow motion, stop/neutral bias | feasible but sluggish |
| B: Balanced v25 first build | SF7/BW250 | 10 Hz | main field-test profile | recommended |
| C: Near-field fast stick | SF7/BW500 | 20 Hz | close-range implement work | feasible, shorter range |
| D: Current written plan | SF7/BW125 | 20 Hz | none | not feasible with current envelope |

Profile B is the best starting point. It halves airtime relative to BW125, keeps more range than BW500, and makes a clean 100 ms half-duplex cycle natural.

### 3.6 What Profile B looks like with basic cleanup

Assume:

- SF7/BW250/CR4/5, single PHY both directions.
- Control at 10 Hz.
- Heartbeat merged into active ControlFrame.
- Compact telemetry frames use grant windows.

Budget:

| Stream | Cadence | Airtime each | Airtime demand per second |
|---|---:|---:|---:|
| ControlFrame | 10 Hz | about 46 ms | about 460 ms/s |
| Idle/standalone heartbeat | only when no active control | low | low |
| Compact telemetry | 2 Hz | about 51 ms | about 102 ms/s |
| P0 command reserve | event-driven | about 30-50 ms | burst only |
| Margin for P1/P2/P3 and guards | | | about 350-400 ms/s |

That is not lavish, but it is real. It leaves space for telemetry and occasional image chunks without letting them steal control time.

### 3.7 Timing recommendations

1. Replace the written 20 Hz SF7/BW125 control assumption with a measured profile table.
2. Freeze Profile B for first field validation: SF7/BW250/CR4/5, 10 Hz control.
3. Keep BW500 as a deliberate near-field mode, not the hidden default.
4. Keep BW125/SF8/SF9 as degraded-link or range modes with lower control cadence and conservative speed limits.
5. Merge heartbeat into ControlFrame while active control exists.
6. Define latency gates in terms of packet inter-arrival and coil drop timing, not just nominal frame cadence.

---

## 4. Half-duplex management

### 4.1 The key rule

The SX1276 cannot transmit and receive at the same time. It also cannot receive SF7/BW125 control and SF9/BW250 telemetry at the same time. Once a packet starts, P0 cannot preempt it until the packet finishes.

This means priority queues alone are not enough. The system must also bound the maximum low-priority packet airtime and must decide when each side is allowed to speak.

### 4.2 Three possible MAC models

#### Model 1: Priority CSMA

Each node listens, checks channel activity, backs off if busy, and sends the highest-priority queued frame.

Pros:
- Simple.
- Works for sparse traffic.
- No clock sync.

Cons:
- No deterministic worst-case latency under load.
- Hidden-node risk with handheld/base/tractor geometry.
- Collisions become likely when tractor telemetry and base control are both active.
- Image chunks can still block RX once they are already on air.

Verdict: useful fallback, not strong enough as the main v25 MAC once imagery exists.

#### Model 2: Asymmetric grant windows

The active controller direction owns the timeline. In base-control mode, the base sends a control frame every 100 ms. That frame grants the tractor a short uplink window for telemetry/image chunks before the next control slot.

Example 100 ms cycle for Profile B:

| Window | Direction | Purpose |
|---|---|---|
| 0-50 ms | base/handheld -> tractor | P0 control or command |
| 50-55 ms | guard/CAD | radio turn-around and margin |
| 55-90 ms | tractor -> base | P1 telemetry, P2 diagnostics, one small P3 chunk if budget allows |
| 90-100 ms | reserve | recovery, jitter, emergency command opportunity |

The exact windows should be measured on hardware. The important point is the ownership model: the tractor only sends routine uplink traffic when it has a grant, and P3 never exceeds the grant.

Pros:
- No wall-clock sync required.
- Gives bounded half-duplex behavior.
- Easy to reason about in firmware.
- Makes P3 starvation of P0 impossible if fragment sizes obey the grant.

Cons:
- Tractor-initiated data waits for the next grant.
- Needs a little more header/control state.

Verdict: recommended for v25.

#### Model 3: Full TDMA superframe

All nodes share a timebase and fixed slots. This is the cleanest long-term answer, especially if the optional handheld and future autonomy source both become active.

Pros:
- Best determinism.
- Clean extension to more sources.
- Excellent for testing and formal latency budgets.

Cons:
- Requires reliable time sync and stricter implementation.
- More work before first hardware validation.

Verdict: good v26+ target, not needed for the first v25 bench/field path.

### 4.3 How handheld changes the grant model

Handheld has highest priority. When handheld is active, it should become the P0 schedule owner for control slots. The base becomes mostly receive-only/read-only, except for emergency E-stop. Tractor uplink grants can still exist, but the base should not keep emitting control traffic.

Practical rule:

- HANDHELD active: handheld owns P0 control slots; base UI controls disabled; base E-stop still allowed.
- BASE active: base owns P0 control slots; handheld can preempt with take-control.
- No active source: tractor emits low-rate status and waits; any valid source can acquire.

### 4.4 E-stop handling under half-duplex

E-stop must not wait behind P3. There are two parts:

1. Queue priority: E-stop is P0 and goes first when the radio is idle.
2. Airtime bound: no low-priority frame may be long enough to create unacceptable head-of-line blocking once it has started transmitting.

For Profile B, keep P3 fragments small enough to fit inside the uplink grant. A practical first cap is 32-48 bytes of LoRa PHY payload for image fragments, then tune after measurement. Larger 80-byte fragments are acceptable at BW500, but at BW250 they can consume roughly 70 ms before accounting for guard time, which is too much for a 100 ms cycle.

Also fix base command sequencing. The current base E-stop path constructs a command payload with header sequence 0. After normal base control traffic, that can collide with replay protection semantics. Base P0 commands should use the same monotonic source sequence authority as base ControlFrames.

### 4.5 Adaptive SF/BW management

Adaptive SF is useful, but only if it is scheduled. The current idea of sending link-tune frames at both old and new PHYs back-to-back doubles airtime when the link is already degraded.

Safer v25 method:

1. Tractor decides link step-up/down based on received P0 SNR/loss.
2. Tractor announces `target_profile` and `switch_at_sequence` in several consecutive P1/P0-safe frames at the current PHY.
3. Both ends switch at the named sequence/window boundary.
4. If the new profile fails, both sides revert after a deterministic timeout.
5. Step-down to faster profiles requires a long clean interval, for example 30 s.

Do not maintain separate control and telemetry PHYs in v25 unless retune windows are explicitly scheduled.

---

## 5. QoS and airtime governance

### 5.1 Priority classes

Use static priority derived from `frame_type`, command opcode, and telemetry `topic_id`. A header priority field can wait.

| Class | Content | Rule |
|---|---|---|
| P0 | ControlFrame, E-stop, clear E-stop, link tune, take-control | no retries on stale control; never queued behind lower classes |
| P1 | source state, mode, errors, compact GPS, compact hydraulics, link state | bounded cadence; drop oldest if stale |
| P2 | engine, battery, health, logs, crop-health summaries | opportunistic; rate-limit aggressively |
| P3 | image chunks, large logs, plan fragments, bulk transfers | grant-window only; first to drop |

### 5.2 Drop policy

For control, the next frame is the retry. Do not retry stale stick positions. If a P0 control frame misses its window, drop it and send the next one.

For telemetry, drop stale data rather than queueing a backlog. A 5-second-old pressure value is worse than no value if it looks fresh.

For image chunks, maintain one current image object. If a newer image arrives before the old one has finished sending, discard the old partial object unless the old object is almost complete and the queue is otherwise empty.

### 5.3 Airtime accounting

Each node should track actual transmitted airtime in rolling windows, not just bytes. The scheduler needs at least:

- `airtime_p0_ms_last_1s`
- `airtime_p1_ms_last_1s`
- `airtime_p2_ms_last_1s`
- `airtime_p3_ms_last_1s`
- `p0_missed_windows`
- `last_valid_control_ms`
- `last_valid_command_ms`
- `last_image_chunk_ms`

Initial caps for Profile B:

| Class | Suggested cap | Notes |
|---|---:|---|
| P0 | reserved by schedule | control slot always protected |
| P1 | 150-250 ms/s | core telemetry |
| P2 | 50-100 ms/s | diagnostics only |
| P3 | 0-200 ms/s | only after P0/P1 are healthy |

The exact numbers should be measured after firmware can report real on-air time.

---

## 6. Data payload optimization

### 6.1 Highest-value optimization: remove redundant packets

The current 20 Hz heartbeat is the top payload optimization target. The ControlFrame already contains:

- source ID
- sequence number
- flags
- heartbeat counter
- fresh joystick/buttons

While active control is flowing, that is sufficient for liveness. Keep a separate HeartbeatFrame only for idle, parked, source acquisition, and take-control assertion when no control frame is flowing.

Expected savings at current envelope:

- SF7/BW125: about 1640 ms/s recovered at 20 Hz.
- SF7/BW250: about 820 ms/s recovered at 20 Hz.
- SF7/BW500: about 420 ms/s recovered at 20 Hz.

This one change is larger than any byte-packing tweak.

### 6.2 Use compact fixed binary for hot paths

Control, mode, E-stop, source state, and compact hydraulics should stay fixed binary structs. CBOR is useful for sparse diagnostics or future extensible telemetry, but it is not better than a fixed struct for a 10 Hz control path.

Recommended first-build payload targets:

| Payload type | Target cleartext size | Notes |
|---|---:|---|
| Control | 12-16 B | current 16 B is acceptable if heartbeat is merged |
| P0 command | 8-16 B | include monotonic source sequence and command token where needed |
| Compact telemetry | 12-32 B | aggregate related values |
| Bulk telemetry | 32-64 B | P2 only |
| Image chunk | 24-48 B RF payload in Profile B | tune after measuring grant windows |

### 6.3 Aggregate telemetry by use, not by sensor source

Avoid sending GPS, IMU, hydraulics, engine, and mode as many tiny encrypted packets at independent cadences. The fixed security overhead dominates small packets.

Instead, define compact aggregate topics:

- `state_fast`: active source, mode, estop state, link profile, loss counters.
- `motion_fast`: position delta, heading, speed, selected IMU summary.
- `hydraulics_fast`: pressure, selected coil state, flow setpoints, fault bits.
- `health_slow`: battery, engine, temperatures, uptime.
- `events`: sparse fault/event records.

The LAN-side bridge can expand those into full MQTT topics for Grafana/UI. The LoRa side should not pay per-topic encryption overhead for every individual value.

### 6.4 Delta encoding

For P1/P2 telemetry:

- Send full snapshots periodically, for example every 5-10 s.
- Send deltas between snapshots.
- Quantize to engineering resolution, not floating-point convenience.

Examples:

| Signal | Suggested wire form |
|---|---|
| GPS lat/lon | int32 baseline plus int16 centimeter/decimeter deltas inside a local tile |
| Heading | uint16 centidegrees or int16 delta |
| Hydraulic pressure | uint16 PSI/kPa quantized, then int8/int16 delta |
| Battery | uint16 millivolts, low cadence |
| IMU | send yaw/pitch/roll or hazard flags, not full quaternion at 5 Hz unless needed |
| Faults | event bitmap and monotonic event counter |

### 6.5 Security envelope options

Current AES-GCM overhead is 28 bytes per frame: 12-byte explicit nonce plus 16-byte tag. That is bigger than the useful control payload.

Options:

| Option | Bytes saved | Risk | Recommendation |
|---|---:|---|---|
| Keep explicit nonce + 128-bit tag | 0 | lowest crypto risk | fine for first bench if PHY/rates are fixed |
| Implicit nonce from source + 32-bit sequence + boot epoch | 12 | requires correct persistence and replay design | strong candidate after first bench |
| 64-bit GCM tag for high-rate control | 8 | smaller forgery margin but still strong for this use | consider after threat-model sign-off |
| Drop plaintext CRC under AEAD | 2 | less debug redundancy | defer until real crypto is stable |
| Use separate short-tag control and full-tag command profiles | variable | complexity | possible v26 hardening |

Nonce uniqueness matters more than nonce secrecy. If implicit nonce is adopted, use a monotonic boot epoch and a 32-bit sequence space. A 16-bit sequence rolls over too quickly for a long-running machine.

### 6.6 Header and framing options

Smaller improvements:

- Pack version, source, and frame type into one byte if a protocol version bump is acceptable.
- Keep KISS only on stream boundaries such as UART, not necessarily over the RF packet itself.
- Add a flags/grant byte only if it pays for itself by enabling half-duplex scheduling.
- Use one base-side sequence authority. The browser, bridge, and RF command wrapper should not each invent independent source sequence behavior.

### 6.7 Payload optimization priority order

1. Merge active heartbeat into ControlFrame.
2. Freeze viable PHY/rate and scheduler.
3. Aggregate telemetry to reduce packet count.
4. Add delta/event encoding.
5. Bound P3 fragment size.
6. Consider implicit nonce and 64-bit tag after crypto review.
7. Remove RF KISS only after base radio architecture is settled.

---

## 7. Image and video over LoRa

### 7.1 Honest framing

LoRa is not the primary video link. It is a fallback situational-awareness link. The right mental model is not "video stream"; it is "prioritized tiny visual evidence plus structured scene data."

Real 30 fps video should use a separate link: local WiFi, point-to-point 5 GHz, airMAX, WebRTC over LAN, or future cellular if scope changes. LoRa remains the control and fallback awareness link.

### 7.2 Modes worth supporting

| Mode | What crosses LoRa | Typical size | Best use |
|---|---|---:|---|
| M0: metadata only | object/event list, bucket pose, free-space mask summary | 20-500 B/event | safety awareness, low bandwidth |
| M1: micro-thumbnail | 64x48 to 128x72 JPEG/WebP/AVIF | 300-1500 B/image | proof-of-view |
| M2: ROI tile | only implement, hitch, obstacle, rear camera tile | 100-800 B/update | operator-selected detail |
| M3: keyframe + deltas | occasional small keyframe plus changed blocks | 1-4 KB keyframe, 50-300 B deltas | static or slow scenes |
| M4: progressive/SSDV image | chunks that improve partial image as received | 1-8 KB/image | robust slideshow |
| M5: neural latent | quantized features decoded at base | 32-200 B/frame | research, possible future |
| M6: synthetic scene | structured data rendered by base | 50-1000 B/frame | very high compression, clear synthetic UI |

Recommended v25: M0 plus M1/M2. Keep M3/M4 as research/Phase C. Treat M5/M6 as promising but not a first hydraulic-test dependency.

### 7.3 Tractor-side shrinking options

#### Option A: Micro-thumbnail baseline

Pipeline:

1. Capture UVC frame on tractor X8.
2. Downsample to 96x64 or 128x72.
3. Convert to YCbCr 4:2:0 or grayscale in marginal link mode.
4. Encode WebP or JPEG at low quality.
5. Chunk into P3 fragments that fit the grant window.
6. Send only if P0/P1 health is good.

Pros: simple, shippable, debuggable.

Cons: low fidelity and low frame rate.

Use this as the first fallback visual pipeline.

#### Option B: ROI-first imagery

Do not always send the whole frame. Send the region that matters:

- rear camera when reversing,
- implement/hitch tile when using loader/boom,
- lower center crop for ground/row alignment,
- detected obstacle crop.

The operator can request ROI mode from the base. The tractor X8 can also auto-select ROI using current stick direction and camera mode.

Pros: often 4-10x better useful detail per byte.

Cons: bad ROI selection can hide context, so UI must show which ROI is being displayed.

#### Option C: Scene-change gating

Before encoding, compute a cheap change detector:

- pHash on a downsampled luma frame,
- SAD on a 32x32 luma frame,
- optical-flow magnitude bucket,
- object/event change from detector output.

Only send a new image if:

- the scene changed,
- the operator requested a fresh snapshot,
- a mode transition occurred,
- a long heartbeat interval elapsed.

This is very high value for parked or slow-moving scenes.

#### Option D: Progressive/SSDV-style image chunks

SSDV-style chunking is proven in low-bandwidth radio. It lets the base display a partial image as chunks arrive and tolerate missing chunks.

Pros: graceful degradation and easy progress indication.

Cons: still a slideshow, not video.

Good fit for "proof of life" and incident context snapshots.

#### Option E: Keyframe plus changed blocks

Send a small keyframe, then send only changed macroblocks or deltas until a refresh keyframe.

Pros: useful when camera and scene are mostly static.

Cons: motion, dust, lighting flicker, and vibration can turn the whole frame into "changed" data. Needs careful fallbacks.

This is useful later, but not the first implementation.

#### Option F: Semantic/structured payloads

Send meaning instead of pixels:

- object class, bounding box, confidence,
- obstacle distance and bearing,
- free-space mask compressed as a low-res grid,
- bucket/implement pose,
- row/furrow centerline,
- crop-health summary,
- "rear path clear" boolean with confidence.

Pros: best safety value per byte.

Cons: depends on CV model quality and must expose confidence honestly.

This should accompany thumbnails rather than replace them at first.

#### Option G: Neural latent compression

The tractor encodes a frame into a tiny latent vector; the base decodes it with a trained model.

Pros: potentially the only way to make LoRa visual updates feel semi-fluid.

Cons: research risk, dataset requirement, hallucination risk, CPU budget unknown on X8, hard to validate for safety.

Defer until the boring pipeline is measured.

### 7.4 Adaptive visual ladder

The image service should consume link state from the LoRa scheduler and choose a rung:

| Link/scheduler state | Visual behavior |
|---|---|
| P0 loss or E-stop state | image off; metadata only if needed |
| Marginal link | grayscale ROI, slow cadence, tiny chunks |
| Good link | 96x64 or 128x72 color thumbnail, event-gated |
| Excellent link and idle channel | larger snapshot or progressive image |
| Parked and stable | bulk snapshot/log transfer allowed, still P3 |

Do not use RSSI alone. Use SNR, packet loss, P0 missed windows, queue depth, and active operating mode.

---

## 8. Base-station manipulation and enhancement

### 8.1 First job: make uncertainty visible

The base UI should always show:

- image age,
- percent received if progressive,
- link profile,
- whether the image is raw, enhanced, or synthesized,
- confidence for object overlays,
- whether P3 is currently suppressed.

This matters because enhanced imagery can look more trustworthy than the data deserves.

### 8.2 Reassembly and recovery

Base-side image receiver should:

- reassemble chunks by image ID,
- discard stale incomplete images,
- request retransmit only for parked/bulk modes, not active tele-op thumbnails,
- display partial progressive images when useful,
- keep a last-known-good image with a stale badge.

### 8.3 Classical enhancement

Cheap and useful:

- deblocking/denoising after JPEG/WebP decode,
- contrast-limited adaptive histogram equalization for low-light/dust,
- sharpening tuned for implement edges,
- color correction from camera profile,
- stabilization using IMU yaw/pitch/roll if included in telemetry.

These are deterministic and easier to trust than generative methods.

### 8.4 Super-resolution

Use lightweight super-resolution to upscale 64x48, 96x64, or 128x72 inputs to an operator-friendly tile.

Candidates:

- OpenCV DNN super-resolution models for first experiments.
- Lightweight SwinIR/ESRGAN variants if CPU budget allows.
- Domain-trained tractor/implement model later.

Do not present super-resolved output as raw camera truth. Mark it as enhanced.

### 8.5 Temporal enhancement

Base can combine several poor frames:

- temporal denoising,
- burst averaging when scene is static,
- optical-flow alignment,
- frame interpolation for smoother UI transitions,
- persistent fade from last image to newest image.

This improves legibility without sending more bytes.

### 8.6 Semantic overlays

Base can run detectors on the upscaled image and overlay labels:

- person/vehicle/animal/unknown obstacle,
- bucket edge or implement outline,
- rear path zone,
- ground/furrow line,
- crop-health state.

If the tractor already sent structured detections, the base should display both the tractor-sent detection and any base-side interpretation. If they disagree, show low confidence rather than hiding the mismatch.

### 8.7 Synthetic situational view

For very low bandwidth, the base can render a synthetic view from structured data:

- tractor pose from GPS/IMU,
- implement state from controls/hydraulics,
- obstacle markers from CV metadata,
- free-space grid,
- planned path or row centerline,
- CAD/simple 3D tractor model.

This can be more useful than bad video for navigation, as long as the UI labels it as synthetic. It should complement, not replace, a raw last-seen thumbnail.

### 8.8 Local raw recording remains mandatory

The tractor X8 should record full-quality local video or periodic snapshots to local storage when cameras are present. LoRa-transmitted images are for awareness, not incident reconstruction. If something goes wrong, the raw local recording is the source of truth.

---

## 9. Recommended implementation phases

### Phase 0: Decide and instrument

1. Freeze first-build PHY: SF7/BW250/CR4/5 at 10 Hz control, unless field constraints force another profile.
2. Add measured airtime logging per frame type and class.
3. Log P0 inter-arrival, P0 misses, queue drops, RSSI/SNR, and active PHY.
4. Align docs and active `radio.begin()` settings.

### Phase 1: Make half-duplex deterministic

1. Merge active heartbeat into ControlFrame.
2. Add P0/P1/P2/P3 queues.
3. Implement grant windows for tractor uplink.
4. Cap P3 fragment size to fit inside the grant.
5. Fix base command sequencing so E-stop uses monotonic source sequence behavior.

### Phase 2: Reduce packet count and payload overhead

1. Aggregate telemetry into compact topics.
2. Add delta/event encoding.
3. Move KISS out of the RF payload if the selected base radio architecture allows it.
4. Consider implicit nonce and 64-bit tag after crypto review.

### Phase 3: Add fallback visual awareness

1. Implement M0 metadata/events.
2. Implement M1 micro-thumbnail with scene-change gating.
3. Implement M2 ROI mode for implement/rear views.
4. Add base reassembly, stale badges, and enhancement stack.
5. Keep P3 disabled by default until P0/P1 latency tests pass.

### Phase 4: Research-grade visual compression

1. Record real tractor camera datasets locally.
2. Test AVIF/WebP/JPEG XL/SSDV baselines.
3. Prototype keyframe plus delta and semantic scene streams.
4. Only then evaluate neural latent video or synthetic scene generation.

---

## 10. Validation gates

Before hydraulic field testing with LoRa control:

| Gate | Pass condition |
|---|---|
| P0 inter-arrival | within selected profile target for 10 minutes |
| P0 loss | less than 1 percent at 500 m LOS gate, measured at tractor |
| E-stop latency | less than 500 ms p99 to dummy coil de-energize |
| P3 starvation test | image flood causes zero P0 deadline misses over 30 minutes |
| Half-duplex schedule | no unscheduled tractor TX during base/handheld P0 slot |
| Link degrade test | profile fallback reduces speed/cadence and never queues stale control |
| Replay/auth test | command replay and tampered frames rejected |
| UI truthfulness | stale/enhanced/synthesized status always visible |

For image/video specifically:

| Gate | Pass condition |
|---|---|
| Image chunk cap | worst-case P3 in-flight airtime below grant/window cap |
| Stale display | UI clearly marks image age and no-fresh-data state |
| Enhancement label | enhanced or synthetic views are visibly marked |
| Local raw storage | tractor retains raw source material for review |

---

## 11. Open decisions

1. Should first-build control use Profile B (SF7/BW250, 10 Hz) as the canonical field-test profile?
2. Is the base radio path direct Linux SPI or H747 modem for v25?
3. Will KISS remain over RF, or only on serial boundaries?
4. What is the accepted P0 command latency target: 250 ms, 500 ms, or separate targets for control vs E-stop?
5. Are implicit nonce and shorter GCM tags acceptable after threat-model review, or should first build keep the larger conservative envelope?
6. What is the v25 visual promise: proof-of-view thumbnails only, or enhanced situational display?
7. Is a separate real video radio allowed later, or must all remote awareness remain LoRa-only?

---

## 12. Bottom line

The LoRa problem is not just bandwidth. It is timing ownership.

With the current written SF7/BW125 plus 20 Hz control plus 20 Hz heartbeat plan, the channel is oversubscribed before telemetry and video are added. With a realistic Profile B, merged heartbeat, grant-window half-duplex, compact telemetry, and strict P3 suppression, the same hardware can become a robust long-range control link.

For imagery, the winning strategy is to shrink ruthlessly on the tractor and enhance honestly at the base. Send thumbnails, ROI tiles, and structured scene data; reconstruct an operator-useful view with labels, staleness, confidence, and enhancement badges. Keep real video on a separate link or local storage.

That combination gives v25 a practical LoRa stack: safe control first, trustworthy telemetry second, and visual awareness only when the radio budget says it can afford it.

---

End of document v1.1.
