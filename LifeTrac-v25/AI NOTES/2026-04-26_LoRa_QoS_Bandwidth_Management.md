# LoRa QoS / Bandwidth Management for LifeTrac v25 — Research & Design Sketch

**Date:** 2026-04-26
**Status:** Draft for review. No code written from this yet.
**Related:**
- [`DESIGN-CONTROLLER/LORA_PROTOCOL.md`](../DESIGN-CONTROLLER/LORA_PROTOCOL.md) — current frame format, multi-source arbitration
- [`DESIGN-CONTROLLER/RESEARCH-CONTROLLER/EXAMPLE_CODE/`](../DESIGN-CONTROLLER/RESEARCH-CONTROLLER/EXAMPLE_CODE/) — current draft pipeline
- [`DESIGN-CONTROLLER/RESEARCH-CONTROLLER/VIDEO_COMPRESSION/`](../DESIGN-CONTROLLER/RESEARCH-CONTROLLER/VIDEO_COMPRESSION/) — phased video plan (separate doc)
- [`DESIGN-CONTROLLER/WIRELESS_OPTIONS.md`](../DESIGN-CONTROLLER/WIRELESS_OPTIONS.md)

---

## 1. Question, restated

Should we build a QoS / priority system on top of the v25 LoRa link so that

1. **Steering (ControlFrame, E-stop, command)** always wins,
2. **Position / hydraulics / IMU telemetry** comes next, and
3. **Video thumbnails** are dropped or downscaled when the channel is busy?

And specifically:

- Can image output bitrate be **auto-adjusted from RSSI / SNR / link budget**?
- What are the realistic options for managing LoRa bandwidth on this stack?
- Are there existing open-source projects we can borrow from?

**Short answer:** Yes, this is the right idea, and at v25's scale it's
**cheap to do well** because we already control both ends, both modems, and
the entire framing layer. The main risk is over-engineering — pick the
smallest scheme that solves the actual failure modes (stale joystick under
video flood; loss of GPS when the operator is at the edge of range), not
the textbook one.

---

## 2. The bandwidth budget we're working with

From the [`LORA_PROTOCOL.md`](../DESIGN-CONTROLLER/LORA_PROTOCOL.md) settings
table (915 MHz, SX1276 SiP on the Portenta Max Carrier, 22 dBm TX, 0 dBi
antenna gain at the modem, 8 dBi outside):

| Preset | SF | BW | Coding | Air-rate | Use |
|---|---|---|---|---|---|
| **Control link** | SF7 | 500 kHz | 4/5 | **21.9 kbps** | Handheld ↔ tractor ControlFrame @ 20 Hz |
| **Telemetry link** | SF9 | 250 kHz | 4/5 | 3.5 kbps | Tractor → base Telemetry @ 1–5 Hz |
| **Long-range fallback** | SF11 | 250 kHz | 4/5 | 1.07 kbps | Edge-of-range survival |
| **"Short Turbo"** (fastest legal) | SF7 | 500 kHz | 4/5 | 21.88 kbps | Same as control |

(Rates from Semtech LoRa calculator + cross-checked against the Meshtastic
preset table.)

**Reality check:** even at the *fastest* preset we have, the entire link is
~22 kbps before headers, retransmits, framing, and FCC duty-cycle
compliance. After ~30 % control-channel utilization (the 16-byte
ControlFrame at 20 Hz already eats ~5 kbps of payload + ~30 % overhead =
~6.5 kbps), we have **~15 kbps left for everything else.**

This is the central fact. It is not "video quality" we are trading off. It
is **whether anything else gets through at all** while the joystick stream
is running. Phrased that way, the QoS question is a *correctness*
requirement, not a luxury.

For comparison, a single 320×240 JPEG thumbnail at moderate quality is
8–15 KB. **One thumbnail = ~3–6 seconds of full-channel airtime at SF7/BW500.**
You cannot send a thumbnail every second. You can barely send one every 10.

---

## 3. What can actually go wrong without QoS?

The dangerous case isn't "video looks bad." It's:

1. **Operator drives toward an obstacle.** Joystick says reverse.
2. **Tractor's video uplink is mid-thumbnail** (a 12 KB JPEG is being
   fragmented across 5 LoRa packets).
3. **The thumbnail TX blocks the radio's RX window** — half-duplex SX1276
   can't receive while it transmits.
4. **The tractor's ControlFrame timeout (200 ms in `tractor_m7.ino`) trips.**
   The watchdog is doing its job — valves drop to neutral. But the operator
   sees the tractor go limp for 1–2 seconds while the radio finishes its
   thumbnail and re-enters RX. Confusing at best, dangerous at worst if the
   operator was steering *away* from a tip-over.

The same scenario applies to:

- **GPS update gap** — if the base map freezes for 6 s while a thumbnail
  goes up, the operator's mental model of where the tractor is gets stale.
- **Bidirectional E-stop** — base sends E-stop while tractor is mid-thumbnail.
  Worst-case latency = full thumbnail airtime + RX guard = several seconds.

So the *actual* requirements are:

- **Hard:** never block control or E-stop with bulk telemetry. Bound the
  worst-case latency for a `FT_COMMAND/CMD_ESTOP` to <200 ms.
- **Hard:** never let a low-priority frame in flight starve a higher-priority
  one waiting in the queue.
- **Soft:** when conditions are bad, *gracefully* drop video first, then
  IMU detail, then GPS rate — keep control flowing.
- **Soft:** drop bitrate before dropping fix.

These map directly onto the four-class scheme below.

---

## 4. Proposed scheme: 4 priority classes + airtime budget per class

### 4.1 Classes

| Class | Examples | Frame size | Cadence | Drop policy |
|---|---|---|---|---|
| **P0 — Safety / control** | E-stop, CMD_CLEAR_ESTOP, ControlFrame, Heartbeat | 16–24 B | 20 Hz | **Never dropped, never queued behind anything.** Pre-empts in-flight P3 if it just-entered the queue (we cannot pre-empt the radio mid-TX; see §6). |
| **P1 — State telemetry** | GPS @ 1 Hz, IMU @ 5 Hz, hydraulic-pressure block @ 1 Hz, mode switch, error bitmap | 18–32 B | 1–5 Hz | Drop oldest-in-class if class P0 backs up. |
| **P2 — Bulk metrics** | Engine RPM/temp, battery V, telemetry replay, log fragments | 32–64 B | 0.2–1 Hz | Drop entire class if link RSSI < threshold. |
| **P3 — Opportunistic / video** | JPEG thumbnail fragments, opportunistic high-rate IMU | up to ~200 B | best-effort | Send only when 3 consecutive 100 ms windows have had no P0/P1/P2 traffic *and* link RSSI is above the bitrate ladder threshold (§5). |

This is **strict priority with starvation prevention via a per-class
airtime budget.** P0 is unlimited (it's tiny anyway). P1 gets up to 30 % of
duty-cycled airtime. P2 gets up to 15 %. P3 gets the leftover, capped at
20 %, and is the first thing turned off under stress.

### 4.2 The single output queue

We currently have one radio. So one TX queue. The scheduler sketch:

```
on every loop iteration of tractor_m7.ino:
    if radio is mid-TX: return                   # half-duplex, just wait
    pick highest priority queue with anything in it
    if that queue is non-empty AND class budget not exhausted:
        pop oldest frame, hand to radio
    else:
        try the next-lower class
    rotate budgets every 1 s (sliding window)
```

In code terms: replace the current `radio.transmit(kiss, kl)` direct call
in [`emit_topic`](../DESIGN-CONTROLLER/RESEARCH-CONTROLLER/EXAMPLE_CODE/tractor_h7/tractor_m7.ino)
with `tx_queue_push(class, kiss, kl)`, and add a small `tx_pump()` step in
`loop()` between `poll_radio()` and `apply_control()`.

### 4.3 Where the priority bit lives

Two clean options:

- **Add a 2-bit field to `LoraHeader`.** Makes the priority visible to the
  receiver and the bridge, useful for analytics. Costs a protocol-version
  bump.
- **Derive priority from `frame_type` + `topic_id` at the queue boundary.**
  Zero protocol change. Both ends agree on a static table:
  `FT_COMMAND/CMD_ESTOP → P0`, `FT_CONTROL → P0`, `FT_HEARTBEAT → P0`,
  `topic_id ∈ {0x01, 0x05, 0x06, 0x07} → P1`, etc.

**Recommendation: option 2 first** (no protocol bump), promote to option 1
only if we end up wanting per-frame priority (e.g. "this thumbnail is
urgent because the operator just hit *snapshot*").

---

## 5. Adaptive video bitrate from RSSI / SNR

**Yes, this is straightforward.** The SX1276 reports `getRSSI()` and
`getSNR()` after every received packet (already used in
[`tractor_m7.ino`](../DESIGN-CONTROLLER/RESEARCH-CONTROLLER/EXAMPLE_CODE/tractor_h7/tractor_m7.ino)
to stamp `g_src[i].rssi_dbm`). The base station gets the same numbers from
its own radio. Both ends therefore have a continuously updated picture of
link quality.

### 5.1 Bitrate ladder

At a sane refresh rate (say, recompute once per second):

| Link state | RSSI (dBm) at base | SNR (dB) | Action on tractor X8 ffmpeg |
|---|---|---|---|
| **Excellent** | > −90 | > 8 | 320×240 JPEG, q=70, every 5 s (~12 KB) |
| **Good** | −90 to −105 | 4–8 | 240×180 JPEG, q=55, every 10 s (~6 KB) |
| **Marginal** | −105 to −115 | 0–4 | 160×120 JPEG, q=40, every 30 s (~2 KB) |
| **Poor** | < −115 | < 0 | **Stop sending video.** Send 1× tile every 60 s as a heartbeat-of-life. |
| **Lost** | no fix for 5 s | — | LoRa preset auto-falls back to SF11 (`Long Fast`); video off; control stays on. |

(The boundary numbers come from the Meshtastic radio-settings link-budget
table, scaled for v25's 8 dBi base antenna.)

### 5.2 Where the decision lives

Two reasonable architectures:

- **Tractor-side (closed loop on base RSSI):** the bridge publishes link
  quality on `lifetrac/v25/status/link` once per second; the tractor
  receives that as a low-rate telemetry-style frame and drives its ffmpeg
  pipeline. Pros: one decision-maker. Cons: the link-quality frame *itself*
  can be the thing that doesn't make it through when the link is bad.
- **Tractor-side (open loop on its own RX of base):** the tractor measures
  RSSI/SNR of every base→tractor frame it receives (it already does), and
  uses *that* directly. Asymmetric, since 915 MHz LoRa isn't perfectly
  reciprocal under multipath, but generally close enough. Pros: zero extra
  air traffic. Cons: blind to base-side noise floor.

**Recommendation:** go with the second. We already have the data; no extra
frames; degrades gracefully even when the link is dying.

### 5.3 Hysteresis

Don't oscillate. Implement a 30 s minimum dwell at each rung of the ladder
before moving down, and a 60 s dwell before moving back up. Prevents the
"flicker storm" that hits naive ADR implementations.

### 5.4 Scene-change suppression — "fewer keyframes when nothing is happening"

**As written, the plan sends a JPEG every interval regardless of whether
the scene changed.** JPEG has no inter-frame coding; every frame *is* a
keyframe. So a parked tractor pointed at a quiet field would happily burn
~6 KB every 10 s sending the same picture over and over.

That's silly. The fix is a 5-line scene-change gate on the X8 *before* the
thumbnail enters the LoRa queue:

```python
# in tractor_x8/video_service.py (planned, doesn't exist yet)
def should_send(prev_jpeg_phash, current_jpeg_phash, ticks_since_last):
    # always send a "heartbeat" frame on a long timer so the operator
    # knows the camera + uplink are still alive
    if ticks_since_last >= HEARTBEAT_INTERVAL:  # e.g. 60 s parked, 30 s tele-op
        return True
    # otherwise only send if the scene actually changed
    hamming = popcount(prev_jpeg_phash ^ current_jpeg_phash)
    return hamming > PHASH_DELTA_THRESHOLD  # ~6 out of 64 bits is a typical pick
```

Two cheap detectors that work well at thumbnail resolution:

1. **Perceptual hash (pHash, 64 bit)** — DCT of an 8×8 downsample, sign
   bit per coefficient. Robust to lighting changes and JPEG noise.
   ~50 µs to compute on the X8. Hamming distance > ~6 = "scene changed."
2. **SAD (sum of absolute differences) on a 32×32 luma downsample** — even
   cheaper, ~20 µs, but more sensitive to lighting flicker. Threshold
   ~5 % of total pixel range.

Either one runs *before* JPEG encoding. If it says "no change," we don't
even spend the CPU on the encode, let alone the airtime on the upload.

### 5.5 Operating-mode × scene-change matrix

Putting §5.1 (RSSI) × §12 (mode) × §5.4 (scene change) together, the
actual thumbnail emission rate is:

| Mode | Link | Scene static | Scene changing |
|---|---|---|---|
| **Tele-op** | Excellent | 1 / 30 s heartbeat | 1 / 5 s |
| **Tele-op** | Good | 1 / 60 s heartbeat | 1 / 10 s |
| **Tele-op** | Marginal | 1 / 5 min heartbeat | 1 / 30 s |
| **Autonomy** | Excellent | 1 / 30 s heartbeat | 1 / 3 s |
| **Autonomy** | Good | 1 / 60 s heartbeat | 1 / 5 s |
| **Parked** | Excellent | **1 / 5 min heartbeat** | 1 / 30 s (someone walked into frame) |
| **Parked** | Good | **1 / 10 min heartbeat** | 1 / 60 s |
| **Parked** | Marginal | 1 / 30 min heartbeat | 1 / 5 min |

Notice the parked / static / good case: **one frame every 10 minutes.**
That's ~6 KB/hour from video. The channel is essentially silent, leaving
all of it free for parked-mode bulk transfer (waypoint plans, log
uploads — see §13).

### 5.6 Why a heartbeat frame is non-negotiable

The temptation is "if nothing changed, send nothing." Don't. The operator
UI needs to distinguish three states:

1. **Camera healthy, scene static** — show the last frame with a "static
   for N min" badge.
2. **Camera healthy, uplink dead** — show the last frame with a "stale
   N min, link down" warning.
3. **Camera dead** — show a "no signal" placeholder.

Without a heartbeat frame, the UI can't tell #1 from #2. With a heartbeat,
it can: missing heartbeat → degrade the badge. So we always send a frame
on the long timer even if the pHash says nothing changed.

The heartbeat frame can be *tiny* — a 64×48 single-pHash-bucket-encoded
"still alive, scene unchanged" telemetry frame on the existing telemetry
class would do, no full JPEG required. ~24 B instead of ~2 KB. But that's
a Phase C+ optimization; for first light, just send the small JPEG.

---

## 6. Caveats — things QoS can't fix

- **Half-duplex.** We can't pre-empt a packet that has already started
  transmitting on the SX1276. The mitigation is to keep P3 frames *short*
  (≤ 80 B per fragment ≈ 80 ms airtime at SF7/BW500). Maximum head-of-line
  blocking is then ~100 ms — which is below the 200 ms ControlFrame
  timeout.
- **FCC duty cycle / EU duty cycle.** EU 868 MHz at +14 dBm is capped at
  1 % duty in the SRD band. The QoS scheme has to count *transmitted*
  airtime against a rolling 1-hour budget, not just queue depth. (US 915
  MHz has no such cap, but the same accounting is good hygiene.)
- **CSMA / listen-before-talk.** RadioLib's `transmit()` is blocking and
  doesn't LBT by default. For multi-tractor / shared-band deployments
  (later phase) we'll want to add CSMA, which is a separate piece of
  infrastructure.
- **Cellular failover** (Cat-M1 already on the Max Carrier) bypasses all of
  this. If we land cellular, video streams over LTE and LoRa goes back to
  being a control-plane-only link. Treat the QoS scheme as "what happens
  when LTE is down."

---

## 7. Existing open-source projects worth borrowing from

| Project | License | What we'd take | What we'd skip |
|---|---|---|---|
| **[Meshtastic](https://github.com/meshtastic/firmware)** | GPL-3.0, C++ | The priority queue + per-class airtime accounting. Their `MeshPacket.priority` field has eight levels (0..127, with named constants like `MAX/RELIABLE/DEFAULT/BACKGROUND`); the router is in [`src/mesh/`](https://github.com/meshtastic/firmware/tree/master/src/mesh). Their **adaptive transmit-history** (#10120 series) tracks recent airtime for duty-cycle compliance. | The mesh routing — we are point-to-(point|few), not a flood mesh. Their channel-hashing crypto. |
| **[Reticulum](https://reticulum.network/)** | MIT/Reticulum License | The conceptual model: addresses, links, and announce packets vs. data. Their *per-link* identity-and-priority discipline is exactly the right shape for v25's "tractor + base + handheld" set. | The source-anonymity goals (we want the *opposite* — explicit `source_id`). |
| **[LoRaWAN ADR (TTN/Chirpstack)](https://www.chirpstack.io/)** | MIT | The math for the **adaptive data rate** ladder (SF/BW + TX power tied to per-frame SNR margin). | The whole MAC layer / network server architecture. We don't need a NetworkServer for two nodes. |
| **[LoRa-Dev / RadioHead](https://www.airspayce.com/mikem/arduino/RadioHead/)** | GPL/commercial | Their `RHReliableDatagram` retry-with-exponential-backoff is a clean reference. | Older library, not as well-maintained as RadioLib (which we already use). |
| **[NRC's go-lorawan-priority](https://github.com/brocaar/chirpstack-network-server)** | MIT | Per-class queue with per-class drop policy, written in Go but the algorithm transfers cleanly. | Cloud orientation. |
| **[OpenMHP / OpenAMP-style RTOS queues](https://github.com/OpenAMP/open-amp)** | BSD | Generic priority-queue primitives if we end up putting the queue in M4-shared SRAM. | Most of the IPC scaffolding. |
| **Academic — "[Quality of Service in LoRaWAN](https://ieeexplore.ieee.org/document/9119506)" (IEEE 2020) and follow-ups** | n/a | Their discussion of why pure strict-priority starves under high P3 demand, and the case for **deficit-round-robin within priority classes**. | Scope beyond what we need. |

The closest "this already exists in OSS" answer is **Meshtastic's router** —
specifically their `Router.cpp` / `RadioInterface.cpp` priority-queue
discipline. We don't need to fork it; the design pattern is what we'd
copy, ~200 lines of C++ in the M7 sketch.

---

## 8. Is it more trouble than it's worth?

For LifeTrac v25, **no — but only if we keep the scope tight.**

- The 4-class scheme + airtime budget is a few hundred lines of C++ in
  [`tractor_m7.ino`](../DESIGN-CONTROLLER/RESEARCH-CONTROLLER/EXAMPLE_CODE/tractor_h7/tractor_m7.ino)
  and matching Python in
  [`base_station/lora_bridge.py`](../DESIGN-CONTROLLER/RESEARCH-CONTROLLER/EXAMPLE_CODE/base_station/lora_bridge.py).
- The RSSI-driven bitrate ladder is ~50 lines of Python wired into the
  ffmpeg-thumbnail pipeline (which doesn't even exist yet — we're shipping
  `topic_id=0x20` thumbnails as the entire video plan for first light).
- Most of the safety win comes from **rule #1 alone**: "never let bulk
  telemetry block control." Even just splitting the queue into "P0
  control" and "everything else" gets us 80 % of the benefit.

### Where it *would* be over-engineering

- **Token-bucket per source ID.** We have three sources max
  (handheld/base/autonomy). Strict priority + airtime budget is enough.
- **Reordering on the receiver.** UDP-style "drop late frames" is fine for
  control. We don't need TCP-style re-ordering.
- **Per-class encryption keys.** The fleet key already covers everything;
  splitting by class adds key management without a real threat-model
  benefit.
- **Adaptive SF/BW (real ADR).** Tempting, but switching SF mid-flight is
  fragile (both ends have to agree on the change *before* the next frame
  goes out). For v25, hard-coded "control link uses SF7/BW500, telemetry
  link uses SF9/BW250" is correct.

---

## 9. Recommended phased plan

### Phase A — *Just the queue split.* (~1 week of work)
- Add a 4-class TX queue in `tractor_m7.ino` and the symmetric path in the
  bridge.
- Static priority table by `frame_type` + `topic_id`.
- No bitrate adaptation, no airtime accounting yet — strict priority only.
- Goal: prove the ControlFrame never blocks behind a thumbnail.

### Phase B — *Airtime budget + idle backoff.* (~1 week)
- Add per-class rolling 1 s airtime accounting.
- Hook the GPS service's existing `parked_since` idle-rate logic (already
  in [`gps_service.py`](../DESIGN-CONTROLLER/RESEARCH-CONTROLLER/EXAMPLE_CODE/tractor_x8/gps_service.py))
  into a generic "system idle" signal that the queue uses to release P3
  airtime.

### Phase C — *RSSI-driven bitrate ladder.* (~1 week)
- Implement the 5-rung ladder in
  `tractor_x8/video_service.py` (new file, doesn't exist yet).
- Hysteresis as in §5.3.
- Publish current rung to the base on `lifetrac/v25/status/link_quality`
  for the operator UI.

### Phase D — *Cellular failover hand-off.* (deferred)
- When cellular (Cat-M1) is up, the X8 publishes video on its own MQTT
  topic over LTE; the LoRa video class goes silent automatically.

---

## 10. Concrete next deliverables (if we proceed)

1. New section in [`LORA_PROTOCOL.md`](../DESIGN-CONTROLLER/LORA_PROTOCOL.md)
   defining the 4-class priority table and the static topic→class mapping.
2. New `tx_queue.{h,cpp}` in
   [`EXAMPLE_CODE/lora_proto/`](../DESIGN-CONTROLLER/RESEARCH-CONTROLLER/EXAMPLE_CODE/lora_proto/)
   with the priority queue + airtime accounting, header-only friendly so
   the bridge can mirror it in Python.
3. Wire the queue into `tractor_m7.ino` between `emit_topic()` and
   `radio.transmit()`.
4. New `tractor_x8/video_service.py` skeleton (doesn't exist yet) that
   takes the RSSI ladder as input and runs ffmpeg accordingly. Output topic
   stays `0x20` (already in `TOPIC_BY_ID`).

None of those four are required to ship Phase 1 of v25 — Phase 1 is a bench
test with no video — but Phase A is the right time to add the *control*
queue split, since rebuilding the framing layer after a working video
demo is more painful than getting it right up front.

---

## 11. Open questions for review

1. **Do we want a 5th class for autonomy waypoint commands?** Currently the
   sketch lumps them into P0 (control) — is that right, or do we want them
   at P1 with explicit hand-off rules?
2. **Per-frame or per-class TX power?** RadioLib lets us set TX power per
   call. Cranking video to 22 dBm and dropping control to 17 dBm makes no
   sense, but cranking *control* to 22 dBm and dropping *video* to 14 dBm
   does. Worth ~3 dB extra link budget on the safety-critical class.
3. **Do we ever want the base to push priority hints to the tractor?**
   E.g. "I just lost telemetry, please drop video to lowest rung for the
   next 30 s." A `FT_COMMAND/CMD_LINK_HINT` opcode would do it; tracked as
   future work.
4. **Are we OK with priority-by-static-table forever, or do we want the
   2-bit `priority` header field?** I lean toward static-table for v25 and
   header-field for v30+ when we have heterogeneous operators.

---

## 12. Operating-mode bandwidth windfalls

The QoS scheme above is designed for the *worst* case: human tele-op with
a 20 Hz joystick stream eating ~30 % of channel airtime. But the tractor
spends a lot of its life **not** in that state. Two operating modes free
up huge amounts of LoRa bandwidth, and the airtime-budget version of the
queue (Phase B) picks them up *automatically* without any mode-aware
code path. This section names the windfalls and what we should do with
them.

### 12.1 Mode → airtime model

| Mode | Active source | ControlFrame rate | Control airtime | Channel free |
|---|---|---|---|---|
| **Tele-op (active driving)** | handheld | 20 Hz, 16 B | ~30 % | baseline |
| **Tele-op (idle joystick, deadband)** | handheld | 20 Hz, 16 B (still sent) | ~30 % | baseline |
| **Autonomy (waypoint following)** | X8 autonomy | 1–2 Hz, 16 B | ~2 % | **+28 %** |
| **Parked (E-stopped or `parked_since` set)** | none / heartbeat-only | 0.2 Hz heartbeat only | ~0.5 % | **+29 %** |
| **Cold standby (engine off, key on)** | none | 0.1 Hz heartbeat | ~0.3 % | **+30 %** |

The "parked" row is what the GPS service already detects via
[`gps_service.py`'s `parked_since` field](../DESIGN-CONTROLLER/RESEARCH-CONTROLLER/EXAMPLE_CODE/tractor_x8/gps_service.py)
(no GPS movement for ≥ 30 s drops the GPS rate from 1 Hz to 0.2 Hz). We
can reuse that exact signal as the system-wide "idle" indicator.

### 12.2 What to do with the windfall — *automatically*

Because Phase B's airtime accountant works as a strict priority cascade,
the slack falls through to the next class without anyone telling it to:

- P0 (control) consumes < 5 % → P1 (GPS/IMU/state) gets up to its 30 % cap
- P1 hits its cadence ceiling → P2 (bulk metrics) fills its 15 %
- P2 satisfied → P3 (video) gets the rest

**No mode-switching code required.** The lower classes just see more
green light. This is the cleanest possible behavior and it's why the
Phase B airtime accountant is worth the extra week over Phase A.

### 12.3 What to do with the windfall — *deliberately*

A few things benefit from being *explicitly* mode-aware rather than
relying on cascade:

| Service | Tele-op cadence | Autonomy cadence | Parked cadence |
|---|---|---|---|
| **GPS** ([`gps_service.py`](../DESIGN-CONTROLLER/RESEARCH-CONTROLLER/EXAMPLE_CODE/tractor_x8/gps_service.py)) | 1 Hz | 5 Hz (operator wants to watch the path) | 0.2 Hz (already implemented) |
| **IMU** ([`imu_service.py`](../DESIGN-CONTROLLER/RESEARCH-CONTROLLER/EXAMPLE_CODE/tractor_x8/imu_service.py)) | 5 Hz | 20 Hz | 1 Hz |
| **Video thumbnail** | every 10 s @ q=55 | every 3 s @ q=70 | every 30 s (just a "still alive" frame) or **on demand** |
| **Hydraulic pressure** | 1 Hz | 2 Hz | 0.2 Hz |
| **Engine telemetry** | 0.2 Hz | 0.5 Hz | 0.05 Hz |

Implementation: the X8 autonomy controller publishes
`lifetrac/v25/status/op_mode ∈ {teleop, autonomy, parked, standby}` on its
ROS-style local bus. Each service subscribes and picks its cadence from a
1-line table. Total cost: ~5 lines per service, no LoRa-side change.

### 12.4 Parked mode is special — it's a *bulk-transfer window*

When the tractor is parked, the LoRa channel is essentially idle except
for heartbeats. This is the right time to do anything that involves
moving more than a few hundred bytes:

- **Waypoint plan upload** (see §13)
- **Mission map fragments** (occupancy-grid tiles, coverage masks)
- **Configuration sync** (PID tuning blobs, calibration tables)
- **Log fragments** (sensor logs collected during the prior run, trickled
  back to base for analysis)
- **Firmware delta downloads** (already discussed in [TODO.md](../DESIGN-CONTROLLER/TODO.md)
  as a future item — parked mode is when it should run)

All of these are P3-class — they should *never* run during tele-op or
autonomy. The queue makes that automatic: the bulk-transfer service emits
P3, and the airtime accountant only releases P3 budget when the higher
classes are quiet. Parked mode just happens to be the steady state where
that's almost always true.

---

## 13. Waypoint plans over LoRa (parked-mode bulk transfer)

**Yes, this is exactly the right use of the parked-mode bandwidth.**
Sending a multi-kilobyte waypoint plan during active driving would be a
disaster (fragments compete with control). Sending it while parked is
nearly free — the channel is otherwise empty.

### 13.1 What does a waypoint plan actually weigh?

A reasonable v25 plan format:

```
WaypointPlan {
    uint8  plan_id         // 1 B
    uint8  version         // 1 B
    uint16 n_waypoints     // 2 B
    Waypoint[] points      // 24 B each
    uint16 crc16           // 2 B
}

Waypoint {
    int32  lat_e7          // 4 B  (1.1 cm resolution)
    int32  lon_e7          // 4 B
    int16  alt_dm          // 2 B  (0.1 m)
    uint16 speed_cms       // 2 B  (cm/s, 0..655 m/s, plenty)
    uint16 heading_cdeg    // 2 B  (centi-degrees, 0..359.99)
    uint8  action_id       // 1 B  (stop / pickup / drop / dump bucket)
    uint8  action_param    // 1 B
    uint16 dwell_ms        // 2 B
    uint16 reserved        // 2 B  (alignment + future use)
    uint16 wp_crc          // 2 B  (per-waypoint CRC for fragment-resync)
}
```

24 B per waypoint × N waypoints + 6 B header.

| Plan size | Raw bytes | LoRa packets (≤80 B P3 fragments) | Air time @ SF7/BW500 |
|---|---|---|---|
| 10 waypoints (a small field test) | 246 B | 4 | ~0.4 s |
| 50 waypoints (typical 2-acre scenario) | 1206 B | 16 | ~1.6 s |
| 200 waypoints (1 ha coverage path) | 4806 B | 61 | ~6 s |
| 1000 waypoints (full farm map) | 24006 B | 301 | **~30 s** |

Even the 1000-waypoint case is a 30 s parked-mode transfer. That's
trivial. Compare to LTE / Wi-Fi which would do it in 100 ms — but we
aren't paying for LTE coverage on every parked tractor, so LoRa is the
right channel.

### 13.2 Protocol sketch — `topic_id = 0x30`, "plan transfer"

Add one new topic to [`lora_bridge.py`'s `TOPIC_BY_ID`](../DESIGN-CONTROLLER/RESEARCH-CONTROLLER/EXAMPLE_CODE/base_station/lora_bridge.py):

```
0x30  bulk/plan       (P3 priority, parked-mode-only)
```

Direction: **base → tractor** (most plans flow from operator to robot;
reverse direction reuses the same topic for "tractor reports current
plan back to base for verification").

Frame structure inside the existing TelemetryFrame envelope:

```
PlanFragment {
    uint8  plan_id        // matches the WaypointPlan.plan_id
    uint8  version        // matches the WaypointPlan.version
    uint16 fragment_idx   // 0..n-1
    uint16 fragment_count // n
    uint8  flags          // bit0 = last; bit1 = retransmit
    uint8  payload_len    // up to ~70 B
    uint8[64] payload     // a slice of the serialized WaypointPlan
}
```

72 B per fragment. Fits in the P3 ≤80 B size budget. Fragment-level CRC
piggy-backs on the existing AES-GCM tag.

### 13.3 Reliability — *minimal* ARQ on top of P3

Per-fragment retransmit is essential for plans (unlike video, where a
dropped thumbnail just means the next thumbnail). The simplest scheme that
works:

1. Base sends all N fragments back-to-back at P3.
2. Tractor responds with a single **bitmap-NACK** TelemetryFrame:
   `{plan_id, version, missing_bitmap[]}` — one bit per fragment, so a
   1000-waypoint / 301-fragment plan needs a 38 B bitmap. Fits in one
   frame.
3. Base retransmits only the missing fragments.
4. Repeat until bitmap is all zeros, then tractor emits an `ACK` frame
   with `{plan_id, version, plan_crc16}` to confirm.
5. Tractor commits the plan to flash and reports `current_plan_id` in its
   regular status telemetry from then on.

This is **selective-ACK** rather than go-back-N. About 80 lines of code
per side. The Meshtastic mesh has a similar mechanism for chunked text
messages — worth glancing at for reference.

### 13.4 Safety interlocks

Because waypoint plans drive autonomous motion, the protocol needs
explicit guards:

- **Plan can only be uploaded while parked.** Tractor refuses
  `topic 0x30` ingress if `op_mode != parked` and replies with `error_id
  = PLAN_REFUSED_NOT_PARKED`. No exceptions.
- **Plan must be CRC-verified end-to-end before it becomes the active
  plan.** Stage in flash as `pending`, then a separate `CMD_PLAN_COMMIT
  plan_id, plan_crc16` opcode (P0) flips it to `active`.
- **Operator confirmation required at the base UI.** After tractor
  reports `pending` plan ready, the operator must hit "engage" before
  `CMD_PLAN_COMMIT` is sent. Matches the existing E-stop discipline:
  motion only happens with explicit human consent.
- **Plan version monotonicity.** Tractor rejects any plan whose
  `(plan_id, version)` is not strictly newer than the active one. Stops
  replay attacks and out-of-order delivery.
- **Bounded plan size.** Hard-cap at e.g. 4096 waypoints (~98 KB, ~2 min
  airtime). Prevents an accidental upload that ties up the channel for
  hours.

### 13.5 What about *receiving* plans on the tractor?

The tractor sometimes wants to *report* its current intent back — useful
when a different operator takes over a parked machine. Same protocol,
direction reversed, same topic. Bitmap-NACK on the base side. Cheap.

### 13.6 What this is NOT

- **It is not a streaming map.** Don't try to push live cost-map updates
  over LoRa. That's an LTE / Wi-Fi job.
- **It is not a runtime command channel.** "Add waypoint 47, change
  heading on waypoint 23" is *not* this protocol. The protocol is
  whole-plan replace, atomic. Editing a plan = upload a new version.
- **It is not for high-frequency replanning.** If the autonomy needs
  obstacle-avoidance replan-on-the-fly, that has to happen on the X8
  itself, not by round-tripping to the base.

### 13.7 Phasing

Slot this between Phase B and Phase C of the QoS plan:

- **Phase B+** (after airtime accountant lands): add `topic 0x30` and
  the plan-fragment / bitmap-NACK protocol. Operator can upload static
  paths to a parked tractor.
- **Phase C+**: extend with `topic 0x31 = bulk/log` and `topic 0x32 =
  bulk/firmware_delta` reusing the same selective-ACK fragment protocol.
- **Phase D**: when LTE is up, plan transfer prefers LTE; LoRa stays as
  the fallback path for sites without cell coverage.

### 13.8 Why this is a good fit for v25

LifeTrac's expected use is "drive to field, park, switch to autonomy
mode, hop off, walk back to the base station, watch it work." The
human-walk-back step is **exactly** the parked-mode window. By the time
the operator reaches the base station, the plan-upload UI is already
populated with the tractor's current state. Hit "load plan", wait
~1–30 s for the transfer (visible progress bar from fragment ACKs), hit
"engage" — robot starts moving. This is the natural workflow, and the
LoRa link is the right channel for it.

---

## 14. Bottom line

A 4-class strict-priority queue + a 5-rung RSSI-driven video ladder is the
right amount of QoS for this radio. It's small enough to land in a single
review pass on `tractor_m7.ino` + `lora_bridge.py`, it removes the
worst-case "thumbnail blocks E-stop" scenario, and it gives the operator a
visible "link quality" indicator that's grounded in physics rather than
guesswork. The Meshtastic router is the closest existing reference; we
don't need to fork it, just borrow the queue discipline.

The two operating-mode windfalls — autonomy (~28 % free) and parked
(~29 % free) — are picked up automatically by the airtime cascade, with
the bonus that **parked mode becomes a natural bulk-transfer window** for
waypoint plans, log uploads, and firmware deltas. The waypoint-plan
protocol on `topic 0x30` with selective-ACK is ~160 lines total, gated by
parked-mode + operator consent + monotonic plan versions. It turns the
"walk back to the base station" step of a typical autonomy workflow into
the natural place where the next plan gets loaded.
