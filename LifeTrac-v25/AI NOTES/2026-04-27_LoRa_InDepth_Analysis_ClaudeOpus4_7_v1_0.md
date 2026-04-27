# LifeTrac v25 — In-Depth LoRa Analysis: Timing, Half-Duplex, Payload Optimization, and Image/Video Strategy

**Author:** GitHub Copilot (Claude Opus 4.7)
**Document version:** v1.0
**Date:** 2026-04-27
**Scope:** Physical-layer airtime accounting, half-duplex MAC strategies, byte-by-byte payload optimization, and a layered image/video pipeline that pushes the heavy lifting to the base station.
**Companion docs:** [MASTER_PLAN.md §8.17](../DESIGN-CONTROLLER/MASTER_PLAN.md#817-lora-control-link-phy--sf7--bw-125-khz--cr-4-5-default-no-retries-on-controlframe-adaptive-sf7sf8sf9-fallback-when-snr-margin-degrades), [LORA_PROTOCOL.md](../DESIGN-CONTROLLER/LORA_PROTOCOL.md), [VIDEO_OPTIONS.md](../DESIGN-CONTROLLER/VIDEO_OPTIONS.md), [2026-04-26_LoRa_QoS_Bandwidth_Management.md](2026-04-26_LoRa_QoS_Bandwidth_Management.md), [2026-04-27_MASTER_PLAN_Review_ClaudeOpus4_7_v1_0.md](2026-04-27_MASTER_PLAN_Review_ClaudeOpus4_7_v1_0.md).

---

## 0. TL;DR

1. The v25 plan's airtime numbers are **off by ~3×**. SF7/BW 125 kHz on a 44-byte on-air ControlFrame is ~85–95 ms, not ~30 ms. Until that is fixed, every cadence in the plan is fictional.
2. SX1276 is **single-channel half-duplex**. Two PHY profiles (control SF7/BW 125, telemetry SF9/BW 250) cannot be received simultaneously; we either pay the retune cost or unify the PHY.
3. The biggest single win is a **deterministic TDD superframe** with explicit slots — it removes collisions, makes airtime budgeting trivial, and turns adaptive-SF from a hazard into a parameter.
4. The biggest **bytes-per-frame** win is dropping the per-frame AES-GCM 12-byte nonce + 16-byte tag (28 B fixed overhead on a 16-B payload) in favour of an **8-byte truncated tag and a 4-byte implicit nonce** derived from `(source_id || sequence_num || epoch)`. That saves 16 B per frame and ~30 ms of airtime per frame at SF7/BW 125 — without weakening the realistic threat model.
5. Video over LoRa is **not video** — it is a low-FPS situational-awareness *thumbnail stream*. Treat it as a gallery, not a stream. Compress brutally on the tractor, **upscale and enhance on the base**.
6. Real video lives on a **separate, unsynchronised 2.4 / 5 GHz link** that is *not* in the LoRa plan. Don't try to shoehorn it.

---

## 1. Timing — Where the airtime budget actually goes

### 1.1 The Semtech LoRa airtime formula (refresher)

For a payload of $PL$ bytes, spreading factor $SF$, bandwidth $BW$, coding rate $CR \in \{1..4\}$ where $4/(4+CR)$ is the rate, $H = 0$ for explicit header, $DE \in \{0,1\}$ for low-data-rate optimise (forced on at SF11/SF12 BW 125), preamble length $n_{pre}$:

Symbol time:
$$T_s = \frac{2^{SF}}{BW}$$

Number of payload symbols:
$$n_{payload} = 8 + \max\!\left(\left\lceil \frac{8 PL - 4 SF + 28 + 16 - 20 H}{4(SF - 2 DE)} \right\rceil \cdot (CR + 4),\ 0\right)$$

Total time on air:
$$T_{air} = (n_{pre} + 4.25) \cdot T_s + n_{payload} \cdot T_s$$

(See Semtech AN1200.13 / SX1276 datasheet.)

### 1.2 Actual airtime for our packets

For an **on-air payload of 44 B** (16-B ControlFrame + 5-B header + 12-B GCM nonce + 16-B GCM tag − 5-B header already counted = 44 B once you also account for KISS/CRC; round to 44), CR 4/5, preamble 8 symbols, explicit header on, CRC on:

| Profile | $T_s$ | $T_{air}$ for 44 B | What plan claims | Reality vs claim |
|---|---:|---:|---:|---:|
| SF7 / BW 125 kHz | 1.024 ms | **~92 ms** | ~30 ms | **3.0×** worse |
| SF8 / BW 125 kHz | 2.048 ms | **~165 ms** | ~55 ms | 3.0× worse |
| SF9 / BW 125 kHz | 4.096 ms | **~300 ms** | ~100 ms | 3.0× worse |
| SF10 / BW 125 kHz | 8.192 ms | **~580 ms** | — | n/a |
| SF7 / BW 250 kHz | 0.512 ms | **~46 ms** | — | n/a |
| SF7 / BW 500 kHz | 0.256 ms | **~23 ms** | (old draft) | matches |
| SF6 / BW 500 kHz (implicit hdr only) | 0.128 ms | **~12 ms** | — | n/a |

For a **64-B TelemetryFrame** at SF9/BW 250 kHz:

| Profile | $T_s$ | $T_{air}$ for 64 B |
|---|---:|---:|
| SF9 / BW 250 kHz | 2.048 ms | **~165 ms** |
| SF8 / BW 250 kHz | 1.024 ms | **~90 ms** |
| SF7 / BW 250 kHz | 0.512 ms | **~50 ms** |

(Round-trip the formula to check; table is for budgeting, not certification.)

### 1.3 Consequence — channel duty at planned cadences

Plan: 20 Hz ControlFrame + 20 Hz Heartbeat + 2 Hz Telemetry on a single SX1276.

| Stream | Cadence | Per-frame airtime | $\sum T_{air}$ per second |
|---|---:|---:|---:|
| ControlFrame @ SF7/BW125 | 20 Hz | 92 ms | **1840 ms/s** |
| HeartbeatFrame @ SF7/BW125 (12 B) | 20 Hz | 70 ms | **1400 ms/s** |
| TelemetryFrame @ SF9/BW250 | 2 Hz | 165 ms | **330 ms/s** |
| **Total demanded** | | | **3570 ms/s** |
| **Available (one half-duplex radio)** | | | **1000 ms/s** |
| **Oversubscription** | | | **3.6×** |

This is before retries, CSMA backoff, retune time, and FCC dwell limits. **The air budget is not tight; it is broken.** Any optimisation below is meaningful only after one of the structural fixes in §2 lands.

### 1.4 Three coherent profiles to choose from

Pick exactly one for v25 first build. All three meet the §8.15 field-test gate; they trade range for responsiveness differently.

| Profile | Control PHY | Cadence | Range (8 dBi mast, +20 dBm) | Single-frame airtime | 1 Hz tel airtime | Channel margin |
|---|---|---:|---:|---:|---:|---:|
| **A — Long range** | SF9 / BW 125 / CR 4/5 | 5 Hz | ~6–10 km LOS | ~300 ms | ~330 ms | tight |
| **B — Balanced (recommended)** | SF7 / BW 250 / CR 4/5 | 10 Hz | ~3–5 km LOS | ~46 ms | ~50 ms | comfortable |
| **C — Fast stick** | SF7 / BW 500 / CR 4/5 | 20 Hz | ~1–2 km LOS | ~23 ms | ~25 ms | comfortable |

**Recommendation: Profile B.** A 10 Hz stick (100 ms tick) is below human reaction time on a tractor, leaves >50% airtime for telemetry/video/commands, and at SF7/BW 250 with the 8 dBi mast comfortably covers a typical farm. Profile C is reserved for "joystick precision implement work close to the base." Profile A is for large-acreage applications and would be a later option, not v25.

---

## 2. Half-duplex management — the SX1276 truth

### 2.1 What the radio actually does

- **Single channel at a time.** No simultaneous TX and RX. No simultaneous reception of two PHYs. Mode change RX→TX is ~150 µs; **PHY retune (changing SF or BW) is ~5–15 ms** of PLL settling on the SX1276.
- **CAD (Channel Activity Detection)** can sniff in ~$T_s$ for a preamble and is much cheaper than full RX. We should be using it; the current CSMA logic uses RSSI sampling, which is less reliable and more power-hungry.
- **Receiver-busy-during-TX:** while TX'ing 92 ms at SF7/BW 125, the tractor cannot hear the base. This means an E-stop sent during a tractor heartbeat TX is delayed by up to one frame.

### 2.2 Three MAC strategies — choose one

#### 2.2.1 Strategy 1 — Pure CSMA-CA with priority queue (current plan)

What we have now. Simple, no time sync needed, but:
- Hidden-node problem if a third radio (handheld) is out of range of the tractor but in range of the base.
- No deterministic latency bound.
- High collision rate when both ends want to talk (control going down, telemetry/video going up).

**Verdict:** Acceptable for *pure* control + heartbeat, fragile once telemetry + video + commands stack up. Keep as a fallback.

#### 2.2.2 Strategy 2 — Asymmetric "uplink-priority" half-duplex (recommended)

Designate one direction as primary at any moment. Default is **base → tractor (control)**. Tractor TX is gated by a 1-bit "TX permission" piggybacked in every received ControlFrame header.

- Base sends ControlFrame every 100 ms (Profile B). Frame includes `tx_grant_ms` (1 byte: how many ms the tractor is allowed to TX before next base TX).
- Tractor uses its grant window for telemetry + thumbnails + commands.
- If the tractor misses 3 consecutive grants (300 ms), it falls back to opportunistic CSMA for one window, then re-syncs on next received frame.

Pros: deterministic, no clock sync needed beyond the most-recently-received frame, trivial to implement on SX1276.
Cons: tractor-initiated alerts (e.g. "engine over-temp telemetry") can be delayed by up to one window. At 100 ms windows, this is acceptable for everything we care about except E-stop — and E-stop is a base-driven concept anyway.

#### 2.2.3 Strategy 3 — Full TDMA superframe (overkill for v25, ideal for v26+)

Carve a 100 ms superframe into fixed slots; both ends know the schedule by GPS-disciplined time. No CSMA, no backoff, no collisions.

```
0 ms                                                                       100 ms
├─────────────────┬─────────────────┬──────────────┬──────────────┬──────────┤
│  CTRL DOWN      │  CTRL ACK +     │  TEL UP      │  THUMB UP    │  GUARD   │
│  base → tractor │  HEARTBEAT      │  tractor →   │  tractor →   │  retune  │
│  46 ms          │  base ← tractor │  base 25 ms  │  base 15 ms  │  4 ms    │
│                 │  10 ms          │              │              │          │
└─────────────────┴─────────────────┴──────────────┴──────────────┴──────────┘
```

Requires GPS PPS on both ends (not currently available — see [MASTER_PLAN review §1.10 / E11](2026-04-27_MASTER_PLAN_Review_ClaudeOpus4_7_v1_0.md)). Defer to v26.

**Recommendation for v25: Strategy 2.** Adds one byte to the ControlFrame header, gives us deterministic-enough timing, and the tractor never needs to know what time it is.

### 2.3 PHY-switching as a hazard to plan out

Today's "adaptive SF7→SF8→SF9" with simultaneous dual-SF announcement (MASTER_PLAN §8.17) **doubles airtime exactly when the link is worst**. Replace with:

1. Tractor-driven SF election only (tractor has the better view of incoming SNR).
2. New SF announced in 3 consecutive frames at the *current* SF, with `switch_at_seq=N`.
3. Both sides switch at sequence N. No dual-SF transmission, ever.
4. Revert criterion: 30 s of clean reception at the lower SF before stepping back down.
5. Telemetry SF stays locked equal to control SF (single-PHY rule, §2.1) — drop the separate telemetry PHY entirely. The telemetry frames just sit in the tractor's TX-grant window from §2.2.2.

Single-PHY operation also lets us drop one column from every airtime table and removes the retune-during-RX problem entirely.

---

## 3. Payload optimization — bytes are airtime

### 3.1 Current ControlFrame budget

Today (LORA_PROTOCOL.md):

| Layer | Bytes |
|---|---:|
| KISS FEND (start) | 1 |
| Common header (`version | source_id | frame_type | sequence_num`) | 5 |
| Encrypted payload (ControlFrame) | 11 |
| AES-GCM nonce | 12 |
| AES-GCM auth tag | 16 |
| CRC-16/CCITT | 2 |
| KISS FEND (end) | 1 |
| Byte-stuffing overhead (avg ~1%) | ~0–1 |
| **On-air payload (LoRa PHY input)** | **~48 B** |

The cipher overhead (28 B) is **larger than the actual control data (11 B)**.

### 3.2 Win #1 — Implicit nonce (saves 12 B → ~25 ms at SF7/BW 125)

GCM requires the (key, nonce) pair to be unique. The 12-byte nonce on the wire is a wasted self-description. Reconstruct it on the receiver from fields the receiver already has:

```
nonce[0..3]   = source_id || frame_type || flags || version  (4 B, on-wire header)
nonce[4..7]   = sequence_num (extended to 32 bits, big-endian)
nonce[8..11]  = epoch_id (boot-counter, 32 bits, advertised at link-up)
```

`epoch_id` is exchanged once at link-up via a Command frame, persisted on the tractor in non-volatile memory (incremented at each boot — see §3.4 for the wear-out fix), and is never on the per-frame wire. Sequence number is already in the header.

**Result:** 12 B saved per frame. No security loss — the nonce is just as unique, and uniqueness is the only property GCM needs.

### 3.3 Win #2 — Truncated GCM tag (saves 8 B → ~17 ms at SF7/BW 125)

NIST SP 800-38D allows GCM tags as short as 32 bits for high-cadence integrity-only use. We control both ends and operate at 10–20 Hz; an attacker has at most a few seconds to forge before being detected by the next legit frame.

| Tag length | Forgery probability per attempt | Bytes saved |
|---:|---:|---:|
| 128 bit (current) | $2^{-128}$ | 0 |
| 64 bit | $2^{-64}$ | 8 |
| 32 bit | $2^{-32}$ ≈ 1 in 4 billion | 12 |

**Recommendation:** 64-bit tag. Saves 8 B, still cryptographically strong against a determined attacker for our operational window. (32-bit is fine for control-only but uncomfortable for command frames that latch state.)

### 3.4 Win #3 — Persisted-counter wear-out fix (no airtime, but enables §3.2)

Current plan persists `sequence_num` to flash on every frame at 20 Hz → 72k writes/hr → flash dies in days (Gemini review §1.3, this review §1.5). Fix:

- RAM holds the live 32-bit counter.
- Flash holds an `epoch_id` that is bumped by `+10000` once per boot, written **once at boot**.
- Live counter on boot = `epoch_id × 10000`, so at most 10000 sequence numbers ever reuse — well below GCM safety.
- Boot counter is in the implicit nonce (§3.2), so peer recovers state from the next received header.

10000 frames per epoch × 100 ms cadence (Profile B) = ~17 minutes between forced reboots before a single epoch worth of sequence space is consumed. In practice, the system runs for hours per epoch and we are nowhere near the GCM birthday bound.

### 3.5 Win #4 — Bit-pack the ControlFrame payload (saves 4–6 B)

Current 11-byte ControlFrame has 4 stick axes (likely 16-bit each = 8 B), several flags, a watchdog counter. We can pack:

| Field | Current | Optimized | Notes |
|---|---:|---:|---|
| 4× joystick axes | 4 × 16 b = 8 B | 4 × 10 b = 5 B | 10-bit ADC is plenty; visible deadband is ~1% |
| Buttons / flags | 8 b | 8 b | unchanged |
| Watchdog counter | 8 b | 4 b | rolling 0..15 is enough for 100 ms cadence |
| Mode | — | 4 b | room for it once we save the watchdog bits |
| **Total payload** | **11 B** | **7 B** | save 4 B |

Combined with §3.2 + §3.3, the ControlFrame on-air drops from ~48 B to ~24 B — **half the airtime**. Profile B then comfortably handles 20 Hz control if we ever want it.

### 3.6 Win #5 — Compress the per-frame header

Today's 5-byte header has `version | source_id | frame_type | sequence_num`. `version` is constant for a deployment. `frame_type` and `source_id` together have ~6 useful values, fit in 4 bits. Pack as:

```
byte 0:  [version:2 | frame_type:3 | source_id:3]   (constant version → 4 epochs reserved)
byte 1:  [flags:8]                                  (NEW — see review §1.6)
byte 2-3: sequence_num (16 bits, low 16 of the 32-bit nonce input)
```

4 bytes total, **with a flags byte added**. Net: same 4 B but more capability.

### 3.7 ControlFrame on-air byte budget after all wins

| Layer | Now | After §3.2–§3.6 |
|---|---:|---:|
| KISS FEND × 2 | 2 | 2 |
| Header | 5 | 4 |
| Payload | 11 | 7 |
| GCM nonce | 12 | **0** (implicit) |
| GCM tag | 16 | **8** (truncated 64) |
| CRC | 2 | 2 |
| **On-air** | **~48 B** | **~23 B** |
| Airtime SF7/BW 125 | ~92 ms | **~50 ms** |
| Airtime SF7/BW 250 | ~46 ms | **~25 ms** |

This single cleanup makes Profile B at 20 Hz feasible (50% airtime instead of 92%).

### 3.8 TelemetryFrame — the same wins apply, plus CBOR

For variable-length telemetry, switch from packed structs to **CBOR** ([RFC 8949](https://www.rfc-editor.org/rfc/rfc8949)) only where fields are sparse; for fixed schemas keep packed structs. Use **deterministic-encoded CBOR** with integer keys and 1-byte type tags — saves 30–40% on a typical hydraulics record vs JSON, almost the same as packed structs but extensible.

Even better for telemetry: send **deltas only**, with a full snapshot every N frames as a keyframe. A pressure reading at 2 Hz that changes by ±20 PSI is 1 byte signed delta vs 4 bytes float.

---

## 4. Image and video — the real strategy

The honest framing: **you cannot stream useful video over LoRa.** SF7/BW 250 gives ~11 kbps raw, ~5 kbps usable after our framing overhead. That's 600 B/s. A useful 320×240 colour JPEG is 8–15 KB. So a single thumbnail takes 13–25 *seconds*.

What we can do is run a **low-rate, high-impact, situational-awareness gallery** by being aggressive on the tractor and clever on the base. The split is:

- **Tractor:** make the bytes as few as physically possible. Quality is a fraction of a thumbnail.
- **Base station X8:** spend CPU/GPU lavishly to reconstruct a viewable image from those few bytes, using ML upscaling and prior knowledge. **The operator never sees the raw transmitted thumbnail.**

### 4.1 Per-frame budget — pick a target first

| Target ergonomic update | Bytes per image | At Profile B's ~200 B/s spare in the TX window |
|---|---:|---:|
| One image per second (live) | ~200 B | feasible at heavy compression |
| One image per 5 seconds (fluid) | ~1000 B | comfortable |
| One image per 30 seconds (situational) | ~6 KB | uses ~10 s of airtime per image; queue depth = 1 |

**Recommendation:** Target **1 image per 5 s, 800–1000 B per image**. Operator gets a "what is the tractor pointed at" view that updates fast enough to verify the implement is engaged correctly without ever pretending to be a video.

### 4.2 Tractor-side compression — three layered techniques

#### 4.2.1 Resolution & colour reduction (do this first, free)

| Setting | Output |
|---|---:|
| Native UVC capture (Kurokesu C2) | 1920×1080 RGB |
| Downsample (Lanczos) on X8 GPU | **96×64** YCbCr |
| Convert to YCbCr 4:2:0 (chroma subsample) | 96×64 luma + 48×32 chroma × 2 |
| Total raw pixels | ~9.2 KB |

#### 4.2.2 JPEG with aggressive quantisation

`mozjpeg` quality 25, progressive, optimised Huffman: the 96×64 YCbCr 4:2:0 image lands at **~600–900 B**. Looks awful at native size, but that's expected — the base station will upscale (§4.3).

#### 4.2.3 Better: WebP or AVIF (saves ~20–30%)

`libwebp` lossy mode, q=20: same image at **~450–700 B**. AVIF (libavif) is even smaller (~350–500 B) but encodes ~10× slower; if the tractor X8 has spare CPU between control duties, worth it. Otherwise WebP.

#### 4.2.4 Best (drastic): dictionary-coded latents

Trained per-deployment: an **autoencoder** (tiny, ~30k params) compresses the 96×64 frame to a 64-dim latent, quantised to 4 bits each = **32 B** plus a 4-B header. Decoder runs on the base. The dictionary/weights are shipped with the firmware so the tractor never transmits them.

| Method | Bytes per frame | Tractor CPU | Transmit time at SF7/BW 250 |
|---|---:|---:|---:|
| Raw 96×64 RGB | 18432 | 0 | ~3 minutes |
| JPEG q25 | ~700 | low | ~7 s |
| WebP q20 | ~500 | low-mid | ~5 s |
| AVIF q20 | ~400 | mid | ~4 s |
| **Autoencoder latent** | **~36** | mid-high (X8 NPU helps) | **~0.4 s** |

The autoencoder option is the only one that genuinely makes "video" feel responsive over LoRa. It is also a research project — defer to v26 unless someone wants to take it on.

### 4.3 Base-station enhancement — where the value is

The base X8 is a quad-A53 with ~2 GB RAM. Burn it.

#### 4.3.1 Super-resolution

Run **Real-ESRGAN-x4-anime** or **SwinIR-lightweight** (both ~2–4 M params) to upscale the 96×64 → 384×256. CPU inference time ~150 ms per frame, well within the 5 s frame budget. The result is *not* photorealistic — it is *legible*. Operator can tell "implement is up" vs "implement is engaged" with confidence.

#### 4.3.2 Temporal stacking

Three consecutive low-quality JPEGs ≈ 1500 B can be stacked on the base into a single high-SNR frame via burst-photography averaging (à la Google HDR+). Removes JPEG quantisation noise. Trade: latency of 3× the per-frame interval.

#### 4.3.3 Detail prior from base camera (if any)

If the operator has a fixed-position base camera pointing at the work site (a USB webcam on the base X8 covering the field), the base can use that **high-resolution local view as a guide image** for **guided super-resolution** of the LoRa thumbnail. The technique is well-studied in computational photography (Deep Guided Filters, IBP). Output looks dramatically better than upscaling alone, because the high-res priors come from a real photo of the same scene.

#### 4.3.4 Object-detection overlay

Run a tiny YOLO (yolo-nano, 4 MB) on the upscaled image, draw bounding boxes ("tractor cab", "hitch", "implement", "ground line") on the operator's UI. The detection runs on the base, so the model never touches LoRa. This compensates for low resolution by *labelling* what's there.

#### 4.3.5 Persistent overlay

The base UI keeps the *previous* thumbnail visible at 50% opacity, with the new one fading in over 200 ms. Visually this turns a 0.2 fps slideshow into something that feels like motion. Cheap UX win.

### 4.4 Pipeline summary

```
Tractor                                  LoRa                Base station X8
─────────────────────────────────────    ──────              ─────────────────────────────────────
UVC capture 1920×1080 RGB
   │
   ▼ Lanczos downscale (GPU)
96×64 YCbCr 4:2:0
   │
   ▼ WebP q20 (or AVIF)
~500 B blob
   │
   ▼ Chunk: 50–100 B per Command-class frame, framed with (chunk_id | total_chunks | data)
   │
   ▼ Queue P3 in TX-grant window  ─────► RX, reassemble blob
                                                            │
                                                            ▼ WebP decode → 96×64 YCbCr
                                                            │
                                                            ▼ Real-ESRGAN ×4 (CPU, ~150 ms)
                                                            │
                                                            ▼ Optional: guided SR with base-cam prior
                                                            │
                                                            ▼ YOLO-nano detect + label overlay
                                                            │
                                                            ▼ Push to web UI via WebSocket
                                                  Operator sees a clean labelled 384×256 image
                                                  every ~5 s, with persistent fade-in.
```

### 4.5 Real video

For genuine 30 fps video, the answer is **not LoRa**. Add a separate, *physically distinct* 2.4 GHz or 5 GHz radio (e.g. a Ubiquiti Bullet airMAX pair) carrying H.264/H.265 from the X8 to the base. This was already the conclusion of [VIDEO_OPTIONS.md](../DESIGN-CONTROLLER/VIDEO_OPTIONS.md). Reaffirm: **the LoRa thumbnail path is for situational awareness when the WiFi link drops, not as a primary view**. Document this distinction in MASTER_PLAN.md so nobody tries to "improve" the LoRa video.

---

## 5. FCC Part 15.247 sanity check

US 902–928 MHz ISM under §15.247 requires:

- **Digital modulation:** ≤30 dBm conducted, ≤6 dBi antenna without further power back-off (we have +20 dBm + 8 dBi = effective +28 dBm EIRP, fine if antenna gain accounting is right).
- **Maximum dwell time:** if treated as a frequency-hopping system, 0.4 s per channel per 20 s. We are *not* hopping. As a digital-modulation system with 6 dB bandwidth ≥ 500 kHz, **there is no dwell limit**. SF7/BW 125 has 6-dB bandwidth ~125 kHz, so technically it does not qualify under digital-modulation rules without channel hopping.

**Action item separate from this analysis:** confirm with an RF compliance reference (or a consult) whether SF7/BW 125 single-channel operation at +20 dBm is compliant for our duty cycle. If not, the BW 250 / BW 500 profiles (which exceed the 500 kHz threshold) are also a compliance fix, not just an airtime fix. Profile B incidentally lands us in compliance.

---

## 6. Recommended changes, prioritised

| # | Change | Effort | Airtime saved per frame | Other benefit |
|---|---|---:|---:|---|
| 1 | Adopt **Profile B** (SF7/BW 250, 10 Hz control, single PHY for telemetry) | trivial — config | n/a (re-baseline) | FCC margin, simpler RX |
| 2 | Fix airtime numbers in MASTER_PLAN §8.17 + LORA_PROTOCOL.md | trivial | 0 | docs/code agree |
| 3 | Adopt Strategy 2 — uplink-priority half-duplex with `tx_grant_ms` | small fw change | n/a | deterministic latency |
| 4 | Implicit GCM nonce (§3.2) | small fw change | -12 B / -25 ms | correctness |
| 5 | Truncated 64-bit GCM tag (§3.3) | trivial | -8 B / -17 ms | still secure |
| 6 | Bit-packed ControlFrame (§3.5) | small | -4 B / -8 ms | room for `mode` |
| 7 | Header re-pack with `flags` byte (§3.6) | small | -1 B + capability | review §1.6 fix |
| 8 | Wear-out-safe persisted counter (§3.4) | small fw change | 0 | flash longevity |
| 9 | Drop separate telemetry PHY; reuse control PHY in tractor's TX-grant window | small | n/a | removes retune hazard |
| 10 | Tractor video pipeline: 96×64 + WebP q20 + chunked over Command frames | medium | n/a | useful images, no control impact |
| 11 | Base video pipeline: Real-ESRGAN + YOLO overlay + persistent fade | medium | n/a | dramatic UX upgrade |
| 12 | Defer autoencoder-latent video to v26 | n/a | n/a | clear scope |
| 13 | Defer GPS-PPS TDMA superframe to v26 | n/a | n/a | clear scope |
| 14 | Reaffirm: real video is on a separate WiFi link, not LoRa | doc | n/a | scope discipline |

Items 1, 2, and 4 are the minimum to make the plan internally consistent. Items 3, 5, 6, 9 turn it from "consistent" to "comfortable." Items 10–11 are the difference between an unusable thumbnail and a useful situational-awareness display.

---

## 7. What does this look like once shipped?

**Profile B + all §6 wins:**

- 10 Hz control, ~25 ms per frame, ~250 ms/s control airtime. Operator perceives sub-100 ms responsiveness on stick movement.
- Heartbeat piggybacked in the ControlFrame `flags` byte, no separate heartbeat traffic.
- Tractor TX-grant window 50 ms per 100 ms cycle. In that window: 2 Hz telemetry (~50 ms), and one chunk of a thumbnail every other window (~50 ms). Image arrives complete every ~5 s.
- Single PHY both directions, no retune cost, no two-PHY scheduling problem.
- E-stop latency: stick position to coil de-energise in <120 ms p99 worst case (one missed window + one fresh window + Modbus tick + PSR drop).
- Range with the 8 dBi mast: comfortably 3 km LOS, sufficient for typical farm operation.
- Operator sees: real-time stick response, 2 Hz telemetry plot, a clean labelled super-resolved camera image refreshed every 5 s.
- FCC: comfortably inside §15.247 digital-modulation rules.

This is the goal. The current plan can get there with the changes above.

---

**End of document v1.0.** Successor versions should bump the filename suffix (`_v1_1`, `_v2_0`) and link back to this one.
