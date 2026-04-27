# LifeTrac v25 — In-Depth LoRa Analysis v1.1 (Update + Synthesis)

**Author:** GitHub Copilot (Claude Opus 4.7)
**Document version:** v1.1
**Date:** 2026-04-27
**Supersedes:** [2026-04-27_LoRa_InDepth_Analysis_ClaudeOpus4_7_v1_0.md](2026-04-27_LoRa_InDepth_Analysis_ClaudeOpus4_7_v1_0.md) — keep that file for the airtime physics derivation, profile comparison, and base-image pipeline. This v1.1 incorporates ideas from the parallel reviews and revises a few of my own conclusions.
**Reviewed in this update:**
- [2026-04-27_LoRa_InDepth_Analysis_Copilot_v1.0.md](2026-04-27_LoRa_InDepth_Analysis_Copilot_v1.0.md)
- [2026-04-27_LoRa_InDepth_Analysis_Copilot_v1_1.md](2026-04-27_LoRa_InDepth_Analysis_Copilot_v1_1.md)
- [2026-04-27_LoRa_InDepth_Analysis_Gemini_3.1_Pro_v1.0.md](2026-04-27_LoRa_InDepth_Analysis_Gemini_3.1_Pro_v1.0.md)

---

## 0. What changed vs my v1.0

Three independent reviewers converged on the same airtime physics and the same recommended profile (SF7 / BW 250 / 10 Hz, single PHY, asymmetric grant-window MAC). Confirming that.

What v1.1 *adds* over v1.0:

| # | New item | Origin | Severity |
|---|---|---|---|
| A | **Active firmware is at BW 500 while plan is BW 125** — a 4× PHY mismatch hiding in `handheld.ino` and `tractor_m7.ino` | Copilot v1.1 §2.1 | Critical |
| B | **Drop KISS framing on the RF wire** (keep it only on the H747↔Linux UART) — saves 2–5 B/frame | Gemini §3, Copilot v1.1 §2.4 | Easy win |
| C | **Cap P3 fragment airtime** so an in-flight image chunk cannot stall an E-stop | Copilot v1.1 §4.4 | Safety |
| D | ~~Digital-twin overlay from telemetry only~~ — **deferred to v26+**. v25 lacks the boom/bucket/arm IMUs and the attachment-detection plumbing required to drive a faithful twin; revisit once those sensors land. See §4. | Gemini §4.2 | Deferred |
| E | **AI frame interpolation (RIFE)** on the base side — turn a 0.2 fps thumbnail stream into ~15 fps display | Gemini §4.2 | UX |
| F | **AI colorization** of grayscale-only transmission, seeded by a daily reference photo | Gemini §4.2 | Optional |
| G | **Optical-flow / motion-vector-only mode** as a sub-thumbnail transmission rung | Gemini §4.1 | Optional |
| H | **Hard operator-UX safety rules** for displayed imagery (staleness timer, "enhanced/synthetic" badge, never hide packet loss) | Copilot v1.1 §7.3 | Safety |
| I | **Explicit MAC-airtime governance metric**: zero P3-induced P0 starvation over a 30-min stress test | Copilot v1.1 §9 | Validation |
| J | **Revised v1.0 §2.3 PHY-switch announcement**: was "3 frames at current SF" → align with Copilot v1.1's `switch_at_seq=N` deterministic style | self | clarification |

Everything else from v1.0 stands. The Profile B recommendation, the implicit-nonce / truncated-tag / bit-packing wins, and the Real-ESRGAN base pipeline are unchanged.

---

## 1. Critical addition — the firmware-vs-plan PHY mismatch

Copilot v1.1 caught something I missed: the active draft firmware initialises the radio at **BW 500 kHz**, not the plan's BW 125 kHz:

- `firmware/handheld_mkr/handheld.ino` → `radio.begin(915.0, 500.0, 7, 5, 0x12, 14)`
- `firmware/tractor_h7/tractor_m7.ino` → `radio.begin(915.0, 500.0, 7, 5, 0x12, 20)`

This is *worse* than a documentation drift. It means:

1. Any bench test run today is at SF7/BW 500 (~23 ms airtime) — looks plausible.
2. Anyone "fixes" the docs by changing the firmware to BW 125 — bench tests immediately break (~92 ms airtime, 4× worse), and they'll think their change broke something.
3. The MASTER_PLAN §8.17 timing claims accidentally match the *firmware* (BW 500) not the *plan* (BW 125). The plan was edited to BW 125 for range without recomputing airtime, leaving the numbers from the BW 500 era still in the text.

**Action.** Pick the profile (recommended Profile B = SF7/BW 250) and change *both* the firmware constant and the doc number in the same commit. Add a constexpr `RADIO_BANDWIDTH_HZ` and `RADIO_SF` in `firmware/common/lora_proto/phy_config.h` so there's exactly one place to change them, and reference it from MASTER_PLAN §8.17.

This is now item #1 in the action queue, ahead of even the airtime-doc fix from v1.0.

---

## 2. Easy byte-level win — drop KISS on the RF wire

Gemini and Copilot v1.1 both noted this and they're right. KISS framing exists because UART is a byte stream with no packet boundaries. **The SX1276 is already packet-oriented**: every TX call gives the radio a length-delimited buffer; every RX call returns a length-delimited buffer with PHY-level CRC.

Costs of keeping KISS on RF:
- 2 bytes for FEND start/end.
- ~1% byte-stuffing overhead (FESC sequences for 0xC0 / 0xDB in the payload).
- Mental model error: encourages thinking of LoRa as a stream.

Where KISS *does* still belong: on the **H747 ↔ Linux UART** boundary if the §8.2 "dumb modem" fallback gets activated, because that link *is* a byte stream. So the right answer is:

- `firmware/common/lora_proto/frame_rf.c` — packet-mode, no KISS, no FEND.
- `firmware/common/lora_proto/frame_uart.c` — KISS-wrapped, only used on serial paths.

Net saving: ~3 B per frame, ~6 ms at SF7/BW 125, ~3 ms at SF7/BW 250. Combined with the v1.0 implicit-nonce + truncated-tag wins, on-air ControlFrame drops from ~48 B to **~20 B** — under half of what was in the original plan.

---

## 3. Safety addition — cap P3 fragment airtime

A subtle safety hole my v1.0 underweighted: even with a priority queue, **once a packet is on the air, it cannot be preempted**. If the tractor starts transmitting an 80-byte image chunk at SF7/BW 250 (~58 ms), and the base sends an E-stop one millisecond later, the base's frame collides or is missed entirely until the next RX window.

Mitigation:

1. **Cap P3 (image) fragment payload to ≤32 B at the chosen PHY**, so any single P3 frame's airtime is ≤25 ms at SF7/BW 250.
2. Image blob (e.g. 500 B WebP) is therefore split into ~16 chunks. At one chunk per uplink-grant window of the 100 ms cycle, the operator gets a fresh image every ~1.6 s — actually faster than my v1.0's "every 5 s" estimate.
3. Define the cap in `phy_config.h` as `P3_MAX_FRAGMENT_BYTES`, computed from `MAX_FRAGMENT_AIRTIME_MS = 25` and the active SF/BW.

This makes the worst-case head-of-line blocking on E-stop a deterministic ≤25 ms — small enough to be invisible in the §8.15 field gate.

---

## 4. Strategic option deferred — telemetry-driven digital twin

Gemini's strongest single contribution was a browser-side 3D digital twin of the LifeTrac, animated purely from telemetry (boom angle, bucket angle, IMU heading/tilt, GPS, wheel/track speeds), giving the operator a fluid 30+ fps third-person view at zero LoRa pixel cost.

**Decision for v25: defer.** Reasons:

1. **Sensor coverage isn't there yet.** A faithful twin needs boom-angle, bucket-angle, and (for tracked variants) attachment-pose ground truth. v25 has only the cab BNO086. Driving boom/bucket from valve-actuation time is open-loop and will drift; driving from cylinder-length sensors requires hardware (hall-effect linear sensors or string pots) that isn't in the current BOM. Without those, the twin will visibly disagree with reality whenever a cylinder hits a stop, slips past a relief, or moves under external load — exactly the moments the operator most needs to trust the view.
2. **Attachment detection isn't implemented.** The twin is only useful if it shows the *currently mounted* implement (bucket vs forks vs auger vs none). v25 has no attachment-ID scheme — no RFID/1-Wire ID on the quick-attach, no operator-side selector in the UI, no telemetry field. Showing a generic bucket when there's actually forks on the machine is worse than showing nothing.
3. **Safety inversion.** Promoting the twin to *default* view while its underlying sensors are open-loop would mean the operator is steering by a model that's *predicting* what the tractor is doing, not *observing* it. That's the wrong direction for a 4000-lb hydraulic machine.

**What v25 ships instead:** image transmission as the primary view, per v1.0 §4 and §3 of this document — color WebP thumbnails at ~1.6 s refresh, Real-ESRGAN super-resolved on the base, optionally RIFE-interpolated (§5.1). Real video over the separate WiFi link when in range. The thumbnails are ground-truth pixels; the operator is never asked to trust a synthesised pose.

**Re-open in v26+ when:** (a) cylinder-length sensors or equivalent are on the boom and bucket cylinders and reporting in telemetry, (b) an attachment-ID scheme exists end-to-end (mechanical ID at the quick-attach → telemetry field → UI selector → 3D model swap), (c) the twin can be validated against a thumbnail side-by-side over a 30-minute mixed-task run with <5° steady-state error on each joint. At that point the twin becomes a *complementary* view (PiP or toggle), still not a replacement for thumbnails.

The rest of this document keeps thumbnails as the operator's primary view, in line with v1.0.

---

## 5. Base-side enhancement upgrades (from Gemini)

Add these to the v1.0 §4.3 base pipeline:

### 5.1 RIFE frame interpolation

[RIFE](https://github.com/megvii-research/ECCV2022-RIFE) (Real-Time Intermediate Flow Estimation) is a small (~10 M params) neural-net frame interpolator that runs at >30 fps on a CPU for low resolutions. Given LoRa thumbnails arriving every ~1.6 s (per §3), the base interpolates 23 intermediate frames at 384×256, producing a fluid-looking 15 fps display from ~0.6 fps source.

Caveat: interpolation is *hallucinated motion*. For a tractor where motion is almost entirely caused by operator-commanded valves whose state we already know, this is actually a good prior — interpolation should be *constrained* by known control state (e.g. "boom is rising" → blend toward upward boom motion). Plain RIFE doesn't know this; a control-state-guided interpolator produces noticeably better results. Defer the custom version to v26 (it's a natural companion to the deferred digital twin from §4); ship plain RIFE in v25 if at all.

### 5.2 Grayscale + AI colorization

If we transmit *only the luma channel* (drop chroma), the WebP blob shrinks ~40%. On the base, run a colorization model (e.g. DeOldify or a small Stable-Diffusion ControlNet) seeded with a single color reference photo taken at the start of each day. Output looks correct; tractor bandwidth drops correspondingly.

This is *optional* — only worth doing if §3's 1.6 s refresh isn't fast enough. The cleaner path is just to ship color WebP at the higher rate.

### 5.3 Optical-flow / motion-vector mode

Below ~300 B/frame, instead of pixels, transmit a **dense optical-flow field** computed on the tractor X8: a 16×12 grid of 2D motion vectors at 8-bit precision = 384 B. The base renders this as a vector overlay on the most recent thumbnail. Useful as a *degraded-link mode* when WebP isn't getting through; not useful as a primary view.

### 5.4 Operator-UX safety rules (mandatory if any of §5.1–5.3 ship)

These are non-negotiable:

1. **Staleness clock visible at all times.** "Last frame: 1.2 s ago" / "Last frame: 8.4 s ago" in red.
2. **"Enhanced" / "Synthetic" badge** on any image whose pixels are not 1:1 from a tractor camera. Includes: super-resolved, frame-interpolated, AI-colorized.
3. **Never hide loss.** If the chunk reassembler is missing fragments, show the partial / corrupt image with the gaps visible. Do *not* extrapolate-and-hide.
4. **One-click toggle** to "raw mode" (most-recent received bytes, no enhancement) — for when the operator needs to verify what the camera actually saw.
5. **Audit log** of which operator command was issued under which view (raw vs enhanced). Stored in the §8.10 black-box logger.

The risk we're avoiding: an operator confidently steering through a person because the AI super-resolved them out of the frame. These rules make that impossible (or at least loudly visible when it happens).

---

## 6. Validation metrics — refine the v1.0 §6 list

Adopt Copilot v1.1's metric for the bench-to-field gate:

| # | Metric | Target |
|---|---|---|
| 1 | P99 inter-arrival of ControlFrame at tractor (Profile B at 10 Hz) | ≤ 110 ms |
| 2 | P99 base-click → tractor-coil-de-energise latency for E-stop | ≤ 250 ms |
| 3 | **Zero P3-induced P0 starvation over a 30-min stress test** (P3 chunks at the §3 cap should never delay a P0 frame by more than `P3_MAX_FRAGMENT_AIRTIME_MS`) | new |
| 4 | Stable operation through induced 10% / 30% / 50% packet loss | hard gate |
| 5 | Image pipeline never delays a P0 frame's TX-start by more than 25 ms | new |
| 6 | Browser click → SX1276 TX-start ≤ 100 ms p99 | from MASTER_PLAN review §1.10 |

Items 3 and 5 are measuring the same property — "the operator's safety-critical actions are never blocked by visual content" — from two different perspectives. Both should pass. (The deferred digital-twin render-rate gate is dropped along with the twin; revisit in v26.)

---

## 7. Revised conclusion

The picture I'd ship for v25 first build:

- **PHY:** SF7 / BW 250 / CR 4-5, 10 Hz control, single profile both directions.
- **Firmware:** match doc to code via `phy_config.h`. Drop KISS on RF. Implicit nonce + 64-bit tag + bit-packed payload → ~20 B on-air per ControlFrame.
- **MAC:** asymmetric grant-window (Strategy 2 from v1.0). 100 ms cycle. Tractor TX only inside its grant. P3 fragment cap = 25 ms airtime.
- **Operator default view: WebP thumbnails over LoRa** at ~1.6 s refresh (~16 chunks of 32 B each, P3 fragment cap = 25 ms airtime), Real-ESRGAN super-resolved on the base, optionally RIFE-interpolated. Color, ground-truth pixels.
- **Real video:** separate 2.4 / 5 GHz WiFi link, not LoRa, when in WiFi range. This is the operator's primary view when it's available; LoRa thumbnails are the fallback when it isn't.
- **Digital twin:** *deferred to v26+* pending boom/bucket sensors and attachment-ID plumbing (§4).
- **Safety UX:** staleness timer, enhanced-pixel badge, one-click raw mode, audit log — all mandatory before any live hydraulic test.
- **Validation:** the six gates in §6.

With this picture, the operator gets ground-truth photographs of the worksite (thumbnails) at the best refresh LoRa allows, real video whenever WiFi is available, and never has their controls blocked by visual content (P3 fragment cap + grant window). The plan becomes buildable on the existing hardware, the existing budget, and the existing v25 timeline — and leaves the digital-twin upgrade path clean for v26 once the sensor coverage catches up.

---

**End of v1.1.** v1.0 remains valid for the airtime derivation, the per-frame byte-savings table, and the Real-ESRGAN pipeline detail. v1.1 should be read as a delta on top of v1.0.
