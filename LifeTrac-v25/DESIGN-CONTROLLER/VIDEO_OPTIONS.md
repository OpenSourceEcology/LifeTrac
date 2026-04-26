# Wireless Video Options for LifeTrac

This document analyzes options for streaming video from a LifeTrac tractor-mounted camera to a remote operator. It complements [WIRELESS_OPTIONS.md](WIRELESS_OPTIONS.md), which covers the control and telemetry links.

## Contents

1. [Why Video Is Hard on Long-Range Radio](#why-video-is-hard-on-long-range-radio)
2. [Bitrate vs Quality Reference](#bitrate-vs-quality-reference)
3. [Option-by-Option Comparison](#option-by-option-comparison)
4. [Video Over LoRa — Detailed Feasibility](#video-over-lora--detailed-feasibility)
5. [Recommended Architecture for LifeTrac](#recommended-architecture-for-lifetrac)
6. [Hardware Recommendations](#hardware-recommendations)
7. [References](#references)

---

## Why Video Is Hard on Long-Range Radio

Video has three properties that fight long-range radio:

1. **High continuous bitrate** — even an aggressively-compressed 360p stream is 400+ kbps, which is 30–100× more than LoRa can carry.
2. **Latency sensitivity** — for *control* video (driving/steering), end-to-end glass-to-glass latency must be ≤150 ms. For *monitoring* video, 1–2 s is acceptable.
3. **Loss intolerance** — a single dropped keyframe can freeze the picture for seconds without re-keying. Long-range radio links have higher packet loss than LANs.

The tradeoff space is roughly: *bitrate × range × latency = constant*. You can have any two cheaply; the third costs money.

---

## Bitrate vs Quality Reference

| Stream type | Resolution | Frame rate | Codec | Bitrate |
|---|---|---|---|---|
| 4K Netflix | 3840×2160 | 30 fps | H.265 | 15,000 kbps |
| 1080p YouTube | 1920×1080 | 30 fps | H.264/H.265 | 5,000 kbps |
| 720p Zoom | 1280×720 | 30 fps | H.264 | 1,500 kbps |
| 480p video call | 854×480 | 30 fps | H.264 | 800 kbps |
| 360p video call | 640×360 | 30 fps | H.264 | 400 kbps |
| Ham-radio DATV | 320×240 | 15 fps | H.265 | 50–150 kbps |
| H.265 "postage stamp" | 160×120 | 5 fps | H.265 | 30–60 kbps |
| Lowest watchable H.265 | 96×80 | 5 fps | H.265 | 8–15 kbps |
| 1 JPEG thumbnail every 5 s | 160×120 | 0.2 fps | JPEG | 2–5 kbps |
| 1 JPEG thumbnail every 30 s | 320×240 | 0.03 fps | JPEG | 1–2 kbps |

For comparison, here's what each link can carry:

| Link | Useful sustained throughput | Range |
|---|---|---|
| WiFi 5 (802.11ac local) | 50–200 Mbps | ≤100 m NLoS / 300 m LoS |
| Ubiquiti airMAX 5.8 GHz P2P | 100+ Mbps | 5–15 km LoS |
| Cellular 4G LTE | 2–10 Mbps up | unlimited (coverage) |
| Cellular 5G | 50–500 Mbps up | unlimited (coverage) |
| WiFi long-range mesh | 5–20 Mbps | 500 m–1 km |
| **LoRa (custom stack, SF7/BW500, US)** | **~12 kbps** | 5–15 km LoS |
| LoRa (LoRaWAN, 1% duty cycle) | ~0.1 kbps avg | 5–15 km LoS |
| XBee-PRO 900HP @ 200 kbps | ~50–80 kbps | ~3 km LoS |
| XBee-PRO 900HP @ 10 kbps | ~3–5 kbps | ~14 km LoS |

**Headline:** LoRa is roughly **2 orders of magnitude too slow for live video** and **1 order of magnitude too slow for usable low-frame-rate video calls**. Anything resembling a real-time camera feed needs WiFi or cellular.

---

## Option-by-Option Comparison

| Option | Best resolution / fps | Latency | Range | Cost | Recommended for |
|---|---|---|---|---|---|
| **WiFi (local, 802.11ac)** | 1080p / 30 fps | 50–200 ms | ≤100 m NLoS | $5–$30 (already in v25) | Yard / shop / close-range field work |
| **Ubiquiti airMAX 5.8 GHz P2P** | 1080p / 30 fps | 5–20 ms | 5–15 km LoS | $200–$400 (pair) | Fixed long-range link from base station to a known field |
| **Cellular 4G LTE** | 720p / 30 fps | 100–300 ms | unlimited | $20–$60/mo | Roaming / off-property / multi-site |
| **Cellular 5G** | 1080p / 30–60 fps | 20–80 ms | unlimited | $30–$100/mo | Same as 4G but where 5G coverage exists |
| **WiFi mesh (UniFi / OpenWRT)** | 1080p / 30 fps | 50–200 ms | ≤1 km | $100–$500 | Multi-acre farms with mesh nodes |
| **Analog FPV (5.8 GHz)** | ~640×480 equiv | 30–50 ms | 1–5 km LoS | $30–$80 (TX+RX) | Drone-style line-of-sight piloting |
| **DJI Air Unit / O3 (digital FPV)** | 1080p / 60 fps | 30 ms | 5–10 km LoS | $200–$400 | Highest-grade low-latency FPV; closed ecosystem |
| **HamTV / DATV (70 cm or 23 cm ham bands)** | 320×240 / 15 fps | 200–500 ms | 5–20 km LoS | $100–$300 | Amateur radio operators only; licensed use |
| **LoRa (slideshow only)** | 160×120 every 1–10 s | 1–10 s | 5–15 km LoS | $30–$50 (board) | Fallback "proof of life" image when other links fail |

---

## Video Over LoRa — Detailed Feasibility

LoRa cannot stream video in any conventional sense, but it *can* carry low-rate imagery if you treat it as a slideshow rather than a stream and use a custom stack to bypass LoRaWAN's duty-cycle restrictions.

### Throughput ceiling

| Region | Rule | LoRa max effective rate |
|---|---|---|
| US (FCC §15.247) | No duty-cycle limit on DTS modulation | **~12 kbps** (SF7/BW500, custom stack) |
| EU (ETSI EN 300 220, sub-band g3) | 10% duty cycle, 14 dBm ERP | **~1.2 kbps** |
| EU (sub-band g1) | 1% duty cycle | ~0.12 kbps |
| LoRaWAN (any region) | 1% duty cycle (effectively) | ~0.1 kbps |

So in the US with a custom stack you can sustain ~12 kbps; everywhere else you're 10–100× lower.

### What 12 kbps buys you

| Approach | Bitrate | Frame interval | Resolution | Use case |
|---|---|---|---|---|
| **Single JPEG every 10 s** | 2–5 kbps | 10 s | 160×120 | "Is the tractor still where I left it?" |
| **JPEG every 2 s, ROI-only** | 8–12 kbps | 2 s | 96×72 crop | Slow situational awareness |
| **Difference frames + keyframe every 30 s** | 6–10 kbps | 1–2 s | 160×120 | Static scenes (stopped tractor monitoring) |
| **Codec2 audio + 1 JPEG/min** | 2.4 kbps voice + bursty image | varies | — | Voice + occasional photo, no live video |
| **Thermal frames (32×24)** | 2–5 kbps | 1–2 fps | 32×24 | FLIR Lepton / MLX90640 thermal grid |
| **On-device CV metadata** | 1–5 kbps | event-driven | — | "obstacle at bbox (x,y,w,h)", "person detected" |

### How a custom stack helps

Stock LoRaWAN cannot do *any* of the above. A custom stack on SparkFun hardware (see [LORA_CUSTOM_STACK_TODO.md](LORA_CUSTOM_STACK_TODO.md)) unlocks the radio's real capacity through several stacking savings:

| Saving | How it helps | Bitrate gain |
|---|---|---|
| **Skip LoRaWAN headers** | LoRaWAN MAC ~13 bytes/packet; KISS ~2 bytes | +20–30% payload |
| **Skip duty-cycle limits** | FCC §15.247 has no DTS duty-cycle cap in the US | **10–100× throughput** |
| **Drop ACK/retry** | Image data is loss-tolerant with FEC | +30–50% throughput |
| **Application-layer FEC** | Drop lost packets instead of retransmitting | +20–40% throughput |
| **Differential / progressive JPEG** | Low-res keyframe + refinement layers | 2–3× perceived quality |
| **ROI streaming** | Send only the changed region | 5–10× on static scenes |
| **On-device CV preprocessing** | Send detections, not pixels | **100–1000×** — the real win |
| **Channel hopping across multiple SX1262s** | 2–4 modems in parallel on adjacent channels | 2–4× throughput |

### Why "the real win" is on-device CV

1 kbps of "obstacle detected at 3 m, 30° left of center, confidence 0.92" is more useful for safe remote operation than 10 kbps of pixels would be. An ESP32 + OpenMV camera or a Raspberry Pi Zero 2W with TFLite can do basic CV (object detection, motion detection, row alignment) on the tractor and send the *meaning* of the image over LoRa instead of the image itself.

This collapses the bandwidth problem: you no longer need to push pixels at all for most use cases.

---

## Recommended Architecture for LifeTrac

Use the right link for the right kind of video data:

```
Operator station                        LifeTrac v25 tractor
┌──────────────────┐                   ┌────────────────────────┐
│ Web dashboard    │ ◄── WiFi (local) ─│ Pi camera / USB cam    │  ← 720p–1080p MJPEG / WebRTC
│ (camera tile)    │ ◄── Cellular ─────│ (RTSP / WebRTC)        │  ← 480p–720p when off WiFi
│                  │                   │                        │
│ "Last seen"      │ ◄── LoRa ─────────│ JPEG thumbnail/30 s    │  ← 160×120 fallback image
│ thumbnail        │                   │ + CV metadata (1 kbps) │
│                  │                   │                        │
│ Obstacle alerts  │ ◄── LoRa ─────────│ On-device CV detector  │  ← "obstacle ahead", "row OK"
└──────────────────┘                   └────────────────────────┘
```

**Layered fallback:**

1. **WiFi/MQTT (already in v25)** — primary high-quality video when in range. RTSP, MJPEG-over-HTTP, or WebRTC at 720p/30 fps.
2. **Cellular LTE/5G** — when out of WiFi range. 480p–720p at 30 fps; ~1–2 GB/day at light use.
3. **LoRa thumbnail + CV metadata** — when both above fail. 160×120 JPEG every 30–60 s as a "proof of life" image, plus on-device CV detections. Just enough for the operator to confirm the tractor is safely stopped.

**Reserve LoRa for emergencies and metadata.** Don't try to carry continuous video over LoRa; you'll lose half the v25 feature set fighting the radio. Use the bandwidth for what LoRa is uniquely good at: long-range, low-rate, resilient delivery of *meaning*, not pixels.

---

## Hardware Recommendations

### Tractor-side cameras

| Camera | Resolution | Bus | Link | Approx. price | Notes |
|---|---|---|---|---|---|
| Raspberry Pi Camera Module 3 | 4K still / 1080p60 | CSI-2 | Pi → MQTT/RTSP | ~$25 | Best image quality for the price |
| Arducam IMX519 / IMX708 | 4K still / 1080p60 | CSI-2 | Pi → MQTT/RTSP | ~$30 | Drop-in for Pi Camera; better optics options |
| ESP32-CAM (OV2640) | 1600×1200 / 1080p15 | onboard | WiFi → MQTT | ~$10 | Cheapest option; modest quality |
| OpenMV Cam H7 | 320×240 / 60 fps | UART/USB | On-board CV → LoRa metadata | ~$80 | Runs detection on-board |
| FLIR Lepton 3.5 | 160×120 thermal | SPI | Pi/ESP32 → LoRa or WiFi | ~$200 | Thermal — small enough to send over LoRa |
| MLX90640 | 32×24 thermal | I²C | Any MCU → LoRa | ~$50 | Cheap thermal grid; tiny; LoRa-friendly |

### Operator-side display

- **Raspberry Pi 7" touchscreen + RTSP client (`mpv`, `gstreamer`)** — already part of the v25 stack, low latency
- **Phone / tablet running a WebRTC dashboard** — easiest portable option

### Long-range bridge hardware

- **Ubiquiti NanoStation 5AC Loco** (~$90) — best-value 5.8 GHz P2P link for video at 5–10 km
- **Ubiquiti LiteBeam 5AC Gen2** (~$80) — for >10 km LoS shots
- **Waveshare SIM7600G-H 4G HAT for Pi** (~$50) — drop-in cellular for video when out of WiFi range

---

## Multi-camera deployment

v25 supports multiple cameras on the tractor X8 (single USB root hub, ~480 Mbit/s shared). The **selected camera at any moment** is what gets thumbnailed for LoRa and (when available) live-streamed over LTE. Other feeds stay local on the tractor for log replay.

Camera positions and their LoRa topic IDs (from [LORA_PROTOCOL.md § Telemetry topic IDs](LORA_PROTOCOL.md#telemetryframe-variable-9128-bytes)):

| Position | Topic | When auto-selected | Hardware suggestion |
|---|---|---|---|
| **Front (cab forward)** | `0x20` `lifetrac/v25/video/thumbnail` | tele-op forward, autonomy, default parked | Kurokesu C2-290C boxed + 2.8–12 mm varifocal CS |
| **Rear (reverse)** | `0x21` `lifetrac/v25/video/thumbnail_rear` | reverse-stick deflection >50% for >1 s | Second Kurokesu C2-290C boxed + fixed 3.6 mm CS lens |
| **Implement (boom / hitch)** | `0x23` `lifetrac/v25/video/thumbnail_implement` | manually selected | Kurokesu C1 PRO board + 6 mm M12 lens |
| **Crop-health (down-looking)** | computed → `0x24` summary, no live feed | always-on background analysis | MAPIR Survey3W NDVI/OCN, or NoIR + 850 nm filter — see § Crop-health analysis |

Switching is driven by **`CMD_CAMERA_SELECT`** (see [LORA_PROTOCOL.md § Command frame opcodes](LORA_PROTOCOL.md#command-frame-opcodes)). The default mode is `0x00 auto-by-mode` which gives operators the right camera without thinking; the base-station UI also has a manual override toggle.

The reverse-camera auto-switch deserves to be on the tractor X8 rather than the base, because the round-trip latency of "stick goes back → base sees it → base sends `CMD_CAMERA_SELECT` → tractor switches" is ~150 ms minimum. The X8 watches the local CAN/Modbus drive command and switches in <50 ms with no LoRa hop required.

---

## Crop-health analysis

LifeTrac is fundamentally an agricultural machine. Once we have a down-looking camera on the tractor, we get **NDVI / GNDVI / canopy-cover analysis essentially for free** because the i.MX 8M Mini on the X8 has the headroom for it. This complements (not replaces) drone-based or satellite imagery; tractor-mounted is **continuous, ground-truth, and georeferenced via the same NEO-M9N feeding `topic 0x01`.**

### Why do it onboard the X8 vs. uploading raw frames

The bandwidth case is decisive: a single 1080p NDVI frame is ~2 MB; even over LTE at 1 Mbit/s that's 16 s per frame. Onboard, the X8 reduces it to a **30 B summary per row swept**: mean NDVI, std, percent-canopy-cover, dominant-class flag (healthy / stressed / bare). That fits in one LoRa P2-class TelemetryFrame on `topic 0x24` once per minute. The raw frames stay on the tractor's microSD for later download over WiFi when the tractor is parked next to the shed.

### Camera options

| Option | Cost | Output | OSHW | Notes |
|---|---|---|---|---|
| **MAPIR Survey3W OCN (Orange/Cyan/NIR)** | ~$365 | 12 MP, USB | partial (sensor closed, mount/sw open) | Designed for agriculture. Calibration card included. Single sensor, no alignment. |
| **MAPIR Survey3W NDVI (Red+NIR)** | ~$300 | 12 MP, USB | partial | Cheaper than OCN, narrower spectral coverage |
| **Raspberry Pi NoIR + 850 nm long-pass filter** | ~$45 | 8 MP, CSI | yes (Pi side) + filter is OSHW-neutral | Cheapest path. Single-band NIR-only — derives NDVI by *adding* a second visible-band camera. |
| **Dual Pi NoIR (visible + NIR)** | ~$90 | 2× 8 MP | yes | Two synchronized Pi cams with different filters. Spatial alignment is the work. |
| **Sentera Single (PPK option)** | ~$1,500+ | 12 MP NDVI/NDRE | no | Professional-grade. Out of scope for v25 budget. |
| **Reuse the C2-290C front cam + diurnal calibration** | $0 (already on tractor) | RGB only | yes | No NIR, so no true NDVI — but **GNDVI proxy + Excess Green Index (ExG) + canopy-cover %** are all computable from RGB and useful as *trend* metrics. Worth doing as Phase-1 even before adding NIR. |

For v25 first light: **start with the RGB-only ExG/canopy-cover analysis on the existing front cam** ($0 hardware adder). It's not as good as NDVI, but it's free, ships now, and the X8 code (Python + OpenCV) is the same shape as the eventual NDVI pipeline. Add the MAPIR or dual NoIR in Phase 3.

### Onboard pipeline (runs on the X8 Linux side)

```
[USB UVC cam] ──► gstreamer/v4l2 ──► OpenCV ──► classify rows
                                       │            │
                                       ▼            ▼
                                  per-row stats  geotag from
                                  (mean NDVI,    topic 0x01 GPS
                                   std, canopy)  + IMU heading
                                       │
                                       ▼
                                 30 B summary ──► topic 0x24 ──► base UI map
                                       │
                                       ▼
                                 raw frame to /var/lib/lifetrac/crop/  (microSD)
                                 retrievable via WiFi when parked
```

Compute budget: at 5 fps × 320×240 grayscale NDVI frames, OpenCV `meanStdDev` + thresholding + connected-components is ~10 ms/frame on one A53 core. Less than 5 % of one core. Even a small ML classifier (MobileNet-v3-tiny INT8 → "healthy / stressed / weed" head) runs in ~50 ms/frame on NEON without bothering the GPU we don't have.

### What the operator sees

The base-station map view (see [BASE_STATION.md](BASE_STATION.md)) overlays a per-row NDVI heatmap — green where canopy is healthy, yellow where stressed, red where bare. Updated continuously as the tractor sweeps the field. **No live video required for this** — just the 30 B summaries plus the GPS track. Drag-select a region to download the raw frames over WiFi.

### Open questions

- **Calibration:** NDVI is meaningful only when the sensor is calibrated. MAPIR ships a calibration card; Pi-NoIR DIY needs a reference panel each session. Track in TODO.
- **Light invariance:** dawn / dusk / overcast change the absolute NDVI numbers. Use the calibration card-based normalization, or accept that we're tracking *relative* trends within a session.
- **What about disease detection / weed ID?** MobileNet-v3 on the X8 plus a small custom dataset gets you "is this row a weed" at field-relevant accuracy. Out of scope for v25; track in [TODO.md](TODO.md) Phase 4+.

---

## References

- [WIRELESS_OPTIONS.md](WIRELESS_OPTIONS.md) — main control/telemetry link analysis
- [LORA_CUSTOM_STACK_TODO.md](LORA_CUSTOM_STACK_TODO.md) — implementation plan for the LoRa stack referenced above
- [WebRTC for low-latency streaming](https://webrtc.org/)
- [GStreamer RTSP server](https://gstreamer.freedesktop.org/documentation/rtsp-server/)
- [OpenMV — on-camera computer vision](https://openmv.io)
- [Codec2 — 700 bps to 3.2 kbps voice codec](https://www.rowetel.com/?page_id=452) (for voice + LoRa thumbnail combo)
- [Ubiquiti airMAX product line](https://www.ui.com/airmax/)
- [LoRa AirTime calculator](https://www.semtech.com/design-support/lora-calculator)
