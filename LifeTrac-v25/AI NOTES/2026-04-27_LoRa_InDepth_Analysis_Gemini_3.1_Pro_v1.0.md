# LifeTrac v25 LoRa In-Depth Analysis & Optimization Strategy

**Reviewer:** Gemini 3.1 Pro (Preview)
**Date:** 2026-04-27
**Document Version:** v1.0

This document provides an in-depth analysis of the LoRa physical and MAC layer constraints proposed in the v25 Master Plan. It specifically addresses MAC timings, half-duplex channel management, payload packing, and extreme measures for image/video transmission over a highly constrained low-bandwidth link.

---

## 1. The Timing & Half-Duplex Situation

### 1.1 Airtime vs. Cycle Time Constraints
The `MASTER_PLAN.md` specifies a 20 Hz control loop (50 ms cycle time) using LoRa at Spreading Factor 7 (SF7), 125 kHz Bandwidth (BW), and 4/5 Coding Rate (CR).
* **Control Frame:** 16-byte payload + AES-GCM tag (16 bytes) + KISS framing. Approximately 34-40 bytes on the wire.
* **Airtime Calculation:** At SF7/125kHz, a ~36-byte payload takes roughly **30 to 35 ms** to transmit.
* **The 50 ms Window:** If the Base Station transmits for ~32 ms, the channel is idle for only **18 ms** before the next Base Station transmission must begin to maintain 20 Hz.

### 1.2 Half-Duplex Turnaround Latency
The SX1276 is a half-duplex radio. It cannot receive while transmitting. 
* Switching from TX mode to RX mode (and vice versa) involves PLL settling times and SPI register writes, adding 1-2 ms of delay.
* The Tractor must receive the packet, validate it (AES-GCM decryption takes MCU cycles), process it, format a `TelemetryFrame` or video chunk, switch to TX, transmit, and finish transmitting **before** the Base Station begins its next 20 Hz ping.
* **Conclusion:** It is physically impossible to fit a 30 ms Base TX and a 30 ms Tractor TX into the same 50 ms window. At 20 Hz, the link is strictly unilateral.

---

## 2. Half-Duplex Management Solutions

To allow bidirectional data (Telemetry and Images from the Tractor) without colliding with the Base Station's control stream, the MAC layer requires strict orchestration. CSMA (Carrier Sense Multiple Access) is insufficient here due to the high duty cycle; a deterministic approach is required.

### 2.1 Strict TDMA (Time Division Multiple Access)
Instead of unsynchronized 20 Hz firing:
* **Asymmetric Frames:** Reduce the Base Station control frame transmission to 10 Hz (100 ms). 
* **Slot A (0 - 40 ms):** Base Station transmits `ControlFrame`.
* **Slot B (40 - 50 ms):** Turnaround buffer.
* **Slot C (50 - 90 ms):** Tractor transmits `TelemetryFrame` or `VideoChunk`.
* **Slot D (90 - 100 ms):** Turnaround buffer.
This ensures zero collisions and provides a guaranteed 40 ms baseline for the Tractor to push data back to the base.

### 2.2 Dynamic Polling / Master-Slave MAC
The Base Station acts as the strict Master. The Tractor *never* transmits unless explicitly polled by the Base Station, or it attaches its telemetry asynchronously only to the ACK of a Base station packet. 

---

## 3. Data Payload Optimization

To maximize the limited airtime, every bit counts.
* **Drop KISS Framing on Air:** KISS framing (adding `0xC0` bounds and escaping) is designed for raw serial UART streams where byte boundaries might be lost. LoRa provides explicit packet length and CRC in the physical layer header. Stripping KISS framing before handing the buffer to the SX1276 saves 3-5 bytes per packet.
* **Delta Encoding:** For continuous telemetry (e.g., GPS coordinates, hydraulic pressure), do not send absolute 32-bit floats. Send an absolute baseline once every 5 seconds, and send 8-bit or 16-bit deltas (differences from the baseline) in the fast telemetry frames.
* **Bit-Packing:** Compress joystick axes and booleans. If a joystick has 10-bit ADC resolution (0-1023), pack it into exactly 10 bits, not a 16-bit integer. 8 directional valves can be packed into a single 8-bit byte.

---

## 4. Extreme Image/Video Management over LoRa

Transmitting video over SF7/125kHz (~5.4 kbps Max) is practically impossible under standard paradigms. However, by manipulating the data drastically at the edge and reconstructing it at the base, an illusion of a video feed (or highly functional snapshot feed) can be achieved.

### 4.1 Transmission-Side Shrinking (Tractor Edge AI)
* **Grayscale & Extreme Downscaling:** Downscale the USB camera feed to 120x90 pixels. Strip all chroma (color) channels, transmitting only the Luma (grayscale) channel.
* **Region of Interest (ROI) Cropping:** If the goal is to monitor the hydraulic manifold or the bucket, crop the image heavily to a 64x64 pixel box containing only the critical moving mechanism.
* **Motion Vectors / Optical Flow Only:** Instead of transmitting the image, run a lightweight optical flow algorithm on the Tractor's Yocto Linux sidecar. Send only the bounding boxes of moving objects or a vector map of bucket velocity.
* **High-Compression Formats:** Avoid JPEG. Use WebP at minimal quality settings, or modern deep-learning-based compression schemes designed for kbps limits. It is possible to compress a 120x90 grayscale image to ~600-800 bytes. At 1 chunk/sec (using TDMA slots), this is 1 Frame Per Second (1 FPS).

### 4.2 Base Station Image Enhancing & Manipulation
Once the tiny, highly degraded data chunks arrive at the operator's Base Station X8 Yocto processor, powerful local compute can hallucinate/reconstruct the missing data.
* **Super-Resolution Upscaling (AI):** Use models like Real-ESRGAN or standard ESRGAN. The base station receives a blurry 120x90 image and locally upscales it to a crisp 480x360 image. The model learns what hydraulic hoses and tractor parts look like and fills in the degraded details.
* **AI Frame Interpolation (RIFE):** If the LoRa link can only manage 1 Frame Per Second, the video will look jerky. Run a real-time frame interpolation model (like RIFE) on the Base Station. It will artificially generate 14 intermediate frames between every LoRa packet, turning a 1 FPS slideshow into a smooth 15 FPS stream. It hallucinates the movement between states.
* **AI Colorization:** Since only grayscale is transmitted to save bandwidth, run a generative colorization model on the Base Station. By seeding it with one high-res color photo taken at the start of the day, the AI will perfectly colorize the incoming grayscale feed in real-time.
* **Digital Twin Overlay:** Instead of video, use the incoming telemetry (boom angle, bucket tilt, steering angle) to animate a 3D digital model of the tractor in the browser using WebGL/Three.js. This uses exactly zero video bandwidth but provides complete situational awareness.