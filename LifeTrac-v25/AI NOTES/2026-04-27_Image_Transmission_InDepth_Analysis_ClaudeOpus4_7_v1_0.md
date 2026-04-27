# LifeTrac v25 — In-Depth Image-Transmission Analysis v1.0

**Author:** GitHub Copilot (Claude Opus 4.7)
**Document version:** v1.0
**Date:** 2026-04-27
**Scope:** Treats the LoRa image pipeline as a first-class subsystem now that the digital twin is deferred ([2026-04-27_LoRa_InDepth_Analysis_ClaudeOpus4_7_v1_1.md](2026-04-27_LoRa_InDepth_Analysis_ClaudeOpus4_7_v1_1.md) §4). Explores what the Portenta X8 on each end can realistically do, which open-source AI models fit, and how to spend our ~800–1000 B/refresh budget more cleverly than "compress harder."
**Companion docs:**
- [2026-04-27_LoRa_InDepth_Analysis_ClaudeOpus4_7_v1_0.md](2026-04-27_LoRa_InDepth_Analysis_ClaudeOpus4_7_v1_0.md) §4 — base-side Real-ESRGAN pipeline
- [2026-04-27_LoRa_InDepth_Analysis_ClaudeOpus4_7_v1_1.md](2026-04-27_LoRa_InDepth_Analysis_ClaudeOpus4_7_v1_1.md) §3, §5 — P3 fragment cap, RIFE, optical-flow degraded mode

---

## 0. Executive summary

Stop thinking "send a compressed image." Start thinking "**send the minimum description of what changed**, and let both sides reconstruct the rest from priors that were already on disk."

Six wins, in priority order:

| # | Idea | Bytes saved per refresh | Adds latency? | Implementation effort | Ship in v25? |
|---|---|---|---|---|---|
| 1 | **Tile-delta transmission** (only re-send the 32×32 tiles whose pHash changed) | 60–95% on a static scene | No (computed at capture) | Medium | Yes |
| 2 | **Region-of-interest budget allocation** (spend 80% of bytes on the tile under the bucket / centre 1/3 of frame) | Re-allocates rather than saves; doubles useful detail | No | Low | Yes |
| 3 | **Persistent base-side scene buffer + temporal super-resolution** (stack the last N partial updates, run SR on the stack) | None on tractor; 4× perceived detail on base | +0–200 ms display | Medium (base only) | Yes |
| 4 | **On-tractor semantic compression** (small CNN classifies "is this a person / animal / vehicle / obstacle?" and biases ROI toward those tiles) | Re-allocates; *zero* missed-obstacle frames | <50 ms | High (model deploy) | Stretch |
| 5 | **On-base scene completion / inpainting** of dropped tiles using the persistent buffer + Stable-Diffusion-Turbo or a small inpainter | None on tractor; eliminates visible holes | +50–200 ms | Medium (base only, GPU helps but not required) | Stretch |
| 6 | **Learned codec** (a small autoencoder trained on farm imagery, transmit only latents) | 50–80% over WebP at the same perceptual quality | +20 ms tractor + 50 ms base | High (training pipeline + deploy) | v26 |

The big idea behind #1–#3 is that **the operator's environment changes slowly** (a tractor moves through a field at <10 mph, the field itself doesn't move) but the *important* parts change rapidly (the bucket, an approaching person, a deer crossing). A delta-tile pipeline matches our budget to that asymmetry directly.

---

## 1. Hardware reality check — what the Portenta X8 can actually run

We have two X8s in the loop. They are *not* equivalent — base has wall power and a USB host where you can hang anything; tractor is on battery and inside a sealed enclosure. Plan around that asymmetry.

### 1.1 Portenta X8 silicon

[Portenta X8 (ABX00049)](https://www.arduino.cc/pro/hardware-product-portenta-x8/) is built around the NXP i.MX 8M Mini:

| Block | Spec | What it's good for |
|---|---|---|
| CPU | 4× Cortex-A53 @ 1.8 GHz | General-purpose ARM Linux, NEON SIMD for image ops |
| GPU | Vivante GC NanoUltra | OpenGL ES 3.1 / Vulkan 1.0; small NN inference via OpenCL is *technically* possible but driver support is poor — treat as graphics-only |
| VPU | Hantro (decode only on Mini variant) | H.264/H.265 *decode*; no hardware encode on the Mini |
| RAM | 2 GB LPDDR4 | Comfortable for ~100–200 MB models |
| Storage | 16 GB eMMC | Plenty for a model zoo |
| Co-MCU | STM32H747 (M7 + M4) inside the same module | Real-time work, KISS to radio MCU |
| Network | WiFi/BT (Murata 1DX), Gigabit Ethernet over carrier, SDIO | The base's "fat pipe" path |

**Heads up!** The i.MX 8M Mini does *not* have the NPU that the i.MX 8M Plus has. There is no dedicated neural accelerator on the X8. Every inference runs on the A53 cores, with NEON. Plan model sizes accordingly.

### 1.2 What "fits" in practice on the tractor X8

Realistic CPU-only A53 throughput, single-threaded INT8 with NEON-optimised kernels (TFLite / ncnn / ONNX Runtime XNNPACK):

| Model class | Example | Latency on 4× A53 1.8 GHz | Notes |
|---|---|---|---|
| MobileNetV3-Small classifier (224×224, INT8) | image classification | 25–40 ms | Fits easily; ~3 MB model |
| YOLOv8-nano (320×320, INT8) | object detection | 80–150 ms | ~6 MB model; viable for 1–2 fps detection |
| YOLOv8-nano (192×192, INT8) | object detection | 35–60 ms | Aggressive; fine for our farm classes |
| MobileSAM (segment anything, mobile) | segmentation prompt | 200–400 ms | Stretch; only on-demand |
| Tiny U-Net for change detection | binary mask | 15–30 ms | Custom-trained; plausible at every frame |
| Small denoiser (3-layer CNN) | preprocess | 10–20 ms | Worth running before WebP |
| Whisper-tiny | audio transcription | not relevant | mentioned for completeness — no audio in v25 |

**Pro tip:** The H747 co-MCU's M7 core has an FPU and ~480 KB SRAM. Tiny CNNs (TensorFlow Lite Micro, ~50 KB models) can run on it for things like "is the camera image dark / overexposed / lens-occluded" — preprocessing decisions that don't need the A53. This frees the A53 for the heavier model.

### 1.3 What "fits" on the base X8

Same silicon, but with three big advantages:

1. **Wall power** — multi-threaded inference is fine, can run all 4 cores hot.
2. **USB host** — you can plug in a [Coral USB Accelerator](https://www.digikey.com/en/products/detail/coral/G950-01456-01/10256669) (Edge TPU, 4 TOPS INT8, ~$60) or a [Hailo-8L M.2](https://www.digikey.com/en/products/detail/hailo/HM21LB1C2LAE/19200381) on the carrier's M.2 slot (13 TOPS, ~$130). That's a ~30–100× speedup for compatible models.
3. **No latency budget on the radio side** — the base can spend 200 ms post-processing without affecting the operator's control loop.

With a Coral USB on the base, the model menu opens up dramatically:

| Model | Resolution | Coral Edge TPU latency | Use |
|---|---|---|---|
| Real-ESRGAN-x4 (Edge-TPU port) | 96×64 → 384×256 | 15–30 ms | Super-resolve thumbnails (already in v1.0 §4) |
| YOLOv8-medium | 640×640 | 20–40 ms | Robust farm-object detection |
| MiDaS-small (depth) | 256×256 | 25–50 ms | Synthetic depth overlay |
| Stable-Diffusion-Turbo (4-step) on a small GPU | 512×512 | 1–3 s | Inpainting dropped tiles (§5) |
| RIFE-v4 | 384×256 | 20 ms | Frame interpolation between thumbnails |
| Segment-Anything (SAM-Lite) | 1024×1024 | 200 ms | "Click to highlight this object" UI |

**Recommended base-side accelerator:** [Coral M.2 Accelerator with Dual Edge TPU](https://www.digikey.com/en/products/detail/coral/G650-04686-01/13770893) — fits the X8 carrier's M.2 slot, ~$40, no USB cable, no thermal issues. Approve in the BOM as an *optional* line; the pipeline must degrade gracefully without it.

---

## 2. The big idea — tile-delta transmission

This is the single highest-impact change. Instead of sending a whole new compressed image every refresh, **send only the tiles that changed**.

### 2.1 Concept

Divide the captured frame into a grid of small tiles (e.g. 32×32 px on a 384×256 image → 12×8 = 96 tiles). For each tile compute a perceptual hash (pHash, 64-bit) and a small motion estimate. Transmit:

1. **Header (8 B):** frame counter, wall-clock seconds, tile-grid version.
2. **Change bitmap (12 B):** one bit per tile saying "did this tile change vs last sent." 96 bits = 12 B.
3. **Changed tiles (variable):** for each "1" bit, a WebP-compressed 32×32 sub-image. With aggressive WebP at q15–q25, a 32×32 tile is ~40–120 B.
4. **Optional ROI burst (variable):** see §3 — the centre tiles get extra-quality WebP regardless.

### 2.2 Bandwidth model

Assume the operator is driving forward through a field. The horizon (top 1/3 of frame) and the immediate ground in front of the tractor (bottom 1/3) change every refresh; the middle 1/3 (mid-field) changes slowly. Typical change rate: **30% of tiles per refresh** when moving, **<5%** when stationary.

| Scene state | Tiles changed | Bytes per refresh | Refresh achievable on Profile B (~600 B/s P3 budget) |
|---|---|---|---|
| Stationary, just IMU/GPS sway | ~3 of 96 | 8 + 12 + 3×80 = 260 B | every 0.43 s |
| Slow forward driving | ~20 of 96 | 8 + 12 + 20×80 = 1620 B | every 2.7 s |
| Fast turning / dust | ~60 of 96 | 8 + 12 + 60×80 = 4820 B | every 8.0 s — **falls back to keyframe** |
| **Full keyframe (current pipeline)** | 96 of 96 | ~900 B (one big WebP) | every 1.5 s |

So delta transmission **wins big when stationary or driving slowly** (the common case during digging, loading, precision work) and **loses to keyframe when scene is changing fast everywhere**. Solution: hybrid mode.

### 2.3 Hybrid keyframe + delta protocol

Borrow from H.264:

- **I-frame:** full WebP image, ~900 B, every N=20 refreshes (~30 s) or whenever delta would exceed keyframe cost.
- **P-frame:** delta tiles only, every other refresh.
- **B-frame:** none. We have no future-frame buffer and the operator can't tolerate the look-ahead latency.

Tractor decides I vs P locally each refresh:

```
estimated_p_bytes = HEADER + BITMAP + sum(webp_size_estimate(t) for t in changed_tiles)
if estimated_p_bytes > keyframe_bytes * 0.7:
    send_keyframe()
else:
    send_delta()
```

The base maintains a **persistent canvas** of the last full reconstruction. P-frames overwrite only the changed tiles. Loss handling: each P-frame header carries the I-frame sequence number it's deltaing from; if base hasn't seen that I, request a fresh I via a P0 control byte (`req_keyframe=1` flag in the next ControlFrame ACK).

### 2.4 Tile-change detection on the tractor

Three options, increasing in cost:

1. **pHash + threshold** (~1 ms/tile, 100 ms total for 96 tiles): compute 64-bit DCT-based pHash; transmit if Hamming distance > T from last sent. Simple, robust, no false negatives on illumination shifts; some false positives.
2. **SSD on downsampled tile** (~0.3 ms/tile, 30 ms total): compute sum-of-squared-differences on the 8×8 down-sampled tile. Faster, slightly more false positives.
3. **Tiny U-Net change detector** (~30 ms total, single pass): one CNN inference outputs a 12×8 binary "changed" mask directly. Catches semantic changes (a person walking by at low contrast) that pHash misses. Custom-trained on a ~10k-frame farm dataset.

Ship #1 in v25; promote to #3 in v26 once we have training data.

### 2.5 Risks and mitigations

| Risk | Mitigation |
|---|---|
| **Drift:** rounding errors in successive deltas accumulate across many P-frames | I-frame every 20 refreshes flushes the canvas |
| **Loss:** a P-frame's tiles never arrive, base shows stale tiles indefinitely | Each tile carries a 4-bit age in the canvas; if age > 5 refreshes the tile is rendered with a "stale" tint |
| **Jitter:** changed-tile count varies wildly, so refresh cadence varies | Operator-visible "refresh: 1.2 s avg / 4.8 s max" indicator; fallback to fixed-cadence keyframe-only if delta is starving |
| **Camera shake** (engine vibration): every tile changes every frame, defeats the scheme | Apply 2-pixel sub-pixel registration on the tractor before tile-diffing. Also defeated by stiffer camera mount — partly a mechanical fix |

### 2.6 On-air format sketch

```
TileDeltaFrame {
    uint8_t  frame_type;        // 0=I, 1=P
    uint16_t seq;
    uint16_t base_seq;          // for P: which I this deltas from
    uint32_t wall_seconds;
    uint8_t  grid_w, grid_h;    // e.g. 12, 8
    uint8_t  changed_bitmap[ceil(grid_w*grid_h/8)];  // 12 B for 12x8
    // Then for each '1' bit, in row-major order:
    //   uint8_t  tile_size_minus1;   // bytes-1 of the WebP blob
    //   uint8_t  tile_data[size];
}
```

P3 priority. Fragmented across LoRa frames using the existing fragment scheme, with the §3 25 ms airtime cap per fragment.

---

## 3. Region-of-interest (ROI) budget allocation

The operator does not look at every pixel equally. They look at:

1. **What the bucket is touching** (bottom-centre tiles, 80% of attention during digging/loading).
2. **The path ahead** (centre tiles, 80% of attention during driving).
3. **Anywhere a moving object appeared** (priority spike, see §4).

Right now the WebP encoder distributes quality uniformly across the frame. Replace that with:

### 3.1 Salience-weighted tile budget

After the §2 changed-tile selection, allocate the remaining byte budget non-uniformly:

```
operator_mode = current valve activity:
    if boom/bucket valves active     -> "loading" mode, ROI = bottom-centre 4 tiles
    elif drive valves active         -> "driving" mode, ROI = centre 6 tiles
    else                             -> "idle"   mode, ROI = uniform

For each changed tile:
    if tile in ROI:
        encode at WebP q40 (~140 B avg)
    else:
        encode at WebP q15 (~50 B avg)
```

The operator's *current task* is a free signal — we already have the valve states in telemetry. Use them.

### 3.2 Operator-cued ROI

Add a "look here" tap on the base UI. When the operator taps a region in the displayed image, the base sends a P0 control byte `roi_x, roi_y` (3 B total) and the tractor moves the ROI window there for the next N refreshes. Useful for "I see something funny in the upper-right, give me detail there."

### 3.3 Persistent ROI for safety zones

Configurable ROI tiles that are *always* high-quality, regardless of mode:

- The bucket-tip area (always — it's where collisions happen).
- A configurable "operator's blind spot" tile (e.g. behind-left if there's a known obstacle).

These cost ~3 tiles × 140 B = 420 B per keyframe, pre-allocated.

---

## 4. On-tractor semantic compression — open-source AI on the X8

This is the high-effort, high-reward path. Run a small object detector on the tractor X8, use its output to bias what gets transmitted in detail.

### 4.1 Model selection — what fits and is publicly available

All Apache-2.0 / MIT / GPL-compatible:

| Model | License | Size (INT8) | A53 latency | Fitness for farm/offroad |
|---|---|---|---|---|
| [YOLOv8-nano](https://github.com/ultralytics/ultralytics) | AGPL-3.0 (concern!) | 6 MB | 80 ms @ 320² | Strong general-purpose; license blocks commercial closed-source |
| [YOLOv5-nano](https://github.com/ultralytics/yolov5) | AGPL-3.0 (concern!) | 4 MB | 60 ms @ 320² | Same |
| [NanoDet-Plus](https://github.com/RangiLyu/nanodet) | Apache-2.0 | 2.3 MB | 30 ms @ 320² | **Recommended** — clean license, very fast |
| [PP-PicoDet](https://github.com/PaddlePaddle/PaddleDetection) | Apache-2.0 | 1.0 MB | 20 ms @ 320² | Even faster; less mature ecosystem |
| [MobileNetV3-Small classifier](https://github.com/tensorflow/models) | Apache-2.0 | 3 MB | 25 ms @ 224² | Pure classifier; pair with sliding window |
| [EfficientDet-Lite0](https://github.com/google/automl) | Apache-2.0 | 4 MB | 50 ms @ 320² | Solid alternative to NanoDet |
| [YOLOX-Nano](https://github.com/Megvii-BaseDetection/YOLOX) | Apache-2.0 | 3.5 MB | 45 ms @ 320² | Strong accuracy/license trade |

**Recommendation:** **NanoDet-Plus** as primary, fine-tuned on a custom farm dataset. Apache-2.0 is OSE-friendly. ~30 ms per inference at 320×320 on the A53 leaves headroom for the rest of the image pipeline.

**Avoid Ultralytics YOLOv8/v5 unless OSE is comfortable with AGPL.** OSE's other code is generally permissive — adopting AGPL anywhere in the firmware path may have implications for the rest of the project.

### 4.2 What classes to train on

Public datasets with Apache/MIT/CC-BY weights or trainable from scratch:

- **COCO** (CC-BY-4.0) — provides "person", "bicycle", "car", "truck", "dog", "cat", "horse", "sheep", "cow", "bird". Enough to start.
- **OpenImages V7** (CC-BY-4.0) — adds "tree", "fence", "tractor"(!), "plant", "rock", "puddle".
- **Roboflow Universe** (mixed licenses, check each) — many farm/agriculture datasets including weeds, livestock, fences.
- **Custom OSE dataset** — collect 2–5k labelled frames from actual LifeTrac operation. This is the highest-impact addition; do this once and the model gets dramatically better at *our* environment.

Target classes for v25: **person, animal (any), vehicle (any), large_obstacle (rock/log/post), implement_attached, implement_detached**. Six classes, simple to hand-label, directly useful.

### 4.3 How detection biases transmission

```python
detections = nanodet_infer(frame)              # ~30 ms on A53
high_pri_tiles = set()
for det in detections:
    if det.class_id in {PERSON, ANIMAL, VEHICLE, OBSTACLE}:
        # Compute which tiles overlap this bbox
        for tile in tiles_overlapping(det.bbox):
            high_pri_tiles.add(tile)

# In §3.1 budget allocation:
for tile in changed_tiles:
    if tile in high_pri_tiles:
        encode_quality = WEBP_Q60   # ~210 B avg
    elif tile in roi_tiles:
        encode_quality = WEBP_Q40
    else:
        encode_quality = WEBP_Q15

# Also: send a tiny "detections" sidecar (P2 priority)
sidecar = pack_detections(detections)   # ~6 B per detection, max 8 dets = 50 B
```

The base receives both the image and the detection metadata. It draws bounding boxes on the rendered canvas in **vector graphics**, not bitmap — so detection overlays are crisp regardless of how compressed the underlying tiles are. This is a huge UX win.

### 4.4 The *killer* feature — "person detected" pre-emption

If a `PERSON` detection appears in the frame and was *not* present in the previous frame, the tractor:

1. Promotes the next image transmission to **P1** priority for one frame.
2. Sends a 1-byte alert (`PersonAppearedFrame`) at **P0** in the very next slot.
3. Adds a "PERSON DETECTED" annotation to the tractor's local logger regardless of whether the operator sees the image.

The base UI pops a yellow-border alert on the operator's console. This is the kind of feature that justifies the AI inference cost — it converts "saw a thumbnail too late" into "got an explicit warning within 200 ms."

**Pro tip:** Tune the model's confidence threshold *high* (e.g. 0.7) for this alert path to avoid alarm fatigue from false positives. Use a lower threshold (0.4) for the ROI-biasing path where false positives only cost bytes.

### 4.5 Other on-tractor model uses

- **Lens-occlusion detector:** 50 KB classifier on the H747 M7 → "wipe the lens" UI hint when the camera is mostly mud.
- **Auto-exposure lookahead:** detect that the camera is about to point at the sun, pre-tone-map.
- **Quality predictor:** 200 KB MobileNet-Tiny → predicts "this frame is uninteresting, defer transmission" when the scene hasn't meaningfully changed (similar to §2.4 but at the whole-frame level).

---

## 5. Base-side AI — where the real horsepower goes

The base X8 (especially with a Coral M.2) can run models 100× larger than the tractor's. Use it.

### 5.1 Persistent scene buffer + temporal super-resolution

This is the architectural twin of §2. The base maintains:

- **Canvas:** the most recent reconstruction at native (e.g. 384×256) resolution.
- **History:** the last 8 reconstructions, time-stamped.
- **Super-res output:** the canvas after Real-ESRGAN ×4 → 1536×1024.

When a new I- or P-frame arrives:

1. Update the canvas (replace changed tiles).
2. Stack the 8 most recent canvases in time (8×384×256×3).
3. Run a temporal super-resolution model on the stack — even **simple averaging** with sub-pixel registration produces a sharper output than single-frame SR. Better: a small ConvLSTM-based VSR model.

Public models that fit:

- [BasicVSR++](https://github.com/open-mmlab/mmagic) (Apache-2.0) — 8 MB, 100 ms on Coral for our resolution. State-of-the-art video SR.
- [RVRT](https://github.com/JingyunLiang/RVRT) (Apache-2.0) — heavier, better quality.
- [Real-ESRGAN-General-x4v3](https://github.com/xinntao/Real-ESRGAN) (BSD) — single-frame; already in v1.0 plan.

Temporal SR gives ~30% sharper output than single-frame on stationary or slowly-moving scenes. On scenes that just changed, it gracefully degrades to single-frame.

### 5.2 Inpainting dropped tiles

If a P-frame's tile fragment is lost and never recovered, the tile in the canvas goes stale. Two options:

1. **Visible stale tint** (default, §2.5): tint the tile yellow, show its age. Honest, mandatory for safety.
2. **Optional AI inpainting overlay**: run a small inpainter to fill the stale tile from neighbouring fresh tiles. Display under the §5 of [v1.1 LoRa doc](2026-04-27_LoRa_InDepth_Analysis_ClaudeOpus4_7_v1_1.md) "Enhanced" badge.

Models that fit on Coral:

- [LaMa-Fourier](https://github.com/saic-mdal/lama) (Apache-2.0) — 50 MB, 80 ms for a small region. Excellent at structure-preserving fill.
- [Stable-Diffusion-Turbo-1step inpainting](https://huggingface.co/stabilityai/sd-turbo) (CreativeML-OpenRAIL-M; *check terms for commercial OSE deployment*) — much heavier; needs a real GPU. Not v25.

Recommend LaMa-Fourier; the SD path is overkill for a 32×32 tile.

### 5.3 Frame interpolation between tile-deltas

RIFE (already in [v1.1 LoRa doc §5.1](2026-04-27_LoRa_InDepth_Analysis_ClaudeOpus4_7_v1_1.md#section-5)) interpolates *whole frames*. With tile-delta the base now has per-tile arrival times. We can do per-tile temporal smoothing:

- Each tile, on update, fades from its old value to the new value over ~3 display frames (50 ms transition at 60 Hz).
- No hallucinated content; just a perceptual-smoothness improvement.
- Costs ~0 ms; pure GLSL fragment shader on the GPU.

This is a clean win and should be in v25. RIFE itself is more aggressive (synthesises in-between frames) and remains optional per [v1.1 LoRa doc](2026-04-27_LoRa_InDepth_Analysis_ClaudeOpus4_7_v1_1.md).

### 5.4 Semantic upscaler — fight the small-resolution problem

[GFPGAN](https://github.com/TencentARC/GFPGAN) (Apache-2.0) and [CodeFormer](https://github.com/sczhou/CodeFormer) (CC-BY-NC; *NC blocks commercial — skip*) specialise in *face* restoration — irrelevant. But the same technique exists for general scenes:

- [SUPIR](https://github.com/Fanghua-Yu/SUPIR) (Apache-2.0) — Stable Diffusion-based scene SR. Incredible quality; needs a full GPU. Not v25; possible v26 if base gets a desktop-class GPU.
- [Real-ESRGAN-General-x4v3](https://github.com/xinntao/Real-ESRGAN) — already planned.

### 5.5 Safety-classification cross-check

Run a *second*, independent detector on the base that double-checks the tractor's detection sidecar:

- If tractor says "no person", base runs YOLOv8-medium at full 640² resolution and confirms.
- If base finds a person the tractor missed, *highlight it in the UI* and log the miss for later model retraining.
- Disagreement is a flag, not an alarm — but the operator should see it.

This is cheap insurance: the base has the cycles, the tractor's model is small and might miss things. Two independent models with different training cuts make false negatives much rarer.

### 5.6 Depth + 2.5D rendering

[MiDaS-small](https://github.com/isl-org/MiDaS) (MIT) on the base, fed the latest canvas, gives a per-pixel depth map. Render the canvas as a 2.5D mesh in the browser (Three.js) — the operator can subtly tilt their viewpoint with mouse motion, getting a sense of depth from a single 2D image. Stretch goal; very cool; debatable whether it helps real operations.

---

## 6. Outside-the-box ideas

### 6.1 Procedural texture synthesis for the static background

The field doesn't change. After 5 minutes of operation, the base has seen the same patch of ground 50 times. Instead of re-transmitting it, *synthesise* it from a procedural model:

- Per session, the tractor uploads a high-quality 256×256 swatch of the current ground texture (~2 KB, once).
- The base tiles that swatch wherever the canvas tile is "background grass" (classified by the on-tractor detector).
- Transmitted bytes for the background drop to *zero*.

Trade-off: the displayed "grass" is no longer the *actual* grass — it's a stylised stand-in. Mark it with the "Synthetic" badge per [v1.1 LoRa doc §5.4](2026-04-27_LoRa_InDepth_Analysis_ClaudeOpus4_7_v1_1.md). For the operator scanning for *non-grass*, this is fine — it actively *helps* by making non-grass tiles pop visually.

### 6.2 Audio as a backchannel for "what just happened"

If the tractor X8 has a microphone (it can — USB audio is trivial), a tiny audio classifier ([YAMNet](https://github.com/tensorflow/models/tree/master/research/audioset/yamnet) — Apache-2.0, 3.8 MB, 50 ms inference) can detect:

- engine knock
- hydraulic squeal (cavitation, broken seal)
- impact (struck a rock)
- voice (shouted instruction)

Each detection → 4 B sidecar at P2. The operator gets "engine knock detected 3 s ago" without any audio bytes flowing — just the *classification* result. Not strictly image transmission, but it shares the bandwidth budget and is dramatically cheaper than transmitting audio.

### 6.3 Three-camera bandwidth multiplexing

Suppose the tractor has 3 cameras (front, rear, bucket-cam). Round-robin transmission would give each 1/3 the budget. **Better:** allocate adaptively based on operator focus and detection results.

- Default: front=70%, bucket=25%, rear=5%.
- During reverse: rear=70%, bucket=20%, front=10%.
- If detector sees a person in any camera: that camera jumps to 90% for the next 5 refreshes.

The operator never explicitly switches cameras; the system follows their attention.

### 6.4 Keyframe pre-loading from satellite imagery

The tractor knows its GPS position. Before operations begin, the base downloads a high-resolution satellite tile of the operating area (free from USGS / OpenAerialMap, no LoRa needed). The displayed "scene" is a hybrid:

- Far field (>30 m): rendered from satellite imagery, georeferenced to current tractor pose.
- Near field (<30 m): live LoRa thumbnail.

Costs zero LoRa bytes for the far field but gives the operator a wide-area context they otherwise wouldn't have. Useful for navigation; not for safety (satellite is days/weeks old).

### 6.5 Event-driven transmission — skip refreshes when nothing matters

A consequence of §4: if no detection class above some confidence threshold appears in the frame, **and** the tile-change bitmap has <5% changed tiles, **and** the operator hasn't moved any control in the last 2 s, **skip this transmission entirely**. Send a 1-byte heartbeat instead.

This frees the channel for the next ControlFrame burst and saves battery on the tractor. The base displays "scene quiescent — last update 6 s ago" honestly.

### 6.6 Reverse super-resolution — train a model on *our* footage

Real-ESRGAN is trained on generic imagery. A version fine-tuned on LifeTrac footage would do markedly better at our specific scene types (dirt, vegetation, tractor parts in frame). After ~10 hours of operation we have ~50k frames; fine-tune the model overnight, deploy to the base. The model improves over the life of the project. Free (CPU training, slow but works), and the dataset is OSE's to keep.

### 6.7 Latent-space transmission (the v26 endgame)

Train a small autoencoder (~2 MB, ~30 ms encode on A53):

- Tractor encodes frame to a 64-dim latent vector (256 B).
- Transmits the latent (P3, fits in 8 LoRa fragments).
- Base decodes with the matching decoder (~50 ms on Coral) → 384×256 RGB.

Quality at 256 B is *much* better than WebP at 256 B because the autoencoder is trained on the actual data distribution. The catch: encoder and decoder must be perfectly matched, both bumped together on firmware updates, and the model needs farm-imagery training data we don't have yet. **v26 work; collect the data in v25.**

---

## 7. Gemini 3.1 Pro: Top Recommendation & Action Plan for v25

After reviewing all theoretical and practical pipelines in this analysis, the ultimate solution for v25 must balance **deterministic radio airtime**, **safety**, and **operator situational awareness** without requiring a massive R&D cycle for custom AI models. 

### The Recommended Architecture: "Smart-Delta + Semantic Safety"

1. **Primary Transport: Tile-Delta Transmission (§2).** 
   This is the non-negotiable foundation. Transmitting full frames over LoRa at 20Hz control rates is physically destructive to the network. Establishing a persistent HTML5 Canvas on the Base UI and only drip-feeding changed 32x32 WebP blocks using the priority queue method solves the physics problem of LoRa bandwidth using computer science.
   
2. **Safety Bandwidth Override: NanoDet-Plus (§4).** 
   While visual tiles may update slowly, safety cannot wait for a block transfer. Run **NanoDet-Plus** on the Tractor X8. If a `PERSON` or `OBSTACLE` is detected, the system must bypass the visual tile queue and immediately transmit a lightweight semantic alert (bounding box coordinates + class ID). The Base UI then instantly draws a vector box to warn the operator, even if the underlying pixels haven't updated yet. This directly separates the latency of *safety data* from the latency of *visual data*.
   
3. **Base-Side Offload: Temporal Super-Resolution (§5).** 
   The Tractor should not waste precious cycles or battery encoding high-resolution images. Send blocky, highly compressed tiles (WebP q15), and let the Base Station's X8 (or better, the Operator's WebBrowser via WebGL/WebGPU) run the heavy lifting. Using a tool like Real-ESRGAN/BasicVSR++ to smooth and upscale the incoming patches creates an acceptable UX out of heavily degraded transmission data.

**Why this is the winner:** 
It perfectly silos the problems. It relies on proven, off-the-shelf open-source models (NanoDet + WebP + ESRGAN) instead of requiring a custom latent-space autoencoder training pipeline immediately. It guarantees safety via tiny AI metadata, keeps the channel clear for control frames by only sending what changed, and uses the unconstrained base-side compute to make the video feed actually usable for the human operator.

### GitHub Copilot recommendation: "No-Coral Smart-Delta Baseline"

I agree with the Gemini recommendation, but I would make one ordering change: treat Coral and heavy base-side AI as optional accelerators, not as requirements for the v25 production path. The top recommendation is a **no-Coral-first Smart-Delta pipeline** that works on the two Portenta X8s alone, then improves automatically if the base-side accelerator spike passes.

**Recommended v25 stack:**

1. **Primary visual transport:** hybrid keyframe plus tile-delta P3 image objects, using pre-diff image registration, 32x32 or 24x24 WebP tiles, and a strict fragment-airtime cap so imagery can never starve control, heartbeat, or safety traffic.
2. **ROI policy:** three-ring ROI allocation driven by valve state, active camera, operator click, and later detector hits. Ring 0 gets the bucket tip, contact patch, person/animal/obstacle boxes, and any operator-cued tile; Ring 2 collapses first under weak RF.
3. **Tractor compute:** keep the tractor conservative. Do registration, tile scoring, WebP encode, and eventually a CPU-only INT8 NanoDet-Plus or YOLOX-Nano sidecar for person/animal/vehicle/obstacle. Do not depend on a tractor-side accelerator in v25.
4. **Base reconstruction:** maintain a persistent per-camera canvas with per-tile age, confidence, stale tint, completeness percent, and explicit RAW / ENHANCED / CACHED / PREDICTED / SYNTHETIC / STALE badges. The operator must always know which pixels are current truth and which are display help.
5. **Browser as display engine:** push compositing, fades, vector overlays, bounding boxes, staleness clocks, and raw/enhanced toggles into the browser whenever possible. This keeps the base X8 available for reassembly, logging, and optional safety cross-checks.
6. **Base AI as additive:** start with deterministic OpenCV denoise/deblock/sharpen and optional CPU super-resolution. If the Coral M.2 validation passes, use it on the base for independent object detection and super-resolution; if it fails, the image pipeline still ships.
7. **Safety sidecars:** when tractor-side detection is available, send tiny P0/P1 semantic alerts and P2 detection sidecars immediately. Do not wait for the visual tile stream to catch up before warning the operator.

**Why this is my top pick:**
It gives the project the most useful visual system before any custom dataset, learned codec, or accelerator dependency exists. It is deterministic enough to schedule, honest enough for operator trust, and modular enough to accept the later wins: Coral base detection, motion-vector microframes, background cache, and LifeTrac-trained super-resolution. The canonical v25 build should therefore be **real pixels first, targeted deltas second, semantic alerts third, and enhancement fourth**.

**Fallback ladder:**

| Link condition | Display mode |
|---|---|
| Healthy | Tile-delta + ROI rings + canvas + browser enhancement |
| Moderate pressure | ROI-first tile deltas, lower cadence, fewer context tiles |
| Weak | Y-only ROI or SSDV/progressive proof-of-life frames |
| Very weak | Motion-vector or wireframe degraded mode, clearly marked predicted/wireframe |
| Control-only | Frozen canvas with age clock, semantic alerts only, no image chunks |

### Copilot final recommendation summary

If one recommendation must be treated as canonical for v25, use this:

1. Ship the no-Coral Smart-Delta baseline as the default production path.
2. Make deterministic keyframe plus tile-delta transport, ROI rings, and staleness-honest UI mandatory before optional AI work.
3. Keep tractor compute focused on prioritization and safety sidecars, while base and browser do the heavy reconstruction and rendering.
4. Treat Coral acceleration as a strict add-on: automatic upgrade when present, zero functional dependency when absent.
5. Enforce the validation gate that image traffic never causes P3-induced P0 starvation.

---

## 8. Implementation plan for v25

What ships in the first build, in implementation order:

### Phase 1 — foundation (can ship without any AI)

1. **Tile-delta protocol** (§2). I/P frames, change bitmap, per-tile WebP. ~2 weeks.
2. **ROI budget allocation from valve state** (§3.1). Reads current valve activity from telemetry, biases WebP quality. ~3 days.
3. **Persistent base canvas** (§5.1 first half — no SR yet). Just the tile-replacement logic. ~1 week.
4. **Per-tile fade transition** (§5.3). GLSL shader. ~2 days.
5. **Stale-tile tinting** ([v1.1 LoRa doc §5.4](2026-04-27_LoRa_InDepth_Analysis_ClaudeOpus4_7_v1_1.md) integration). ~1 day.
6. **Operator-cued ROI** (§3.2). Click-to-zoom, 3 B back-channel. ~1 week.

### Phase 2 — AI on the base (no tractor changes needed)

7. **Real-ESRGAN-General-x4v3 on Coral** (§5.1, [v1.0 LoRa doc §4.3](2026-04-27_LoRa_InDepth_Analysis_ClaudeOpus4_7_v1_0.md)). ~1 week to deploy.
8. **YOLOv8-medium on Coral as safety cross-check** (§5.5, runs on the canvas). ~1 week.
9. **Optional RIFE frame interpolation** ([v1.1 LoRa doc §5.1](2026-04-27_LoRa_InDepth_Analysis_ClaudeOpus4_7_v1_1.md)). ~3 days.

### Phase 3 — AI on the tractor (the stretch goal)

10. **NanoDet-Plus on tractor X8, COCO-trained** (§4). ~2 weeks to deploy and validate.
11. **Detection-biased ROI** (§4.3). ~3 days once detector is live.
12. **PersonAppearedFrame P0 alert** (§4.4). ~3 days.
13. **Detection sidecar transmission** (§4.3 last paragraph). ~3 days.

### Phase 4 — data collection for v26

14. **Log all canvases + detections + operator actions to the §8.10 black-box logger.** Builds the OSE-specific training set for fine-tuning in v26. ~2 days; do this from day one.

If schedule slips, drop Phase 3. Phases 1+2 alone produce a *significantly* better operator experience than the v1.0 baseline.

---

## 8. What this changes in the canonical docs

When the design lands, update:

- **MASTER_PLAN.md §8.17** — replace the simple "WebP thumbnail every N seconds" wording with the I/P-frame protocol and ROI mode.
- **LORA_PROTOCOL.md** — add `TileDeltaFrame` (P3) and `PersonAppearedFrame` (P0) frame types.
- **HARDWARE_BOM.md** — add an *optional* line for the Coral M.2 Dual Edge TPU on the base carrier.
- **arduino_libraries.txt** — no change (this is all Linux-side).
- **TRACTOR_NODE.md / BASE_STATION.md** — add an "image pipeline" section pointing to this analysis.
- **TODO.md** — phases 1–4 above.

Do *not* change [v1.0 LoRa doc](2026-04-27_LoRa_InDepth_Analysis_ClaudeOpus4_7_v1_0.md) or [v1.1 LoRa doc](2026-04-27_LoRa_InDepth_Analysis_ClaudeOpus4_7_v1_1.md). They remain valid; this document layers on top.

---

## 9. Open questions for the next review

1. Is OSE comfortable with AGPL anywhere in the firmware tree? (Settles the YOLOv8 vs NanoDet question.)
2. Is a $40 Coral M.2 in the BOM acceptable, or must everything degrade to no-accelerator pure-CPU?
3. How many cameras on v25 — one, two, or three? §6.3 only matters if ≥2.
4. Is there a microphone in the v25 enclosure plan? §6.2 needs one.
5. Will OSE collect and host the LifeTrac-specific training dataset publicly? (Big community win if yes.)

These don't block design; they refine scope.

---

**End of v1.0.** Companion to the LoRa v1.0 / v1.1 analyses. Treat tile-delta + ROI + persistent canvas as the *baseline* image pipeline going forward; treat on-tractor detection and base-side temporal SR as the high-value additions; treat learned codecs and procedural-background as the v26 frontier.

---

## 10. Peer-review synthesis (added 2026-04-27)

After this v1.0 was written, three companion analyses landed in the same folder:

- [2026-04-27_Image_Transmission_Analysis_Gemini_3.1_Pro_v1.0.md](2026-04-27_Image_Transmission_Analysis_Gemini_3.1_Pro_v1.0.md) — Gemini 3.1 Pro
- [2026-04-27_Image_Transmission_Portenta_AI_Analysis_Copilot_v1_0.md](2026-04-27_Image_Transmission_Portenta_AI_Analysis_Copilot_v1_0.md) — GitHub Copilot v1.0
- [2026-04-27_Image_Transmission_Portenta_AI_InDepth_Analysis_Copilot_v1.0.md](2026-04-27_Image_Transmission_Portenta_AI_InDepth_Analysis_Copilot_v1.0.md) — GitHub Copilot In-Depth v1.0

The three reviews independently converge on most of the same conclusions as this doc (tile-delta over whole-frame thumbnails, ROI from control state, persistent base canvas with staleness, two-detector safety pattern, Phase 1 ships without AI). That convergence is reassuring — the design is in a stable basin.

But each review contributed at least one genuinely new idea worth folding into the v25 plan. Listed in priority order:

### 10.1 Operator browser as a third compute tier (Gemini §1, Copilot v1.0 §2.4 / §4.4)

**Idea:** The operator's laptop or tablet is almost always more powerful than the base X8 (modern laptop = 10–100× the FLOPS of an i.MX 8M Mini, with a real GPU). Treat the browser as a third compute surface and push display-only work — patch compositing, per-tile fade shaders, vector overlays, optional WebGPU/WebNN inference — into it via WebAssembly/WebGL/WebGPU/WebCodecs. The base X8 stays the *trusted* reassembly + safety-detector node; the browser becomes the *display engine*.

**Why it matters:** Frees the base X8's A53 cores for safety cross-check inference instead of canvas painting. Also makes the system degrade gracefully — if the browser device is weak, the base falls back to server-side rendering; if it's strong, the operator gets smoother visuals at no extra base cost.

**Ship in v25?** Yes, opportunistically. The per-tile fade in §5.3 already runs as a GLSL fragment shader — that's already browser-side. Extend the same pattern to vector overlays, staleness clocks, and bounding-box rendering. Defer WebGPU inference to v26 because of browser-compatibility variance.

**Doc impact:** Add a short "browser-side compute" subsection to [BASE_STATION.md § Image pipeline](../DESIGN-CONTROLLER/BASE_STATION.md). One paragraph; no new code surface today, just a published architectural rule.

### 10.2 Pre-diff image registration (Copilot v1.0 §5.4)

**Idea:** Before computing per-tile pHash deltas, run a cheap global image-registration step (~5 ms with NEON + 8×8 phase correlation on the downsampled luma plane). Tractor vibration, engine shake, and small camera mount flex make every tile *look* changed even when the scene is static — the result is a falsely large change-bitmap and wasted bytes on essentially-duplicate tiles. Compensating for the global shift first means only *real* content changes survive into the tile-delta path.

**Why it matters:** §2.3 quotes "60–95 % bytes saved on a static scene" but that assumes a stationary tractor. With registration, the same savings hold while the tractor is *moving slowly* — which is most of the duty cycle. Without it, the savings drop to maybe 30–50 % during driving.

**Ship in v25?** Yes, in Phase 1 (§7). It's ~50 lines of OpenCV / ndimage code on the tractor X8, runs in a couple of milliseconds, and pays for itself within the first minute of operation.

**Doc impact:** Add a step between "capture" and "tile-diff" in [TRACTOR_NODE.md § Image pipeline](../DESIGN-CONTROLLER/TRACTOR_NODE.md). One bullet in `tile_diff.py`'s description.

### 10.3 ROI ring encoding (Copilot In-Depth §7.1)

**Idea:** Instead of a single rectangular ROI mask in §3, structure it as concentric quality rings:

- **Ring 0 — critical:** bucket tip + immediate contact patch + any detected person/animal. WebP q60.
- **Ring 1 — operational:** surrounding work zone (lower-centre, manipulator boom). WebP q40.
- **Ring 2 — context:** far field, horizon, sky. WebP q15 or skip-tile entirely.

This is more elegant than a binary "is this tile ROI yes/no" because it maps continuously to the operator's attention budget and degrades the right way under bandwidth pressure (Ring 2 collapses first).

**Why it matters:** The §3 single-ROI scheme has an implicit problem — tiles immediately *adjacent* to the ROI are still important (peripheral vision) but get the same low quality as the horizon. Rings fix that without protocol changes — it's just three quality rungs instead of two when allocating per-tile budget.

**Ship in v25?** Yes, in Phase 1 (§7). Replace the binary ROI mask in `roi.py` with a 3-level integer mask. Same `TileDeltaFrame` wire format.

**Doc impact:** Update §3 of this doc and the `roi.py` description in [TRACTOR_NODE.md § Image pipeline](../DESIGN-CONTROLLER/TRACTOR_NODE.md).

### 10.4 Motion-vector microframes (Copilot In-Depth §7.3)

**Idea:** Between full I/P frames, send tiny optical-flow microframes — a 12×8 grid of (dx, dy) int8 pairs = 192 B per microframe. The base displays them as drift on the existing canvas tiles (each tile shifts in the indicated direction over the next display interval) so the operator perceives continuous motion even when no new pixel data has arrived. Costs essentially nothing on the air.

**Why it matters:** Big win for the "scene quiescent" case in §6.5 *and* for low-link conditions where pixel updates are sparse. The operator sees the world *flowing* past the cab even when bytes are scarce — no more frozen-canvas-when-driving-down-a-road problem.

**Ship in v25?** Phase 2 (§7) — once the canvas + per-tile-fade shader is up, microframe overlay is a few dozen lines of GLSL plus one new topic ID (suggest `0x28 video/motion_vectors`) and one new opcode in [LORA_PROTOCOL.md](../DESIGN-CONTROLLER/LORA_PROTOCOL.md). Mark every microframe-driven canvas region with the **"Predicted"** badge from the v1.1 LoRa §5.4 honesty rules — these pixels were never actually transmitted.

**Doc impact:** New topic ID `0x28` in [LORA_PROTOCOL.md](../DESIGN-CONTROLLER/LORA_PROTOCOL.md); new section in [BASE_STATION.md § Image pipeline](../DESIGN-CONTROLLER/BASE_STATION.md); new test gate ("Predicted-badge always visible on motion-vector regions") in TODO Phase 5C.

### 10.5 Vectorised wireframe / geometry-first degraded mode (Gemini §3.1, Copilot In-Depth §7.4)

**Idea:** As an *extreme* degraded mode — when the link drops below the byte budget where even tile-delta is starving — fall back to transmitting only edge primitives (Canny or [PiDiNet](https://github.com/hellozhuo/pidinet) on the tractor, ~10 ms on A53), packed as line-segment vectors `(x0, y0, x1, y1)` int8 quads = 4 B per segment. A scene of 30–40 segments fits in ~150 B and gives the operator a Tron-style wireframe of bucket position, ground edge, and obstacle silhouettes — enough situational awareness to *stop safely* even when no pixels can flow.

**Why it matters:** This is a real **safety-critical** degraded mode, not a stunt. The v1.1 LoRa analysis §5.5 talks about "what does the operator see when the link collapses?" — the current answer is "stale tiles with yellow tint." This gives them *something live* even at 50 B/s of P3 budget. Mark with **"Wireframe"** badge.

**Ship in v25?** Phase 3 stretch (§7). Belongs alongside the optical-flow degraded mode already in v1.1 LoRa §5.3. Two degraded modes are better than one — the system can pick whichever costs less under current conditions (motion-vector microframes for slow drift, wireframe for "I have to ship *something* recognisable").

**Doc impact:** Mention in [BASE_STATION.md § Image pipeline](../DESIGN-CONTROLLER/BASE_STATION.md) as a deferred-but-planned degraded mode. Probably new topic ID `0x29 video/wireframe` for v25.5 or v26.

### 10.6 Long-lived background cache with foreground/background invalidation (Copilot In-Depth §7.5)

**Idea:** §6.1 (procedural background) is one way to avoid re-sending the static background. Another, simpler way: the base maintains a **long-lived background cache** per camera (e.g., last 30 s rolling median per tile). The tractor's segmenter (§4) classifies each tile as `foreground` or `background`. The background cache only refreshes from `background`-class tiles; the foreground (operator's bucket, moving objects, animals) gets the live stream. When a tile is missed, the base patches with the background cache rather than a yellow-tint hole — usually correct, always visually continuous.

**Why it matters:** Synergy with §6.1 — the procedural-texture idea works for "field of grass," but the *cached real background* works for "shed wall, pile of timber, parked truck" too. Less alien-looking than procedural texture and just as cheap on the wire.

**Ship in v25?** Phase 2 (§7) once the segmenter is up. ~100 lines of Python on the base; no protocol change. Marked with the **"Cached"** badge with age in seconds — never silently substituted.

**Doc impact:** Extend the `canvas.py` description in [BASE_STATION.md § Image pipeline](../DESIGN-CONTROLLER/BASE_STATION.md). Adds one bullet to the §5 base-side responsibilities.

### 10.7 Semantic-map fallback for total degradation (Gemini §3.3, Copilot In-Depth §7.2)

**Idea:** In the absolute worst-link case, the tractor sends a tiny RLE-encoded classification grid — e.g., 12×8 tiles, 1 B per tile (sky/dirt/grass/metal/obstacle/person) → ~30–50 B after RLE. The base renders a stylised top-down or 2.5-D synthetic view from this. **Not pixel-accurate, just semantically accurate** — "there's an obstacle two tiles ahead-left."

**Why it matters:** Combined with the existing GPS + map overlay, this gives the operator a *navigable* picture from <100 B/s. It's the lowest-bitrate visual mode short of pure metadata.

**Ship in v25?** Phase 3 / v26. Same model already runs for ROI biasing (§4) — only the encoding-and-render path is new. Marked with the **"Synthetic"** badge from v1.1 LoRa §5.4.

**Doc impact:** Same topic-ID treatment as wireframe. Defer concrete spec to v26.

### 10.8 Two ideas evaluated and **not** adopted

For completeness, two ideas from the peer reviews that I considered and rejected for v25:

- **Variational autoencoder latent transport (Gemini §3.2, Copilot In-Depth §10 Phase D).** Already covered by §6.7 above. Same conclusion: needs farm-specific training data we don't yet have. v26.
- **Heavy on-tractor monocular depth (MiDaS-class, mentioned in Copilot In-Depth §4.2).** Cost-prohibitive on the i.MX 8M Mini's A53 cores at any useful framerate (>200 ms per frame), and the resulting depth is brittle on dust, rain, low light — the conditions where it's needed most. Stereo cameras or a dedicated ToF/LiDAR sensor is the right answer, and that's a v26+ hardware decision.

### 10.9 Summary of v1.0 → v1.0+peer-review delta

| # | New idea | Source | Phase | Wire-format change? |
|---|---|---|---|---|
| 10.1 | Operator browser as third compute tier | Gemini, Copilot v1.0 | 1 (opportunistic) | No |
| 10.2 | Pre-diff image registration on tractor | Copilot v1.0 | 1 | No |
| 10.3 | ROI rings (3 levels, not binary) | Copilot In-Depth | 1 | No |
| 10.4 | Motion-vector microframes (`0x28`) | Copilot In-Depth | 2 | New topic ID |
| 10.5 | Wireframe degraded mode (`0x29`) | Gemini, Copilot In-Depth | 3 / v25.5 | New topic ID |
| 10.6 | Long-lived background cache | Copilot In-Depth | 2 | No |
| 10.7 | Semantic-map degraded mode | Gemini, Copilot In-Depth | 3 / v26 | New topic ID |

The four wire-format additions (`0x28` motion vectors, `0x29` wireframe, plus a v26 semantic-map ID, plus `Predicted` / `Cached` / `Wireframe` / `Synthetic` badge enum extensions in the UI) should be **reserved now** in [LORA_PROTOCOL.md](../DESIGN-CONTROLLER/LORA_PROTOCOL.md) even if only `0x28` ships in v25 — keeps the ID space tidy and signals intent.

**End of peer-review synthesis.** Treat 10.1, 10.2, 10.3, and 10.6 as Phase 1/2 "definitely ship in v25." Treat 10.4 as a stretch goal. Treat 10.5 and 10.7 as deferred-but-reserved.

---

## 11. Comparative ranking of image-transmission & processing schemes

All numbers are **engineering estimates** for the v25 hardware (Portenta X8 quad-A53 @ 1.8 GHz, 2 GB LPDDR4, no NPU on the i.MX 8M Mini) over the v25 LoRa link (SF7/BW250, ~800–1000 B per refresh of P3 budget per [v1.1 LoRa §3](2026-04-27_LoRa_InDepth_Analysis_ClaudeOpus4_7_v1_1.md), refresh ~1.5 s). They are **not** measured. Field benchmarks will move these by ±2× in either direction. Use the table to *rank* options, not to size buffers.

### 11.1 Metric definitions

| Metric | Meaning | Scale |
|---|---|---|
| **Fidelity** | Subjective quality of what the operator finally sees on-screen, vs. holding the actual camera in their hand. | 1 (unrecognisable) → 10 (looks like cable TV) |
| **FPM** | Useful display-frames-per-minute the operator perceives (after fade / interpolation / canvas patching). Higher is better. | frames/minute |
| **Tractor CPU** | % of one A53 core used continuously by this stage on the tractor X8. | 0–400 % (4 cores) |
| **Base CPU** | Same, on the base X8 (no Coral; CPU-only baseline). | 0–400 % |
| **Bytes / refresh** | Mean P3 air-bytes consumed per ~1.5 s refresh interval. | bytes |
| **Latency added** | Extra glass-to-glass latency on top of the ~600–1000 ms LoRa baseline. | ms |
| **Feasibility** | How confident we are this works in v25 with the team and the schedule. | 1 (research) → 10 (textbook) |
| **Safety honesty** | Does the operator always know what's real vs. synthetic? | ✅ / ⚠️ / ❌ |
| **Ship phase** | Where it lands in [§7](#7-implementation-plan-for-v25). | 1 / 2 / 3 / v26 |

### 11.2 Master ranking table — all schemes

Sorted by recommended ship order; phase 1 first, v26 last.

| # | Scheme | Fidelity | FPM | Tractor CPU | Base CPU | Bytes / refresh | Latency added | Feasibility | Safety honesty | Phase |
|---|---|---:|---:|---:|---:|---:|---:|---:|:-:|:-:|
| A | **Whole-frame WebP thumbnail** *(v1.0 LoRa baseline; what we'd ship without this analysis)* | 3 | ~40 | 30 % | 10 % | 800 | 0 | 10 | ✅ | (was 1) |
| B | **Tile-delta I/P frames** ([§2](#2-the-big-idea--tile-delta-transmission)) | 4 | ~40 | 50 % | 30 % | 250–800 | +20 | 9 | ✅ | 1 |
| C | **B + pre-diff image registration** ([§10.2](#102-pre-diff-image-registration-copilot-v10-54)) | 4 | ~40 | 55 % | 30 % | 200–500 | +25 | 9 | ✅ | 1 |
| D | **C + ROI rings (3-level)** ([§10.3](#103-roi-ring-encoding-copilot-in-depth-71)) | 5 (centre 7) | ~40 | 60 % | 30 % | 200–500 | +25 | 9 | ✅ | 1 |
| E | **D + persistent base canvas + per-tile fade** ([§5.1](#5-base-side-ai--where-the-real-horsepower-goes), [§5.3](#5-base-side-ai--where-the-real-horsepower-goes)) | 6 (centre 7) | ~120¹ | 60 % | 50 % | 200–500 | +50 | 9 | ✅ | 1 |
| F | **E + operator-cued ROI** ([§3.2](#3-region-of-interest-roi-budget-allocation)) | 7 in clicked tile | ~120 | 60 % | 50 % | 200–500 | +50 | 9 | ✅ | 1 |
| G | **F + browser-side render tier** ([§10.1](#101-operator-browser-as-a-third-compute-tier-gemini-1-copilot-v10-24--44)) | 7 | ~150² | 60 % | 30 % ↓ | 200–500 | +50 | 8 | ✅ | 1 |
| H | **G + base-side super-res (Real-ESRGAN-x4 CPU)** ([§5.1](#5-base-side-ai--where-the-real-horsepower-goes)) | 8 | ~150 | 60 % | 280 % | 200–500 | +250 | 7 | ⚠️ Enhanced badge | 2 |
| I | **G + long-lived background cache** ([§10.6](#106-long-lived-background-cache-with-foregroundbackground-invalidation-copilot-in-depth-75)) | 7 (no holes) | ~150 | 60 % | 50 % | 200–500 | +50 | 8 | ⚠️ Cached badge | 2 |
| J | **G + base-side independent YOLOv8-nano CPU safety cross-check** ([§5.5](#5-base-side-ai--where-the-real-horsepower-goes)) | 7 + det. overlay | ~150 | 60 % | 200 % | 200–510 | +150 | 8 | ✅ | 2 |
| K | **G + motion-vector microframes (`0x28`)** ([§10.4](#104-motion-vector-microframes-copilot-in-depth-73)) | 7 (smooth) | ~600³ | 65 % | 60 % | 250–700 | +50 | 7 | ⚠️ Predicted badge | 2 |
| L | **G + RIFE base-side frame interpolation (CPU)** *(v1.1 LoRa §5.1)* | 8 | ~600³ | 60 % | 350 % | 200–500 | +200 | 5 (CPU-only marginal) | ⚠️ Enhanced badge | 2 |
| M | **G + on-tractor NanoDet-Plus + detection-biased ROI + sidecar** ([§4](#4-on-tractor-semantic-compression--open-source-ai-on-the-x8)) | 7 + det. overlay | ~150 | 130 % | 30 % | 220–520 | +60 | 6 | ✅ | 3 |
| N | **G + `CMD_PERSON_APPEARED` P0 alert** ([§4.4](#4-on-tractor-semantic-compression--open-source-ai-on-the-x8)) | safety win | n/a | +10 % | 0 % | +5 | 0 (event) | 7 | ✅ | 3 |
| O | **Procedural background substitution** ([§6.1](#61-procedural-texture-synthesis-for-the-static-background)) | 6 (alien-tinted) | ~150 | +20 % | +20 % | −100 | 0 | 6 | ⚠️ Synthetic badge | 3 |
| P | **Wireframe / vector edge degraded mode (`0x29`)** ([§10.5](#105-vectorised-wireframe--geometry-first-degraded-mode-gemini-31-copilot-in-depth-74)) | 2 (Tron) | ~300 | 80 % | 20 % | 100–200 | +100 | 6 | ⚠️ Wireframe badge | 3 / v25.5 |
| Q | **Optical-flow-only degraded mode** *(v1.1 LoRa §5.3)* | 2 | ~600³ | 70 % | 30 % | 50–150 | +50 | 7 | ⚠️ Predicted badge | 3 |
| R | **Audio sidecar (YAMNet)** ([§6.2](#62-audio-as-a-backchannel-for-what-just-happened)) | n/a (audio) | n/a | 50 % | 5 % | +4 (event) | 0 | 8 | ✅ | 3 |
| S | **Three-camera attention multiplexing** ([§6.3](#63-three-camera-bandwidth-multiplexing)) | 6 (better cam selection) | ~150 | +5 % | +5 % | 200–500 | 0 | 9 | ✅ | 3 |
| T | **Satellite far-field hybrid** ([§6.4](#64-keyframe-pre-loading-from-satellite-imagery)) | 7 wide-area | ~150 | 0 % | +10 % | 0 (no LoRa) | 0 | 8 | ⚠️ Aged badge | 3 |
| U | **Event-driven skip-the-refresh** ([§6.5](#65-event-driven-transmission--skip-refreshes-when-nothing-matters)) | 6 (honest) | ~50 (during quiet) | −20 % | −20 % | 1 (heartbeat) | 0 | 9 | ✅ | 3 |
| V | **Semantic-map degraded mode** ([§10.7](#107-semantic-map-fallback-for-total-degradation-gemini-33-copilot-in-depth-72)) | 3 (stylised) | ~150 | 130 % | 50 % | 30–80 | +50 | 5 | ⚠️ Synthetic badge | v26 |
| W | **Latent-space autoencoder transport** ([§6.7](#67-latent-space-transmission-the-v26-endgame)) | 7 (data-distribution-matched) | ~150 | 100 % | 250 % | 256 | +80 | 3 (needs dataset) | ⚠️ Enhanced badge | v26 |
| X | **Custom-trained Real-ESRGAN on OSE footage** ([§6.6](#66-reverse-super-resolution--train-a-model-on-our-footage)) | 9 (best in class) | ~150 | 60 % | 280 % | 200–500 | +250 | 4 (needs dataset) | ⚠️ Enhanced badge | v26 |
| Y | **LaMa inpainting of stale tiles (CPU)** ([§5.1](#5-base-side-ai--where-the-real-horsepower-goes)) | 7 (no holes) | ~150 | 60 % | 380 % | 200–500 | +400 | 3 | ⚠️ Enhanced badge | v26 (CPU) |

¹ Persistent canvas raises *perceived* FPM well above the actual ~40 fpm tile-update rate because tiles fade in independently and old tiles remain on screen.
² Browser-side rendering offloads compositing/staleness/fade work; the base CPU column drops because the base now publishes raw canvas + metadata only.
³ Microframes / RIFE / optical-flow give *display-frame* smoothness; underlying *information* refresh stays at ~40 fpm.

### 11.3 Reading the table

- **Phase 1 (A → G):** Pure CPU, no AI, no Coral. Each step is incremental — every row strictly dominates the previous on at least one metric. This is the spine of the v25 image pipeline. A team that ships only A→G has succeeded.
- **Phase 2 (H → L):** AI on the base only. Each one is independent — pick the subset that fits the schedule. **J (independent safety detector) is the single most safety-relevant addition** and should never be cut for schedule.
- **Phase 3 (M → U):** Tractor-side AI plus the "outside-the-box" strategies. M and N are paired (the detector enables the P0 alert). The rest are adoption-on-merit.
- **v26 (V → Y):** Need data we don't have, or compute we don't have, or both. Park them.

### 11.4 What I'd cut if the schedule shrinks

In order of reluctance to cut:

1. First out: Y (LaMa CPU — too slow), V (semantic map — need data), W (autoencoder — need data).
2. Next out: L (RIFE CPU — marginal), X (custom SR — need data), O (procedural background — quality gain unclear).
3. Defend with both arms: J (safety detector), N (P0 person alert), G (browser tier — frees base for J).

---

## 12. Coral-only methods — what unlocks if the spike passes

Per [MASTER_PLAN.md §8.19](../DESIGN-CONTROLLER/MASTER_PLAN.md), the Coral Mini PCIe Accelerator (single Edge TPU, 4 TOPS INT8, ~$25) is in the BOM as **strongly recommended optional, unproven on the Max Carrier hardware**. A 2-day validation spike (PCIe enumeration, gasket/apex driver on the X8 Yocto kernel, thermal sustain, slot power) gates whether it stays. **None of these methods may be on the safety-critical path** — they all degrade to the CPU-only equivalent above if Coral is offline (`accel_select.py` flips the UI to "AI accelerator: offline" within 10 s).

Estimates assume the [Coral Edge TPU compiler](https://coral.ai/docs/edgetpu/compiler/) successfully maps the model. Models that don't compile cleanly fall back to CPU at the latencies in §11.2.

### 12.1 Metrics specific to Coral

| Metric | Meaning |
|---|---|
| **Coral % util** | Edge TPU duty cycle this stage consumes. >100 % means it can't keep up alone. Combine to <100 % aggregate when running stages back-to-back on one TPU. |
| **Coral latency** | Per-inference time on the Edge TPU once the model is loaded. |
| **Speedup vs CPU** | Wall-clock improvement over the CPU-only row in §11.2. |
| **Compiler risk** | Likelihood the model needs ops the Edge TPU compiler doesn't support and falls back to CPU partially. |

### 12.2 Coral-accelerated method ranking

Sorted by safety value × feasibility.

| # | Method | Resolution | Coral latency | Coral % util at 1 fps | Speedup vs CPU | Fidelity gain | Compiler risk | Feasibility | Replaces / augments |
|---|---|---|---:|---:|---:|---:|---|---:|---|
| Cα | **YOLOv8-nano safety cross-check** ([§5.5](#5-base-side-ai--where-the-real-horsepower-goes)) | 320×320 | 8–15 ms | 1.5 % | ~10× | det. recall +20 % | low | 9 | replaces J (CPU YOLOv8-nano) |
| Cβ | **YOLOv8-medium safety cross-check** | 640×640 | 25–40 ms | 4 % | ~25× | det. recall +35 % over Cα | low | 9 | upgrade of Cα — *use this if it compiles* |
| Cγ | **NanoDet-Plus base-side cross-check** (Apache-2.0 alternative if AGPL is unacceptable) | 416×416 | 6–12 ms | 1.2 % | ~12× | det. recall +18 % | low | 9 | replaces Cα for AGPL-free build |
| Cδ | **Real-ESRGAN-General-x4v3 super-res** ([§5.1](#5-base-side-ai--where-the-real-horsepower-goes)) | 96×64 → 384×256 | 15–30 ms | 3 % | ~10× over CPU H | fidelity 7 → 9 | medium (some ops fallback) | 8 | replaces H (CPU SR) |
| Cε | **RIFE-v4 frame interpolation** *(v1.1 LoRa §5.1)* | 384×256, 4× | 30–50 ms | 6 % | ~7× over CPU L | smooth 40 → 240 fpm | medium | 7 | replaces L (CPU RIFE — marginal); becomes practical |
| Cζ | **LaMa-Fourier inpainting of stale tiles** | 256×256 mask | 40–80 ms | 8 % | ~6× over CPU Y | no visible holes | high (FFT ops) | 6 | replaces Y (CPU LaMa — infeasible) |
| Cη | **MiDaS-small monocular depth (base-side, on canvas)** | 256×256 | 25–50 ms | 5 % | ~8× | adds depth-tinted overlay | medium | 6 | new — enables 2.5-D operator view |
| Cθ | **DPT-Hybrid panoptic segmentation** | 384×384 | 60–120 ms | 12 % | ~6× | per-pixel scene class | high | 4 | enables semantic-map mode (§V) on the *base* without tractor changes |
| Cι | **AnimateDiff-style temporal smoothing** | 256×256 | 80–150 ms | 15 % | n/a (no CPU baseline) | smooth motion priors | high | 3 | research; v26 |
| Cκ | **CLIP-tiny scene retrieval** ("does this look like the field at site B?") | 224×224 | 5–10 ms | 1 % | ~15× | enables auto-recall of cached background | low | 7 | feeds §I background cache |
| Cλ | **Custom OSE-trained super-res** ([§6.6](#66-reverse-super-resolution--train-a-model-on-our-footage), Coral-compiled) | 96×64 → 384×256 | 18–35 ms | 4 % | ~10× | fidelity 7 → 9.5 | medium | 4 (needs dataset) | upgrade of Cδ; v26 |
| Cμ | **Latent-codec decoder** ([§6.7](#67-latent-space-transmission-the-v26-endgame), Coral-compiled) | latent → 384×256 | 10–20 ms | 2 % | ~12× over CPU W | fidelity 7 (matches data) | medium | 3 (needs dataset) | replaces W; v26 |

### 12.3 Coral budget — what fits on a single TPU

A single Coral Edge TPU has 4 TOPS INT8 and ~8 MB on-chip SRAM. Models hot-swap from host RAM but that costs ~10–30 ms per swap. Aim to keep the running set ≤ ~15 MB total compiled-model size to avoid swap thrash. Suggested simultaneous loadout for the v25 base, in priority order:

1. **Cβ** YOLOv8-medium safety cross-check (~6 MB)
2. **Cδ** Real-ESRGAN-General-x4v3 (~4 MB)
3. **Cε** RIFE-v4 (~3 MB) *if RIFE ships*

Aggregate ~13 MB compiled; aggregate Coral utilisation at 1 fps cross-check + 1 fps SR + 4 fps RIFE ≈ 4 % + 3 % + 24 % = ~31 %, leaves plenty of headroom for occasional Cζ inpainting bursts when the operator opts in. Cη / Cθ / Cκ run on demand only, never both at once.

### 12.4 Coral *not* used on the tractor — explicit decision

The tractor X8 stays Coral-free in v25 even if the base spike passes:

- **Power:** the Edge TPU draws ~2 W under load. Tractor X8 + Murata WiFi already run hot inside the sealed enclosure; another 2 W of localised heat near a thermally-sensitive package is the wrong direction.
- **Real-time risk:** the on-tractor inference path is in the safety loop (NanoDet-Plus → ROI bias → bytes on the air → operator awareness). A Coral failure mid-operation that we *did* depend on is a worse failure mode than running NanoDet-Plus at the slower CPU rate from the start.
- **Carrier-form-factor uncertainty:** the same "unproven on Max Carrier" caveat from the base applies, and proving it twice (once per tractor) doubles the validation cost.

NanoDet-Plus on the tractor stays CPU-only INT8 at the §11.2 row M numbers. Revisit in v26 if the base Coral has been stable in the field for a season.

### 12.5 Reading the Coral table

- **Cα/Cβ/Cγ are the safety-detector trio.** Pick exactly one (Cβ if it compiles; Cα as fallback; Cγ if AGPL is forbidden). One of them must always be running.
- **Cδ is the visual-fidelity headliner.** Single biggest perceived-quality win on the operator screen.
- **Cε turns L from "marginal" to "ships."** RIFE on CPU is a coin-flip; on Coral it's textbook.
- **Cζ enables Y.** LaMa CPU is infeasible per §11.4; Coral makes it real.
- **Cκ is the cheap multiplier** — combine with §I background cache and you get a system that *recognises* it has seen this scene before and uses the right cached background. ~$0 in compute, high UX value.

---

## 13. Combination compatibility — what stacks with what

§11 and §12 list schemes individually. In practice the v25 image pipeline is a **stack** — multiple stages running concurrently on the same frame. Not every pair plays nicely. This section maps the legal combinations and recommends concrete stacks.

### 13.1 Compatibility classes

Every scheme in §11 / §12 falls into one of seven roles. **Within a role, schemes are mutually exclusive (pick one). Across roles, schemes generally compose.**

| Role | What it does | Picks-one-from set |
|---|---|---|
| **R1 — Wire encoding** | How pixel data crosses the LoRa link | A whole-frame, B–G tile-delta, P wireframe, Q optical-flow, V semantic-map, W latent codec |
| **R2 — Tractor pre-process** | Runs before the encoder | (none), C image-registration, M NanoDet-Plus, O procedural-bg classifier |
| **R3 — Tractor ROI policy** | How the byte budget is sliced across tiles | (uniform), §3.1 valve-state, D ROI rings, M detection-biased, F operator-cued |
| **R4 — Base reconstruction** | Builds the displayed canvas from received fragments | E persistent canvas, I background cache, K motion-vector overlay |
| **R5 — Base AI enhancement** | Improves the canvas after reconstruction | (none), H/Cδ super-res, L/Cε RIFE, Y/Cζ inpainting, X/Cλ custom-trained SR |
| **R6 — Base AI safety** | Independent cross-check on the displayed canvas | (none), J YOLOv8-nano CPU, Cα/Cβ Coral YOLO, Cγ NanoDet-Plus |
| **R7 — Sidecar channels** | Non-image bytes that travel alongside | N CMD_PERSON_APPEARED, R audio events, S three-cam mux, T satellite, U event-skip, Cη depth, Cθ panoptic, Cκ scene retrieval |

**Rule of thumb:** Pick at most one from each of R1–R6. Stack as many R7 sidecars as fit the byte budget. Every Phase-1/2/3 build is one column from each role.

### 13.2 Pairwise compatibility matrix (representative pairs)

✅ stacks fine. ⚠️ technically possible, watch for the listed conflict. ❌ mutually exclusive (same role) or actively breaks.

| | B Tile-delta | C Registration | D ROI rings | E Canvas | F Op-cued ROI | G Browser tier | H/Cδ Super-res | I BG cache | J/Cα/Cβ Safety det. | K Motion-vec | L/Cε RIFE | M Tractor det. | N Person alert | O Procedural BG | P Wireframe | Q Optical-flow | R Audio | S 3-cam mux | T Satellite | U Event-skip | V Sem-map | W Latent | Y/Cζ Inpaint |
|---|:-:|:-:|:-:|:-:|:-:|:-:|:-:|:-:|:-:|:-:|:-:|:-:|:-:|:-:|:-:|:-:|:-:|:-:|:-:|:-:|:-:|:-:|:-:|
| **B Tile-delta**       | — | ✅ | ✅ | ✅ | ✅ | ✅ | ✅ | ✅ | ✅ | ✅ | ✅ | ✅ | ✅ | ✅ | ❌ | ❌ | ✅ | ✅ | ✅ | ✅ | ❌ | ❌ | ✅ |
| **C Registration**     |   | — | ✅ | ✅ | ✅ | ✅ | ✅ | ✅ | ✅ | ⚠️¹ | ✅ | ✅ | ✅ | ✅ | ✅ | ⚠️¹ | ✅ | ✅ | ✅ | ✅ | ✅ | ✅ | ✅ |
| **D ROI rings**        |   |   | — | ✅ | ✅² | ✅ | ✅ | ✅ | ✅ | ✅ | ✅ | ✅³ | ✅ | ✅⁴ | ❌ | ❌ | ✅ | ✅ | ✅ | ✅ | ❌ | ❌ | ✅ |
| **E Canvas**           |   |   |   | — | ✅ | ✅ | ✅ | ✅ | ✅ | ✅ | ✅ | ✅ | ✅ | ✅ | ⚠️⁵ | ⚠️⁵ | ✅ | ✅ | ✅ | ✅ | ⚠️⁵ | ✅ | ✅ |
| **F Op-cued ROI**      |   |   |   |   | — | ✅ | ✅ | ✅ | ✅ | ✅ | ✅ | ✅⁶ | ✅ | ✅ | ✅ | ✅ | ✅ | ✅⁷ | ✅ | ⚠️⁸ | ✅ | ✅ | ✅ |
| **G Browser tier**     |   |   |   |   |   | — | ✅ | ✅ | ✅ | ✅ | ✅ | ✅ | ✅ | ✅ | ✅ | ✅ | ✅ | ✅ | ✅ | ✅ | ✅ | ✅ | ✅ |
| **H/Cδ Super-res**     |   |   |   |   |   |   | — | ✅ | ✅ | ✅ | ✅⁹ | ✅ | ✅ | ✅ | ❌¹⁰ | ❌¹⁰ | ✅ | ✅ | ✅ | ✅ | ❌¹⁰ | ⚠️¹¹ | ⚠️¹² |
| **I BG cache**         |   |   |   |   |   |   |   | — | ✅ | ✅ | ✅ | ✅¹³ | ✅ | ⚠️¹⁴ | ❌¹⁰ | ❌¹⁰ | ✅ | ✅ | ✅ | ✅ | ❌¹⁰ | ❌ | ⚠️¹⁵ |
| **J/Cα/Cβ Safety det.**|   |   |   |   |   |   |   |   | — (pick one) | ✅ | ✅ | ✅¹⁶ | ✅ | ⚠️¹⁷ | ❌¹⁸ | ❌¹⁸ | ✅ | ✅ | ✅ | ✅ | ❌¹⁸ | ⚠️¹⁹ | ⚠️²⁰ |
| **K Motion-vec µframes**|  |   |   |   |   |   |   |   |   | — | ⚠️²¹ | ✅ | ✅ | ✅ | ⚠️²² | ❌²³ | ✅ | ✅ | ✅ | ✅ | ⚠️²² | ✅ | ✅ |
| **L/Cε RIFE**          |   |   |   |   |   |   |   |   |   |   | — | ✅ | ✅ | ✅ | ❌¹⁰ | ❌¹⁰ | ✅ | ✅ | ✅ | ✅ | ❌¹⁰ | ✅ | ✅ |
| **M Tractor det.**     |   |   |   |   |   |   |   |   |   |   |   | — | ✅²⁴ | ✅²⁵ | ✅ | ✅ | ✅ | ✅ | ✅ | ✅²⁶ | ✅²⁷ | ✅ | ✅ |
| **N Person alert (P0)**|   |   |   |   |   |   |   |   |   |   |   |   | — | ✅ | ✅ | ✅ | ✅ | ✅ | ✅ | ✅²⁸ | ✅ | ✅ | ✅ |
| **O Procedural BG**    |   |   |   |   |   |   |   |   |   |   |   |   |   | — | ❌¹⁰ | ❌¹⁰ | ✅ | ✅ | ⚠️²⁹ | ✅ | ❌¹⁰ | ❌ | ⚠️³⁰ |
| **P Wireframe**        |   |   |   |   |   |   |   |   |   |   |   |   |   |   | — | ❌³¹ | ✅ | ✅ | ⚠️³² | ✅ | ❌³¹ | ❌³¹ | ❌¹⁰ |
| **Q Optical-flow**     |   |   |   |   |   |   |   |   |   |   |   |   |   |   |   | — | ✅ | ✅ | ⚠️³² | ✅ | ❌³¹ | ❌³¹ | ❌¹⁰ |
| **R Audio sidecar**    |   |   |   |   |   |   |   |   |   |   |   |   |   |   |   |   | — | ✅ | ✅ | ✅ | ✅ | ✅ | ✅ |
| **S 3-cam mux**        |   |   |   |   |   |   |   |   |   |   |   |   |   |   |   |   |   | — | ✅ | ✅ | ✅ | ✅ | ✅ |
| **T Satellite far-field**|  |   |   |   |   |   |   |   |   |   |   |   |   |   |   |   |   |   | — | ✅ | ✅³³ | ✅ | ✅ |
| **U Event-skip**       |   |   |   |   |   |   |   |   |   |   |   |   |   |   |   |   |   |   |   | — | ✅ | ✅ | ✅ |
| **V Semantic map**     |   |   |   |   |   |   |   |   |   |   |   |   |   |   |   |   |   |   |   |   | — | ❌³¹ | ❌¹⁰ |
| **W Latent codec**     |   |   |   |   |   |   |   |   |   |   |   |   |   |   |   |   |   |   |   |   |   | — | ❌¹⁰ |
| **Y/Cζ Inpaint**       |   |   |   |   |   |   |   |   |   |   |   |   |   |   |   |   |   |   |   |   |   |   | — |

**Footnotes:**

¹ Optical-flow / motion-vector microframes need raw frame-to-frame pixels for flow estimation — registration must run *after* flow capture, not before, or the flow vectors are zeroed out. Order matters.
² ROI rings *contain* the operator-cued ROI: the clicked tile becomes Ring 0 for the cue duration, surrounding tiles Ring 1.
³ Detection-biased ROI augments rings — a detected person promotes its tile to Ring 0 even if it was Ring 2 by geometry.
⁴ Procedural background only fills tiles classified as Ring 2 by both geometry *and* the segmenter — never Ring 0/1.
⁵ Wireframe / optical-flow / semantic-map are *whole-frame* encodings; they bypass the tile canvas. The canvas freezes during these modes and resumes when normal encoding returns.
⁶ Operator click on a detected object explicitly confirms the detection — log the agreement to the §8.10 black-box logger as a positive training signal.
⁷ Operator click on a non-active camera implicitly switches the multiplexer to that camera. Single gesture, two effects.
⁸ Event-skip suppresses transmission only when the operator is *also* idle; an explicit operator click cancels the skip immediately.
⁹ Order: super-res *before* RIFE. RIFE on already-upscaled frames is sharper; RIFE on raw thumbnails wastes the SR budget.
¹⁰ R1 wire-encoding conflict: degraded modes (P/Q/V) and W replace tile-delta entirely. Base AI stages H/I/L/Y/Cδ/Cε/Cζ assume a tile-canvas input and have nothing to operate on.
¹¹ Latent codec produces a fixed-resolution decoder output — extra super-res helps if you trained the codec at 192×128 and want to display at 384×256, otherwise it's redundant.
¹² Inpainting fills *stale* tiles; super-res then runs on the filled canvas. Fine, but mark inpainted-then-upscaled tiles with **both** "Cached" and "Enhanced" badges.
¹³ Tractor segmenter feeds the base cache the foreground/background classification — the cache only refreshes from `background` tiles, never patches over a `foreground` tile.
¹⁴ Procedural background and cached background overlap. Pick one per session: cache if the operator has been on this site before, procedural if first visit.
¹⁵ Inpainting fights cache: cache fills with *real previous pixels* (more honest), inpainting *generates* (less honest). Prefer cache when both are eligible.
¹⁶ Two-detector safety pattern (the canonical recommendation): tractor M + base J/Cα/Cβ. Disagreements logged for v26 retraining per [§5.5](#5-base-side-ai--where-the-real-horsepower-goes).
¹⁷ Safety detector should run on the **real** received tiles, not on procedurally-substituted ones. Run detector before O substitution overlays the canvas.
¹⁸ Whole-frame degraded modes don't carry a real canvas — the safety detector has nothing to verify against. Tractor-side detection M still runs and the sidecar still fires.
¹⁹ Latent codec output is post-decoder-hallucinated — running a safety detector on it can hallucinate detections too. Run safety detector on the *next* normal-encoded frame, not on latent-decoded output.
²⁰ Same as ¹². Detector on inpainted tiles is fine but tag the detection as "low-confidence (synthetic source)" in the UI.
²¹ Optical-flow microframes and registration both estimate motion. If both are running, share the estimate (one global computation, two consumers) — don't pay twice.
²² K motion-vector overlay needs a base canvas to push pixels around on. P/V both flush the canvas — no canvas, no overlay. Restart microframes when the canvas comes back.
²³ K and Q are the same idea at different protocol layers (K is the "I'm still alive" overlay during normal operation; Q is the *only* visual when the link collapses). Run one or the other, not both.
²⁴ N is a strict subset of M — the detector fires the alert. Cannot have N without M.
²⁵ Tractor detector classifies foreground vs background → enables procedural substitution to know what's safe to replace.
²⁶ Detector confidence drives the event-skip threshold — high-confidence person in frame *blocks* event-skip even if motion is low.
²⁷ Same detector backbone produces both bounding boxes (M/N sidecars) and the coarse class grid for V; one inference, two outputs.
²⁸ Person alert is P0 and overrides event-skip immediately.
²⁹ Satellite far-field renders the horizon; procedural background renders the immediate ground plane. They tile vertically — satellite above the horizon line, procedural below. Compatible if both are tagged.
³⁰ Inpainting + procedural-background both *generate* pixels. Pick one rule for each tile, never blend (operator can't tell what they're looking at).
³¹ Same role (R1 wire encoding). Pick one per session, switch dynamically per [v1.1 LoRa §5.3](2026-04-27_LoRa_InDepth_Analysis_ClaudeOpus4_7_v1_1.md).
³² Wireframe / optical-flow modes don't carry world coordinates; satellite far-field still renders correctly underneath because it's georeferenced to the GPS, not the camera frame.
³³ Semantic-map renders a top-down stylised view; satellite is *also* a top-down view. They can co-render — satellite as the basemap, semantic classes as colour-coded overlay. Genuinely synergistic.

### 13.3 Recommended canonical stacks

Pick the row that matches your build target. Each stack is "one R1 + one R2 + one R3 + one R4 + one R5 + one R6 + N R7s."

| Stack name | Target | R1 wire | R2 tractor pre | R3 ROI policy | R4 base recon | R5 base enhance | R6 safety | R7 sidecars | Coral required |
|---|---|---|---|---|---|---|---|---|---|
| **Stack-MVP** *(Phase 1, ships alone)* | First field test | B tile-delta | C registration | D rings + §3.1 valve | E canvas | (none) | (none) | U event-skip | No |
| **Stack-Base-AI** *(Phase 2, CPU-only)* | First field test with AI | B tile-delta | C registration | D rings + §3.1 valve | E canvas + I bg-cache | H super-res CPU | J YOLOv8-nano CPU | R audio, S 3-cam, U event-skip | No |
| **Stack-Coral** *(Phase 2, Coral spike passed)* | Recommended v25 production | B tile-delta | C registration | D rings + §3.1 valve | E canvas + I bg-cache + K µframes | Cδ super-res + Cε RIFE | Cβ YOLOv8-medium | Cκ scene retrieval, R audio, S 3-cam, T satellite, U event-skip | Yes |
| **Stack-Tractor-AI** *(Phase 3, Coral on base)* | The full v25 vision | B tile-delta | C registration + M NanoDet-Plus | D rings + §3.1 valve + M detection bias + F op-cued | E canvas + I bg-cache + K µframes | Cδ super-res + Cε RIFE | Cβ YOLOv8-medium | N person-alert, Cκ scene retrieval, R audio, S 3-cam, T satellite, U event-skip | Yes |
| **Stack-Degraded-Drift** *(auto-fallback)* | Link <300 B/refresh | Q optical-flow | (none) | (n/a) | (canvas frozen, K active) | (none) | M tractor still runs | N person-alert, U event-skip | No |
| **Stack-Degraded-Wireframe** *(auto-fallback)* | Link <150 B/refresh | P wireframe | (none) | (n/a) | (canvas frozen) | (none) | M tractor still runs | N person-alert, R audio | No |
| **Stack-v26-Latent** *(research)* | Bandwidth-starved future | W latent codec | M NanoDet-Plus | M detection bias | E canvas | Cλ custom SR + Cζ inpaint | Cβ YOLOv8-medium | full sidecar set | Yes |
| **Stack-v26-Semantic** *(research)* | Total degradation, semantic only | V semantic-map | M NanoDet-Plus | (n/a) | (browser renders) | Cθ panoptic enrichment | (sem-map *is* the safety view) | T satellite, R audio | Yes |

### 13.4 Auto-fallback ladder

The pipeline must shift between stacks **automatically** based on link health (per [v1.1 LoRa §5.3](2026-04-27_LoRa_InDepth_Analysis_ClaudeOpus4_7_v1_1.md)). Recommended ladder, top is best link:

```
link bytes/refresh available    →    stack
─────────────────────────────────────────────────────────
≥ 800 B                         →    Stack-Tractor-AI (full)
500–800 B                       →    Stack-Coral (drop M detection bias)
300–500 B                       →    Stack-Base-AI (Coral idle, CPU only)
150–300 B                       →    Stack-Degraded-Drift (Q + K)
< 150 B                         →    Stack-Degraded-Wireframe (P)
0 B (control-only)              →    canvas frozen, age clock + R audio + N alerts only
```

The transition is hysteretic — climb up one stack only after 5 consecutive refreshes at the higher band, drop down immediately on starvation. This prevents flapping between Stack-Coral and Stack-Base-AI when the link is right at the boundary.

### 13.5 Three things you should *never* combine

1. **Procedural background O + inpainting Y/Cζ on the same tile.** Both generate pixels by different rules; the operator can't tell which one made the patch. Pick one per tile.
2. **Two whole-frame encodings P + V (or P + Q, or V + W) in the same refresh.** They share R1; only one wire encoding fits. Switch per refresh, not within.
3. **Latent codec W + safety detector running on the W output.** Hallucinated input + AI verifier = compounded hallucination. If W ships in v26, the safety detector runs on the *previous* tile-delta canvas, not the latent reconstruction.

---

## 14. Top recommendation — no-Coral system

If the Coral validation spike per [MASTER_PLAN.md §8.19](../DESIGN-CONTROLLER/MASTER_PLAN.md) fails, OSE chooses to skip the accelerator, or the BOM is locked before the spike completes, this is the recommended v25 image-pipeline stack. It is a **slight expansion of Stack-Base-AI** from §13.3 with the tractor-side detector folded in, because without Coral the base lacks an independent strong detector and the tractor-side detector becomes more important to the safety story.

### 14.1 The stack — "Stack-NoCoral-Recommended"

| Role | Pick | Why this one |
|---|---|---|
| **R1 wire encoding** | **B tile-delta I/P frames** | Only encoding that hits the byte budget honestly while preserving real pixels. P/V/W are degraded modes, not steady-state. |
| **R2 tractor pre-process** | **C image registration + M NanoDet-Plus** | Registration defends the byte savings during driving (§10.2). NanoDet-Plus replaces the strong base detector that Coral would have provided — it must run on the tractor instead. ~50 ms p99 on the X8 A53s, well within budget. Apache-2.0 license, no AGPL exposure. |
| **R3 ROI policy** | **D ROI rings (3-level) + §3.1 valve-state + M detection bias + F operator-cued** | All four compose. Valve-state gives baseline rings; detector promotes person/animal tiles to Ring 0; operator click overrides for the cued duration. |
| **R4 base reconstruction** | **E persistent canvas + I background cache** | Canvas is mandatory. Background cache fills missed tiles with real previous pixels (Cached badge with age) — the only way to get hole-free display without an AI inpainter. Skip K motion-vector microframes for now (Phase 2 stretch — needs the canvas + fade infrastructure stable first). |
| **R5 base AI enhancement** | **(none in Phase 1) → H Real-ESRGAN-General-x4v3 CPU in Phase 2** | The single biggest perceived-quality win on the operator screen. ~280 % of one A53 core (≈70 % of total CPU) on the base — sustainable because base has wall power and no real-time loop. Latency cost +250 ms is acceptable on the display path. **Defer L/RIFE entirely** — without Coral it's marginal (Cε feasibility 5 in §11.2), the operator already gets smooth perceived motion from the per-tile fade shader, and the CPU budget doesn't have room for both H and L. |
| **R6 safety detector** | **J YOLOv8-nano CPU** *(or NanoDet-Plus CPU if AGPL is unacceptable)* | Independent of M is the whole point — different architecture, different failure modes, runs on the reconstructed canvas. ~150 ms / ~50 % of one A53 core at 1 fps. Disagreements with M get logged to §8.10 black-box logger for v26 retraining per [§5.5](#5-base-side-ai--where-the-real-horsepower-goes). |
| **R7 sidecars** | **N CMD_PERSON_APPEARED (P0) + R audio (YAMNet) + S three-cam mux + U event-skip + T satellite far-field** | All cheap, all compose, all ✅ in §13.2. N is the highest-value safety addition the system can have at any cost tier. T costs zero LoRa bytes. Skip O procedural background — without I background cache as a peer it's strictly worse than cache; with cache it's redundant. |

**Coral required:** No. **Spike required:** No. **AGPL exposure:** Only J YOLOv8-nano on the base (substitutable for NanoDet-Plus; M on the tractor is already Apache-2.0).

### 14.2 What the operator actually sees

Steady-state, healthy link:

- A 384×256 canvas per camera, updated tile-by-tile with cross-fade (≈40 fpm tile updates, perceived as ~120 fpm because of fade + persistence).
- Centre tiles + bucket tiles + any detected person/animal at WebP q60; surrounding tiles at q40; horizon at q15 (or skipped during quiet periods).
- Each tile carries a staleness clock; tiles older than ~5 s show a yellow tint with age.
- Real-ESRGAN-x4v3 upscales the canvas to display-native resolution every ~1.5 s. **"Enhanced" badge** visible.
- Independent YOLOv8-nano on the base draws bounding boxes from a second pass; tractor's NanoDet-Plus draws from the sidecar. Disagreements are colour-coded.
- Three-camera attention mux defaults to front=70/bucket=25/rear=5; a person detected on rear flips priority within one refresh.
- Far-field beyond ~30 m is rendered from a pre-loaded satellite tile, georeferenced to live GPS.
- Audio events ("hydraulic squeal," "engine knock") flash a banner on the UI within ~50 ms of detection.

Degraded link (300–500 B/refresh):

- Canvas still updates, just slower. Stale tiles back-fill from background cache (Cached badge, age in seconds).
- Real-ESRGAN keeps running on whatever canvas exists. Operator sees soft pixels marked Enhanced.
- N person alert still fires at P0 — never delayed by image starvation.

Severely degraded link (<300 B/refresh):

- Auto-fallback to Stack-Degraded-Drift (Q optical-flow) per §13.4. Canvas freezes; flow vectors push pixels around with **"Predicted" badge**.
- Below 150 B → Stack-Degraded-Wireframe (P). Tron-style line edges with **"Wireframe" badge**.
- N person alert still fires.

### 14.3 Estimated metrics for the recommended stack

Drawing from §11.2 columns, summed across stages:

| Metric | Value | Notes |
|---|---|---|
| **Fidelity (steady-state)** | ~7 (centre 8 with SR) | Operator sees sharp centre, soft periphery, stale-tinted unchanged regions. |
| **Perceived FPM** | ~150 | Tile updates ~40, but fade + persistence + canvas continuity make it feel like ~150. |
| **Tractor CPU** | ≈140 % of one core (~35 % of total) | C registration ~5 %, M NanoDet-Plus 1 fps ~80 %, encode + fragment ~50 %. Two cores idle for headroom. |
| **Base CPU** | ≈350 % of one core (~88 % of total) | H super-res ~280 %, J safety detector ~50 %, canvas + cache + sidecars ~20 %. **Tight; this is the bottleneck.** |
| **Bytes / refresh** | 200–500 + sidecars | Tile-delta with rings + registration brings the average down hard. Sidecars (N, R, S) add <50 B. |
| **Glass-to-glass latency** | ~850 ms p50, ~1300 ms p99 | LoRa baseline ~700 ms + tile-delta +25 + canvas +50 + super-res +250. p99 dominated by fragment loss recovery. |
| **Person-alert (N) latency** | ~250 ms p99 | Tractor detector 80 ms + fragment 25 + LoRa 100 + base render 50 = well under the 250 ms gate from [TODO Phase 5D](../DESIGN-CONTROLLER/TODO.md). |
| **Safety honesty** | ✅ | Every non-1:1 pixel badged; staleness always visible; raw-mode toggle one click. |
| **Hardware delta vs. Stack-MVP** | $0 | No new hardware required. |

The base CPU at ~88 % is the only real worry. Two mitigations:

1. **Drop Real-ESRGAN to 0.5 fps** (super-res every other refresh, fade between). Drops base CPU to ~50 %.
2. **Move per-tile fade and staleness rendering to the browser tier (G)** — already recommended in §10.1. Drops base CPU another ~10 %.

With both mitigations the base sits at ~40 % CPU with comfortable headroom for the safety detector to run reliably.

### 14.4 Build order — the no-Coral path

In implementation order, this is what to actually do:

1. **Week 1–2:** Tile-delta protocol on the wire (tractor encode, base reassemble). [`encode_tile_delta.py`](../DESIGN-CONTROLLER/TRACTOR_NODE.md), [`reassemble.py`](../DESIGN-CONTROLLER/BASE_STATION.md). Validate against [TODO Phase 5D](../DESIGN-CONTROLLER/TODO.md) zero-P0-starvation gate.
2. **Week 2:** Image registration in [`tile_diff.py`](../DESIGN-CONTROLLER/TRACTOR_NODE.md). One commit, ~50 lines of NEON-accelerated phase correlation.
3. **Week 2–3:** ROI rings + valve-state policy in [`roi.py`](../DESIGN-CONTROLLER/TRACTOR_NODE.md).
4. **Week 3–4:** Persistent canvas + per-tile fade GLSL shader (browser-side) + staleness clock + Cached badge. [`canvas.py`](../DESIGN-CONTROLLER/BASE_STATION.md) + browser bundle.
5. **Week 4:** Operator-cued ROI (CMD_ROI_HINT, opcode `0x61`) round-trip. Bench gate.
6. **Week 4–5:** Background cache in [`canvas.py`](../DESIGN-CONTROLLER/BASE_STATION.md) — rolling median per tile keyed on the segmenter class. Activates once M is online (Week 7).
7. **Week 5:** Three-camera attention mux. Pure scheduling logic.
8. **Week 5–6:** Real-ESRGAN-General-x4v3 ncnn deploy on the base. Benchmark gate ≤300 ms/frame at 0.5–1 fps. UI Enhanced badge.
9. **Week 6:** YOLOv8-nano CPU safety detector on the base, [`detect_yolo.py`](../DESIGN-CONTROLLER/BASE_STATION.md). Decide AGPL stance now or substitute NanoDet-Plus.
10. **Week 6–7:** YAMNet audio sidecar (R), event-skip (U), satellite far-field (T). Three small services in parallel.
11. **Week 7–9:** NanoDet-Plus on the tractor X8, `detect_nanodet.py`. Benchmark gate ≤50 ms p99. Wire it into ROI bias and the detection sidecar (topic `0x26`).
12. **Week 9:** CMD_PERSON_APPEARED (opcode `0x60`) end-to-end at P0. Walk-the-person gate from [TODO Phase 5D](../DESIGN-CONTROLLER/TODO.md): ≤250 ms p99.
13. **Week 9–10:** Auto-fallback ladder (§13.4) implementation. Test by attenuating the LoRa link and watching the stack downshift.
14. **Continuous from Week 1:** Black-box logging of every canvas, detection, sidecar, and operator action per [MASTER_PLAN.md §8.10](../DESIGN-CONTROLLER/MASTER_PLAN.md). Builds the v26 training set and the v26 Coral-spike retry justification.

Total: ~10 engineer-weeks for the full no-Coral stack. Stack-MVP alone (steps 1–5) is ~4 weeks and ships first-field-test value on its own.

### 14.5 What I'd cut if the schedule shrinks further

In order:

1. **First out: T satellite far-field, R audio sidecar.** Pure conveniences. Keep the hooks in the protocol; defer the implementations to v25.5.
2. **Next: H Real-ESRGAN super-res.** Operator loses the Enhanced view but keeps the honest-pixel canvas. Big perceived-quality drop, but the safety story is intact.
3. **Next: M tractor NanoDet-Plus + N person alert.** This is a real safety regression — only cut if the schedule absolutely demands it, and only if J base detector ships in compensation.
4. **Defend with both arms: B tile-delta, E canvas + I cache, J base safety detector, browser tier G.** These are the spine. Anything that ships without all four is not credibly safer than v1.0 baseline.

### 14.6 Why not just ship Stack-Coral and hope

Tempting, especially with the Coral at $25. The reasons not to:

- **Coral on the Max Carrier mini-PCIe is documented zero times anywhere I can find.** The first team to make it work spends 2 unbudgeted days; the second team copies that work. v25 is the first team. Failure is plausible.
- **Even if the spike passes on the bench**, thermal sustain inside the base enclosure under summer ambient is a separate question the spike doesn't answer.
- **The safety story cannot depend on Coral.** Per [MASTER_PLAN.md §8.19](../DESIGN-CONTROLLER/MASTER_PLAN.md). So the no-Coral stack has to exist and be tested anyway. **Building it as the primary, with Coral as a strict upgrade, is cheaper than building both.**
- **OSE may decide AGPL on YOLOv8 is unacceptable.** Substituting NanoDet-Plus is trivial in the no-Coral stack (same Apache-2.0 family the tractor already uses); harder if Coral compilation steered the model choice toward Ultralytics-only ops.
- **"Looks good enough on day one" matters.** Stack-NoCoral-Recommended at fidelity ~7 is *visibly* better than the v1.0 LoRa-doc baseline at fidelity ~3, with no new hardware risk and no new licensing exposure.

If Coral later proves itself in v25.5 or v26, the upgrade is **strictly additive** — drop in Cβ to replace J, drop in Cδ to replace H, drop in Cε for new functionality. No protocol changes, no UI changes beyond flipping the existing "AI accelerator: offline" badge to "online."

### 14.7 Final answer

**Build Stack-NoCoral-Recommended (§14.1).** Prioritize browser-tier offload (G from §10.1, fully analysed in [§15](#15-browser-tier-offload--pros-cons-mobile-compatibility-automation-impact)) so the base CPU stays comfortable. Ship Stack-MVP first (4 weeks), then layer on safety detector, then tractor-side detector + person alert, then super-res. Log everything from day one for the v26 retraining set. Treat Coral as a strict future upgrade, not a v25 dependency.

---

## 15. Browser-tier offload — pros, cons, mobile compatibility, automation impact

§10.1 introduced the operator browser as a third compute tier. §14 leans on it heavily to keep the base X8 CPU below 50 %. This section is the full analysis of *why* we lean on it, *where it fails*, and *what rules keep it from blocking future autonomy*.

### 15.1 Pros — what browser-tier offload buys

- **Free hardware.** A 2022 mid-range phone has ~1 TFLOPS on its Mali/Adreno GPU plus a 5–15 TOPS NPU you'd otherwise have to buy as a Coral. A modern laptop is 10–100× the FLOPS of the i.MX 8M Mini. The base X8 sustains ~4 GFLOPS on CPU. The asymmetry is enormous.
- **Frees the base CPU for safety-critical work.** Per [§14.3](#143-estimated-metrics-for-the-recommended-stack), the base sits at ~88 % CPU under the recommended Stack-NoCoral load. Pushing fade / staleness rendering / compositing into the browser drops the base to ~40 %. That headroom is what keeps the safety detector from being preempted under stress.
- **Per-operator scaling is automatic.** Two people watching the feed on two devices = 2× rendering compute, free. The base doesn't multiply its load — only the canvas + metadata stream is duplicated, which is bandwidth-bound on LAN, not CPU-bound.
- **Display polish wants to live where the display is.** Per-tile cross-fade at 60 fps requires the renderer to be at the display. Doing it on the base means streaming pre-faded frames at 60 fps over LAN — wasteful. Doing it in the browser means streaming the canvas update once per ~1.5 s refresh and animating locally.
- **Hot-swappable UI iteration.** Browser code redeploys instantly on page reload; base firmware needs an OTA + service restart. UI polish iterates in seconds instead of minutes.

### 15.2 Cons — what browser-tier offload costs

- **Device-quality variance.** A 2018 budget Android tablet has roughly 1/20 the GPU of a 2024 iPad. The same UI must degrade gracefully across that range. Real engineering cost: feature-detection paths, fallbacks, two or three quality tiers.
- **Browser-API fragmentation.** WebGL 2 is universal. WebGPU is patchy on iOS Safari < 17 and on older Android Chrome. WebNN is essentially Chromium-only. *Anything* added to the browser path must be tested on at least Chrome / Safari / Firefox × {desktop, Android, iOS}.
- **Trust boundary moves.** Code running in the browser is inspectable and modifiable by the operator. **The safety detector and the "Enhanced / Synthetic / Cached / Wireframe / Predicted" badge *decisions* must stay on the base.** A malicious or curious operator who patches their own JS to hide the staleness clock has just disabled a safety mechanism. **The browser renders the badges; it does not decide them.** The base attaches the badge as a property of the published canvas tile; the browser must display it; refusal-to-display should be detectable and logged.
- **Cold-start latency.** First page load on a phone is 1–3 s of WebAssembly compile + texture upload before anything paints. The base could've been streaming JPEGs in 200 ms. For "I just opened the operator console after rebooting my phone," that gap matters and is visible to the operator.
- **Battery on the operator device.** Continuous WebGL + WebSocket + decode is ~2–4 W on a phone. The control tablet will run hot and lose ~20 %/hr of battery. Plan for either AC power or two-tablet rotation in the field.
- **Offline / standalone fallback is harder.** If the base ever needs to drive a directly-attached display (HDMI on the Max Carrier) for a pit-of-despair fallback console, the browser-tier code doesn't help — the base needs the rendering path to exist. Mitigation: keep a minimal server-side render path alive even when nobody is using it.

### 15.3 Will it work on a cell-phone browser?

**Yes, for the recommended scope, with a clear capability floor.** Concrete numbers as of April 2026 (sources: [caniuse.com](https://caniuse.com/), MDN browser-compat tables):

| Capability | Safe on phones today? | Coverage | Use in v25? |
|---|---|---|---|
| WebSocket binary frames | ✅ universal | ~100 % | Yes — transport. |
| HTML5 Canvas 2D + ImageBitmap | ✅ universal | ~100 % | Yes — tile patching. |
| WebGL 2 fragment shaders | ✅ near-universal | ~99 % (iOS 15+, Android Chrome 80+) | **Yes — per-tile fade, staleness tint, vector overlays.** This is the floor. |
| ResizeObserver / OffscreenCanvas | ✅ | ~95 % | Yes — render off-thread. |
| WebCodecs (HW H.264 decode) | ⚠️ Chromium-everywhere; Safari iOS 16+ | ~85 % | Not needed — we don't ship video. |
| WebGPU (compute shaders, WGSL) | ⚠️ patchy | ~75 % (iOS 18+, Android Chrome 113+) | **Opportunistic only** — feature-detect, fall back to WebGL. |
| WebNN (browser-side AI inference) | ❌ flag-gated, Chromium-only | <30 % | **Defer to v26+.** |
| WebUSB / WebSerial | ❌ no on iOS | ~50 % desktop only | Not needed — base owns the radio. |

**Recommendation:** Target **WebGL 2 + Canvas 2D as the floor** for the operator UI. Anything we put there will work on every phone shipped since ~2019.

**Specific test matrix to gate browser releases:**

1. Latest Chrome on a $200 Android tablet
2. Latest Safari on a 2020 iPhone SE
3. Latest Firefox on a Linux laptop
4. Latest Chrome on a Windows laptop

If those four all render the canvas + per-tile fade + safety badges correctly, ~95 % of likely operators are covered.

### 15.4 Will browser-tier offload limit future automation?

**No, if and only if you draw the trust line correctly.** The hard rule:

> **The base X8 publishes the canonical machine-readable state. The browser is one consumer among many.**

If the base publishes:

- the reconstructed canvas as a raw image stream over WebSocket,
- the detection vector list as JSON (mirrors topic `0x26`),
- the safety verdicts as MQTT topics,
- the per-tile staleness map and badges as metadata,
- the "AI accelerator: online/offline/degraded" status,

then **any** future consumer — a self-driving autopilot service running in another container on the X8, a fleet management dashboard, a remote OSE QA reviewer, an automated regression test — can subscribe to the same streams. The browser is not authoritative; it's just one renderer of state that already exists in machine-readable form.

**Where browser-tier offload *would* limit automation** is if a derived signal exists *only* in the browser. So:

> **Hard rule: any signal a future autopilot might consume must be produced on the base or the tractor, never only in the browser.**

What stays browser-only (display polish, never authoritative):

- per-tile cross-fade animation
- sub-tile temporal interpolation (visual-only smoothing between received tiles)
- staleness-clock visual rendering (badge *value* comes from base; badge *display* is browser)
- vector / bounding-box overlay rendering (data comes from base detection topics)
- layout, theming, audio cues, controller vibration

What never goes to the browser:

- the safety cross-check detector (J / Cα / Cβ) — must be on the base, must run against the canvas the operator actually sees, must publish its verdicts as topics
- ROI generation, valve-state interpretation, detection-bias decisions
- the staleness clock *value* (browser displays the number; base computes it)
- the "Enhanced / Synthetic / Cached / Wireframe / Predicted" badge *decisions*
- anything that decides whether to send a P0 ControlFrame

Maintaining this split means v26+ autopilot work plugs into the same backend that drives the browser. The UX layer and the perception/decision layer stay separate. **Browser-tier offload doesn't constrain autonomy — it actually *helps* by forcing a clean machine-readable state interface.**

### 15.5 Penalty for keeping it all on the base X8

Quantitative comparison, recommended Stack-NoCoral load:

| Item | Base-only | Browser-tier offload | Penalty for staying base-only |
|---|---|---|---|
| Base CPU steady-state | ~88 % | ~40 % | **48 % CPU** — safety-detector headroom evaporates under load |
| Per-tile fade frame rate | ~30 fps (CPU-rendered, GPU upload bottleneck on Vivante) | 60 fps (native to display device) | Visible jitter on fast-moving objects |
| Latency added to display | +50 ms (server-side render + LAN stream) | +5 ms (local render only) | ~45 ms — small but real on the operator-perception loop |
| Power draw on base | ~6 W steady | ~3 W steady | 50 % more heat to dissipate inside the indoor enclosure (matters for summer ambient) |
| Multi-operator scaling | linear penalty (each viewer = full re-render) | trivial (each viewer renders locally) | If 2+ people ever watch, the base falls over |
| Iteration speed for UI changes | OTA + service reboot | refresh tab | Days vs. seconds |
| Cold-start time after operator-device reboot | <500 ms (server already warm) | 1–3 s first load | Operator notices on first connect; cached after |

**The real cost of staying base-only is not the CPU percentage on paper.** It's that a base running at 88 % CPU lets the OS scheduler decide what to drop. Linux CFS will preempt the safety detector before it preempts the canvas-painting thread, because both run at the same nice level. **Push the canvas painting to the browser and the safety detector now runs against an idle CPU instead of competing with rendering.** That's the move that buys reliability under stress, not just headroom on paper. This is the deciding argument for browser-tier offload in v25.

### 15.6 What browser-tier offload changes in the recommended stack

Updates to [§14 Stack-NoCoral-Recommended](#14-top-recommendation--no-coral-system):

- **R4 base reconstruction** publishes the canvas + per-tile staleness + per-tile badge enum + detection vectors over WebSocket. **Server-side fade rendering is removed from the base.**
- **Browser** subscribes to the canvas stream, draws Canvas 2D blits per tile, runs a WebGL 2 fragment shader for per-tile cross-fade and staleness tint, renders bounding boxes from the detection topic, displays badge text/icons from the badge enum, plays audio-event banners.
- **Browser MUST refuse to render any tile whose badge enum is missing or malformed** — fail closed. Logged to console + back to the base health endpoint.
- **Server-side fallback render path stays alive** at 1 fps for the HDMI console and for headless QA. Same code path as today, just at lower priority.

### 15.7 Build-order impact for §14.4

Browser-tier offload becomes part of Phase 1 (was previously listed as opportunistic). Updated steps:

- **Week 3–4:** Persistent canvas + per-tile fade GLSL shader **in the browser**, staleness clock, Cached badge. [`canvas.py`](../DESIGN-CONTROLLER/BASE_STATION.md) publishes raw tiles + metadata; browser bundle does the rendering. Same week as before; the work just moves from base Python to browser JS.
- **Week 4 (new):** Browser test matrix — Chrome/Android, Safari/iOS, Firefox/Linux, Chrome/Windows. Gate before declaring Phase 1 complete.
- All later steps unchanged.

No schedule slip. The work is the same magnitude; it's just on the other side of the WebSocket.

### 15.8 Final answer

**Adopt browser-tier offload as a Phase-1 deliverable**, not a Phase-2 nice-to-have. Target WebGL 2 + Canvas 2D as the capability floor. Keep the trust boundary strict: base computes, browser renders. Maintain a minimal server-side render path for the HDMI console and headless QA. The 48 % base-CPU recovery is what makes Stack-NoCoral fit on a single i.MX 8M Mini without preempting the safety detector — that's the deciding reason, not the device-side polish.

---

## 16. Cross-review consensus check — do the four analyses agree?

This document, plus the three peer reviews from §10, all reached an independent top recommendation. Comparing them is the cleanest sanity check we have on whether the design is in a stable basin or whether an LLM is just confidently inventing.

### 16.1 Side-by-side recommendations

| Aspect | **This doc (Claude Opus 4.7)** §14 | **Gemini 3.1 Pro** v1.0 §4 | **Copilot v1.0** §1 + §2.4 | **Copilot In-Depth v1.0** §12 |
|---|---|---|---|---|
| **Wire encoding (R1)** | Tile-delta I/P frames | "Smart Block / Priority Queue" tile delta | Keyframe + tile-delta + ROI | Tile-delta I/P with priority score |
| **Tractor pre-process (R2)** | Image registration + NanoDet-Plus | None first; YOLOv8n later | None first; NanoDet-Plus / YOLOX-Nano / EfficientDet-Lite0 / SSDLite-MobileNet later | None first; PicoDet/NanoDet later |
| **ROI policy (R3)** | 3-level rings + valve-state + detection-bias + op-cued | Saliency-weighted blocks (centre crop / no sky) | Control-mode quality allocation | Quality rings (concentric) + control + hazard + age |
| **Base reconstruction (R4)** | Persistent canvas + background cache | Persistent HTML5 Canvas, browser-side patching | Persistent canvas + per-tile staleness | Persistent canvas + per-tile age + scene-memory background cache |
| **Base AI enhancement (R5)** | Real-ESRGAN-General-x4v3 CPU | Defer (browser handles smoothing) | Light enhancement OK | Real-ESRGAN small + optional RIFE / LaMa on base |
| **Base AI safety (R6)** | YOLOv8-nano CPU (or NanoDet-Plus) | YOLOv8n on tractor only (not base) | Two-detector pattern (tractor + base) | Two-detector pattern (tractor + base) |
| **Sidecars (R7)** | Person-alert + audio + 3-cam mux + event-skip + satellite | Bounding-box objects only | Detection metadata, link health | Hazard sidecars, motion vectors |
| **Browser-tier offload** | **Phase 1, mandatory** (§15) | Mid-term, "operator laptop GPU should assemble UI" | Yes — "third compute tier" | Yes — explicit fourth compute surface |
| **Coral / accelerator** | Optional, unproven, gate-spike | Not addressed | Optional USB Coral; degrade gracefully | Optional; "USB Coral safer than M.2"; degrade gracefully |
| **Long-term endgame** | Latent autoencoder (v26) | Latent autoencoder ("ultimate solution") | Learned compression (v26+) | Learned codec (Phase D / v26+) |
| **Safety honesty (badges)** | Mandatory: Enhanced/Synthetic/Cached/Wireframe/Predicted | Implicit (wireframe is obviously wireframe) | Explicit: never hide loss, label synthetic | Explicit: badges + raw-mode toggle + audit log |

### 16.2 The four-way consensus

**All four reviews agree on the following points** (this is the design's stable basin):

1. **Tile-delta over whole-frame thumbnails.** Unanimous. The byte savings on a slowly-changing scene are too large to ignore.
2. **Persistent canvas at the base / browser.** Unanimous. Frames are partial; the canvas accumulates.
3. **ROI matters and should be derived from control state.** Unanimous, even though we use different vocabulary (rings / saliency / mode).
4. **Tractor compute decides *what* to send; base compute reconstructs and enhances.** Unanimous asymmetry.
5. **First shippable pipeline has zero AI.** Unanimous. AI is a Phase-2+ layer on top of a working tile-delta system.
6. **Small detectors fit on the X8 A53s; large foundation models do not.** Unanimous on the constraint, with very similar model picks (NanoDet-Plus, YOLOX-Nano, PicoDet, MobileNet variants, YOLOv8n).
7. **Permissive licenses preferred** (Apache-2.0 / BSD over AGPL); flag Ultralytics. Three-way agreement (Gemini doesn't address it).
8. **Browser-tier offload is real and helpful.** Unanimous. This doc is the only one that elevates it to "mandatory Phase 1," but all four endorse the principle.
9. **Honest synthetic / enhanced / stale labelling.** Three-way explicit (Gemini implicit via wireframe-look).
10. **Latent autoencoder is the long-term direction once we have farm data.** Unanimous on direction; unanimous on "v26 not v25."

That's 10/10 on the design fundamentals across four independently-generated analyses by three different LLM families. The basin is stable.

### 16.3 Where the recommendations diverge — and why

Five real disagreements, in descending order of significance:

#### 16.3.1 Disagreement A — should the base run a second safety detector?

- **This doc + Copilot v1.0 + Copilot In-Depth: yes**, two-detector pattern (tractor M + base J/Cα/Cβ). Disagreements are logged for v26 retraining.
- **Gemini: no, only on tractor.** Gemini's plan has the base render bounding boxes from the tractor's metadata only. No independent verification.

**Who's right:** The two-detector recommendation. Without an independent base-side detector, an undetected obstacle on the tractor side is *also* invisible to the operator UI — single point of failure. The cost is ~50 % of one A53 core or ~1.5 % of one Coral Edge TPU; the safety win is the entire reason the system has cameras in the first place. **Adopt the three-way recommendation; reject Gemini's single-detector simplification.**

#### 16.3.2 Disagreement B — extreme degraded mode: wireframe vs. semantic-map vs. autoencoder

- **This doc:** wireframe (P, §10.5) for "ship something recognisable at 50 B/s" + optical-flow (Q) for slow drift; semantic-map deferred to v26.
- **Gemini:** wireframe + autoencoder are co-equal; semantic-map is also a peer.
- **Copilot v1.0:** none specifically; vague "send semantic metadata."
- **Copilot In-Depth:** semantic-map *and* motion-vector microframes *and* geometry-first all listed as v25-eligible.

**Who's right:** Probably this doc, with a caveat. Wireframe is the cheapest fallback that the operator can interpret without training. Semantic-map needs a mental model the operator doesn't have on day one ("what does it mean if there's a brown blob ahead-left?"). Autoencoder needs training data we don't have. **Ship wireframe + optical-flow in v25.5; defer semantic-map to v26 once operators have lived with the wireframe long enough to know whether they want richer fallback semantics.** Copilot In-Depth is right that *all three* belong in the long-term toolkit; the question is shipping order, not inclusion.

#### 16.3.3 Disagreement C — Y-only / grayscale baseline

- **Copilot v1.0:** explicit recommendation ("Y-only grayscale frames with occasional color references"). Cited as part of the first-shippable non-AI pipeline.
- **This doc, Gemini, Copilot In-Depth:** not addressed.

**Who's right:** Copilot v1.0 has a real point that I missed. Y-only WebP at the same byte budget gives roughly 30–40 % more spatial detail than full YCbCr because chroma is dropped. For a tele-op safety scenario, *spatial detail in the luma channel is what matters* — colour is mostly cosmetic. **Adopt: add Y-only mode to the §11.2 table as scheme Z (Phase 1, no wire change, just an encoder flag).** Send a colour reference frame every 30 s; the base re-colourises grayscale tiles by sampling the most recent colour reference. Marked **"Recolourised"** badge — same honesty rules as the rest. Add to the §13 compatibility matrix as a parallel R1 modifier (works with B, augments E and I).

#### 16.3.4 Disagreement D — base-side super-res in v25

- **This doc:** yes (H), Phase 2, Real-ESRGAN-General-x4v3 CPU. Frees the operator from squinting at thumbnails.
- **Copilot In-Depth:** yes, Phase B, with Real-ESRGAN small + optional RIFE.
- **Copilot v1.0:** light enhancement OK; no specific model.
- **Gemini:** **explicitly defers SR to the browser** ("operator laptop GPU should assemble the UI") rather than the base.

**Who's right:** Both, layered. Gemini is correct that *some* enhancement should run in the browser (per §15.1 — display polish wants to live at the display, free GPU). But Real-ESRGAN-x4 is too heavy for reliable cross-browser deploy in 2026 (WebGPU coverage <75 %, WebNN <30 % per §15.3). **Run Real-ESRGAN on the base; let the browser do the cheap GLSL shader sharpening + per-tile fade + bounding-box overlay.** This is what §15.6 already specifies. Gemini's instinct is right but the *which* enhancement goes where matters: heavy stable-API on the base, light bleeding-edge on the browser.

#### 16.3.5 Disagreement E — pre-diff image registration

- **Copilot v1.0:** explicit (§5.4); my §10.2 picked it up.
- **This doc:** adopted in §14.1 R2.
- **Gemini, Copilot In-Depth:** not mentioned.

**Who's right:** Copilot v1.0 + this doc. The omission in the other two reviews is a real gap. Without pre-diff registration, the §2 byte-savings claim falls apart the moment the tractor starts driving (engine vibration alone moves every tile by a few pixels = every tile flagged "changed"). **Adopt; this is non-negotiable for any deployed v25 system. Already in §14.4 build week 2.**

### 16.4 Net assessment

Out of ~10 design fundamentals: **4 of 4 reviews agree.** Out of ~5 second-order choices: **3 of 4 agree on each, with the lone dissent always wrong on at least one specific.** No major architectural disagreement. No "I would build a fundamentally different system" anywhere.

What this means practically:

- **The design is over-determined.** Multiple independent paths reach the same place. Confidence is high that this is the right shape of system.
- **The remaining disagreements are tactical, not strategic.** All five disagreements above are about which item ships in which phase, not about whether to build the thing.
- **Five additions land back into this doc** (Y-only as scheme Z; explicit base-vs-browser SR split per §15.6; pre-diff registration confirmed; two-detector defended; degraded-mode shipping order finalised). All are already captured above; this section just makes the cross-review reasoning explicit.
- **Two open scope decisions remain for OSE** (not LLM-resolvable): the AGPL stance on Ultralytics (§9 Q1) and whether to put the Coral on the BOM (§14.6). These would settle once a human stakeholder weighs in.

**No further design analysis should be needed before implementation.** The next layer of useful work is benchmarking the actual hardware to replace the engineering estimates in §11–§12 with measurements.

### 16.5 New scheme Z added to the master pipeline

Per the §16.3.3 adoption, add to §11.2 master ranking and §13.1 role taxonomy:

| # | Scheme | Fidelity | FPM | Tractor CPU | Base CPU | Bytes / refresh | Latency added | Feasibility | Safety honesty | Phase |
|---|---|---:|---:|---:|---:|---:|---:|---:|:-:|:-:|
| Z | **Y-only luma + 30 s colour reference + base recolourisation** *(Copilot v1.0)* | 5 (mono detail > colour blur) | ~40 | 50 % | 35 % | 150–350 | +5 | 9 | ⚠️ Recolourised badge | 1 |

Compatibility (R1 modifier; combines with B / C / D / E / F / G / H / I / J / K / M / N / S / T / U; conflicts with W which has its own colour model; redundant with O / Y on tiles where colour is being filled-in by other means).

Build-order impact: insert as **Week 2.5** in the §14.4 / Stack-NoCoral build (one-day flag in the WebP encoder + a recolouriser stage in `canvas.py` on the base). Toggleable per-link-quality — auto-enable Y-only when bytes/refresh drops below ~400 B.

---


---


---




