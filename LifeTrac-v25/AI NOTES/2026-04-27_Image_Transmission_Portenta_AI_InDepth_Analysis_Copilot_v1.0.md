# LifeTrac v25 Image Transmission + Portenta AI In-Depth Analysis - Copilot v1.0

Date: 2026-04-27
Author: GitHub Copilot
Scope: Deep analysis of image transmission over the v25 LoRa stack, with focus on maximizing compute across Portenta hardware (tractor and base, especially base), evaluating open-source AI model fit, and designing targeted keyframe-like image updates.
Primary references:
- [DESIGN-CONTROLLER/LORA_PROTOCOL.md](../DESIGN-CONTROLLER/LORA_PROTOCOL.md)
- [DESIGN-CONTROLLER/VIDEO_OPTIONS.md](../DESIGN-CONTROLLER/VIDEO_OPTIONS.md)
- [DESIGN-CONTROLLER/BASE_STATION.md](../DESIGN-CONTROLLER/BASE_STATION.md)
- [DESIGN-CONTROLLER/ARCHITECTURE.md](../DESIGN-CONTROLLER/ARCHITECTURE.md)
- [AI NOTES/2026-04-27_LoRa_InDepth_Analysis_Copilot_v1.0.md](2026-04-27_LoRa_InDepth_Analysis_Copilot_v1.0.md)

---

## 1. Executive Summary

The image link should be treated as a layered perception pipeline, not a "tiny video stream".

Most important conclusions:

1. The best near-term strategy is hybrid keyframe + targeted updates (tile deltas), not repeated full thumbnails.
2. Portenta X8 can run useful open-source models at low resolution and low FPS when quantized; the base station can run much heavier enhancement than the tractor.
3. The base station should become a reconstruction engine: reassembly, enhancement, interpolation, semantic overlay, and uncertainty visualization.
4. The tractor should spend compute on deciding what matters (change detection + ROI + small detector), while the base spends compute on making low-bandwidth data operator-usable.
5. Some model families fit now (NanoDet/PicoDet/Fast-SCNN/Real-ESRGAN at small sizes); large diffusion-style models do not fit practically on CPU-only X8 for real-time.
6. A strict safety UX policy is mandatory whenever synthetic/enhanced pixels are shown.

---

## 2. Current Constraints and Opportunity

From current docs and protocol:

- LoRa is half-duplex and contention-sensitive.
- Image topics already exist (`0x20`, `0x21`, `0x23`) and camera selection command exists (`CMD_CAMERA_SELECT`).
- Existing design already expects low-rate thumbnails and base-side reconstruction/enhancement.
- Base station X8 is always on and LAN-connected to operator browser, enabling additional processing/offload.

Key opportunity: move from whole-image pushes to selective update semantics while preserving control-path priority (P0/P1 over P3).

---

## 3. Use All Portenta Compute: Workload Partition

### 3.1 Compute surfaces to use

There are four practical compute surfaces in the deployed system:

1. Tractor X8 Linux side (A53 cores): camera ingest, diffing, lightweight CV, payload packing.
2. Tractor H747 (M7/M4): deterministic scheduling, queue governance, hard real-time guardrails.
3. Base X8 Linux side (A53 cores): reassembly, enhancement, object/depth overlays, quality scoring.
4. Operator browser GPU/CPU: final rendering, compositing, animation, optional WebAssembly/WebGPU inference.

### 3.2 Recommended task split

| Stage | Tractor X8 | Tractor H747 | Base X8 | Browser |
|---|---|---|---|---|
| Camera capture / resize | yes | no | optional sanity checks | no |
| Tile change detection | yes | no | no | no |
| ROI scoring and ranking | yes | no | optional override | optional operator ROI input |
| LoRa priority/token gate | no | yes | no | no |
| Packet reassembly | no | no | yes | no |
| Super-resolution / deblocking | no | no | yes | optional |
| Interpolation / smoothing | no | no | yes | yes |
| Overlay rendering (boxes, masks) | minimal metadata only | no | yes | yes |
| Final UI compositing | no | no | optional | yes |

### 3.3 "Especially base station" strategy

The base should host a dedicated visual reconstruction service with these responsibilities:

1. Validate and assemble image fragments.
2. Maintain a persistent scene canvas and per-tile staleness map.
3. Run enhancement pipeline (deblock, SR, optional interpolation).
4. Fuse side-channel metadata (detections, hazard flags, motion vectors).
5. Publish both raw and enhanced outputs to UI.

This allows the tractor to remain conservative and deterministic while still delivering a high-quality operator view.

---

## 4. Open-Source AI Model Fit on Portenta X8

### 4.1 Practical fit rule

On CPU-only X8 (quad A53), assume real-time means:

- Tractor side target: <= 40 to 80 ms per inference for frequent tasks.
- Base side target: <= 50 to 200 ms per enhancement step is acceptable.
- Quantized INT8 models with small input size are required for steady performance.

### 4.2 Candidate models that can fit now

Approximate fit guidance for INT8/optimized runtime on Portenta X8:

| Model | Primary use | Typical size | X8 CPU feasibility | License notes |
|---|---|---:|---|---|
| NanoDet-Plus | object detection | ~2-4 MB | good (low-res, few FPS) | Apache-2.0 |
| PP-PicoDet | object detection | ~1-3 MB | very good | Apache-2.0 |
| YOLOX-Nano | object detection | ~3-6 MB | moderate-good | Apache-2.0 |
| Fast-SCNN | semantic segmentation | ~1-5 MB | good at low res | implementation-dependent, often Apache-2.0 |
| BiSeNetV2 (small) | segmentation | ~5-15 MB | moderate | implementation-dependent |
| MiDaS small variants | monocular depth | ~10-30 MB | moderate-low FPS | usually MIT variants |
| Real-ESRGAN small/general x4 | super-resolution | ~15-70 MB | moderate on base, weak on tractor | BSD-3-Clause |
| RIFE (small) | frame interpolation | ~10-40 MB | moderate on base only | check repo license version |
| LaMa (small region inpainting) | fill dropped tiles | ~20-60 MB | moderate on base only | Apache-2.0 |

### 4.3 Models that are generally not practical on CPU-only X8 for real-time

1. Large diffusion image models (Stable Diffusion class) for low-latency live reconstruction.
2. Heavy video transformers without accelerator.
3. Large multi-task foundation models expecting desktop GPU.

These are still useful offline or for occasional repair frames, but not as continuous real-time path.

### 4.4 Critical licensing caution

- Ultralytics YOLO repositories are AGPL for recent releases.
- If you need permissive licensing alignment, prefer PicoDet/NanoDet/YOLOX families and verify exact license at the pinned commit.

---

## 5. Runtimes and Toolchains for Portenta

Recommended inference/runtime stack:

1. TFLite + XNNPACK for smallest deployment friction.
2. NCNN for very small mobile-class CV models.
3. ONNX Runtime for portability where model conversion is easier.

Engineering guidance:

- Standardize on INT8 quantized exports.
- Use fixed input sizes per mode (for example 160x96 or 192x112) to avoid allocator churn.
- Pin one runtime per model family to reduce maintenance complexity.

---

## 6. Targeted Image Updates (Keyframe-Style, Adapted to LoRa)

### 6.1 Frame taxonomy

Use three visual frame classes over existing video topic channels:

1. I-frame (Keyframe): full thumbnail payload, periodic and on resync.
2. D-frame (Delta): only changed tiles/regions since last acknowledged visual state.
3. M-frame (Metadata): semantic updates only (detections, hazard classes, active ROI, optional motion vectors).

### 6.2 Proposed tile-delta protocol behavior

At each candidate image tick:

1. Downsample to working frame (for example 128x72 or 160x90).
2. Partition into fixed grid tiles.
3. Compute per-tile change metric against last acknowledged state.
4. Compute per-tile priority score:

$$score_i = w_c C_i + w_r R_i + w_h H_i + w_a A_i$$

Where:
- $C_i$: visual change magnitude
- $R_i$: ROI relevance (operator/task region)
- $H_i$: hazard relevance (detector overlap)
- $A_i$: age/staleness penalty

5. Select top tiles within current P3 byte budget.
6. Transmit D-frame containing tile coordinates + compressed tile payloads.
7. Force I-frame on schedule or when delta cost exceeds keyframe cost threshold.

### 6.3 Why this beats full-thumbnail mode

- Static background stops consuming bandwidth.
- Bucket/workzone updates stay frequent.
- Link collapse degrades gracefully into sparse updates + periodic keyframes.
- Control-path starvation risk is lower because payload bursts are bounded.

### 6.4 Recommended defaults for v25

1. Keyframe interval: 10 to 30 s adaptive by motion/link quality.
2. Tile size: 16x16 or 32x32 depending on selected working resolution.
3. Delta admission threshold: dynamic by queue pressure.
4. Hard cap: no visual fragment can violate P3 max airtime slice.

---

## 7. Advanced Targeted-Update Variants (Outside-the-Box)

### 7.1 ROI ring encoding

Instead of one rectangular ROI, encode concentric quality rings:

- Ring 0 (critical): bucket + near obstacle zone, highest fidelity.
- Ring 1 (operational): surrounding work zone, medium fidelity.
- Ring 2 (context): far field/horizon, low or sparse updates.

This aligns bytes with operator attention better than uniform compression.

### 7.2 Semantic refresh mode

When link is weak, send semantic map deltas rather than pixel tiles:

- coarse class grid
- obstacle boxes
- confidence
- motion direction

Base reconstructs a synthetic guidance overlay over last good keyframe.

### 7.3 Motion-vector microframes

Send low-byte optical-flow or block-motion vectors between keyframes.

Use case: preserve motion awareness when pixel updates are sparse.

### 7.4 Geometry-first mode

Transmit edge/contour primitives (line segments or sparse polygonal boundaries) for critical structures (bucket edge, obstacle boundary).

This can provide high tactical awareness at very low bitrate.

### 7.5 Scene-memory background cache

Base station maintains long-lived background tile cache.

Tractor transmits only invalidation messages for changed background regions plus high-priority foreground updates.

---

## 8. Farm/Offroad-Specific Model Strategy

### 8.1 Class set that matters operationally

Prioritize classes and outputs that affect immediate safety and productivity:

1. person
2. animal
3. vehicle/equipment
4. obstacle mass (rock/log/post/fence)
5. bucket/implement silhouette
6. free-space confidence region

### 8.2 Dataset strategy

General public models should be fine-tuned on farm/offroad data:

- Start from COCO/OpenImages pretrained backbones.
- Add LifeTrac-specific data captures from tractor cameras (dust, mud, dawn/dusk, vibration, glare).
- Track false negatives in field logs and retrain iteratively.

### 8.3 Two-detector safety pattern

Run a lightweight detector on tractor for scheduling.
Run a stronger detector on base for verification and operator warning.

This gives better safety than relying on one small edge model.

---

## 9. Base-Station Reconstruction Pipeline (Recommended)

### 9.1 Pipeline stages

1. Ingress and validation
- packet order/replay checks
- fragment completeness checks

2. Reconstruction state
- persistent canvas
- per-tile age map
- keyframe anchor state

3. Enhancement chain
- deblock/denoise
- super-resolution
- optional interpolation
- optional inpaint for non-critical stale tiles

4. Semantics and overlays
- detector boxes
- confidence masks
- stale/synthetic badges

5. UI publish
- raw channel (truth first)
- enhanced channel (operator-assist)

### 9.2 Safety display rules

1. Always show age of latest real pixel update.
2. Clearly label enhanced/synthetic content.
3. Offer one-click raw-only view.
4. Never hide packet loss or stale regions.

---

## 10. Concrete v25 Implementation Path

### Phase A: No-model targeted updates (fastest high-value)

1. Implement I-frame + D-frame + staleness map.
2. Add ROI scoring from control state (bucket/drive context).
3. Add browser-side patch compositing and age overlays.

Expected result: large bandwidth reduction with immediate operator value.

### Phase B: Lightweight tractor AI + stronger base AI

1. Tractor: PicoDet/NanoDet for hazard-aware tile ranking.
2. Base: Real-ESRGAN + optional RIFE at modest resolution.
3. Base: independent detector for cross-check.

Expected result: better detail where it matters and better hazard awareness.

### Phase C: Semantic and synthetic fallback modes

1. semantic refresh mode for weak-link operation.
2. motion-vector microframes.
3. optional inpaint for visual continuity.

Expected result: graceful degradation under poor RF.

### Phase D (v26+): Learned codec experiments

1. small autoencoder latent transport.
2. keyframe + learned residual decode.
3. benchmark against tuned tile-delta baseline.

Expected result: potential additional compression, higher complexity.

---

## 11. Validation and Gate Metrics

Add image-system gates alongside existing control-link metrics:

1. P99 control latency remains within safety target while image updates enabled.
2. Zero P3-induced P0 starvation in 30-minute stress tests.
3. Mean and worst-case image age stay inside defined profile limits.
4. Hazard-detection recall for person/animal classes meets field threshold.
5. Operator can always switch to raw view and identify stale/synthetic regions.

---

## 12. Final Recommendations

1. Ship targeted delta updates first. This is the largest practical gain per engineering hour.
2. Use tractor compute for prioritization, not heavy reconstruction.
3. Use base compute aggressively for reconstruction and assistive visualization.
4. Start with permissive-license small models (PicoDet/NanoDet/Fast-SCNN class) and avoid licensing surprises.
5. Keep a strict honesty policy in UI for any synthesized pixels.
6. Treat learned codecs as follow-on after tile-delta baseline is measured.

Bottom line: Yes, public open-source AI models can fit on Portenta X8 for farm/offroad tasks, but the architecture must be asymmetric: tractor decides what to send, base does the heavy reconstruction, and the protocol prioritizes targeted updates over full-frame transmission.

---

## 13. Consolidated Option Review and Top Recommendation

This section explicitly reviews the major image-transmission and image-management options considered across the current analyses and selects one primary recommendation for v25.

### 13.1 Option families reviewed

1. Full-thumbnail microframe transport.
- Simple and low-risk, but wastes airtime whenever most of the scene is unchanged.

2. Keyframe plus targeted tile deltas.
- Best bytes-to-information ratio for mostly-static farm scenes.

3. ROI-prioritized tile transmission (including operator-cued ROI and ROI rings).
- Keeps critical work zones clear while preserving context.

4. Metadata and semantic sidecars (detections, alerts, class grids).
- Very strong safety and control value at low bandwidth cost.

5. Motion-vector or optical-flow microframes.
- Good degraded-link continuity layer, but not a primary truth source.

6. Geometry-first wireframe mode.
- Useful as an emergency fallback, not as a normal operator view.

7. Semantic-map synthetic rendering.
- Very bandwidth-efficient, but lower trust and requires clear synthetic labeling.

8. Learned latent codecs (autoencoder family).
- High long-term potential, but dataset/training and validation burden is too high for first v25 integration.

9. Base-heavy enhancement stack (super-resolution, interpolation, inpaint, overlays).
- Strong operator-value multiplier if uncertainty is displayed clearly.

10. Multi-camera adaptive allocation.
- High value once the single-camera path is stable; should be adaptive, not round-robin.

### 13.2 Decision criteria

Options were scored against practical v25 criteria:

1. Safety under half-duplex contention.
2. Protection of P0/P1 traffic from visual traffic.
3. Fidelity per byte over real LoRa budgets.
4. Fit to CPU-only Portenta X8 on tractor and base.
5. Implementation and maintenance risk for v25 timeline.
6. Transparency of operator UX (raw vs enhanced vs synthetic).

### 13.3 Top recommendation

Top recommendation for v25 is the following integrated stack:

1. Transport baseline:
- Hybrid keyframe + tile-delta transport as the primary image channel.
- Hard P3 fragment airtime cap so image traffic cannot starve control.

2. Tractor-side intelligence:
- pHash or SSD tile change detection plus lightweight pre-diff registration.
- ROI ring allocation driven by machine state and optional operator cue.
- Add lightweight detector sidecar for hazard-aware prioritization when schedule allows.

3. Base-side reconstruction:
- Persistent canvas with per-tile age/staleness map.
- Enhancement chain: deblock/denoise -> super-resolution -> optional interpolation.
- Independent base safety detector for cross-check and operator warning.

4. UI policy:
- Always expose image age, stale regions, and enhanced/synthetic labels.
- Keep one-click raw view available at all times.

5. Fallback ladder:
- Normal: tile-delta + enhancement.
- Marginal link: ROI-first + reduced cadence.
- Poor link: metadata plus motion/geometry degraded modes.

### 13.4 Why this wins over alternatives

1. Better than full-thumbnails:
- Avoids re-sending static background, reducing airtime pressure.

2. Better than metadata-only as primary:
- Preserves human-verifiable visual truth while still using semantic assist.

3. Better than immediate learned-codec adoption:
- Ships now on known toolchains and avoids training-data bottlenecks.

4. Better than synthetic-first views:
- Maintains operator trust by centering raw pixel evidence and explicit uncertainty.

### 13.5 Practical v25 adoption path

1. Phase 1 (must ship): keyframe+delta, ROI policy, persistent canvas, staleness UX.
2. Phase 2 (high-value): base SR/interpolation, independent base detector.
3. Phase 3 (stretch): tractor detector biasing, motion-vector degraded mode, multi-camera adaptive budget.
4. v26+: learned latent codecs and richer synthetic rendering.

Final call: For v25, build around deterministic hybrid delta transport plus base reconstruction, then layer semantic intelligence on top. This gives the best safety, fidelity-per-byte, and delivery confidence on current Portenta hardware.
