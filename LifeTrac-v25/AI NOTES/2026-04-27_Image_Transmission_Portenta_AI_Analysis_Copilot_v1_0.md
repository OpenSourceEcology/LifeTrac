# LifeTrac v25 Image Transmission and Portenta AI Analysis - Copilot v1.0

Date: 2026-04-27
Author: GitHub Copilot
Document version: Copilot v1.0
Status: Design analysis and recommendation note. No code changes are made by this document.

Primary references:
- [DESIGN-CONTROLLER/VIDEO_OPTIONS.md](../DESIGN-CONTROLLER/VIDEO_OPTIONS.md)
- [DESIGN-CONTROLLER/MASTER_PLAN.md](../DESIGN-CONTROLLER/MASTER_PLAN.md)
- [DESIGN-CONTROLLER/LORA_PROTOCOL.md](../DESIGN-CONTROLLER/LORA_PROTOCOL.md)
- [DESIGN-CONTROLLER/RESEARCH-CONTROLLER/VIDEO_COMPRESSION/README.md](../DESIGN-CONTROLLER/RESEARCH-CONTROLLER/VIDEO_COMPRESSION/README.md)
- [DESIGN-CONTROLLER/RESEARCH-CONTROLLER/VIDEO_COMPRESSION/GRAYSCALE_RECOLORIZATION.md](../DESIGN-CONTROLLER/RESEARCH-CONTROLLER/VIDEO_COMPRESSION/GRAYSCALE_RECOLORIZATION.md)
- [DESIGN-CONTROLLER/RESEARCH-CONTROLLER/VIDEO_COMPRESSION/FUTURE_FRAME_PREDICTION.md](../DESIGN-CONTROLLER/RESEARCH-CONTROLLER/VIDEO_COMPRESSION/FUTURE_FRAME_PREDICTION.md)
- [AI NOTES/2026-04-27_LoRa_InDepth_Analysis_Copilot_v1_1.md](2026-04-27_LoRa_InDepth_Analysis_Copilot_v1_1.md)

Companion notes already exist from Gemini and Claude. This note tries to be the practical system-design version: what to run on the tractor X8, what to run on the base X8, what public open-source models plausibly fit, how targeted image updates should work, and where the strange but useful ideas belong.

---

## 1. Executive verdict

The best image-transmission strategy for LifeTrac v25 is not "make JPEG smaller." It is a layered perception pipeline:

1. Keep raw camera capture and full-quality local recording on the tractor X8.
2. Transmit a tiny, honest visual stream over LoRa as P3 data only when P0/P1 radio health allows it.
3. Maintain a persistent image canvas at the base station.
4. Send targeted updates: full keyframes occasionally, tile deltas often, ROI tiles at higher quality, and semantic metadata whenever it is more useful than pixels.
5. Spend base-station compute on reassembly, smoothing, recolorization, super-resolution, cross-check detection, synthetic overlays, and operator UI clarity.
6. Use open-source AI models carefully: small detectors and segmenters fit on the Portenta X8 CPU; large foundation models do not. The base station can run heavier models, especially if we use the operator browser or an optional USB accelerator.

The first shippable pipeline should be non-AI:

- keyframe plus tile-delta updates,
- ROI quality allocation based on control mode,
- Y-only grayscale frames with occasional color references,
- persistent base canvas with stale-tile indicators,
- browser-side canvas patching and smooth fades.

The first AI upgrade should be small and pragmatic:

- tractor-side NanoDet-Plus, YOLOX-Nano, EfficientDet-Lite0, or SSDLite MobileNet for person/animal/vehicle/obstacle detection,
- detection metadata sidecars over LoRa,
- base-side higher-confidence cross-check detector,
- object/ROI overlays in the UI.

The more speculative path is learned compression: a custom LifeTrac autoencoder or keyframe-plus-latent codec. That can work, but it needs real tractor footage first. The best v25 move is to collect the dataset while shipping the deterministic tile/ROI pipeline.

---

## 2. Hardware and compute reality

### 2.1 The Portenta Max Carrier is not the computer

The Max Carrier provides carrier-board functions: power, connectors, radios, Ethernet, mPCIe/cellular integration, antennas, USB, field buses, and access to the X8/H747. The actual Linux compute in this plan is the Portenta X8 module.

When this note says "Portenta compute," it means:

- i.MX 8M Mini Linux side on the Portenta X8,
- STM32H747 co-MCU for realtime LoRa/control/Modbus work,
- optional operator laptop/tablet browser connected to the base over LAN.

The H747 should not do image processing beyond maybe tiny health checks. It owns realtime control and safety-related radio work. The Linux A53 cores own cameras, compression, logging, and AI.

### 2.2 Portenta X8 practical envelope

Per the current master plan, both tractor and base use the Portenta X8:

| Resource | Practical meaning for image work |
|---|---|
| Quad Cortex-A53 at 1.8 GHz | Enough for OpenCV, WebP/JPEG, small INT8 TFLite/ONNX/ncnn models |
| 2 GB RAM | Enough for several small models, frame buffers, and Python services |
| NEON SIMD | Important for TFLite XNNPACK, OpenCV, ncnn, image transforms |
| Linux + Docker/Yocto | Easy service split: capture, encode, detect, reassemble, enhance |
| H747 co-MCU | Keep safety-critical LoRa/control away from Linux jitter |
| No high-end NPU assumed | Treat all ML as CPU unless an optional accelerator is explicitly added |

The X8 can run small AI models. It cannot run desktop-class vision foundation models at useful rates.

Approximate CPU-only guidance, assuming INT8 or otherwise optimized inference at small resolutions:

| Workload | Tractor X8 fit | Base X8 fit | Notes |
|---|---|---|---|
| pHash/SAD tile change detection | easy | easy | first-build material |
| WebP/JPEG thumbnail encode/decode | easy | easy | measure actual Yocto codec support |
| OpenCV optical flow at 160x120 to 256x192 | moderate | moderate/easy | good base-side tool |
| Small detector at 192x192 or 320x320 | moderate | easy | 1-5 fps depending model |
| Lightweight segmentation at 160x120 to 320x240 | moderate | easy | useful for semantic maps |
| Tiny autoencoder encode/decode | moderate | easy | needs training data |
| RIFE-tiny/FILM-small interpolation | too heavy for tractor | possible on base at low res | optional UX layer |
| Real-ESRGAN/SwinIR full models | no | slow unless heavily reduced | use tiny models or optional accelerator |
| SAM/GroundingDINO/Stable Diffusion/large world models | no | no, unless external GPU/cloud | not v25 onboard |

### 2.3 The base has more usable compute than the tractor

The base station is indoors, powered, cooled more easily, and not in the safety loop. Use that asymmetry.

The base can:

- use all A53 cores without worrying about tractor battery draw,
- maintain larger buffers and model sets,
- run post-processing after LoRa delivery without affecting the H747 control loop,
- pass work to the operator browser using WebAssembly, WebGL, WebGPU, Canvas, or WebCodecs,
- optionally use a USB AI accelerator if a future BOM permits it.

Important caution: do not assume an M.2 accelerator fits until the actual Max Carrier slot and cellular configuration are checked. A USB Coral-style accelerator is the safer optional-add-on assumption because it only needs USB and power. The pipeline must still degrade gracefully with no accelerator.

### 2.4 Use the operator browser as a third compute tier

The operator's laptop/tablet is often stronger than the base X8. The browser can do:

- persistent canvas patching,
- per-tile fade transitions,
- WebGL shader sharpening,
- stale-tile overlays,
- vector/object overlays,
- map/digital-twin rendering,
- possibly WebGPU/WebNN inference on newer devices.

Recommendation: keep the base X8 as the trusted reassembly/router/enhancement service, but move display-only smoothing and rendering to the browser. This uses compute already present and keeps the base responsive.

---

## 3. Radio budget constraints for image data

The LoRa note concluded that Profile B is the most practical first-build target: SF7/BW250/CR4/5, 10 Hz control, single PHY both directions, explicit grant windows, P3 traffic only when P0/P1 are healthy.

The exact image budget depends on the final scheduler, but first-order design should assume a few hundred bytes per second of safe P3 image throughput, not tens of kilobytes.

That drives these rules:

1. Image data is P3 and must disappear before control suffers.
2. A single image object can take seconds to complete.
3. Any chunk can be dropped.
4. The base UI must show age and completeness.
5. The best visual stream is a persistent reconstruction, not independent standalone frames.

The target is not video. The target is a continuously maintained visual state estimate.

---

## 4. Recommended system architecture

### 4.1 Tractor X8 responsibilities

The tractor X8 should do only the compute that must be close to the raw camera:

- capture UVC/MIPI frames,
- stabilize/crop/downsample,
- keep local full-quality recording,
- decide which camera and ROI matters,
- compute tile changes,
- encode tiny keyframes/tiles,
- run small detection/segmentation models if enabled,
- publish image chunks and metadata to the H747 LoRa queue,
- never block H747 control reads.

### 4.2 H747 responsibilities

The H747 should not understand images deeply. It should:

- enforce P0/P1/P2/P3 scheduling,
- drop P3 when control is stressed,
- bound P3 fragment airtime,
- carry opaque image chunks,
- expose link health to the X8.

### 4.3 Base X8 responsibilities

The base X8 should do the expensive and stateful work:

- receive image chunks from MQTT/LoRa bridge,
- reassemble image objects,
- maintain a per-camera canvas,
- track per-tile age and confidence,
- request keyframes or color references when needed,
- run base-side enhancement models,
- run independent detection cross-checks,
- publish display frames and metadata to the web UI.

### 4.4 Browser responsibilities

The browser should render the operator experience:

- apply canvas patches,
- fade tile updates,
- draw bounding boxes and vectors,
- draw the digital twin/map overlay,
- show badges: RAW, ENHANCED, RECOLORIZED, PREDICTED, SYNTHETIC, STALE,
- never hide loss or age.

---

## 5. Targeted image updates: keyframe plus smart tile deltas

This is the main recommendation.

### 5.1 Concept

Maintain a persistent canvas at the base. The tractor sends:

- occasional full keyframes,
- frequent changed tiles,
- ROI tiles at higher quality,
- object/scene metadata sidecars,
- refresh requests/resyncs when needed.

This is adapted from video I/P frames, but it is not normal H.264. It is built for short, lossy, P3 LoRa fragments.

### 5.2 Frame grid

Start small:

- Source working frame: 192x128 or 256x160.
- Tile size: 32x32.
- Grid examples:
  - 192x128 -> 6x4 = 24 tiles.
  - 256x160 -> 8x5 = 40 tiles.
  - 384x256 -> 12x8 = 96 tiles, probably too much for first build.

Use fewer larger tiles first. A 24- or 40-tile grid is easier to transmit, debug, and visualize.

### 5.3 Tile delta object

A practical object format:

```c
ImageDeltaObject {
    uint8_t  stream_id;          // front, rear, implement, crop
    uint8_t  object_type;        // keyframe, delta, roi_tile, color_ref, metadata
    uint16_t image_seq;
    uint16_t base_key_seq;       // delta depends on this keyframe
    uint16_t capture_age_ms;     // age at tractor enqueue time
    uint8_t  grid_w;
    uint8_t  grid_h;
    uint8_t  tile_size_px;       // 16, 24, 32
    uint8_t  codec;              // raw_luma, jpeg, webp, avif, rle_mask
    uint8_t  flags;              // enhanced_allowed, y_only, synthetic_sidecar, urgent
    uint8_t  changed_bitmap[];
    TileBlob tile_blobs[];
}
```

Each `TileBlob` should include tile index, quality rung, length, and a small checksum. It then fragments across existing LoRa telemetry/image fragments. Do not require all tiles to arrive for the UI to update.

### 5.4 Change detection

First-build options:

| Method | Tractor compute | Good at | Weakness |
|---|---:|---|---|
| SAD on downsampled luma tile | very low | motion/change detection | lighting flicker causes false changes |
| pHash per tile | low/moderate | robust perceptual changes | may miss tiny important objects |
| block motion estimate | moderate | camera motion compensation | more code |
| tiny change-mask CNN | moderate/high | semantic changes | needs dataset |

Recommended v25 start: SAD plus pHash, after a cheap global image registration step. Camera shake can make every tile look changed; aligning the current frame to the previous keyframe by a few pixels before tile comparison is worth it.

### 5.5 Keyframe decision

Every update cycle, estimate delta cost. If delta cost is too large, send a keyframe instead.

```text
if changed_tile_count == 0:
    send heartbeat-only image status
elif estimated_delta_bytes < 0.70 * estimated_keyframe_bytes:
    send tile delta
else:
    send keyframe
```

This avoids the bad case where a dirty, dusty, fast-turning scene sends 40 tiny tiles that are larger than one small keyframe.

### 5.6 Per-tile age and honesty

The base canvas must track age per tile. If a tile has not been refreshed recently:

- age 0-2 refresh intervals: normal,
- age 3-5: subtle stale tint,
- age 6+: obvious stale overlay or blur,
- missing base keyframe: discard deltas and request keyframe.

This is central. A patched canvas can look complete even when half of it is old. The UI must not let that happen silently.

### 5.7 Why this beats whole-frame thumbnails

Whole-frame thumbnail:

- simple,
- independent frames,
- predictable decode,
- wastes bytes on unchanged sky/field/cab structure.

Tile-delta canvas:

- more stateful,
- needs resync,
- updates important areas faster,
- lets the base use temporal accumulation,
- turns LoRa's low bitrate into a usable visual memory.

For a farm/off-road vehicle, much of the scene changes slowly. The tile-delta model matches the environment.

---

## 6. ROI and saliency: spend bytes where work is happening

### 6.1 Control-derived ROI

Use signals already available:

| Control state | Preferred visual ROI |
|---|---|
| driving forward | center/lower-center path tiles |
| reversing | rear camera, lower-center path tiles |
| boom up/down | implement/bucket tiles |
| bucket curl/dump | bucket lip and material contact patch |
| parked | wide low-rate context, no high-rate ROI unless event occurs |
| manual camera pin | operator-selected camera gets most P3 budget |

This requires no AI. It is just a table mapping operating mode to visual priority.

### 6.2 Operator-cued ROI

Let the operator tap a region in the video tile. The base sends a tiny ROI hint:

```text
camera_id, roi_x_cell, roi_y_cell, duration_s
```

This is maybe 4 bytes. For the next few seconds, the tractor spends image bytes there. This is a powerful interaction because it lets the human use intuition and lets LoRa only answer the specific question.

### 6.3 Detection-derived ROI

Once a detector exists, detected person/animal/vehicle/obstacle tiles become high-priority ROI automatically. The detector does not need to be perfect to be useful here: a false positive costs image bytes; a true positive focuses attention.

Use two thresholds:

- low threshold for ROI boosting,
- high threshold plus persistence for alerting.

### 6.4 Persistent safety zones

Some tiles should always get more attention:

- bucket tip / implement contact zone,
- rear path when reverse is active,
- lower center for ground hazards,
- any configured blind spot.

These are small enough to reserve in the image budget.

---

## 7. Pixel shrinking options

### 7.1 The boring baseline: small WebP/JPEG tiles

Use WebP if available on the X8 image; otherwise JPEG. At 32x32 or 64x64 tile sizes, encoder overhead matters, so measure both.

Rules:

- low-quality non-ROI tiles,
- higher-quality ROI tiles,
- grayscale mode under weak link,
- bounded max tile payload,
- no retransmit during active tele-op unless the tile is keyframe-critical.

### 7.2 Y-only plus occasional color reference

Transmit luminance most of the time. Send a color reference frame or color reference tiles occasionally.

Base recolorization options:

| Method | Base compute | Trust level | v25 status |
|---|---:|---|---|
| Lab color transfer | very low | low/medium | good first add-on |
| optical-flow chroma propagation | moderate | medium/high for slow scenes | strong target |
| small exemplar colorization CNN | high | medium; can fail weirdly | research |

This saves bandwidth and gives operators a more legible display, but never use recolored imagery for crop-health measurement. Crop-health runs on raw tractor frames.

### 7.3 Progressive image layers

Send the image in layers:

1. tiny grayscale global base layer,
2. color reference or chroma hints,
3. ROI detail tiles,
4. edge/contour layer,
5. optional residual layer if budget allows.

The base can show something useful after layer 1, then improve it over time. This matches lossy LoRa better than a single all-or-nothing image blob.

### 7.4 Edge/vector transmission

Sometimes edges are more useful than pixels. The tractor can run Canny/LSD/contour extraction and transmit line segments:

```text
class_hint, x1, y1, x2, y2, confidence
```

Use cases:

- bucket edge,
- ditch/furrow line,
- row centerline,
- fence/post edges,
- implement outline.

The browser draws crisp vector overlays over a stale/low-res image. This can communicate geometry in tens of bytes.

### 7.5 Keyframe plus motion vectors

Classic video codecs spend lots of bits on motion vectors and residuals. LifeTrac can do a simpler version:

- keyframe every N seconds,
- per-tile motion vector field at low precision,
- residual only for ROI tiles,
- base warps old canvas forward.

This is especially useful when the tractor is moving steadily and the ground texture scrolls through the image.

### 7.6 Semantic maps

Instead of pixels, transmit class maps:

- sky,
- vegetation,
- bare soil,
- tractor/implement,
- obstacle,
- person/animal/vehicle,
- unknown.

A 20x15 grid with 3-bit classes is only 900 bits raw, and RLE often makes it far smaller. The base renders it as a clean situational display. This is not camera truth, so label it as synthetic.

---

## 8. Public open-source AI models that may fit

This section separates "fits on tractor X8," "fits on base X8," and "does not fit unless externally accelerated." Exact performance must be benchmarked on the actual Yocto image with the selected runtime.

### 8.1 Runtimes to evaluate

| Runtime | Why use it |
|---|---|
| TensorFlow Lite + XNNPACK | strong ARM CPU support, lots of tiny model examples |
| ONNX Runtime Mobile | portable model format, good for exported PyTorch models |
| ncnn | very good ARM inference, popular for YOLO/Real-ESRGAN/RIFE ports |
| OpenCV DNN | simple deployment for classical CV + some DNNs |
| TFLite Micro on H747 | only for tiny health classifiers, not real vision |

### 8.2 Tractor-side detector candidates

Good first targets:

| Model family | License notes | Why it fits | Suggested use |
|---|---|---|---|
| NanoDet-Plus | Apache-2.0 | small, fast, ncnn-friendly | primary detector candidate |
| YOLOX-Nano | Apache-2.0 | good accuracy/speed, permissive | alternate detector |
| PP-PicoDet | Apache-2.0 | very small, fast | obstacle/person sidecar |
| EfficientDet-Lite0 | Apache-2.0 | well-known TFLite path | robust baseline |
| SSDLite MobileNetV2/V3 | Apache-2.0 | common TFLite examples | easiest first proof |
| Ultralytics YOLOv8n/YOLOv5n | AGPL/GPL-style obligations | technically fits, license needs review | avoid unless OSE accepts terms |

Target classes for v25:

- person,
- animal,
- vehicle,
- large obstacle,
- bucket/implement,
- hitch/attachment,
- row/furrow/path if segmentation is added.

Start from COCO/OpenImages for people/vehicles/animals, then collect a small OSE dataset for tractor-specific classes. OpenImages includes many rural objects, but actual LifeTrac footage will matter more than model choice.

### 8.3 Tractor-side segmentation candidates

Segmentation can be more useful than detection for "where can I drive?" and "what is vegetation vs soil?"

Candidates:

| Model family | Fit | Use |
|---|---|---|
| Fast-SCNN | likely fits at low resolution | free-space / soil / vegetation masks |
| ENet | likely fits | very fast semantic mask baseline |
| BiSeNetV2 small variants | maybe fits | better segmentation, more tuning |
| DeepLabV3-MobileNetV3 | maybe fits on base, slower on tractor | higher-quality masks |
| Tiny U-Net custom | fits if small | change mask, crop-health mask, implement mask |

Recommendation: do not start with full semantic segmentation. Start with tile-delta and detection sidecars. Add segmentation when the dataset exists.

### 8.4 Base-side enhancement candidates

These are more reasonable on the base than the tractor:

| Model/tool | License/availability | Fit on base X8 | Use |
|---|---|---|---|
| FSRCNN/ESPCN/OpenCV super-resolution | open implementations | fits | first SR baseline |
| Real-ESRGAN small/ncnn variants | open, BSD-style for repo | possible but slow CPU | higher-quality SR if benchmark passes |
| SwinIR lightweight | open research code | maybe slow | compare only after baseline |
| RIFE-tiny | MIT | possible at small res | interpolation/prediction smoothing |
| FILM small | Apache-2.0 | maybe slow | interpolation alternative |
| OpenCV inpainting Telea/Navier-Stokes | BSD/OpenCV | fits | honest dropped-tile fill baseline |
| LaMa/Fourier inpainting | Apache-2.0 repo | probably heavy CPU | optional if accelerator exists |
| MiDaS-small | MIT | low-rate possible | depth overlay, 2.5D display |

Do not make Real-ESRGAN/RIFE/MiDaS blocking requirements. Make them optional enhancers that turn off automatically when CPU is busy.

### 8.5 Models that are too large for onboard v25

Avoid promising these on the Portenta X8 alone:

- Segment Anything full models,
- GroundingDINO,
- Stable Diffusion or SD-Turbo,
- large diffusion inpainting,
- large world models such as GAIA/Cosmos/Sora-class systems,
- full commaVQ/commaCarSim without distillation,
- desktop-size Real-ESRGAN/BasicVSR/RVRT at high resolution.

They may be useful on a separate laptop/GPU at the operator station, but they are not Portenta-native v25 features.

### 8.6 Custom tiny models are the most promising

The best-fitting models are probably not general public weights. They are small public architectures trained on OSE data:

- tiny autoencoder for 64x64 or 96x64 farm images,
- tiny U-Net for changed-tile masks,
- NanoDet/YOLOX fine-tuned for person/animal/obstacle/implement,
- simple MobileNetV3 classifier for lens occlusion, dust, darkness, "interesting frame" scoring,
- small segmentation model for soil/vegetation/sky/implement.

The architecture can be public and open-source. The weights become useful after LifeTrac-specific footage exists.

---

## 9. Base-station enhancement stack

### 9.1 The base canvas

For each camera stream, base stores:

- latest full keyframe sequence,
- tile pixels,
- tile age,
- tile confidence,
- detection sidecars,
- semantic masks,
- color reference,
- display history for temporal smoothing.

This is the heart of the system. Without a persistent canvas, every LoRa image must stand alone, which wastes bandwidth.

### 9.2 Enhancement order

Recommended order after receiving chunks:

1. patch canvas,
2. mark tile age/confidence,
3. recolorize if Y-only,
4. denoise/deblock,
5. upscale,
6. draw vector/detection overlays,
7. apply tile fade/smoothing,
8. render badges and staleness.

Never run safety or crop-health decisions on base-enhanced pixels. Those are display products.

### 9.3 Prediction and interpolation

Base can create predicted frames between real updates. Use this only as display smoothing:

- optical-flow extrapolation first,
- RIFE-tiny optional,
- distilled world-model later,
- reset to real received data whenever it arrives,
- show PREDICTED badge and prediction age.

Prediction should never replace real frame transmission. It fills the human-display gap.

### 9.4 Detection cross-check

If tractor sends detections, base can run a second detector on the reconstructed canvas or last raw thumbnail.

Use this for:

- UI warning when base sees a likely person/vehicle/animal,
- logging model disagreement,
- choosing which tile to request next,
- dataset mining for future retraining.

It should not be the safety authority. Tractor-local IPC to H747 is the path for future vision stop.

### 9.5 Synthetic/digital-twin overlay

The base can render a synthetic situational view using:

- tractor pose,
- heading/speed,
- bucket/valve state,
- detections,
- free-space mask,
- stored map/satellite tile,
- simple CAD/digital twin geometry.

This may be more useful than bad video when LoRa is weak. It should be clearly labeled SYNTHETIC and shown alongside the last raw/enhanced thumbnail, not instead of it.

---

## 10. Outside-the-box options worth exploring

### 10.1 Far field from map, near field from LoRa

Use GPS to place the tractor on a cached aerial/satellite/base-map view. Use LoRa image data only for the near field: bucket, ground, obstacles, hitch, rear path. This gives the operator context without sending far-field pixels.

### 10.2 Procedural background textures

If segmentation says a tile is "grass" or "soil" and no hazard is present, the base can render a procedural texture instead of demanding fresh pixels. This makes non-background objects stand out and costs zero LoRa bytes. It is synthetic, so label it.

### 10.3 Vector-only implement mode

For loader/hitch work, the operator may need geometry more than color. Send:

- bucket edge line,
- hitch triangle,
- ground contact line,
- obstacle bbox,
- distance/bearing estimate.

The UI draws crisp vectors over a stale image. This can feel more responsive than a blurry thumbnail.

### 10.4 "Ask a visual question" UI

Instead of passively streaming, let the operator ask for specific visual data:

- "show rear hitch detail,"
- "refresh bucket tile,"
- "is there a person ahead?"
- "send full keyframe now,"
- "increase implement ROI for 10 seconds."

Each request is tiny. The response is targeted. This fits LoRa better than continuous video thinking.

### 10.5 Multi-camera attention scheduler

If front, rear, and implement cameras all exist, do not round-robin equally. Allocate P3 budget by task:

| Mode | Budget idea |
|---|---|
| forward driving | front 80%, implement 15%, rear 5% |
| reverse | rear 80%, implement 15%, front 5% |
| loader work | implement 60%, front 35%, rear 5% |
| parked | event-driven only |
| detected person/animal in any camera | that camera gets priority for next few refreshes |

### 10.6 Event-first image transmission

If nothing changed and no operator input is active, send no image. Send a 1-byte image heartbeat saying "camera healthy, scene unchanged." The base can keep showing the last canvas with a fresh health indicator.

### 10.7 Train from LifeTrac footage

After 10 hours of recorded operation, LifeTrac has a valuable dataset. Use it to:

- fine-tune detector classes,
- train a changed-tile mask model,
- train a tiny autoencoder,
- fine-tune a super-resolution model for tractor/field imagery,
- mine false positives and false negatives.

Dataset collection is the highest-leverage v25 investment for future AI.

---

## 11. Safety and truthfulness rules

Every displayed visual must carry provenance.

Use badges:

- RAW: direct received image/tile.
- ENHANCED: deblocked/upscaled/contrast-adjusted.
- RECOLORIZED: luminance received, chroma reconstructed.
- PREDICTED: frame or tile extrapolated from prior data.
- SYNTHETIC: rendered from semantic/map/digital-twin data.
- STALE: tile/image older than freshness threshold.

Rules:

1. Never hide tile age.
2. Never let synthetic/enhanced imagery look like raw truth.
3. Always keep local raw tractor recording for incident review.
4. Do not use base-enhanced pixels for crop-health measurements.
5. Do not use LoRa-delivered image frames as the safety authority for obstacle stop.
6. Future vision-stop decisions must happen tractor-local and fail safe through H747/PSR logic.

---

## 12. Recommended v25 implementation path

### Phase A: deterministic image foundation

1. Define `ImageChunk` and `ImageDeltaObject` payload format.
2. Implement keyframe plus tile-delta encoder on tractor X8.
3. Implement persistent base canvas.
4. Add per-tile age, stale tint, and completeness percent.
5. Add ROI quality allocation from control mode.
6. Keep all image chunks P3 and bounded by LoRa grant size.

### Phase B: base-side enhancement without ML dependency

1. Y-only frames plus occasional color references.
2. Lab color transfer baseline.
3. Optical-flow chroma propagation if CPU allows.
4. OpenCV denoise/deblock/sharpen.
5. Browser-side per-tile fade transitions.
6. Operator-cued ROI requests.

### Phase C: small public models

1. Benchmark TFLite, ONNX Runtime, ncnn on the actual X8 image.
2. Deploy one permissively licensed small detector: NanoDet-Plus, YOLOX-Nano, PP-PicoDet, EfficientDet-Lite0, or SSDLite MobileNet.
3. Send binary detection sidecars, not JSON.
4. Use detections for ROI boost and UI overlays.
5. Log all frames/detections for dataset improvement.

### Phase D: base-side advanced display

1. Test FSRCNN/ESPCN first for super-resolution.
2. Test RIFE-tiny or optical-flow extrapolation for display smoothing.
3. Add base-side detector cross-check.
4. Add optional depth/2.5D display if useful.
5. Keep all generated pixels labeled.

### Phase E: learned compression research

1. Capture 10+ hours of LifeTrac footage with control/IMU/GPS synchronized.
2. Train tiny autoencoder and changed-tile model.
3. Compare against WebP/JPEG/SSDV/AVIF/JPEG XL on real footage.
4. Only promote if it beats tile-delta WebP at the same trust level.

---

## 13. Protocol additions to consider

Do not rush these into the wire format until the LoRa timing issue is settled. When ready, consider:

| Item | Priority | Purpose |
|---|---|---|
| `topic 0x25 video/image_chunk` | P3 | generic chunk stream for keyframes/deltas |
| `topic 0x26 video/detection_sidecar` | P2/P1 event | detections and ROI hints from tractor |
| `topic 0x27 video/image_status` | P2 | camera health, image seq, dropped chunks |
| `CMD_VIDEO_REFRESH_KEYFRAME` | P0/P1 | base requests clean resync |
| `CMD_VIDEO_SET_ROI` | P0/P1 | operator-cued ROI |
| `CMD_VIDEO_QUALITY` | P1 | force visual ladder rung |
| `CMD_VIDEO_REQUEST_COLOR_REF` | P1 | base asks for new color reference |

Avoid adding a topic per camera per subtype. Use `stream_id` inside the image payload so the protocol does not explode as cameras are added.

---

## 14. Validation plan

Bench tests:

1. Replay recorded tractor video through the encoder at fixed P3 byte budgets.
2. Measure keyframe rate, delta rate, bytes per useful update, and dropped chunks.
3. Force packet loss and verify stale-tile behavior.
4. Verify image flood causes zero P0/P1 misses.
5. Benchmark each model on the real X8 image.
6. Compare browser rendering load on laptop/tablet.

Field tests:

1. Parked scene: verify event-first suppression and camera health heartbeat.
2. Slow loader work: verify implement ROI refreshes faster than background.
3. Forward driving: verify path ROI and tile-delta cost.
4. Reverse: verify rear camera budget switch.
5. Person/animal/obstacle test: verify detection sidecar and UI alert.
6. Dust/vibration: verify fallback to keyframes and no runaway delta cost.

Metrics:

- median and p95 bytes per display update,
- median and p95 image age per ROI tile,
- stale tile percent,
- detection sidecar latency,
- P0 missed windows during image stress,
- base CPU load,
- tractor CPU load,
- operator subjective usefulness.

---

## 15. Option review and top recommendation

After reviewing the current video plan, LoRa timing analysis, Gemini image note, Claude image note, and the video-compression research folder, the options fall into three groups: shippable now, useful but optional, and research/deferred.

### 15.1 Options reviewed

| Option | Strength | Weakness | v25 verdict |
|---|---|---|---|
| Whole-frame WebP/JPEG thumbnail | Simple, easy to debug, preserves raw pixels | Re-sends static background and burns P3 budget quickly | Keep as baseline/fallback only |
| SSDV/progressive slideshow | Robust over loss; partial images are possible | Slow, still whole-image oriented | Good proof-of-life fallback |
| Keyframe plus tile deltas | Best fit for mostly-static farm scenes | Needs persistent state and resync logic | Primary transport recommendation |
| ROI rings and operator-cued ROI | Spends bytes where the operator is working | Bad ROI policy can hide context | Ship early |
| Y-only plus color reference | Real bandwidth saving, base can recolorize | Reconstructed color is not measurement truth | Good Phase B enhancement |
| Motion-vector microframes | Smooths sparse visual updates cheaply | Predicted, not raw truth | Optional degraded-link layer |
| Wireframe/edge vectors | Extremely low bandwidth geometry | Not a normal visual view | Emergency degraded mode |
| Semantic maps/class grids | Tiny payloads and useful for navigation | Synthetic, model-dependent | Defer until detector/segmenter is proven |
| Detection sidecars | Best safety value per byte | Depends on model recall and calibration | High-value AI add-on |
| Base super-resolution/deblocking | Large perceived quality gain | Must be labeled ENHANCED; CPU can be tight | Optional, benchmark first |
| Base independent detector | Catches tractor model misses, mines dataset | CPU/accelerator budget needed | Defend if schedule allows |
| Background cache/procedural background | Avoids re-sending static context | Can hide stale/generated pixels if unlabeled | Cache is preferred; procedural is later |
| Learned autoencoder/latent codec | Highest long-term compression potential | Needs LifeTrac dataset and validation | v26 research |
| Digital twin/synthetic scene | Can be very low bandwidth | Needs pose/implement sensors and clear labels | Complement only, not primary v25 view |
| Multi-camera adaptive allocation | Matches attention to task | Needs stable single-camera path first | Add after baseline works |
| Event-driven no-refresh mode | Frees channel when scene is quiet | Must show image age honestly | Ship with canvas/status system |

### 15.2 Top recommendation: Smart-Delta plus semantic safety

The best v25 image-management stack is:

1. **Wire transport:** hybrid keyframe plus tile-delta image objects over P3, with a hard fragment-airtime cap from the LoRa scheduler. Do not allow image chunks to block P0/P1.
2. **Tractor preprocessing:** downsample, run cheap global image registration, compute SAD/pHash tile deltas, and choose keyframe vs delta based on estimated byte cost.
3. **ROI policy:** three-level ROI rings driven by control state, active camera, and operator click. Later, detected person/animal/vehicle/obstacle tiles promote to the highest ring.
4. **Base reconstruction:** persistent per-camera canvas with per-tile age, confidence, stale tint, completeness percent, and explicit resync/keyframe request behavior.
5. **Browser rendering:** canvas patching, per-tile fade transitions, vector overlays, badges, and raw/enhanced toggle run in the operator browser where possible.
6. **First AI layer:** permissively licensed small detector on the tractor, preferably NanoDet-Plus, YOLOX-Nano, PP-PicoDet, EfficientDet-Lite0, or SSDLite MobileNet. Send binary detection sidecars, not JSON.
7. **Base-side assist:** deterministic OpenCV enhancement first; then benchmark small super-resolution and independent base detection. If a USB accelerator is later validated, use it on the base only, not the tractor.
8. **Truth policy:** every non-raw pixel is labeled. Use RAW, ENHANCED, RECOLORIZED, PREDICTED, SYNTHETIC, CACHED, and STALE badges as appropriate.

This is the top recommendation because it preserves real visual evidence, scales down cleanly under weak RF, protects the control channel, and does not depend on a custom training pipeline before the tractor can be tested.

### 15.3 Fallback ladder

Use link health and scheduler pressure to move automatically between modes:

| Link state | Mode |
|---|---|
| Healthy P0/P1 and spare P3 | Tile-delta + ROI rings + base canvas + enhancement |
| Moderate pressure | ROI-first tile deltas, lower cadence, fewer context tiles |
| Weak link | Y-only ROI or SSDV/progressive proof-of-life |
| Very weak link | Detection sidecars + motion-vector or wireframe degraded mode |
| Control-only | No image chunks; show frozen canvas with age clock and alerts only |

The ladder should drop down immediately when P0/P1 deadlines are threatened and climb back up only after several clean refresh windows. This prevents visual traffic from making the control link unstable.

### 15.4 What not to lead with

Do not lead v25 with learned latent codecs, large generative reconstruction, full digital-twin replacement, or semantic-only synthetic views. They are interesting and worth preserving as v26 research, but they are harder to validate than tile deltas and easier for the operator to over-trust.

Final call: build the v25 image path around **real pixels first, targeted deltas second, semantic alerts third, and base/browser enhancement fourth**. That ordering gives the best safety, bandwidth efficiency, implementation confidence, and operator trust.

---

## 16. Bottom line

Use LoRa for visual state, not video.

The practical v25 answer is a smart patch stream: keyframes, tile deltas, ROI quality, scene-change suppression, Y-only plus color references, and semantic sidecars. The tractor X8 should shrink and prioritize. The base X8 should remember, enhance, cross-check, and explain. The browser should render the visual truth honestly.

Public open-source AI can help, but the fit is narrow. Small detectors, small segmenters, optical flow, tiny super-resolution, and custom autoencoders are realistic. Foundation models and desktop-grade video generation are not Portenta-native. The most valuable thing v25 can do for future AI is log real LifeTrac footage and build the dataset.

Treat tile-delta plus persistent base canvas as the baseline image pipeline. Treat AI as a saliency and enhancement layer around it, not as the thing that makes the radio magically wide.

---

End of document v1.0.
