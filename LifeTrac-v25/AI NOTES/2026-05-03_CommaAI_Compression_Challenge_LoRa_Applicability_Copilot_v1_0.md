# LifeTrac v25 - comma.ai Video Compression Challenge LoRa Applicability v1.0

**Date:** 2026-05-03
**Author:** GitHub Copilot
**Status:** Research note. No code changes proposed here.
**Question:** Which compression scripts and methods in `commaai/comma_video_compression_challenge`, especially PR #55 and PR #62, could help the LifeTrac LoRa image/video transmission system?

Related local context:

- [../DESIGN-CONTROLLER/IMAGE_PIPELINE.md](../DESIGN-CONTROLLER/IMAGE_PIPELINE.md) - canonical v25 LoRa image pipeline.
- [../DESIGN-CONTROLLER/LORA_PROTOCOL.md](../DESIGN-CONTROLLER/LORA_PROTOCOL.md) - air interface, topic IDs, TileDeltaFrame, fragment cap.
- [2026-04-27_Image_Transmission_InDepth_Analysis_ClaudeOpus4_7_v1_0.md](2026-04-27_Image_Transmission_InDepth_Analysis_ClaudeOpus4_7_v1_0.md) - tile-delta, ROI, base reconstruction, learned-codec discussion.
- [../DESIGN-CONTROLLER/RESEARCH-CONTROLLER/VIDEO_COMPRESSION/README.md](../DESIGN-CONTROLLER/RESEARCH-CONTROLLER/VIDEO_COMPRESSION/README.md) - previous comma.ai challenge notes.

---

## 1. Short answer

Yes, the challenge is useful, but not mainly as drop-in code.

The useful part is the **method pattern**:

1. Compress semantics rather than pixels.
2. Send ego-motion or pose side data.
3. Let a tiny learned decoder reconstruct a display image at the receiver.
4. Bias bits toward task-relevant regions and task-relevant metrics.

For LifeTrac LoRa, that maps to:

- Send **tile deltas, masks, detections, ROI hints, motion vectors, and occasional real keyframes**, not a conventional video stream.
- Keep **real safety decisions on raw tractor-side perception and telemetry**, not on hallucinated base-side reconstructions.
- Badge every reconstructed, cached, recolorized, predicted, or synthetic pixel in the operator UI.

The top PRs (#55 `quantizr`, #62 `fp4_mask_gen`) are strong evidence that semantic-mask-plus-generator codecs can beat normal codecs when the metric rewards semantics. But their actual submitted pipelines are **offline, challenge-metric-specific, GPU-oriented, and too large in per-minute payload for our current LoRa budget** unless heavily downscaled and redesigned.

---

## 2. LifeTrac LoRa constraints that matter

The current v25 image pipeline assumes:

- Image traffic is **P3 only** and must never block P0/P1/P2 traffic.
- Total image budget is roughly **800-1000 bytes per 1.5 s refresh** in the canonical plan, before real-world loss and queueing.
- The `TileDeltaFrame` path already exists conceptually on topic `0x25`.
- Image fragments must not cause **P0 ControlFrame TX-start delay > 25 ms**.
- Existing notes already call out that a 32-byte SF7/BW250 image fragment is around 36 ms, so the fragment cap still needs redesign.

That means any imported idea has to survive a brutal test: can it be broken into tiny, preemptible fragments and still provide value when most frames are dropped or stale?

---

## 3. Upstream repo structure and scripts

Main repo: <https://github.com/commaai/comma_video_compression_challenge>

Important root scripts:

| Path | Role |
|---|---|
| `evaluate.sh` | Unzips `archive.zip`, runs a submission's `inflate.sh`, then scores raw RGB output. |
| `evaluate.py` | Computes distortion and final score. |
| `frame_utils.py` | Video loading, conversion, resize helpers. |
| `modules.py` | Challenge SegNet, PoseNet, DistortionNet definitions and model paths. |
| `submissions/*/compress.sh` | Optional compressor from original videos to `archive.zip`. |
| `submissions/*/inflate.sh` | Required decompressor from `archive/` to raw RGB frames. |

Challenge metric:

```text
score = 100 * segnet_distortion + 25 * compression_rate + sqrt(10 * posenet_distortion)
```

This is a major reason the winning methods look unusual: they optimize semantic segmentation and ego-motion, not PSNR/SSIM or operator-visible photorealism.

---

## 4. PR #55 - `quantizr`

PR: <https://github.com/commaai/comma_video_compression_challenge/pull/55>

Merged files:

| Upstream path | Purpose |
|---|---|
| `submissions/quantizr/compress.py` | End-to-end extraction, mask/pose compression, generator training, model quantization. |
| `submissions/quantizr/compress.sh` | Runs `compress.py`, then stores `*.br` files into `archive.zip` without extra ZIP compression. |
| `submissions/quantizr/inflate.py` | Loads compressed model, mask video, and pose data, then generates raw RGB frames. |
| `submissions/quantizr/inflate.sh` | Shell wrapper around `inflate.py`. |
| `pyproject.toml` | Adds `brotli`. |

Reported result:

- Score: **0.33**.
- Archive size: **299,970 bytes** for the one-minute 37.5 MB challenge video.
- Compression rate: **0.00799**.
- Requires GPU for evaluation/inflation per PR.

Method summary:

1. Use SegNet during compression to create **5-class semantic masks** at 512x384.
2. Encode the mask sequence as an AV1 OBU stream, then Brotli-compress it.
3. Use PoseNet during compression to extract a **6-D pose vector** per frame pair, stored as `pose.npy.br`.
4. Train a tiny depthwise-separable generator that maps:

```text
mask2 + pose6 -> reconstructed frame1 and frame2
```

5. Store generator weights as an FP4-quantized, Brotli-compressed state dict in `model.pt.br`.
6. In `inflate.py`, decode mask/pose/model and synthesize frames at 512x384, then upsample to the original camera size.

Notable architecture pieces:

- `FP4Codebook` for packed 4-bit neural weights.
- `QConv2d`, `QEmbedding`, depthwise separable convs.
- `SharedMaskDecoder` to turn class masks plus coordinate grid into features.
- FiLM conditioning on pose vectors.
- Separate heads for the two frames in each pair.

LifeTrac relevance:

- **Very useful conceptually:** semantic masks + pose side channel + receiver-side generator.
- **Not directly shippable:** needs GPU, is trained against comma's SegNet/PoseNet, assumes offline access to the original full clip, and reconstructs hallucinated RGB.
- **Bandwidth problem:** 300 KB/min is about 40 kbps averaged over a minute. Even if the model were preinstalled and not counted every minute, the mask/pose stream is likely still above our P3 LoRa budget unless reduced drastically.

---

## 5. PR #62 - `fp4_mask_gen`

PR: <https://github.com/commaai/comma_video_compression_challenge/pull/62>

Merged files:

| Upstream path | Purpose |
|---|---|
| `submissions/fp4_mask_gen/compress.py` | Improved mask/pose/generator training and archive creation. |
| `submissions/fp4_mask_gen/compress.sh` | Runs `compress.py`, stores `model.pt.br`, `mask.obu.br`, `pose.bin.br`. |
| `submissions/fp4_mask_gen/inflate.py` | Loads FP4 model, AV1 mask stream, quantized pose stream, synthesizes frames. |
| `submissions/fp4_mask_gen/inflate.sh` | Shell wrapper around `inflate.py`. |

Reported result:

- Score: **0.37** on official eval.
- Archive size: **249,624 bytes** for the one-minute challenge video.
- Compression rate: **0.00665**.
- Requires GPU for evaluation/inflation per PR.

PR description of payload:

- AV1 CRF=55 Brotli-compressed mask video, 5-class SegNet output, 512x384, about **175 KB**.
- UInt16-quantized poses, about **7 KB**.
- FP4-quantized neural generator, about **66 KB**, around **88K params**.

Method improvements over `quantizr`:

1. Stores mask frames more efficiently.
2. Stores pose as:

```text
12 fp32 values for per-dimension min/max + N * 6 uint16 pose samples
```

3. Uses FP4 quantized depthwise-separable generator weights.
4. Uses FiLM conditioning on pose.
5. Trains against noisy round-tripped masks, so the generator learns the actual artifacts it will see at decode time.

LifeTrac relevance:

- **Most relevant upstream method** from the linked PRs.
- The pose codec is directly interesting: our equivalent side signals are steering command, wheel speed, IMU yaw, GPS velocity, hydraulic state, and camera selection.
- The tiny generator is small enough in storage terms, but the runtime still needs benchmarking and likely INT8/TFLite/ncnn conversion for the Portenta X8 CPU.
- Payload still too high for continuous LoRa: excluding the model, mask + pose is about 182 KB/min, around 24 kbps. Our canonical P3 image budget is closer to a few kbps and is preemptible.

---

## 6. Other relevant submissions and methods

| Submission | Method | LifeTrac value |
|---|---|---|
| `baseline_fast` | Basic downscale + ffmpeg encode + inflate resize. | Good baseline only. Not enough for LoRa. |
| `svt_av1_lanczos_fg`, `svtav1_*`, `av1_*` | SVT-AV1/AV1, downscale, long GOP, film grain, sharpening. | Useful for WiFi/cellular or local storage. Too continuous for LoRa except maybe parked bulk transfer. |
| `roi_v2`, `roi_gop300_c34`, `v4_qp_aq2_roi` | ROI preprocessing or per-block QP maps guided by SegNet. | Strong idea. Map to LifeTrac ROI tile budgets rather than AV1 QP maps. |
| `svtav1_dilated_ren` | AV1 plus decoder-side residual enhancement network, about 20K params. | Useful base-side enhancement pattern. Does not reduce over-air payload much by itself. |
| `neural_inflate` | AV1 plus neural postfilter/inflation. | Useful as an enhancement baseline, less radical than mask-generation. |
| `delta_codec` | Each 2-frame pair as low-res base frame plus residual frame, with ROI-aware residual quantization. | Closest classical match to LifeTrac tile-delta/P-frame work. But our tile-delta plan is more LoRa-native. |
| `codex_metric_yshift_av1` | Heavy decoder-side postprocess, inverse resize/edge/temporal tricks tuned to metric. | Some postfilter ideas useful, but mostly challenge-specific. |
| `qpose14...` / later leaderboard variants | Further packing of masks, model, pose, color LUTs, and side channels. | Watch for pose compression and preinstalled model ideas, but not a direct v25 dependency. |

---

## 7. What to borrow for LifeTrac

### 7.1 Borrow now

1. **Task-aware metrics.** Evaluate image quality by operator and detector usefulness, not pixel fidelity alone. For LifeTrac: obstacle/person/bucket/furrow visibility, stale-tile honesty, and control latency.
2. **Semantic sidecars.** Add low-rate object/region data as first-class LoRa payloads. This already fits the local `0x26 video/detections`, `0x28 motion_vectors`, `0x29 wireframe` direction.
3. **Pose/ego-motion side channel.** Use steering, IMU yaw, speed, and valve state to drive base-side prediction or tile motion. The comma methods show how valuable pose is for preserving temporal metrics.
4. **Quantized tiny models.** Preinstall small INT8/FP4-ish models on the base and maybe tractor. Do not send model weights repeatedly over LoRa.
5. **Train on round-trip artifacts.** If we use mask/tile codecs, train or tune the reconstructor on the actual post-LoRa, post-fragment-loss stream, not ideal source masks.
6. **ROI allocation.** Spend bytes on bucket, path center, obstacle boxes, and operator-tapped regions.

### 7.2 Borrow later

1. **Mask-to-RGB generator.** A v26 research path: transmit a coarse farm-scene mask plus ego-motion and let the base render a badged synthetic view.
2. **Frame-pair generator.** Similar to `quantizr`/`fp4_mask_gen`, but trained on LifeTrac footage and constrained to CPU inference.
3. **Learned motion or appearance latents.** If tile-delta saturates, send 32-128 byte latents per refresh instead of tiles.
4. **Decoder-side enhancement network.** A tiny REN/super-resolution model can improve the operator display without increasing LoRa payload.

### 7.3 Do not borrow directly

1. Full `quantizr` / `fp4_mask_gen` inflation scripts as the v25 operator display path.
2. GPU-only decompression assumptions.
3. A synthetic image without explicit UI badges.
4. Continuous AV1 as the LoRa video transport.
5. Challenge-specific use of comma SegNet/PoseNet as production dependencies.

---

## 8. Suggested LifeTrac adaptation path

### Phase A: stay with canonical tile-delta

Keep the current v25 image plan as the build path:

- WebP or similar tiny tile payloads.
- pHash/SSD tile-change detection.
- ROI quality allocation.
- P3-only preemptible fragments.
- Persistent base canvas with stale-tile ages and badges.

### Phase B: semantic LoRa sidecars

Add a very low-rate semantic stream:

```text
class_id, confidence, bbox_cx, bbox_cy, bbox_w, bbox_h, age_ms
```

Use this for UI overlays and alerting even when pixels are stale. This is more bandwidth-effective than sending another image.

### Phase C: mask microframes

Instead of full 512x384 comma masks, try very small masks:

| Mask size | Classes | Raw bits | Notes |
|---|---:|---:|---|
| 32x24 | 8 | 2304 bits = 288 B | Good first experiment. |
| 48x32 | 8 | 4608 bits = 576 B | Fits one refresh only if other image data is sparse. |
| 64x48 | 8 | 9216 bits = 1152 B | Too large unless sent rarely or compressed well. |

Compress with RLE, small dictionaries, or tile-delta on class labels. Avoid large block codecs that add latency and make preemption awkward.

### Phase D: preinstalled base generator

Train a small model on LifeTrac data:

```text
coarse mask + stale canvas + ego-motion + ROI state -> display tile or low-res frame
```

Constraints:

- CPU-only on Portenta X8 unless the optional base accelerator is installed.
- INT8/ncnn/TFLite target first; FP4 only if runtime support is practical.
- Output must be badged as `Synthetic` or `Predicted` unless it is direct received pixels.
- Raw tractor-side footage remains the incident-review ground truth.

---

## 9. Bandwidth reality check

The linked PRs achieve impressive compression ratios, but their averaged challenge payloads are still above our LoRa budget:

| Method | Reported archive | Approx avg over 60 s | LoRa assessment |
|---|---:|---:|---|
| #55 `quantizr` | 299,970 B | about 40 kbps | Too high. Concept useful. |
| #62 `fp4_mask_gen` full archive | 249,624 B | about 33 kbps | Too high. Concept useful. |
| #62 mask + pose only | about 182 KB | about 24 kbps | Still too high for continuous P3. |
| LifeTrac canonical P3 target | 800-1000 B / 1.5 s | about 4-5 kbps payload envelope | Must be preemptible and lossy. |

Therefore: use comma.ai methods to design **what information to send**, not to justify continuous video over LoRa.

---

## 10. Recommendation

For v25, do not replace the existing `IMAGE_PIPELINE.md` tile-delta plan with `quantizr` or `fp4_mask_gen`.

Instead:

1. Add #62-inspired **pose/ego-motion sidecars** to the image pipeline design.
2. Add an experiment for **tiny semantic mask microframes** as a degraded mode between tile-delta and wireframe.
3. Use the #55/#62 generator architecture as a reference for a **future preinstalled base-side synthetic renderer**, trained on LifeTrac footage.
4. Keep the operator UI honesty rules: real pixels, cached pixels, enhanced pixels, predicted pixels, and synthetic pixels must remain visibly different.

The strongest immediately usable idea is not the neural generator itself. It is the discovery that a few kilobytes of **semantic structure plus motion** can preserve the useful content of a video better than a much larger stream of generic pixels. For LifeTrac over LoRa, that should become: detections, masks, motion vectors, ROI hints, and occasional real keyframes, all under strict QoS.
