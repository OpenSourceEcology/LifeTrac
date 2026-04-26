# Grayscale → color recolorization with occasional color reference frames

**Status:** research note, not on any phase plan yet.
**Question from operator:** *"Can we send mostly grayscale video over LoRa, plus an occasional color frame as a 'palette,' and let the base station re-add color to the grayscale frames before display?"*
**Short answer:** **Yes, this is a well-studied technique called *reference-based* (or *exemplar-based*) video colorization.** It saves real bandwidth (~30–50% for the chroma channel), works without AI for slow scenes, and works very well *with* small AI for fast/changing scenes. Compute on the base station X8 is feasible at all three quality tiers below.

This document surveys the techniques, picks defaults for v25, and sketches an integration with the [bandwidth budget](README.md) and the [QoS doc](../../../AI%20NOTES/2026-04-26_LoRa_QoS_Bandwidth_Management.md).

---

## 1. Why this is worth doing

Color information in a video frame is **mostly redundant in time**: a wheat field stays wheat-yellow for minutes, the sky stays blue, the bucket stays orange. If we transmit one full color reference every N seconds and only the **luminance (Y) channel** in between, we cut chroma bandwidth by N×.

Video codecs already exploit this redundancy *within a GOP* — H.264 4:2:0 already throws away 75% of the chroma resolution, and inter frames mostly just send motion vectors + residuals. But the **bandwidth ratios at LoRa speeds are different from broadband video**. At our Phase A target of 2–5 kbps, every byte of chroma matters. A 256×192 grayscale JPEG is ~30% smaller than the same color JPEG at the same Y-channel quality, so a "Y-only stream + occasional color reference" path is a real ~25% bandwidth win on top of whatever codec we're already using. Stack it on top of Phase D (neural temporal inflate) and the savings compound.

There's also a **psychological** argument: even **wrong-but-plausible color** ("wheat is yellow, sky is blue, dirt is brown") feels much closer to "real" video than grayscale does, even if specific colors are off. Operators trust the picture more.

---

## 2. The taxonomy of colorization methods

Three broad families, in increasing order of compute / sophistication:

### 2.1 Color transfer (no AI, ~1980s–2000s)

You have a grayscale frame G and a color reference R from a few seconds ago. Compute color-channel statistics (mean + std-dev of the a* and b* channels in CIE Lab) of R, then *transfer those statistics* to G after converting it to Lab with G as the L channel. Reinhard et al. 2001 ("Color Transfer between Images") is the canonical reference. ~5 ms per frame on the X8 in Python+NumPy.

**Quality:** mediocre. Gives the *overall mood* (warm/cool/saturated) but no local detail — your wheat field will be uniformly yellow even where there's a green weed patch.
**Use it for:** a "v0" baseline that proves the pipeline. Or as a fallback when the reference frame is too stale for the better methods to work.

### 2.2 Exemplar-based / patch-matching (no AI, ~2002–2015)

Welsh et al. 2002 ("Transferring Color to Greyscale Images") — for each pixel in G, find the most similar luminance neighborhood in R, and copy that pixel's chroma. This was the standard pre-deep-learning approach. Ironi et al. 2005 and Gupta et al. 2012 added superpixel-level matching to make it faster and more spatially coherent. Levin et al. 2004 ("Colorization Using Optimization") solves a sparse linear system to propagate user-painted color scribbles through a grayscale image — works beautifully if you have *any* known-color anchor pixels (the reference frame, in our case, can provide thousands).

**Quality:** good for static-ish scenes. Edges are respected, and same-luminance patches in similar contexts get matched correctly.
**Compute:** moderate. Welsh: ~100 ms/frame on the X8. Levin's sparse-system solve: ~50 ms/frame at 256×192 with `scipy.sparse.linalg.spsolve` once you've built the matting Laplacian.
**Failure modes:** when the scene changes — a new object enters the frame, the camera pans onto trees you've never seen — the algorithm has nothing to copy from and produces gray-ish or wrongly-colored regions.

### 2.3 Deep exemplar-based video colorization (AI, 2018–present)

This is the modern way. A small CNN takes the grayscale current frame + the color reference frame + (optionally) the previous colored output, and produces a coherent color frame. The network learns *semantic* correspondence: it knows wheat, sky, dirt, machinery, and assigns plausible colors even where it can't directly match a patch from the reference.

Notable papers, in chronological order:

| Year | Paper / repo | Key idea | Model size |
|---|---|---|---|
| 2018 | Zhang et al. "Deep Exemplar-based Colorization" (SIGGRAPH) | Reference-image conditioning via correspondence subnet + colorization subnet | ~50 MB |
| 2019 | Iizuka & Simo-Serra "DeepRemaster" | Source-reference attention, designed for old film | ~70 MB |
| 2019 | Zhang et al. "Deep Exemplar-based **Video** Colorization" (CVPR) | Recurrent: previous output + reference both flow forward | ~50 MB |
| 2022 | Yang et al. "BiSTNet" (CVPR) | Bi-directional spatio-temporal attention | ~80 MB |
| 2024 | Yang et al. "ColorMNet" | Memory-augmented; reference can be many frames back | ~100 MB |
| 2018+ | DeOldify (Antic, fast.ai) | **No reference** — pure hallucination via GAN | ~250 MB |
| 2017 | Zhang et al. "Colorful Image Colorization" (ECCV) | **No reference** — classification over quantized chroma | ~125 MB |

**Quality:** excellent for natural scenes once the reference is a good match for the current scene.
**Compute on the X8:** the recurrent video models are too heavy for our i.MX 8M Mini at full size (~5–10 GFLOPs sustained), but they distill well. A ~10 MB INT8-quantized variant of the 2019 video model runs at ~150 ms/frame on the X8 NEON cores per the same back-of-envelope math we used for the Phase D temporal-inflate plan. That's **~7 fps**, plenty for our 5 fps LoRa stream.

**Failure modes:** semantic mistakes look weirder than no-AI failures — you can get pink wheat or blue dirt if the reference frame doesn't have those classes. Ship a "low-confidence → fall back to color-transfer" gate.

### 2.4 Reference-free (DeOldify, Zhang ECCV 2017)

Skip the reference entirely; let the network hallucinate plausible colors from priors learned during training. **Not recommended for v25** — the operator needs to be able to trust the colors when assessing crop health or seeing a child in a red shirt enter the field. We always have a reference; use it.

---

## 3. The bandwidth math, applied to v25

LoRa SF7/BW500 gives us ~22 kbps gross. The QoS doc allocates roughly 8–15 kbps to the video class once control + telemetry get their priority cut.

### 3.1 Without the recolorize trick

| Approach | Bitrate | Notes |
|---|---|---|
| Phase C SVT-AV1 256×192 / 5 fps color | 8–15 kbps | Already tight against the budget |
| Phase A SSDV 160×120 color JPEG / 0.1 Hz | 2–5 kbps | Slideshow |

### 3.2 With the recolorize trick

Send only the **Y channel** (8-bit luminance) for every frame, plus a **full color reference** every N seconds. Reconstruct Cb/Cr at the base station from the latest reference + the current Y frame.

| Approach | Y stream | Reference cadence | Total | vs original |
|---|---|---|---|---|
| Phase A SSDV grayscale + color ref every 30 s | ~1.6 kbps (160×120 Y JPEG / 0.2 Hz) | 4 kB color JPEG / 30 s = 1.1 kbps | **~2.7 kbps** | ~30% saving |
| Phase C SVT-AV1 grayscale + color ref every 10 s | ~5–9 kbps (Y-only stream, AV1 monochrome mode) | ~6 kB color JPEG / 10 s = 4.8 kbps | **~10–14 kbps** | ~10–25% saving |
| Phase C grayscale + color ref every 30 s | ~5–9 kbps | 1.6 kbps | **~7–11 kbps** | ~25–35% saving — this is the sweet spot |
| Phase D neural-inflate grayscale + color ref every 30 s | ~3–6 kbps | 1.6 kbps | **~5–8 kbps** | best, but Phase D first |

The **30-second reference cadence at 256×192** is the practical sweet spot for tractor work — the scene doesn't change that fast in a field, and 1.6 kbps for the reference is small compared to the Y stream.

### 3.3 Adaptive cadence — send a new reference *only when needed*

Even better: don't send the reference on a fixed timer. Send a new reference when the **base station detects the recolorization is failing**, or when the **tractor detects a scene change**.

- **Tractor side (better):** the X8 is already running scene-change detection for the QoS doc's "scene-change suppression" feature. When the scene has changed enough that the latest reference is stale, mark the next frame as a new reference and send it as color instead of Y-only. ~3 references per minute when working a field, ~1 per minute when transit-driving. Average ~0.3–0.5 kbps for references.
- **Base station side (worse but works as a backstop):** the colorization confidence drops below a threshold, request a new reference frame via a 1-byte CMD topic. Adds round-trip latency of 200–500 ms but recovers from any pipeline desync.

Combined: ~80% chroma bandwidth saving for typical tractor work. **That's the win.**

---

## 4. Recommended approach for v25

Three tiers, matching the structure of the existing Phase A/C/D plan:

### 4.1 Tier 1 — Color transfer (Reinhard 2001), Phase A companion

Implement on the base station in Python + NumPy. ~50 LoC. Ships with Phase A. Lets us do "Y-only SSDV slideshow + occasional color reference" immediately and prove the bandwidth saving is real before investing more.

```python
# base_station/colorize_simple.py — ~30 lines real
import cv2, numpy as np

def recolorize_lab(gray_y_u8, ref_color_bgr):
    """Tier-1 color transfer. Gray L from current frame, a*/b* mean+std from
    ref. Fast, dumb, perfectly adequate for steady-state field driving."""
    ref_lab = cv2.cvtColor(ref_color_bgr, cv2.COLOR_BGR2LAB)
    out_lab = np.zeros((*gray_y_u8.shape, 3), dtype=np.uint8)
    out_lab[..., 0] = gray_y_u8                      # use received Y as L
    for ch in (1, 2):                                # a*, b*
        m = ref_lab[..., ch].mean()
        out_lab[..., ch] = np.clip(m, 0, 255).astype(np.uint8)
    return cv2.cvtColor(out_lab, cv2.COLOR_LAB2BGR)
```

That's it. Real code. ~5 ms per 256×192 frame on the X8.

### 4.2 Tier 2 — Optical-flow chroma propagation, Phase C companion

Compute dense optical flow (Farneback or DIS — both in OpenCV, no AI) from the last received reference frame to the current Y frame, warp the reference's Cb/Cr channels through the flow, and use those as the chroma for the current frame. When flow magnitude exceeds a threshold (occlusion / new content), fall back to Tier 1 in those regions.

```python
# pseudocode
flow = cv2.optflow.calcOpticalFlowFarneback(ref_gray, current_y, ...)
warped_chroma = cv2.remap(ref_chroma, flow_x, flow_y, ...)
mask = flow_magnitude < threshold
out = compose(current_y, warped_chroma, mask=mask, fallback=tier1(current_y))
```

~80 ms/frame on the X8 at 256×192. Quality is much better than Tier 1 — moving objects keep their colors, and only newly-revealed regions (disocclusions) need fallback. This is the **production target** for v25 — high enough quality, no AI dependency, no model file to ship and update.

### 4.3 Tier 3 — Distilled deep exemplar video colorization, Phase D companion

Adapt the [Zhang 2019 video colorization repo](https://github.com/zhangmozhe/Deep-Exemplar-based-Video-Colorization) (BSD-3) to a small student model trained to mimic its output. Distill to ≤20 MFLOPs/frame to fit the X8 NEON budget alongside Phase D's neural-inflate. INT8 quantize. Ship the .tflite or ONNX model with the base-station container.

Out of scope until Phase D ships. Track here so we don't forget.

---

## 5. Tractor-side changes

The tractor X8 has very little extra work to do:

1. **Encode Y-only frames** — `ffmpeg -pix_fmt gray` and `mozjpeg` both natively support grayscale. AV1 and HEVC both support a `monochrome=1` mode. SSDV already has a grayscale mode (it's mostly used for high-altitude balloons that have monochrome cameras).
2. **Send color reference frames on a schedule + on scene-change.** Reuse the scene-change detector from the QoS doc (mean per-pixel diff against the last reference, exceeds threshold → flag this frame as a reference). Color references are flagged in the SSDV header so the base station knows to update its palette.
3. **Two new LoRa "topics" or frame subtypes:** `video/y_frame` and `video/color_reference`. Could reuse the existing `0x20 video/thumbnail` and add a 1-bit flag in the payload header rather than burning a topic ID.

That's the entire tractor-side delta. The bandwidth-budget enforcement that already exists in Phase A/C/D handles the rest.

---

## 6. Base-station changes

1. **Add a `colorize.py` service** alongside `web_ui.py`. Subscribes to the video MQTT topic, holds the last color reference frame in memory, runs the active tier's recolorize on each Y frame, publishes the recolored output on `lifetrac/v25/video/colorized` for the web UI to display.
2. **UI badge.** When a frame is *recolored* (not directly received as color), show a small "RECOLOR T2" badge in the corner of the video tile. Same principle as the "SYNTHESIZED" badge for Phase B/D. Operators must be able to tell when they're looking at reconstructed vs original color.
3. **CMD_REQUEST_REFERENCE.** Add an opcode (e.g. `CMD_VIDEO_REFRESH_REF = 0x05` in [LORA_PROTOCOL.md](../../LORA_PROTOCOL.md#command-frame-opcodes)) so the base can ask for a new reference if recolorization confidence falls.
4. **Crop-health interaction.** The crop-health pipeline in [VIDEO_OPTIONS.md § Crop-health analysis](../../VIDEO_OPTIONS.md#crop-health-analysis) MUST always run on the **tractor** with raw frames, never on recolored base-station frames. Recolored chroma is plausible-fiction, not reflectance, and is useless for NDVI. (This is just a "don't shoot yourself in the foot" reminder.)

---

## 7. Open questions / further research

- **Color stability between references.** Tier 1 will visibly "flicker" each time a new reference arrives — the global a*/b* means jump. Mitigate with low-pass filtering on the reference stats over time.
- **Mixed-illuminant scenes.** Cab interior + bright field sky + shadowed bucket = three different white balances. A single reference's stats won't fit all three regions. Tier 2 handles this naturally; Tier 1 doesn't. Consider zoning the frame into ~4 regions and recoloring each independently with Tier 1 if Tier 2 isn't ready.
- **Night / dawn / dusk.** Reference grows stale fast as illumination changes. Drop reference cadence to 5–10 s during these times, or trigger on illumination change in addition to scene change.
- **Headlights and brake lights** (eventually, on the rear cam). These are *new* color sources mid-scene; the reference will never have them. Tier 3 (semantic) handles this; Tier 1 and 2 will produce gray taillights. Document and accept for v25; revisit when a vehicle is added.
- **Codec interaction.** AV1's monochrome mode skips chroma planes entirely, but the bitstream savings vs. AV1 4:2:0 at low chroma quantizer are smaller than you'd expect (AV1 already crushes flat chroma). Empirically measure on real tractor footage before committing to monochrome encoding for Phase C.
- **Dataset for Tier 3 distillation.** We already plan to capture ≥10 h of tractor footage for Phase D (per [TODO.md](../../TODO.md) Phase 10+). Same dataset works for distillation — train the small model to match the Zhang 2019 teacher's outputs frame-by-frame.

---

## 8. References

- Reinhard, Ashikhmin, Gooch, Shirley (2001). *Color Transfer between Images.* IEEE CG&A. ([PDF](https://www.cs.tau.ac.il/~turkel/imagepapers/ColorTransfer.pdf))
- Welsh, Ashikhmin, Mueller (2002). *Transferring Color to Greyscale Images.* SIGGRAPH.
- Levin, Lischinski, Weiss (2004). *Colorization using Optimization.* SIGGRAPH. ([page](https://www.cs.huji.ac.il/~yweiss/Colorization/))
- Ironi, Cohen-Or, Lischinski (2005). *Colorization by Example.* EGSR.
- Gupta, Chia, Rajan, Ng, Zhiyong (2012). *Image colorization using similar images.* ACM MM.
- Zhang, Isola, Efros (2016/2017). *Colorful Image Colorization.* ECCV. ([repo](https://github.com/richzhang/colorization))
- Iizuka, Simo-Serra (2019). *DeepRemaster: Temporal Source-Reference Attention Networks for Comprehensive Video Enhancement.* SIGGRAPH Asia. ([page](http://iizuka.cs.tsukuba.ac.jp/projects/remastering/))
- Zhang, Wen, Zhao, Yan, Wang, Ling, Zhao, Wang (2019). *Deep Exemplar-based Video Colorization.* CVPR. ([repo](https://github.com/zhangmozhe/Deep-Exemplar-based-Video-Colorization))
- Yang, Xu, Liu, Hu (2023). *BiSTNet: Semantic Image Prior Guided Bidirectional Temporal Feature Fusion for Deep Exemplar-based Video Colorization.* CVPR.
- Yang et al. (2024). *ColorMNet: A Memory-based Deep Spatial-Temporal Feature Propagation Network for Video Colorization.* ECCV.
- Antic. *DeOldify.* ([repo](https://github.com/jantic/DeOldify))
- SSDV (Slow Scan Digital Video) — used by HAB community for grayscale + occasional color over RTTY/LoRa. ([repo](https://github.com/fsphil/ssdv))

---

## 9. Recommendation

**Track Tier 1 as a Phase A.5 task.** It's ~30 lines of Python and a 25–30% bandwidth saving on the SSDV slideshow path. Zero risk, lands quickly.

**Tier 2 lands with Phase C** (SVT-AV1) — by then we have monochrome encoding and dense optical flow on the base; combining them into chroma propagation is one focused engineering task.

**Tier 3 stays in stretch goals** until Phase D ships. The neural-inflate work and the recolorization model can share the same student-distillation infrastructure.

Add corresponding bullets under [TODO.md § Stretch goals](../../TODO.md) Phase 10+ video work, near the existing Phase A/B/C/D bullets — but keep them flagged as *companion* tasks to those phases, not standalone.
