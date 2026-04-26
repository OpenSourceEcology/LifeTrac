# VIDEO_COMPRESSION — research notes

**Status:** research / brainstorming. Nothing here is on the build path. The active video plan for v25 is in [../../VIDEO_OPTIONS.md](../../VIDEO_OPTIONS.md), which currently treats LoRa as carrying *no* live video and uses WiFi / cellular / airMAX for the actual stream.

The premise of this folder: **we have a Portenta X8 on each end of the LoRa link** (tractor and base station — see [../../ARCHITECTURE.md](../../ARCHITECTURE.md)). Each X8 has a quad‑core Cortex‑A53 i.MX 8M Mini with a hardware VPU (H.264/H.265 encode+decode) and enough headroom to run small neural nets. So the question is no longer *"can LoRa carry a normal video stream?"* (it can't — see [../../VIDEO_OPTIONS.md § Why Video Is Hard on Long-Range Radio](../../VIDEO_OPTIONS.md#why-video-is-hard-on-long-range-radio)) but **"can the compute on either end of the LoRa bottleneck synthesize something useful from a tiny number of bits per second?"**

This document:

1. Restates the bandwidth budget so the targets are concrete.
2. Summarizes the comma.ai video‑compression challenge and what it teaches us.
3. Surveys existing projects in the predicted‑frame / generative‑video‑codec space.
4. Lays out compression options ranked from "boring and shippable" to "research‑grade".
5. Picks a recommended starting point.

---

## 1. The bandwidth budget

From [../../VIDEO_OPTIONS.md](../../VIDEO_OPTIONS.md):

| Link | Useful sustained throughput |
|---|---|
| LoRa custom stack, SF7/BW500, US ISM | **~12 kbps** raw, **~6–8 kbps** after framing/FEC/AES‑GCM |
| LoRa SF9/BW250 (more range, less rate) | ~3 kbps useful |
| LoRaWAN 1% duty cycle | ~0.1 kbps avg |

For comparison, watchable conventional video starts around 30 kbps (H.265, 96×80, 5 fps). We are **3–10× below that floor**, so a conventional codec can't get us there. We need either:

- A codec that exploits the fact that the *content* is highly predictable (driving / yard work, a slowly moving camera over mostly‑static ground), OR
- A scheme where the receiver has a strong prior about the world and only needs corrections, OR
- Acceptance that the LoRa fallback is a "slideshow + telemetry overlay" rather than a video stream.

Targets to design against:

| Mode | Frame size | Frame rate | Bitrate | Use case |
|---|---|---|---|---|
| **A — slideshow** | 160×120 JPEG | 0.1–0.2 fps | 2–5 kbps | Proof‑of‑life when WiFi/cellular both gone |
| **B — predicted video** | 320×240 reconstructed | 5–10 fps perceived | 6–10 kbps | Situational awareness, *not* fine control |
| **C — semantic stream** | 320×240 rendered from a scene model | 5 fps | 4–8 kbps | Driving with caveats (operator sees a synthesized view; ground truth annotated) |
| **D — keyframe + diff** | 256×192 H.265 I‑frames every N seconds | event‑driven | 8–10 kbps avg | Hybrid; degrades gracefully to A |

These are *fallback* modes. When WiFi or cellular is up, we use the full 720p path and these compression schemes are off.

---

## 2. comma.ai video compression challenge — what it teaches us

Source: <https://github.com/commaai/comma_video_compression_challenge> (closes May 3 2026; LifeTrac is not entering, we're studying the techniques).

**Setup:**

- One 1‑minute, 37.5 MB dashcam clip (`videos/0.mkv`). Real driving, urban + suburban.
- Score = `100 * segnet_distortion + 25 * compression_rate + sqrt(10 * posenet_distortion)` — *lower is better*.
- `segnet_distortion`: how much a pretrained semantic‑segmentation network's output disagrees on original vs. reconstructed frames.
- `posenet_distortion`: MSE on a pretrained ego‑motion (PoseNet) network's output for consecutive frame pairs.
- Compression rate: compressed / original size.

So the metric explicitly rewards keeping **what a downstream perception network sees** rather than pixel fidelity. PSNR / SSIM are not used. This is the right metric for our use case too — operators care that a person, post, or pile of dirt is *there and roughly where*, not that the leaves are crisp.

**Leaderboard insights (as of late April 2026):**

| Rank | Score | Submission | Approach |
|---|---|---|---|
| 1 | 0.33 | `quantizr` | (Top entry — heavy custom pipeline; details in PR #55) |
| 2 | 0.60 | `mask2mask` | Segmentation‑mask‑level reconstruction |
| 3 | 1.89 | `neural_inflate` | Neural decoder upsamples a tiny bitstream |
| 4 | 1.94 | `roi_v2` | ROI‑weighted AV1 (more bits where SegNet cares) |
| ~ | 1.95–2.20 | many `svtav1_*` / `av1_*` variants | Tuned SVT‑AV1 / AV1 with ROI, GOP, sharpening, film‑grain synthesis |
| ~ | 2.55–3.32 | `h265_*` variants | Tuned H.265 |
| baseline | 4.39 | `baseline_fast` | Default ffmpeg run |
| ceiling | 25.0 | `no_compress` | Reference upper bound |

**Take-aways for LifeTrac:**

1. **AV1 + ROI is a strong "boring" baseline.** Most of the leaderboard between 1.9 and 2.2 is some flavor of SVT‑AV1 with ROI weighting, large GOPs (300–360 frames), spline/lanczos rescale, and film‑grain synthesis. This is reproducible with ffmpeg/SVT‑AV1 on the X8 today — no ML required.
2. **Neural decoders ("inflate") beat hand‑tuned codecs by ~2× score** at the cost of GPU at decode time. Encoder runs on the tractor X8; decoder runs on the base station X8 — both have CPU, neither has a GPU. So we need a model small enough for ARMv8 NEON / TFLite XNNPACK inference (≤ ~50 MFLOPs/frame at 5 fps target).
3. **Segmentation/mask‑level reconstruction** (`mask2mask`, score 0.60) is the closest match to the "predicted environment from highly compressed images" idea in this task. The encoder essentially sends a label map + sparse style cues; the decoder hallucinates the pixels. This is the pattern we should look at hardest.
4. **GOP can be very long** (300–360 frames at 20 fps = 15–18 s between keyframes) on dashcam footage because the scene is *mostly* a translation. Tractor cameras at 1–5 mph see even slower scene change — so an even longer GOP is plausible.
5. **The metric ignores pixel fidelity.** Operators are not the metric — but for low‑bandwidth situational awareness, the analogous metric for us is "can the operator (or an onboard CV pipeline) still recognize obstacles, edges of furrows, people, the bucket position?" That's closer to SegNet than PSNR.

---

## 3. Existing projects in this space

### Generative / neural video codecs

- **comma.ai `neural_inflate` and `mask2mask`** — see above. Code is in the challenge repo's `submissions/` directory (PRs #49 and #53). Worth porting the inflate model architecture as a starting point.
- **NVIDIA Maxine** — face‑centric generative codec; sender transmits a face landmark vector + occasional keyframe, receiver reconstructs at 10–100× compression vs H.264. Wrong domain (faces) but the architecture (keyframe + sparse driving signal + GAN decoder) maps directly onto our problem.
- **DeepMind / Google "MIMT" and follow‑ons** — masked image transformer codecs, send a few tokens per frame.
- **Microsoft DCVC / DCVC‑DC / DCVC‑FM** (2022–2024) — open‑source neural video codec, beats H.266/VVC on PSNR at very low bitrates. Models are large (tens of MB) but the conditional‑coding idea (each P‑frame conditioned on a *learned* feature of the previous reconstructed frame, not on pixels) is portable.
- **CompressAI** (InterDigital) — PyTorch toolkit for learned image/video compression with several reference architectures (SSF, ELIC, Cheng2020). Best library to prototype in.
- **Google "Lyra v2"** is for audio, but the architectural pattern — quantized features + small generative decoder — is what we'd copy for video.

### "Send a scene, render the frame" / world‑model codecs

- **Comma.ai's own driving simulator + world models** (MuZero / Dreamer descendants used internally) — they don't ship one as a codec, but the underlying primitive (encode the observation as a small latent that a decoder can rollout) is exactly the predicted‑environment idea.
- **GAIA‑1 (Wayve, 2023)** and **Vista (2024)** — driving world models that take a few tokens per frame and generate plausible future driving video. Not real‑time on a Cortex‑A53, but the *encoder side* (tokenize a frame to ~1 kbps of indices) is feasible at low resolution.
- **NeRF / Gaussian‑splatting streaming** — for *static* scenes (yard, shed, fixed worksite), the tractor could stream *pose updates only*, while the base station renders from a pre‑built 3D model of the property. Scene changes are sent as residual photos. This is the "predicted environment from highly compressed images" idea taken to its limit; it's research‑grade but extremely cheap on bandwidth.

### Classical / shippable today

- **SVT‑AV1 with ROI maps + film‑grain synthesis** — see comma leaderboard rows 4–13. Pure ffmpeg, runs on the X8, no ML. Probably wins us 2–3× over plain H.265 at the same score.
- **H.265 with motion‑constrained tiles + reference‑frame substitution** — keep a "background plate" reference frame and only encode foreground tiles. Used in old surveillance codecs (Axis Zipstream, Hanwha WiseStream).
- **JPEG2000 / JPEG XL slideshow** — for Mode A, JPEG XL beats JPEG by 30–60% at thumbnail sizes; encoder is small enough to run on the H7 if we ever lose the X8.
- **Codec2 / FreeDV‑style "send the scene parameters, not pixels"** for slowly‑changing on‑screen content — overkill comparison but the philosophy is what we want.

### Existing low‑bandwidth video‑over‑LoRa work

- **HamPi / SSDV** — slow‑scan digital video used by amateur radio and high‑altitude balloon projects, JPEG slices over FSK. ~1–5 kbps, slideshow‑style. Already proven in the field at LoRa‑comparable rates. Most boring possible solution; a good fallback floor.
- **TinyML "video over LoRa" demos** (Nicla Vision, OpenMV) — almost all cheat by running detection on the tractor and only sending bounding boxes + class labels, not video. Useful primitive (see Option 5 below).

---

## 4. Compression options for LifeTrac v25

Ranked from least to most ambitious. Each option lists what runs on the **encoder X8 (tractor)** and the **decoder X8 (base station)**, expected bitrate, and an honest assessment.

### Option 1 — SSDV slideshow (boring, shippable, baseline)

- **Encoder:** ffmpeg / libjpeg on tractor X8 grabs one 160×120 JPEG every 5–10 s; SSDV‑style chunked into ~250‑byte packets with FEC.
- **Decoder:** SSDV reassembler on base station X8; web UI shows latest thumbnail with timestamp.
- **Bitrate:** 2–5 kbps.
- **Latency:** 5–30 s.
- **Ship effort:** Days. ffmpeg + SSDV are off‑the‑shelf.
- **Verdict:** This is the **fallback floor**. Build it first regardless of what else we attempt — it's the "the WiFi died and I need to know the tractor isn't on fire" mode.

### Option 2 — SVT‑AV1 with long GOP + ROI + film‑grain synthesis

- **Encoder:** SVT‑AV1 on the X8 at 256×192 / 5 fps, GOP = 150 (30 s), CRF 50‑ish, ROI map weighting the lower‑center (where the bucket / ground‑contact is) and top‑center (horizon for obstacle detection). Film‑grain table sent once.
- **Decoder:** dav1d on base station X8, hardware‑accelerated where possible. Renders to a small live tile.
- **Bitrate:** 8–15 kbps. Probably *just* over the SF7/BW500 budget — likely needs SF7/BW500 with 100% of the link given to video, which competes with control packets. May need to drop to 192×144 / 3 fps to fit.
- **Latency:** 1–3 s end‑to‑end (long GOP + buffering).
- **Ship effort:** 1–2 weeks. All open‑source; the work is in the ROI map and tuning grid (the comma.ai leaderboard's grid‑search CSV is a directly‑applicable starting point).
- **Verdict:** Best classical option. Should be implemented after Option 1 and used as the comparison baseline for the neural options below.

### Option 3 — Keyframe + neural temporal inflate (the question you asked)

This is the **"occasional keyframe to draw a predicted environment from highly compressed images"** idea, made concrete.

- **Encoder (tractor X8):**
  - Every N seconds (start with N = 5–10 s) send a real keyframe: a heavily‑compressed JPEG/AVIF at 256×192, ~3–5 kB.
  - Between keyframes, run a tiny encoder network (think MobileNet‑v3‑small backbone, distilled) that produces a ~64–128‑byte per‑frame latent capturing motion + appearance change. 5 fps × 100 B = ~4 kbps.
  - Optionally include sensor ego‑motion (wheel speed, IMU yaw rate from CAN — already on the LoRa control channel) as a side signal — *free*, since it's already being sent for telemetry.
- **Decoder (base station X8):**
  - Holds the most recent keyframe.
  - For each latent received, runs a small generator (think U‑Net with FiLM conditioning on the latent + ego‑motion, ~3–10 M params, INT8) that warps/refines the keyframe into the current frame.
  - On the next keyframe, snaps back to ground truth — no error accumulation across keyframes.
- **Bitrate:** 6–10 kbps. Fits SF7/BW500 with control‑channel headroom.
- **Latency:** 200–600 ms (one inference on each end + LoRa hop).
- **Ship effort:** 4–8 weeks if we adapt an existing model (DCVC‑FM or comma's `neural_inflate`); 3–6 months if we train from scratch on tractor footage.
- **Verdict:** **This is the answer to the user's actual question.** It's the right architecture for our compute and bandwidth situation. Deferred behind Options 1+2 because we should not ship it without the boring fallbacks underneath it.
- **Open questions:**
  - Can the X8 i.MX 8M Mini run the encoder + decoder networks at 5 fps without the NPU? The 8M *Mini* (vs Plus) has no NPU; we have CPU NEON only. Rough budget: 4 × A53 @ 1.8 GHz × NEON ≈ 30 GFLOPS peak, realistically 5–10 GFLOPS sustained. A 50 MFLOP/frame model at 5 fps = 250 MFLOPS sustained = feasible. A 500 MFLOP/frame model is not.
  - How do we resync after a lost keyframe? Probably: receiver requests retransmit; until it arrives, Option 1 takes over (slideshow with the most recent keyframe shown).
  - Domain shift: comma.ai's models are trained on dashcam footage at 20 mph in cities. A bucket loader at 2 mph in mud will look like out‑of‑distribution data. Need to fine‑tune on tractor footage we record ourselves.

### Option 4 — Mask‑level / "send the scene graph, render the pixels"

The `mask2mask` (comma score 0.60) approach generalized.

- **Encoder:** Run a small semantic‑segmentation net on the tractor X8. Send a quantized segmentation map (e.g. 64×48 with 16 classes = 1.5 kB per keyframe) plus a few "style tokens" (PCA of feature maps, ~50 B per frame).
- **Decoder:** Conditional GAN / diffusion mini‑model takes (mask + style + previous frame) → rendered RGB.
- **Bitrate:** 4–8 kbps.
- **Latency:** 500 ms – 2 s.
- **Ship effort:** 6–12 weeks; needs labeled tractor‑scene data (or we use a pretrained outdoor‑driving SegNet and accept its labels).
- **Verdict:** Higher visual quality than Option 3 at lower bitrate, *but* the rendered image is a hallucination. Operator must understand they are seeing a *plausible* reconstruction, not a recording. This matters for safety — annotate the UI with "synthesized" overlay, log raw masks for replay.

### Option 5 — "Don't compress video, compress the situation"

The most aggressive interpretation. Don't send pixels at all.

- **Encoder:** Run object detection (YOLO‑nano, ~5 MB INT8, ~30 ms on A53), lane/edge detection, and bucket‑pose estimation on the tractor X8. Output: a list of bounding boxes, classes, confidences, plus bucket angle, plus a coarse free‑space mask.
- **Bitstream:** ~50–500 B per frame at 5 fps = ~2–20 kbps depending on scene complexity.
- **Decoder (base station X8):** Renders a synthetic top‑down or first‑person view from the structured data: ground plane + tractor model (we already have CAD) + detected obstacles as labeled markers + free‑space shaded green. Like a video‑game minimap of what the tractor sees.
- **Ship effort:** 4–8 weeks; a lot of integration but no novel ML.
- **Verdict:** Extremely safe (operator knows they're looking at a synthesized situational display, not video), extremely robust (works at any link rate), composes naturally with telemetry, and arguably *more useful* than low‑res video for the actual driving task. Loses incident‑review value (you can't go back and look at "what really happened" from the LoRa stream — but the tractor X8 keeps the full local recording for that).
- **This pairs well with Option 3:** structured situation overlay *plus* a 5‑fps predicted‑video tile in the corner.

### Option 6 — Static world model + pose updates (research)

- Pre‑build a 3D model (Gaussian splatting / NeRF) of the property from a one‑time mapping pass.
- Tractor sends only its pose + a tiny residual when something has changed (a pile of dirt that wasn't there last week).
- Base station renders the X8's current view from the pre‑built model.
- **Bitrate:** ~100 bps for pose; residuals event‑driven.
- **Verdict:** Beautiful for fixed worksites (yard, shed, repeat fields). Useless on a new property until it has been mapped. Filed under "future work after Option 3 ships".

---

## 5. Recommendation

Build in this order:

1. **Option 1 (SSDV slideshow)** — fallback floor, days of work, gives us a baseline number to beat.
2. **Option 5 (situational overlay)** — high practical value per engineering hour, leverages onboard CV that we'll want anyway for autonomous features.
3. **Option 2 (SVT‑AV1 + ROI)** — best classical baseline; reuses comma.ai's published ffmpeg grid‑search results.
4. **Option 3 (keyframe + neural inflate)** — the actually‑novel one and the answer to the user's framing of the question. Adapt comma.ai's `neural_inflate` (PR #49) or `mask2mask` (PR #53) to a tractor‑scene dataset, distill to fit ~50 MFLOPs/frame on the i.MX 8M Mini.

Options 4 and 6 are filed for later.

The header of [../../VIDEO_OPTIONS.md](../../VIDEO_OPTIONS.md) should eventually gain a row "LoRa (with v25 neural codec)" once Option 3 has been measured on a real link. Until then, keep its current line — *LoRa carries no live video* — as the honest design assumption.

---

## 6. Datasets we'd need

- **comma2k19** (already public, 2.4 GB sample at <https://huggingface.co/datasets/commaai/comma2k19>) — for Option 3 pretraining.
- **Self‑recorded tractor footage** at 256×192 / 5 fps, with synchronized CAN ego‑motion, ~10–20 hours covering: yard work, field driving, bucket loading, mud, dust, dusk/dawn, rain. Recordable now via the Portenta X8 MIPI camera path that's already on the [../../TODO.md](../../TODO.md) (Phase 10 stretch).
- **Property scan** (pose‑annotated images or a Gaussian‑splat capture) — only if we ever attempt Option 6.

---

## 7. References

- comma.ai video compression challenge: <https://github.com/commaai/comma_video_compression_challenge>
- comma.ai grid search CSV (ffmpeg parameters vs score): linked from the challenge README under "going further".
- comma2k19 dataset: <https://huggingface.co/datasets/commaai/comma2k19>
- DCVC family (Microsoft, neural video codec): <https://github.com/microsoft/DCVC>
- CompressAI (InterDigital, learned compression toolkit): <https://github.com/InterDigitalInc/CompressAI>
- SSDV (slow‑scan digital video for low bandwidth radio): <https://github.com/fsphil/ssdv>
- SVT‑AV1: <https://gitlab.com/AOMediaCodec/SVT-AV1>
- GAIA‑1 driving world model (Wayve): <https://wayve.ai/thinking/scaling-gaia-1/>
- NVIDIA Maxine generative codec for video calling: <https://developer.nvidia.com/maxine>
- Existing v25 video plan and link budget: [../../VIDEO_OPTIONS.md](../../VIDEO_OPTIONS.md)
- Architecture context (compute on each end of the LoRa link): [../../ARCHITECTURE.md](../../ARCHITECTURE.md)
