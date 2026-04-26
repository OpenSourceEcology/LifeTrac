# Predicted / interpolated future frames between LoRa keyframes

**Status:** research note, not on any phase plan yet.
**Question from operator:** *"comma.ai does future-frame prediction. Could we use that to invent intermediate frames between the rare images we receive over LoRa, then blend into the next real frame when it arrives?"*
**Short answer:** **Yes — and the research literature is large and mature.** Two distinct techniques apply: **video frame interpolation (VFI)** in the gap, and **future-frame prediction (FFP)** for the "leading edge" before the next real frame arrives. Both are real engineering options for v25, both cost meaningful compute on the base-station X8 (i.MX 8M Mini, NEON only — no NPU), and there's a clean fallback hierarchy if compute runs out.

This complements three earlier docs: the [Phase D neural-inflate plan in VIDEO_COMPRESSION/README.md](README.md), the [grayscale recolorization note](GRAYSCALE_RECOLORIZATION.md), and the [LoRa QoS bandwidth doc](../../../AI%20NOTES/2026-04-26_LoRa_QoS_Bandwidth_Management.md). Where Phase D inflates a keyframe forward in time using *latent codes* sent from the tractor, this doc is about inflating a keyframe forward in time using **only side signals** — CAN-bus ego-motion, IMU, and prior frames — *with no per-frame data from the tractor at all*.

---

## 1. The two related but distinct problems

| Problem | When does it apply | What's known | What's unknown |
|---|---|---|---|
| **Video frame interpolation (VFI)** — fill the gap between two known frames | After the next keyframe arrives, but you want smooth playback of the gap | Frame at t=0 and t=T (both real) | Frames at t∈(0, T) |
| **Future-frame prediction (FFP)** — extrapolate forward from the last known frame | Live, while waiting for the next keyframe | Frame at t=0 only, plus side signals | Frame at t>0 |

Both can run in v25, and they answer different needs:

- **FFP keeps the operator's display moving in real time** while waiting for the next LoRa frame. Without it, the display freezes for 0.5–10 s between received frames — disorienting and unsafe-feeling for tele-op.
- **VFI smooths out playback during recorded log review.** Less urgent. Not needed live.

For a tele-op operator, **FFP is the prize.** VFI is a nice-to-have for log review and for the "blend back into the next real frame" merge step at the end of the gap.

---

## 2. Frame interpolation (VFI) — the easy half

This is solved. There are a dozen production-quality models and a strong no-AI baseline. Picking the right one is mostly a compute-budget exercise.

### 2.1 No-AI baseline: dense optical flow + warp

OpenCV's Farneback or DIS optical flow + `cv2.remap`. Works well for the **midpoint** of two close frames; degrades as the gap grows or as objects enter/leave. Same machinery as Tier 2 of the [recolorization doc](GRAYSCALE_RECOLORIZATION.md), so we get this for free if we ship that.

~80 ms/frame at 256×192 on the X8. Quality: passable for in-gap frames at ≤5 frame interpolation factor; poor for new content (disocclusions).

### 2.2 RIFE (2021) — the modern default

[RIFE: Real-Time Intermediate Flow Estimation for Video Frame Interpolation](https://github.com/megvii-research/ECCV2022-RIFE) (Huang et al., ECCV 2022). MIT license. Predicts intermediate flow + a fusion mask in one network. Pretrained models from ~7 MB (rife-tiny) to ~50 MB (rife-v4.6).

Real-time on a desktop GPU; on the X8 the tiny INT8-quantized variant runs ~120 ms/frame at 256×192 per the Phase D napkin math. **~8 fps achievable** — fine for our 5 fps target stream.

Robust to gap sizes up to ~8 frames at typical motion. **This is the recommended VFI model for v25.**

### 2.3 Other VFI models worth knowing

| Year | Model | License | Why mention |
|---|---|---|---|
| 2017 | SepConv (Niklaus et al.) | MIT | First widely-used learned VFI; superseded |
| 2018 | Super-SloMo (Jiang et al.) | MIT | Multi-frame interpolation in one shot |
| 2020 | XVFI (Sim et al.) | non-commercial | Handles large motion well; license blocks production use |
| 2022 | RIFE v4 | MIT | **Default pick** |
| 2022 | FILM (Reda et al., Google) | Apache-2 | Larger but better for "near-duplicate" photos |
| 2023 | EMA-VFI (Zhang et al.) | MIT | Better than RIFE on benchmarks; ~3× the compute |
| 2023 | AMT (Li et al.) | MIT | Another RIFE-class option |

For v25, RIFE is the right balance of license + compute + quality.

---

## 3. Future-frame prediction (FFP) — the hard half, but where the win is

### 3.1 The literature, summarized

Future-frame prediction is an active research area going back to ~2014. The taxonomy:

| Year | Paper / repo | Approach | Frames ahead | Notes |
|---|---|---|---|---|
| 2014 | Ranzato et al. "Video (language) modeling" | LSTM on patches | 1–3 | First deep FFP |
| 2015 | Srivastava et al. "Unsupervised LSTM" | ConvLSTM | 1–10 | Moving-MNIST baseline |
| 2016 | Mathieu et al. "Beyond MSE" | Adversarial loss | 1–4 | Sharper than MSE-trained |
| 2017 | Finn et al. "Stochastic Variational Video Prediction" (SV2P) | Variational stochastic | 1–10 | Robot-arm domain |
| 2018 | Wichers et al. "Hierarchical Long-term Video Prediction" (Google Brain) | Two-stage | up to 64 | Diverging artifacts past ~20 |
| 2018 | **comma.ai chauffeurnet / Learning a Driving Simulator** | World-model approach | continuous | The reference the operator is asking about |
| 2019 | Castrejon et al. "Improved CVAE for video prediction" | Hierarchical CVAE | 1–25 | DeepMind |
| 2020 | Villegas et al. "High Fidelity Video Prediction" (Google) | Stochastic + autoregressive | 1–28 | Best fidelity at the time |
| 2021 | Wu et al. "Greedy Hierarchical VAE" | Compute-efficient | 1–10 | |
| 2022 | comma.ai "**Learning a Driving Simulator**" research | VQ-VAE + transformer (open) | 1–60 | Conditioned on **steering + speed** — exactly our signals |
| 2023 | Ho et al. "Imagen Video" | Diffusion | 1–48 | High quality, expensive |
| 2024 | Brooks et al. **OpenAI Sora** technical report | Diffusion transformer | seconds | Unconditional / text-conditioned, not driving-conditioned |
| 2024 | Wayve "GAIA-1" | World model | seconds | Open paper, closed weights |
| 2024 | comma.ai **commavq / commaCarSim** | Quantized latent + AR | continuous | **Open weights** |
| 2025 | Wayve "GAIA-2" / NVIDIA "Cosmos" | Foundation world models for driving | seconds | Multi-billion params; not feasible on X8 |

### 3.2 What comma.ai actually does

The thing the operator is referencing is comma.ai's **driving simulator / world model** line of work. The current open release is **commaVQ** + **commaCarSim** ([github.com/commaai/commavq](https://github.com/commaai/commavq), MIT license):

1. **VQ encoder/decoder.** A vector-quantized autoencoder compresses each 128×256 frame to a small grid of integer "tokens" (e.g. 8×16 = 128 tokens, each from a 1024-token vocabulary). ~80 MB encoder, ~80 MB decoder. Pre-trained.
2. **Autoregressive transformer ("commaCarSim").** Takes the past N frames' tokens + the current **steering angle and speed** as conditioning, and predicts the next frame's tokens. ~50–200 MB depending on size.
3. **Decode.** Run the VQ decoder on the predicted tokens to render the predicted next frame.

The **conditioning on steering + speed** is the magic: this is *not* unconstrained "what might happen?" prediction; it's "given that the driver is turning the wheel +12° at 35 km/h, what does the next frame look like?". For a tractor with our CAN ego-motion already on the LoRa control channel, **we have those signals for free** — they're already going across the link as control telemetry.

This is the same insight that motivated Phase D (neural-inflate uses CAN ego-motion as a "free side signal" to warp the keyframe forward). FFP is Phase D's logical extension: instead of just *warping* the keyframe, *generate plausible new content* in the regions the warp can't reach, conditioned on where the tractor is going.

### 3.3 How far ahead can we predict?

The bad news: **prediction quality decays with horizon.** All published FFP models exhibit:

- **0–5 frames (~0–1 s):** indistinguishable-from-real-on-static-content. Moving objects sometimes blur. **Production-usable.**
- **5–25 frames (~1–5 s):** structure intact, but textures degrade, edges soften, novel content (a person walking out from behind a tree) doesn't appear. **Trust-as-context, not as ground-truth.**
- **25+ frames (~5+ s):** semantically plausible but increasingly hallucinated. Useful for entertainment/research, **not** for safety-critical tele-op.

For LoRa with keyframes every 0.5–10 s:

- At Phase A (one keyframe every 5–10 s): we'd be predicting 25–50 frames between keyframes. **Beyond the trust-as-context boundary.** Not safe to drive on.
- At Phase C (one frame every 200 ms = 5 fps real): we'd be predicting 1–4 intermediate frames at 25 fps perceived. **Right in the sweet spot.**
- At Phase D (one keyframe every 5–10 s + tiny per-frame latent at 5 fps real): the per-frame latent already provides *real* per-frame information. FFP is the fallback for moments when even the latent is dropped by the link. **Cleanest integration point.**

So FFP is most useful **paired with Phase C or D**, not as a replacement for sending real frames at 5 fps.

---

## 4. Compute budget — the honest numbers

The X8 (i.MX 8M Mini quad A53 @ 1.8 GHz, NEON SIMD, no NPU) gives us ~5–10 GFLOPs sustained for ML across all four cores combined. That's our budget. The base-station X8 must also run the web UI, MQTT broker, lora_bridge, recolorization, and (eventually) the Phase D neural-inflate. Realistically FFP gets ~2–4 GFLOPs/s of that.

Per 256×192 frame at 5 fps (so 200 ms budget) on 2–4 GFLOPs/s = **~400–800 MFLOPs per frame** budget for FFP.

| Model class | Approx. FLOPs/frame | Fits? |
|---|---|---|
| Optical-flow warp (Farneback / DIS) | ~50 MFLOPs | yes, comfortable |
| RIFE-tiny INT8 | ~200 MFLOPs | yes |
| commaVQ encoder + decoder + small transformer (distilled) | ~500–2000 MFLOPs | tight; needs distillation |
| commaVQ at full published size | ~5–20 GFLOPs | **no** — won't fit |
| Diffusion-based FFP (Sora-class) | 100s of GFLOPs | **no, not even close** |

So the **realistic v25 path** is:

1. **Tier 1 (no AI): kinematic flow extrapolation.** Take the last received frame's optical flow, extrapolate it forward by k×Δt, warp the frame. Falls apart fast (~3 frames) but costs nothing. ~50 MFLOPs.
2. **Tier 2 (light AI): RIFE forward extrapolation.** RIFE was designed for *interpolation* between two known frames, but it can be coaxed into *extrapolation* by feeding it (frame_t-2, frame_t-1) and asking for the "midpoint" of (frame_t-1, frame_t) — i.e., asking it to extrapolate one step. Quality drops vs. true interpolation but works for 1–3 frames ahead. ~200 MFLOPs/frame, fits.
3. **Tier 3 (real FFP): distilled commaVQ.** Take the full commaVQ pipeline as a teacher; train a student model ~5× smaller; INT8 quantize. ~500–1000 MFLOPs/frame. Borderline-fits. **This is a research-tier item; not Phase D, but Phase D+1.**

---

## 5. Blending the predicted gap into the next real frame

When the next real frame arrives — typically every 200 ms at Phase C — we have:

- A predicted frame (what we *thought* would be at this timestamp)
- A real frame (what actually is at this timestamp)

If we just snap to the real frame, the operator sees a visible "pop." Better to **cross-fade** over ~50–100 ms (1–2 displayed frames at 25 fps). Two ways:

1. **Pixel-space cross-fade.** Linear blend: `display_frame = α × predicted + (1-α) × real`, with α ramping from 1.0 to 0.0 over 2 frames. Cheap and good enough.
2. **Optical-flow guided blend.** Warp the predicted frame to align with the real frame using flow between them, then blend. Smoother for cases where the prediction was geometrically off but semantically right. ~80 ms compute.

For v25, **pixel-space cross-fade is the recommendation.** The Phase C cadence is fast enough that the prediction error per gap is small; spending compute on flow-guided blending is wasted.

---

## 6. The drift problem and how to bound it

A naive FFP loop **diverges over time**: predict frame N+1 from frame N, then N+2 from your predicted N+1, etc. Errors compound; after a few seconds you're hallucinating freely.

Mitigations, in order of importance:

1. **Reset to real frame on every keyframe arrival.** Hard reset, then cross-fade. The keyframe cadence bounds the worst-case drift duration.
2. **Confidence gate.** If the FFP model exposes a per-pixel uncertainty (commaVQ's transformer output entropy is a free proxy), darken or grey-out high-uncertainty regions in the displayed frame so the operator visually distinguishes "predicted with confidence" from "guessed."
3. **Disocclusion masking.** If the predicted optical flow says "this region had no valid source pixel in the previous frame," don't predict there — show a placeholder color (matching the recolorization reference's a*/b* mean is a cheap default).
4. **"PREDICTED" badge.** Same UX rule as the [recolorization](GRAYSCALE_RECOLORIZATION.md) "RECOLOR" badge and the Phase B [SYNTHESIZED](README.md) badge: any pixels that are not direct from the tractor MUST be visually distinguishable from those that are. Show a small `PREDICTED t+150ms` overlay in the corner. Tractor X8 keeps the **un-predicted** real footage cached locally for incident review, independent of what crosses LoRa.

---

## 7. Interaction with everything else

This adds nicely on top of the existing stack but needs the right integration points:

- **Phase D (neural-inflate) is the strongest pairing.** Phase D already uses CAN ego-motion as a side signal and produces per-frame outputs; FFP fills the (rare) gap when even the per-frame latent is dropped. The two share most of the infrastructure: ego-motion handling, base-station model runtime, "synthesized pixel" UX badge.
- **Recolorization (Tier 1/2/3) runs *after* FFP in the pipeline.** Predicted frames are luminance-only; recolorize them using the same reference frame as the real ones. Cleanly composable.
- **Crop-health analysis runs only on tractor side, on raw frames.** Predicted frames are plausible-fiction and useless for NDVI. (Same caveat as recolorization.)
- **Bandwidth arbitration is unaffected.** FFP runs entirely on the base; the LoRa stream doesn't change.
- **Latency.** The base-station prediction loop adds latency equal to the predict-frame compute time (~100–200 ms in Tier 2/3). For tele-op this is meaningful — consider it part of the end-to-end latency budget alongside LoRa airtime + decode.

---

## 8. Recommendation for v25

| Tier | Compute | When to ship | Effort |
|---|---|---|---|
| **Tier 1 — flow extrapolation + cross-fade** | ~50 MFLOPs/frame | Phase A.5 companion | ~1 week of engineering, no model files |
| **Tier 2 — RIFE-tiny forward extrapolation** | ~200 MFLOPs/frame | Phase C companion | ~2 weeks, ship model with base-station container |
| **Tier 3 — distilled commaVQ FFP** | ~500–1000 MFLOPs/frame | Phase D+1 stretch | months, requires the same ≥10 h tractor dataset as Phase D |

Add bullets to [TODO.md § Stretch goals](../../TODO.md) Phase 10+ video work near the existing Phase A/B/C/D bullets, flagged as **companion** tasks the same way the recolorization tiers are. Do not block any other phase on this; it's pure UX polish on top of phases that already meet the safety bar.

---

## 9. Open questions / further research

- **How well does commaVQ generalize from highway driving (its training distribution) to a tractor in a wheat field?** Likely poorly. Distillation would need our own dataset, which we already plan to capture for Phase D.
- **Do operators *want* simulated frames, or does it feel uncanny / make them mistrust the display?** Worth a small user study with Tier 1 once we have it. The UX badge helps but doesn't eliminate the question.
- **Can FFP help with the rear camera switch latency?** When the operator hits reverse and the rear camera takes ~200 ms to switch + transmit a fresh frame, can we use FFP to render a *plausible* rear view from the front view + pose? Probably not — rear is not derivable from front. But worth thinking about: maybe blank-fade to the rear view rather than sit on a stale front view.
- **Can the tractor side help by sending tiny "hints" mid-gap?** A 32-byte "key region changed" hint between keyframes could anchor the prediction. This starts to look exactly like Phase D, which is the right framing — **FFP without hints is a "Phase C+" upgrade; FFP with hints just *is* Phase D** with a different decoder design.
- **Diffusion-based FFP (Sora, GAIA-2) is too big for the X8 today.** Will it be too big in 5 years? Probably yes for the X8; possibly no for whatever replaces it. Track but don't pursue.

---

## 10. References

- comma.ai (2016). *Learning a Driving Simulator.* arXiv:1608.01230. ([repo](https://github.com/commaai/research))
- comma.ai. **commavq** — quantized latent driving simulator. ([repo](https://github.com/commaai/commavq), MIT)
- Mathieu, Couprie, LeCun (2016). *Deep multi-scale video prediction beyond mean square error.* ICLR.
- Finn, Goodfellow, Levine (2016). *Unsupervised Learning for Physical Interaction through Video Prediction.* NeurIPS. (origin of SV2P)
- Wichers, Villegas, Erhan, Lee (2018). *Hierarchical Long-term Video Prediction without Supervision.* ICML.
- Castrejon, Ballas, Courville (2019). *Improved Conditional VRNNs for Video Prediction.* ICCV.
- Villegas, Pathak, Kannan, Erhan, Le, Lee (2019). *High Fidelity Video Prediction with Large Stochastic Recurrent Neural Networks.* NeurIPS.
- Niklaus, Mai, Liu (2017). *Video Frame Interpolation via Adaptive Separable Convolution.* ICCV. (SepConv, MIT)
- Jiang, Sun, Jampani, Yang, Learned-Miller, Kautz (2018). *Super SloMo.* CVPR.
- Huang, Zhang, Heng, Shi, Zhou (2022). *Real-Time Intermediate Flow Estimation for Video Frame Interpolation.* ECCV. ([repo](https://github.com/megvii-research/ECCV2022-RIFE), MIT)
- Reda et al. (2022). *FILM: Frame Interpolation for Large Motion.* ECCV. ([repo](https://github.com/google-research/frame-interpolation), Apache-2)
- Zhang et al. (2023). *Extracting Motion and Appearance via Inter-Frame Attention for Efficient Video Frame Interpolation.* CVPR. (EMA-VFI)
- Ho, Chan, Saharia, Whang et al. (2022). *Imagen Video.* arXiv:2210.02303.
- Brooks et al. (2024). *Video generation models as world simulators.* OpenAI Sora technical report.
- Hu, Russell, Yeo et al. (2023). *GAIA-1: A Generative World Model for Autonomous Driving.* Wayve.
- NVIDIA (2025). *Cosmos World Foundation Model Platform for Physical AI.* NVIDIA technical report.
- Hafner, Lillicrap, Norouzi, Ba (2020). *Mastering Atari with Discrete World Models* (Dreamer-V2). — same world-model lineage that drives commavq's architecture choice.

---

## 11. Bottom line

- **Yes, this is real and well-studied.** comma.ai, Wayve, Google Brain, NVIDIA, and ~50 academic groups have working implementations at various scales.
- **Yes, it costs more compute than recolorization.** Tier 1 is free; Tier 2 (RIFE-tiny) doubles the base-station ML budget; Tier 3 (distilled commaVQ) saturates it.
- **Yes, it pairs cleanly with v25's existing stack.** The CAN-bus ego-motion already on the LoRa link is exactly the conditioning signal these models want, and the "SYNTHESIZED" UX pattern from Phase B/D extends naturally to a "PREDICTED" badge.
- **No, it should not replace sending real frames.** It's a smoothing layer over a real video stream, not a substitute for one. The drift problem is bounded only by the keyframe cadence; cut keyframes too aggressively and FFP starts hallucinating.
- **Sequence:** ship Tier 1 alongside Phase A; ship Tier 2 alongside Phase C; consider Tier 3 only after Phase D is stable, since both share the same dataset capture and student-distillation infrastructure.
