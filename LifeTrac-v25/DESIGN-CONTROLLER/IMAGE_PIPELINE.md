# LifeTrac v25 — Image Pipeline Implementation Plan

> **Status (2026-04-27):** Canonical implementation plan for the v25 LoRa image pipeline. Synthesized from the four cross-reviewed design analyses in [`../AI NOTES/`](../AI%20NOTES/). The upstream design reasoning lives in [2026-04-27_Image_Transmission_InDepth_Analysis_ClaudeOpus4_7_v1_0.md](../AI%20NOTES/2026-04-27_Image_Transmission_InDepth_Analysis_ClaudeOpus4_7_v1_0.md) §14 (recommended stack), §15 (browser-tier offload), §16 (cross-review consensus). **This document is the source of truth for *what to build*; the AI NOTES analysis is the source of truth for *why*.**
>
> **Headline:** Build **Stack-NoCoral-Recommended** as the primary; treat Coral as a strict additive upgrade. Browser-tier offload is **mandatory in Phase 1**, not deferred. Ship Stack-MVP at week 4; full no-Coral stack at week 10. **Total LoRa budget: ~800–1000 B per ~1.5 s P3 refresh; P0/P1 always preempt.**

---

## 1. Scope and constraints

### 1.1 What this plan covers

- The full tractor → base → browser image pipeline for v25.
- Encoding, transport, reconstruction, enhancement, safety detection, operator UX, validation gates, and build order.
- The trust boundary between base and browser, and the rules that keep browser-tier offload from blocking v26+ autonomy.

### 1.2 What this plan does NOT cover

- **MIPI CSI camera bring-up** — defer to v25.5 (USB UVC ships first per [TODO.md Phase 0](TODO.md)).
- **Latent autoencoder learned codec** — defer to v26+, needs a farm-data corpus this project does not yet have.
- **Semantic-map degraded mode** — defer to v26+ per [analysis §16.3.2](../AI%20NOTES/2026-04-27_Image_Transmission_InDepth_Analysis_ClaudeOpus4_7_v1_0.md). Wireframe + optical-flow are the v25.x degraded modes.
- **Coral on the BOM** — optional; gated by the Phase-0 spike; pipeline ships and is validated CPU-only. See [HARDWARE_BOM.md Tier 2](HARDWARE_BOM.md#tier-2--base-station-portenta-max-carrier--portenta-x8) and [MASTER_PLAN.md §8.19](MASTER_PLAN.md).

### 1.3 Hard constraints (do not violate)

| # | Constraint | Source |
|---|---|---|
| C1 | **Zero P0 ControlFrame TX-start delay > 25 ms** attributable to image fragments. | [TODO Phase 5D](TODO.md), [LORA_PROTOCOL.md](LORA_PROTOCOL.md) |
| C2 | Image traffic at priority **P3 only**; preempted by any P0/P1/P2 frame. | [LORA_PROTOCOL.md](LORA_PROTOCOL.md) |
| C3 | `CMD_PERSON_APPEARED` (opcode `0x60`) reaches base UI in **≤250 ms p99**. | [TODO Phase 5D](TODO.md) |
| C4 | **No Coral on the tractor in v25.** Tractor stays CPU-only. | [MASTER_PLAN.md §8.19](MASTER_PLAN.md) |
| C5 | **The browser renders badges; the base decides them.** Trust boundary in §6.1. | [analysis §15.4](../AI%20NOTES/2026-04-27_Image_Transmission_InDepth_Analysis_ClaudeOpus4_7_v1_0.md) |
| C6 | **No silent extrapolation.** Every non-1:1 pixel carries a badge. Every stale tile shows age. | [analysis §14.2 / §16.2](../AI%20NOTES/2026-04-27_Image_Transmission_InDepth_Analysis_ClaudeOpus4_7_v1_0.md) |
| C7 | **AGPL exposure is a project-level decision.** NanoDet-Plus (Apache-2.0) is the always-available substitute for Ultralytics YOLOv8. | [TODO Phase 5B](TODO.md) |

---

## 2. The recommended stack

Adopted from [analysis §14.1](../AI%20NOTES/2026-04-27_Image_Transmission_InDepth_Analysis_ClaudeOpus4_7_v1_0.md). Each row is one of the seven roles R1–R7 from the role taxonomy in analysis §13.1.

| Role | Pick | Phase | License |
|---|---|:-:|---|
| **R1 wire encoding** | **B tile-delta I/P frames** + **Z Y-only luma** auto-enabled below 400 B/refresh | 1 | — |
| **R2 tractor pre-process** | **C image registration** (phase correlation, NEON) + **M NanoDet-Plus** | 1 / 2 | Apache-2.0 |
| **R3 ROI policy** | **D 3-level ROI rings** + valve-state classifier + **F operator-cued (CMD_ROI_HINT)** + M detection bias | 1–2 | — |
| **R4 base reconstruction** | **E persistent canvas** + **I background cache** (rolling per-tile median) | 1 | — |
| **R5 base AI enhancement** | **H Real-ESRGAN-General-x4v3** (ncnn, A53 CPU, 0.5–1 fps) | 2 | BSD |
| **R6 base safety detector** | **J YOLOv8-nano CPU** *or* **NanoDet-Plus CPU** (AGPL decision pending) | 2 | AGPL-3.0 / Apache-2.0 |
| **R7 sidecars** | **N CMD_PERSON_APPEARED (P0)** + **R YAMNet audio** + **S 3-cam attention mux** + **U event-skip** + **T satellite far-field** | 1–2 | varies |

**Hardware delta vs. v25 baseline:** $0. **Coral required:** No.

### 2.1 Estimated steady-state budget

From [analysis §14.3](../AI%20NOTES/2026-04-27_Image_Transmission_InDepth_Analysis_ClaudeOpus4_7_v1_0.md), with browser-tier offload applied:

| Metric | Target |
|---|---|
| Tractor CPU | ≈140 % of one core (~35 % of total 4 cores) |
| Base CPU | ≈40 % of total 4 cores (with browser offload + 0.5 fps super-res) |
| Bytes / refresh | 200–500 + sidecars (Y-only mode: 150–350) |
| Glass-to-glass latency | ~850 ms p50 / ~1300 ms p99 |
| `CMD_PERSON_APPEARED` latency | ~250 ms p99 |
| Perceived FPM | ~150 (40 raw tile updates + per-tile cross-fade + canvas persistence) |

---

## 3. Wire format additions

The transport spine is already in [LORA_PROTOCOL.md](LORA_PROTOCOL.md). This section enumerates the additions this plan requires.

### 3.1 Already shipped (do not re-add)

- Topic `0x25` `video/tile_delta` — `TileDeltaFrame` body
- Topic `0x26` `video/detections` — tractor detector sidecar
- Topic `0x27` `audio_event` — YAMNet audio sidecar
- Opcode `0x60` `CMD_PERSON_APPEARED` (P0)
- Opcode `0x61` `CMD_ROI_HINT`
- Opcode `0x62` `CMD_REQ_KEYFRAME`

### 3.2 To reserve in [LORA_PROTOCOL.md](LORA_PROTOCOL.md)

| ID | Name | Direction | Purpose | Phase |
|---|---|---|---|---|
| `0x28` | `video/motion_vectors` | tractor → base | Optional optical-flow microframes for degraded mode (Q). | 2 |
| `0x29` | `video/wireframe` | tractor → base | PiDiNet edge map for extreme degraded mode (P). | 2 |
| *(reserved)* | `0x2A` `video/semantic_map` | — | **Reserved for v26**, do not implement in v25. | — |

### 3.3 Badge enum (transmitted as a property of every reconstructed tile)

Adopted from [analysis §15.6 / §16.5](../AI%20NOTES/2026-04-27_Image_Transmission_InDepth_Analysis_ClaudeOpus4_7_v1_0.md). The base attaches the badge; the browser displays it; the browser **must refuse to render any tile whose badge is missing or malformed** (fail-closed per C5).

| Value | Meaning | Visual |
|---|---|---|
| `Raw` | 1:1 pixels, no enhancement applied. | (no badge) |
| `Cached` | Filled from background cache; show age in seconds. | grey badge + age |
| `Enhanced` | Real-ESRGAN super-resolved. | blue badge |
| `Recolourised` | Y-only luma re-coloured from older colour reference. | teal badge |
| `Predicted` | Optical-flow / RIFE pixel-pushed; no fresh data. | amber badge |
| `Synthetic` | LaMa-inpainted or model-generated. | red badge |
| `Wireframe` | PiDiNet edges only, not photographic pixels. | "WIREFRAME" overlay |

Add to LORA_PROTOCOL.md alongside the existing topic table.

### 3.4 Auto-fallback ladder (base-side policy)

From [analysis §13.4 / §14.2](../AI%20NOTES/2026-04-27_Image_Transmission_InDepth_Analysis_ClaudeOpus4_7_v1_0.md), revised per §13.2 Revisit-3 (2026-04-27) to be PHY-aware. The base monitors **rolling 10 s image-link airtime as a percentage of the refresh window** and instructs the tractor (via `CMD_ENCODE_MODE`) which mode to encode in. Airtime is computed locally from the current image-link PHY parameters in `link_monitor.py`, so the thresholds remain correct if the image link ever drops from SF7 to SF8.

Let `T_refresh` = nominal P3 refresh window (default 1500 ms) and `t_air` = measured rolling 10 s mean image-link on-air time per refresh. Define `U = t_air / T_refresh`.

| Image-link utilisation `U` | Mode | Encoder action | Rough byte-equivalent at SF7/BW250/CR4-5 |
|---|---|---|---|
| `U < 25 %` (~< 375 ms airtime) | Full B tile-delta + colour | Default. | ≥ 400 B / refresh |
| `25 % ≤ U < 50 %` (375–750 ms) | Z Y-only luma + 30 s colour reference | Drop chroma; periodic colour ref every 30 s. `Recolourised` badge. | 150–400 B |
| `50 % ≤ U < 80 %` (750–1200 ms) | Q optical-flow microframes (`0x28`) | No new tiles; push existing canvas with motion vectors. `Predicted` badge. | 50–150 B |
| `U ≥ 80 %` (≥ 1200 ms) | P wireframe (`0x29`) | PiDiNet edges only. `Wireframe` overlay. | < 50 B |

The "byte-equivalent" column is the rough envelope at the default SF7/BW250/CR4-5 image profile; `link_monitor.py` does not key off bytes directly. If the image link transitions to SF8 (~1.8× airtime per byte), the same byte counts move into the next-degraded bucket automatically because the airtime % moves with PHY.

**N person-alert (P0) fires at every level**, never gated by image bandwidth.

Reserve a new opcode for the encoder hint: **`0x63 CMD_ENCODE_MODE`** (base → tractor, P2) carrying one of `{full, y_only, motion_only, wireframe}`.

**`link_monitor.py` implementation requirements** (per §13.3 R-8 hysteresis + Revisit-3 measurement plan):

- Compute airtime per fragment via `RadioLib::getTimeOnAir(payload_len)` as fragments leave the queue; sum into a 10 s ring buffer keyed on the current `T_refresh`.
- **Hysteresis: require U to cross a threshold for ≥3 consecutive refresh windows** before sending `CMD_ENCODE_MODE`. Without this the ladder hunts at the boundaries (the same bug R-8 fixes for the SF ladder).
- Surface current `U` and current ladder rung in TelemetryFrame topic `0x10` (`control/source_active`) so the operator UI can show "IMG: full (38 % airtime)" or "IMG: y_only (47 % airtime)". Without this surface, operators cannot tell why the image looks degraded.
- Built alongside Phase-1 deliverables (week 5 of [§8 build order](#8-build-order-10-engineer-week-plan)), not deferred to Phase 2.

---

## 4. Tractor-side software

Lives in `firmware/tractor_x8/image_pipeline/` on the X8 Linux side (not the M7). All targets are CPU-only.

### 4.1 Module layout

| File | Role | Phase |
|---|---|:-:|
| `capture.py` | V4L2 → 384×256 YCbCr buffer per camera. | 1 |
| `register.py` | Phase-correlation image registration vs. previous frame; NEON-accelerated; ~5 % CPU. | 1 |
| `tile_diff.py` | pHash 32×32 tile-change detector, ≤30 ms for 96 tiles. Consumes registered frame. | 1 |
| `roi.py` | Read valve activity from H747 over IPC; classify mode (loading / driving / idle); produce ROI mask; honour `CMD_ROI_HINT`. | 1 |
| `encode_tile_delta.py` | Per-tile WebP at q15/q40/q60 by ROI/detection. Honours `CMD_ENCODE_MODE` (`full` / `y_only` / `motion_only` / `wireframe`). Assembles `TileDeltaFrame`. | 1 (full + y_only), 2 (motion + wireframe) |
| `encode_motion.py` | Optical-flow microframe encoder for `0x28`. | 2 |
| `encode_wireframe.py` | PiDiNet edge encoder for `0x29`. | 2 |
| `fragment.py` | Split into ≤25 ms airtime fragments. | 1 |
| `ipc_to_h747.py` | Hand fragments to M7 firmware ring buffer at P3. | 1 |
| `detect_nanodet.py` | NanoDet-Plus (Apache-2.0), 320×320 INT8, six classes; ≤50 ms p99 on A53s. | 2 |
| `person_alert.py` | On first new high-confidence person/animal/vehicle, emit `CMD_PERSON_APPEARED` (P0); promote next image transmission to P1 for one frame. | 2 |
| `cam_mux.py` | 3-camera attention multiplexer: front=70 / bucket=25 / rear=5 default; reverse-stick flip; person-detection 90 % promotion. | 1 |
| `event_skip.py` | Drop redundant tiles when scene unchanged longer than threshold. | 2 |

### 4.2 Tractor-side validation gates

- **Registration:** drive over rough ground; without registration ≥80 % of tiles flag changed; with registration ≤30 %.
- **`detect_nanodet.py`:** ≤50 ms p99 on the X8 A53s, 30-min sustained, no thermal throttle.
- **Encode + fragment + IPC:** zero P0 starvation events > 25 ms in the 30-min mixed-mode stress run (C1).

---

## 5. Base-side software

Lives in `base_station/image_pipeline/` on the X8 Linux side. Browser-tier offload is mandatory; the base publishes canonical state, the browser renders.

### 5.1 Module layout

| File | Role | Phase |
|---|---|:-:|
| `accel_select.py` | Auto-detect Coral at startup; export `HAS_CORAL`; expose status to UI. | 1 |
| `reassemble.py` | Collect `TileDeltaFrame` (`0x25`) fragments; time out missing fragments; mark stale tiles. | 1 |
| `canvas.py` | Persistent tile canvas; replace changed tiles in place; on `base_seq` mismatch send `CMD_REQ_KEYFRAME` (`0x62`). Per-tile staleness clock. **Attaches badge enum to every published tile.** | 1 |
| `recolourise.py` | Re-colour Y-only tiles from rolling 30 s colour reference; sets `Recolourised` badge. | 1 (week 2.5) |
| `bg_cache.py` | Rolling per-tile median keyed by tractor's segmenter class output; fills holes with `Cached` badge + age. | 1 |
| `link_monitor.py` | Rolling 10 s `bytes/refresh`; emits `CMD_ENCODE_MODE` per the §3.4 ladder. | 1 |
| `superres_cpu.py` | Real-ESRGAN-General-x4v3 via ncnn on A53s; 0.5–1 fps; ≤300 ms/frame; sets `Enhanced` badge. | 2 |
| `superres_coral.py` | Edge-TPU port of Real-ESRGAN; only used iff `HAS_CORAL` and Phase-0 spike passed; ≤30 ms/frame. | 2 (gated) |
| `detect_yolo.py` | Independent base-side safety detector (R6). CPU = YOLOv8-nano OR NanoDet-Plus per AGPL decision. Compares against `0x26` sidecar; logs disagreements. | 2 |
| `motion_replay.py` | Apply `0x28` motion vectors to existing canvas; sets `Predicted` badge. | 2 |
| `wireframe_render.py` | Render `0x29` wireframe over canvas; sets `Wireframe` overlay. | 2 |
| `audio_event.py` | YAMNet event subscriber for `0x27`; UI banner. | 2 |
| `satellite_render.py` | Far-field render from cached satellite tiles, georeferenced to live GPS. | 2 |
| `state_publisher.py` | WebSocket publisher: canvas tiles + per-tile age + per-tile badge enum + detection vectors + safety verdicts + accelerator status. | 1 |
| `fallback_render.py` | Server-side 1 fps render of the canvas, for the HDMI console + headless QA. Same code paths, lower priority. | 1 |

### 5.2 Base-side validation gates

- End-to-end image latency ≤500 ms p99 CPU-only (≤300 ms with Coral).
- `CMD_REQ_KEYFRAME` recovery: induce I-frame loss, fresh I within 1 refresh.
- Coral fallback: yank Coral mid-operation, UI flips to "AI accelerator: offline" within 10 s, pipeline continues degraded.
- Auto-fallback ladder: attenuate the LoRa link in 50 B/s steps; verify the encoder downshifts through `full → y_only → motion_only → wireframe` without operator intervention and without losing the canvas.

---

## 6. Browser tier (mandatory in Phase 1)

Per [analysis §15](../AI%20NOTES/2026-04-27_Image_Transmission_InDepth_Analysis_ClaudeOpus4_7_v1_0.md). The browser is the **third compute tier**; the analysis quantifies a 48 % base-CPU recovery from moving display polish off the base — that's what makes Stack-NoCoral fit on a single i.MX 8M Mini without preempting the safety detector.

### 6.1 Trust boundary (hard rule, do not relax)

> **The base X8 publishes the canonical machine-readable state. The browser is one consumer among many. Any signal a future autopilot might consume must be produced on the base or the tractor — never only in the browser.**

| Stays browser-only (display polish) | Never goes to the browser (authoritative) |
|---|---|
| Per-tile cross-fade animation | Safety cross-check detector (R6) |
| Sub-tile temporal interpolation | ROI generation, valve-state interpretation |
| Staleness-clock visual rendering (badge *value* from base) | Detection-bias decisions |
| Vector / bounding-box overlay rendering (data from base) | Staleness-clock *value* (computed on base) |
| Layout, theming, audio cues, controller vibration | "Enhanced / Cached / Recolourised / Predicted / Synthetic / Wireframe" badge *decisions* |
| GLSL fragment-shader sharpening | Anything that decides whether to send a P0 ControlFrame |

The browser **must refuse to render any tile whose badge is missing or malformed** (fail-closed). Refusal-to-display is logged to the base health endpoint.

### 6.2 Capability floor and test matrix

Target **WebGL 2 + Canvas 2D** as the floor. Anything beyond is opportunistic with WebGL fallback.

| API | Coverage (Apr 2026) | v25 use |
|---|---|---|
| WebSocket binary frames, Canvas 2D, ImageBitmap, ResizeObserver, OffscreenCanvas | ~95–100 % | Yes — required. |
| WebGL 2 fragment shaders | ~99 % (iOS 15+, Android Chrome 80+) | Yes — required for fade/tint/overlays. |
| WebGPU (compute shaders) | ~75 % | Opportunistic only; feature-detect; fall back to WebGL. |
| WebNN | <30 %, flag-gated | **Defer to v26+.** |

Browser test matrix to gate releases:

1. Latest Chrome on a $200 Android tablet
2. Latest Safari on a 2020 iPhone SE
3. Latest Firefox on a Linux laptop
4. Latest Chrome on a Windows laptop

### 6.3 Browser-side modules

Lives in `base_station/web_ui/static/img/`.

| File | Role |
|---|---|
| `canvas_renderer.js` | WebSocket subscriber; per-tile blits to Canvas 2D; OffscreenCanvas in a Worker. |
| `fade_shader.js` | WebGL 2 fragment shader for per-tile cross-fade (~3 display frames, target 60 fps local). |
| `staleness_overlay.js` | Yellow tint + age-in-seconds rendering; consumes per-tile `age_ms` from the base. |
| `badge_renderer.js` | Reads badge enum per tile; renders the §3.3 visual; **fail-closed if missing/malformed**. |
| `detection_overlay.js` | Bounding-box + label rendering from `state_publisher` detection vectors. |
| `accel_status.js` | "AI accelerator: online / offline / degraded" pill, always visible. |
| `raw_mode_toggle.js` | One-click toggle: most-recent-bytes-only view, no enhancement. Choice logged to base audit endpoint. |

---

## 7. Operator UX safety rules (mandatory before any live hydraulic test)

Adopted from [analysis §14.2](../AI%20NOTES/2026-04-27_Image_Transmission_InDepth_Analysis_ClaudeOpus4_7_v1_0.md) and the existing [TODO Phase 5C](TODO.md). Re-stated here so the implementation team has a single rule sheet:

1. **Staleness clock visible on every displayed canvas, always.**
2. **Badge on every non-1:1 pixel** (Cached / Enhanced / Recolourised / Predicted / Synthetic / Wireframe).
3. **Never hide loss** — partial / corrupt canvas shown with gaps visible, never silently extrapolated.
4. **One-click "raw mode" toggle** — most-recent received bytes only, no enhancement. Choice logged.
5. **Audit log:** which view-mode the operator was using when each command was issued; persisted to the [§8.10 black-box logger](MASTER_PLAN.md).
6. **"AI accelerator: online / offline / degraded"** pill visible at all times.
7. **Two-detector disagreement banner:** when tractor and base detectors disagree on a high-confidence object, surface in UI; log for v26 retraining.

---

## 8. Build order (10-engineer-week plan)

From [analysis §14.4 + §15.7 + §16.5](../AI%20NOTES/2026-04-27_Image_Transmission_InDepth_Analysis_ClaudeOpus4_7_v1_0.md). Stack-MVP (steps 1–5) ships first-field-test value at week 4.

| Week | Deliverable | Phase |
|---|---|:-:|
| 1–2 | Tile-delta protocol on the wire (tractor `encode_tile_delta.py`, base `reassemble.py`). Pass the C1 zero-P0-starvation gate. | 1 |
| 2 | **Pre-diff image registration** in `register.py`. Non-negotiable — without it the byte-savings claim collapses the moment the tractor moves. | 1 |
| 2.5 | **Y-only luma encode + recolouriser** (`encode_tile_delta.py --y-only`, `recolourise.py`). Auto-enable below 400 B/refresh. | 1 |
| 2–3 | ROI rings + valve-state policy in `roi.py`. | 1 |
| 3–4 | Persistent canvas + per-tile fade GLSL shader (browser) + staleness clock + Cached badge. `canvas.py` + `state_publisher.py` + browser bundle. | 1 |
| 4 | Operator-cued ROI (`CMD_ROI_HINT` round-trip). **Stack-MVP shippable.** Bench gate. | 1 |
| 4 | **Browser test matrix** (Chrome/Android, Safari/iOS, Firefox/Linux, Chrome/Windows). Phase-1 completion gate. | 1 |
| 4–5 | Background cache (`bg_cache.py`) — rolling per-tile median keyed on segmenter class. Activates fully once M is online (week 7). | 1 |
| 5 | Three-camera attention mux (`cam_mux.py`). Auto-fallback ladder (`link_monitor.py` + `CMD_ENCODE_MODE`). | 1 |
| 5–6 | Real-ESRGAN-General-x4v3 ncnn deploy on the base (`superres_cpu.py`). Benchmark gate ≤300 ms/frame at 0.5–1 fps. UI `Enhanced` badge. | 2 |
| 6 | YOLOv8-nano CPU OR NanoDet-Plus CPU safety detector on base (`detect_yolo.py`). **Decide AGPL stance now.** Two-detector disagreement banner in UI. | 2 |
| 6–7 | YAMNet audio sidecar (`audio_event.py`), event-skip (`event_skip.py`), satellite far-field (`satellite_render.py`). Three small services in parallel. | 2 |
| 7–9 | NanoDet-Plus on the tractor (`detect_nanodet.py`). Benchmark gate ≤50 ms p99. Wire into ROI bias and `0x26` sidecar. | 2 |
| 9 | `CMD_PERSON_APPEARED` (`0x60`) end-to-end at P0. Walk-the-person gate ≤250 ms p99 (C3). | 2 |
| 9–10 | Auto-fallback ladder full validation: attenuate link, watch encoder downshift `full → y_only → motion_only → wireframe`. Implement `encode_motion.py`, `encode_wireframe.py`, `motion_replay.py`, `wireframe_render.py`. | 2 |
| Continuous from week 1 | **Black-box logging** of every canvas, detection, sidecar, and operator action per [MASTER_PLAN.md §8.10](MASTER_PLAN.md). Builds the v26 training set and the v26 Coral-spike retry justification. | — |

### 8.1 Cuts if the schedule shrinks (in order)

1. **First out:** T satellite far-field, R audio sidecar.
2. **Next:** H Real-ESRGAN super-res. (Operator loses Enhanced view; honest-pixel canvas survives.)
3. **Next:** M tractor NanoDet-Plus + N person alert. **Real safety regression — only if the schedule absolutely demands and J base detector ships in compensation.**
4. **Defend with both arms:** B tile-delta, E canvas + I cache, J base safety detector, browser tier. These are the spine.

---

## 9. Validation gates (must all pass before field test)

From [TODO Phase 5D](TODO.md) and [analysis §14.3](../AI%20NOTES/2026-04-27_Image_Transmission_InDepth_Analysis_ClaudeOpus4_7_v1_0.md).

| # | Gate | Target |
|---|---|---|
| V1 | P0 starvation under image load (C1) | 30-min mixed-mode stress, **zero P0 TX-start delays > 25 ms** attributable to image fragments |
| V2 | End-to-end image latency | ≤500 ms p99 CPU-only; ≤300 ms p99 with Coral |
| V3 | `CMD_PERSON_APPEARED` end-to-end (C3) | ≤250 ms p99 walk-the-person test |
| V4 | `CMD_REQ_KEYFRAME` recovery | Fresh I within 1 refresh after induced loss |
| V5 | Coral fallback | UI flips to "AI accelerator: offline" within 10 s; pipeline continues degraded |
| V6 | Auto-fallback ladder | Encoder downshifts cleanly through all four modes without canvas loss or operator intervention |
| V7 | Browser test matrix | All four target browsers render canvas + fade + badges correctly |
| V8 | Two-detector cross-check | Disagreements surface in UI within one refresh; logged to §8.10 |
| V9 | Operator UX safety rules (§7) | All seven rules visible and functional — pre-condition for any live hydraulic test |
| V10 | Trust-boundary fail-closed | Patch a tile to remove its badge enum in transit; browser must refuse to display and log to base |

---

## 10. Open scope decisions (need a human stakeholder)

These two decisions are explicitly NOT LLM-resolvable and must be made by OSE before the relevant build week:

| # | Decision | Deadline | Default if undecided |
|---|---|---|---|
| O1 | **AGPL stance on Ultralytics YOLOv8** for the base safety detector. | Before week 6. | Ship NanoDet-Plus (Apache-2.0); accept ~5 % accuracy reduction. |
| O2 | **Coral on the v25 BOM** — order it for the spike or skip entirely? | Before week 5 (Phase-0 spike must complete by then). | Ship CPU-only Stack-NoCoral as primary; Coral added in v25.5 if a later spike succeeds. |

---

## 11. Cross-document propagation

Files this plan touches; updates needed when this plan changes:

- [LORA_PROTOCOL.md](LORA_PROTOCOL.md) — reserve `0x28`, `0x29`, opcode `0x63 CMD_ENCODE_MODE`; add badge enum table from §3.3.
- [TRACTOR_NODE.md](TRACTOR_NODE.md) — add `register.py`, `recolourise` flag in `encode_tile_delta.py`, `encode_motion.py`, `encode_wireframe.py`, `event_skip.py`, `cam_mux.py`.
- [BASE_STATION.md](BASE_STATION.md) — add `recolourise.py`, `bg_cache.py`, `link_monitor.py`, `motion_replay.py`, `wireframe_render.py`, `state_publisher.py`, `fallback_render.py`; add the §6.1 trust-boundary table.
- [TODO.md](TODO.md) — phase-by-phase tasks (see additions appended in Phase 0 / Phase 5A / Phase 5B / Phase 5D).
- [MASTER_PLAN.md](MASTER_PLAN.md) §8.19 — add reference back to this plan.

---

## 12. Where to read further

| Topic | File |
|---|---|
| Why this stack vs. alternatives | [analysis §14](../AI%20NOTES/2026-04-27_Image_Transmission_InDepth_Analysis_ClaudeOpus4_7_v1_0.md) |
| Why browser-tier offload is mandatory | [analysis §15](../AI%20NOTES/2026-04-27_Image_Transmission_InDepth_Analysis_ClaudeOpus4_7_v1_0.md) |
| Why the four LLM reviews agree on this shape | [analysis §16](../AI%20NOTES/2026-04-27_Image_Transmission_InDepth_Analysis_ClaudeOpus4_7_v1_0.md) |
| Master ranking of all 25 schemes | [analysis §11.2](../AI%20NOTES/2026-04-27_Image_Transmission_InDepth_Analysis_ClaudeOpus4_7_v1_0.md) |
| Coral-only methods (if the spike passes) | [analysis §12](../AI%20NOTES/2026-04-27_Image_Transmission_InDepth_Analysis_ClaudeOpus4_7_v1_0.md) |
| Compatibility matrix between schemes | [analysis §13](../AI%20NOTES/2026-04-27_Image_Transmission_InDepth_Analysis_ClaudeOpus4_7_v1_0.md) |
| Coral policy (no Coral on tractor; spike-gated on base) | [MASTER_PLAN.md §8.19](MASTER_PLAN.md) |
| Hardware BOM Tier-2 Coral lines | [HARDWARE_BOM.md](HARDWARE_BOM.md) |

---

## 13. LoRa PHY revisit (2026-04-27)

Once the image-pipeline byte budget was settled, eight inconsistencies and weaknesses surfaced in the existing LoRa PHY decisions in [LORA_PROTOCOL.md](LORA_PROTOCOL.md) and [MASTER_PLAN.md §8.17](MASTER_PLAN.md). Five are shipped as fixes, one is documented-but-not-adopted, two are queued for implementation alongside existing Phase-1 deliverables.

### 13.1 Shipped fixes

- **Revisit-1 — Image-link PHY split.** The telemetry-link default of SF9/BW250/CR4-8 makes a 32 B image fragment ~120 ms on-air, violating the C1 25 ms-per-fragment cap. **Fix:** added a third "Image link" column to the LORA_PROTOCOL.md PHY table at SF7/BW250/CR4-5 (~22 ms per fragment), independent of the telemetry-link SF. Radio retunes per-frame using the existing `CMD_LINK_TUNE` mechanism (cost bench-measured per R-7). SF8 image is opt-in only because it violates C1.
- **Revisit-2 — Coding rate 4/5 for image.** Tile-delta image already tolerates loss via canvas + `CMD_REQ_KEYFRAME` + the §3.4 auto-fallback ladder. CR4-8 was a ~37 % airtime tax for redundancy the application layer already provides. **Fix:** image link now CR4-5; CR4-8 retained for `0x26` detections and `0x27` audio-event topics where loss would silently hide a safety alert.
- **Revisit-3 — Airtime-percentage ladder thresholds.** The original §3.4 ladder keyed off bytes/refresh, which is PHY-blind — the same byte count costs different airtime on different SF rungs. **Fix:** §3.4 ladder now keys off `U = t_air / T_refresh` (rolling 10 s image-link airtime as a fraction of the refresh window). `link_monitor.py` computes airtime locally via `RadioLib::getTimeOnAir`, applies 3-window hysteresis (R-8), and surfaces `U` + ladder rung on telemetry topic `0x10`. Built in week 5, not week 6.
- **Revisit-5 — FCC §15.247 frequency hopping.** All v25 PHY profiles use BW < 500 kHz, which under §15.247 hybrid-system rules requires either digital-modulation (≥500 kHz) or frequency hopping. **Fix:** documented FHSS approach in [MASTER_PLAN.md §8.17](MASTER_PLAN.md) — 8 channels across 902–928 MHz (3.25 MHz spacing), rotated per refresh window on a deterministic shared sequence seeded by the AES key ID, ~12.5 % per-channel dwell. Avoids the ~3 dB sensitivity loss of widening to BW ≥500 kHz that would cost mast-antenna range margin. Bench-verify duty calc with a spectrum analyser before [Phase 9 FCC verification](TODO.md).
- **R-6 — Telemetry frames inherit the 25 ms airtime cap.** A 64 B SF9/BW250 telemetry frame is ~250 ms on-air — the same C1-violation mechanism that protects P0 from image fragments must also protect it from oversized telemetry. **Fix:** [MASTER_PLAN.md §8.17](MASTER_PLAN.md) bullet added — P2 telemetry obeys the same ≤25 ms-per-fragment cap; oversized topic payloads fragment using the `TileDeltaFrame` scheme; base-station bridge reassembles before MQTT publish.
- **R-8 — Hysteresis on both SF ladders.** The 30 s clean-link rule protects SF↑ → SF↓ from hunting, but the SF↓ → SF↑ path was unprotected — a 2 s interference burst could yank the link to SF9. **Fix:** [MASTER_PLAN.md §8.17](MASTER_PLAN.md) bullet added — require N=3 consecutive bad 5 s windows for any SF transition. Same rule applies to the `CMD_ENCODE_MODE` auto-fallback ladder in §3.4.

### 13.2 Documented but not adopted

- **Revisit-4 — Two-radio split (control on radio A, image+telemetry on radio B).** Documented in [MASTER_PLAN.md §8.17.1](MASTER_PLAN.md#8171-two-radio-split--documented-not-adopted-for-v25) as the v26 escape hatch if browser-tier offload + super-res + two-detector ever pushes us over the C1 budget under stress. **Not adopted for v25** — Stack-NoCoral meets latency targets with the 25 ms fragment cap and the SF7 image profile shipped in Revisit-1. No build action.

### 13.3 Queued for implementation (tracked in [TODO.md Phase 5.0](TODO.md))

- **R-7 — Bench-measure `CMD_LINK_TUNE` retune cost in week 1.** The earlier ~1 ms estimate is unverified. Measure the actual `setFrequency` + `setSpreadingFactor` + `setBandwidth` + `setCodingRate` sequence on the SX1276 via RadioLib. **If > 5 ms**, implement burst-batching of image fragments so the radio retunes at most twice per refresh window (once into image PHY, once back to telemetry PHY) instead of per fragment. Until measured, plan the burst-batching code path conservatively in `lora_proto.cpp`.


---

## 14. Operator toggle (Coral on/off)

The optional Coral Edge TPU is gated by a single operator master switch in the base-station web UI (Settings panel) and persisted to `/var/lib/lifetrac/base_settings.json` via [`base_station/settings_store.py`](base_station/settings_store.py). The dispatchers in [`base_station/image_pipeline/superres.py`](base_station/image_pipeline/superres.py) and [`base_station/image_pipeline/detect.py`](base_station/image_pipeline/detect.py) call `accel.is_active()` per inference, so the toggle and hot-unplug both take effect within one frame. Three-level state (`present` / `usable` / `enabled`) is exposed via `GET /api/settings/accel`; see [`CORAL.md`](CORAL.md) for the operator workflow and the four pill states.

