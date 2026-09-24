# Vector Scene Rung: research, prior art and design review (Claude Opus 5.5, v1.0)

**Date:** 2026-09-22
**Author:** Claude Opus 5.5 (AI analysis, requested by the LifeTrac maintainer)
**Status:** Research record. This note explains *why*. [../DESIGN-CONTROLLER/VECTOR_SCENE.md](../DESIGN-CONTROLLER/VECTOR_SCENE.md) is the source of truth for *what to build*.
**Scope:** A lowest-bandwidth image mode that sends the camera view as layered, gradient-filled vector polygons, with the tractor's own parts drawn at the base from CAD. It is proposed as a new codec and encode mode below `mono_g4` on the shipped strict path; the April `wireframe` rung of the [IMAGE_PIPELINE.md §3.4](../DESIGN-CONTROLLER/IMAGE_PIPELINE.md#34-auto-fallback-ladder-base-side-policy) ladder, which the first draft targeted, is only a legacy alias today.

> **Correction (2026-09-23).** The first version of this note and of `VECTOR_SCENE.md` were written against a checkout from 2026-05-01. `origin/main` was 337 commits ahead (through 2026-09-15) and carries the Murata L072 radio, the strict image path, the FHSS slot-clock design and the July–September bench campaign. After the maintainer pointed this out, the tree was fast-forwarded and the code findings were redone on the current stack. §3 is that second pass; §3.10 lists what the April draft got wrong, including the "10 B per frame" budget. The prior-art survey (§4) is unchanged.

---

## 1. The question

The maintainer asked whether the LoRa image link could fall back to a vector representation when the other image modes fail. The request had these parts:

1. Convert everything in the frame to polygons, each filled with a simple colour gradient that matches the average colour captured.
2. Send sky and ground first, then add detail as bandwidth allows.
3. Show trees and shrubs as basic shapes whose level of detail depends on the available bandwidth.
4. Store a 3D model of the tractor's own visible parts (hood, loader arms, bucket) at the base and render it as an overlay, so the same vectors are not re-sent.
5. Suggest further optimisations.
6. Find other projects that have tried something similar.
7. Show how to build it into the existing firmware and software so it can be tested on the base station website.

Decisions the maintainer made during the session:

- **Deliverable:** research and design documents only; no code yet.
- **Wireframe:** the vector scene **replaces** the wireframe rung. The 12 KB bitmap format is retired, and edges become one layer of the vector scene. *(On the current tree `wireframe` is only a legacy alias for low-quality grey WebP and the real floor is `mono_g4`; VS1 becomes a codec and mode below it, §3.5.)*
- **Self-model:** it must work **with or without** arm/bucket sensors. Start with a static hood mask; use sensed pose when sensors exist; use clearly labelled estimates otherwise.
- **Website testing:** the Vector Lab uses the **live base-station canvas** as its input.

## 2. Method

1. **Code reading.** Four read-only agents mapped:
   - the tractor image path (X8 → M7 → radio);
   - the base path (bridge → reassembly → canvas → publisher);
   - the browser;
   - the CAD, sensor and calibration inputs a self-model would need.
2. **Prior-art research.** Four web-research agents covered:
   - image vectorisation;
   - object-, model-based and semantic codecs;
   - field-robotics and teleoperation analogues;
   - the embedded implementation toolkit.

   Each was told to cite only sources it had actually opened and to mark estimates. Figures that came only from search-result snippets are labelled as such below.
3. **Design panel.** Three independent designs were written from different lenses: MVP-first, bandwidth-optimal codec, and operator safety first. A judge scored them and synthesised one design, using the winner as the backbone.
4. **Adversarial review.** Three reviewers tried to break the synthesised design:
   - bytes and airtime, recomputed with the repo's own `lora_proto.py` functions;
   - CPU feasibility and robustness on real farm scenes;
   - safety, honesty and integration against the actual code paths.

   They raised 60 issues: 3 blockers, 34 major and 23 minor. The final design resolves every blocker and major issue. [Appendix B](#appendix-b-adversarial-review-dispositions) records each disposition.
5. **Rebase (2026-09-23).** Steps 1–4 ran against the 2026-05-01 checkout. After the FHSS bench work surfaced, `origin/main` was fetched and fast-forwarded, and three further read-only agents mapped the current radio stack, the image pipeline and the bench campaign from the repository's own RESULTS.md files and TODO narrative. The design's budget, transport, mode and integration sections were rewritten on that basis; the layer model, record set, temporal rules, self-model and Lab carried over.

All airtime figures below come from `base_station/lora_proto.py` (`lora_time_on_air_ms`) at `PHY_IMAGE_BW250` (profiles 0/1) and `PHY_IMAGE_BW500` (profile 2), SF7/CR4-5, preamble 8 (`lora_proto.py:150-151`), including the 8 B hop and 4 B fragment headers; nothing on the strict path is encrypted.

---

## 3. What the codebase shows (origin/main at `d3751286`, 2026-09-15)

Paths are relative to `LifeTrac-v25/DESIGN-CONTROLLER/`; short names after first use sit under `base_station/`, `base_station/image_pipeline/`, `firmware/tractor_x8/` or `firmware/murata_l072/`, and `DESIGN-STRUCTURAL/` means `../DESIGN-STRUCTURAL/`. Bench-evidence folders are under `bench-evidence/`.

### 3.1 The strict image path and its frame format

The M7 RadioLib path that the April docs describe is off the image path: it is still the default H7 build (`firmware/tractor_h7/tractor_h7.ino:22-26, 136`), but the Max Carrier does not route the Murata SPI pins, so RadioLib cannot reach the radio (`TODO.md:3344`). Since May the radio is the Murata L072 firmware (`firmware/murata_l072/`), driven from the X8 over a COBS host link on `/dev/ttymxc3`:

```
camera_service.py → MQTT cmd/image_frame → image_tx_daemon.py → L072 → air
   → base L072 → image_rx_daemon.py → MQTT video/tile_delta → web_ui.py → /ws/state
```

| Fact | Evidence |
|---|---|
| Image fragments and `0xFB` commands go on air in **plaintext**; no KISS, no AES-GCM on this path | `firmware/tractor_x8/image_tx_daemon.py:36-42`; `base_station/image_rx_daemon.py:12-14` |
| Every routed transmission carries an 8 B hop-sync header (schema, profile, hop_idx, slot_offset, epoch); no MIC | `firmware/murata_l072/include/lora_pkt_hdr.h:10-31`; `SETTINGS_REFERENCE.md` §2.2 |
| Fragment header 4 B (`0xFE`), duplicate-copy header 5 B (`0xFD`), parity 4 B (`0xFC`); body ≤ 247 B | `base_station/lora_proto.py:819-876, 835-836` |
| Per-fragment airtime cap **170 ms**; the April 25 ms cap survives only for telemetry | `base_station/lora_proto.py:833, 837`; `TODO.md` RS-9.7 |
| Fragment bodies **203 B** (profiles 0/1, BW250) and **243 B** (profile 2, BW500) | `image_tx_daemon.py:308`; `SETTINGS_REFERENCE.md` §2.1 |
| `TileDeltaFrame`: 6 B header (`frame_kind`, `seq`, `grid_w`, `grid_h`, `tile_px`, `codec`) + 12 B bitmap + `size-1, blob` per tile; parser rejects codec > 15 and trailing bytes | `base_station/image_pipeline/frame_format.py:6-17, 79-88, 136-200` |
| Crypto profiles exist in code but are not on air: GCM-128 explicit (+28 B, the dead bridge), D13 GCM-64 implicit (+12 B), D14 split-trust for image (+6 B, no MAC) | `base_station/lora_proto.py:287-301, 338-396, 414-445`; `LORA_PROTOCOL.md` priority-class table |

### 3.2 Budget per profile (computed with `lora_proto.lora_time_on_air_ms`)

| Profile | Body | On air (body + 4 + 8) | Airtime | Vector body (body − 6 B header) |
|---|---|---|---|---|
| p1 FHSS BW250 | 203 B | 215 B | 169.1 ms, one per 200 ms slot | 197 B |
| p2 DTS BW500 | 243 B | 255 B | 99.9 ms | 237 B |

The worked vector scene (1,017 record bits, a 129 B body with its 13-bit header) is one fragment of 147 B: 120.4 ms at BW250, 60.2 ms at BW500. The bench measured 699–812 B/s on FHSS and 1.75–2.0 KB/s on DTS as tractor-side goodput (`TODO.md:209-210, 2153-2154`; `bench-evidence/FW_BATCH1_acceptance_2026-07-30/RESULTS.md:86`).

### 3.3 Radio profiles and the FHSS design as tested

| Profile | Modem | Regime |
|---|---|---|
| p0 `BENCH_ONLY_FIXED_915` | SF7/BW250/CR4-5 | Single carrier, bench only |
| p1 `FCC_15_247_FHSS_50CH_BW250` | SF7/BW250/CR4-5 | 50 channels 902.75–927.25 MHz at 500 kHz spacing; 200 ms slots, 12 ms head-start, 15 ms guard; 10 s epoch; 400 ms per channel per 10 s legal dwell; 380 ms per-packet cap; per-link hop seed (CFG 0x17/0x18) |
| p2 `FCC_15_247_DTS_BW500` | SF7/BW500/CR4-5 | Single carrier, PSD-based; 950 ms/s airtime budget; no slot grid |

Sources: `firmware/murata_l072/host/host_cfg_profile.c:193-247`, `include/sx1276_fhss_clock.h:21-78`, `include/sx1276_fhss_chantab.h:20-34`, `SETTINGS_REFERENCE.md` §2.1–2.2.

- **Lock-step follower.** The transmitter anchors its slot clock on its first FHSS transmission; the receiver re-anchors on every accepted header and, while locked, retunes at every slot boundary (`radio/sx1276_rx.c:784-828`).
- **Clock authority (RS-12.15 v2).** A node is the originator after ≥ 8 own transmissions less than 1 s apart; it refuses a lagging echo of its own grid (`include/sx1276_fhss_authority.h:27-62`). Demotion after 2 s without a valid frame; re-acquisition scans 500 ms per channel (`include/sx1276_rx_scan_policy.h:63-91`).
- **Image and commands share the active profile.** The April three-profile split was never built. `AutoRadioPolicy` prefers DTS and falls back to FHSS (`base_station/web_ui.py:378-538`), with a two-phase switch `0x65/0x66/0x67`.

### 3.4 Control plane and command delivery

- The control-first TDMA of `CONTROL_PLANE_DESIGN.md` (200 ms superframe, skip/ditto/full slots of 10.3/15.4/20.5 ms) was designed but **not built**; firmware Batch 2 was demoted to an optimisation (`TODO.md:157-159`).
- As tested: image fragments are paced by time-on-air at 0.92 headroom (`image_tx_daemon.py:355-411`), leaving ~17 ms gaps in which a 10.3 ms command fits; on FHSS a command steals a slot. The base sends commands only on the completion-aligned pump or the idle drain, behind one shared 1.0 s gate while streaming, with exponential retry (`base_station/image_rx_daemon.py:265-273`, `base_station/cmd_timing.py`).
- No acks on actuation: suppressing the tractor echo took in-stream command delivery from 56 % to 91 %, and smooth pacing to 99.8 % (628/629) (`TODO.md:130-159`).
- **Hydraulic control is not on air.** No ControlFrame has flown; Route B (RS-9) is open; E-stop is the 200 ms deadman.
- Base → tractor delivery on FHSS is the weak direction: 1/17 (leg H) and 49/281 (leg I) vs 56/58 on DTS (`TODO.md:2661-2676`).

### 3.5 Image pipeline: what is shipped and proven

| Item | State |
|---|---|
| Codecs on air | WEBP (0) q55 ≈ 26 B/tile; WEBP_LUMA (4) for `y_only`/`motion_only`/`wireframe` (grey WebP at capped quality); MONO_G4 (1) 1-bit dither + zlib, ≥ 14 B/tile, transcoded to WebP at the base; RAWSTREAM (5) container-stripped WebP. BTC4 (2, 3) and ADAPTIVE are not implemented and clamp to `y_only`. |
| Encode modes | `EncodeMode` 0–7 on the base (`lora_proto.py:80-91`); the tractor adds `RAWSTREAM = 8`, which the base rejects (`camera_service.py:501`; `image_rx_daemon.py:1175-1203`). `wireframe` (3) is a legacy alias for grey WebP at q ≤ 20; no wireframe or vector code runs. |
| Mode selection | Operator-only: `_ENCODE_MODE_UI_CHOICES` = full, y_only, motion_only, mono_g4 (`web_ui.py:99-104`), persisted in `.encode_mode_override`, sent as `0xFB 0x63 [mode, quality]`, acked by `0x68` with the codec id, and confirmed by the codec byte of later frames. No automatic encode ladder runs in production (`web_ui.py:91-92`). |
| Encode-to-fit (RS-0.14a) | The byte budget covers the whole wire payload; live budget from retained `tractor/link_budget` (243/203 B, one fragment). Verified on air 2026-07-31: keyframes 2381–2430 B against a 2436 B budget, 472 frames (`bench-evidence/RS_3_3_real_camera_2026-07-30/RESULTS.md:14-26`). |
| Keyframes | Period 10 s; clipped to budget and rotated (Method C); never batched; duplicated when the tractor's cumulative fragment TX-failure rate exceeds 0.5 % (`image_tx_daemon.py:1103-1112`); air loss is invisible to it. Self-heal keyframe requests measured net harmful (+1.88 pts loss, −16 % frames, +31 % timeouts) and replaced by the `0x6C` stale-tile report (F10) and gated (F11). |
| Real camera | Flown on nine bench days (07-31, 08-01, 08-02, 08-22, 09-06, 09-07, 09-12, 09-14, 09-15) at 2 fps (`camera_service.py:109`); no AE/AWB lock. |
| Pacing and loss | Smooth pacing at 0.92 headroom is the default; fragment-10 notch 59 → 2.15 %; DTS camera legs at 0–4.8 % loss (FHSS camera legs 2.1–62.8 %, §3.8); hold on the last fragment of long trains reduces the penultimate loss (RS-12). |

### 3.6 Base rendering path

The April path still exists and is live, now fed by `image_rx_daemon`: `web_ui._ingest_tile_delta` (`web_ui.py:1417-1462`) → `Canvas.apply` transcodes each tile to WebP and assigns the codec's badge (`image_pipeline/canvas.py:35-38, 101`) → `StatePublisher.snapshot` on `/ws/state` (up to 8 clients, `web_ui.py:929, 1635`) → `canvas_renderer.js` (plain 2D context) → `badge_renderer.js` fail-closed (`web/img/badge_renderer.js:20-86`). The April context conflict is fixed; `source_guard.js` samples `#image-canvas`, so overlays must not draw on it. `encode_mode` in the snapshot is never updated (`state_publisher.py:45`).

### 3.7 Self-model inputs (unchanged since April)

- **Geometry exists.** `DESIGN-STRUCTURAL/openscad/lifetrac_v25.scad` is parametric (`loader_arms()`, `bucket_attachment()`, `ARM_LIFT_ANGLE`, `BUCKET_TILT_ANGLE`). No STL/glTF exports; STL was removed on purpose (`DESIGN-STRUCTURAL/REQUIREMENTS.md`).
- **No pose sensor.** The M7 knows only commanded coil bits and a shared flow set-point. Spare Opta analog AI6 rides hydraulics telemetry.
- **No camera calibration.** Front lens 2.8–12 mm varifocal; no camera bracket in the CAD; `CALIBRATION.md` has sections 1–5 only.
- The 2026-04-27 LoRa note deferred the telemetry-driven digital twin until joint sensing, an attachment ID and a < 5° validation exist.

### 3.8 The FHSS and radio bench campaign (July–September 2026)

| Date | Result |
|---|---|
| 07-24 | First working two-peer FHSS link (slot-clock hopping): 699 B/s, about 2× fixed-channel |
| 07-25 | First DTS BW500 link: 1,755 B/s; LoRa-only `0xFB` control plane |
| 07-26 | RS-4.14: the ~50 % FHSS loss was control-plane self-interference (a keyframe-request storm), not follower timing; RS-12.14/12.15 later showed the storm does break follower lock. 30-min DTS soak (run W) 1,873 → 1,846 B/s, 0 TX failures |
| 07-29/31 | Echo off → 91 % command delivery; smooth pacing → 99.8 % |
| 08-01 | Firmware Batch 1 (F6–F9) accepted on air on FHSS: 41–43 decoded frames per 50-slot epoch, 812 B/s |
| 08-02 | RS-11.5: antenna-switch mapping bug (receiver listened through the TX arm); +33 dB recovered |
| 08-16/17 | RS-11.6: two external emitters found; RS-12 bulk floor traced to the short-final-fragment ride; NoParkLast hold 3.3 → 1.8 % loss |
| 09-12 | RS-12.11: base command timing fix, camera loss 3.1 → 1.0 %; RS-12.12: FHSS profile 1 with a quiet command plane 2.1 % loss, with a keyframe storm 62.8 %; RS-12.14: storm bounded to 38.8 % |
| 09-14 | RS-12.15 v2 clock authority: camera A/B 40.2 → 20.9 % loss, lock-loss gaps 2 → 0 |
| 09-15 | Leg U on the shipped build: 537 frames in 268 s, 0 gaps, 10.5 % air loss; leg V: the auto policy's dead-air demotion and 60 s promote-back validated; the FHSS switch reverted (RS-12.19) and the loss input misses lost single-fragment frames (RS-12.20) |

Sources: `TODO.md:126-210, 2432-2470`; `bench-evidence/RS_12_12_fhss_validation_2026-09-12/RESULTS.md:90-121`; `bench-evidence/RS_12_15_clock_authority_2026-09-14/RESULTS.md:300-366`; `bench-evidence/RS_1_4_auto_policy_2026-09-15/RESULTS.md`; also `TODO.md:684-746, 2152-2160, 2273-2283`, `bench-evidence/RS_11_4_train_length_sweep_2026-08-02/RESULTS.md:400-405`, `bench-evidence/RS_12_11_command_timing_2026-09-12/RESULTS.md:142-147`, `bench-evidence/RS_12_14_keyframe_storm_2026-09-12/RESULTS.md:3`.

The record keeps its own retractions (the "171/1" convergence figure, the 44 ms decomposition, the URC-drop explanation, the F4 preamble recommendation, several leg-loss figures); the numbers above are the corrected ones as of 2026-09-15. Open at that date: the ~21 % residual under the camera keyframe storm on FHSS, FHSS reverse delivery, RS-12.16 loss input, RS-12.19/12.20/12.21, crypto (RS-7/RS-8), spectrum measurements, and the drive plane.

### 3.9 What bears on a very-low-bandwidth image mode

- **One fragment per frame is the safe shape.** At the measured 4.3 % fragment loss, a single-fragment frame survives 95.7 % of the time vs 56.5 % for a 13-fragment frame (computed; `CONTROL_PLANE_DESIGN.md:264-265`); the penultimate fragment of a train with a short tail carried 28–42 % of all fragment losses on 13-fragment trains (about 8 % if losses were uniform), so 11–18 % of trains lost it (`bench-evidence/RS_12_bulk_floor_2026-08-16/RESULTS.md:6-11, 207-211`; `TODO.md:2207-2208`).
- **Keyframe-request storms are the FHSS enemy**; a mode with neither keyframe requests nor trains avoids the mechanism entirely.
- **Airtime is quantised** (3.5 B per 1.28 ms step at BW500); frames under ~12 B cost the same 10.3 ms as a skip frame.
- **Any uplink shares the 1.0 s command gate**; in-band repair is free, requests are not.
- **The stale-tile worker** will flood `0x6C` reports if tiles stop updating, so a non-tile mode must gate it.
- **Base → tractor commands may not arrive on FHSS**, so a floor mode needs a tractor-side trigger as well.

### 3.10 What the April draft got wrong

| April claim | Now |
|---|---|
| "~10 B of image data per LoRa frame" | 203–243 B per fragment, one fragment per frame |
| "KISS is transmitted on air", "12 B nonce + 16 B tag per frame" | Neither exists on the strict path; image fragments are plaintext with an 8 B hop header and a 4 B fragment header |
| "The base cannot receive on the image PHY" | Both ends run the same profile; the base receives every fragment |
| "The tractor ignores `CMD_ENCODE_MODE`" | It applies and acks it (demonstrated 2026-08-01); modes 4, 5, 7 clamp to `y_only` |
| "Image bytes never reach the radio" | True only for the stock deploy config (RS-4.8); the bench harness has delivered thousands of frames |
| "Nothing has gone on air", "no bench evidence" | Over 800 bench-evidence folders since May (29 with a RESULTS.md); the numbers in §3.8 |
| "`encode_wireframe.py` is the bottom rung" | `wireframe` (3) is grey WebP at q ≤ 20; the bottom rung is `mono_g4` |
| "`app.js` / `canvas_renderer.js` context conflict" | Fixed |
| "Synthetic gradient camera only" | Real camera flown on nine bench days since 07-31 |
| "The 25 ms fragment cap" | Dropped for image (RS-9.7); the cap is 170 ms |

---

## 4. Prior art

Items marked *(snippet)* were seen only in search results and not opened. Items marked *(est.)* are estimates by the research agents, not measurements. Full URL lists are in [Appendix A](#appendix-a-sources).

### 4.1 Image to vector: primitives, meshes and tracing

| Work | What it does | Licence / maturity | Fit for LifeTrac | Lesson taken |
|---|---|---|---|---|
| **fogleman/primitive** (github.com/fogleman/primitive) | Adds one shape at a time: the shape that most reduces error, found by random candidates plus hill-climbing. Colour is computed, not searched. 50–200 shapes look recognisable. | MIT, Go, about 13k stars | Too slow for per-frame use on an A53. 200 triangles took 21.5 s on 8 desktop cores (primitive-gpu README). | **Greedy best-first order is naturally progressive**: truncate anywhere and it is still the best picture for the bytes. VS1 ranks records by error reduction per bit (CELF). |
| **primitive-gpu `--reuse`** (github.com/Frenzie/primitive-gpu) | Carries the previous frame's shapes forward, re-scores them, and keeps the useful ones | — | Concept only | Persistent shape IDs; send only new or changed shapes. |
| **Geometrize** (github.com/Tw1ddle/geometrize-lib) | C++ primitive-style library with SVG/JSON export | MIT | A compiled extension is possible later, after benchmarking | Licence-clean if hill-climbing is ever needed. |
| **Marwood, Massimino, Covell, Baluja, "Representing Images in 200 Bytes: Compression via Triangulation"** (ICIP 2018, arXiv 1809.02257) | Delaunay vertices on a grid, an 8–16 colour palette at 6 bits per YCoCg channel, adaptive arithmetic coding; connectivity is implicit | Paper | Directly relevant | **Beats JPEG q20 and WebP q10 on PSNR/SSIM at 200 B; WebP catches up at about 400 B.** Geometry beats DCT in exactly our byte range. The palette plus implicit connectivity is the right bitstream shape. |
| **VTracer** (github.com/visioncortex/vtracer) | Hierarchical colour clustering; "stacked" painter's-order shapes, so there are no holes; O(n) pipeline | MIT, Rust; Python and WASM bindings | Tens of ms at 384×256 *(est.)*. No byte-budget control. | Stacked order: if an upper shape is lost, the layer underneath still shows. |
| **Potrace / AutoTrace** | Binary and colour tracing | GPL | Licence review needed | — |
| **SQIP** (perfplanet 2017) | primitive → SVG → blur placeholder: about 800–1000 B as SVG text, 400–600 B gzipped | — | — | SVG text wastes about half the bytes: use a binary format. A browser blur hides polygon edges cheaply. |
| **ThumbHash** (evanw.github.io/thumbhash) | DCT placeholder of about 20–30 B | MIT | Possible real-pixel reality check at E3+ | A 25 B "real pixels" frame is affordable occasionally. |
| **Ardeco** (Lecot & Lévy 2006) *(snippet)*; **Demaret, Dyn & Iske** linear splines over adaptive triangulations | Vector primitives with gradients from an importance-weighted triangulation | Papers | — | "Polygon + linear gradient" is an established representation; importance maps correspond to the ROI and corridor weight. |
| **Diffusion curves** (Orzan et al. 2008); **Mainberger/Weickert** edge + homogeneous-diffusion compression | Send boundaries plus colours on each side; the decoder diffuses | Papers; GPU diffusion runs at 4K > 60 fps (arXiv 2401.06744) | Decoding in the browser is fine; encoding is harder | Optimisation #17: smooth sky and soil with no fill bits. |
| **LIVE** (CVPR 2022), **DiffVG**, **SAMVG**, **StarVector** | Differentiable or learned vectorisers | Apache-2.0 and others | **139–2,609 s per image on an RTX 3090** (SAMVG table) | Offline quality references only. LIVE's "place the next path at the largest-error region" heuristic is cheap to copy. |
| **Temporal Superpixels** (Chang et al. CVPR 2013) *(snippet)*; **Vectorizing Cartoon Animations** (TVCG 2009) *(snippet)* | Superpixel IDs that persist across frames; one background plus moving foreground regions | Papers | Concept | Coherence comes from matching regions over time, not from re-segmenting each frame. |

### 4.2 Codec structure: objects, scene graphs and embedded bitstreams

| Work | Lesson taken into VS1 |
|---|---|
| **MPEG-4 Part 2 object coding**: VOPs, static sprites warped by global motion, 2D mesh animation, resync markers (mpeg.chiariglione.org) | The tractor's own parts are a known object that is never coded. The background is warped by a few global parameters (GSHIFT, GZOOM, IMU). Every packet decodes on its own. |
| **MPEG-4 BIFS / BIFS-Anim** | Define shapes once with persistent IDs, then animate only the fields that change; quantise each field to the precision it needs. |
| **Flash SWF `DefineShape` / `PlaceObject2`** (m2osw.com) | The cheapest update model found: ID + flags + only the changed fields. Depth order = layer order. |
| **SPIHT / EZW** embedded bitstreams; **Priority Encoding Transmission** (Albanese et al. 1994) | Order by importance so every fragment boundary is a valid stopping point, and protect the base layer more heavily (VS1 repeats key frames once). |
| **SSDV** (TT7 LoRa SSDV, tt7hab.blogspot.com) | **Every LoRa packet must decode on its own.** Loss leaves holes, never corruption. Its "Mode 3" (SF7/BW250/CR4-6, about 8.5 kbit/s) is the same class of link as LifeTrac's. |
| **Mapbox Vector Tile 2.1**, **TopoJSON** | Command grammar with zigzag deltas; shared arcs for planar partitions (optimisation #20 at E3+). |
| **Context-adaptive chain coding** (arXiv 2603.03073) | Lossless label maps cost about 2.7 KB at 2 MP, far over budget. The rung must be lossy polygons. |

### 4.3 Generative and semantic codecs (2023–2026)

Byte figures are the agents' conversions to 384×256.

| Codec | Reported rate | Notes |
|---|---|---|
| Text+Sketch (arXiv 2307.01944) | about 0.013 bpp (about 160 B) | ControlNet decoder; "synthesizes different textures or colors" |
| PerCo (arXiv 2310.10325) | down to 0.003 bpp (about 37 B) | 0.67–2.54 s per image on an A100; CC BY-NC-SA |
| PLIC (ECML-PKDD 2025) | 0.004 bpp (about 49 B) | Conditions on text, **Canny edges and a colour palette**, almost exactly the contents of a VS1 frame |
| Extreme video diffusion (arXiv 2402.08934) | 0.02 bpp | Sends a new keyframe when perceptual quality drops below a threshold |
| GVC-RT (arXiv 2608.04891) | < 0.02 bpp | 1080p decode at 55 fps on an RTX 4090 |
| Deep JSCC (arXiv 1809.01733) | — | Analogue channel symbols; not usable on digital LoRa CSS |

**Conclusion.**
- All of these need a GPU decoder, and they produce *plausible* pixels, not true ones.
- On a tractor such output would have to carry the `Synthetic` badge and stay off by default.
- The useful takeaway: because PLIC's inputs match a VS1 frame, a future GPU base could render the *same* VS1 packets generatively with no protocol change.

### 4.4 Field robotics and teleoperation analogues

| Work | Lesson taken |
|---|---|
| **JPL RSVP / HyperDrive** (robotics.jpl.nasa.gov) | Heavy models live on the ground; the link carries state and commands. |
| **NASA "phantom robot"** (Bejczy, Kim, Venema; NTRS 19920000396) | A calibrated CAD robot drawn over delayed video. **Calibrate once, and keep the commanded pose and the measured pose visually distinct.** This is the model for the self-model overlay's pose-source styling. |
| **SeePlusPlus excavator digital twin** (Sensors 2022, PMC9571626) | Orientation sensors on the boom, stick and bucket drive the model: bucket error 0.06 ± 0.05 m. It needs matching joints *and* matching link lengths. A CAD overlay driven only by valve commands would only ever be "predicted". |
| **US10425622B2** (US Army, 2017) | Latency compensation that splits the image into a ground plane and a far plane, each with its own transform. Maps onto VS1's `far`/`ground` groups. Its claims need review before use. |
| **US9519286B2** (Robotic Research) | Warps a Delaunay mesh driven by vehicle state. A vector frame is already a mesh. |
| **TerraSentia+ delay compensation** (arXiv 2409.09921) | Tested by replaying recorded rosbags with simulated delay. It inpaints disocclusions; LifeTrac must show "NO DATA" instead (rule C6). |
| **STRIPE** (CMU 1995–97); **Mellinkoff et al. 2017** | Under these constraints it is "impractical to put an operator behind a steering wheel", and **5 fps is the minimum** for effective teleoperated exploration. At about 0.67 fps VS1 is a supervisory "look, then move" tier, so a speed cap is part of the design. |
| **Shamshiri et al. 2024**, LoRa + digital shadow for an agricultural robot | The closest precedent: pose and obstacle state over LoRa rendered as a digital shadow, **about 12 % loss at 2.3 km**. VS1's design loss target comes from this. |
| **FleetAgent** (arXiv 2606.21222) | Vectorised messages instead of images: up to 625× less uplink. |
| **"Every Move You Make"** (CHI 2026) | A predicted-path overlay under 2.56 s delay cut task time and workload; network-timeline widgets did nothing. |
| **Iridium SBD, Meshtastic, StuartCAM, SSTV** | Nobody ships pixels at this size. Per-packet ACKs double the traffic, so VS1 uses idempotent re-sends instead. |
| **Skyline and horizon detection** (arXiv 2107.10997, 2110.13694; UAV Otsu + Hough) | Horizon extraction is cheap on a CPU. Invert it: predict the horizon from the IMU and send only the residual (optimisation #13). |

### 4.5 Embedded toolkit on the i.MX 8M Mini (4× A53, no NPU)

| Tool | Reported cost | A53 at the VS1 working resolution *(est.)* | Use in VS1 |
|---|---|---|---|
| `cv2.resize`, `findContours` + `approxPolyDP` | Standard | 1–3 ms on a 96×64 label map | L1 contours |
| `cv2.kmeans` (K = 8, 3 iterations, label-seeded) | — | 5–15 ms at 96×64 | L1 colour clusters |
| ExG−ExR vegetation index (Meyer & Neto 2008) *(snippet: about 0.87 accuracy)* | One numpy pass | 3–6 ms at 384×256 | L1/L2 vegetation, after white-balance normalisation |
| fast-slic (NEON path) | 20 ms at 640×480 on x86 | 20–40 ms | Alternative L1 segmenter |
| SEEDS / Felzenszwalb | 160 ms at 512×1024 on CPU (SSN table) | 30–150 ms | Alternative L1 segmenter |
| EDLines / EDPF (ED_Lib, MIT; in OpenCV contrib) | 9.45 ms (paper; size not given) | 10–40 ms | L3 edges (optimisation #14) |
| `calcOpticalFlowPyrLK` on 30–50 corners | — | 2–4 ms at 192×128 | GZOOM ground-motion estimate |
| MobileNetV3-Small + LR-ASPP (fastseg, MIT) | 327 ms at 512×1024 on one Pixel 3 big core | 60–150 ms at 384×256 on 4 threads | Optional later upgrade; competes with NanoDet for CPU |
| ncnn MobileNetV3 at 224 on a Pi 3B+ (A53 at 1.4 GHz) | 86.4 ms | — | Anchor for A53 neural-net cost |

**Browser side:**
- Canvas2D `Path2D` + `createLinearGradient` is sufficient, at under 1 ms for about 100 shapes *(est.)*.
- earcut and d3-delaunay are available if needed.
- three.js with `STLLoader`/`GLTFLoader` can load CAD. The segments.ai camera-intrinsics recipe maps K to a three.js projection.
- VS1 instead projects on the base with `cv2.projectPoints`, which handles lens distortion natively. It sends 2D polygons, and three.js is deferred to Phase 4.

### 4.6 The prior art, distilled

1. **Send the situation, not pixels.** Everything that works at kbit/s rates sends state or vectors: RSVP, STRIPE, FleetAgent, Shamshiri, SBD users.
2. **Geometry beats DCT below about 400 B** (Marwood et al.), which is exactly the vector rung's range.
3. **Greedy best-first ordering makes any truncation useful** (primitive, SPIHT, PET).
4. **Every packet must decode on its own** (SSDV, MPEG-4 resync). Assume more than 10 % loss.
5. **Define once, then update only the fields that change** (SWF, BIFS). Move groups with global motion (MPEG-4 sprites, GSHIFT).
6. **A CAD self-overlay is only as good as its calibration and joint sensing** (phantom robot, SeePlusPlus). Keep measured and predicted visually distinct.
7. **At under 5 fps the operator supervises, not drives** (STRIPE, Mellinkoff).
8. **Generative decoders are GPU-bound and invent content.** Keep them out of anything safety-relevant.

---

## 5. How the research shaped the design

| Lesson | VS1 design element ([VECTOR_SCENE.md](../DESIGN-CONTROLLER/VECTOR_SCENE.md)) |
|---|---|
| One-fragment frames survive 95.7 % vs 56.5 % for trains; the penultimate-fragment ride | One `TileDeltaFrame` (codec 6) per frame, sized to the live 203/243 B budget; no trains and no envelope ladder (§1, §3.1) |
| SSDV self-contained packets | Frame-atomic records; every frame decodes alone; the encoder repeats the epoch start itself, because `image_tx_daemon`'s duplicate fires only on its local TX-failure rate, which does not track air loss (§3.1–3.2) |
| primitive / SPIHT embedded order | CELF lazy greedy ranking of ΔD per bit against a mirror of the base display (§2.9) |
| Marwood palette + grid + implicit structure | Static farm palette plus measured RGB444, grid-quantised EG2 vertex deltas, triangle-first polygons (§3.3) |
| VTracer stacking | Painter's order: L0 → L1 (area-descending) → L2 → L4 → L3 (§2.2) |
| SWF/BIFS define-and-update | Persistent IDs; UPD, UCOL, DEL, CONFIRM and DIGEST records (§3.3, §4.3) |
| MPEG-4 sprites, US10425622B2 ground/far split | GSHIFT groups, GZOOM ground expansion, IMU far-field warp at the base (§4.4) |
| Horizon detection + IMU | L0 with chroma/texture ordering, an IMU prior, and a 15-bit residual (§2.4) |
| ExG−ExR, k-means, contours | Classical L1/L2 at 96×64: no neural net needed for v25 (§2.5–2.6) |
| Phantom robot, SeePlusPlus | Pose sources S0–S4 with distinct styles; calibration plan; bucket never masked (§5) |
| STRIPE, 5 fps floor | Supervisory tier; the speed cap is recorded as a requirement on the pending hydraulic drive plane (§6) |
| Shamshiri 12 % loss; FHSS legs at 2–21 % | Repeat-once + re-verified carousel: lost defines 12 % → 1.44 % (§4.3) |
| TerraSentia+ replay testing | Vector Lab with seeded loss models; `vector_bench.py` replay on the tractor X8; a bench leg in the campaign's RESULTS.md convention (§8) |

---

## 6. Design panel

Three designs were written independently. The judge scored each from 1 to 10.

| Lens | Feasibility | Bytes | Operator value | Safety / honesty | Integration | Fidelity to request | Total |
|---|---|---|---|---|---|---|---|
| MVP-first | 7 | 6 | 7 | 7 | 8 | 9 | 44 |
| **Bandwidth-optimal codec (winner)** | 7 | **9** | **8** | 7 | 8 | 8 | **47** |
| Operator safety first | 6 | 6 | 8 | **9** | 8 | 7 | 44 |

**Why the codec lens won.** It was the only proposal that noticed **KISS framing was transmitted on air** on the April M7 path (the strict path has none, §3.10). The other two assumed 13 B per frame, and their anchor records did not fit. It also contributed:
- a complete prefix code;
- the envelope ladder (E0–E4 plus the S4 alternative; withdrawn in the rebase, design §11);
- progressive polygons (VW order + INSERT);
- GSHIFT motion groups and far/ground reprojection;
- orphan records as an implicit NACK;
- rejecting a new P2 pose topic (213 ms on the telemetry PHY).

**Grafted from the MVP lens:**
- measured-mean fills instead of palette-only colour;
- ellipses for trees and shrubs;
- repeat-once plus carousel;
- a loss-aware asymmetric ladder;
- the hand-drawn static mask;
- `feed_canvas.py` and the Lab design.

**Grafted from the safety lens:**
- capture age in every frame (now written by `camera_service`; the base treats it as a lower bound, design §3.2);
- last-writer-wins by capture time;
- CONFIRM tags;
- the self-mask anomaly test;
- dynamic masking only from a sensed pose;
- "NO DATA" hatching;
- the banner drawn inside the vector canvas, so raw mode cannot hide it;
- the speed cap (now a requirement on the pending drive plane, design §6);
- HIL gate W4-11 (now the Phase 2 bench leg, design §8.6).

**Kept from the maintainer's request over the safety lens's preference.** The safety lens put the hazard layer before the scenery. The synthesis keeps sky → ground → trees → detail as the progression, but puts corridor anomalies (ANOM) at the top of the budget order and the paint order whenever one is present.

**What the adversarial review changed most:**

| Issue | Change |
|---|---|
| b1: deterministic nonce bytes cause correlated KISS-escape blackouts | F reduced from 11 to 10 |
| s1: the base cannot receive on the image PHY | Base-granted image RX windows (B13) |
| s2: a dynamic mask could hide something in the bucket | The bucket is never masked |
| c1: a Lab encode inside uvicorn would stall `/ws/control` | Sidecar worker |
| b3/s6: INSERT depended on arrival order | Order-independent INSERT |
| s3: no person detector exists | "SAFETY DETECTOR: NO PIXELS" chip, and field use conditioned on B8 |

**Scope of that review.** The panel and the three reviewers worked against the 2026-05-01 checkout. Their transport findings (b1, b4, b9, b10, b17, s1, s11, s16, s20 and the framing of c1) concern the retired M7 path and were superseded by the rebase; the codec, honesty, self-model and Lab findings carried over unchanged.

---

## 7. Relationship to earlier notes

- **[2026-04-27_Image_Transmission_InDepth_Analysis_ClaudeOpus4_7_v1_0.md](2026-04-27_Image_Transmission_InDepth_Analysis_ClaudeOpus4_7_v1_0.md) §10.5** proposed 4 B line segments, 30–40 per scene, in about 150 B. VS1 generalises this:
  - edges become L3;
  - polygons, fills, persistence and layering are added;
  - the budget is one fragment per frame: up to 197 B (FHSS) or 237 B (DTS) of VS body (§3.2).
- **The same note's §10.7** (semantic-map mode, `0x2A`) stays reserved for v26. VS1 does not use `0x2A`.
- **[Gemini 3.1 Pro §3.1](2026-04-27_Image_Transmission_Analysis_Gemini_3.1_Pro_v1.0.md)** ("Tron-style" vector lines, 20–30 lines in under 100 B) is the likely origin of the "vector lines" suggestion that prompted this request.
- **[Copilot Portenta AI §7.4 / §10.3](2026-04-27_Image_Transmission_Portenta_AI_Analysis_Copilot_v1_0.md)** (edge/vector transmission, vector-only implement mode) is folded in as L3 and the self-model.
- **[CODE REVIEWS/2026-07-23_LoRa_Comm_and_Image_TX_RX_Optimization_Review_Copilot_v1_0.md](CODE%20REVIEWS/2026-07-23_LoRa_Comm_and_Image_TX_RX_Optimization_Review_Copilot_v1_0.md)**, the 2026-07-25 roadmap and keyframe-elimination notes, and [`CONTROL_PLANE_DESIGN.md`](../DESIGN-CONTROLLER/CONTROL_PLANE_DESIGN.md) define the strict path VS1 rides on. VS1 takes the keyframe-elimination idea to its limit: a mode with no keyframe trains at all.
- **[2026-05-25_Grayscale_Quantization_Encoding_Research_Copilot_v1_0.md](2026-05-25_Grayscale_Quantization_Encoding_Research_Copilot_v1_0.md)** produced `mono_g4`, today's floor; VS1 sits below it on the ladder but shows the whole scene in one fragment, where a 243 B `mono_g4` frame carries at most 16 of the 96 tiles.
- **[TODO.md Phase B "Situational overlay"](../DESIGN-CONTROLLER/TODO.md)**: "don't compress video, compress the situation", with a synthetic view from structured data and CAD labelled as synthesized. VS1 is a concrete, byte-level realisation of that item, with real measured colours instead of a purely synthetic view.
- **[2026-04-27_LoRa_InDepth_Analysis_ClaudeOpus4_7_v1_1.md](2026-04-27_LoRa_InDepth_Analysis_ClaudeOpus4_7_v1_1.md)** deferred the telemetry-driven digital twin. Proposed decision D-VS7 partially supersedes that:
  - the console may show the static hood (S0) and a sensed arm (S1);
  - vision (S2) and valve dead-reckoning (S3) poses, and any bucket model, stay Lab-only until that note's re-open criteria are met.

## 8. Open questions for OSE

Copied from design §11 (2026-09-23 revision).

1. Frames published, loss and coverage for VECTOR vs `mono_g4` on the same camera workload, on p1 and p2 (the Phase 2 bench leg).
2. Does the tractor self-select (D-VS6b) trigger correctly from tractor-side signals alone (received-command SNR and command silence, relayed by `image_tx_daemon`), and does it ever flap?
3. Which camera sees the loader, and at what zoom? This sets the minimum-object table.
4. Buy the AI6 arm sensor, and a bucket sensor (which grows `0x04` from 12 to 14 B)? (D-VS5)
5. Detail-level mapping of the quality byte: linear in INSERT budget, or in residual threshold?
6. Handheld vector view?

---

## Appendix A: Sources

These are the URLs the research agents reported opening on 2026-09-22, grouped by research track. Pages that failed to load (403, paywall or unparseable PDF) are not listed. Claims that rest only on search snippets are marked *(snippet)* in §4.

### A.1 Image-to-vector and primitive approximation

- <https://github.com/fogleman/primitive>
- <https://github.com/Tw1ddle/geometrize-lib>
- <https://github.com/visioncortex/vtracer>
- <https://www.visioncortex.org/vtracer-docs>
- <https://arxiv.org/abs/1809.02257>
- <https://ar5iv.labs.arxiv.org/html/1809.02257>
- <https://ma-xu.github.io/LIVE/>
- <https://rogerjohansson.blog/2008/12/09/genetic-programming-mona-lisa-faq/>
- <https://arxiv.org/abs/2306.06441>
- <https://evanw.github.io/thumbhash/>
- <https://calendar.perfplanet.com/2017/sqip-vague-vectors-for-performant-previews/>
- <https://arxiv.org/abs/2406.05404>
- <https://github.com/Algy/fast-slic>
- <https://arxiv.org/abs/2311.05276>
- <https://arxiv.org/html/2311.05276>
- <https://github.com/BachiLi/diffvg>
- <https://blog.forret.com/2021/01/14/cpu-benchmark-apple-silicon-m1/>
- <https://github.com/Frenzie/primitive-gpu>
- <https://scikit-image.org/docs/stable/auto_examples/segmentation/plot_segmentations.html>
- <https://inria.hal.science/inria-00105620/en/>
- <https://cg.cs.tsinghua.edu.cn/papers/TVCG_2009_cartoon.pdf>

### A.2 Object-, model-based and semantic coding

- <https://mpeg.chiariglione.org/standards/mpeg-4/video.html>
- <https://mpeg.chiariglione.org/standards/mpeg-4/scene-description-and-application-engine.html>
- <https://www.m2osw.com/swf_tag_placeobject2>
- <https://cris.tau.ac.il/en/publications/image-compression-by-linear-splines-over-adaptive-triangulations>
- <https://github.com/ma-xu/LIVE>
- <https://orbit.dtu.dk/en/publications/edge-based-compression-of-cartoon-like-images-with-homogeneous-di/>
- <https://artis.inrialpes.fr/Publications/2008/OBWBTS08/>
- <https://arxiv.org/abs/2401.06744>
- <https://arxiv.org/abs/2401.06747>
- <https://ris.uni-paderborn.de/publication/3040>
- <https://nvlabs.github.io/face-vid2vid/>
- <https://arxiv.org/abs/2209.10507>
- <https://www.usenix.org/conference/nsdi24/presentation/sivaraman>
- <https://arxiv.org/abs/1809.01733>
- <https://arxiv.org/html/2307.01944>
- <https://arxiv.org/html/2310.10325v2>
- <https://arxiv.org/abs/2402.13536>
- <https://mlanthology.org/ecmlpkdd/2025/hassan2025ecmlpkdd-qualitypreserving/>
- <https://arxiv.org/abs/2505.16177>
- <https://arxiv.org/abs/2402.08934>
- <https://arxiv.org/html/2608.04891>
- <https://arxiv.org/abs/2604.12525>
- <https://arxiv.org/abs/2602.05213>
- <https://github.com/google/draco>

### A.3 Field robotics and teleoperation

- <https://arxiv.org/abs/2606.21222>
- <http://tt7hab.blogspot.com/2018/04/the-lora-ssdv.html>
- <https://stuartsprojects.github.io/2021/11/21/StuartCAM-ESP32CAM-Picture-Transfers-with-LoRa.html>
- <https://github.com/cotteux/MeshFile>
- <https://docs.groundcontrol.com/iot/rockblock/iridium>
- <https://bruxy.regnet.cz/web/hamradio/EN/compare-resolution-of-sstv-modes/>
- <https://www-robotics.jpl.nasa.gov/what-we-do/flight-projects/mars-2020-rover/rsvp-mars-2020/>
- <https://robotics.jpl.nasa.gov/what-we-do/flight-projects/mars-exploration-rovers/interfaces/>
- <https://ntrs.nasa.gov/citations/19920000396>
- <https://pmc.ncbi.nlm.nih.gov/articles/PMC9571626/>
- <https://driescardinaels.be/papers/every-move-you-make/>
- <https://patents.google.com/patent/US10425622B2/en>
- <https://patents.google.com/patent/US9519286B2/en>
- <https://arxiv.org/abs/2409.09921>
- <https://arxiv.org/html/2409.09921>
- <https://arxiv.org/abs/2211.11918>
- <https://publications.ri.cmu.edu/operator-interface-design-issues-in-a-low-bandwidth-and-high-latency-vehicle-teleoperation-system>
- <https://ar5iv.arxiv.org/html/1706.03752>
- <https://researchportal.hw.ac.uk/en/publications/internet-of-robotic-things-with-a-local-lora-network-for-teleoper/>
- <https://arxiv.org/abs/2605.15952>
- <https://arxiv.org/abs/2107.10997>
- <https://arxiv.org/abs/2110.13694>
- <https://arxiv.org/abs/2402.07556>
- <https://pmc.ncbi.nlm.nih.gov/articles/PMC8749809/>
- <https://bluerobotics.com/remote-control-of-a-bluerov-using-underwater-acoustic-transmissions/>
- <https://arxiv.org/abs/2510.27324>
- <https://arxiv.org/abs/2605.09670>

### A.4 Embedded implementation toolkit

- <https://arxiv.org/abs/1309.3848>
- <https://ar5iv.labs.arxiv.org/html/1807.10174>
- <https://ar5iv.labs.arxiv.org/html/1905.02244>
- <https://arxiv.org/abs/2204.05525>
- <https://github.com/hustvl/TopFormer>
- <https://arxiv.org/abs/2304.05152>
- <https://arxiv.org/abs/2103.12417>
- <https://arxiv.org/abs/2307.10267>
- <https://arxiv.org/html/2607.06600>
- <https://arxiv.org/html/2603.03073>
- <https://github.com/InterDigitalInc/LosslessSegmentationMapCompression>
- <https://arxiv.org/abs/2408.15741>
- <https://arxiv.org/html/2605.21136v1>
- <https://github.com/ekzhang/fastseg>
- <https://github.com/CihanTopal/ED_Lib>
- <https://davidstutz.de/implementation-of-felzenswalb-and-huttenlochers-graph-based-image-segmentation/>
- <https://github.com/k29/horizon_detection>
- <https://raw.githubusercontent.com/Tencent/ncnn/master/benchmark/README.md>
- <https://github.com/ARM-software/armnn/issues/784>
- <https://gist.github.com/cocoa-xu/520dd745e82ce7bf1a7398aed25d97fe>
- <https://pypi.org/project/onnxruntime/>
- <https://pypi.org/project/opencv-contrib-python-headless/>
- <https://pypi.org/project/constriction/>
- <https://github.com/rygorous/ryg_rans>
- <https://github.com/mapbox/vector-tile-spec/blob/master/2.1/README.md>
- <https://github.com/d3/d3-delaunay>
- <https://developer.mozilla.org/en-US/docs/Web/API/CanvasRenderingContext2D/createLinearGradient>
- <https://segments.ai/blog/simulating-cameras-three-js/>
- <https://gltf-transform.dev/cli>
- <https://github.com/avbentem/airtime-calculator>

---

## Appendix B: Adversarial review dispositions

*Historical: this is the 2026-09-22 review of the April-stack draft. Items about the M7 transport (nonce, KISS, envelope, base RX PHY, tile-rung feasibility) are superseded by the rebase; see §6.*

Section numbers (§) refer to the 2026-09-22 draft of [VECTOR_SCENE.md](../DESIGN-CONTROLLER/VECTOR_SCENE.md); rows superseded by the rebase point at sections it rewrote. Issue IDs: b = bytes/airtime reviewer, c = CPU-feasibility reviewer, s = safety/integration reviewer.

**Key:** F = fixed, P = partially adopted (reason given), R = rejected (reason given).

### Bytes lens

| # | Issue | Disposition |
|---|---|---|
| b1 | Deterministic nonce prefix makes F = 11 lose about 1.6 % in blackouts | **F.** E0 is now F = 10 with transmit-time seq skip, re-roll ×6 and a ≤ 1 s hold. Recomputed with all 4 cycling fixed bytes: F = 11 would lose 3.1 % (1.57 % with seq skip); F = 10 loses 0.0063 % (§1). SIL walks every seq/t byte. |
| b2 | Padding vs truncation rules contradict | **F.** An all-zero remainder stops decoding first; goldens cover 0–16 bits (§3.4). |
| b3 | INSERT depends on arrival order | **F.** Addressed as define edge + k, relative to the define-edge midpoint; same-hash re-sends keep INSERTs; DIGEST includes n_inserts (§3.3). |
| b4 | F = 9 fallback cannot carry L1 | **F.** No F = 9 mode exists. F = 10 works even before the re-roll (4.7 % loss), and every record fits in 69 bits. |
| b5 | GSHIFT range and epoch wrap deadlock | **F.** GSHIFT is dx8/dy7 (21 bits). Epoch acceptance adds a K flag, capture time, a 3 s silence rule and self-heal (§3.5). |
| b6 | Carousel re-sends get false fresh ages | **F.** The tractor re-verifies every re-send; per-field LWW; same-hash defines do not reset offsets (principle 2, §3.4, §4.3). |
| b7 | N = 1 A/B arithmetic | **F.** New templates (§4.2); ABS exempt from forcing; completion restated as 27 s static and about 50 s moving (computed). |
| b8 | "8 s at N = 12" | **F.** 3 s at N = 12, 7.5 s at N = 4 (recomputed for 18 frames). |
| b9 | Tile comparison understated | **F.** F − 4 per fragment: 25 frames at E0, 17 at E1, 13 at E2. |
| b10 | Sizer wording | **F.** "Counts 9 B, omits GCM + KISS; returns 37, truth 7 (6 with escape slack)" (§7.1). |
| b11 | Field ranges | **F.** UPD −32..+28; CONFIRM count−1; v0 per-axis ranges with y in-frame; HZN y8 offset; SKYLINE scale sc2; 24 samples only at E2+ (asserted). |
| b12 | vfill label and RESID reference | **F.** vfill is 9 or 17 bits; ABS 41–57; RESID is relative to the epoch ABS or IMU prediction. |
| b13 | TX-done saving | **F.** About 98 ms (122 → 24.4 ms). |
| b14 | Credits on `0xC1` | **F.** New opcode under `0xC0`. |
| b15 | L overstated by tractor drops | **F.** Transmit-time seq; L_img from granted slots; drop counters on `0x06`. |
| b16 | byte0 KISS rationale | **F.** Removed; the marker is 1 bit and exists only for 0xFE avoidance. |
| b17 | S4 row unquantified | **F.** 102 B explicit, 105 B implicit (computed); F ≈ 63–68. |

### CPU lens

| # | Issue | Disposition |
|---|---|---|
| c1 | Lab blocks the event loop and control | **F.** Sidecar worker (cpus 1, SCHED_IDLE), 202 + poll, work cap 16, shared extraction, queued while sticks are active, auto-pause, decode outside the lock, latency test (§8.2). |
| c2 | Lab `encode_ms` is not a tractor figure | **F.** Labelled indicative, with thread time; the gate comes only from tractor `vector_bench.py` with a concurrent workload and a 30-minute soak. |
| c3 | Live canvas is stale and seamed | **F.** Tile-metadata weight map, seam suppression, age display, `still` source, "vs decoded canvas" labelling. The limitation is stated in §8.3. |
| c4 | Forward-driving expansion | **F.** GZOOM (additive u, idempotent), LK estimation, prediction before matching, separate drive budget. |
| c5 | `register.py` sign and confidence | **F.** B14 plus a sign/magnitude test; the "moving" threshold is in 384 px units. Verified in code: `cross = fa*conj(fb)`, raw peak (`register.py:61-74`). |
| c6 | L0 luma ordering, fisheye sag, bucket, no sky | **F.** Chroma/texture ordering, undistorted points or quadratic, swept-envelope exclusion, temporal gate, NO_HORIZON as normal. |
| c7 | ExG−ExR trees and white balance | **F.** Tree = darker, textured, touching the horizon; treeline → SKYLINE; WB normalisation + Otsu; trunk only when fx > 300. |
| c8 | Holes painted over / 6-cell floor | **P.** RETR_CCOMP, HOLE records, 2-cell corridor floor at 192×128, and the published minimum-object table are adopted. "Hatch unreported holes" is **rejected**: the base cannot know about a hole it was never sent. That residual is disclosed instead (§2.3, principle 4). |
| c9 | Canny thresholds, contour flood, edge matching | **F.** Sobel percentiles, top-K pruning, chamfer matching. |
| c10 | Packer cost underestimated | **F.** CELF on a 48×32 mirror, ≤ 64 candidates; the gate is end-to-end M→P. |
| c11 | Geometrize search in Python | **F.** Closed-form moments or Delaunay; compiled extension only after benchmark. |
| c12 | Intensity MAD anomaly; no dynamic-mask check | **F.** Structural NCC/chamfer, rolling reference, 2-of-3 hysteresis, the test applied in every mask, 2-cell erosion. |
| c13 | S3 dead-reckoning physics | **F.** Integrate only on the sole active coil; freeze and widen otherwise; stricter resync; swept band; unsourced figure removed; Lab-only by default. |
| c14 | S2 IoU on thin edges | **F.** Chamfer on a distance transform of unmasked edges, seeded search, margin confidence, no masking until validated. |
| c15 | k-means warm start and exposure flood | **F.** Label-seeded kmeans, GAIN record, AE/AWB lock. |
| c16 | SKYLINE height and pixel IMU band | **F.** sc2 scale; band ±3° in angle. |
| c17 | Corridor shadow | **F.** Chromaticity 2-component trimmed GMM, envelope exclusion, suppression of tractor-fixed regions. |

### Safety lens

| # | Issue | Disposition |
|---|---|---|
| s1 | Base RX PHY missing | **F.** B13 plus base-granted image windows (§7.4); Phase 2 exit; W4-11 measures received frames. |
| s2 | Dynamic mask hides bucket hazards | **F.** Bucket and cutting edge are never masked; arm-only masking with a ≤ 50 ms capture-synchronous, stationary pose, a structural test and 2-cell erosion; attachment "unknown" by default; SIL test for an object inside the bucket. |
| s3 | No person detector exists | **F.** Stated (principle 6, B15); "NO PIXELS" chip; field-use condition in D-VS4; corridor floor lowered; corr_n in STATUS. |
| s4 | Axis caps are ineffective with shared flow | **F.** M7 flag caps the shared set-point; the base uses one factor for all axes; SIL test. |
| s5 | Anomaly cannot fit at E0 | **F.** ANOM record (37 bits) in every frame when corr_n > 0; ANOM + STATUS = 67 ≤ 69. **R:** putting corr_n in the header or a STATUS-lite; ANOM carries the location, which matters more. |
| s6 | INSERT order dependence | **F.** Same fix as b3. |
| s7 | CONFIRM without a state tag | **F.** tag2 per confirmed id; no age reset on mismatch or while DIGEST mismatches. |
| s8 | Epoch reboot and outage; 50 % rule | **F.** §3.5 rules; hand-over uses DIGEST n_live. |
| s9 | Parser padding | **F.** Same as b2. |
| s10 | Nonce prefix | **F.** Same as b1. |
| s11 | L polluted by tractor drops and mixed PHYs | **F.** Transmit-time seq, L_img from slots, drop counters. **R:** the 4-bit P3 header counter; granted-slot accounting gives image-PHY loss for 0 bits. |
| s12 | Lab inject and feed_canvas contaminate the console | **F.** Separate `lab/*` topics and stores; no console inject; refusal gating; audit. |
| s13 | Lab CPU stalls control | **F.** Same as c1. **P:** runs are *queued* while sticks are active rather than refused whenever a session exists, so the Lab stays usable at the bench. |
| s14 | Unbounded mask route; 4-bit hash | **P.** Bounds, confirm, audit, console outline and params-service transport are adopted. In-band hashes are raised to 16 + 16 bits, and the full SHA-256 is checked at install. **R:** sending ≥ 32-bit hashes on a P2 topic, because P2 on the telemetry PHY costs ≥ 213 ms per frame (computed). |
| s15 | The re-open criterion is not met | **P.** The claim is removed and D-VS7 records a partial supersession. S2/S3 and the bucket model are Lab-only by default. They are not deleted, because the maintainer's self-model decision (see "1. The question" in this note) requires clearly labelled estimates when sensors are absent. |
| s16 | PHY honesty and BW500 vs Revisit-5 | **F.** Principle 8, "NO VECTOR DATA" at SF8/SF9, D-VS3a with an FCC note. **R:** an SF8/BW500 image rung, because 18 B fits below the envelope (computed). |
| s17 | W4-11 blind to tractor control loss | **F.** Tractor-side inter-arrival criterion; P3 only in granted slots. |
| s18 | Rename misses files; alias name; staging; parity | **F.** Full list in §7.1; map.js treats VECTOR as bad; edit common, then re-stage; new parity tests. |
| s19 | W4-11 breaks HIL completeness tests | **F.** Required changes listed in §8.6. |
| s20 | Ladder vs R-8 and V6; tile infeasibility | **F.** D-VS6 supersedes; tile rungs stated infeasible at E0–E2 and config-disabled; arithmetic ceil(150/(F−4)). |
| s21 | "8 s" figure | **F.** Same as b8. |
| s22 | v0 ranges | **F.** Same as b11. |
| s23 | Raw mode shows the model; old clients not fail-closed | **F.** Model hidden and masks hatched in raw mode; schema-2 handshake plus cache-busting. |
| s24 | Citations, mask_anom forcing, S3 flow sharing | **F.** 1 Hz at `:1418` and the doc rate fixed in Phase 0; mask_anom forces STATUS; S3 limits stated. |
| s25 | Tractor PYTHONPATH | **F.** B10 and §7.1. |
| s26 | Sky down-weighting starves overhead lines; cap only in VECTOR | **F.** Overhead L3 class ×3, sky penalty lifted in the swept volume; cap triggered by refresh age and fps on every rung. |
