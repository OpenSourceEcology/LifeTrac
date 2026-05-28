# Image-Over-LoRa Pipeline — Roadmap to Production (v25.0.1 → v25.1.0)

Date: 2026-05-27
Author: Copilot (Claude Opus 4.7)
Scope: Concrete, ordered plan to evolve the just-demonstrated **Image-Over-LoRa Pipeline v25.0.1** (see [2026-05-27_Image_Over_LoRa_Pipeline_v25.0.1_First_Working_Demo.md](2026-05-27_Image_Over_LoRa_Pipeline_v25.0.1_First_Working_Demo.md)) into the first **field-legal production release v25.1.0**. Each milestone is a separately tagable git release with an entry/exit checklist so we always know what "done" looks like.

This roadmap intentionally avoids parallel sprawl: each milestone closes one risk class before the next opens. Bench → bench-hardened → field-trial → production.

---

## 0. Definition of "production" for v25.1.0

A production image-over-LoRa pipeline must satisfy **all** of:

1. **Field-legal.** FCC Part 15 §15.247 FHSS profile active (`FCC_50CH_FHSS`, not `BENCH_ONLY_FIXED_915`). Power, dwell, and channel set audited against [2026-05-19_FCC_Part15_902-928_Compliance_Notes_Copilot_v1_0.md](2026-05-19_FCC_Part15_902-928_Compliance_Notes_Copilot_v1_0.md).
2. **Recoverable.** Tractor + base survive a power cycle, a USB camera unplug/replug, a LoRa link outage of ≥ 60 s, and a single-container OOM, all without operator intervention.
3. **Observable.** Operator UI shows live B/s, fragments/s, fragment-loss %, current encode mode, link RSSI/SNR, and a 30-frame rolling preview thumbnail.
4. **Adaptive.** Encoder ladder is driven by measured airtime headroom, not a static env var. At least three modes (`full` / `motion_only` / `mono_g4`) demonstrably swap in under load.
5. **Reproducible.** Tagged git commit + release manifest dir + signed firmware/container digests as governed by [2026-05-26_v25.0.1_Release_Packaging_Plan_Copilot_v1_0.md](2026-05-26_v25.0.1_Release_Packaging_Plan_Copilot_v1_0.md).
6. **Documented.** Operator runbook + dev runbook + regulatory compliance statement, all in `LifeTrac-v25/RELEASE/v25.1.0/`.
7. **Tested.** Automated smoke (bench), 1-hour soak (bench), 30-minute field trial at ≥ 100 m line-of-sight, and a manual destructive-recovery checklist all green on the tagged commit.

Anything short of all seven ships as a v25.0.x point release, not v25.1.0.

---

## 1. Milestone ladder

```
v25.0.1  ─── first working demo (DONE, 2026-05-27)
   │
   ├──► v25.0.2  Persist + Pin
   ├──► v25.0.3  Observability
   ├──► v25.0.4  Adaptive Encoder Ladder
   ├──► v25.0.5  Resilience & Recovery
   ├──► v25.0.6  Bandwidth & Quality Sweep
   ├──► v25.0.7  Regulatory Profile Gate
   ├──► v25.0.8  Field Trial (controlled)
   │
   └──► v25.1.0  Production Release
```

Each `v25.0.x` is a bench/lab milestone. `v25.1.0` is the first field-legal, operator-facing release.

---

## 2. v25.0.2 — Persist & Pin

**Goal:** Eliminate "works only because containers are hot-patched" risk. Everything currently live on the two X8s must survive a `reboot`.

Entry: v25.0.1 demo green.

Work:

- Commit the demo config to git:
  - [camera_service.py](../DESIGN-CONTROLLER/firmware/tractor_x8/camera_service.py) (V4l2FfmpegCamera path).
  - [docker-compose.video-test.yml](../DESIGN-CONTROLLER/firmware/tractor_x8/docker-compose.video-test.yml) with the v4l2/MJPEG/640×480@5 env block.
  - [fade_shader.js](../base_station/web/img/fade_shader.js) grid-aware fix.
  - `web_ui.py`, `canvas.py`, `frame_format.py`, `codec_decode.py` workspace edits.
- Bake the static arm64 ffmpeg into the camera container image rather than bind-mounting from `/opt/lifetrac/bin/ffmpeg`. Pin version 7.0.2 + sha256.
- Pin every container image by digest (not `:latest`) in both compose files.
- Add a `make verify` target that hashes the running container images vs the pinned digests on each X8.
- Remove orphan `video-test_default` docker network.
- Add `systemd` unit verification: `systemctl is-enabled` for both stacks logged at boot.

Exit:

- Cold-boot both X8s; web UI shows live tiles within 60 s without manual `docker network connect` or `docker cp`.
- `git diff HEAD` clean on both hosts after boot.
- Tag `v25.0.2-bench`.

---

## 3. v25.0.3 — Observability

**Goal:** No more diagnosing through `mosquitto_sub` shells. The operator UI and a `/api/stats` JSON endpoint expose everything needed to spot regressions.

Entry: v25.0.2 tagged.

Work:

- Tractor: publish per-second link telemetry on `lifetrac/image/stats` — bytes sent, fragments sent, encode mode, queue depth, last keyframe age, camera fps, encode latency p50/p95.
- Base: subscribe and surface in the footer of the web UI (replace static "WS control 20 Hz · USB Gamepad: ..." with a live strip).
- Base: expose `/api/stats` returning the last 60 s of samples as JSON for scripting.
- Add a rolling 30-frame thumbnail strip below the main canvas (debug, toggleable via the existing ⚙ settings panel).
- Add link RSSI/SNR from the SX1276 status reads into the same telemetry channel.
- Add a `make smoke` target that exits non-zero if any stats sample shows fragment-loss % > 5 over a 60 s window.

Exit:

- UI footer shows live B/s, fragments/s, loss %, encode mode, RSSI/SNR.
- `/api/stats` returns valid JSON with at least 60 samples after 90 s uptime.
- `make smoke` green for 5 consecutive runs.
- Tag `v25.0.3-bench`.

---

## 4. v25.0.4 — Adaptive Encoder Ladder

**Goal:** Honor the back-channel. The code paths exist in `camera_service.py` / `web_ui.py` but are bypassed under `USE_LORA_BRIDGE=1`. Wire them.

Entry: v25.0.3 tagged.

Work:

- Surface `CMD_ENCODE_MODE` (0x63), `CMD_LINK_PROFILE`, `CMD_ROI_HINT` (0x61) across the LoRa bridge in both directions.
- Implement an airtime-pressure controller on the tractor: if measured TX duty cycle in a 10 s window exceeds 60 %, step down `full → motion_only → mono_g4`; if < 20 %, step back up. Hysteresis ≥ 5 s per step.
- Manual override: `POST /api/encode_mode/cycle` (cookie session) must actually change the radio-side encoder mode end-to-end, with `enc:` header in the UI reflecting reality within 2 s.
- Validate all three modes on air, including a transition under simulated load (`tc qdisc` on the bridge or a synthetic motion injector).
- Document the mode ladder + thresholds in the release manifest dir.

Exit:

- Manual mode cycle changes the on-air bytes within 2 s and the UI header within 4 s.
- Auto controller demonstrably steps down under a 60 s synthetic motion burst and recovers within 30 s after burst ends.
- No mode flap > 1 transition per 5 s in any 5-minute soak.
- Tag `v25.0.4-bench`.

---

## 5. v25.0.5 — Resilience & Recovery

**Goal:** Survive the four canonical fault modes without operator intervention.

Entry: v25.0.4 tagged.

Work:

- **Camera unplug.** `V4l2FfmpegCamera` detects EOF/ENODEV, restarts ffmpeg with capped exponential backoff (1 s → 16 s), emits a `camera_down` telemetry flag. UI shows a banner.
- **Link outage.** Tractor keeps encoding but drops oldest fragments when the LoRa TX queue exceeds N pending. Base UI shows "link stalled" after 5 s of no fragments and "link lost" after 30 s. Auto-recover on next fragment.
- **Container OOM.** Add `restart: unless-stopped` + memory limits to both compose stacks. Add a watchdog that pings `mosquitto` every 10 s and restarts the camera container if `mosquitto` DNS fails (the network-disconnect class of bug we already hit).
- **Power cycle.** Confirmed at v25.0.2 exit, re-asserted here with a 10-cycle soak.
- Add a destructive-recovery checklist to `LifeTrac-v25/RELEASE/v25.0.5/` with explicit operator and developer steps for each fault.

Exit:

- 10 random unplug/replug cycles of the USB camera, each recovered ≤ 30 s without manual action.
- 60 s LoRa link interruption (cable pull on the bridge UART or RF cage) recovered ≤ 15 s after restoration.
- 10 power cycles, each green at v25.0.2 exit criteria.
- Tag `v25.0.5-bench-hardened`.

---

## 6. v25.0.6 — Bandwidth & Quality Sweep

**Goal:** Find the operating point that maximises subjective utility per byte. This is data-collection, not new code.

Entry: v25.0.5 tagged.

Work:

- Parametric sweep with the smoke harness:
  - `WEBP_QUALITY` ∈ {15, 20, 25, 30, 35}.
  - `KEYFRAME_PERIOD_S` ∈ {30, 60, 120}.
  - Tile grid ∈ {6×4×32, 8×6×24, 12×8×16}.
  - Encode modes ∈ {full, motion_only, mono_g4}.
- For each cell record: idle B/s, motion B/s (synthetic injector), keyframe size, p95 frame age on base, subjective sharpness from a 5-frame still strip.
- Recolouriser shader pass on the base UI for low-Q WebP banding (was deferred in v25.0.1).
- Publish results as `LifeTrac-v25/AI NOTES/2026-XX-XX_Image_Pipeline_Operating_Point_Sweep.md`.

Exit:

- Sweep matrix complete, at least one operating point identified per use case (idle monitoring vs active driving vs degraded link).
- Recolouriser shader merged behind a UI toggle, default off.
- Tag `v25.0.6-bench-hardened`.

---

## 6.5. v25.0.6.5 — FHSS RX-Scan Closure (NEW, inserted 2026-05-27)

**Goal:** Close the 2026-05-25 "two peers pick different starting channels under wide mask" bug so that `REG_PROFILE = FCC_15_247_FHSS_50CH_BW250` actually carries traffic between two peers without a single-channel rendezvous hack. This is now the **critical-path prerequisite** for v25.0.7, promoted ahead per user direction ("prioritize the FHSS … default bench transmission style", 2026-05-27).

**Why this is a separate milestone (not folded into v25.0.7):** Track A firmware modules (`sx1276_rx_scan_*.c`, `sx1276_fhss.c`, `host_cfg_profile.c`) are in the tree but have **never been demonstrated to close the loop end-to-end with two peers under a wide mask**. Until they do, "regulatory profile gate" can only be passed in single-channel mode, which defeats the FHSS purpose. See [2026-05-27_FHSS_Bench_Default_Activation_Plan_v1_0.md](2026-05-27_FHSS_Bench_Default_Activation_Plan_v1_0.md) §3.

**Status preconditions already met (2026-05-27):**
- Host-side activator landed: [LifeTrac-v25/tools/bench_activate_fhss.py](../tools/bench_activate_fhss.py) (dry-run by default).
- `REG_PROFILE=1` can now be staged from the host at any time; firmware ACKs OK on both X8s' L072s (per `host_cfg_profile.c` validator audit).

Entry: v25.0.6 tagged AND `bench_activate_fhss.py --apply --mask single --ch 0` succeeds end-to-end on both X8s with `RUNTIME_PROFILE_ENUM=1` readback.

Work:
1. **Reproduce the 2026-05-25 bug** with the new activator's `--mask wide` mode: TX_DONE=OK on tractor, `rx_frames=0` on base, confirmed by `radio_state_dump.py` showing different `freq_mhz` per peer.
2. **Diagnose the per-board channel pick**: trace `sx1276_fhss_next_channel()` and `host_cfg_profile_activate()` to find where the starting channel is selected. Confirm whether the RX scan policy (`sx1276_rx_scan_policy.c`) is even invoked under wide-mask + RegHopPeriod=0.
3. **Pick a fix path** (orthogonal to firmware-default-on decision):
   - **a. Deterministic same starting channel** — make activation always pick `mask.lowest_set_bit()` as starting channel.
   - **b. RX scan-and-lock** — make RX iterate through the mask until it sees a valid preamble, then dwell.
   - **c. Per-packet hop** — set RegHopPeriod > 0 so both peers actually hop in sync (requires authenticated `(profile_id, epoch, hop_idx)` header per plan §A6).
4. **Acceptance criterion**: 60 s air test with `bench_activate_fhss.py --apply --mask wide` on both peers reports `rx_frames > 0` on base AND the RFCO instrument shows `hop_idx` varying (path c) OR a stable channel-of-rendezvous (paths a/b).

Exit:
- 60 s air test green under wide mask.
- Documented decision (a/b/c) recorded in `LifeTrac-v25/AI NOTES/` and `/memories/repo/`.
- Tag `v25.0.6.5-fhss-airlink`.

---

## 7. v25.0.7 — Regulatory Profile Gate

**Goal:** Promote from `BENCH_ONLY_FIXED_915` (profile 0) to `FCC_50CH_FHSS`. This is the gate that separates bench from field.

Entry: v25.0.6.5 tagged (was: v25.0.6). The activator script is wired into the boot orchestrator on both X8s (systemd one-shot or daemon `_open_link()` precondition — TBD per §6 question 1 of the bench-default plan doc).

Work:

- Activate the 50-channel FHSS plan from [2026-05-19_LoRa_FCC_50CH_FHSS_Implementation_Plan_Copilot_v1_0.md](2026-05-19_LoRa_FCC_50CH_FHSS_Implementation_Plan_Copilot_v1_0.md).
- Re-run the bench smoke from v25.0.3 on the new profile; image pipeline must still deliver tiles, with hop-related latency characterised.
- Cross-check power, dwell, and channel set against [2026-05-19_FCC_Part15_902-928_Compliance_Notes_Copilot_v1_0.md](2026-05-19_FCC_Part15_902-928_Compliance_Notes_Copilot_v1_0.md). Record measurements.
- Resolve any open items in [2026-05-22_W1-11_RF_Blocked_FHSS_Gate_v1_0.md](2026-05-22_W1-11_RF_Blocked_FHSS_Gate_v1_0.md).
- Add a startup gate on the tractor: refuse to TX unless the active profile is FHSS or the explicit bench override env var is set.

Exit:

- 1-hour bench soak on FHSS profile with image pipeline green.
- Compliance worksheet signed off.
- Bench override env var documented and audit-logged on use.
- Tag `v25.0.7-fcc-ready`.

---

## 8. v25.0.8 — Field Trial (controlled)

**Goal:** Validate everything outdoors, at distance, with the actual operator workflow.

Entry: v25.0.7 tagged.

Work:

- 30-minute outdoor trial at ≥ 100 m line-of-sight, tractor stationary, operator at base.
- 30-minute outdoor trial with tractor under manual drive (ground crew, no hydraulic load).
- Capture full telemetry from `/api/stats` to disk for both runs.
- Operator subjective form: was the image useful for situational awareness? Latency tolerable? Mode transitions noticeable/distracting?
- File any new bugs as `v25.0.9` candidates; field-trial failures **do not** auto-block v25.1.0 unless they hit a Definition-of-Production item from §0.

Exit:

- Both trials complete with no unrecovered faults.
- Telemetry archived to `LifeTrac-v25/RELEASE/v25.0.8/trials/`.
- Operator sign-off recorded.
- Tag `v25.0.8-field-trial`.

---

## 9. v25.1.0 — Production Release

**Goal:** Cut the first production release per the packaging plan.

Entry: v25.0.8 tagged with operator sign-off.

Work:

- Execute the release procedure from [2026-05-26_v25.0.1_Release_Packaging_Plan_Copilot_v1_0.md](2026-05-26_v25.0.1_Release_Packaging_Plan_Copilot_v1_0.md):
  - Annotated git tag `v25.1.0` on the green commit.
  - Populate `LifeTrac-v25/RELEASE/v25.1.0/` with Dockerfiles, compose, firmware binaries + sha256, install script, version pins, release notes.
  - Move clearly-orphaned code to `LifeTrac-v25/DESIGN-CONTROLLER/_retired/` with `git mv`.
- Author the three required documents into the release manifest dir:
  - Operator runbook (login, mode cycle, E-stop, recovery from each fault class).
  - Developer runbook (build, deploy, debug, telemetry inspection).
  - Regulatory compliance statement (profile, measurements, worksheet reference).
- Re-run `make smoke` and the v25.0.5 destructive-recovery checklist on the tagged commit, archive outputs.
- Announce in `LifeTrac-v25/README.md` and `LifeTrac-v25/TODO.md`.

Exit (= Definition of Production, §0):

1. ✅ Field-legal FHSS profile active.
2. ✅ All four recovery scenarios green.
3. ✅ UI shows B/s, fragments/s, loss %, encode mode, RSSI/SNR, thumbnail strip.
4. ✅ Adaptive ladder swaps three modes under load.
5. ✅ Tagged commit + release manifest dir + signed digests.
6. ✅ Operator + developer + compliance docs in release dir.
7. ✅ Bench smoke + 1-hour soak + 30-min field trials + destructive-recovery checklist all green.

Tag: `v25.1.0`.

---

## 10. Non-goals (explicitly deferred past v25.1.0)

These are good ideas that **do not** block production. Listed here to keep scope honest:

- Audio over LoRa.
- Multi-camera (front + rear) tile mux.
- Recolouriser ML upscaler (beyond the simple banding-reduction shader in v25.0.6).
- Operator gamepad haptics tied to encode-mode steps.
- Cloud relay / multi-base-station handoff.
- Sub-second p95 latency. v25.1.0 ships with the ~1 fps tile cadence demonstrated at v25.0.1; lower latency is a v25.2.x topic.
- Replacement of the SX1276 with a higher-bandwidth radio.

---

## 11. Risk register

| Risk | Likelihood | Impact | Mitigation milestone |
|---|---|---|---|
| FHSS profile breaks image pipeline timing (hop > fragment) | Medium | High — blocks v25.0.7 | Characterise in v25.0.4 controller; budget hop time in `FRAGMENT_BUDGET`. |
| USB camera firmware quirk reappears after reboot | Medium | Medium | v25.0.5 unplug/replug soak; capped backoff. |
| Adaptive ladder oscillates under marginal link | Medium | Medium | Hysteresis ≥ 5 s; flap-rate alert in `/api/stats`. |
| Bench override env var leaks into field deployment | Low | High — regulatory | v25.0.7 audit-log on use; release script refuses to package with override set. |
| Container image drift between tractor and base | Low | Medium | v25.0.2 digest pinning + `make verify`. |
| Field trial reveals unrecovered fault class | Medium | Medium | v25.0.8 surfaces it as v25.0.9 candidate; loop back only if it hits §0. |

---

## 12. Cross-references

- v25.0.1 demo: [2026-05-27_Image_Over_LoRa_Pipeline_v25.0.1_First_Working_Demo.md](2026-05-27_Image_Over_LoRa_Pipeline_v25.0.1_First_Working_Demo.md)
- Release packaging governance: [2026-05-26_v25.0.1_Release_Packaging_Plan_Copilot_v1_0.md](2026-05-26_v25.0.1_Release_Packaging_Plan_Copilot_v1_0.md)
- Bench plan that got us here: [2026-05-18_W2-02_Image_Over_LoRa_Bench_Plan_Copilot_v1_0.md](2026-05-18_W2-02_Image_Over_LoRa_Bench_Plan_Copilot_v1_0.md)
- Prior air-link proof: [2026-05-26_Image_Over_LoRa_AirLink_Proven_Status_Copilot_v1_0.md](2026-05-26_Image_Over_LoRa_AirLink_Proven_Status_Copilot_v1_0.md)
- FHSS implementation plan: [2026-05-19_LoRa_FCC_50CH_FHSS_Implementation_Plan_Copilot_v1_0.md](2026-05-19_LoRa_FCC_50CH_FHSS_Implementation_Plan_Copilot_v1_0.md)
- Regulatory notes: [2026-05-19_FCC_Part15_902-928_Compliance_Notes_Copilot_v1_0.md](2026-05-19_FCC_Part15_902-928_Compliance_Notes_Copilot_v1_0.md)
- FHSS gate blocker: [2026-05-22_W1-11_RF_Blocked_FHSS_Gate_v1_0.md](2026-05-22_W1-11_RF_Blocked_FHSS_Gate_v1_0.md)
- LoRa QoS sizing: [2026-04-26_LoRa_QoS_Bandwidth_Management.md](2026-04-26_LoRa_QoS_Bandwidth_Management.md)
- TX power adaptation & safety burst: [2026-05-18_TX_Power_Adaptation_And_Safety_Burst_Design_Copilot_v1_0.md](2026-05-18_TX_Power_Adaptation_And_Safety_Burst_Design_Copilot_v1_0.md)

---

## 13. Trajectory Review and Next-Step Recommendation (GitHub Copilot v1.0)

### Current status after reviewing the plan stack

The current trajectory is no longer "can we get a picture over LoRa?" That question has been answered by the v25.0.1 demo: real UVC camera frames are captured on the tractor X8, encoded as tile WebP, fragmented through the Murata L072/SX1276 path, reassembled on the base X8, and rendered in the browser canvas. The active question is now release engineering: how to turn a hot-patched, bench-only demo into a reproducible, observable, recoverable, field-legal system.

The roadmap above is therefore directionally correct: v25.0.2 should freeze and persist the working bench state before we chase more bandwidth, v25.0.3 should make the link observable before we tune it, v25.0.4 should wire adaptive control, and v25.0.7 must be the hard boundary between bench fixed-frequency operation and field-legal FHSS.

### What still matches the older image-processing plans

The older image pipeline documents still got the strategic shape right: tile deltas, persistent canvas, freshness over completeness, ROI/fairness scheduling, browser rendering from canonical `/ws/state`, and an encoder ladder are still the right architecture. The 2026-05-25 method tracker also remains relevant: Method A's row-major/noise-driven behavior explains the old top-strip starvation, while Method C's magnitude plus sweep fairness is still the best near-term tile-selection upgrade once the demo is pinned.

The difference is sequencing. The older plans spend a lot of attention on NanoDet, base-side super-resolution, background cache, optical flow, wireframe mode, badges, and Coral. Those are good v25.x/v26 ideas, but they should not interrupt the production ladder above. For v25.1.0, the useful subset is smaller: full WebP, one low-bandwidth fallback (`mono_g4` or equivalent), live telemetry, recovery behavior, and a measured operating-point sweep. The rest should stay documented but explicitly non-blocking unless it supports one of the Definition-of-Production items in section 0.

### What still matches the older LoRa transmission plans

The LoRa plans still match at the priority and safety level: P0/P1/P2/P3 naming, strict preemption, no full MQTT over the air, local MQTT only as X8 IPC, and the need to keep image traffic from starving control are all still correct.

The older radio compliance details do not fully match the current trajectory. [LORA_IMPLEMENTATION.md](../DESIGN-CONTROLLER/LORA_IMPLEMENTATION.md) and [MASTER_PLAN.md](../DESIGN-CONTROLLER/MASTER_PLAN.md) still contain the older 8-channel FHSS story, while the current legal path is the later `FCC_15_247_FHSS_50CH_BW250` plan from the FCC notes and 50-channel implementation plan. Treat the 8-channel text as superseded. The roadmap's v25.0.7 gate is the right place to reconcile this by activating the 50-channel profile, proving image delivery under hopping, and archiving the required power/dwell/channel evidence.

One more transport detail should be made explicit before production: v25.0.1 deliberately uses the image-over-LoRa bridge path, not the full production LoRa-frame/AES/replay envelope described by older architecture docs. That is fine for the bench release if it is labeled, but before v25.1.0 we need a written decision: either route image fragments through the canonical bridge/envelope path, or define and audit the raw HostLink image path as an accepted production transport with its own integrity, observability, and regulatory evidence story. I would not let this remain implicit past v25.0.4.

### TODO and master-plan alignment

[../TODO.md](../TODO.md) mostly matches the current direction at the top level: it records the image-over-LoRa milestone, the v25.0.1-bench packaging approach, and Stage S1.5 as the 50-channel FHSS pre-launch blocker. It is close enough to use as the program checklist, but it needs a refresh now that v25.0.1 has progressed from "visual-pixel confirmation pending" to the 2026-05-27 first working demo. It should also demote older "all RF testing blocked" language where later evidence has superseded it.

[../DESIGN-CONTROLLER/TODO.md](../DESIGN-CONTROLLER/TODO.md) is more stale. Its current gate summary is still anchored around 2026-05-12 W1-10/W1-9 bring-up status and does not reflect the later air-link proof or v25.0.1 demo. Its LoRa validation section also still references 8-channel FHSS in places. It should receive a short supersession banner pointing to this roadmap, the v25.0.1 demo note, and the 50-channel FCC plan rather than being hand-edited piecemeal.

[../DESIGN-CONTROLLER/MASTER_PLAN.md](../DESIGN-CONTROLLER/MASTER_PLAN.md) is mixed. Section 8.19/8.20 is still broadly aligned because it says X8 CPU first, LoRa tile-delta path, base reconstruction, `/ws/state`, and `CMD_ENCODE_MODE`. Section 8.17 is stale because it still preserves the old SF7/BW125 control mismatch and the 8-channel FHSS rationale. My recommendation is not to rewrite the master plan immediately; add a dated supersession note near section 8.17 saying FCC/legal field work is now governed by the 50-channel FHSS documents and this v25.0.1-to-v25.1.0 roadmap.

### Recommended next steps

1. Finish v25.0.2 exactly as written, with one addition: capture the live two-X8 delta into git before any tuning. The first production risk is not bandwidth, it is losing the hot-patched working state on reboot.
2. In the same v25.0.2 pass, update only the document headers/banners that are now misleading: top-level TODO status, DESIGN-CONTROLLER TODO current gate state, MASTER_PLAN section 8.17 supersession note, and LORA_IMPLEMENTATION's 8-channel FHSS headline.
3. Do v25.0.3 observability before codec work. The UI and `/api/stats` should expose bytes/s, fragments/s, loss, queue depth, encode mode, RSSI/SNR, and decoded RFCO/ERR_PROTO reason counts. Raw `1001080000`-style failures should not survive into the next milestone.
4. Do v25.0.4 as a narrow adaptive-ladder milestone: prove `POST /api/encode_mode/cycle` changes on-air bytes under the LoRa bridge, and validate only the modes that already have strong vertical slices (`full`, `y_only`, `mono_g4` or the chosen low-bandwidth fallback). Keep BTC4/adaptive experiments behind the later sweep.
5. Keep v25.0.5 recovery work before v25.0.6 quality work. Camera unplug, link outage, container OOM, and power-cycle recovery are more production-defining than prettier pixels.
6. Use v25.0.6 to choose the operating point from data: WebP quality, keyframe period, grid size, Method C sweep fairness, and low-bandwidth mode. Do not promote subjective codec choices without the smoke harness and archived samples.
7. Treat v25.0.7 as a real legal/product gate, not a configuration flip. The image path must be green on `FCC_50CH_FHSS`, compliance evidence must be archived, bench override use must be audit-logged, and the raw image-bridge transport decision must be resolved before any outdoor demo.

Bottom line: the roadmap is the right spine. The TODO files and master plan should be refreshed to point at it, not used to pull the project backward into stale 8-channel or pre-demo assumptions. The next move is not more invention; it is pin, observe, recover, then legalize.

*Signed:* GitHub Copilot, Trajectory Review v1.0 (2026-05-27)

---

## 14. FHSS-First Bench Protocol Adjustment (GitHub Copilot v1.1)

After the user review on 2026-05-27, I would pull the 50-channel FHSS work forward. The right next step is still to preserve the v25.0.1 working demo in git so the hot-patched state is not lost, but immediately after that the next RF milestone should become:

> Make `FCC_15_247_FHSS_50CH_BW250` the normal two-peer communication profile, prove image traffic over it, and keep it as the default bench protocol for future LoRa tests.

This is a better trajectory than spending several more cycles improving the fixed-915 demo. Fixed 915 MHz has done its job: it proved the camera-to-browser and L072 air-link plumbing. Continuing to tune against fixed 915 risks optimizing the wrong system. The production system must live under hopping, so the bench system should live under hopping too once the profile is actually green.

### Legal/compliance nuance

The 50-channel FHSS path is the safest regulatory direction we have identified for US 902-928 MHz operation under FCC Part 15.247. It uses a real hopping profile, at least 50 channels, BW250, dwell accounting, channel-set evidence, and the existing +17 dBm hardware clamp. That is the correct compliance architecture.

However, it should not be described as already legally complete just because the plan is sound. The legal option becomes defensible when the firmware is actually running the 50-channel profile and the evidence gates are archived: channel count, hopping distribution, dwell, occupied bandwidth, out-of-band emissions, power clamp, and profile lock. So the wording I would use is: **50-channel FHSS is our intended compliant profile and is safe to make the bench default for engineering, but field/legal release still depends on the evidence gate.**

### Why this can be the next milestone

The firmware is no longer starting from zero. The routed build already contains the profile enum, 50-channel table, hop scheduler, TX retune path, hop-sync header, RX scan/snap logic, legal dwell plumbing, and RFCO per-TX telemetry. The X8 helper scripts also already understand `LIFETRAC_REG_PROFILE=1`.

The remaining problem is not "invent FHSS." It is "make profile 1 actually work as the two-peer default." The current demo compose and concurrent smoke scripts still force `LIFETRAC_REG_PROFILE=0`, and the helper contains a known bench workaround that force-pins `RegFrf` to 915 MHz only for profile 0. The known blocker is the 2026-05-25 profile-1 RX/FRF trap: profile activation can ACK OK without re-applying the radio frequency/profile state in a way that makes both peers decode each other.

### New recommended sequence

1. Finish a minimal v25.0.2 persistence pin: commit the exact working v25.0.1 image-over-LoRa demo state and deployment deltas.
2. Insert a new FHSS-first milestone before codec/adaptive-quality work. Call it `v25.0.2a` or promote it to `v25.0.3`; the name matters less than the gate.
3. Fix profile-1 activation at the firmware/root-cause layer, not by carrying the Python fixed-FRF workaround forward. Activating `FCC_15_247_FHSS_50CH_BW250` should initialize the 50-channel scheduler, program BW250, program/retune the SX1276 to the correct hop channel, and make RX scanning lock to the hop-sync header without fixed 915 MHz writes.
4. Change smoke/orchestrator defaults only after the above is proven: `run_concurrent_smoke.ps1`, tractor/base compose env, image TX/RX daemons, and packaging docs should default to `LIFETRAC_REG_PROFILE=1` with the wide 50-channel mask. Keep `BENCH_ONLY_FIXED_915` as an explicit `--bench-fixed-915` or environment escape hatch for diagnosis only.
5. Require each bench artifact to stamp active profile, channel mask, BW, selected frequency or hop histogram, RFCO per-TX status, dwell usage, and runtime profile readback. A bench run without profile evidence should not count as RF evidence.
6. Re-run the image pipeline smoke under hopping: synthetic first, then real `/dev/video1`, then the longer soak. Only then resume image-quality improvements such as Method C, adaptive mode cycling, and low-bandwidth codec comparison.

### Acceptance criteria before making it the default bench protocol

- Both boards report `RUNTIME_PROFILE_ENUM=1` and reject masks with fewer than 50 channels.
- TX RFCO shows nonzero hop/channel/frequency fields rotating across the 50-channel table.
- RX receives real frames under profile 1 without `LIFETRAC_FORCE_FRF_HZ` and without profile 0.
- The browser image canvas updates through the same `/ws/state` path under profile 1.
- Dwell and packet-airtime counters stay below the configured caps during a representative image run.
- The old fixed-915 path is still available, but every use is visually/logically marked bench-diagnostic and excluded from field-readiness evidence.

Bottom line: yes, make 50-channel FHSS the next RF implementation target and the future bench default. Just do it as a gated migration, not as a blind config flip. Once hopping is green, all meaningful image-over-LoRa work should happen on the hopping profile.

*Signed:* GitHub Copilot, FHSS-First Bench Protocol Review v1.1 (2026-05-27)

---

## 14. Roadmap Alignment & Next Steps

### Trajectory Alignment Analysis
- **TODO.md:** Fully aligned with current physical-layer realities. The recent P8 unblockers (FHSS_TX_ROUTED) and physical-layer gates (SYNC_OK=0) dictate our critical path. It accurately reflects our pivot from pure "Image over LoRa" towards "FHSS survival first". 
- **MASTER_PLAN.md:** Misaligned. It lists high-level hardware (Kurokesu UVC cameras, 915 MHz SX1276) but glosses over the extreme bandwidth/safety constraints we've discovered. It likely assumed naive video streaming was viable without addressing the 50ms FCC FHSS hop penalty or dual-core M4 safety limits on the X8.

### Tactical Next Steps
We must subordinate image-quality goals until the transport layer is fundamentally stable under FHSS. 

1. **Unblock SWD (P1 Regression fix):** 
   Update our OpenOCD scripts or flash utilities to invoke `fuser -k /dev/ttymxc3` before trying to flash the Murata module. Python daemons maintaining zombie locks are currently masquerading as `DPIDR 0xdeadbeef` hardware failures.
2. **Clear the Base UI Router (`0.0.0.0` Fix):**
   Update the FastAPI UI bind address to `0.0.0.0` off localhost so external diagnostic browsers on the LAN can reach `/ws/state`. 
3. **Resolve the P8 FHSS TX Reject (`ERR_PROTO`):**
   The Murata module is throwing exceptions on TX because no active FHSS profile is armed by the Host. We must implement `cfg_set(CFG_KEY_REG_PROFILE=0x14)` in the python test harness immediately so our test runs (W1-11, W2-02) can proceed.
4. **Implement Async CPU yielding:**
   Remove synchronous `time.sleep()` blocks in the Python TX daemons. Use `asyncio` streams so the base station UI thread isn't starving during image encoding blocks.

*Signed:* GitHub Copilot, Protocol Alignment & Trajectory Review v2.0 (2026-05-27)
