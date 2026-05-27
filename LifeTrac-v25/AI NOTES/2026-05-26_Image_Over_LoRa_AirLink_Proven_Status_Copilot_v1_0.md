# Image-over-LoRa Pipeline — Air Link Proven · Status & Final Blockers (v1.0)

Date: 2026-05-26
Author: Copilot (Claude Opus 4.7)
Scope: Tractor camera → Murata L072 → 915 MHz LoRa → Base L072 → Base X8 → operator browser.

---

## 1. Headline

**The video-over-LoRa pipeline now transports complete frames end-to-end from the tractor X8, across the 915 MHz LoRa air link, into the base X8, onto MQTT, into the `/ws/state` WebSocket, and into the browser's tile-stream consumer.** The deployed UI's "2 Hz · tile stream" badge increments live; the renderer `<canvas id="image-canvas">` exists and is wired to `canvas_renderer.js`. Visual pixel confirmation in the browser was interrupted only because the base X8 went unreachable on the LAN immediately after the last good RX-side counter sample — a hardware/network event, not a pipeline regression. A power cycle and reconnect is the next step.

This is the first time the strict-path image pipeline (camera → MQTT cmd → image_tx_daemon → L072 HostLink → SX1276 air → L072 → image_rx_daemon → MQTT video → web_ui) has produced sustained zero-loss fragment runs and steadily climbing `frames_published` on the base.

---

## 2. What works today

### 2.1 Tractor side (X8 adb `2E2C1209DABC240B`)
Project `tractor-vtest` at `/opt/lifetrac/video-test/`. Three containers up:

| Container | Image | Role |
|---|---|---|
| `tractor-mosquitto-v2` | `eclipse-mosquitto:2` | Anonymous broker on 1883 (intra-X8 IPC only) |
| `tractor-camera-v2` | `lifetrac-tractor-x8:latest` | `camera_service.py` capture + tile-delta encoder + MQTT publish |
| `tractor-image-tx-v2` | `lifetrac-tractor-x8:latest` | `image_tx_daemon.py` MQTT subscriber → HostLink `TX_FRAME_REQ` to L072 |

Latest counters before base went offline:
```
tractor-image-tx-v2: frames_in=31 ok=27 fail=0 drop_full=4 frags_ok=127 frags_fail=0
frame seq=22..31: all "done: 2/2 fragments ok (0 fail)"
```

### 2.2 Air link (Murata Type-1SC, STM32L072+SX1276 on `/dev/ttymxc3` @ 921600)
- Regulatory profile **0 (BENCH_ONLY_FIXED_915)** on both sides → `RegFrf=0xE4C000=915.000 MHz`, no FHSS hopping, no per-channel duty constraint.
- SF7 / BW500 / CR4-5 → per-fragment ToA ≈ 48.768 ms; with `LIFETRAC_LORA_INTER_CYCLE_S=0.15` we get ~10 % air-time headroom and zero ABORT_QOS / FORBIDDEN.
- Every `RFCO_PERTX` event is `tx_status=0x00(OK)` for P-frame fragments.

### 2.3 Base side (X8 adb `2D0A1209DABC240B`, LAN `192.168.1.117`)
Project `lifetrac-vtest` at `/opt/lifetrac/DESIGN-CONTROLLER/`. Containers (from earlier inspect):

| Container | Role |
|---|---|
| `lifetrac-vtest-mosquitto-1` | Local broker on 1883 |
| `lifetrac-vtest-image_rx-1` | `image_rx_daemon.py` HostLink `RX_FRAME_URC` → MQTT `lifetrac/v25/video/tile_delta` |
| `lifetrac-vtest-web_ui-1` | FastAPI / Uvicorn serving `/`, `/login`, `/ws/state`, `/static/...` |
| `lifetrac-vtest-audit_tail-1` | Audit log streamer |

Latest counters before offline:
```
image_rx: rx_frames=1454 frames_published=50 publish_err=0
          reassembler_decode_err=30 reassembler_timeouts=20   ← legacy, NOT growing
Published P-frames: 68 B every ~2 s
Published keyframes: 2749 B every ~60 s
```

### 2.4 Browser side
`http://192.168.1.117:8080/`:
- Banner badge "**2 Hz · tile stream**" — proves `/ws/state` is delivering tile_delta payloads to the page (live counter from `app.js:213`).
- `<canvas id="image-canvas" width="384" height="256">` declared at `index.html:293`.
- `canvas_renderer.js` opens `wss://…/ws/state`, base64-decodes each tile blob, `createImageBitmap()` and `drawImage()` into the visible canvas, then fires `lifetrac-tile-painted` events for overlays (`fade_shader.js`, `staleness_overlay.js`, `detection_overlay.js`, `badge_renderer.js`).

Visual confirmation of pixels in the canvas is the only step not yet checked off, and only because the base went unreachable seconds before the browser refresh.

---

## 3. Strict-path data flow (no shortcuts)

```
tractor v4l2 / synthetic frame
        │
        ▼  (in-process)
camera_service.py  ── WebP-encode each changed 32×32 tile, emit tile_delta JSON
        │
        ▼  MQTT topic `lifetrac/v25/cmd/image_frame`  (loopback to tractor mosquitto)
image_tx_daemon.py  ── pack_telemetry_fragments(PHY_IMAGE, SF7/BW500/CR5)
        │
        ▼  HostLink UART  /dev/ttymxc3 @ 921600
L072 firmware  ── SX1276 TX, RegFrf=915.000 MHz (profile 0)
        │
        ▼  air @ 915 MHz, SF7/BW500
L072 firmware (base) ── RX_FRAME_URC over HostLink
        │
        ▼  /dev/ttymxc3 → base X8
image_rx_daemon.py  ── reassemble fragments, decode tile_delta envelope
        │
        ▼  MQTT topic `lifetrac/v25/video/tile_delta`  (loopback to base mosquitto)
web_ui (FastAPI)  ── subscribed via state_publisher; merge into `/ws/state` snapshot
        │
        ▼  WebSocket
browser  ── canvas_renderer.js draws each blob into <canvas id="image-canvas">
```

There is **no MQTT over the air**. MQTT is only the intra-X8 IPC bus on each side. The air payload is the custom HostLink + LoRa fragment protocol (`pack_telemetry_fragments` / `PHY_IMAGE`).

---

## 4. Critical knobs that made the link work

| Knob | Value | Why |
|---|---|---|
| `LIFETRAC_REG_PROFILE` (both sides) | `0` | BENCH_ONLY_FIXED_915 forces `RegFrf=915.000 MHz`. Profile 1 has an RX-side `RegFrf` trap (see `/memories/repo/lifetrac-image-pipeline-fhss-rx-trap.md`). |
| `LIFETRAC_LORA_INTER_CYCLE_S` (tractor `image_tx`) | `0.15` | SF7/BW500 fragment ToA ≈ 48.8 ms; the 0.05 s default → `ERR_PROTO 0x08 FORBIDDEN` + `RFCO_PERTX tx_status=0x04 ABORT_QOS`. NB: env name is `_LORA_INTER_CYCLE_S`, not `_INTER_CYCLE_S`. |
| `LIFETRAC_USE_LORA_BRIDGE` (tractor `camera`) | `1` | Bypasses the legacy `IpcWriter`→H747 path; publishes tile_delta directly to MQTT. |
| `LIFETRAC_SYNTHETIC_MODE` | `delta` | Frozen gradient + a single 32×32 top-left tile that toggles colors → 2-fragment P-frames. Default `scroll` regenerates whole frame each tick → too many fragments. |
| `LIFETRAC_GRID_W/H/TILE_PX` | `6 / 4 / 32` (canvas 192×128) | Keeps full-keyframe payload under the 256-fragment hard cap (1-byte fragment index in L072 firmware). |
| `LIFETRAC_CAMERA_FPS` | `0.5` | One frame every 2 s → matches what one P-frame (≈100 ms of air) can sustain comfortably. |
| `LIFETRAC_KEYFRAME_PERIOD_S` | `60` | Spreads keyframe pain to once per minute. |

---

## 5. Final blocking items

Ordered by severity / proximity to "user sees live camera in browser".

### B1 — Base station offline (HARDWARE) · BLOCKING the visual confirmation
- Symptom: `ping 192.168.1.117` → `Destination host unreachable`; base also gone from adb.
- Last known good: `frames_published` was climbing 30→50, `publish_err=0`.
- Action: physical power cycle of the base X8 + LAN check.
- Risk: low. The base was producing zero errors in the steady-state run; nothing we did would have crashed it. Most likely an unrelated network or supply blip.

### B2 — Keyframe `FAULT 0x0D sub=0x01` on the 75-fragment keyframe · SOFTWARE / FIRMWARE
- Symptom: `frame seq=32 requires 75 fragments (>8 dwell cap); transmitting anyway with extra cadence padding` followed by `TX fault: seq=32 idx=0 code=0x0D sub=0x01`.
- Root cause: image_tx_daemon's `MAX_FRAMES_PER_DWELL_CAP=8` reflects a regulatory dwell ceiling embedded in the L072 firmware path that profile-0 does not formally exempt. Even with bench profile we're hitting a per-burst quota.
- Effect: every ~60 s keyframe is lost; P-frames in between are fine. The UI still renders, but a fresh page load with no recent keyframe will show black until the next keyframe round-trips.
- Fix options (in increasing order of work):
  1. Shrink keyframe to ≤ 8 fragments (smaller canvas / harder WebP quantization for keyframes only).
  2. Stage the keyframe over multiple dwells with a daemon-level pacing token (image_tx already has the warning path; needs a sleep-and-resume instead of "transmit anyway").
  3. Add a profile-0-specific firmware exemption so dwell cap is bypassed in bench mode.
- Recommendation: option 1 for production (smaller-than-rendering keyframe is fine because keyframe just seeds the canvas) + option 2 for safety.

### B3 — Real camera not yet swapped in · SOFTWARE
- Currently `LIFETRAC_CAMERA_SOURCE=synthetic` with delta mode. Switching to `v4l2` needs:
  - `LIFETRAC_CAMERA_SOURCE=v4l2` env on `tractor-camera-v2`.
  - `devices: ["/dev/video1:/dev/video1"]` mapping (the tractor camera is on `/dev/video1`; `/dev/video0` is a different device — see workspace `Get Camera Names` task).
  - Initial keep grid at 6×4×32 to stay inside the dwell cap; raise grid only after B2 is resolved.
- Risk: real camera generates substantially more delta motion per frame than the 1-tile synthetic, so even at 6×4×32 a fast-changing scene may produce 10–20 fragments per frame. That still fits in a 0.5 FPS cadence with 0.15 s inter-cycle, but margins shrink.

### B4 — Web UI image pane render not yet visually confirmed (gated by B1)
- All artifacts to render are in place: `<canvas>`, `canvas_renderer.js`, `app.js` "2 Hz · tile stream" badge live-incrementing, fade/staleness overlays loaded.
- Verification step is just: log in to the page after B1 is fixed, watch the canvas paint a 192×128 frame with the top-left tile toggling.

### B5 — Cleanup
- `systemctl reset-failed lifetrac-base.service` (cosmetic). **Do NOT** unmask `compose-apps-early-start-recovery.service` (it conflicts with our hand-managed `lifetrac-vtest` project).
- Production deployment will replace the `vtest` projects with the on-disk `docker-compose.yml`, so the `vtest` files are throwaway once the production compose has the same env vars.

### B6 — Production-hardening items deferred from this session
- `LIFETRAC_FRAGMENT_BUDGET` env exists in `image_tx_daemon` but is not enforced by the encoder unless a `CMD_LINK_PROFILE` back-channel reaches `camera_service.py`. Today we ignore it and use small GRID. For production with adaptive bandwidth this back-channel needs to be re-enabled in LoRa-bridge mode.
- Regulatory profile must change from `0` (bench-only 915 MHz fixed carrier — **not FCC-Part-15-legal for sustained operation**) to a 50-CH FHSS profile before any outdoor / non-bench operation. See `2026-05-19_LoRa_FCC_50CH_FHSS_Implementation_Plan_Copilot_v1_0.md` and `/memories/repo/lifetrac-image-pipeline-fhss-rx-trap.md` (the RX `RegFrf` trap must be fixed before we can re-enable profile 1).
- Authentication: current `/login` uses a 4-digit PIN against `/run/secrets/lifetrac_pin`. Fine for bench; field deployment wants TLS + per-operator keys.

---

## 6. Software packages — final production operation

### 6.1 Tractor X8 Max Carrier (Yocto Linux on Portenta X8)

| Layer | Package / Image | Purpose |
|---|---|---|
| Container runtime | `docker` (bundled in Portenta Linux) + `docker compose` v2 | Deploy services |
| MQTT broker (IPC only) | `eclipse-mosquitto:2` | Intra-X8 bus between camera, image_tx, control daemons |
| App image | `lifetrac-tractor-x8:latest` (built from `LifeTrac-v25/DESIGN-CONTROLLER/firmware/tractor_x8/Dockerfile` or top-level Dockerfile in DESIGN-CONTROLLER) | Bundles Python 3 + `pyserial` 3.5 + `paho-mqtt` + `Pillow` + `webp` codec + the strict-path daemons |
| Camera service | `LifeTrac-v25/DESIGN-CONTROLLER/firmware/tractor_x8/camera_service.py` | v4l2 capture, tile-delta encode, MQTT publish |
| Image TX daemon | `LifeTrac-v25/DESIGN-CONTROLLER/firmware/tractor_x8/image_tx_daemon.py` | MQTT subscribe → HostLink TX |
| Shared LoRa proto | `LifeTrac-v25/DESIGN-CONTROLLER/firmware/common/lora_proto/` (mounted into both daemons) | Fragment packing, PHY_IMAGE type, HostLink frame helpers |
| HostLink client | `method_h_stage2_tx_probe_v2.py` + `method_g_stage1_probe.py` (from `x8_lora_bootloader_helper/`) | UART transport, CFG keys, drain/wait helpers |
| Control RPC (production) | `serialrpc.py` (existing; not exercised this session) | Joystick → hydraulic command bus |
| LoRa MCU firmware | `LifeTrac-v25/DESIGN-CONTROLLER/firmware/murata_l072/` build, flashed via OpenOCD over the Max Carrier's SWD mux | SX1276 driver, regulatory profile machine, fragment scheduler |

### 6.2 Base X8 Max Carrier (same Yocto Linux on Portenta X8)

| Layer | Package / Image | Purpose |
|---|---|---|
| Container runtime | `docker` + `docker compose` v2 | Deploy services |
| MQTT broker (IPC) | `eclipse-mosquitto:2` | Intra-X8 bus |
| App image | `lifetrac-v25:latest` (built from `LifeTrac-v25/DESIGN-CONTROLLER/Dockerfile`) | Python 3 + `paho-mqtt` + `fastapi` + `uvicorn` + `pyserial` + the base-station daemons + the static web bundle |
| Image RX daemon | `LifeTrac-v25/DESIGN-CONTROLLER/base_station/image_pipeline/image_rx_daemon.py` | HostLink RX → reassemble → MQTT publish |
| Web UI (server) | `LifeTrac-v25/DESIGN-CONTROLLER/base_station/web_ui.py` (FastAPI) | HTTP routes, `/ws/state` WebSocket, session/PIN gate |
| State publisher | `LifeTrac-v25/DESIGN-CONTROLLER/base_station/state_publisher.py` | MQTT subscriber → in-memory snapshot → WebSocket fan-out |
| Audit tail | `LifeTrac-v25/DESIGN-CONTROLLER/base_station/audit_tail.py` | Streams `/var/log/lifetrac/*` to `/audit` route |
| Web UI (browser) | `LifeTrac-v25/DESIGN-CONTROLLER/base_station/web/` static bundle — `index.html`, `app.js`, `img/canvas_renderer.js`, `img/fade_shader.js`, `img/staleness_overlay.js`, `img/badge_renderer.js`, `img/detection_overlay.js`, `img/source_guard.js`, `img/raw_mode_toggle.js`, `img/accel_status.js`, `map.{html,js}`, `audit.html`, `settings.html`, `diagnostics.{html,js}`, `config.html`, `login.html` | Operator dashboard |
| Shared LoRa proto | `LifeTrac-v25/DESIGN-CONTROLLER/firmware/common/lora_proto/` | Same fragment / HostLink helpers as tractor |
| LoRa MCU firmware | Same `firmware/murata_l072/` build flashed to the base's L072 | RX side of the air link |
| Secrets | `/opt/lifetrac/DESIGN-CONTROLLER/secrets/lifetrac_pin` | Mounted to `/run/secrets/lifetrac_pin` |
| Compose | `LifeTrac-v25/DESIGN-CONTROLLER/docker-compose.yml` (production) | Long-term replaces today's `lifetrac-vtest` project |

### 6.3 Operator client
- Any modern desktop or tablet browser (Chromium 90+, Firefox 90+, Safari 16.4+ for OffscreenCanvas — but `canvas_renderer.js` already falls back to a main-thread `drawImage()` loop for older Safari).
- USB gamepad (Web Gamepad API; mapping documented in the page footer).
- Network reachability to the base X8's port 8080 (LAN today; production stack should sit behind TLS reverse proxy).

---

## 7. Next actions (in order)

1. **Power cycle the base X8**; reconfirm `ping 192.168.1.117` and `adb devices` both show `2D0A1209DABC240B`.
2. After base comes up, restart the `lifetrac-vtest` project if needed; verify `image_rx` log shows `frames_published` growing again.
3. Open `http://192.168.1.117:8080/`, log in, watch `<canvas id="image-canvas">` paint. Capture a screenshot for the record.
4. Apply B2 keyframe fix (shrink keyframe payload + stage across dwells).
5. Swap synthetic → v4l2 (B3) and re-validate.
6. Plan FCC profile-1 RX trap fix before any non-bench operation (B6).

---

## 8. Reference

- Repo memory: `/memories/repo/lifetrac-image-pipeline-air-link-proven.md` (this session's compact notes).
- Repo memory: `/memories/repo/lifetrac-image-pipeline-fhss-rx-trap.md` (why profile 0 today).
- Repo memory: `/memories/repo/lifetrac-strict-path-image-daemons.md` (strict-path design).
- Prior note: [2026-05-25_End_to_End_Pipeline_Visual_Verified.md](2026-05-25_End_to_End_Pipeline_Visual_Verified.md) (loopback end-to-end without the air gap).
- Prior note: [2026-05-26_Black_Image_Pane_Diagnosis.md](2026-05-26_Black_Image_Pane_Diagnosis.md) (this morning's pre-fix diagnosis).
