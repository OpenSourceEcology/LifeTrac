# Base Station — Portenta Max Carrier + Portenta X8

> **Scope (2026-04-26):** LoRa-only per [MASTER_PLAN.md](MASTER_PLAN.md). The "cellular fallback" section and any Cat-M1 references in this document are **archived**. The retained non-LoRa path is the operator's browser reaching the base station X8 over LAN/WiFi/Ethernet — the wireless link to the tractor is LoRa.
>
> **MQTT note:** Mosquitto on the base-station X8 is **in scope** as a base-station-internal IPC bus (loopback or LAN-only) between `lora_bridge`, `web_ui`, the logger, and any future autonomy planner. It is *not* used as a wireless transport to the tractor. The legacy MQTT-over-WiFi-to-Opta path is archived in [RESEARCH-CONTROLLER/](RESEARCH-CONTROLLER/).

The base-station controller. Hosts the operator web UI on a local LAN, bridges LoRa ↔ MQTT, and serves as the long-range RF endpoint for the tractor.

## Hardware overview

| Component | Role |
|---|---|
| Portenta Max Carrier (ABX00043) | Carrier with onboard LoRa (Murata SX1276), Gigabit Ethernet, and the disabled/not-used Cat-M1 hardware present on the board |
| Portenta X8 (ABX00049) | i.MX 8M Mini quad-core Cortex-A53 + M4 + M0; runs Yocto Linux with Docker |
| 8 dBi 915 MHz omni antenna | Mast-mounted for long-range LoRa to tractor |
| LMR-400 coax + lightning arrestor | Mast-to-carrier RF feed |
| ~3 m galvanized mast + ground rod | Outdoor mount |
| 12 V / 5 A indoor power supply + UPS | Survives brief power drops |
| Indoor enclosure (ventilated) | Wall-mount or shelf |

Full BOM in [HARDWARE_BOM.md § Tier 2](HARDWARE_BOM.md#tier-2--base-station-portenta-max-carrier--portenta-x8).

## Software stack (on the X8 Linux side)

```
┌──────────────────────────────────────────────────────────────┐
│  Operator's laptop browser                                   │
│  ────────────────────────────────────────────────────────────│
│  http://lifetrac-base.local                                  │
│  (or http://192.168.1.42, mDNS-resolvable on local LAN)      │
└──────────────────────────┬───────────────────────────────────┘
                           │ HTTP/WebSocket
                           ▼
┌──────────────────────────────────────────────────────────────┐
│  Portenta X8 (Yocto Linux)                                   │
│  ────────────────────────────────────────────────────────────│
│  Docker containers:                                          │
│    • nginx          (reverse proxy, static UI, no TLS in v25) │
│    • web_ui         (Python FastAPI + Jinja2 + WebSockets)    │
│    • mosquitto      (MQTT broker, port 1883)                  │
│    • lora_bridge    (Python: SX1276 SPI ↔ MQTT; serial bench  │
│                      fallback until the SPI driver lands)      │
│    • timeseries     (InfluxDB or SQLite for telemetry log)    │
│    • grafana        (optional: telemetry dashboards)          │
└──────────────────────────┬───────────────────────────────────┘
                           │ SPI + GPIO IRQ on Max Carrier
                           ▼
┌──────────────────────────────────────────────────────────────┐
│  Murata LoRa SiP (SX1276)                                    │
│  ────────────────────────────────────────────────────────────│
│  • Raw LoRa P2P modem driven directly by Linux               │
│  • No active base-station H747 firmware target               │
└──────────────────────────┬───────────────────────────────────┘
                           │ RF
                           ▼
                  Murata LoRa SiP → SMA → mast antenna
```

## Web UI design

### Pages

| URL | Purpose |
|---|---|
| `/` | Live operator console: dual virtual joysticks (touch/mouse), implement buttons, video tile, telemetry sidebar |
| `/map` | GPS track + waypoints (Leaflet + OpenStreetMap tiles cached locally) |
| `/telemetry` | Time-series graphs (engine RPM, hydraulic pressures, battery V) |
| `/log` | Recent events: source-handover, errors, faults |
| `/settings` | Channel selection, antenna gain config, key rotation, network setup |
| `/diagnostics` | RSSI, packet loss, link state for each radio |

### Live operator console (`/`)

Layout:

```
┌─────────────────────────────────────────────────────────────────┐
│  LifeTrac v25 — Base Station          [HANDHELD HAS CONTROL]    │
│  ────────────────────────────────────────────────────────────── │
│  ┌──────────────┐  ┌────────────────────────┐  ┌──────────────┐ │
│  │              │  │ [FRONT][REAR][IMPL][AUTO*] │ Telemetry    │ │
│  │  LH joystick │  │   Live video tile      │  │ ────────────│ │
│  │  (drive)     │  │   (WebRTC from camera) │  │ RPM:  1850  │ │
│  │              │  │   active: front · t-2s │  │ OilT: 78°C  │ │
│  └──────────────┘  └────────────────────────┘  │ Bat:  13.2V │ │
│  ┌──────────────┐  ┌────────────────────────┐  │ GPS:  fix 8 │ │
│  │              │  │   [BUCKET CURL]        │  │ RSSI: -78dB │ │
│  │  RH joystick │  │   [BUCKET DUMP]        │  │ Loss: 0.2%  │ │
│  │  (boom/aux)  │  │   [AUX 1] [AUX 2]      │  │ NDVI: 0.71  │ │
│  │              │  │   [E-STOP - LARGE]     │  │ Source:     │ │
│  └──────────────┘  └────────────────────────┘  │  HANDHELD   │ │
│                                                  └──────────────┘ │
│  ────────────────────────────────────────────────────────────── │
│  Status bar: LoRa link OK · SF7 · 0.2% pkt loss                │
└─────────────────────────────────────────────────────────────────┘
```

Behavior:
- When source is HANDHELD or AUTONOMY, the on-screen joysticks are disabled and shown greyed out (the base UI cannot send control while a higher-priority source is active).
- "REQUEST CONTROL" button (visible when not active) — sends a request frame; takes effect when handheld releases TAKE CONTROL or after handheld heartbeat times out.
- E-STOP button is **always active** regardless of source; sends authoritative E-stop over LoRa AND cellular for redundancy.
- **Camera switcher** above the video tile picks which camera produces the LoRa thumbnail / WebRTC stream. `[AUTO*]` is the default and means the tractor X8 picks based on operating mode (forward → front, sustained reverse stick → rear, parked → last selected). Manual buttons pin the selection until `[AUTO*]` is re-selected; a small `MANUAL` badge shows when pinned. Active camera is echoed back over LoRa on `topic 0x22` so the badge reflects what the tractor is actually sending, not just what the UI requested. See [LORA_PROTOCOL.md § Command frame opcodes](LORA_PROTOCOL.md#command-frame-opcodes) (`CMD_CAMERA_SELECT`).
- The **NDVI** mini-readout in the telemetry sidebar comes from `topic 0x24` `lifetrac/v25/telemetry/crop_health` — a 30 B summary computed onboard the X8 (see [VIDEO_OPTIONS.md § Crop-health analysis](VIDEO_OPTIONS.md#crop-health-analysis)). The full per-row heatmap is on the `/map` view.

### Joystick widget

- Touch-friendly virtual stick (works on tablets too)
- Outputs values 50 Hz as joystick-X, joystick-Y in [-127, +127]
- Sent via WebSocket -> `web_ui.py`, which packs a 16-byte `ControlFrame` and publishes it to `lifetrac/v25/cmd/control` at 20 Hz.
- `lora_bridge.py` validates the frame, encrypts it, and transmits it over the base X8 LoRa transport.

### WebSocket message format (browser ↔ web_ui)

```json
{
  "lhx": 0,
  "lhy": 0,
  "rhx": 32,
  "rhy": -45,
  "buttons": 0,
  "flags": 0
}
```

```json
{
  "type": "telemetry",
  "topic": "lifetrac/v25/telemetry/engine",
  "data": { "rpm": 1850, "coolant_c": 78.4, "oil_p_psi": 42 }
}
```

## MQTT topic conventions

All topics under `lifetrac/v25/`. Mosquitto runs in a container on the X8 and the lora_bridge translates LoRa frames to/from MQTT.

| Direction | Topic | Producer | Consumer |
|---|---|---|---|
| ↓ control | `.../control/base/{lh_x,lh_y,rh_x,rh_y,buttons,estop}` | web_ui | lora_bridge |
| ↓ command | `.../cmd/camera_select` | web_ui | lora_bridge → tractor (`CMD_CAMERA_SELECT`) |
| ↓ command | `.../cmd/estop`, `.../cmd/clear_estop` | web_ui | lora_bridge → tractor |
| ↑ telemetry | `.../telemetry/{gps,engine,battery,hydraulics,mode,errors,imu,sensor_faults,crop_health}` | lora_bridge (from tractor) | web_ui, timeseries |
| ↑ video | `.../video/{thumbnail,thumbnail_rear,thumbnail_implement,active_camera}` | lora_bridge (from tractor) | web_ui |
| ↑ source state | `.../control/source_active` | lora_bridge | web_ui |
| ↕ link health | `.../link/lora/{rssi,snr,loss}` | lora_bridge | web_ui, diagnostics |
| ↕ link health | `.../link/cellular/{state,rssi}` | lora_bridge | web_ui, diagnostics |

## Mast antenna setup

```
                        ┌────────────────┐
                        │  8 dBi omni    │
                        │  915 MHz       │
                        │  N-female      │
                        └───────┬────────┘
                                │
                          N-male connector
                                │
                       ┌────────┴────────┐
                       │  ~3 m mast      │
                       │  galvanized     │
                       │  steel pipe     │
                       └────────┬────────┘
                                │
                         LMR-400 coax
                         (3 m, low loss
                          ~1.2 dB at 915)
                                │
                       ┌────────┴────────┐
                       │  Lightning      │
                       │  arrestor       │
                       │  (N-F bulkhead) │
                       │  → ground rod   │
                       └────────┬────────┘
                                │
                       N-male to SMA-male
                       jumper (~30 cm)
                                │
                                ▼
                       Max Carrier J9 (LoRa SMA)
```

**EIRP check (FCC §15.247):** +20 dBm TX − 1.2 dB cable − 0.5 dB connectors + 8 dBi antenna = **+26.3 dBm EIRP**. FCC limit for 915 MHz DTS is +36 dBm (4 W) EIRP, so we have ~10 dB headroom and can step up to a 12 dBi Yagi if needed.

## Networking

- Static IP recommended on the office LAN (e.g., 192.168.1.42)
- Hostname: `lifetrac-base` (publish via Avahi/mDNS so `lifetrac-base.local` works on macOS/Linux/iOS; Windows needs Bonjour Print Services or `lifetrac-base.local` may need an explicit hosts entry)
- Firewall: allow inbound 80/443 from LAN only; Mosquitto 1883 LAN-only; never expose to public internet
- TLS: self-signed cert acceptable for LAN; use Let's Encrypt only if the base station is behind a publicly-routable hostname

## Link-loss behavior

When the LoRa link to the tractor is lost (no telemetry for 30 s), the base UI shows a banner, stops sending joystick `ControlFrame`s, and leaves E-stop available. There is no Cat-M1 or MQTT-over-cellular fallback in v25; recovery is LoRa link restoration or a local hardware/service intervention.

## Bring-up checklist

1. [ ] Flash X8 with latest Yocto image, verify SSH access
2. [ ] Install Docker on X8 (`apt install docker-ce` in the Yocto container manager)
3. [ ] Pull/build containers: nginx, web_ui (FastAPI), mosquitto, lora_bridge
4. [ ] Verify M7 + Murata LoRa TX/RX with bench partner
5. [ ] Mount mast, run coax, verify SWR < 2:1 with VNA
6. [ ] Verify lightning arrestor grounded to dedicated rod (not building electrical ground)
7. [ ] Connect Ethernet to office LAN, verify mDNS works
8. [ ] Browser smoke-test: load `/`, see joystick widget, see live telemetry from a bench partner
9. [ ] End-to-end test: laptop joystick → base UI → LoRa → tractor → valves
10. [ ] 24-hour soak with telemetry continuously logged

## Source code layout

```
base_station/
├── lora_bridge.py              # LoRa transport ↔ MQTT bridge; serial bench fallback today, SPI target per MASTER_PLAN.md §8.2
├── lora_proto.py               # Python mirror of firmware/common/lora_proto
├── link_monitor.py             # Airtime ledger + CMD_ENCODE_MODE hysteresis
├── web_ui.py                   # FastAPI app + WebSocket handlers
├── requirements.txt
├── web/
│   ├── index.html              # Operator console
│   └── app.js                  # Gamepad/touch controls + telemetry UI
├── image_pipeline/             # Planned; see § Image pipeline below
│   ├── canvas.py               # Persistent tile canvas, I/P-frame reassembly
│   ├── reassemble.py           # Fragment → TileDeltaFrame
│   ├── superres_cpu.py         # Real-ESRGAN-General-x4v3 (CPU fallback, ncnn)
│   ├── superres_coral.py       # Real-ESRGAN Edge-TPU port (optional, used iff Coral online)
│   ├── detect_yolo.py          # YOLOv8-nano CPU / YOLOv8-medium Coral cross-check
│   ├── interp_rife.py          # Optional RIFE frame interpolation
│   ├── inpaint_lama.py         # Optional LaMa-Fourier (Coral only)
│   ├── accel_select.py         # Auto-detect Coral; pick CPU vs Coral path per stage
│   └── models/                 # Bundled INT8 models (CPU and Edge-TPU variants)
├── mosquitto/                  # Planned deployment config
│   └── mosquitto.conf          # Use existing config from ../RESEARCH-CONTROLLER/config/mosquitto.conf (archived)
└── timeseries/
    └── (InfluxDB or SQLite config)
```

## Image pipeline (Portenta X8 Linux side)

Per [MASTER_PLAN.md §8.19](MASTER_PLAN.md) and [`../AI NOTES/2026-04-27_Image_Transmission_InDepth_Analysis_ClaudeOpus4_7_v1_0.md`](../AI%20NOTES/2026-04-27_Image_Transmission_InDepth_Analysis_ClaudeOpus4_7_v1_0.md). The base receives [`TileDeltaFrame`](LORA_PROTOCOL.md#tiledeltaframe-image-pipeline-i--p-frames) on topic `0x25`, reassembles the persistent canvas, runs post-processing, and publishes the result to the web UI over WebSocket.

**Stages**:

1. **Reassemble** — `base_station/lora_bridge.py` collects `TileDeltaFrame` fragments, hands the assembled frame to `image_pipeline/reassemble.py`. Missing fragments after a configurable timeout → tile is marked stale (yellow tint, age in seconds visible in UI).
2. **Canvas update** — `canvas.py` overwrites the changed tiles in the persistent canvas. If `base_seq` does not match the current I, send `CMD_REQ_KEYFRAME` (opcode `0x62`).
3. **Per-tile fade** — when a new tile arrives the GLSL fragment shader on the browser side cross-fades from the prior value over ~3 display frames. Zero CPU cost.
4. **Super-resolution** — Real-ESRGAN-General-x4v3 (BSD) on the canvas. CPU path: ncnn at ~250 ms/frame; Coral path: ~15–30 ms via the Edge-TPU port. `accel_select.py` picks at startup.
5. **Independent safety detector cross-check** — second-pass [YOLOv8-nano](https://github.com/ultralytics/ultralytics) (CPU, ~150 ms) or YOLOv8-medium (Coral, ~25–40 ms) on the reconstructed canvas. Compare class set against the tractor's `0x26 video/detections` sidecar. **Disagreements (especially "base saw a person tractor missed") are highlighted in the UI and logged for v26 model retraining.** Note: Ultralytics models are AGPL-3.0 — see `../AI NOTES/2026-04-27_Image_Transmission_InDepth_Analysis_ClaudeOpus4_7_v1_0.md` §4.1; substitute NanoDet-Plus if AGPL is unacceptable for the OSE codebase.
6. **(Optional) RIFE frame interpolation** — fills in display frames between thumbnail arrivals so a 1.5 s cadence presents as ~15 fps. Coral-only.
7. **(Optional) LaMa-Fourier inpainting** — fills stale tiles when the operator opts in. *Mandatory* "Enhanced" badge on any inpainted tile per the v1.1 LoRa analysis §5.4 safety rules. Coral-only.
8. **Publish to UI** — final canvas + detection metadata + bounding-box vectors + staleness clock per tile go to the WebSocket consumers as a single message.

**Accelerator policy** (per [MASTER_PLAN.md §8.19](MASTER_PLAN.md)):

- The base ships in **CPU-only mode by default**. Stages 4 and 5 fall back to single-frame Real-ESRGAN at lower fps and YOLOv8-nano on CPU; stages 6 and 7 are *unavailable* without an accelerator.
- If a Coral Mini PCIe (or USB) Accelerator is present, `accel_select.py` enables the Coral paths automatically and the UI shows **"AI accelerator: online"**. If Coral fails to enumerate or load drivers, UI shows **"AI accelerator: offline"** and the pipeline transparently degrades to CPU.
- **Coral on the Max Carrier is unproven as of 2026-04-27** — see [MASTER_PLAN.md §8.19](MASTER_PLAN.md). A 2-day validation spike must pass before BOM lock. Until then, all Coral-dependent stages are best-effort and the safety story does not depend on them.
- Even with Coral the base must continue to function if Coral fails mid-operation (thermal, driver crash). `accel_select.py` re-evaluates every 10 s and degrades on the fly.

**Outside-the-box ideas deferred** (documented in image-transmission analysis §6, not in v25 first build): procedural texture synthesis for static background, satellite-imagery far-field rendering, MiDaS depth + 2.5D rendering, learned autoencoder codec (v26).

## Bring-up checklist (additions for image pipeline)

11. [ ] Reassemble + canvas: feed synthetic `TileDeltaFrame` fragments at the bench, confirm I/P logic and `CMD_REQ_KEYFRAME` recovery
12. [ ] CPU-only super-res benchmark: confirm Real-ESRGAN-x4v3 ncnn ≤300 ms/frame on the X8 A53s
13. [ ] CPU-only safety cross-check: YOLOv8-nano ≤200 ms/frame on the X8 A53s (or NanoDet-Plus if AGPL avoidance is required)
14. [ ] **Coral spike (gate before BOM lock)** — install Coral Mini PCIe in the Max Carrier, confirm: PCIe enumeration in `lspci`, `gasket`/`apex` driver loads against the X8 Yocto kernel, sustained inference for 30 min without thermal throttle, slot power budget adequate. Document outcome in [HARDWARE_BOM.md](HARDWARE_BOM.md). If fail, repeat with Coral USB Accelerator.
15. [ ] End-to-end image-pipeline latency: tractor capture → base UI repaint, ≤500 ms p99 in CPU-only mode, ≤300 ms p99 with Coral
16. [ ] UI safety badges: confirm staleness clock, "Enhanced/Synthetic" badge on any non-1:1 pixel, one-click raw-mode toggle, audit-log of "view-mode at command time" in the §8.10 black-box logger

## Trust boundary — server vs. browser

Per [IMAGE_PIPELINE.md §6.1](IMAGE_PIPELINE.md), the browser is **untrusted**
for any decision that affects machine motion. Anything that controls valves,
trips an E-stop, blocks autopilot moves, or sets ROI hints is computed
server-side (in `web_ui.py` + `image_pipeline/*` on the X8) and only the
*result* of that decision is shipped to the browser for display. The browser
is allowed to render polish that doesn't change what the operator commands —
fades, badge labels, status pills, raw-mode toggle.

| Concern                                       | Server-only (`base_station/image_pipeline/*` + `web_ui.py`) | Browser (`web/img/*.js`) |
|-----------------------------------------------|-------------------------------------------------------------|---------------------------|
| Reassembly of `TileDeltaFrame` fragments      | ✅ `image_pipeline/reassemble.py`                            | ❌                        |
| Per-tile badge assignment + trust enum        | ✅ `image_pipeline/canvas.py` + `lora_proto.Badge`           | ❌                        |
| Background-cache fill (Badge.CACHED)          | ✅ `image_pipeline/bg_cache.py`                              | ❌                        |
| Y-only re-colourisation (Badge.RECOLOURISED)  | ✅ `image_pipeline/recolourise.py`                           | ❌                        |
| Motion-vector replay (Badge.PREDICTED)        | ✅ `image_pipeline/motion_replay.py`                         | ❌                        |
| Wireframe replay (Badge.WIREFRAME)            | ✅ `image_pipeline/wireframe_render.py`                      | ❌                        |
| CPU super-res (Badge.ENHANCED)                | ✅ `image_pipeline/superres_cpu.py`                          | ❌                        |
| Independent safety detector (R6)              | ✅ `image_pipeline/detect_yolo.py`                           | ❌                        |
| Detector cross-check + verdict                | ✅ `image_pipeline/detect_yolo.cross_check`                  | ❌                        |
| ROI hint generation (`CMD_ROI_HINT` 0x61)     | ✅ `firmware/tractor_x8/image_pipeline/roi.py`               | ❌                        |
| Keyframe-request decision (`CMD_REQ_KEYFRAME`)| ✅ `image_pipeline/canvas.py` → `web_ui.py` MQTT publish     | ❌                        |
| Encode-mode demotion (`CMD_ENCODE_MODE`)      | ✅ `link_monitor.py` orchestrator                            | ❌                        |
| Audit-log "view-mode at command time"         | ✅ `audit_log.py`                                            | ❌ (sends notifications)  |
| Per-tile OffscreenCanvas painting             | —                                                            | ✅ `canvas_renderer.js`   |
| 3-frame alpha fade animation                  | —                                                            | ✅ `fade_shader.js`       |
| Yellow staleness tint (server-supplied age)   | —                                                            | ✅ `staleness_overlay.js` |
| Badge label text + colour ring                | —                                                            | ✅ `badge_renderer.js`    |
| Badge enforcement (refuse missing/invalid)    | ✅ accepts refusal via `POST /api/health/refusal`            | ✅ `badge_renderer.js`    |
| Bounding-box overlay drawing                  | —                                                            | ✅ `detection_overlay.js` |
| Detector-disagree banner toggle               | ✅ source of truth in state snapshot                         | ✅ `detection_overlay.js` |
| Accel-status pill                             | —                                                            | ✅ `accel_status.js`      |
| Raw-mode local toggle + post-back             | ✅ accepts via `POST /api/audit/view_mode`                   | ✅ `raw_mode_toggle.js`   |

**Fail-closed contract.** A tile that arrives at the browser without a
recognised badge enum is **not painted** (`badge_renderer.js` blacks the
slot, labels it `BADGE?`, and POSTs the refusal back to the server so the
audit_log captures the rejection). The same fail-closed rule applies if the
browser ever fails to receive an `age_ms` field — the staleness overlay
treats missing age as "infinitely stale" rather than "fresh".


## See also

- [HARDWARE_BOM.md § Tier 2](HARDWARE_BOM.md#tier-2--base-station-portenta-max-carrier--portenta-x8)
- [LORA_PROTOCOL.md](LORA_PROTOCOL.md) — what's on the wire over LoRa
- [TRACTOR_NODE.md](TRACTOR_NODE.md) — what the base station talks to
- [VIDEO_OPTIONS.md](VIDEO_OPTIONS.md) — how video gets from tractor camera to the web UI
- [RESEARCH-CONTROLLER/config/mosquitto.conf](RESEARCH-CONTROLLER/config/mosquitto.conf) — broker config carried over from the archived MQTT-era design; bind to LAN/loopback only
- [RESEARCH-CONTROLLER/raspberry_pi_web_controller/](RESEARCH-CONTROLLER/raspberry_pi_web_controller/) — earlier Pi-based web UI; many UI patterns can be reused
