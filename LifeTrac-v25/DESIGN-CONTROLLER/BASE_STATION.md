# Base Station — Portenta Max Carrier + Portenta X8

> **Scope (2026-04-26):** LoRa-only per [MASTER_PLAN.md](MASTER_PLAN.md). The "cellular fallback" section and any Cat-M1 references in this document are **archived**. The retained non-LoRa path is the operator's browser reaching the base station X8 over LAN/WiFi/Ethernet — the wireless link to the tractor is LoRa.
>
> **MQTT note:** Mosquitto on the base-station X8 is **in scope** as a base-station-internal IPC bus (loopback or LAN-only) between `lora_bridge`, `web_ui`, the logger, and any future autonomy planner. It is *not* used as a wireless transport to the tractor. The legacy MQTT-over-WiFi-to-Opta path is archived in [RESEARCH-CONTROLLER/](RESEARCH-CONTROLLER/).

The base-station controller. Hosts the operator web UI on a local LAN, bridges LoRa ↔ MQTT, and serves as the long-range RF endpoint for the tractor.

## Hardware overview

| Component | Role |
|---|---|
| Portenta Max Carrier (ABX00043) | Carrier with onboard LoRa (Murata SX1276) + Cat-M1 (SARA-R412M), Gigabit Ethernet |
| Portenta X8 (ABX00049) | i.MX 8M Mini quad-core Cortex-A53 + M4 + M0; runs Yocto Linux with Docker |
| 8 dBi 915 MHz omni antenna | Mast-mounted for long-range LoRa to tractor |
| LMR-400 coax + lightning arrestor | Mast-to-carrier RF feed |
| ~3 m galvanized mast + ground rod | Outdoor mount |
| Cellular antenna (SMA) | Backup uplink to tractor |
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
│    • nginx          (reverse proxy, static UI, TLS)           │
│    • web_ui         (Python FastAPI + Jinja2 + WebSockets)    │
│    • mosquitto      (MQTT broker, port 1883)                  │
│    • lora_bridge    (Python: serial ↔ MQTT, runs on X8 OR    │
│                      direct from H7 over UART)                │
│    • timeseries     (InfluxDB or SQLite for telemetry log)    │
│    • grafana        (optional: telemetry dashboards)          │
└──────────────────────────┬───────────────────────────────────┘
                           │ UART (over high-density connector)
                           ▼
┌──────────────────────────────────────────────────────────────┐
│  M7 core (firmware/base_h7/base.ino)                         │
│  ────────────────────────────────────────────────────────────│
│  • LoRa modem driver (RadioLib SX1276)                       │
│  • Cellular modem driver (SARA-R412M, optional)              │
│  • Frames bridged byte-stream to/from X8 over UART           │
└──────────────────────────┬───────────────────────────────────┘
                           │ SPI
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
│  Status bar: LoRa link OK · Cellular standby · 0.2% pkt loss   │
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
- Sent via WebSocket → web_ui container → MQTT publish to `lifetrac/v25/control/base/lh_axis` etc.
- `lora_bridge` subscribes, packs into ControlFrame, hands to H7 via UART at 20 Hz

### WebSocket message format (browser ↔ web_ui)

```json
{
  "type": "control",
  "ts_ms": 1714000000123,
  "axes": { "lh_x": 0, "lh_y": 0, "rh_x": 32, "rh_y": -45 },
  "buttons": { "bucket_curl": false, "bucket_dump": false, "estop": false }
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

## Cellular fallback behavior

When LoRa link to tractor is lost (no telemetry for 30 s), the base UI shows a banner and:
- Telemetry continues to flow via cellular (tractor MQTT publishes over Cat-M1)
- Control input is *not* automatically routed over cellular (latency too high; operator must explicitly enable "cellular control mode" with a 2-step confirmation)
- When LoRa heartbeat returns, system auto-reverts to LoRa control

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

## Source code layout (when implemented)

```
firmware/base_h7/
├── base.ino                    # M7: LoRa modem, UART bridge to X8
├── pinmap_max_carrier_h7.h     # Pin definitions
└── platformio.ini

base_station/                   # Runs on X8 in Docker
├── docker-compose.yml          # All services
├── nginx/
│   ├── nginx.conf
│   └── certs/                  # TLS certs (self-signed by default)
├── web_ui/
│   ├── main.py                 # FastAPI app
│   ├── ws.py                   # WebSocket handlers
│   ├── templates/              # Jinja2 HTML
│   ├── static/                 # JS, CSS, joystick widget
│   └── requirements.txt
├── lora_bridge/
│   ├── bridge.py               # Serial ↔ MQTT translation
│   ├── proto.py                # Frame pack/unpack (mirrors firmware/common/lora_proto.cpp)
│   ├── crypto.py               # AES-GCM (mirrors firmware/common/crypto.cpp)
│   └── requirements.txt
├── mosquitto/
│   └── mosquitto.conf          # Use existing config from ../RESEARCH-CONTROLLER/config/mosquitto.conf (archived)
└── timeseries/
    └── (InfluxDB or SQLite config)
```

## See also

- [HARDWARE_BOM.md § Tier 2](HARDWARE_BOM.md#tier-2--base-station-portenta-max-carrier--portenta-x8)
- [LORA_PROTOCOL.md](LORA_PROTOCOL.md) — what's on the wire over LoRa
- [TRACTOR_NODE.md](TRACTOR_NODE.md) — what the base station talks to
- [VIDEO_OPTIONS.md](VIDEO_OPTIONS.md) — how video gets from tractor camera to the web UI
- [RESEARCH-CONTROLLER/config/mosquitto.conf](RESEARCH-CONTROLLER/config/mosquitto.conf) — broker config carried over from the archived MQTT-era design; bind to LAN/loopback only
- [RESEARCH-CONTROLLER/raspberry_pi_web_controller/](RESEARCH-CONTROLLER/raspberry_pi_web_controller/) — earlier Pi-based web UI; many UI patterns can be reused
