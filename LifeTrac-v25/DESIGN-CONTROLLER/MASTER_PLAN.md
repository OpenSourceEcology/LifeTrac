# LifeTrac v25 — Controller Master Plan

**Date:** 2026-04-26
**Status:** Canonical scope and architecture for the v25 controller build. This document is the single source of truth for *what we are building*. Anything in `RESEARCH-CONTROLLER/` is research and prior-art, not commitment.

---

## 1. Scope

The LifeTrac v25 controller is a **LoRa-only wireless control system** for the v25 tractor.

### In scope

- **Tractor onboard controller** — Arduino Portenta Max Carrier + Portenta X8, driving an Arduino Opta (with D1608S + A0602 expansions) over RS-485 / Modbus-RTU as the industrial valve I/O layer.
- **Base station** — second Arduino Portenta Max Carrier + Portenta X8, hosting a website-based control interface for an operator on the local LAN.
- **Wireless link tractor ↔ base station** — custom LoRa stack on the Murata CMWX1ZZABZ-078 SiP (Semtech SX1276), 915 MHz US / 868 MHz EU. This is the **only** RF link to the tractor.
- **Optional handheld** — Arduino MKR WAN 1310 with the same Murata SiP, joysticks, E-stop, OLED. Highest-priority control source when present.
- **Operator-browser ↔ base-station link** — local LAN over Ethernet/WiFi. The browser reaches the base station X8's web UI; the base station then talks to the tractor over LoRa. This is *not* a wireless link to the tractor; it is the operator's desk-to-base path.

### Out of scope (archived)

- **WiFi / MQTT-over-WiFi to the tractor.** Earlier v25 designs ran the Opta on tractor WiFi with an MQTT broker on a Raspberry Pi. Archived to [`RESEARCH-CONTROLLER/`](RESEARCH-CONTROLLER/) (`arduino_opta_controller/`, `esp32_remote_control/`, `raspberry_pi_web_controller/`, `config/`, `test_scripts/`, related docs). **Note:** MQTT itself is *not* archived — it remains in scope as the **base-station-internal IPC bus** (Mosquitto on the X8, loopback/LAN-only) between `lora_bridge`, `web_ui`, the logger, and any future autonomy planner. What is archived is MQTT as a *wireless control transport to the tractor*; the wireless link is LoRa frames, period.
- **BLE / DroidPad.** The Bluetooth direct-control path on the Opta is archived in `RESEARCH-CONTROLLER/` (`DROIDPAD_BLE_SETUP.md`, `DROIDPAD_INTEGRATION.md`, `MODE_SWITCH_WIRING.md`, etc.).
- **Cellular (Cat-M1 / SARA-R412M / LTE).** No SIMs, no cellular modems, no "cellular fallback" telemetry path. Removed from the BOM and from the active control flow. Top-level docs that still mention cellular contain a banner pointing here; treat those mentions as historical.
- **ROS2 bridge over MQTT.** The ROS2 integration prototype is archived; not part of the v25 build.
- **Alternative wireless technologies.** The full XBee / ELRS / 5G / Meshtastic comparison lives in the archived [`RESEARCH-CONTROLLER/WIRELESS_OPTIONS.md`](RESEARCH-CONTROLLER/WIRELESS_OPTIONS.md). Historical reference only — the v25 build is committed to LoRa on the Murata SX1276.

---

## 2. The system

```
┌────────────────────┐                         ┌────────────────────────────┐
│  Handheld Remote   │   LoRa 915 MHz          │  Tractor                   │
│  (OPTIONAL)        │ ◄────────────────────►  │  Portenta Max Carrier      │
│  MKR WAN 1310      │   proximity link        │  + Portenta X8 (Linux)     │
│  Murata CMWX1ZZ    │                         │  + Murata CMWX1ZZ          │
└────────────────────┘                         │                            │
                                                │  ──────RS-485 / Modbus───▶│
                                                │  Arduino Opta WiFi*        │
                                                │  + D1608S (8× SSR 24VDC/2A)│
                                                │  + A0602 (2× 0–10 V)       │
                                                │  → 8× hydraulic valves     │
                                                │  → 1–2× Bürkert 8605 flow  │
                                                └────────────┬───────────────┘
                                                             │ LoRa 915 MHz
                                                             │ (mast antenna,
                                                             │  long range)
                                                             ▼
                          ┌──────────────────────────────────────────────────┐
                          │  Base Station                                    │
                          │  Portenta Max Carrier + Portenta X8 (Linux)      │
                          │  + Murata CMWX1ZZ                                │
                          │                                                  │
                          │  • lora_bridge  (LoRa ↔ MQTT, Python)            │
                          │  • web_ui       (FastAPI, joysticks + telemetry) │
                          │  • mosquitto    (loopback IPC broker)            │
                          │  • nginx        (static + reverse proxy, no TLS) │
                          └────────────────────────┬─────────────────────────┘
                                                   │ Ethernet / WiFi (LAN)
                                                   ▼
                                         Operator browser (laptop/tablet)
```

\* The Opta's WiFi **and BLE radios are physically present but disabled in v25 firmware**. The Opta is used **only** as a Modbus-RTU slave on RS-485 from the Max Carrier's J6 connector — it is the hydraulic-system industrial-I/O layer, nothing else. No WiFi station/AP, no BLE GATT server, no MQTT client on the Opta. (The legacy Opta firmware that exposed WiFi/MQTT/BLE control surfaces is archived in [`RESEARCH-CONTROLLER/arduino_opta_controller/`](RESEARCH-CONTROLLER/arduino_opta_controller/).) Bench debugging uses Modbus-over-RS-485 or the Opta's USB-C, not its radios.

---

## 3. Hardware (canonical)

Authoritative BOM is [`HARDWARE_BOM.md`](HARDWARE_BOM.md). Cellular line items there are archived per this plan. The minimum-viable v25 build is:

| Tier | Part | Role |
|---|---|---|
| Tractor | Arduino Portenta Max Carrier (ABX00043) | Carrier with LoRa Murata SiP, RS-485, USB host, expansion |
| Tractor | Arduino Portenta X8 (ABX00049) | Linux + onboard STM32H747 co-MCU (M7+M4) |
| Tractor | Arduino Opta WiFi (AFX00002) | Industrial PLC, Modbus-RTU slave, 4× onboard relays |
| Tractor | Opta Ext D1608S (AFX00006) | **8× solid-state relays (SSR) 24 VDC / 2 A** + 16× programmable voltage inputs — directional valve coils ride here for low-latency switching (§8.18) |
| Tractor | Opta Ext A0602 | 6× analog in + 2× 0–10 V analog out (Bürkert 8605 flow) |
| Tractor | LoRa 915 MHz whip antenna + cab-roof mount | RF |
| Tractor | **Kurokesu C2-290C** USB camera × 1–3 (front / rear / implement) — *or any UVC-class USB camera* | Cab/rear/implement video into the X8 (USB host) |
| Tractor | **SparkFun Qwiic GPS — u-blox NEO-M9N** (GPS-15733) | Position + time-of-day; PPS for X8 clock discipline |
| Tractor | **Adafruit BNO085 9-DoF IMU** (Stemma QT / Qwiic) | Heading, tilt, vibration; sensor fusion onboard |
| Tractor | **Adafruit FT232H breakout (USB → I²C, Stemma QT/Qwiic)** | Brings the Qwiic GPS + IMU onto the X8's USB bus instead of contesting the H747 I²C; one adapter daisy-chains both via Qwiic cables |
| Tractor | Phoenix Contact PSR safety relay | Hardware E-stop chain |
| Tractor | Industrial enclosure, DIN rail, 12 V power, fuses, EMI filter | Power and packaging |
| Base | Arduino Portenta Max Carrier (ABX00043) | Same as tractor |
| Base | Arduino Portenta X8 (ABX00049) | Linux host for web UI / broker / bridge |
| Base | 8 dBi 915 MHz mast antenna + LMR-400 + lightning arrestor | Long-range RF |
| Base | Indoor 12 V supply, mini UPS, Ethernet **or WiFi** to LAN (per §8.13) | Power and network |
| Handheld *(optional)* | Arduino MKR WAN 1310 (ABX00029) | Same Murata SiP as tractor/base |
| Handheld *(optional)* | Joysticks ×2, E-stop, OLED, LiPo, IP54 enclosure | Operator I/O |

---

## 4. Software (canonical)

Authoritative source layout, matching [`TODO.md`](TODO.md):

### Implementation tree

```
DESIGN-CONTROLLER/
├── MASTER_PLAN.md                  ← this document
├── ARCHITECTURE.md                 ← system overview
├── LORA_PROTOCOL.md                ← air-interface spec
├── TRACTOR_NODE.md                 ← tractor wiring + Modbus map
├── BASE_STATION.md                 ← base-station design
├── HANDHELD_REMOTE.md              ← optional handheld
├── HARDWARE_BOM.md
├── TODO.md
├── ARDUINO_CI.md
├── arduino_libraries.txt
├── VIDEO_OPTIONS.md                ← camera analysis (LoRa thumbnails)
│
├── firmware/                       ← Arduino sketches, draft
│   ├── handheld_mkr/               ← MKR WAN 1310 (optional)
│   ├── tractor_h7/                 ← Portenta X8 onboard H747 co-MCU (M7+M4)
│   ├── tractor_x8/                 ← Portenta X8 Linux services (sensors, log, video)
│   ├── tractor_opta/               ← Opta Modbus-RTU slave
│   └── common/
│       └── lora_proto/             ← shared C/Python LoRa frame layer
│
├── base_station/                   ← Python services on base-station X8
│   ├── lora_bridge.py              ← LoRa ↔ MQTT
│   ├── web_ui.py                   ← FastAPI operator UI
│   ├── web/                        ← static HTML/JS
│   └── requirements.txt
│
└── RESEARCH-CONTROLLER/            ← archived prior art and research
    ├── README.md
    ├── WIRELESS_OPTIONS.md         ← archived wireless comparison
    ├── arduino_opta_controller/    ← legacy WiFi/MQTT/BLE Opta firmware
    ├── esp32_remote_control/       ← legacy ESP32 Qwiic-joystick remote
    ├── raspberry_pi_web_controller/← legacy Pi web UI + MQTT broker
    ├── ros2_bridge/                ← ROS2 prototype
    ├── config/                     ← Mosquitto + WiFi configs
    ├── test_scripts/               ← legacy MQTT bench tests
    ├── PATH_PLANNING/
    ├── VIDEO_COMPRESSION/
    ├── DROIDPAD_*.md, MODE_SWITCH_*.md, IMPLEMENTATION_SUMMARY.md, …
    └── (research notes, code reviews of legacy code, etc.)
```

### Wire contract

- Air interface: [`LORA_PROTOCOL.md`](LORA_PROTOCOL.md) — `ControlFrame`, `HeartbeatFrame`, `TelemetryFrame`, KISS framing, AES-128-GCM, nonce shape.
- Tractor RS-485: [`TRACTOR_NODE.md` § Modbus RTU register map](TRACTOR_NODE.md) — slave 0x01, 50 Hz writes of `valve_coils` + flow setpoints + `watchdog_counter`, 10 Hz reads of telemetry.
- Operator browser ↔ base: WebSocket JSON to `web_ui.py` (FastAPI). The browser does **not** speak MQTT. Internally, `web_ui.py` ↔ `lora_bridge.py` use MQTT topics under `lifetrac/v25/…` on Mosquitto bound to `127.0.0.1` — base-station-internal IPC only.

---

## 5. Bring-up plan (canonical, simplified)

The full task list is in [`TODO.md`](TODO.md). The condensed critical path is:

1. **Procure** the canonical hardware in §3.
2. **Stand up CI**: Arduino-CLI compile workflow for the firmware targets (`handheld_mkr`, `tractor_h7` — the X8's onboard H747 co-MCU sketch — and `tractor_opta`). The base station has no Arduino firmware target (per §8.2). See [`ARDUINO_CI.md`](ARDUINO_CI.md).
3. **Resolve readiness blockers** in [`AI NOTES/CODE REVIEWS/2026-04-26_v25_implementation_readiness_review.md`](../AI%20NOTES/CODE%20REVIEWS/2026-04-26_v25_implementation_readiness_review.md):
    - Pin one canonical Modbus register map and align master + slave + docs.
    - Fix M4 alive-tick (M7 must write `SHARED->alive_tick_ms`).
    - Fix Opta watchdog refresh logic (only on `REG_WATCHDOG_CTR` writes).
    - Make `apply_control()` require a fresh valid `ControlFrame`.
    - Implement `CMD_ESTOP` latch on tractor.
    - Complete `all_coils_off()` for all 8 relays + both 0–10 V outputs.
    - Resolve `ControlFrame` size mismatch (15 vs 16 bytes).
    - Fix handheld AES-GCM nonce/sequence skew.
    - Replace crypto stub or guard with `LIFETRAC_ALLOW_STUB_CRYPTO` for sim builds only.
4. **Bench bring-up with LED stand-ins**: tractor M7+M4 → Opta → 8 LEDs + multimeter on 0–10 V. Verify Modbus link, watchdogs, E-stop drop-out.
5. **LoRa loopback**: base ↔ tractor over the bench. Verify frame parity, key handling, RSSI.
6. **Web UI smoke test**: laptop browser → base UI → LoRa → tractor → LEDs. End-to-end, still no hydraulics.
7. **Single-valve hydraulic test**: low-pressure bench manifold, one solenoid open-loop, written test card, hardware E-stop verified.
8. **Optional handheld**: build only after the base + tractor path is bench-proven.
9. **Field test**: short range first, then mast antenna at the build site.

---

## 6. What goes where, going forward

When considering a change, ask first: **is this LoRa-control / Max-Carrier / Opta-Modbus / web-UI work?**

- **Yes** → it belongs in `DESIGN-CONTROLLER/` (root or `firmware/` or `base_station/`).
- **No** (it is WiFi/BLE/cellular/ROS2/alt-radio research) → it belongs in `DESIGN-CONTROLLER/RESEARCH-CONTROLLER/`. Do not promote without updating this MASTER_PLAN.

When a code review or readiness analysis is performed, scope it to the canonical implementation tree (`firmware/`, `base_station/`, the canonical docs). Items in `RESEARCH-CONTROLLER/` are **not** subject to readiness gates and should not be flagged as blockers for hardware testing.

---

## 7. Open decisions that *would* change this plan

These are the only decisions that, if reversed, would require updating MASTER_PLAN.md before resuming work:

- **Drop Opta + RS-485 in favor of a direct GPIO drive from the Max Carrier H7.** Currently rejected on safety/industrial-IO grounds (Opta gives us PSR-grade relays + galvanic isolation + UL listing). Revisit only if Opta supply-chain blocks the build.
- **Add cellular telemetry as a one-way uplink.** Currently rejected on scope grounds. Revisit only after the LoRa link is field-proven and a real operational gap is identified.
- **Drop the optional handheld permanently.** Currently kept as an optional tier so the design is complete. No code or hardware procurement happens for it until the base + tractor path is bench-proven.
- **Add waypoint-following autonomy and/or vision-based obstacle stop.** Currently rejected for v25 on scope grounds. Wire-protocol slots (Command opcodes `0x40`–`0x44` for autonomy + `0x50` for obstacle stop, telemetry topic IDs `0x40`–`0x43` + `0x50`) are **reserved** in [`LORA_PROTOCOL.md`](LORA_PROTOCOL.md) and one D1608S input + a 4-conductor harness run to the bumper are reserved during v25 assembly per [`RESEARCH-CONTROLLER/AUTOMATION_AND_ROUTE_PLANNING.md`](RESEARCH-CONTROLLER/AUTOMATION_AND_ROUTE_PLANNING.md) §10. **Waypoint plan transport is not on LoRa** — the recommended pattern keeps the plan on the base-station X8 and only streams next-waypoint commands over LoRa. Promotion requires v25 field-proof first.

---

## 8. Pinned decisions and clarifications

The items below were ambiguous in earlier drafts. Each is now pinned to a best-guess default. Items marked **⚠ CONFIRM** still need explicit user sign-off; the rest are inferred from the existing repo and called out here so they don't drift.

### 8.1 LoRa region — **915 MHz US** (canonical)

The build targets **915 MHz US ISM** as the canonical region. EU 868 MHz is a **build-time `#define`** in `firmware/common/lora_proto/`, not a runtime switch. CI builds the US target by default; an EU compile job validates the alternate define but is not the shipped artefact. Antennas in the BOM are 915 MHz; sourcing 868 MHz parts is left as a future port.

### 8.2 Base-station LoRa radio — **Linux on X8 drives the SX1276 directly over SPI, no Arduino firmware**

The base-station Max Carrier exposes the Murata LoRa SiP (SX1276) on SPI. Linux on the X8 talks to it directly from `base_station/lora_bridge.py` using `spidev` plus a Python SX1276 driver (`pySX127x`, a vendored RadioLib port, or a thin in-tree driver). **No Arduino sketch runs on the base station's onboard H747 co-MCU** — there is no `firmware/base_*/` target to build, flash, or CI.

Reasoning: the base is not safety-critical, the radio chip is well-served by Linux SPI, and dropping a firmware target removes a flash/debug surface. If SPI-from-Linux ever proves flaky in practice (hot-plug, IRQ latency, kernel SPI driver quirks), the fallback is to add a small "dumb modem" sketch on the H747 that does only radio init + FIFO drain + UART/serial bytes to Linux. That fallback is explicitly out of scope until a real problem appears.

The tractor is unaffected by this: the tractor's H747 still runs the full M7 (LoRa + arbitration + Modbus master) + M4 (realtime control) firmware because the tractor *is* safety-critical and must keep running if Linux on the X8 reboots.

### 8.3 Control-source priority — **HANDHELD > BASE > AUTONOMY-reserved**

Authoritative in [`LORA_PROTOCOL.md` § Multi-source arbitration](LORA_PROTOCOL.md#multi-source-arbitration) and enforced by `pick_active_source()` on the tractor M4 every 50 ms. When the handheld is **active** (sending Heartbeats at 20 Hz with `priority_request` honored), the base UI is **read-only**: telemetry continues, but joystick inputs are dropped at the base before encoding. The web UI must visually indicate "HANDHELD ACTIVE — controls disabled". `CMD_ESTOP` from any source preempts everything, always.

### 8.4 Operator-browser ↔ base transport — **WebSocket-only to FastAPI**

The browser speaks **WebSocket JSON to `web_ui.py`** (FastAPI). `web_ui.py` is the only MQTT client the browser reaches. Mosquitto on the X8 is **loopback-only (`127.0.0.1`)** and is not exposed to the browser, the LAN, or the internet. This means: no MQTT-over-WebSocket from the browser, no MQTT broker auth surface to harden. MQTT remains pure base-station-internal IPC per §1.

### 8.5 Web-UI auth model — **shared PIN, LAN-only, plain HTTP for v25**

Keep it simple for the first build:

- **Single shared PIN** (4–6 digits) configured on the base station at first boot. Operator enters it on the web UI to unlock controls.
- **LAN-only**, plain HTTP on port 80. The base station is on a private LAN (home/farm network or a dedicated AP); the operator's browser is on the same LAN. No TLS for v25 — the threat model is "physical LAN access = trusted operator", same as the on-base hardware E-stop button.
- Session cookie, HttpOnly + SameSite=strict, idle timeout (e.g. 30 min). Wrong-PIN rate-limited (e.g. 5 attempts → 60 s lockout) to discourage casual scanning.
- Telemetry views may be unauthenticated (read-only) so a phone can glance at status without typing the PIN; **only joystick / valve / E-stop-release controls require the PIN**. The hardware E-stop on the base is auth-independent and always works.

**Future (out of v25 scope, kept as upgrade path):** if the base is ever exposed beyond the LAN, add (a) nginx + TLS with a real cert, (b) per-operator login with hashed passwords (argon2), (c) optional VPN (WireGuard/Tailscale) instead of public exposure. Document that upgrade in a future MASTER_PLAN amendment, not now.

### 8.6 AES-128-GCM key provisioning — **single shared key in a build-time header**

Keep it simple:

- **One AES-128 key**, 16 bytes, shared by handheld + tractor + base.
- Stored in `firmware/common/lora_proto/key.h` — **gitignored**. A `key.h.example` with a placeholder is committed.
- Generated once at build time: `openssl rand -hex 16` (or any 32-hex-char string the operator picks). Pasted into `key.h` before first compile.
- **Rotation:** edit `key.h`, reflash all three devices, confirm link. No on-air key exchange, no key server, no per-device keys for v25.
- The key is a *defence against casual sniffing on a shared 915 MHz band*, not a defence against a determined attacker with the key. The real safety story is the hardware E-stop chain and the Opta watchdog, not the cipher.

The current `crypto_stub.c` is acceptable **only** when the build defines `LIFETRAC_ALLOW_STUB_CRYPTO` (sim/CI only). Production firmware must compile against `key.h` with real AES-GCM. Future hardening (per-device keys, secure-element storage, rotation procedure) lives in `RESEARCH-CONTROLLER/` until promoted.

### 8.7 Network topology — **1 base + 1 tractor + ≤1 handheld**

Exactly one tractor and one base per LoRa network. At most one handheld. Multi-tractor, multi-base, mesh, and roaming are explicitly out of v25 scope (move proposals to `RESEARCH-CONTROLLER/`).

### 8.8 Camera / video in v25 — **Kurokesu USB cameras canonical, other UVC cameras allowed**

The canonical camera is the **Kurokesu C2-290C** (boxed, CS-mount, USB UVC), per [`VIDEO_OPTIONS.md`](VIDEO_OPTIONS.md). Up to three units fit the documented camera plan: front (cab forward, varifocal), rear (reverse, fixed wide), and implement (boom/hitch, Kurokesu C1 PRO board variant). The tractor X8 hosts them on its USB ports; selection and thumbnail streaming over LoRa follow [`LORA_PROTOCOL.md`](LORA_PROTOCOL.md) (`CMD_CAMERA_SELECT`, topic IDs `0x20`/`0x21`/`0x22`/`0x23`).

**Other USB cameras are allowed** as long as they are:

- **UVC-class** (USB Video Class) — Linux on the X8 sees them as `/dev/videoN` without proprietary drivers.
- Capable of MJPEG or H.264 at the resolution/fps the operator selects (the LoRa thumbnail path is generated by re-encoding on the X8, so any UVC source works).
- Powered within the USB-host budget on the Max Carrier (5 V / shared bus). High-draw cameras may need a powered hub; that's fine, just call it out per install.

Non-canonical UVC cameras do **not** require a MASTER_PLAN amendment — they're a parts substitution, like swapping a brand of fuse. The Kurokesu units stay canonical because they're the ones documented in [`VIDEO_OPTIONS.md`](VIDEO_OPTIONS.md) with known good optics, enclosure, and mounting hardware for tractor use.

For the **first hardware bench test**, cameras are optional — the LED-stand-in bring-up in §5 step 4 doesn't need them. Add at least the front Kurokesu before the §5 step 7 hydraulic test so the operator has eyes on the manifold.

### 8.9 Compute platform — **Portenta X8 on both tractor and base**

The canonical build uses the **Portenta X8 on both nodes**. Committing to the X8 everywhere lets us assume Linux on both ends (Mosquitto, FastAPI, Python services, SSH, Docker). The X8 already contains the STM32H747 co-MCU silicon, so the M7+M4 realtime firmware in `firmware/tractor_h7/` runs on the X8's onboard co-MCU — the directory name is historical, not a separate board.

If the X8 is genuinely unavailable at procurement time, treat it as a §7 open decision and revisit before resuming work. Substituting a Raspberry Pi 5 / mini-PC for the *base* Linux host is acceptable only as an emergency workaround (the Max Carrier still owns the LoRa radio); the *tractor* must be the X8 because the onboard co-MCU runs the safety-critical control firmware.

### 8.10 Tractor X8 role — **Linux sidecar: sensors, logging, parameters, video**

The tractor X8 is a quad-core Cortex-A53 @ 1.8 GHz with 2 GB RAM running Yocto Linux + Docker. It is **not** in the realtime control loop — the H747 co-MCU's M7 (LoRa + arbitration + Modbus master) and M4 (100 Hz control) own all safety-critical work on bare metal. The X8 is a Linux sidecar.

**Hard rule that keeps the X8 off the control hot path:**

> The X8 ↔ H747 IPC is **non-blocking in the H747 direction**. If Linux is hung, busy, or rebooting, every H747 read of an X8-published value returns the last-known value with a stale-flag, and control continues. The H747 never waits on the X8 for anything.

Services running on the X8 (each in `firmware/tractor_x8/`):

#### Pinned for v25 (built before bench bring-up)

- `gps_service.py` — SparkFun Qwiic u-blox NEO-M9N over the **Adafruit FT232H USB→I²C/Qwiic adapter** → publishes pose to shared memory / IPC for the H747.
- `imu_service.py` — Adafruit BNO085 over the same FT232H adapter (Qwiic-daisy-chained off the GPS) → same IPC region.
- `logger_service.py` — telemetry / black-box recorder. Every `ControlFrame`, `TelemetryFrame`, source-handover, watchdog event, and E-stop latch is written to a local rotating SQLite (or InfluxDB) with monotonic timestamps. ~few MB/hour. Critical for bench bring-up debugging and post-incident analysis.
- `params_service.py` — single TOML/JSON file on disk for tunables (deadbands, max speeds, ramp rates, future geofence polygons, camera defaults). The H747 reads parameters at boot over IPC and re-reads at most 1 Hz when a `params_dirty` flag is set; parameter changes are never on the 100 Hz control path.
- `event_recorder.py` — when the H747 trips the Opta watchdog or latches an E-stop, the X8 records the last few seconds of `ControlFrame`s, RSSI history, source state, and Opta register snapshots. Without this, post-mortem on bench failures is guesswork.
- `time_service.py` — disciplines the X8 system clock from GPS PPS / NMEA, and exposes one boot-time write of UTC to the H747's RTC over IPC (§8.14).
- USB host for the Kurokesu / UVC cameras (§8.8) — raw `/dev/videoN` ownership; encoding is done by the video pipeline below.
- SSH for field debug (key-only, no password).

#### Reserved on the X8 (slot named, implementation deferred)

- `video_pipeline/` — encode the UVC camera streams once on the X8, fan out to: LoRa thumbnail re-encoder for the `0x20`/`0x21`/`0x22` topic IDs, local recording to onboard storage, future WebRTC stream to base. Video is **P3-class** in the LoRa QoS spec, queued behind control + heartbeats and dropped on congestion — the H747 never waits for the encoder.
- `fusion_service.py` — EKF / pose-graph combining GPS + IMU + (future) wheel/track odometry into a single pose estimate. The H747 consumes the result over IPC; fusion math runs on the X8.
- `health_service.py` — engine-temp trends, hydraulic-pressure histograms, valve duty-cycle accounting, predictive-maintenance hints. Not safety-critical.
- `log_shipper.py` — rate-limited push of black-box log slices to the base over LoRa (low-priority QoS class), or full dump over Ethernet when the tractor is in the garage on a wire.
- `diag_web.py` — optional FastAPI on the tractor X8, reachable over the dev-radio WiFi flag (§8.13), showing current state / recent frames / sensor readouts. Bench bring-up tool, gated by `LIFETRAC_ENABLE_DEV_RADIO`. Never enabled on a tractor leaving the bench.
- `ota_stage/` — future home for OTA staging when §8.16 is reversed; stages and verifies firmware images for the H747 and Opta.

#### Why USB→Qwiic and not the H747's native I²C

Keeps the realtime co-MCU off a sensor bus that can stall or hot-unplug, and lets the X8 own all hot-pluggable peripherals (cameras + sensors on the same Linux subsystem). Both Qwiic devices share one Adafruit FT232H — the GPS plugs into the adapter, the IMU plugs into the GPS's second Qwiic port. If a sensor fails or is unplugged, the realtime control loop on the H747 is unaffected.

#### What stays on the H747 (must not move to X8)

Anything that, if stalled for 200 ms, could allow a coil to energize unsafely. Specifically: the 100 Hz M4 control loop, the 50 Hz Modbus-RTU master to the Opta, the 50 ms `pick_active_source()` arbitration, the E-stop latch, and the LoRa frame TX/RX hot path including AES-GCM auth-tag verification. Linux scheduling jitter is multi-millisecond at best and hundreds of milliseconds worst case under load — unacceptable for any of these.

If the X8 is offline for any reason, control still works.

### 8.11 Base-station internet access — **LAN-only for v25, WAN exposure deferred**

The base station is a **LAN appliance for the first build**. No outbound internet is required for normal operation, and no inbound exposure is offered. Plain HTTP on port 80 (per §8.5) is only acceptable because of this constraint.

Future (out of v25 scope):

- Operator-controlled WAN exposure via VPN (WireGuard / Tailscale) — preferred path. Adds remote access without opening port 80 to the internet.
- Direct internet exposure with TLS + per-operator auth — requires the §8.5 "future" upgrades first.
- Optional outbound: NTP, OTA mirror pulls, NTRIP for future RTK.

No telemetry leaves the LAN unless the operator deliberately configures a forwarder.

### 8.12 Wire-contract rates — **promote to canonical here**

Restating from §4 / §5 so these are not buried in a bring-up step:

| Link | Direction | Rate |
|---|---|---|
| Handheld / base → tractor `ControlFrame` (LoRa) | uplink | 20 Hz |
| Active source `HeartbeatFrame` (LoRa) | uplink | 20 Hz |
| Tractor → base `TelemetryFrame` (LoRa) | downlink | 2 Hz nominal, 10 Hz burst on event |
| Max Carrier → Opta Modbus writes (RS-485) | tractor-internal | **50 Hz** |
| Opta → Max Carrier Modbus telemetry reads | tractor-internal | **10 Hz** |
| Tractor M4 control loop | tractor-internal | **100 Hz** |
| Opta watchdog timeout | tractor-internal | **200 ms** |

If any of these change, update `LORA_PROTOCOL.md`, `TRACTOR_NODE.md`, **and** this table.

### 8.13 Diagnostic radios on the Max Carriers — **WiFi/BT off in shipped firmware on the tractor; base may use WiFi or Ethernet for LAN**

The X8 module on the Max Carrier has its own WiFi/BT (Murata 1DX). Policy:

- **Tractor:** WiFi/BT **disabled** in shipped firmware (matches the Opta policy). May be enabled on a developer build for bench SSH/diagnostics, gated by a `LIFETRAC_ENABLE_DEV_RADIO` build flag. Never enabled on a tractor leaving the bench.
- **Base station:** **WiFi or Ethernet** to LAN, operator's choice. The base is indoors, not safety-critical, and just needs a way to be reached by the operator's browser. Either link is fine; the §8.5 LAN-only auth model applies the same way to both.

### 8.14 Time sync — **GPS-disciplined on tractor X8; nonce uniqueness independent of clock**

Tractor X8 disciplines its system clock from the SparkFun Qwiic u-blox NEO-M9N (PPS where the breakout exposes it, NMEA-derived otherwise) over the Adafruit FT232H USB→Qwiic adapter (§8.10). The X8's onboard H747 co-MCU and the base get time from the X8 over IPC / NTP respectively. The handheld has no RTC and does not need one. **AES-GCM nonce uniqueness does not depend on wall-clock time** — it uses the per-source monotonic sequence counter from `LORA_PROTOCOL.md`, persisted to flash on the handheld so it survives reboot. (This is one of the readiness blockers in §5 step 3.)

### 8.15 Field test gate — **≥500 m line-of-sight, <1% frame loss, 10 min**

§5 step 9 ("field test") passes when:

- Tractor at ≥500 m line-of-sight from the base mast antenna (antenna at ≥3 m AGL).
- `ControlFrame` loss <1% over a 10-minute continuous run, measured at the tractor.
- `TelemetryFrame` loss <5% over the same window.
- E-stop from base reaches tractor in <500 ms p99, measured by oscilloscope on a dummy coil.
- No spurious E-stop, no spurious source-handover.

Failing any of these reverts to the bench and is logged as a regression. Range targets escalate (1 km, 2 km) only after the 500 m gate is clean.

### 8.16 Firmware updates — **USB-C flashing only for v25**

All firmware (handheld MKR WAN, tractor X8 onboard H747 co-MCU sketch, Opta + expansions) is flashed over **USB-C with the device on the bench**. No OTA, no network flashing, no update-by-LoRa. Linux services on the X8 update via standard `apt` / `pip` / container-image pulls when the base or tractor is on a trusted network; this is operator-initiated, not automatic. OTA firmware update is explicitly out of v25 scope.

### 8.17 LoRa control-link PHY — **SF7 / BW 125 kHz / CR 4-5 default, no retries on `ControlFrame`, adaptive SF7→SF8→SF9 fallback when SNR margin degrades**

The `ControlFrame` and active-source `HeartbeatFrame` are P0-class and latency-critical. Pin the air parameters and retry policy here so the latency budget holds:

- **Default PHY for control + heartbeat:** SF7 / BW 125 kHz / CR 4/5. ~30 ms air time for the 16-byte ControlFrame after AES-GCM + KISS overhead. (Earlier drafts of `LORA_PROTOCOL.md` showed BW 500 kHz; we pin 125 kHz here because Murata SiP / SX1276 + 8 dBi mast antenna get more usable range at 125 kHz with a tolerable air-time penalty, and 125 kHz is the channel that matters for cross-vendor compatibility on the US 915 MHz band.)
- **No retries on `ControlFrame`.** The MAC layer's `transmit()` retry loop is **disabled for P0 ControlFrame**. We only ever care about the *newest* control snapshot — the next frame is in 50 ms (20 Hz cadence) and any older frame is stale by definition. Retrying a stale stick position can only hurt: it consumes channel time the next-newer frame needs, and it can briefly re-apply a stick deflection the operator already released. CSMA's channel-busy backoff still applies (one polite check, no second attempt), but if the channel is busy we drop the frame and rely on the next 20 Hz tick. `TelemetryFrame` retains its existing 3-attempt CSMA behavior; only P0 control changes.
- **Adaptive SF fallback ladder (SF7 → SF8 → SF9).** The tractor (the canonical RX for control) tracks per-source SNR margin on the last N=10 received frames. When the rolling SNR margin drops below a configurable threshold (default +3 dB above demod floor for the current SF) for >2 s **or** when ControlFrame loss exceeds 2% over a 5 s window, the tractor sends a `CMD_LINK_TUNE` Command frame announcing the next ladder rung (SF8, then SF9). The frame is transmitted at *both* the current SF and the new SF (back-to-back) for one cycle, then both ends switch. Reverse direction (SF↑ → SF↓ when conditions improve) requires the link to be clean for 30 s before stepping back down, to avoid hunting. The SF in use is reported in TelemetryFrame topic `0x10` (`control/source_active`) so the web UI can surface it.
- **Air-time impact at each rung** (for the 44-byte on-air ControlFrame including AES-GCM + KISS overhead, approximate): SF7 ~30 ms → SF8 ~55 ms → SF9 ~100 ms. At SF9 we are at the edge of fitting 20 Hz cadence into the duty budget; this is acceptable as a degraded-link mode, not steady state.
- **Failsafe behavior is unchanged.** If the tractor sees no valid control frame at any SF for `HEARTBEAT_TIMEOUT_MS` (500 ms), `pick_active_source()` drops to `SOURCE_NONE` and valves go neutral per § ControlFrame in `LORA_PROTOCOL.md`.
- **Interaction with telemetry SF.** Telemetry stays on its own SF (currently SF9 / BW 250 kHz / CR 4/8 per `LORA_PROTOCOL.md`); the adaptive-SF logic above is for the control + heartbeat slot only.

Authoritative wire-format and `CMD_LINK_TUNE` opcode definition live in [`LORA_PROTOCOL.md`](LORA_PROTOCOL.md). This pin establishes the policy; that file establishes the bytes.

### 8.18 Valve coil drive routing — **directional-valve coils ride the D1608S SSR channels for low-latency switching**

The Opta Ext **D1608S** (AFX00006) carries **8× solid-state relays rated 24 VDC / 2 A** — not electromechanical relays. The EMR variant is the D1608**E** (AFX00005); we are buying the SSR variant on purpose. Routing policy:

- **Directional valve coils (8×) ride D1608S SSR channels.** These are the latency-critical loads on the control hot path. SSR switching is sub-millisecond with no contact bounce, removing the ~8–20 ms mechanical-relay latency term identified in the [`LATENCY_BUDGET.md`](RESEARCH-CONTROLLER/LATENCY_BUDGET.md) analysis. 24 VDC / 2 A comfortably covers typical 24 V hydraulic solenoid coils (≈1.5 A pull, <0.5 A hold).
- **Engine-kill and other slow auxiliary loads** ride the Opta base's onboard EMR channels. These are gated by the PSR safety relay anyway, are not on the 50 Hz control loop, and benefit from the EMR's higher current rating (10 A) and galvanic isolation. Latency on the engine-kill chain is dominated by the PSR de-energization, not the relay coil pickup.
- **Implication for the wiring diagram in [`TRACTOR_NODE.md`](TRACTOR_NODE.md):** the boom-up / boom-down / bucket-curl / bucket-dump / drive-LH-fwd / drive-LH-rev / drive-RH-fwd / drive-RH-rev coils all land on D1608S SSR outputs. The four onboard Opta EMRs are reserved for engine-kill, beacon/horn relay, parking-brake release, and one spare. (Earlier drafts had four directional coils on the onboard EMRs; that was based on the mistaken assumption that the D1608S was electromechanical. Update the harness diagram before bench bring-up.)
- **Coil flyback / surge:** SSRs handle inductive DC switching gracefully when paired with a flyback diode across the coil. Continue to spec a 1N4007-class diode across each solenoid coil per industrial practice; not optional.
- **What we did NOT do:** we did not specify external SSR modules in the harness. The D1608S provides what an external module would have provided, with one less line item and one less terminal block. If a future revision needs faster pickup than the D1608S can deliver (e.g. PWM coil drive at >100 Hz for proportional valves), revisit.

---

If none of the §7 reversals happen and the §8 confirmations land, this plan is the build.
