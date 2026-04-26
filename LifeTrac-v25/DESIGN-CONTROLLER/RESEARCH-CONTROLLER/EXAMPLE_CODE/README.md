# EXAMPLE_CODE — draft prototype sketches for review

**Status:** drafts for **study and review only**. None of this code has been compiled, flashed, or tested. The point of the folder is to make the design concrete enough that we can read it end-to-end and pick which version we actually want to build for the first hardware test.

> **2026-04-26 review pass applied** — the thirteen findings from the
> [pipeline code review](../../AI%20NOTES/CODE%20REVIEWS/2026-04-26_v25_controller_pipeline_example_code_review.md)
> were verified and the fixes are documented in
> [CODE_REVIEW_FIXES.md](CODE_REVIEW_FIXES.md). The Modbus register map now
> matches [`TRACTOR_NODE.md`](../../TRACTOR_NODE.md) and the
> ControlFrame size/layout matches [`LORA_PROTOCOL.md`](../../LORA_PROTOCOL.md).

The files here cover the **minimum viable chain** of software that the v25 controller needs:

```
┌────────────────┐  LoRa custom proto   ┌────────────────────────┐  Modbus RTU   ┌──────────────┐
│   Handheld     │ ───────────────────► │   Tractor (Max Carrier │ ────────────► │   Opta WiFi  │
│   MKR WAN 1310 │                      │   + Portenta X8)       │   RS-485      │  + D1608S    │
│                │                      │                        │  115200 8N1   │  + A0602     │
│  joystick.ino  │                      │  tractor_m7.ino        │               │              │
└────────────────┘                      │  tractor_m4.cpp        │               │  opta_modbus │
                                        │  (radio + arbitration) │               │  _slave.ino  │
┌────────────────┐                      │                        │               │              │
│  Base Station  │                      │                        │               │   drives:    │
│  Portenta X8   │ ◄──── LoRa ────────► │                        │               │   8× valves  │
│  (Linux)       │                      │                        │               │   2× 0–10V   │
│                │                      └────────────────────────┘               └──────────────┘
│ • lora_bridge  │
│   (Python)     │
│ • web_ui       │
│   (FastAPI)    │
│ • web/index    │
│   (HTML+JS,    │
│   touch + USB  │
│   Gamepad API) │
└────────────────┘
```

Out of scope for this folder (deliberately):

- Cellular failover, ROS 2, DroidPad, multi-tractor fleet.
- QR-code pairing (currently the key is provisioned over USB — see [LORA_PROTOCOL.md § Key management](../../LORA_PROTOCOL.md#key-management)).
- Autonomy / path planning.
- Any of the [VIDEO_COMPRESSION/](../VIDEO_COMPRESSION/) phases beyond the simplest periodic JPEG thumbnail.
- Other wireless protocols (XBee, BLE, WiFi mesh).

Those are tracked as enhancements at the bottom of this README.

---

## Folder layout

```
EXAMPLE_CODE/
├── README.md                       (this file)
├── lora_proto/                     shared C header + framing (study target #1)
│   ├── lora_proto.h
│   ├── lora_proto.c
│   └── crypto_stub.c
├── handheld_mkr/                   MKR WAN 1310 sketch
│   └── handheld.ino
├── tractor_h7/                     Portenta H747 (in X8 or H7) sketch
│   ├── tractor_m7.ino              radio + arbitration + Modbus master
│   └── tractor_m4.cpp              valve safety on M4 core (sketch only)
├── tractor_x8/                     Tractor X8 Linux side (Portenta X8 only)
│   ├── imu_service.py              BNO086 over Qwiic/MCP2221A, 5 Hz → M7 UART
│   ├── gps_service.py              NEO-M9N over same Qwiic bus, 1 Hz → M7 UART
│   └── requirements.txt
├── opta_modbus_slave/              Arduino Opta sketch
│   └── opta_modbus_slave.ino       Modbus RTU slave + relay/AO drive
└── base_station/                   X8 Linux side
    ├── lora_bridge.py              serial (UART → H747 → SX1276) ↔ MQTT/WebSocket
    ├── web_ui.py                   FastAPI app with /ws control channel
    ├── requirements.txt
    └── web/
        ├── index.html              touch joysticks + USB gamepad
        └── app.js                  Gamepad API + WebSocket client
```

Every file is **commented heavily** so it reads as documentation. They are intentionally short, share types via [lora_proto/lora_proto.h](lora_proto/lora_proto.h), and avoid features we haven't decided on yet.

---

## What each file is for, and what to look at when reviewing

### 1. [`lora_proto/`](lora_proto/) — the wire format, in code

Mirrors the ControlFrame / Heartbeat / TelemetryFrame definitions in [LORA_PROTOCOL.md § Frame format](../../LORA_PROTOCOL.md#frame-format). This is the *one* file that handheld, tractor, and base station all include — if it changes, everyone changes together.

**Review questions for this file:**
- Is the 16-byte ControlFrame really enough for everything we want a joystick to do? (axes + 16 buttons + flags + heartbeat counter currently)
- Should `crypto_stub.c` be a thin wrapper over MbedTLS GCM (Portenta) or ArduinoBearSSL (MKR) — or do we want our own AES-GCM table? Currently it is a *stub* that no-ops — encryption is left as a TODO inside the function.
- KISS framing or COBS? KISS is simpler and matches the [LORA_PROTOCOL.md](../../LORA_PROTOCOL.md) spec; COBS is faster to scan. Currently KISS.

### 2. [`handheld_mkr/handheld.ino`](handheld_mkr/handheld.ino) — joystick → LoRa frame at 20 Hz

Simplest of the bunch. Reads two analog joysticks + 8 buttons + a TAKE CONTROL momentary, packs a ControlFrame, hands it to `lora_proto_send()`, and emits a Heartbeat at the same cadence.

**Review questions:**
- Are 20 Hz control frames the right cadence? (Air time for 16 B at SF7/BW500 ≈ 15 ms, so 20 Hz uses ~30% of the channel — leaves headroom for base + telemetry.)
- Take-control latch state: we have it on the *handheld* side per [LORA_PROTOCOL.md § Take-control sequence](../../LORA_PROTOCOL.md#take-control-sequence-handheld). Is that the right place? (Trade-off: simpler tractor logic vs. operator's intent stored on a single device they could lose.)

### 3. [`tractor_h7/tractor_m7.ino`](tractor_h7/tractor_m7.ino) — the brain

Runs on the M7 core of the H747 (whether that H747 lives in an X8 or a standalone H7). Receives LoRa frames from both handheld and base, runs the arbitration loop from [LORA_PROTOCOL.md § Multi-source arbitration](../../LORA_PROTOCOL.md#multi-source-arbitration), and writes the chosen control set to the Opta over Modbus RTU. Also sends telemetry frames the other way.

**Review questions:**
- We use `ArduinoModbus` (which only supports MODBUS RTU master mode out of the box) on top of `ArduinoRS485`. Do we want that, or roll a tiny Modbus master in-line so we can fully control timing? Currently using the official library.
- The arbitration loop here runs on the M7. The valve safety watchdog runs on the M4 ([`tractor_m4.cpp`](tractor_h7/tractor_m4.cpp)). Is that split right, given that the *Opta* also has its own watchdog on the Modbus alive register?
- The telemetry path is currently a single 1 Hz loop. Real spec wants per-topic cadences (5 Hz GPS, 1 Hz engine, etc.). For the first test, single-loop is good enough to study.

### 4. [`tractor_h7/tractor_m4.cpp`](tractor_h7/tractor_m4.cpp) — independent valve safety

Tiny M4 sketch that reads a shared-memory "alive" tick from the M7, and pulls a GPIO low if it stops ticking for >200 ms. That GPIO feeds into the PSR safety relay path described in [TRACTOR_NODE.md § Wiring overview](../../TRACTOR_NODE.md#wiring-overview).

**Review question:** is M4 the right place for this, or do we put it entirely on the Opta and let the M4 idle? Currently M4 *and* Opta both have independent watchdogs — belt and suspenders.

### 4b. [`tractor_x8/imu_service.py`](tractor_x8/imu_service.py) + [`tractor_x8/gps_service.py`](tractor_x8/gps_service.py) — sensors on the X8 Linux side

Two sibling services that share one Qwiic bus through one Adafruit MCP2221A USB→I²C bridge (Adafruit 4471, see [`HARDWARE_BOM.md`](../../HARDWARE_BOM.md) Tier 1). The bridge driver `hid-mcp2221` has been mainline since Linux 5.10, so the X8 Yocto image enumerates `/dev/i2c-N` with no install. Both services KISS-frame a `<topic_id:1><payload:N>` packet over the X8↔H747 UART (`/dev/ttymxc0`) at 921600 8N1; the M7's `poll_x8_uart()` reads the framed bytes, peels off the topic byte, and re-emits the rest as a standard `TelemetryFrame` on the LoRa link. The bridge then maps it to MQTT via `TOPIC_BY_ID` in [`base_station/lora_bridge.py`](base_station/lora_bridge.py).

| Service | Sensor | Topic | Rate | Payload |
|---|---|---|---|---|
| `imu_service.py` | SparkFun BNO086 Qwiic, I²C @ 0x4A/0x4B | `0x07` → `lifetrac/v25/telemetry/imu` | 5 Hz | 18 B (Q14 quaternion + accel mg + heading) |
| `gps_service.py` | SparkFun NEO-M9N SMA Qwiic, I²C @ 0x42 | `0x01` → `lifetrac/v25/telemetry/gps` | 1 Hz active / 0.2 Hz parked | 21 B (lat/lon/alt e7+cm, speed cm/s, heading×10, fix/sats/HDOP) |

**Why 1 Hz is enough for GPS.** Tractor top speed is ~5 mph (~2.2 m/s), so 1 Hz misses at most ~2.2 m of travel — inside the NEO-M9N's ~1.5 m standalone accuracy. Bumping to 5–10 Hz costs LoRa airtime and doesn't make a tracked tractor visibly smoother on the base map. `gps_service.py` also auto-backs off to 0.2 Hz after 10 s of <0.1 m/s ground speed (parked), freeing the channel for hydraulics / video / IMU.

**Both services run on the tractor X8 only.** Skip this folder if you are flashing a standalone H7 — in that fallback build the IMU and GPS have to move to native I²C wired to the Max Carrier breakout, polled by the M7. The tradeoffs are documented in [`HARDWARE_BOM.md` § Notes on substitutions](../../HARDWARE_BOM.md#notes-on-substitutions).

**Review questions:**
- Is 1 Hz GPS enough, or does the autonomy / waypoint follower want ~5 Hz once that lands? (Cheap to bump — just `--rate-hz 5` on the service.)
- 21 B fixed-point vs raw NMEA: fixed-point keeps each frame inside one LoRa packet at SF7. NMEA strings would need fragmentation. Worth keeping for now.
- Idle backoff: the 10 s / 0.1 m/s threshold is arbitrary. Should we tie it to the Opta `valve_coils == 0` AND `engine_run == false` instead so it backs off only when *actually* parked, not idling at headlands?

### 5. [`opta_modbus_slave/opta_modbus_slave.ino`](opta_modbus_slave/opta_modbus_slave.ino) — the I/O layer

Modbus-RTU slave on RS-485, holds a register map exposing:

| Register | RW | Meaning |
|---|---|---|
| `0x0000` `alive_tick` | W | Master writes a monotonically increasing value at ≥10 Hz; Opta drops all coils if it stops |
| `0x0001` `flow_setpoint_1` (0..10000) | W | Drives A0602 AO1 → Burkert flow valve #1 |
| `0x0002` `flow_setpoint_2` (0..10000) | W | Drives A0602 AO2 → Burkert flow valve #2 |
| `0x0010..0x0017` `valve_command[8]` | W | Bit 0 = energize coil; one register per directional valve |
| `0x0020..0x0027` `valve_current[8]` | R | Per-coil current readback for fault detection |
| `0x0030` `mode_switch` | R | 0 = handheld, 1 = base, 2 = autonomy lockout |
| `0x0031` `estop_loop` | R | 1 = E-stop chain healthy; 0 = open |
| `0x0040..0x0045` `analog_in[6]` | R | Battery V, hyd P supply, hyd P return, oil T, eng T, spare |
| `0x00FF` `last_error` | R | Bitmap of latched faults |

Map and behaviour deliberately match [TODO.md § Phase 1](../../TODO.md#phase-1--bench-bring-up-of-tractor-node-no-radio-yet) so we can move it to firmware without renaming registers.

**Review questions:**
- Register map: is this small enough for the first test? We can always add. Is anything *missing* that we'll regret later (e.g. a per-coil PWM duty register if we ever want PWM-modulated coils)?
- Should `flow_setpoint` be a single register or two (target + slew rate)? Currently single.
- `mode_switch` here is a *physical* 3-position switch read from a digital input — this is not the same as the LoRa source-arbitration; the physical switch is a hard lockout. Is that the model we want?

### 6. [`base_station/lora_bridge.py`](base_station/lora_bridge.py) — UART ↔ MQTT/WebSocket

Runs as a Docker container on the X8 Linux side. Talks to the H747 co-MCU over UART (which in turn talks to the SX1276 SiP), unpacks frames, publishes to Mosquitto on `lifetrac/v25/…` topics, and forwards control frames the other direction.

**Review questions:**
- Do we need MQTT for the local case at all, or is FastAPI's in-process pub/sub enough? MQTT helps when external clients (Grafana, ROS 2) want to subscribe; otherwise it is overhead. Currently included.
- Serial framing on the UART link: same KISS framing as the air? Currently yes — keeps one framer in the codebase.

### 7. [`base_station/web_ui.py`](base_station/web_ui.py) + [`base_station/web/`](base_station/web/) — operator UI

FastAPI app that serves a single HTML page with:
- Two on-screen virtual joysticks (touch + mouse).
- Live USB gamepad via the browser's [Gamepad API](https://developer.mozilla.org/en-US/docs/Web/API/Gamepad_API) — when a controller is connected, its sticks/buttons preempt the on-screen ones.
- WebSocket back to the server pushing `ControlFrame` updates at 20 Hz.
- Telemetry sidebar updated from a second WebSocket subscribed to `lifetrac/v25/telemetry/+`.
- A "REQUEST CONTROL" button that drives the `take_control_until` latch.
- A big red E-STOP button that bypasses arbitration (sends a special command frame).

The USB gamepad is a *huge* quality-of-life win and adds essentially zero hardware: any $20 game controller works in-browser, no drivers, no native code. We default to "first stick X/Y → drive, second stick X/Y → boom/aux, A=curl, B=dump, X=aux1, Y=aux2, LB=request control, RB=take-control-hold, START=E-stop" — same mapping as the handheld, so muscle memory transfers.

**Review questions:**
- Layout: matches [BASE_STATION.md § Live operator console](../../BASE_STATION.md#live-operator-console-) — agree?
- Should the E-stop go over both LoRa *and* a separate authenticated channel (cellular, Ethernet) for redundancy? Currently LoRa-only; cellular is a TODO once we have a working LoRa link to compare to.
- Browser → server WebSocket frame shape mirrors the on-the-air ControlFrame to keep one schema. Good or over-coupled?

---

## How to read these files in order

If you only have an hour, read in this order:

1. [`lora_proto/lora_proto.h`](lora_proto/lora_proto.h) — 5 minutes. This is the single source of truth for the wire format.
2. [`handheld_mkr/handheld.ino`](handheld_mkr/handheld.ino) — 5 minutes. The simplest LoRa producer.
3. [`tractor_h7/tractor_m7.ino`](tractor_h7/tractor_m7.ino) — 15 minutes. The interesting one — arbitration + bridge from radio to Modbus.
4. [`opta_modbus_slave/opta_modbus_slave.ino`](opta_modbus_slave/opta_modbus_slave.ino) — 10 minutes. What actually moves the metal.
5. [`base_station/lora_bridge.py`](base_station/lora_bridge.py) and [`base_station/web/app.js`](base_station/web/app.js) — 15 minutes. Server side and gamepad input.
6. [`base_station/web_ui.py`](base_station/web_ui.py) and [`base_station/web/index.html`](base_station/web/index.html) — 10 minutes. The operator console.

---

## Build / run notes (planning only — nothing to actually run yet)

These are the steps we would take *once we decide which version is the first test*. None of this is required to study the code.

- **Arduino side** (handheld, tractor, opta): Arduino IDE 2.x or PlatformIO. Boards: *Arduino MKR WAN 1310*, *Arduino Portenta H7 / Portenta X8 (M7+M4 split)*, *Arduino Opta WiFi*. Libraries listed in [arduino_libraries.txt](../../arduino_libraries.txt).
- **Base station Python** (X8 Linux): `pip install -r base_station/requirements.txt` → `uvicorn web_ui:app --host 0.0.0.0 --port 8080` and `python lora_bridge.py /dev/ttymxc0`.
- **HIL bench** before any radio: short the LoRa packet path with a UART loopback between handheld and tractor, swap LEDs in for valve coils on the Opta. Same code, no RF.

---

## Enhancements list (deferred)

To keep this folder focused, the following were *intentionally left out*. Any of them could become its own subfolder later:

- **Other wireless protocols:** XBee 900HP fallback link, ESP-NOW for proximity, Wi-Fi long-range mesh, Ubiquiti airMAX P2P. See [WIRELESS_OPTIONS.md](../../WIRELESS_OPTIONS.md).
- **QR-code or BLE pairing** for over-the-air key provisioning. Today, keys are flashed over USB.
- **Cellular failover** (SARA-R412M Cat-M1 already on the Max Carrier). Wire the same `ControlFrame` over MQTT-over-LTE.
- **DroidPad / mobile-app** as a fourth control source — see [DROIDPAD_INTEGRATION.md](../DROIDPAD_INTEGRATION.md).
- **ROS 2 bridge** — see [ros2_bridge/](../ros2_bridge/).
- **Autonomy & path planning:** GPS waypoint following, headland-turn planner, ag-row alignment.
- **Video compression beyond a periodic JPEG thumbnail:** see [VIDEO_COMPRESSION/README.md](../VIDEO_COMPRESSION/README.md) for the four-phase plan.
- **Multi-tractor base station:** one base, several tractors with unique source IDs.
- **Field-replaceable secrets store** (ATECC608B on each node) instead of pre-shared key in MCU flash.
- **Replay protection persistent across power cycles** — non-volatile sequence counter.
- **Forward error correction on telemetry** (Reed-Solomon (255,223)) per [LORA_PROTOCOL.md § FEC](../../LORA_PROTOCOL.md#forward-error-correction-telemetry-only).
- **Recorded-input replay tool** for regression-testing arbitration logic.
- **TLS termination + client cert auth** on the base-station web UI for off-LAN access.

---

## See also

- [../../ARCHITECTURE.md](../../ARCHITECTURE.md) — system design
- [../../LORA_PROTOCOL.md](../../LORA_PROTOCOL.md) — wire format spec these files implement
- [../../TRACTOR_NODE.md](../../TRACTOR_NODE.md) — tractor wiring & rationale
- [../../BASE_STATION.md](../../BASE_STATION.md) — base-station UI design
- [../../HANDHELD_REMOTE.md](../../HANDHELD_REMOTE.md) — handheld hardware
- [../../TODO.md](../../TODO.md) — phase plan; this folder maps to Phase 1 + Phase 2
- [../VIDEO_COMPRESSION/README.md](../VIDEO_COMPRESSION/README.md) — image-feedback options
