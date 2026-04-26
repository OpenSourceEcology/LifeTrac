# LifeTrac v25 Controller — Development TODO

Comprehensive task list for implementing the three-tier controller architecture (Portenta Max Carrier on tractor + Portenta Max Carrier with X8 base station + MKR WAN 1310 handheld). See [ARCHITECTURE.md](ARCHITECTURE.md) for the design.

Tasks are organized by phase. Hardware purchases come first because lead times dominate the schedule.

---

## Phase 0 — Hardware procurement & shop setup

### Tractor node hardware

- [ ] Order Arduino Portenta Max Carrier (ABX00043)
- [ ] **Order Arduino Portenta X8 (ABX00049)** — primary; same SKU as base station. **Fallback: Portenta H7 (ABX00042)** if X8 unavailable, $85 cheaper, loses Linux/SSH/MIPI camera but keeps full radio + real-time capability (binary-compatible firmware)
- [ ] Order LoRa SMA whip antenna (915 MHz, 3 dBi) — ANT-8/9-IPW1-SMA or equivalent
- [ ] Order cellular SMA antenna (700–2700 MHz)
- [ ] Order **SparkFun NEO-M9N Qwiic GPS — SMA variant (GPS-15733, ~$75)** + panel-mount SMA bulkhead (~$5) + a second 50 mm Qwiic cable (~$1). Daisy-chains off the BNO086 on the same MCP2221A bus, so no extra USB port or driver work. Onboard SMA jack means the cab-roof antenna feedline runs straight to the enclosure wall — no U.FL pigtail. See [HARDWARE_BOM.md Tier 1](HARDWARE_BOM.md#tier-1--tractor-node-portenta-max-carrier--portenta-x8--arduino-opta).
- [ ] Order cab-roof antenna mounting bracket + N-bulkhead pass-through
- [ ] Activate Cat-M1 IoT SIM card (Hologram, Soracom, or equivalent)
- [ ] Order industrial microSD 32 GB
- [ ] Order EMI filter / ferrite beads for 12 V input
- [ ] Order master battery cutoff switch (Blue Sea or equivalent)
- [ ] Order ATC blade fuse holder kit + assorted fuses (5/10/20 A)
- [ ] Order 18650 LiPo cell (3500 mAh, protected) + TP4056 charger module
- [ ] **Order Arduino Opta WiFi (AFX00002)** — industrial PLC, runs Modbus slave for valve I/O
- [ ] **Order Opta Ext D1608S** expansion (8× relay outputs + 8× digital inputs)
- [ ] **Order Opta Ext A0602** expansion (6× analog inputs + 2× 0–10 V analog outputs for Burkert 8605)
- [ ] Order DIN rail 35 mm × 300 mm + end stops for in-enclosure mounting
- [ ] Order RS-485 cable + 120 Ω termination resistors (×2) for Max Carrier J6 ↔ Opta link
- [ ] Order **engine-kill automotive relay** (30 A SPDT, e.g. Bosch 0332019150) — wired through an Opta D1608S relay channel
- [ ] Order **Phoenix Contact PSR safety relay** (PSR-MC38 or equivalent dual-channel monitored)
- [ ] Order signal-conditioning components for the analog inputs already covered by Opta A0602 (only needed if external transducers require scaling beyond A0602's 0–10 V / 4–20 mA ranges)
- [ ] Order status LED panel (3× 12 V panel-mount: POWER / LINK / FAULT)
- [ ] Order 12 V piezo buzzer (panel-mount)
- [ ] Order Deutsch DT connector kit + crimp tool (rent or buy)
- [ ] Order IP65 enclosure ~250×200×100 mm + rubber-bobbin vibration mounts
- [ ] Order conformal coating spray (MG Chemicals 419 acrylic or equivalent)
- [ ] Order cable glands (M16/M20 IP68, ×6)

### Tractor-side sensors and cameras

See the sensor/cabling discussion in [HARDWARE_BOM.md § Notes on substitutions](HARDWARE_BOM.md#notes-on-substitutions) and [TRACTOR_NODE.md § Telemetry sources](TRACTOR_NODE.md). USB UVC is the easy path — the Linux mainline `uvcvideo` driver enumerates any UVC webcam as `/dev/video0` with no custom driver work; MIPI CSI is faster/lower-latency but requires Yocto device-tree overlays so it lands in Phase 2 once the USB pipeline is proven.

- [ ] Order **USB UVC webcam** for first-light video — Logitech C920 (onboard H.264 hw-encode, ~$70) or ELP-USBFHD06H (M12, IP67, ~$90 if you need a sealed connector). UVC = no driver install on the X8 Yocto image.
- [ ] Order **panel-mount USB-A bulkhead** + cable gland to bring the webcam cable through the IP65 enclosure wall (consumer USB cables are not field-sealed).
- [ ] Order **Adafruit MCP2221A breakout (4471, ~$8)** — USB ↔ I²C bridge with Stemma QT/Qwiic. Mainline `hid-mcp2221` driver, no install on the X8 Yocto image. Hosts the IMU on the **Linux side**, not the M7.
- [ ] Order **SparkFun Qwiic 9DoF IMU — BNO086 (DEV-22857, ~$30)** with on-chip sensor fusion (quaternion straight out, no Madgwick/Mahony work). Used for tip-over warning, heading hold, vibration logging.
- [ ] Order **Qwiic / Stemma QT 50 mm cable** for MCP2221A → BNO086 link.
- [ ] *(Production-build hardening)* Order **USB galvanic isolator** (ADuM3160-based, ~$25) for inline use between the X8 USB port and the MCP2221A in the noisy tractor enclosure. Optional for bench bring-up.
- [ ] Order **2× hydraulic pressure sensors**, 0–250 bar 4–20 mA output with M12-A 4-pin cordsets (~$40 each). Wires into Opta A0602 analog inputs (`hyd_supply_psi` 0x0103 / `hyd_return_psi` 0x0104 per [TRACTOR_NODE.md Modbus map](TRACTOR_NODE.md#modbus-rtu-register-map-max-carrier--opta)).
- [ ] *(Phase 2, optional)* Order **MIPI CSI camera** — Arducam IMX219/IMX477 with 22-pin FFC ribbon (~$30–60). Requires building a Yocto image with the camera device-tree overlay enabled; track separately in `RESEARCH-CONTROLLER/VIDEO_COMPRESSION/`.
- [ ] *(Phase 2, optional)* Order **PoE IP camera** (Reolink RLC-510A or equivalent) if night vision / IR / single-cable PoE is wanted — pulled by FFmpeg/GStreamer from RTSP on the X8.

### Industrial cabling and connectors

- [ ] Order **Deutsch DT04-5P connector kit** + crimp tool for engine CAN harness (already covered above for the generic DT kit; verify 5-pin J1939 variant is in the kit).
- [ ] Order **M12-A 5-pin cordsets** (TURCK or Phoenix Contact, preassembled, 2 m) for RS-485 master ↔ Opta — preferred over field-terminated cable. Quantity 2.
- [ ] Order **M12-A 4-pin cordsets** for the 2× hydraulic pressure sensors (matching the sensor side).
- [ ] Order **M12 X-coded → RJ45 field plug** + Cat6a cable for IP camera or dev laptop Ethernet drop into the enclosure.
- [ ] Order **Hammond 1554/1555 polycarbonate IP66 enclosure** (or upgrade the existing IP65 spec) sized to fit Max Carrier + Opta + D1608S + A0602 + IMU breakout on a single DIN rail.
- [ ] Order **inline 5 A automotive blade fuse holder + TVS diode** (e.g. SMBJ33A) for the 12/24 V power feed — load-dump protection.

### Base station hardware

- [ ] Order second Arduino Portenta Max Carrier (ABX00043)
- [ ] Order Arduino Portenta X8 (ABX00049)
- [ ] Order 8 dBi 915 MHz omni mast antenna (L-com or equivalent)
- [ ] Order LMR-400 coax cable (3 m, SMA-M to N-M)
- [ ] Order N-female bulkhead lightning arrestor
- [ ] Order ~3 m galvanized mast + ground rod
- [ ] Order second cellular SMA antenna + activate second Cat-M1 SIM
- [ ] Order indoor 12 V / 5 A power supply
- [ ] Order mini UPS (12 V, 30 min runtime)
- [ ] Order indoor ventilated enclosure
- [ ] Order Cat6 Ethernet cable (5 m)

### Handheld hardware

- [ ] Order Arduino MKR WAN 1310 (ABX00029)
- [ ] Order 1/4-wave 915 MHz whip antenna with u.FL or pigtail
- [ ] Order dual-axis analog joysticks (×2)
- [ ] Order 22 mm latching mushroom E-stop
- [ ] Order momentary push buttons (×4, IP67 tactile)
- [ ] Order SSD1306 OLED 128×64 I²C
- [ ] Order 1S 2000 mAh LiPo battery
- [ ] Order USB-C panel-mount breakout
- [ ] Order IP54 handheld enclosure (Hammond/Polycase ~120×80×40)
- [ ] Design custom PCB for joystick interface to MKR (KiCad)
- [ ] Order PCB from OSHPark or JLCPCB (5-pack)

### Development gear

- [ ] Order RTL-SDR USB receiver (NooElec) for LoRa packet sniffing
- [ ] Order Saleae Logic 8 (or clone) for latency measurement
- [ ] Identify spectrum analyzer rental/borrow source for FCC EIRP verification
- [ ] Order bench power supply (0–30 V, 0–5 A) if not already in shop
- [ ] Order 50 Ω SMA dummy loads (×2, 5 W rated)

### Spare parts (highly recommended)

- [ ] Order ×2 of each: Max Carrier, MKR WAN 1310, Portenta X8 (+ optional 1× Portenta H7 as documented fallback)
- [ ] Order ×2 spare LoRa antennas, cellular antennas, joysticks, OLEDs

---

## Phase 1 — Bench bring-up

- [ ] Set up shared Git repo with subfolders for `firmware/handheld_mkr`, `firmware/tractor_h7` (runs on X8 co-MCU or standalone H7), `firmware/tractor_opta`, `firmware/base_h7` (runs on X8 co-MCU), `firmware/common`, `base_station/` (Docker compose root)
- [ ] Set up PlatformIO with Portenta H7 + Portenta X8 + MKR WAN 1310 + Opta board support
- [ ] Set up Arduino-CLI CI (extend existing [ARDUINO_CI.md](ARDUINO_CI.md) setup)
- [ ] **Tractor:** verify Max Carrier + H7 boots, M7 blink works, M4 blink works
- [ ] **Tractor:** verify LoRa TX/RX from M7 with RadioLib at 915 MHz, SF7, BW500
- [ ] **Tractor:** verify cellular SARA-R412M registers and sends a test MQTT publish
- [ ] **Tractor:** wire DRV8908, drive 8 LEDs as valve stand-ins from M4 core
- [ ] **Base:** verify Max Carrier + X8 boots, get SSH access
- [ ] **Base:** install Docker on X8 Yocto image
- [ ] **Base:** verify M7 LoRa TX/RX (separate from tractor's M7)
- [ ] **Base:** verify Gigabit Ethernet works, get DHCP lease on office LAN
- [ ] **Handheld:** flash MKR WAN 1310 with `RadioLib` "hello world" sketch, verify LoRa TX/RX
- [ ] **Handheld:** verify joystick analog reads + button reads on breadboard
- [ ] **Handheld:** verify OLED display works
- [ ] **All three nodes:** verify they can hear each other's LoRa frames at bench distance with the same parameters (SF7, BW500, freq 915.0 MHz, sync 0x12)

---

## Phase 2 — Common firmware (shared by all three nodes)

Implement [`firmware/common/`](firmware/common/) — the shared protocol layer.

- [ ] Implement KISS framer (FEND/FESC byte stuffing) — ~100 lines
- [ ] Implement CRC-16/CCITT
- [ ] Implement frame structs (`ControlFrame`, `TelemetryFrame`, `HeartbeatFrame`) per [LORA_PROTOCOL.md § Frame format](LORA_PROTOCOL.md#frame-format)
- [ ] Implement `lora_proto_encode()` / `lora_proto_decode()`
- [ ] Implement AES-128-GCM wrapper using MbedTLS (built into Arduino core for both SAMD21 and STM32H7)
- [ ] Implement nonce generator (source_id + sequence + timestamp + random)
- [ ] Unit tests for framer (FEND-in-payload, max-length frame, zero-length frame)
- [ ] Unit tests for crypto (round-trip encrypt/decrypt, replay rejection, tamper rejection)
- [ ] Implement key provisioning utility `provision.py` (writes pre-shared key to flash via USB-CDC)
- [ ] Document key rotation procedure

---

## Phase 3 — Handheld firmware

- [ ] Implement `firmware/handheld_mkr/handheld.ino` skeleton with main loop at 50 Hz
- [ ] Wire joystick reads with deadband and calibration
- [ ] Wire button reads with debounce
- [ ] Implement TAKE CONTROL latch logic (30 s after button release)
- [ ] Implement E-STOP detection and signaling
- [ ] Implement OLED status screen (source state, RSSI, battery, take-control countdown)
- [ ] Build ControlFrame from inputs and TX at 50 Hz
- [ ] RX bench-test: send hand-crafted ControlFrame from PC SDR transmitter or another MKR, verify decode + display update
- [ ] LiPo battery + charging verification
- [ ] Move from breadboard to perfboard or custom PCB
- [ ] Enclosure assembly with cable strain relief

---

## Phase 4 — Tractor firmware

### M7 core (`firmware/tractor_h7/tractor.ino`)

- [ ] LoRa modem driver setup
- [ ] Cellular SARA-R412M setup with MKRNB library
- [ ] Implement `pick_active_source()` arbitration (per [LORA_PROTOCOL.md § Multi-source arbitration](LORA_PROTOCOL.md#multi-source-arbitration))
- [ ] Implement source-state tracking (last heartbeat, sequence #, RSSI)
- [ ] Implement IPC to M4 (push active ControlFrame every 50 ms)
- [ ] Implement **Modbus RTU master** to Opta valve controller per [TRACTOR_NODE.md § Modbus RTU register map](TRACTOR_NODE.md#modbus-rtu-register-map-max-carrier--opta)
  - [ ] 50 Hz writes of `valve_coils` + flow setpoints + watchdog_counter
  - [ ] 10 Hz reads of telemetry + safety_state
  - [ ] Detect Opta `WATCHDOG_TRIPPED` or `ESTOP_LATCHED` and propagate up to operator UI
- [ ] Implement telemetry publishers for each topic
- [ ] Implement MQTT-SN packet builder
- [ ] Implement microSD logging (rotated daily)
- [ ] Implement watchdog (M7 watchdog hits → reset; also stops Modbus master → Opta safe-mode within 200 ms)

### M4 core (`firmware/tractor_h7/tractor_m4.cpp`)

- [ ] 100 Hz deterministic loop
- [ ] Read ControlFrame from IPC shared memory
- [ ] Translate axes → valve-coil bitfield + flow setpoint values for Modbus
- [ ] Watchdog: ControlFrame age > 200 ms → zero coils + flow setpoint (Opta will independently catch this too via Modbus watchdog)
- [ ] Bench test: cycle valves with bench Opta + LED stand-ins at 100 Hz

### Hardware E-stop chain

- [ ] Wire latching safety relay (Phoenix Contact PSR series)
- [ ] Wire engine-kill solenoid path
- [ ] Wire 24 V valve coil rail through NC contact
- [ ] Test: M7 watchdog timeout drops valve power within 200 ms
- [ ] Test: handheld E-stop signal triggers safety relay within 200 ms

### Tractor-side telemetry sources

- [ ] Wire **NEO-M9N GPS via Qwiic** (chains off BNO086 on the MCP2221A bus); mount the SMA active patch antenna on cab roof with U.FL→SMA bulkhead pigtail; verify `i2cdetect` sees both 0x42 (GPS) and 0x4A/0x4B (IMU); add a second X8-side service that publishes NMEA at 1 Hz on `lifetrac/v25/telemetry/gps` (`topic_id=0x01`, already in `lora_bridge.py` `TOPIC_BY_ID`)
- [ ] Wire CAN-FD to engine ECU (if engine ECU available) — Deutsch DT04-5P J1939 harness
- [ ] Wire hydraulic pressure sensors (4–20 mA → Opta A0602 inputs, M12-A cordsets); confirm `hyd_supply_psi`/`hyd_return_psi` register values track a calibrated reference gauge
- [ ] Wire **BNO086 IMU via MCP2221A USB→Qwiic adapter** to an X8 USB host port; verify Yocto enumerates `/dev/i2c-N` (`i2cdetect -y N` shows BNO086 at 0x4A or 0x4B). Run a Python service on the X8 that reads the IMU at 50 Hz and publishes `lifetrac/v25/telemetry/imu` MQTT (roll/pitch/yaw + accel) at 5 Hz. The bridge ships it as `topic_id=0x07` over LoRa. (Switch to native I²C wired to the Max Carrier breakout only if a future use case needs sub-millisecond determinism.)
- [ ] Mount **USB UVC webcam** (Logitech C920 or ELP IP67) — route cable through panel-mount USB bulkhead with gland; verify Yocto enumerates as `/dev/video0` (`v4l2-ctl --list-devices`)
- [ ] Bring up GStreamer pipeline on X8: `v4l2src device=/dev/video0 ! image/jpeg ! jpegdec ! v4l2h264enc ! rtph264pay ! udpsink` for first-light WebRTC test
- [ ] Verify each source publishes to correct MQTT topic (per [LORA_PROTOCOL.md topic table](LORA_PROTOCOL.md#telemetryframe-variable-9128-bytes))

---

## Phase 4.5 — Opta valve-controller firmware (Modbus slave)

This is the *industrial I/O layer* that the Max Carrier H7 talks to over RS-485. Most logic ports from [RESEARCH-CONTROLLER/arduino_opta_controller/](RESEARCH-CONTROLLER/arduino_opta_controller/) — strip the MQTT/BLE control surfaces, replace with Modbus slave.

### Setup

- [ ] Install Arduino IDE + Opta board package (`Arduino Mbed OS Opta Boards`)
- [ ] Install `ArduinoRS485` and `ArduinoModbus` libraries
- [ ] Verify Opta WiFi boots, USB-C serial monitor works
- [ ] Verify Opta Ext D1608S enumerates over the expansion bus (`OptaController` API)
- [ ] Verify Opta Ext A0602 enumerates and 0–10 V output reaches full scale on a multimeter

### Modbus slave implementation (`firmware/tractor_opta/opta_valves.ino`)

- [ ] Initialize RS-485 at 115200 8N1 with `ArduinoRS485`
- [ ] Initialize `ArduinoModbus` as **slave at address 0x01**
- [ ] Allocate holding-register block 0x0000–0x0006 (7 registers) per [TRACTOR_NODE.md § Modbus RTU register map](TRACTOR_NODE.md#modbus-rtu-register-map-max-carrier--opta)
- [ ] Allocate input-register block 0x0100–0x010B (12 registers)
- [ ] On each `poll()`:
  - [ ] If `watchdog_counter` (0x0004) hasn't changed in 200 ms → set `safety_state = WATCHDOG_TRIPPED`, force all coils off, force flow setpoints to 0
  - [ ] Otherwise, copy `valve_coils` bitfield (0x0000) to onboard relays + D1608S relays
  - [ ] Copy `flow_setpoint_1/2` (0x0001/0x0002) to A0602 analog outputs (scale 0..10000 → 0..10 V)
  - [ ] If `arm_engine_kill` (0x0006) non-zero, energize engine-kill relay through PSR safety chain
  - [ ] Read all D1608S digital inputs into `digital_inputs` (0x0101)
  - [ ] Read all A0602 analog inputs, scale, write to 0x0102–0x0107
  - [ ] Update `safety_state` based on ignition-sense input + external E-stop loop monitor
- [ ] Implement onboard hardware watchdog (`IWatchdog`) at 500 ms; reset on every successful Modbus loop iteration
- [ ] Drive a GPIO output "Opta-alive" signal that feeds the PSR safety relay's monitored channel; goes low on `WATCHDOG_TRIPPED`
- [ ] Implement boot self-test: cycle each relay briefly, verify A0602 outputs hit 5 V midpoint, log to USB serial

### Carry-over from RESEARCH-CONTROLLER/arduino_opta_controller

- [ ] Port valve sequencing + deadband logic
- [ ] Port [`MICROTRAC_V17.10_OPTIMIZATION.md`](RESEARCH-CONTROLLER/MICROTRAC_V17.10_OPTIMIZATION.md) flow-valve scaling tables
- [ ] Port the four [code-review safety fixes from 2026-04-25](../AI%20NOTES/CODE%20REVIEWS/) (NaN clamp, non-blocking reconnect equivalent, stale-input zeroing, mode-switch polling) into the new Opta firmware from the start — they are already validated bugs

### Bench tests

- [ ] Master simulator (Python `pymodbus` on PC) writes valve_coils with random patterns at 50 Hz; verify Opta tracks within 50 ms
- [ ] Master simulator stops writing watchdog_counter; verify all coils drop within 200 ms
- [ ] Master simulator writes flow_setpoint = 5000; verify A0602 outputs 5.00 ± 0.05 V
- [ ] Pull RS-485 cable mid-operation; verify Opta drops to safe state, recovers cleanly when reconnected
- [ ] Power-cycle Opta while master is writing; verify coils stay off until Opta has completed boot self-test
- [ ] Run for 24 h with master simulator; verify zero missed cycles, zero false watchdog trips

---

## Phase 5 — Base station firmware (Linux side, runs in Docker on X8)

### Containers

- [ ] Write `docker-compose.yml` with services: nginx, web_ui, mosquitto, lora_bridge, timeseries
- [ ] **Mosquitto:** carry over [config/mosquitto.conf](config/mosquitto.conf), bind to LAN-only interface
- [ ] **lora_bridge** (Python):
  - [ ] Read serial from M7 over UART
  - [ ] Decode frames using shared protocol (port `lora_proto.cpp` to `lora_proto.py`)
  - [ ] Decrypt using shared AES-GCM (port `crypto.cpp` to `crypto.py`)
  - [ ] Publish telemetry to MQTT
  - [ ] Subscribe to control topics from web_ui, encrypt + frame, send over UART
  - [ ] Publish link-health topics (RSSI, SNR, loss)
- [ ] **web_ui** (FastAPI + Jinja2 + WebSockets):
  - [ ] HTTP routes: `/`, `/map`, `/telemetry`, `/log`, `/settings`, `/diagnostics`
  - [ ] WebSocket endpoint `/ws` for control + telemetry stream
  - [ ] Static assets: HTML, CSS, JS joystick widget
  - [ ] Authentication (basic password for now; key-based later)
- [ ] **nginx:** TLS termination, reverse proxy to web_ui, static asset caching
- [ ] **timeseries:** InfluxDB or SQLite storing all `lifetrac/v25/telemetry/*` topics

### Web UI front-end (browser-side JavaScript)

- [ ] Touch-friendly virtual joystick widget (canvas-based; check `nipplejs` library)
- [ ] WebSocket client with auto-reconnect
- [ ] Live telemetry sidebar (RPM, oil T, battery, RSSI, source)
- [ ] E-STOP button (always-active; sends over both LoRa via lora_bridge AND cellular MQTT)
- [ ] Source-active banner ("HANDHELD HAS CONTROL", "BASE HAS CONTROL", etc.)
- [ ] REQUEST CONTROL button (visible when not active source)
- [ ] Map page with Leaflet + cached OpenStreetMap tiles + live GPS marker
- [ ] Telemetry graph page with Plotly or Chart.js
- [ ] Diagnostics page with link-health graphs

### Base station M7 firmware (`firmware/base_h7/base.ino`)

- [ ] LoRa modem driver setup
- [ ] UART bridge to X8 (binary frame stream over high-density connector)
- [ ] Optional: cellular fallback for control if LoRa down (gated by 2-step UI confirmation)

---

## Phase 6 — Mast antenna installation (base station)

- [ ] Site survey: choose mast location (clear LoS to typical work area, away from buildings)
- [ ] Install ground rod, ≥2.5 m driven
- [ ] Erect mast (concrete base or guyed)
- [ ] Mount 8 dBi omni at top
- [ ] Run LMR-400 coax inside conduit
- [ ] Install lightning arrestor at mast base, ground to dedicated rod
- [ ] Verify SWR < 2:1 with VNA (or NanoVNA)
- [ ] Range-test by driving handheld+vehicle away from base, log RSSI vs distance

---

## Phase 7 — Integration & end-to-end testing

- [ ] All three nodes powered, in same room, exchange frames at 1 m bench distance
- [ ] Single-source test: only handheld active → tractor follows handheld
- [ ] Single-source test: only base UI active → tractor follows base
- [ ] Two-source test: both handheld and base active → tractor follows handheld (priority)
- [ ] Handover test: handheld releases control → tractor switches to base after 30 s latch + 500 ms timeout
- [ ] TAKE CONTROL test: base controlling → handheld grabs control with button → tractor switches immediately
- [ ] Failsafe test: power-off handheld while it's active source → tractor goes to neutral within 500 ms
- [ ] Failsafe test: power-off base while it's active source → tractor goes to neutral within 500 ms
- [ ] Replay attack test: capture a frame, retransmit later → tractor rejects
- [ ] Tamper test: flip a bit in a captured frame, retransmit → tractor rejects
- [ ] Latency measurement: handheld joystick → tractor valve, target ≤ 150 ms median
- [ ] Latency measurement: base UI joystick → tractor valve, target ≤ 250 ms median

---

## Phase 8 — Field testing

- [ ] Range test: base mast → tractor at 1 km, 5 km, 10 km, 15 km LoS
- [ ] Range test: base mast → tractor through light foliage at 1 km, 3 km
- [ ] Range test: handheld → tractor at 100 m, 500 m, 1 km, 2 km
- [ ] Vibration test: drive tractor over rough ground, verify no enclosure issues, no spurious failsafes
- [ ] Cellular fallback test: physically unplug LoRa antenna at tractor → verify cellular telemetry continues
- [ ] Engine-crank brown-out test: cold-start engine while tractor MCU is running → verify LiPo backup carries through
- [ ] Rain/IP rating test: spray-test enclosures with garden hose
- [ ] 24-hour soak test in shop: simulate normal operation pattern, log all events
- [ ] 7-day deployment test on real work site, log everything, fix any issues
- [ ] Document field-test results, update [HARDWARE_BOM.md](HARDWARE_BOM.md) with any part substitutions

---

## Phase 9 — Documentation, regulatory, release

- [ ] Write hookup guide (consolidated from [TRACTOR_NODE.md](TRACTOR_NODE.md), [BASE_STATION.md](BASE_STATION.md), [HANDHELD_REMOTE.md](HANDHELD_REMOTE.md))
- [ ] Write operator manual (how to use the handheld + web UI)
- [ ] Verify FCC §15.247 compliance with spectrum analyzer (handheld at +14 dBm, tractor at +20 dBm, base at +20 dBm + 8 dBi antenna = +26.3 dBm EIRP, all under +36 dBm limit)
- [ ] Open-source the firmware under GPLv3 (matching ELRS / Meshtastic norms)
- [ ] Open-source the web UI under AGPLv3 (so improvements stay open)
- [ ] Update top-level [LifeTrac-v25/README.md](../README.md) to reference this controller design
- [ ] Update [LifeTrac-v25/TODO.md](../TODO.md): mark "wireless control" item as in-progress, link here
- [ ] Tag a `controller-v1.0.0` release on GitHub
- [ ] Add controller hardware to the v25 main BOM

---

## Cross-cutting concerns (must address before Phase 7 integration)

These items cut across all three nodes and don't fit neatly into a single phase. Address each before end-to-end integration testing.

### Device pairing & key provisioning

- [ ] Define pairing bootstrap: tractor X8 generates AES-128 key + node ID on first boot, displays as **QR code** on its status OLED (or web UI for headless tractors)
- [ ] Handheld pairing flow: phone app or laptop scans tractor QR → writes key + node ID to MKR WAN 1310 over USB-C serial (one-time setup tool in `tools/pair_handheld.py`)
- [ ] Base station pairing: scan same QR from base X8 web UI `/settings/pair` page → key stored in encrypted config file
- [ ] Re-pair / key-rotation procedure documented in `OPERATIONS_MANUAL.md`
- [ ] Persistent AES-GCM nonce counter in tractor flash (survives reboot) to prevent replay-attack window after power cycle

### Firmware update strategy

- [ ] Document update path per node in `FIRMWARE_UPDATES.md`:
  - Tractor X8 / base X8: OTA over cellular (X8 pulls signed image, A/B partition rollback)
  - Tractor / base H7 co-MCU: flashed from X8 over internal SPI/UART
  - Opta: flashed from tractor X8 over RS-485 (Modbus file-transfer extension) OR USB in shop
  - Handheld MKR: USB-C only (too small a RAM for OTA over LoRa)
- [ ] Code-sign all binaries (Ed25519); X8 verifies before flashing co-MCUs

### Time synchronization

- [ ] Base X8 syncs to NTP over cellular, exposes time as LoRa beacon (1 Hz on a reserved frame type)
- [ ] Tractor X8 disciplines its RTC from base beacon when available, falls back to GPS PPS, falls back to free-running RTC
- [ ] All telemetry timestamps in UTC microseconds; document in `LORA_PROTOCOL.md`

### Safety case & cybersecurity

- [ ] Write `SAFETY_CASE.md`: hazard analysis (HAZOP-lite), claim ISO 13849 PL=c on the E-stop chain, document Phoenix PSR wiring as the safety function
- [ ] Document Modbus RS-485 link as a trust boundary (sealed enclosure, no external access)
- [ ] Base-station web UI: require WireGuard / Tailscale tunnel for any non-LAN access (no public HTTP exposure); document in `BASE_STATION.md`
- [ ] HTTPS + basic auth on LAN-only web UI as defence-in-depth

### Missing documentation

- [ ] Write `NON_ARDUINO_BOM.md` — consolidated DigiKey / Mouser / L-com / Phoenix Contact / Burkert / McMaster order list (counterpart to the Arduino-store list)
- [ ] Write `CALIBRATION.md` — joystick deadband, flow-valve 0–10 V → GPM curve, pressure-sensor zero, GPS antenna offset
- [ ] Write `FIELD_SERVICE.md` — diagnostic flowcharts, fuse map, common failure modes, spare-parts kit contents
- [ ] Write `OPERATIONS_MANUAL.md` — operator-facing (not engineer-facing): power-on, pairing, take-control, E-stop, charging the handheld

### Hardware-in-the-loop bench

- [ ] Add HIL bench rig to `HARDWARE_BOM.md` Dev Gear: 8× 12 V LEDs in place of valve coils, 8× 1 kΩ trimpots in place of pressure transducers, 2× DMM on the 0–10 V Burkert outputs, 12 V bench supply with current meter
- [ ] Document HIL bring-up procedure as part of Phase 1

### Radio vendor lock-in mitigation

- [ ] Abstract the radio HAL in `firmware/common/radio.h` so SX1276 (Murata SiP) and SX1262 (RFM modules) can be swapped firmware-only
- [ ] Keep an RFM95W + bare STM32 reference design sketch in `RESEARCH-CONTROLLER/` as Murata-EOL insurance

---

## Stretch goals (Phase 10+)

- [ ] Add MIPI camera on tractor (Portenta X8 only; needs to swap H7 for X8 on tractor side)
- [ ] WiFi video streaming when tractor + base are within WiFi range (see [VIDEO_OPTIONS.md](VIDEO_OPTIONS.md))
- [ ] LoRa thumbnail JPEGs (~5 KB every 2 seconds) when WiFi out of range
- [ ] **Image processing & transmission for the LoRa fallback link** — see [RESEARCH-CONTROLLER/VIDEO_COMPRESSION/README.md](RESEARCH-CONTROLLER/VIDEO_COMPRESSION/README.md) for full analysis. Build in this order:
  - [ ] **Phase A — SSDV slideshow (fallback floor):** ffmpeg/libjpeg on tractor X8 → 160×120 JPEG every 5–10 s, SSDV-chunked with FEC into ~250 B LoRa packets, reassembler + thumbnail tile in base-station web UI. Target: 2–5 kbps, 5–30 s latency. Days of work; ship first.
  - [ ] **Phase B — Situational overlay (don't compress video, compress the situation):** YOLO-nano + lane/edge + bucket-pose detection on tractor X8 (i.MX 8M Mini, NEON only — no NPU), send bounding boxes + classes + free-space mask + bucket angle (~50–500 B/frame at 5 fps = 2–20 kbps), base-station X8 renders synthetic top-down/first-person view from the structured data + tractor CAD. Annotate UI as "synthesized".
  - [ ] **Phase C — SVT-AV1 + ROI baseline:** 256×192 / 5 fps, GOP 150, CRF ~50, ROI map weighting lower-center (bucket) + top-center (horizon), film-grain table, dav1d on base. Reuse comma.ai grid-search CSV for tuning. Target: 8–15 kbps. Comparison baseline for Phase D.
  - [ ] **Phase D — Keyframe + neural temporal inflate:** real keyframe (256×192 JPEG/AVIF, ~3–5 kB) every 5–10 s + tiny encoder net producing ~64–128 B latent per frame at 5 fps, base-station decoder warps keyframe using latent + CAN ego-motion (already on LoRa control channel — free side signal). Adapt comma.ai `neural_inflate` (PR #49) or `mask2mask` (PR #53). Distill to ≤50 MFLOPs/frame to fit i.MX 8M Mini NEON budget. Target: 6–10 kbps, 200–600 ms latency.
  - [ ] **Dataset capture:** record ≥10 h of tractor footage at 256×192 / 5 fps with synchronized CAN ego-motion (yard, field, bucket loading, mud, dust, dawn/dusk, rain) for Phase D fine-tuning. Use the MIPI camera bullet above.
  - [ ] **Safety annotations:** any phase that synthesizes/hallucinates pixels (B, D, future mask2mask) must overlay a "SYNTHESIZED" badge in the base-station UI; tractor X8 keeps full local recording for incident review independent of what crosses LoRa.
  - [ ] **Bandwidth arbitration:** image stream MUST yield to control + telemetry packets — codec drops to next-lower phase (D → C → A) when LoRa link budget tightens; never starves control loop.
- [ ] Autonomy: GPS waypoint following on the M7 core
- [ ] ROS 2 bridge (port from [RESEARCH-CONTROLLER/ros2_bridge/](RESEARCH-CONTROLLER/ros2_bridge/))
- [ ] DroidPad mobile-app integration as fourth control source (port from [RESEARCH-CONTROLLER/DROIDPAD_INTEGRATION.md](RESEARCH-CONTROLLER/DROIDPAD_INTEGRATION.md))
- [ ] Multi-tractor base station: single base controls a fleet, each tractor has unique source ID

---

## Risk register

| Risk | Likelihood | Impact | Mitigation |
|---|---|---|---|
| Custom firmware control-path bug causes runaway implement | Medium | **Critical** | Hardware E-stop independent of MCU; 200 ms M4 watchdog; bench test before every field session |
| FCC EIRP exceeded with high-gain mast antenna | Low | High (regulatory) | Software EIRP cap; spectrum analyzer verification before deployment |
| LoRa SiP firmware bug in Murata module | Low | Medium | Murata SiP is mature (in production since 2017); fall back to dedicated SX1276 module if discovered |
| Portenta product line discontinuation | Low | High | Use [RESEARCH-CONTROLLER/](RESEARCH-CONTROLLER/) SparkFun stack as alternative path |
| Cellular Cat-M1 not available at tractor work site | Medium | Low | Cellular is backup-only; LoRa is primary; degraded mode is well-tested |
| Mast lightning strike | Low | High | Lightning arrestor + dedicated ground rod; insurance |
| Operator confusion over source priority | High | Medium | Clear UI banner; physical TAKE CONTROL button is unambiguous; operator training |
| Murata CMWX1ZZABZ EOL / single-source | Low | High | Abstract radio HAL; keep RFM95W reference design in RESEARCH-CONTROLLER/ |
| Replay attack after tractor power cycle | Low | High | Persistent AES-GCM nonce counter in flash; session re-key on pairing |

---

## See also

- [ARCHITECTURE.md](ARCHITECTURE.md) — system design
- [HARDWARE_BOM.md](HARDWARE_BOM.md) — what to buy
- [LORA_PROTOCOL.md](LORA_PROTOCOL.md) — air-interface spec
- [TRACTOR_NODE.md](TRACTOR_NODE.md) · [BASE_STATION.md](BASE_STATION.md) · [HANDHELD_REMOTE.md](HANDHELD_REMOTE.md) — per-tier build guides
- [WIRELESS_OPTIONS.md](WIRELESS_OPTIONS.md) — comparison of wireless technologies considered
- [VIDEO_OPTIONS.md](VIDEO_OPTIONS.md) — video streaming options
- [RESEARCH-CONTROLLER/](RESEARCH-CONTROLLER/) — earlier prototypes and research docs
- [LifeTrac-v25/TODO.md](../TODO.md) — top-level v25 TODO list
