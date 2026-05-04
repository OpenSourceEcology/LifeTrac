# LifeTrac v25 Controller — Development TODO

> **📋 Pre-deployment summary:** for a single consolidated view of
> what still has to happen across all subsystems before field
> deployment (controller + structural + hydraulic + integration +
> field tests + regulatory), see the
> **[Pre-field-deployment checklist](../TODO.md#pre-field-deployment-checklist-open-items-as-of-2026-05-04)**
> section in the top-level [LifeTrac-v25/TODO.md](../TODO.md). This
> file remains the authoritative source for the controller-side phase
> plan; the top-level checklist links back to specific items here.

> **Scope (2026-04-26):** LoRa-only per [MASTER_PLAN.md](MASTER_PLAN.md). Cellular (Cat-M1 / SARA-R412M) line items below are **archived** — do not order, do not implement. Operator-browser ↔ base-station LAN/WiFi is retained. Legacy WiFi/BLE/MQTT-over-WiFi work is in [RESEARCH-CONTROLLER/](RESEARCH-CONTROLLER/).
>
> **Image-pipeline scope (2026-04-27):** see the canonical [IMAGE_PIPELINE.md](IMAGE_PIPELINE.md) implementation plan. The image-pipeline tasks in Phases 4–5 below are pulled from that plan; if they ever drift, IMAGE_PIPELINE.md is the source of truth.
>
> **LoRa system scope (2026-04-27):** see the canonical [LORA_IMPLEMENTATION.md](LORA_IMPLEMENTATION.md) implementation plan. The LoRa-stack tasks scattered across Phases 1–5 below are pulled from that plan; if they ever drift, LORA_IMPLEMENTATION.md is the source of truth for *what to build*, [LORA_PROTOCOL.md](LORA_PROTOCOL.md) for the wire bytes, and [MASTER_PLAN.md §8.17](MASTER_PLAN.md) for PHY policy pins.

Comprehensive task list for implementing the three-tier controller architecture (Portenta Max Carrier on tractor + Portenta Max Carrier with X8 base station + MKR WAN 1310 handheld). See [ARCHITECTURE.md](ARCHITECTURE.md) for the design.

Tasks are organized by phase. Hardware purchases come first because lead times dominate the schedule.

---

## Phase 0 — Hardware procurement & shop setup

### Tractor node hardware

- [ ] Order Arduino Portenta Max Carrier (ABX00043)
- [ ] **Order Arduino Portenta X8 (ABX00049)** — same SKU as base station. Canonical per [MASTER_PLAN.md §8.9](MASTER_PLAN.md); the X8's onboard STM32H747 co-MCU runs the realtime M7+M4 firmware bare-metal, and the i.MX 8M Mini side gives Linux/SSH/MIPI camera.
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
- [ ] **Order [Coral Mini PCIe Accelerator](https://coral.ai/products/pcie-accelerator/) (~$25)** — *strongly recommended optional* per [MASTER_PLAN.md §8.19](MASTER_PLAN.md). Unproven on the Max Carrier — the Phase 1 validation spike below decides whether this stays in the BOM, gets swapped for the Coral USB Accelerator, or is dropped entirely.
- [ ] *(Spike-conditional)* Order [Coral USB Accelerator](https://www.digikey.com/en/products/detail/coral/G950-01456-01/10256669) (~$60) **only if** the Mini PCIe spike fails
- [ ] *(If Coral fitted)* Order ~15×15×8 mm stick-on aluminium heatsink for the Edge TPU package
- [ ] **2-day validation spike (gate before BOM lock):** install Coral Mini PCIe in the Max Carrier; confirm PCIe enumeration in `lspci`, `gasket`/`apex` driver loads against the X8 Yocto kernel, sustained 30-min inference without thermal throttle, slot power budget adequate. Document outcome in [HARDWARE_BOM.md](HARDWARE_BOM.md). On failure, repeat with the USB Accelerator; on second failure, ship CPU-only and degrade the image pipeline per [MASTER_PLAN.md §8.19](MASTER_PLAN.md).

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

- [ ] Order ×2 of each: Max Carrier, MKR WAN 1310, Portenta X8
- [ ] Order ×2 spare LoRa antennas, cellular antennas, joysticks, OLEDs

---

## Phase 1 — Bench bring-up

**2026-05-04 bench status:** two Portenta X8 + Max Carrier stacks have
been flashed with the `tractor_h7` M7 image at `0x08040000`. Both M7
cores reached `loop()` and advanced SRAM4 liveness with CFSR/HFSR = 0
using the X8 no-USB, bit-banged-SPI, nonce-PRNG bring-up image. This is
partial W4-pre evidence only; USB-CDC, rail, blink/echo, stock M7<->M4
handshake, and W4-00 TX/RX burst evidence are still open. Newer radio
diagnostics enqueue frames but fail before TX start (`stageMode(TX)`
returns `-16`; direct SX127x register snapshots are zero), so the Max
Carrier LoRa interface must be revalidated before W4-00. Details:
[`../AI NOTES/2026-05-04_Portenta_X8_M7_W4_Pre_Bringup_Status.md`](../AI%20NOTES/2026-05-04_Portenta_X8_M7_W4_Pre_Bringup_Status.md).

- [ ] Set up shared Git repo with subfolders for `firmware/handheld_mkr`, `firmware/tractor_h7` (runs on the X8's onboard STM32H747 co-MCU), `firmware/tractor_opta`, `firmware/tractor_x8` (Linux services), `firmware/common`, `base_station/` (Docker compose root). Per [MASTER_PLAN.md §8.2](MASTER_PLAN.md), there is **no `firmware/base_*/` target**. 2026-05-04 interface update: revalidate the original "Linux drives SX1276 directly over SPI" assumption for the Max Carrier; Arduino's X8 gateway example drives the onboard Murata LPWAN module as an AT modem on `/dev/ttymxc3` with reset on `/dev/gpiochip5` line 3.
- [ ] Set up PlatformIO with Portenta X8 + MKR WAN 1310 + Opta board support
- [ ] Set up Arduino-CLI CI (extend existing [ARDUINO_CI.md](ARDUINO_CI.md) setup)
- [ ] **Tractor:** verify Max Carrier + X8 boots, M7 blink works on the X8's onboard H747 co-MCU, M4 blink works
- [ ] **Tractor:** verify LoRa TX/RX from M7 with RadioLib at 915 MHz, **SF7 / BW 250 kHz / CR 4-5** per [LORA_PROTOCOL.md](LORA_PROTOCOL.md) / [DECISIONS.md](DECISIONS.md) D-A2. 
  **2026-05-04 update:** Definitive confirmation received: The Max Carrier physically does not route the Murata module's SPI pins. It acts purely as a UART AT modem (19200 or 115200 baud). Raw `RadioLib` SPI control is impossible. The tractor M7 firmware must be refactored to use `Serial3` (or the equivalent Linux X8 serial endpoint) and AT commands (e.g. `AT`, `AT+VER?`, `AT+APPEUI`, etc).
- [ ] **Tractor:** verify cellular SARA-R412M registers and sends a test MQTT publish
- [ ] **Tractor:** wire D1608S SSR1–SSR4 to four directional valve coils (boom-up, boom-down, bucket-curl, bucket-dump) per [MASTER_PLAN.md §8.18](MASTER_PLAN.md); wire the remaining four directional coils (drive LH/RH fwd/rev) to D1608S SSR5–SSR8. The Opta base's onboard EMRs are reserved for engine-kill / horn / parking-brake / spare. Drive 8 LEDs as coil stand-ins from M4 core.
- [ ] **Base:** verify Max Carrier + X8 boots, get SSH access
- [ ] **Base:** install Docker on X8 Yocto image
- [ ] **Base:** verify Linux can drive the Max Carrier LoRa module from `base_station/lora_bridge.py` (no Arduino firmware on the base H747 per [MASTER_PLAN.md §8.2](MASTER_PLAN.md)). 2026-05-04 update: first prove whether this is `/dev/ttymxc3` AT commands to the Murata `CMWX1ZZABZ-078` module rather than raw SPI to an exposed SX1276.
- [ ] **Base:** verify Gigabit Ethernet works, get DHCP lease on office LAN
- [ ] **Handheld:** flash MKR WAN 1310 with `RadioLib` "hello world" sketch, verify LoRa TX/RX
- [ ] **Handheld:** verify joystick analog reads + button reads on breadboard
- [ ] **Handheld:** verify OLED display works
- [ ] **All three nodes:** verify they can hear each other's LoRa frames at bench distance with the same parameters (SF7, **BW 250 kHz**, CR 4-5, freq 915.0 MHz, sync 0x12) per [LORA_PROTOCOL.md](LORA_PROTOCOL.md) / [DECISIONS.md](DECISIONS.md) D-A2

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

### Phase 2.LoRa — LoRa-stack tasks consolidated from [LORA_IMPLEMENTATION.md](LORA_IMPLEMENTATION.md)

Major implementation-plan-level tasks not already enumerated in Phases 2–5 above. Cross-link each to the LORA_IMPLEMENTATION.md § that owns the spec.

**Phase 2.LoRa.0 — Week-1 bench measurements (block Phase 1–5 work that depends on the numbers):**

- [ ] **R-7 retune-cost bench** (per [LORA_IMPLEMENTATION.md §8 week 1](LORA_IMPLEMENTATION.md)): measure `setFrequency` + `setSpreadingFactor` + `setBandwidth` + `setCodingRate` cost on SX1276 via RadioLib. Record baseline. **If > 5 ms,** burst-batching code path becomes mandatory in `lora_proto.cpp`. *Sketch in place at [`firmware/bench/lora_retune_bench/`](firmware/bench/lora_retune_bench/); needs an actual run.*
- [ ] **TX→RX turnaround + CSMA backoff bench** (per [LORA_IMPLEMENTATION.md L8](LORA_IMPLEMENTATION.md)): record so the airtime ledger is accurate, not assumed.
- [ ] **Control cadence / PHY blocker** — resolve the 2026-04-27 airtime mismatch before field motion: encrypted `ControlFrame` is ~92 ms at SF7/BW125 while the current control cadence is 20 Hz (50 ms). Decide between BW250/BW500, lower cadence, reduced overhead, or a dedicated control radio/channel; bench-verify before hydraulic testing.
- [ ] **Image fragment cap blocker** — resolve the 2026-04-27 airtime mismatch before image-pipeline bring-up: 32 B at SF7/BW250 estimates at ~36 ms, so the 25 ms cap currently allows only ~15 B cleartext fragments unless PHY/cap changes.
- [x] ~~**Cross-doc cascade pass** — land the IMAGE_PIPELINE.md §3.2 reservations (`0x28`/`0x29`/`0x2A`/`0x63`/badge enum) into [LORA_PROTOCOL.md](LORA_PROTOCOL.md) proper.~~ **✅ Done** — see [LORA_PROTOCOL.md topic table](LORA_PROTOCOL.md#telemetryframe-variable-9128-bytes) (`0x28`/`0x29`/`0x2A`), [opcode table](LORA_PROTOCOL.md#command-frame-opcodes) (`0x63`), and the badge enum table at line 270.

**Phase 2.LoRa.1 — PHY policy implementation (per [LORA_IMPLEMENTATION.md §3](LORA_IMPLEMENTATION.md)):**

- [x] ~~**Three-profile PHY in `lora_proto.cpp`**~~ **✅ Done** — `LP_PHY_CONTROL_SF7/SF8/SF9`, `LP_PHY_TELEMETRY`, `LP_PHY_IMAGE` defined in [`firmware/common/lora_proto/lora_proto.c`](firmware/common/lora_proto/lora_proto.c) and mirrored in [`base_station/lora_proto.py`](base_station/lora_proto.py). Per-frame retune via `CMD_LINK_TUNE` mechanism still needs runtime wiring on the M7.
- [x] **Adaptive control-link SF ladder** with R-8 hysteresis (N=3 consecutive bad 5 s windows for any transition; 30 s clean for SF↑→SF↓). `CMD_LINK_TUNE` sent twice back-to-back at old-then-new SF; revert + fail-counter increment if no Heartbeat at new SF within 500 ms. *Implemented in [`firmware/tractor_h7/tractor_m7.ino`](firmware/tractor_h7/tractor_m7.ino) `poll_link_ladder()` + `try_step_ladder()` + `send_link_tune()`. Reciprocal handheld/base receiver still needs the matching retune handler (handheld.ino + lora_bridge.py both currently no-op on inbound `CMD_LINK_TUNE`).*
- [x] **Image-link auto-fallback ladder** wiring (per [IMAGE_PIPELINE.md §3.4](IMAGE_PIPELINE.md)): `RollingAirtimeLedger` + `EncodeModeController` from [`base_station/link_monitor.py`](base_station/link_monitor.py) are now wired into [`lora_bridge.py`](base_station/lora_bridge.py) — every TX and RX records airtime via `attribute_phy()`, a 1 Hz worker emits `CMD_ENCODE_MODE` on rung change, and the `(U_image, U_telemetry, U_total)` triple is published as retained JSON on `lifetrac/v25/control/source_active`. Alarms fire on `U_telemetry > 30 %` and `U_total > 60 %`.

**Phase 2.LoRa.2 — MAC: FHSS + CSMA (per [LORA_IMPLEMENTATION.md §3.2](LORA_IMPLEMENTATION.md), [MASTER_PLAN.md §8.17 FHSS bullet](MASTER_PLAN.md)):**

- [x] ~~**8-channel hop sequence** in `lora_proto.cpp`~~ **✅ Done** — `lp_fhss_channel_index` / `lp_fhss_channel_hz` in [`firmware/common/lora_proto/lora_proto.c`](firmware/common/lora_proto/lora_proto.c); Python mirror `fhss_channel_index` / `fhss_channel_hz` in [`base_station/lora_proto.py`](base_station/lora_proto.py). Deterministic Fisher-Yates seeded by `key_id`, 8 channels @ 3.25 MHz starting 902 MHz.
- [x] **CSMA skip-busy** — helper landed: `lp_csma_pick_hop()` in [`firmware/common/lora_proto/lora_proto.c`](firmware/common/lora_proto/lora_proto.c) and `pick_csma_hop()` in [`base_station/lora_proto.py`](base_station/lora_proto.py); default threshold –90 dBm, max 4 skips, falls through to last candidate so a control frame still goes out. Unit tests cover clean-channel, single-skip, all-busy, and threshold-boundary cases. *Caller wiring (RadioLib `scanChannel` on tractor/handheld TX path, base-SPI driver on bridge) and the audit-log skip-event hook still open.*
- [ ] **Spectrum-analyser FHSS verification** (week 6, per [LORA_IMPLEMENTATION.md §8](LORA_IMPLEMENTATION.md)) — confirm 8 channels active, ≤ 12.5 % per-channel dwell over 60 s, no out-of-band emissions, EIRP ≤ +36 dBm. Pre-Phase-9 FCC verification.

**Phase 2.LoRa.3 — Priority queue + airtime cap (per [LORA_IMPLEMENTATION.md §4](LORA_IMPLEMENTATION.md)):**

- [x] **P0/P1/P2/P3 priority queue** in the base-station bridge — [`base_station/lora_bridge.py`](base_station/lora_bridge.py) routes every TX through `_tx_queue` (heap by priority + FIFO tiebreaker) drained by a single `_tx_worker` thread; classification via `classify_priority()` in [`base_station/lora_proto.py`](base_station/lora_proto.py) covered by `test_classify_priority_buckets`. P0 = ControlFrame/Heartbeat/CMD_ESTOP/CMD_LINK_TUNE/CMD_PERSON_APPEARED; P1 = CMD_CLEAR_ESTOP/CMD_ROI_HINT/CMD_REQ_KEYFRAME/CMD_CAMERA_SELECT/CMD_ENCODE_MODE; P2 = telemetry; P3 = image fragments. *Firmware-side queue (handheld + tractor in `lora_proto.c`) still open — those nodes only TX a handful of frame types so an explicit queue is lower-priority there.*
- [ ] **L1/R-6 25 ms-per-fragment airtime cap** — enforced uniformly on P2 telemetry and P3 image fragments. No P2/P3 frame can begin TX if remaining airtime in current opportunity > 25 ms.
- [ ] **R-6 telemetry fragmentation** — oversized `TelemetryFrame` payloads fragment using the `TileDeltaFrame` scheme; base-station bridge reassembles before MQTT publish.
- [ ] **Burst-batching code path** (gated by R-7 measurement) — if retune > 5 ms, batch image fragments so radio retunes at most twice per refresh window (once into image PHY, once back).

**Phase 2.LoRa.4 — Security (per [LORA_IMPLEMENTATION.md §6](LORA_IMPLEMENTATION.md)):**

- [x] **Replay-defence sliding window** — 64-frame per-source `LpReplayWindow` lives in [`firmware/common/lora_proto/lora_proto.c`](firmware/common/lora_proto/lora_proto.c) (`lp_replay_init` / `lp_replay_check_and_update`); used per source in [`firmware/tractor_h7/tractor_m7.ino`](firmware/tractor_h7/tractor_m7.ino) `process_air_frame`. Bit-compatible Python mirror `ReplayWindow` in [`base_station/lora_proto.py`](base_station/lora_proto.py) covered by 5 unit tests in [`base_station/tests/test_lora_proto.py`](base_station/tests/test_lora_proto.py) (duplicate, in-order advance, out-of-order, too-old, 16-bit wrap).
- [x] ~~**Nonce generator**~~ **✅ Done** — `build_nonce` in [`base_station/lora_proto.py`](base_station/lora_proto.py); 12 B = `source_id (1) ‖ seq (2) ‖ epoch_s (4) ‖ random (5)`. C-side mirror still needs to land in [`firmware/common/lora_proto/crypto_stub.c`](firmware/common/lora_proto/crypto_stub.c).
- [x] ~~**`tools/provision.py`**~~ **✅ Done** — [`tools/provision.py`](tools/provision.py) writes pre-shared 16 B key + 4 B key ID via USB-CDC.
- [ ] **`KEY_ROTATION.md`** (NEW, week 2 per [LORA_IMPLEMENTATION.md §11](LORA_IMPLEMENTATION.md)) — operator-facing key rotation procedure. Companion to `provision.py`. **Decision L-O2 (rotation cadence) deadline: before week 2.**

**Phase 2.LoRa.5 — Multi-source arbitration (per [LORA_IMPLEMENTATION.md §5](LORA_IMPLEMENTATION.md)):**

- [x] ~~**`pick_active_source()`** runs every M7 loop iteration~~ **✅ Done** — [`firmware/tractor_h7/tractor_m7.ino`](firmware/tractor_h7/tractor_m7.ino) line 130, `HEARTBEAT_TIMEOUT_MS = 500` enforced. Failsafe to neutral on `SOURCE_NONE`.
- [ ] **TAKE CONTROL latch** — 30 s P0 priority after button release; persists across heartbeat misses but not across `SOURCE_NONE`.
- [ ] **Source-active publisher** — topic `0x10` carries `active_source` + per-source RSSI/SNR + current SF rung + airtime-% triple `(U_image, U_telemetry, U_total)` at 1 Hz + on every change. *Wire in [`base_station/lora_bridge.py`](base_station/lora_bridge.py) once airtime ledger is integrated.*
- [ ] **Audit log** — every source transition, SF transition, encode-mode transition, FHSS skip event, replay rejection, GCM-tag rejection, CSMA backoff event appended to the [§8.10 black-box logger](MASTER_PLAN.md).

**Phase 2.LoRa.6 — Observability (per [LORA_IMPLEMENTATION.md §7](LORA_IMPLEMENTATION.md)):**

- [x] **`link_monitor.py`** — rolling 10 s airtime ledger per profile; computes `U_image`, `U_telemetry`, `U_total`; alarms if `U_telemetry > 30 %` or `U_total > 60 %`; emits `CMD_ENCODE_MODE` per the image auto-fallback ladder. *Module at [`base_station/link_monitor.py`](base_station/link_monitor.py) is now wired into [`lora_bridge.py`](base_station/lora_bridge.py); alarm thresholds live as `U_TELEMETRY_ALARM` / `U_TOTAL_ALARM` in the bridge worker.*
- [ ] **`tools/lora_rtt.py`** — handheld→tractor→base RTT measurement harness via timestamp echo. Run nightly during build weeks 1–10; audit-log diff per night; flag regressions.
- [ ] **Operator UI surface** — airtime-% bar (green < 40 %, yellow 40–60 %, red > 60 %); current SF rung pill; FHSS hop indicator (channel 1–8); two-source-active banner if both handheld and base are within heartbeat timeout.

**Phase 2.LoRa.7 — Validation gates (per [LORA_IMPLEMENTATION.md §9](LORA_IMPLEMENTATION.md), all must pass before field test):**

- [ ] **L-V1.** L1 P0 starvation — 30-min mixed-mode stress, zero P0 TX-start delays > 25 ms (joint pass with image V1).
- [ ] **L-V2.** L3 failsafe — power-off active source, valve neutral within 500 ms p99 (repeat for HANDHELD, BASE, AUTONOMY).
- [ ] **L-V3.** Three-source arbitration — 30 s latch + 500 ms timeout handover.
- [ ] **L-V4.** TAKE CONTROL preemption — within next M7 tick (≤ 20 ms).
- [ ] **L-V5.** Replay rejection — captured-frame retransmit rejected + logged.
- [ ] **L-V6.** Tamper rejection — bit-flipped frame rejected via GCM tag + logged.
- [ ] **L-V7.** Adaptive SF ladder — step-attenuator sweep, no hunting, 30 s clean before step-up.
- [ ] **L-V8.** R-6 telemetry fragmentation round-trip — 120 B payload survives bit-identical via fragment+reassemble.
- [ ] **L-V9.** R-7 retune cost recorded; burst-batching code path validated under image load if > 5 ms.
- [ ] **L-V10.** L5 FHSS spectrum-analyser compliance — 8 channels, ≤ 12.5 % dwell, no OOB, EIRP ≤ +36 dBm.
- [ ] **L-V11.** Latency — handheld→valve ≤ 150 ms p99; base→valve ≤ 250 ms p99.
- [ ] **L-V12.** Field range — base mast→tractor ≥ 1 km LoS minimum; handheld→tractor ≥ 500 m minimum.

**Phase 2.LoRa.8 — Open scope decisions (need a human stakeholder):**

- [ ] **L-O1 — Region.** US 915 MHz only for v25, or also EU 868 MHz? Affects FHSS channel plan + per-region key-ID prefix. **Deadline: before week 1 (Phase 0 procurement).** Default if undecided: US 915 MHz only.
- [ ] **L-O2 — Key rotation cadence.** Annual / on-incident-only / never. Affects whether `provision.py` ships with operator-runnable docs or stays a workshop tool. **Deadline: before week 2.** Default if undecided: on-incident-only.
- [ ] **L-O3 — Two-radio split (Revisit-4) field-test escape hatch.** Adopt in v25 if L-V1 / image-V1 joint gate fails in field test, or hold the line and downshift the encoder more aggressively? **Deadline: only on documented gate failure.** Default if undecided: hold the line; revisit only on field failure.

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
- [ ] **Mosquitto:** carry over [RESEARCH-CONTROLLER/config/mosquitto.conf](RESEARCH-CONTROLLER/config/mosquitto.conf), bind to LAN-only interface
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
  - [ ] Authentication: **single shared PIN** (4–6 digits) configured at first boot, per [MASTER_PLAN.md §8.5](MASTER_PLAN.md)
- [ ] **nginx:** reverse proxy to web_ui + static asset caching. **Plain HTTP on port 80, LAN-only** per [MASTER_PLAN.md §8.5](MASTER_PLAN.md) (no TLS for v25; threat model is "physical LAN access = trusted operator").
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

### Base station services (Linux only)

Per [MASTER_PLAN.md §8.2](MASTER_PLAN.md), the base station runs **no Arduino firmware**. There is no `firmware/base_h7/` target to build, flash, or CI. 2026-05-04 interface update: revalidate the original raw-SPI assumption for `base_station/lora_bridge.py`; Arduino's X8 Max Carrier example drives the onboard Murata LPWAN module through `/dev/ttymxc3` AT commands, not a `/dev/spidev*` SX1276 node.

### Image pipeline (per [IMAGE_PIPELINE.md](IMAGE_PIPELINE.md), [MASTER_PLAN.md §8.19](MASTER_PLAN.md), [BASE_STATION.md § Image pipeline](BASE_STATION.md#image-pipeline-portenta-x8-linux-side), [LORA_PROTOCOL.md § TileDeltaFrame](LORA_PROTOCOL.md#tiledeltaframe-image-pipeline-i--p-frames))

**Phase 5.0 — Protocol-level reservations (do FIRST, before any image-pipeline code lands):**

- [x] **Reserve topic ID `0x28`** `video/motion_vectors` in [LORA_PROTOCOL.md](LORA_PROTOCOL.md) (optical-flow microframes, degraded mode Q)
- [x] **Reserve topic ID `0x29`** `video/wireframe` in [LORA_PROTOCOL.md](LORA_PROTOCOL.md) (PiDiNet edges, extreme degraded mode P)
- [x] **Reserve topic ID `0x2A`** `video/semantic_map` as **v26-only** placeholder in [LORA_PROTOCOL.md](LORA_PROTOCOL.md) (do NOT implement in v25)
- [x] **Reserve opcode `0x63`** `CMD_ENCODE_MODE` (base → tractor, P2): `{full | y_only | motion_only | wireframe}` for the §3.4 auto-fallback ladder
- [x] **Add Badge enum table** to [LORA_PROTOCOL.md](LORA_PROTOCOL.md) per [IMAGE_PIPELINE.md §3.3](IMAGE_PIPELINE.md): `Raw`, `Cached`, `Enhanced`, `Recolourised`, `Predicted`, `Synthetic`, `Wireframe` — base attaches, browser must fail-closed if missing/malformed
- [ ] **LoRa PHY revisit — Revisit-3** (per [IMAGE_PIPELINE.md §13.1](IMAGE_PIPELINE.md)): **✅ Spec shipped** in [IMAGE_PIPELINE.md §3.4](IMAGE_PIPELINE.md). `link_monitor.py` implementation (week 5): use `RadioLib::getTimeOnAir`, apply 3-window hysteresis, surface `U` + ladder rung on telemetry topic `0x10`
- [ ] **LoRa PHY revisit — Revisit-4** (per [IMAGE_PIPELINE.md §13.2](IMAGE_PIPELINE.md)): **✅ Documented** in [MASTER_PLAN.md §8.17.1](MASTER_PLAN.md) as the v26 escape hatch. **Not adopted for v25.** No build action; revisit only if the C1 gate fails in field testing.
- [ ] **LoRa PHY revisit — Revisit-5** (per [IMAGE_PIPELINE.md §13.1](IMAGE_PIPELINE.md)): **✅ Policy shipped** in [MASTER_PLAN.md §8.17](MASTER_PLAN.md) — 8-channel FHSS across 902–928 MHz, 3.25 MHz spacing, ~12.5 % per-channel dwell. Implementation: deterministic hop sequence in `lora_proto.cpp` seeded by AES key ID; CSMA skip-busy-channel rule. Bench-verify duty calc with spectrum analyser before [Phase 9 FCC verification](#phase-9--documentation-regulatory-release).
- [ ] **LoRa PHY revisit — R-6** (per [IMAGE_PIPELINE.md §13.1](IMAGE_PIPELINE.md)): **✅ Policy shipped** in [MASTER_PLAN.md §8.17](MASTER_PLAN.md) — P2 telemetry frames inherit the 25 ms airtime cap; oversized topic payloads fragment using the `TileDeltaFrame` scheme. Implementation: extend the fragment scheme to `TelemetryFrame` in `lora_proto.cpp`; base-station bridge reassembles before MQTT publish.
- [ ] **LoRa PHY revisit — R-7** (per [IMAGE_PIPELINE.md §13.3](IMAGE_PIPELINE.md)): **Week 1 bench task** — measure actual `CMD_LINK_TUNE` retune cost (`setFrequency` + `setSpreadingFactor` + `setBandwidth` + `setCodingRate`) on the SX1276 via RadioLib. **If > 5 ms,** implement burst-batching of image fragments so the radio retunes at most twice per refresh window. Until measured, plan the burst-batching code path conservatively.
- [ ] **LoRa PHY revisit — R-8** (per [IMAGE_PIPELINE.md §13.1](IMAGE_PIPELINE.md)): **✅ Policy shipped** in [MASTER_PLAN.md §8.17](MASTER_PLAN.md) and [IMAGE_PIPELINE.md §3.4](IMAGE_PIPELINE.md) — require N=3 consecutive bad 5 s windows for any SF or `CMD_ENCODE_MODE` ladder transition. Implementation: hysteresis state machine in the SF-ladder logic in `lora_proto.cpp` and in `link_monitor.py`.

**Phase 5A — Foundation (CPU-only, no AI accelerator required, ships value alone):**

- [ ] `image_pipeline/reassemble.py` — collect `TileDeltaFrame` (topic `0x25`) fragments, time out missing fragments, mark stale tiles
- [ ] `image_pipeline/canvas.py` — persistent tile canvas; replace changed tiles in place; on `base_seq` mismatch send `CMD_REQ_KEYFRAME` (opcode `0x62`); **attaches the badge enum to every published tile**
- [ ] `image_pipeline/accel_select.py` — auto-detect Coral at startup; export `HAS_CORAL` for downstream stages; expose status to the UI
- [ ] **`firmware/tractor_x8/image_pipeline/register.py`** — phase-correlation pre-diff image registration (NEON-accelerated, ~5 % CPU). **Non-negotiable** per [IMAGE_PIPELINE.md week 2](IMAGE_PIPELINE.md): without this, byte-savings collapse the moment the tractor moves
- [ ] **`encode_tile_delta.py --y-only` + base-side `image_pipeline/recolourise.py`** — scheme Z (Y-only luma + 30 s colour reference + base recolouriser, `Recolourised` badge). Inserted at week 2.5 per [IMAGE_PIPELINE.md §8](IMAGE_PIPELINE.md)
- [ ] **`image_pipeline/link_monitor.py`** — rolling 10 s `bytes/refresh`; emits `CMD_ENCODE_MODE` per the auto-fallback ladder (`full ≥400 B`, `y_only 150–400 B`, `motion_only 50–150 B`, `wireframe <50 B`)
- [ ] **`image_pipeline/bg_cache.py`** — rolling per-tile median keyed on segmenter class output; fills missed tiles with `Cached` badge + age (the only hole-filler available without an AI inpainter)
- [ ] **`image_pipeline/state_publisher.py`** — WebSocket publisher: canvas tiles + per-tile age + per-tile badge enum + detection vectors + safety verdicts + accelerator status. **All authoritative state lives here, not in the browser**
- [ ] **`image_pipeline/fallback_render.py`** — server-side 1 fps render of the canvas, for HDMI console + headless QA (kept alive even when browser is the primary surface)

**Phase 5A.B — Browser-tier offload (mandatory in Phase 1, not deferred — per [IMAGE_PIPELINE.md §6](IMAGE_PIPELINE.md)):**

Lives in `base_station/web_ui/static/img/`. Capability floor = WebGL 2 + Canvas 2D; opportunistic WebGPU with WebGL fallback; defer WebNN to v26.

- [ ] `canvas_renderer.js` — WebSocket subscriber; per-tile blits to Canvas 2D in an OffscreenCanvas Worker
- [ ] `fade_shader.js` — WebGL 2 fragment shader for per-tile cross-fade (~3 display frames, target 60 fps local)
- [ ] `staleness_overlay.js` — yellow tint + age-in-seconds rendering (consumes `age_ms` from base, never computes locally)
- [ ] `badge_renderer.js` — reads badge enum per tile; **fail-closed if missing/malformed** (refusal-to-display logged to base health endpoint)
- [ ] `detection_overlay.js` — bounding-box rendering from `state_publisher` detection vectors
- [ ] `accel_status.js` — "AI accelerator: online / offline / degraded" pill, always visible
- [ ] `raw_mode_toggle.js` — one-click toggle to most-recent-bytes-only view; choice logged to base audit endpoint
- [ ] **Trust-boundary documentation in [BASE_STATION.md](BASE_STATION.md)** per [IMAGE_PIPELINE.md §6.1](IMAGE_PIPELINE.md): table of what stays browser-only (display polish) vs what never goes to the browser (safety detector, ROI generation, badge decisions, anything autopilot might consume)
- [ ] **Browser test matrix gate** (Phase-1 completion gate): Latest Chrome on a $200 Android tablet, latest Safari on a 2020 iPhone SE, latest Firefox on a Linux laptop, latest Chrome on a Windows laptop — all four must render canvas + per-tile fade + staleness + badges + raw-mode toggle correctly

**Phase 5B — Base-side AI (CPU first, Coral if available):**

- [ ] `image_pipeline/superres_cpu.py` — Real-ESRGAN-General-x4v3 via [ncnn](https://github.com/Tencent/ncnn) on the A53 cores at **0.5–1 fps** (not 30 fps); benchmark gate ≤300 ms/frame; sets `Enhanced` badge
- [ ] `image_pipeline/superres_coral.py` — Edge-TPU port of Real-ESRGAN; only used iff `HAS_CORAL` and the spike (Phase 0) passed; benchmark gate ≤30 ms/frame
- [ ] `image_pipeline/detect_yolo.py` — **independent base-side safety detector (R6, two-detector pattern)**; CPU path = YOLOv8-nano OR NanoDet-Plus per AGPL decision, Coral path = YOLOv8-medium. **Two-detector disagreement banner in UI** when tractor `0x26` and base detectors disagree on a high-confidence object; log to §8.10 logger for v26 retraining
  - [ ] **OPEN SCOPE DECISION O1 — AGPL stance** on Ultralytics YOLOv8 (AGPL-3.0) vs. NanoDet-Plus (Apache-2.0). **Deadline: before week 6.** Default if undecided: NanoDet-Plus, accept ~5 % accuracy reduction. See [IMAGE_PIPELINE.md §10](IMAGE_PIPELINE.md)
- [ ] **`image_pipeline/motion_replay.py`** — apply `0x28` motion vectors to existing canvas; sets `Predicted` badge (Q degraded mode)
- [ ] **`image_pipeline/wireframe_render.py`** — render `0x29` wireframe over canvas; sets `Wireframe` overlay (P extreme degraded mode)
- [ ] **Tractor-side `encode_motion.py`** — optical-flow microframe encoder for topic `0x28`
- [ ] **Tractor-side `encode_wireframe.py`** — PiDiNet edge encoder for topic `0x29`
- [ ] **OPEN SCOPE DECISION O2 — Coral on the v25 BOM** — order it for the spike, or skip entirely? **Deadline: before week 5** (Phase-0 spike must complete by then). Default if undecided: ship CPU-only Stack-NoCoral as primary; Coral added in v25.5 if a later spike succeeds. See [IMAGE_PIPELINE.md §10](IMAGE_PIPELINE.md)
- [ ] `image_pipeline/interp_rife.py` *(optional, Coral-only)* — RIFE frame interpolation between thumbnail arrivals; under `Enhanced` badge
- [ ] `image_pipeline/inpaint_lama.py` *(optional, Coral-only)* — LaMa-Fourier fill of stale tiles; under `Synthetic` badge; opt-in only

**Phase 5C — Operator UX safety rules (mandatory before live hydraulic test):**

- [ ] Staleness clock visible on every displayed canvas, always
- [ ] "Enhanced" / "Synthetic" badge on any non-1:1 pixel (super-resolved, RIFE-interpolated, LaMa-inpainted, AI-colorized)
- [ ] Never hide loss — partial / corrupt canvas shown with gaps visible, never extrapolated and hidden
- [ ] One-click "raw mode" toggle — most-recent received bytes only, no enhancement
- [ ] Audit log: which view-mode (raw / enhanced) the operator was using when each command was issued; persisted to the §8.10 black-box logger
- [ ] **"AI accelerator: online / offline / degraded"** indicator visible on the operator console at all times

### Tractor image pipeline (per [TRACTOR_NODE.md § Image pipeline](TRACTOR_NODE.md#image-pipeline-portenta-x8-linux-side)) — *no Coral on tractor*

- [ ] `firmware/tractor_x8/image_pipeline/capture.py` — V4L2 → 384×256 YCbCr buffer per camera
- [ ] `tile_diff.py` — pHash-based 32×32 tile-change detector, NEON-accelerated, ≤30 ms for 96 tiles
- [ ] `roi.py` — read valve activity from H747 over IPC, classify mode (loading / driving / idle), produce ROI mask; honour `CMD_ROI_HINT` (opcode `0x61`)
- [ ] `detect_nanodet.py` — [NanoDet-Plus](https://github.com/RangiLyu/nanodet) (Apache-2.0) at 320×320 INT8, six classes; benchmark gate ≤50 ms p99 on the X8 A53s
- [ ] `encode_tile_delta.py` — per-tile WebP at q15/q40/q60 by ROI/detection; assemble `TileDeltaFrame` body
- [ ] `fragment.py` — split into ≤25 ms airtime fragments
- [ ] `ipc_to_h747.py` — hand fragments to the M7 firmware ring buffer at P3
- [ ] **`CMD_PERSON_APPEARED` (opcode `0x60`)** — on first new high-confidence person/animal/vehicle in frame, emit P0 alert with normalised bbox centroid; promote next image transmission to P1 for one frame
- [ ] **Multi-camera attention multiplexing** — front=70 / bucket=25 / rear=5 default; reverse-stick-driven flip; person-detection-driven 90 % promotion
- [ ] **Logger requirement** — every captured canvas + detection set + operator command + active-view-mode appended to the §8.10 black-box logger (builds the v26 fine-tuning dataset)

**Phase 5D — Validation gates (must all pass before field test, per [IMAGE_PIPELINE.md §9](IMAGE_PIPELINE.md)):**

- [ ] **V1.** Image-pipeline P0 starvation gate: 30-min mixed-mode stress run, **zero P0 ControlFrame TX-start delays >25 ms attributable to image fragments**
- [ ] **V2.** End-to-end image latency: capture → base UI repaint, ≤500 ms p99 CPU-only, ≤300 ms p99 with Coral
- [ ] **V3.** `CMD_PERSON_APPEARED` end-to-end: walk a person across the FOV, alert reaches base UI in ≤250 ms p99
- [ ] **V4.** `CMD_REQ_KEYFRAME` recovery: induce I-frame loss, confirm base detects mismatch and tractor returns a fresh I within 1 refresh
- [ ] **V5.** Coral fallback: yank Coral mid-operation, confirm UI flips to "AI accelerator: offline" within 10 s and pipeline continues degraded
- [ ] **V6.** **Auto-fallback ladder validation** — attenuate the LoRa link in 50 B/s steps; verify the encoder downshifts cleanly through `full → y_only → motion_only → wireframe` without operator intervention and without losing the canvas
- [ ] **V7.** **Browser test matrix** — all four target browsers (Chrome/Android, Safari/iOS, Firefox/Linux, Chrome/Windows) render canvas + fade + badges + raw-mode toggle correctly
- [ ] **V8.** **Two-detector cross-check** — tractor `0x26` vs. base `detect_yolo.py` disagreements surface in UI within one refresh; logged to §8.10 black-box logger
- [ ] **V9.** Operator-UX safety rules (Phase 5C) all visible and functional — pre-condition for any live hydraulic test
- [ ] **V10.** **Trust-boundary fail-closed** — patch a tile in transit to remove its badge enum; browser **must refuse to display** the tile and log the refusal to the base health endpoint

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
- [ ] Open-source the firmware under GPLv3 (matching Meshtastic / RadioLib community norms)
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
  - [ ] **Phase E — Multi-camera arbitration on the tractor X8:** support front + rear + implement cameras simultaneously (single USB root hub); only the *selected* camera produces LoRa thumbnails. Implement `CMD_CAMERA_SELECT` (opcode 0x03 in [LORA_PROTOCOL.md](LORA_PROTOCOL.md#command-frame-opcodes)); auto-flip to rear camera when reverse stick is held >50% for >1 s (decision made on tractor X8, not base, to avoid round-trip latency); echo active camera back on telemetry topic 0x22. Add the second camera (Kurokesu C2 or C1 PRO board + 3.6 mm M12) on the rear ROPS. Mount cabin cam in the cab through windshield, or add Wiegmann WA-series NEMA 4X enclosure for external mounts (see HARDWARE_BOM.md camera-path notes).
  - [ ] **Phase F — Crop-health onboard analysis (the killer feature):** RGB-only ExG / canopy-cover proxy on the *existing* front camera (zero hardware adder, ships with v25 first light); 30 B/min summary on topic 0x24. Phase F.1: add MAPIR Survey3W NDVI/OCN or dual NoIR for true NDVI; X8 NEON computes per-row NDVI + percent-canopy-cover, geotagged from `topic 0x01` GPS + IMU heading; raw frames cached to microSD for WiFi-when-parked retrieval. Base-station `/map` view overlays heatmap. See [VIDEO_OPTIONS.md § Crop-health analysis](VIDEO_OPTIONS.md#crop-health-analysis).
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
- [RESEARCH-CONTROLLER/WIRELESS_OPTIONS.md](RESEARCH-CONTROLLER/WIRELESS_OPTIONS.md) *(archived)* — historical comparison of wireless technologies considered before LoRa was selected
- [VIDEO_OPTIONS.md](VIDEO_OPTIONS.md) — video streaming options
- [RESEARCH-CONTROLLER/](RESEARCH-CONTROLLER/) — earlier prototypes and research docs
- [LifeTrac-v25/TODO.md](../TODO.md) — top-level v25 TODO list
