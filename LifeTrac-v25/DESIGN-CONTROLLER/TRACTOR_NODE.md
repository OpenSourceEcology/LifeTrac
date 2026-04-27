# Tractor Node — Portenta Max Carrier + Portenta X8 + Arduino Opta

> **Scope (2026-04-26):** LoRa-only per [MASTER_PLAN.md](MASTER_PLAN.md). The Cat-M1 / SARA-R412M cellular references below are **archived** — do not populate the cellular Mini-PCIe slot or activate a SIM for v25. Tractor→operator telemetry travels solely over LoRa.

The tractor-side controller. Receives commands from the Handheld (proximity) and Base Station (long range), drives the hydraulic valves, and streams telemetry back.

The tractor uses a **two-MCU split**:

- **Max Carrier + Portenta X8** owns LoRa, arbitration, telemetry, logging, and engine/CAN integration. It is the *brain*. The X8 contains a full **STM32H747 co-MCU** (M7 + M4) running the real-time firmware bare-metal; the **Linux side** (Yocto on i.MX 8M Mini) runs SSH for field debug, log rotation, optional containerized telemetry recorder, sensors, and video services.
- **Arduino Opta + expansions** owns the valve coils, the 0–10 V flow valve drive, the safety-chain interlocks, and per-channel current monitoring. It is the *industrial I/O layer*.
- The two boards talk to each other over **Modbus RTU on RS-485** (Max Carrier J6 ↔ Opta RS-485 port).

This split exists for safety: a crash, hang, or firmware bug in either MCU fails into a known-safe state independently. The Opta has its own watchdog on the "alive" register that drops all coils if the Max Carrier stops writing for >200 ms; the Max Carrier H747 co-MCU has its own watchdog on radio liveness that triggers the Opta to neutral via the same path. **Linux can crash or be rebooted without affecting valve safety** — the H747 co-MCU continues running independently.

## Hardware overview

| Component | Role |
|---|---|
| Portenta Max Carrier (ABX00043) | Carrier with onboard LoRa (Murata SX1276), SMA RF, RS-485, CAN, and disabled/not-used Cat-M1 hardware present on the board |
| **Portenta X8 (ABX00049)** | i.MX 8M Mini quad Cortex-A53 @ 1.8 GHz running Yocto Linux + Docker, **plus STM32H747 co-MCU** (M7 480 MHz + M4 240 MHz). The H747 co-MCU runs the real-time M7+M4 firmware bare-metal; Linux runs on top for SSH/logs/containers. Canonical per [MASTER_PLAN.md §8.9](MASTER_PLAN.md). |
| **Arduino Opta WiFi** (AFX00002) | Industrial PLC with 4× 10 A relays, 8 digital/analog inputs, RS-485, Ethernet. Modbus-RTU **slave** to the Max Carrier. Carries 4 of the 8 directional valve coils + watchdog logic. **WiFi and BLE radios are disabled in firmware** — the Opta is used solely for hydraulic-system I/O over Modbus-RTU/RS-485; no wireless paths to or from the Opta are in scope (see [MASTER_PLAN.md](MASTER_PLAN.md)) |
| **Opta Ext D1608S** expansion (AFX00006) | **8× solid-state relays (SSR), 24 VDC / 2 A each** + 16× programmable voltage inputs (digital 0–24 VDC or analog 0–10 V). Carries the other 4 directional valve coils on SSR channels for low-latency switching (sub-millisecond, no contact bounce). 4 spare SSR channels for aux outputs. The slower onboard EMRs on the Opta base remain available for the engine-kill chain (which is gated by the PSR safety relay anyway, not latency-critical). 16 inputs cover ignition sense / mode switch / external limit switches |
| **Opta Ext A0602** expansion | 6× analog inputs + **2× 0–10 V analog outputs**. Drives the **Burkert 8605 flow valve controller** — supports the dual-flow-valve config from [DESIGN-HYDRAULIC/FLOW_VALVE_CONFIGURATION.md](../DESIGN-HYDRAULIC/FLOW_VALVE_CONFIGURATION.md) natively. 6 analog inputs cover battery V / oil P / coolant T / hyd P / engine T / spare with built-in conditioning |
| Phoenix Contact PSR safety relay (dual-channel) | Hardware E-stop chain; gates valve power rail + engine kill, independent of both MCUs |
| Inline ATC blade fuses (5 A × 8 + 20 A main) | Per-channel + main valve rail protection |
| EMI / ferrite filter on 12 V input | Tractor electrical is electrically noisy |
| Master battery cutoff switch | Shop-maintenance disconnect |
| 18650 LiPo + TP4056 charger | Survives engine-crank brown-outs; supports graceful shutdown |
| microSD 32 GB | Local data logging on Max Carrier |
| LoRa whip antenna | Cab-roof bracket with N-bulkhead pass-through |
| GPS active patch antenna (28 dB LNA) | For onboard u-blox GPS on Max Carrier |
| Status LEDs (POWER / LINK / FAULT) + piezo buzzer | Exterior indicators / audible alerts |
| Deutsch DT connector harness | Weatherproof harness terminations |
| DIN rail (35 mm) inside enclosure | Mounts Opta + expansions |
| IP65 enclosure ~250×200×100 mm + rubber bobbin mounts + conformal coat | Vibration-isolated; mounted in cab or behind ROPS |

Full BOM in [HARDWARE_BOM.md § Tier 1](HARDWARE_BOM.md#tier-1--tractor-node-portenta-max-carrier--portenta-h7).

## Why Opta as the valve controller

We considered a discrete approach (8-ch MOSFET board + MCP4725 DAC + opto-isolator + signal-conditioning op-amps + custom fuse panel) which would save ~$260 in parts. The Opta path was chosen instead because:

1. **Two-MCU safety partition.** The radio MCU and the valve MCU can be debugged, restarted, and crash independently. A bug in the LoRa stack can never directly drive a valve coil — it can only write a register that the Opta then validates and acts on (or ignores, on watchdog timeout).
2. **Pre-certified industrial I/O.** Opta + expansions are CE / UL 61010 rated with built-in flyback protection, fusing, opto-isolation on every input, and field-rated screw terminals. The discrete path requires us to design and validate all of that.
3. **Dual-flow-valve config supported natively.** A0602 has *two* 0–10 V outputs; the discrete MCP4725 path would need a second DAC + op-amp added to support the dual-flow configuration documented in [FLOW_VALVE_CONFIGURATION.md](../DESIGN-HYDRAULIC/FLOW_VALVE_CONFIGURATION.md).
4. **Massive code reuse.** The valve sequencing, deadband logic, and safety state machine in [RESEARCH-CONTROLLER/arduino_opta_controller/](RESEARCH-CONTROLLER/arduino_opta_controller/) port directly — we strip out the MQTT/BLE control surfaces and replace them with a Modbus slave shim.
5. **Field serviceability.** Pop a failed expansion off the DIN rail and clip on a spare in 30 seconds, no soldering. Critical for a machine that's expected to run far from a workbench.

The +$260 vs discrete is small relative to the $20 k+ machine and the engineering hours saved.

## Wiring overview

```
Tractor 12 V battery
        │
        ├──► Master cutoff switch
        │
        ├──► EMI filter / ferrite
        │
        ├──► Main 20 A fuse
        │
        ├──► UPS-style backup (LiPo charger + 18650)
        │
        ├──► Max Carrier 6-36 V power jack
        │       │
        │       ├──► Portenta X8 (via high-density connectors)
        │       │     └──► Linux (Yocto): SSH, journald, optional Docker telemetry recorder
        │       │     └──► H747 co-MCU (M7+M4): LoRa + arbitration + Modbus master
        │       ├──► LoRa Murata SiP    → SMA J9 → cab-roof whip antenna
        │       ├──► RS-485 (J6) ◄══ Modbus RTU 115200 8N1 ══► Opta RS-485
        │       ├──► CAN-FD (J7) → engine ECU (read-only telemetry)
        │       ├──► UART → u-blox GPS (with active antenna)
        │       ├──► GPIO → status LEDs + piezo buzzer
        │       ├──► microSD (J11) → data logging
        │       └──► Gigabit Ethernet (J17) → optional shop debug LAN
        │
        └──► Opta WiFi 12-24 V power input
                │
                ├──► 4× onboard EMR (electromechanical relays, 10 A 250 VAC):
                │       R1: engine kill (through PSR safety relay)
                │       R2: beacon / horn relay
                │       R3: parking-brake release
                │       R4: spare (high-current AUX)
                │      *(EMR latency 8–20 ms is acceptable here — none of these are on the
                │       50 Hz control loop. Directional valve coils moved to D1608S SSR
                │       per MASTER_PLAN §8.18 / RESEARCH-CONTROLLER/LATENCY_BUDGET.md.)*
                │
                ├──► Ext D1608S (DIN rail) — 8× SSR 24 VDC / 2 A:
                │       SSR1: drive LH forward
                │       SSR2: drive LH reverse
                │       SSR3: drive RH forward
                │       SSR4: drive RH reverse
                │       SSR5: boom up
                │       SSR6: boom down
                │       SSR7: bucket curl
                │       SSR8: bucket dump
                │      *(SSR pickup <1 ms, no contact bounce — removes the mechanical-relay
                │       latency term from the control hot path. Each coil paired with a
                │       1N4007-class flyback diode per industrial practice.)*
                │       16× programmable voltage inputs:
                │       I1:  ignition sense
                │       I2:  external E-stop loop monitor
                │       I3:  mode switch
                │       I4–I11: spare digital / analog (limit switches, etc.)
                │       I12: **FUTURE: BUMPER** — reserved for front/rear bumper kill-switch
                │             loop. Wired to a labelled screw block on the enclosure with
                │             a 4-conductor cable run to the front bumper bracket. Not
                │             populated in v25; future obstacle-stop hardware loops here
                │             *and* into the PSR safety chain (redundant). See
                │             RESEARCH-CONTROLLER/AUTOMATION_AND_ROUTE_PLANNING.md §10.
                │       I13–I16: spare digital / analog
                │
                └──► Ext A0602 (DIN rail):
                        AO1: 0-10 V → Burkert 8605 flow valve #1
                        AO2: 0-10 V → Burkert 8605 flow valve #2 (dual-flow config)
                        AI1: battery voltage (via on-board 0-30 V range)
                        AI2: hydraulic supply pressure transducer
                        AI3: hydraulic return pressure transducer
                        AI4: engine coolant temperature
                        AI5: hydraulic oil temperature
                        AI6: spare

Phoenix Contact PSR safety relay (independent of both MCUs)
        │
        ├──► Engine kill relay (Opta R9 + manual E-stop both must permit)
        └──► 12 V valve power rail (de-energizes all 8 directional coils + flow valve)
              ▲
              │
        Latching mushroom E-stop (handheld AND tractor-mounted)
        + Opta watchdog "alive" output (loss → relay drops out within 200 ms)
        + Max Carrier watchdog "alive" via Opta Modbus register (loss → Opta drops watchdog out)
```

## Modbus RTU register map (Max Carrier ↔ Opta)

The two MCUs communicate over RS-485 at 115200 baud, 8N1, Modbus RTU. The Max Carrier H7 is the **master**; the Opta is the **slave** at address 0x01.

### Holding registers (Max Carrier writes, Opta reads)

| Address | Name | Type | Description |
|---:|---|---|---|
| 0x0000 | `valve_coils` | uint16 bitfield | Bits 0-7 = drive LF, drive LR, drive RF, drive RR, boom U, boom D, bkt C, bkt D |
| 0x0001 | `flow_setpoint_1` | uint16 (0..10000) | Maps to 0–10 V on A0602 AO1 |
| 0x0002 | `flow_setpoint_2` | uint16 (0..10000) | Maps to 0–10 V on A0602 AO2 (dual-flow only) |
| 0x0003 | `aux_outputs` | uint16 bitfield | Bits 0-3 = spare AUX relays R10-R12 + engine-kill arm |
| 0x0004 | `watchdog_counter` | uint16 | Must increment ≥10 Hz; Opta drops all coils + flow on stall |
| 0x0005 | `command_source` | uint16 enum | 0=NONE, 1=HANDHELD, 2=BASE, 3=AUTONOMY (logging only) |
| 0x0006 | `arm_engine_kill` | uint16 | Non-zero = energize engine-kill solenoid (E-stop active) |

### Input registers (Opta writes, Max Carrier reads)

| Address | Name | Type | Description |
|---:|---|---|---|
| 0x0100 | `safety_state` | uint16 enum | 0=NORMAL, 1=WATCHDOG_TRIPPED, 2=ESTOP_LATCHED, 3=IGNITION_OFF |
| 0x0101 | `digital_inputs` | uint16 bitfield | I1-I16 raw digital input state |
| 0x0102 | `battery_mv` | uint16 | Battery voltage in mV (raw from AI1) |
| 0x0103 | `hyd_supply_psi` | uint16 | Hydraulic supply pressure (raw from AI2, scaled) |
| 0x0104 | `hyd_return_psi` | uint16 | Hydraulic return pressure (raw from AI3) |
| 0x0105 | `coolant_c` | int16 | Engine coolant temperature, °C (from AI4) |
| 0x0106 | `oil_c` | int16 | Hydraulic oil temperature, °C (from AI5) |
| 0x0107 | `aux_analog` | uint16 | Spare analog input (AI6) |
| 0x0108 | `relay_fault_flags` | uint16 bitfield | Per-coil fault from Opta self-diag |
| 0x0109 | `opta_uptime_s` | uint32 (2 regs) | Opta uptime since boot |
| 0x010B | `opta_fw_version` | uint16 | Opta firmware version BCD (e.g. 0x0102 = v1.02) |

### Communication cycle

- **Control write block** (`0x0000`–`0x0006`, 7 registers): Max Carrier writes at **50 Hz** (every 20 ms) using `WriteMultipleRegisters` (function 0x10)
- **Telemetry read block** (`0x0100`–`0x010B`, 12 registers): Max Carrier reads at **10 Hz** (every 100 ms) using `ReadInputRegisters` (function 0x04)
- **Round-trip budget**: at 115200 baud, ~1.5 ms per 7-register write, ~2 ms per 12-register read; total bus utilization ~10%, leaves headroom for retries

### Watchdog behavior

The Opta independently monitors `watchdog_counter` (register 0x0004). If the counter does not change for **200 ms**, the Opta:

1. De-energizes all 8 directional valve coils
2. Sets both flow setpoints to 0
3. Drops the hardware "alive" output to the PSR safety relay
4. Sets `safety_state` to `WATCHDOG_TRIPPED`
5. Refuses new commands until `watchdog_counter` advances by ≥10 within 1 second (proves master is stable)

This is independent of any LoRa-side timeout — the LoRa watchdog and the Modbus watchdog both have to be healthy for valves to move.

## Firmware structure

### M7 core (`firmware/tractor_h7/tractor_m7.ino`)

Responsibilities:
- LoRa TX/RX via RadioLib (SX1276)
- Multi-source arbitration (see [LORA_PROTOCOL.md § Multi-source arbitration](LORA_PROTOCOL.md#multi-source-arbitration))
- Telemetry packing → MQTT-SN over LoRa to base station
- Modbus RTU master → Opta Modbus-RTU slave
- microSD logging
- M4 IPC: pushes the active ControlFrame to M4 every 50 ms via shared memory

Main loop pseudocode:

```c
void setup() {
    Serial.begin(115200);
    radio.begin(915.0, 125.0, 7, 5, 0x12, 20);  // SF7/BW125/CR4-5
    crypto_init(PRESHARED_KEY);
    
    Modbus.begin(115200);  // RS-485 to Opta slave
    sd.begin(BUILTIN_SDCARD);
    
    // Boot M4 core for valve control
    LL_RCC_ForceCM4Boot();
}

void loop() {
    // 1. Receive any pending LoRa frames
    while (LoRa.parsePacket()) {
        Frame f;
        if (lora_proto_decode(&f) == OK) {
            update_source_state(f.source_id, &f);
        }
    }
    
    // 2. Arbitration tick (50 ms)
    if (millis() - last_arb_ms >= 50) {
        active_source = pick_active_source();
        ipc_push_to_m4(active_source, &latest_control[active_source]);
        last_arb_ms = millis();
    }
    
    // 3. Telemetry tick (variable, per topic)
    telemetry_tick();
    
    // 4. Service watchdogs and non-blocking X8 sidecar IPC
    service_sidecar_ipc();
}
```

### M4 core (`tractor_m4.cpp`)

Responsibilities:
- 100 Hz deterministic loop
- Read latest ControlFrame from shared memory
- Apply axis values → valve PWM (or on/off)
- Watchdog: if no fresh ControlFrame in 200 ms → all valves neutral
- Keep the hardware watchdog independent of Linux and mirror the safe control state

```c
void m4_loop_100hz() {
    ControlFrame cf;
    if (!ipc_read_latest(&cf, /*max_age_ms=*/200)) {
        valves_to_neutral();
        return;
    }
    
    valve_set(VALVE_DRIVE_LH, axis_to_pwm(cf.axis_lh_y));
    valve_set(VALVE_DRIVE_RH, axis_to_pwm(cf.axis_rh_y));
    valve_set(VALVE_BOOM,     axis_to_pwm(cf.axis_rh_x));
    
    if (cf.buttons & BTN_BUCKET_CURL) valve_on(VALVE_BUCKET_CURL);
    else if (cf.buttons & BTN_BUCKET_DUMP) valve_on(VALVE_BUCKET_DUMP);
    else valve_off(VALVE_BUCKET);
}
```

## Telemetry sources

The M7 core gathers and publishes:

| Topic | Source | Cadence |
|---|---|---|
| `gps` | u-blox over UART (optional GPS module) | 5 Hz |
| `engine` | CAN-FD from engine ECU (RPM, coolant T, oil P) | 1 Hz |
| `battery` | Max Carrier ADC + LiPo charger status | 0.2 Hz |
| `hydraulics` | Pressure sensors via I²C (oil P, line P) | 2 Hz |
| `mode` | Active source ID, take-control state | on change |
| `errors` | Watchdog hits, CRC errors, comms drops | on event |

Published as MQTT-SN over LoRa to the base station, which forwards to its Mosquitto broker.

## Failsafe wiring (independent of MCU)

The hardware E-stop chain MUST function even if the H7 freezes:

```
Handheld E-stop signal (LoRa heartbeat with estop_armed=0)
   OR
Base station E-stop button (web UI → LoRa → tractor)
   OR
H7 watchdog timeout
   OR
Hardware E-stop relay (mounted on tractor exterior, momentary)
   │
   ▼
Latching safety relay (DPDT, Phoenix Contact PSR series)
   │
   ├── NC contact: drops 24 V valve coil rail → all valves to spring-return neutral
   └── NC contact: closes engine-stop solenoid
```

Recovery requires manual reset on the tractor (key-switch cycle or dedicated reset button).

## Power budget

| Load | Average | Peak |
|---|---:|---:|
| Portenta X8 + Max Carrier (idle, Linux running) | 2.5 W | 4 W |
| LoRa TX (+20 dBm, 50% duty) | 0.4 W | 0.8 W |
| GPS + sensors | 0.5 W | 0.5 W |
| **Total electronics** | **~3.4 W** | **~5.3 W** |
| Solenoid valves (when energized) | 60–120 W | 240 W (all 8 on, transient) |

The Max Carrier accepts 6–36 V; tractor 12 V system is fine. The 18650 backup gives ~30 min of MCU operation through engine-crank brown-outs.

## Mounting & environmental

- Mount the IP65 enclosure inside the cab on a vibration-isolated bracket (rubber grommets)
- LoRa whip antenna outside the cab, vertical, ground-plane on the cab roof
- Cable runs through cable glands; avoid running RF coax parallel to ignition wiring
- Operating temperature: -20 °C to +60 °C (Portenta industrial spec)
- Vibration: secure with Loctite on screw threads; service inspection every 250 hours

## Bring-up checklist

1. [ ] Bench-test Max Carrier + H7 with USB-C power, blink LED on M7 and M4
2. [ ] Verify LoRa TX/RX with a second Max Carrier (or any RadioLib-compatible board)
3. [ ] Verify Opta Modbus slave over RS-485 with bench LEDs as valve stand-ins
4. [ ] Verify all 8 D1608S SSR channels and both A0602 analog outputs drop safe on watchdog timeout
5. [ ] Wire up E-stop relay chain, test that loss of M7 watchdog drops valve power within 200 ms
6. [ ] Install in tractor, verify GPS lock, CAN read from engine ECU
7. [ ] Field-test hydraulic actuation with all valves wet (engine off, hydraulic pump motor on)
8. [ ] Field-test with handheld at close range, verify no missed packets at SF7/BW125
9. [ ] Field-test with base station at 1 km, 5 km, 10 km LoS — record RSSI and packet loss
10. [ ] 24-hour soak test in shop with simulated joystick input

## Source code layout

```
firmware/tractor_h7/
├── tractor_m7.ino              # M7 main: LoRa, source arbitration, Modbus master
└── tractor_m4.cpp              # M4 independent safety watchdog

firmware/tractor_opta/
└── opta_modbus_slave.ino       # Opta + D1608S + A0602 Modbus-RTU slave

firmware/tractor_x8/
├── gps_service.py              # GPS sidecar service
├── imu_service.py              # IMU sidecar service
└── requirements.txt
```

## Image pipeline (Portenta X8 Linux side)

Per [MASTER_PLAN.md §8.19](MASTER_PLAN.md) and [`../AI NOTES/2026-04-27_Image_Transmission_InDepth_Analysis_ClaudeOpus4_7_v1_0.md`](../AI%20NOTES/2026-04-27_Image_Transmission_InDepth_Analysis_ClaudeOpus4_7_v1_0.md). All inference on the tractor runs on the i.MX 8M Mini's 4× Cortex-A53 with NEON via TFLite-XNNPACK or ncnn — **no Coral on the tractor in v25** (power, thermal, enclosure constraints).

**Pipeline stages** (per camera, runs at the §8.19 budget on the X8 Linux side):

1. **Capture** — `/dev/videoN` (UVC) → 384×256 YCbCr 4:2:0 buffer.
2. **Lens-occlusion / over-exposure pre-check** — tiny (~50 KB) classifier on the H747 M7 co-MCU, reads a downsampled luma thumbnail over IPC. If occluded, the X8 emits a UI hint and skips the heavy pipeline for this refresh.
3. **Detection** — [NanoDet-Plus](https://github.com/RangiLyu/nanodet) (Apache-2.0, ~2.3 MB INT8) at 320×320, ~30 ms on A53. Six classes for v25: `person`, `animal`, `vehicle`, `large_obstacle`, `implement_attached`, `implement_detached`. Output → `0x26 video/detections` sidecar (P2). High-confidence new `person` → `CMD_PERSON_APPEARED` (opcode `0x60`, P0) — see [LORA_PROTOCOL.md § Command frame opcodes](LORA_PROTOCOL.md#command-frame-opcodes).
4. **Tile-diff** — pHash over 32×32 tiles vs the last sent canvas (~30 ms total for 96 tiles, NEON SIMD). Produces the `changed_bitmap` field of the `TileDeltaFrame`.
5. **ROI budget allocation** — read current valve-activity flags from the H747 M7 over IPC; classify operating mode (loading / driving / idle); per-tile WebP quality picks q60 (ROI), q40 (changed-non-ROI), or q15 (low). Bucket-tip area is *always* high quality regardless of mode. Operator-cued ROI from `CMD_ROI_HINT` (opcode `0x61`) overrides for `ttl_refreshes` frames.
6. **WebP encode** — libwebp on A53. Per-tile blobs assembled into the `TileDeltaFrame` body.
7. **Fragment & enqueue** — split into ≤25 ms airtime chunks, hand off to the H747 M7 firmware over the IPC ring buffer for transmission at P3 (image data) — *never* delays a P0 ControlFrame's TX-start.

**Mode selection.** Tractor decides I vs P locally each refresh per [LORA_PROTOCOL.md § TileDeltaFrame](LORA_PROTOCOL.md#tiledeltaframe-image-pipeline-i--p-frames). On receipt of `CMD_REQ_KEYFRAME` (opcode `0x62`) the next image is forced to I.

**Multi-camera attention multiplexing.** When 2+ UVC cameras are attached, default budget split is front=70 % / bucket=25 % / rear=5 %; reverse-stick deflection >50 % for >1 s flips to rear=70 %. A `person`-class detection in any camera promotes that camera to 90 % for the next 5 refreshes. Logic lives entirely on the X8 — base only sends `CMD_CAMERA_SELECT 0x00` for auto-by-mode.

**Audio (optional, future).** If a USB microphone is fitted, [YAMNet](https://github.com/tensorflow/models/tree/master/research/audioset/yamnet) (Apache-2.0, ~3.8 MB, ~50 ms on A53) classifies engine-knock / hydraulic-squeal / impact / voice; class IDs → topic `0x27 video/audio_event` (4 B per detection). Not required for v25 first build.

**Logger requirement.** Every captured canvas, every detection, every operator action and current view-mode is appended to the §8.10 black-box logger so a v26 fine-tuned NanoDet can train on real LifeTrac footage.

**Source layout (X8-side, when implemented):**

```
firmware/tractor_x8/image_pipeline/
├── capture.py            # V4L2 → numpy
├── tile_diff.py          # pHash + bitmap, NEON via numpy
├── roi.py                # valve-state → ROI mask
├── detect_nanodet.py     # NanoDet-Plus inference (TFLite-XNNPACK)
├── encode_tile_delta.py  # WebP per tile, assemble TileDeltaFrame
├── fragment.py           # ≤25 ms airtime fragmentation
├── ipc_to_h747.py        # Hand fragments to the M7 firmware over the IPC ring
└── models/               # NanoDet-Plus INT8, lens-occlusion, (optional) YAMNet
```

## Bring-up checklist (additions for image pipeline)

11. [ ] Capture from each Kurokesu / UVC camera at 384×256 ≥10 fps, no dropped frames over 5 minutes
12. [ ] NanoDet-Plus inference benchmark on the X8: confirm ≤50 ms p99 at 320×320 INT8
13. [ ] End-to-end image-pipeline benchmark: confirm a P-frame from capture to H747 IPC handoff in ≤200 ms p99
14. [ ] Validate the P0 starvation gate: 30-min mixed-mode stress test, confirm zero P0 ControlFrame TX-start delays >25 ms attributable to image fragments
15. [ ] `CMD_PERSON_APPEARED` end-to-end: walk a person across the camera FOV, confirm the alert reaches the base UI in ≤250 ms p99

## See also

- [HARDWARE_BOM.md § Tier 1](HARDWARE_BOM.md#tier-1--tractor-node-portenta-max-carrier--portenta-h7)
- [LORA_PROTOCOL.md](LORA_PROTOCOL.md) — air-interface details
- [BASE_STATION.md](BASE_STATION.md) — what the tractor talks to over long-range LoRa
- [HANDHELD_REMOTE.md](HANDHELD_REMOTE.md) — what the tractor talks to over short-range LoRa
- [DESIGN-HYDRAULIC/](../DESIGN-HYDRAULIC/) — hydraulic schematic (which valves do what)
- [RESEARCH-CONTROLLER/arduino_opta_controller/](RESEARCH-CONTROLLER/arduino_opta_controller/) — superseded Opta-based design (good reference for valve-control logic)
