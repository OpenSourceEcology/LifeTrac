# Tractor Node — Portenta Max Carrier + Portenta X8 + Arduino Opta

> **Scope (2026-04-26):** LoRa-only per [MASTER_PLAN.md](MASTER_PLAN.md). The Cat-M1 / SARA-R412M cellular references below are **archived** — do not populate the cellular Mini-PCIe slot or activate a SIM for v25. Tractor→operator telemetry travels solely over LoRa.

The tractor-side controller. Receives commands from the Handheld (proximity) and Base Station (long range), drives the hydraulic valves, and streams telemetry back.

The tractor uses a **two-MCU split**:

- **Max Carrier + Portenta X8** owns the radios (LoRa + cellular), arbitration, telemetry, logging, and engine/CAN integration. It is the *brain*. The X8 contains a full **STM32H747 co-MCU** (M7 + M4) running the real-time firmware bare-metal; the **Linux side** (Yocto on i.MX 8M Mini) runs SSH for field debug, log rotation, optional containerized telemetry recorder, and the future MIPI camera path.
- **Arduino Opta + expansions** owns the valve coils, the 0–10 V flow valve drive, the safety-chain interlocks, and per-channel current monitoring. It is the *industrial I/O layer*.
- The two boards talk to each other over **Modbus RTU on RS-485** (Max Carrier J6 ↔ Opta RS-485 port).

This split exists for safety: a crash, hang, or firmware bug in either MCU fails into a known-safe state independently. The Opta has its own watchdog on the "alive" register that drops all coils if the Max Carrier stops writing for >200 ms; the Max Carrier H747 co-MCU has its own watchdog on radio liveness that triggers the Opta to neutral via the same path. **Linux can crash or be rebooted without affecting valve safety** — the H747 co-MCU continues running independently.

## Hardware overview

| Component | Role |
|---|---|
| Portenta Max Carrier (ABX00043) | Carrier with onboard LoRa (Murata SX1276) + Cat-M1 cellular (SARA-R412M), both SMA |
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
| LoRa whip + cellular antenna | Both SMA; cab-roof bracket with N-bulkhead pass-through |
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
        │       │     └──► H747 co-MCU (M7+M4): radios + arbitration + Modbus master
        │       ├──► LoRa Murata SiP    → SMA J9 → cab-roof whip antenna
        │       ├──► SARA-R412M cellular → SMA J3 → cab-roof cellular antenna
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

### M7 core (`tractor.ino`)

Responsibilities:
- LoRa TX/RX via RadioLib (SX1276)
- Cellular fallback (SARA-R412M via MKRNB library)
- Multi-source arbitration (see [LORA_PROTOCOL.md § Multi-source arbitration](LORA_PROTOCOL.md#multi-source-arbitration))
- Telemetry packing → MQTT-SN over LoRa to base station
- Modbus RTU master → DRV8908 valve driver
- microSD logging
- M4 IPC: pushes the active ControlFrame to M4 every 50 ms via shared memory

Main loop pseudocode:

```c
void setup() {
    Serial.begin(115200);
    LoRa.begin(915E6);
    LoRa.setSpreadingFactor(7);
    LoRa.setSignalBandwidth(500E3);
    crypto_init(PRESHARED_KEY);
    
    Modbus.begin(115200);  // RS-485 to valve driver
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
    
    // 4. Cellular MQTT keep-alive
    cellular_tick();
}
```

### M4 core (`tractor_m4.cpp`)

Responsibilities:
- 100 Hz deterministic loop
- Read latest ControlFrame from shared memory
- Apply axis values → valve PWM (or on/off)
- Watchdog: if no fresh ControlFrame in 200 ms → all valves neutral
- Write valve commands to DRV8908 over local SPI/I²C

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
Base station E-stop button (web UI → cellular → tractor)
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
| Cellular Cat-M1 (TX burst) | 0.6 W | 2 W |
| GPS + sensors | 0.5 W | 0.5 W |
| **Total electronics** | **~2 W** | **~4 W** |
| Solenoid valves (when energized) | 60–120 W | 240 W (all 8 on, transient) |

The Max Carrier accepts 6–36 V; tractor 12 V system is fine. The 18650 backup gives ~30 min of MCU operation through engine-crank brown-outs.

## Mounting & environmental

- Mount the IP65 enclosure inside the cab on a vibration-isolated bracket (rubber grommets)
- LoRa whip antenna outside the cab, vertical, ground-plane on the cab roof
- Cellular antenna mounted on the cab roof or rear ROPS
- Cable runs through cable glands; avoid running RF coax parallel to ignition wiring
- Operating temperature: -20 °C to +60 °C (Portenta industrial spec)
- Vibration: secure with Loctite on screw threads; service inspection every 250 hours

## Bring-up checklist

1. [ ] Bench-test Max Carrier + H7 with USB-C power, blink LED on M7 and M4
2. [ ] Verify LoRa TX/RX with a second Max Carrier (or any RadioLib-compatible board)
3. [ ] Verify cellular registers with the IoT SIM, sends test MQTT publish
4. [ ] Wire up DRV8908 valve driver, test each channel with bench LEDs as valve stand-ins
5. [ ] Wire up E-stop relay chain, test that loss of M7 watchdog drops valve power within 200 ms
6. [ ] Install in tractor, verify GPS lock, CAN read from engine ECU
7. [ ] Field-test hydraulic actuation with all valves wet (engine off, hydraulic pump motor on)
8. [ ] Field-test with handheld at close range, verify no missed packets at SF7/BW500
9. [ ] Field-test with base station at 1 km, 5 km, 10 km LoS — record RSSI and packet loss
10. [ ] 24-hour soak test in shop with simulated joystick input

## Source code layout (when implemented)

```
firmware/tractor_h7/
├── tractor.ino                 # M7 main; setup() and loop()
├── tractor_m4.cpp              # M4 100 Hz valve loop
├── modbus_valves.cpp           # RS-485 master to DRV8908
├── cellular_mqtt.cpp           # SARA-R412M + MQTT publish over Cat-M1
├── telemetry.cpp               # Topic builders (GPS, engine, battery, etc.)
├── ipc.cpp                     # M7 ↔ M4 shared memory
├── pinmap_max_carrier_h7.h     # Pin definitions for this hardware combo
└── platformio.ini              # PlatformIO build for both cores
```

## See also

- [HARDWARE_BOM.md § Tier 1](HARDWARE_BOM.md#tier-1--tractor-node-portenta-max-carrier--portenta-h7)
- [LORA_PROTOCOL.md](LORA_PROTOCOL.md) — air-interface details
- [BASE_STATION.md](BASE_STATION.md) — what the tractor talks to over long-range LoRa
- [HANDHELD_REMOTE.md](HANDHELD_REMOTE.md) — what the tractor talks to over short-range LoRa
- [DESIGN-HYDRAULIC/](../DESIGN-HYDRAULIC/) — hydraulic schematic (which valves do what)
- [RESEARCH-CONTROLLER/arduino_opta_controller/](RESEARCH-CONTROLLER/arduino_opta_controller/) — superseded Opta-based design (good reference for valve-control logic)
