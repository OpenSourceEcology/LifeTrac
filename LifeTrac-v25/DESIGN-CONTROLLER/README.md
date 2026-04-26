# LifeTrac v25 — Controller Design

This folder contains the **primary controller design** for the LifeTrac v25 tractor: a three-tier wireless control system built on Arduino Pro hardware.

## The system at a glance

```
┌──────────────────────┐                          ┌──────────────────────────┐
│  Handheld Remote     │  LoRa 915 MHz            │  Tractor                 │
│  Arduino MKR         │ ◄────────────────────►   │  Portenta Max Carrier    │
│  WAN 1310            │     proximity link        │  + Portenta H7           │
└──────────────────────┘                          │  (drives hydraulic       │
                                                   │   valves)                │
                                                   └────────┬─────────────────┘
                                                            │ LoRa 915 MHz
                                                            │ (long range,
                                                            │  mast antenna)
                                                            │ + Cellular backup
                                                            ▼
                          ┌────────────────────────────────────────────────┐
                          │  Base Station                                  │
                          │  Portenta Max Carrier + Portenta X8            │
                          │  • Linux, Mosquitto MQTT, nginx web UI         │
                          │  • Gigabit Ethernet → office LAN               │
                          │  • Laptop browser → joysticks, telemetry, map  │
                          └────────────────────────────────────────────────┘
```

## Design documents (read in this order)

1. **[ARCHITECTURE.md](ARCHITECTURE.md)** — system overview, radios, links, control-source arbitration, failsafes
2. **[HARDWARE_BOM.md](HARDWARE_BOM.md)** — full bill of materials with part numbers and prices
3. **[LORA_PROTOCOL.md](LORA_PROTOCOL.md)** — frame format, priority arbitration, MQTT-SN telemetry, security
4. **[TRACTOR_NODE.md](TRACTOR_NODE.md)** — Portenta Max Carrier + H7 on the tractor: wiring, firmware structure, hydraulic interface
5. **[BASE_STATION.md](BASE_STATION.md)** — Portenta Max Carrier + X8 base station: web UI, MQTT broker, mast antenna setup
6. **[HANDHELD_REMOTE.md](HANDHELD_REMOTE.md)** — MKR WAN 1310 handheld: joysticks, E-stop, battery, enclosure
7. **[TODO.md](TODO.md)** — full development roadmap (hardware purchases, firmware, web UI, field testing)

## Cross-cutting reference docs

- **[WIRELESS_OPTIONS.md](WIRELESS_OPTIONS.md)** — comparison of all wireless technologies considered
- **[VIDEO_OPTIONS.md](VIDEO_OPTIONS.md)** — wireless video streaming analysis (WiFi, cellular, LoRa thumbnails)
- **[ARDUINO_CI.md](ARDUINO_CI.md)** — Arduino-CLI continuous integration setup
- **[arduino_libraries.txt](arduino_libraries.txt)** — pinned library versions

## Configuration & test infrastructure

- **[config/](config/)** — Mosquitto broker config, WiFi setup notes
- **[test_scripts/](test_scripts/)** — MQTT test scripts and bench-test utilities

## Earlier design exploration

The **[RESEARCH-CONTROLLER/](RESEARCH-CONTROLLER/)** subfolder archives the earlier prototype designs (Arduino Opta + ESP32 BLE remote + Raspberry Pi web UI, plus SparkFun LoRa stack research). Those designs informed the choices made here and are preserved for reference, code reuse, and as a fallback path.

## Why this hardware combination?

A full justification is in [ARCHITECTURE.md § Hardware rationale](ARCHITECTURE.md#hardware-rationale). The short version:

- **Tractor needs an industrial-grade carrier with multiple radios + Linux-capable host** → Portenta Max Carrier + H7 gives LoRa, Cat-M1 cellular, Gigabit Ethernet, CAN, RS-485, MIPI camera, and rugged power input (6–36 V DC) on one board.
- **Base station needs a web server, broker, and long-range RF** → second Max Carrier + Portenta X8 (Linux) hosts nginx + Mosquitto natively, drives a high-gain mast antenna over SMA, and connects to the office LAN over Gigabit.
- **Handheld needs to be cheap, battery-powered, and use the same LoRa silicon as the tractor** → MKR WAN 1310 has the *exact same Murata CMWX1ZZABZ-078 SiP* as the Max Carriers, so RadioLib firmware ports verbatim, at $45 per unit.
