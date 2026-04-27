# LifeTrac v25 — Controller Design

> **Scope (2026-04-26):** v25 wireless control is **LoRa-only** — see [MASTER_PLAN.md](MASTER_PLAN.md) for the canonical scope statement. WiFi/BLE/cellular paths from earlier design rounds have been moved to [RESEARCH-CONTROLLER/](RESEARCH-CONTROLLER/) and are out of scope for the v25 build. Where this document or its siblings still mention cellular/Cat-M1/MQTT-over-WiFi, treat those references as historical context to be cleaned up; the operator-browser ↔ base-station LAN/WiFi link is the only retained non-LoRa path.

This folder contains the **primary controller design** for the LifeTrac v25 tractor: a three-tier wireless control system built on Arduino Pro hardware.

## The system at a glance

```
┌──────────────────────┐                          ┌──────────────────────────┐
│  Handheld Remote     │  LoRa 915 MHz            │  Tractor                 │
│  Arduino MKR         │ ◄────────────────────►   │  Portenta Max Carrier    │
│  WAN 1310            │     proximity link        │  + Portenta X8           │
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

## Design documents (read in this order)

0. **[MASTER_PLAN.md](MASTER_PLAN.md)** — canonical scope: LoRa-only, Max Carrier on tractor + base, optional MKR WAN handheld. Read first.
1. **[ARCHITECTURE.md](ARCHITECTURE.md)** — system overview, radios, links, control-source arbitration, failsafes
2. **[HARDWARE_BOM.md](HARDWARE_BOM.md)** — full bill of materials with part numbers and prices
3. **[LORA_PROTOCOL.md](LORA_PROTOCOL.md)** — frame format, priority arbitration, MQTT-SN telemetry, security
4. **[TRACTOR_NODE.md](TRACTOR_NODE.md)** — Portenta Max Carrier + X8 (with Opta valve controller) on the tractor: wiring, firmware structure, hydraulic interface
5. **[BASE_STATION.md](BASE_STATION.md)** — Portenta Max Carrier + X8 base station: web UI, MQTT broker, mast antenna setup
6. **[HANDHELD_REMOTE.md](HANDHELD_REMOTE.md)** — MKR WAN 1310 handheld: joysticks, E-stop, battery, enclosure
7. **[TODO.md](TODO.md)** — full development roadmap (hardware purchases, firmware, web UI, field testing)

## Cross-cutting reference docs

- **[VIDEO_OPTIONS.md](VIDEO_OPTIONS.md)** — wireless video streaming analysis (LoRa thumbnails on the air; WiFi/cellular alternatives are archived)
- **[ARDUINO_CI.md](ARDUINO_CI.md)** — Arduino-CLI continuous integration setup
- **[arduino_libraries.txt](arduino_libraries.txt)** — pinned library versions
- **[RESEARCH-CONTROLLER/WIRELESS_OPTIONS.md](RESEARCH-CONTROLLER/WIRELESS_OPTIONS.md)** *(archived)* — historical comparison of wireless technologies considered before LoRa was selected

## Implementation tree

- **[firmware/](firmware/)** — handheld, tractor (M7/M4 + X8 services), and Opta-Modbus-slave firmware drafts
- **[base_station/](base_station/)** — base-station LoRa↔web bridge and operator web UI

## Earlier design exploration

The **[RESEARCH-CONTROLLER/](RESEARCH-CONTROLLER/)** subfolder archives the earlier prototype designs (Arduino Opta + ESP32 BLE remote + Raspberry Pi web UI, plus SparkFun LoRa stack research). Those designs informed the choices made here and are preserved for reference, code reuse, and as a fallback path.

## Why this hardware combination?

A full justification is in [ARCHITECTURE.md § Hardware rationale](ARCHITECTURE.md#hardware-rationale). The short version:

- **Tractor needs an industrial-grade carrier with multiple radios + Linux-capable host** → Portenta Max Carrier + X8 gives LoRa, Cat-M1 cellular, Gigabit Ethernet, CAN, RS-485, MIPI camera, Linux + Docker, and rugged power input (6–36 V DC) on one board. The X8's onboard STM32H747 co-MCU runs the deterministic real-time loop.
- **Base station needs a web server, broker, and long-range RF** → second Max Carrier + Portenta X8 (Linux) hosts nginx + Mosquitto natively, drives a high-gain mast antenna over SMA, and connects to the office LAN over Gigabit.
- **Handheld needs to be cheap, battery-powered, and use the same LoRa silicon as the tractor** → MKR WAN 1310 has the *exact same Murata CMWX1ZZABZ-078 SiP* as the Max Carriers, so RadioLib firmware ports verbatim, at $45 per unit.
