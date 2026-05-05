# Architecture — LifeTrac v25 Three-Tier Control System

> **Scope (2026-04-26):** LoRa-only per [MASTER_PLAN.md](MASTER_PLAN.md). Cellular (Cat-M1 / SARA-R412M) and "cellular fallback" mentions in this document are **archived** — the v25 build relies solely on the LoRa link for tractor↔base and tractor↔handheld. Cellular sections are retained as background pending a doc cleanup pass.

## Overview

LifeTrac v25 uses a **three-tier wireless control architecture** built on Arduino Pro hardware. All three tiers share the same Murata CMWX1ZZABZ-078 LoRa SiP (Semtech SX1276 + STM32L0), so a single firmware codebase serves all of them with only pin-map differences.

```
┌─────────────────────────┐                                  ┌─────────────────────────────┐
│  HANDHELD REMOTE        │                                  │  TRACTOR                    │
│  Arduino MKR WAN 1310   │   LoRa 915 MHz, +14 dBm whip     │  Portenta Max Carrier       │
│  ─────────────────────  │ ◄────────── proximity ──────────►│  + Portenta X8              │
│  • SAMD21 + Murata LoRa │   500 m – 2 km typical            │  ─────────────────────────  │
│  • Dual analog joysticks│                                   │  • Murata LoRa, +20 dBm SMA │
│  • Latching E-stop      │                                   │  • SARA-R412M Cat-M1 (SMA)  │
│  • LiPo + USB-C charge  │                                   │  • Gigabit Ethernet         │
│  • IP54 enclosure       │                                   │  • CAN, RS-485, mPCIe       │
└─────────────────────────┘                                   │  • MIPI camera connector    │
                                                              │  • H7 drives 8× hydraulic   │
                                                              │    valves via I/O expander  │
                                                              └────────────┬────────────────┘
                                                                           │
                                                                           │ LoRa 915 MHz
                                                                           │ +20 dBm both ends
                                                                           │ 5–25 km LoS with
                                                                           │ mast antenna
                                                                           ▼
                                       ┌───────────────────────────────────────────────────┐
                                       │  BASE STATION                                     │
                                       │  Portenta Max Carrier + Portenta X8               │
                                       │  ─────────────────────────────────────────────    │
                                       │  • Murata LoRa, +20 dBm SMA → 8 dBi mast antenna  │
                                       │  • SARA-R412M Cat-M1 (backup uplink)              │
                                       │  • Gigabit Ethernet → office LAN                  │
                                       │  • X8 (Linux) hosts:                              │
                                       │      - nginx + web UI (joysticks, telemetry, map) │
                                       │      - Mosquitto MQTT broker                      │
                                       │      - LoRa ↔ MQTT bridge (Python)                │
                                       │  • Laptop browser → http://lifetrac-base.local    │
                                       └───────────────────────────────────────────────────┘
```

## Control sources & priority

Three independent control sources can command the tractor. The tractor MUST follow exactly one source at any given moment, with deterministic handover.

| Priority | Source | Range | Latency target | Use case |
|---:|---|---|---:|---|
| **1 (highest)** | Handheld MKR WAN 1310 | 500 m – 2 km | ≤ 150 ms RT | Operator working alongside the tractor; safety-critical proximity work |
| **2** | Base station web UI | 5 – 25 km LoS | ≤ 250 ms RT | Remote operation from office, depot, or vehicle cab |
| **3** | Autonomy / scripted (future) | n/a (onboard) | n/a | GPS waypoint following, autonomous tillage |
| **(failsafe)** | None — local watchdog | — | ≤ 500 ms drop | All valves to neutral, engine throttle to idle |

### Arbitration rules (strict-priority + heartbeat)

1. The tractor maintains a **last-heartbeat timestamp per source**. A source is "active" if its heartbeat arrived within the last **500 ms**.
2. The tractor follows the **highest-priority active source**.
3. If no source is active, the tractor enters **failsafe** within **500 ms**: all proportional valves to neutral, all on/off implements off, engine throttle to idle.
4. The handheld has a **physical "TAKE CONTROL" momentary button** that, when pressed, latches handheld priority for **30 s** beyond the normal heartbeat window — this prevents the base station from taking control back during a critical near-tractor maneuver.
5. The base station web UI displays the **currently active source** and the time since each source's last heartbeat.

This mirrors the manned/unmanned aircraft handover model and is simpler than token-passing or TDMA. Full frame format is in [LORA_PROTOCOL.md](LORA_PROTOCOL.md).

## Radio links

### Primary: LoRa 915 MHz (custom KISS-style framing)

- **Modulation:** LoRa CSS, control SF7/BW125/CR4-5, telemetry SF9/BW250/CR4-8, image SF7/BW250/CR4-5
- **Frame:** 16-byte control frame, 32–128 byte telemetry frame, KISS-stuffed with CRC-16 + AES-128-GCM session encryption
- **Air time per encrypted control frame:** ~92 ms at SF7/BW125 before KISS expansion. This does **not** fit the current 20 Hz target; see [MASTER_PLAN.md §8.17](MASTER_PLAN.md#817-lora-control-link-phy--sf7--bw-125-khz--cr-4-5-default-no-retries-on-controlframe-adaptive-sf7sf8sf9-fallback-when-snr-margin-degrades).
- **Round-trip latency budget:** blocked pending the control cadence / PHY decision in [TODO.md](TODO.md).
- **Why custom, not LoRaWAN:** LoRaWAN's join + duty-cycle + ADR overhead is incompatible with sub-second control loops. See [LORA_PROTOCOL.md](LORA_PROTOCOL.md).

### Archived: Cellular Cat-M1 (tractor ↔ base station only)

- **Modem:** SARA-R412M-02B on both Max Carriers
- **Use case:** historical only. v25 does not populate SIMs or route telemetry/control over cellular.
- **Latency:** 100–500 ms typical for Cat-M1; **not used for live joystick control**
- **Cost:** SIM data plan, ~$5–10/month for IoT-class data

### Local: Gigabit Ethernet (base station only)

- Base station's RJ45 connects to the office/depot LAN
- Operator's laptop browser reaches the X8's web UI directly over the LAN — no internet required for normal operation

## Hardware rationale

### Why Portenta Max Carrier + X8 on the tractor?

| Requirement | Max Carrier + X8 delivers |
|---|---|
| Multiple radios on one board | LoRa (Murata) + Cat-M1 (SARA-R412M), both SMA |
| Industrial power input | 6–36 V DC (handles 12 V tractor electrical) |
| Battery backup | 18650 LiPo socket with onboard charger |
| Real-time hydraulic control | X8's onboard **STM32H747 co-MCU** (M7+M4) runs the deterministic Modbus master loop, independent of Linux |
| Linux for advanced features | i.MX 8M Mini quad-A53 runs Yocto + Docker for SSH, log rotation, future MIPI camera, telemetry recorder |
| Same SKU as base station | One Portenta X8 to stock, document, flash, and spare across both Max Carrier nodes |
| Camera input | MIPI CSI-2 (real cameras, native to X8) |
| Field-bus expansion | CAN-FD (TJA1049), RS-232/422/485 (SP335), mPCIe |
| Ethernet | Gigabit RJ45 for shop / debug network |
| Storage | 16 GB eMMC + microSD for logging |
| Security | NXP SE050 secure element for key storage |

### Why a second Max Carrier + X8 at the base station?

| Requirement | Max Carrier + X8 delivers |
|---|---|
| Linux for nginx + Mosquitto + Python | X8 runs full Yocto Linux with Docker |
| Long-range LoRa with high-gain antenna | SMA connector → 8 dBi mast antenna |
| Cellular as backup uplink | Same SARA-R412M as the tractor |
| Office LAN integration | Gigabit Ethernet |
| Same protocol stack as the tractor | Murata LoRa SiP shared with tractor & handheld |
| Web UI hosting | X8 has the CPU and RAM for nginx + WebSockets to multiple clients |

### Why MKR WAN 1310 for the handheld?

| Requirement | MKR WAN 1310 delivers |
|---|---|
| Same LoRa silicon as tractor | Murata CMWX1ZZABZ-078 — RadioLib code is portable verbatim |
| Battery powered | LiPo connector + onboard charger |
| Cheap | ~$45 vs $450 for a Max Carrier handheld |
| Small enough for one-handed use | 67 × 25 mm board fits a small handheld enclosure |
| Enough I/O for joysticks + buttons | 22 GPIO, 7 ADC, 2 UART |
| USB programming + serial | Native USB |

A Max Carrier handheld would be over-engineered and impractical to carry. The MKR's lower TX power (+14 dBm vs +20 dBm) is acceptable because the handheld is *by definition* used near the tractor.

## Failsafe behavior

| Condition | Action | Detection latency | Recovery |
|---|---|---:|---|
| No control source heartbeat within 500 ms | All valves neutral, throttle idle | ≤ 500 ms | Resumes when any source's heartbeat returns |
| Handheld E-stop pressed | All valves neutral + engine kill (relay, hardware-direct) | ≤ 100 ms | Manual reset on tractor required |
| LoRa packet CRC error | Frame discarded, sequence # advances | n/a | Next valid frame |
| Sequence # rollback (replay) | Frame discarded | n/a | Next valid in-sequence frame |
| AES-GCM authentication failure | Frame discarded, log alert | n/a | Next valid frame |
| Tractor onboard MCU watchdog timeout | Hardware reset, valves de-energized via fail-safe relay | ≤ 200 ms | MCU reboots; failsafe holds until source heartbeat resumes |
| Base station LoRa link lost | Web UI shows red banner; cellular fallback engaged for telemetry | ≤ 1 s | Auto-restore on LoRa heartbeat |
| Cellular link lost | Logged; LoRa-only mode | ≤ 30 s | Auto-restore on next data session |

## Software stack summary

| Tier | Hardware | Stack |
|---|---|---|
| Handheld | MKR WAN 1310 | Arduino C++ · RadioLib · MbedTLS (AES-GCM) |
| Tractor real-time | STM32H747 co-MCU on Portenta X8 on Max Carrier | Arduino Mbed · RadioLib · MbedTLS · Modbus RTU master |
| Tractor Linux (X8 only) | i.MX 8M Mini on Portenta X8 | Yocto Linux · SSH · journald · optional Docker telemetry recorder |
| Tractor I/O | Arduino Opta + D1608S + A0602 expansions | Arduino Mbed · ArduinoModbus slave · industrial relay/DAC/ADC |
| Base station MCU | STM32H747 co-MCU on Portenta X8 on Max Carrier | Arduino Mbed (LoRa modem driver) |
| Base station Linux | Portenta X8 | Yocto Linux · Mosquitto · Python (LoRa↔MQTT bridge) · nginx · web UI (HTML/JS + WebSockets) |

## Tractor M7 Radio Integration Modes

`firmware/tractor_h7/tractor_h7.ino` supports two compile-time-exclusive radio
integration modes:

- Default (legacy): onboard SX1276 path via RadioLib.
- Method G host path: enable `LIFETRAC_USE_METHOD_G_HOST=1` and set
    `LIFETRAC_MH_SERIAL` to the selected host UART object (for CI: `Serial1`).

Method G routes `setup()` and `loop()` into `murata_host/mh_runtime`; the
default build keeps those runtime symbols out of the linked image.

## What this design intentionally does NOT include

- **LoRaWAN.** Too much overhead for control loops; we own the air interface end-to-end.
- **Mesh networking.** No Meshtastic; star topology only (tractor is the hub for handheld+base).
- **Joystick control over cellular.** Latency unacceptable; cellular is for telemetry + non-urgent commands.
- **Image streaming over LoRa.** LoRa is too narrow; see [VIDEO_OPTIONS.md](VIDEO_OPTIONS.md) for the WiFi/cellular path.
- **A monolithic single-MCU design.** Initial drafts had the Max Carrier H7 driving valves directly. We re-adopted the **Arduino Opta** as a Modbus-RTU slave for the I/O layer because (a) the radio MCU and the valve MCU then fail independently — a LoRa-stack bug can't drive a coil, only set a register the Opta validates — and (b) Opta + D1608S + A0602 are pre-certified industrial I/O with built-in flyback / opto / fusing / dual 0–10 V outputs for the dual-flow-valve config. The earlier all-Opta + ESP32 + Pi design (now in [RESEARCH-CONTROLLER/](RESEARCH-CONTROLLER/)) was superseded by the *radio* split, not by removing Opta from the I/O role. See [TRACTOR_NODE.md § Why Opta as the valve controller](TRACTOR_NODE.md#why-opta-as-the-valve-controller).

## Where to next

- **[HARDWARE_BOM.md](HARDWARE_BOM.md)** — buy the parts
- **[LORA_PROTOCOL.md](LORA_PROTOCOL.md)** — design the air interface
- **[TRACTOR_NODE.md](TRACTOR_NODE.md)** — build the tractor side
- **[BASE_STATION.md](BASE_STATION.md)** — build the base station
- **[HANDHELD_REMOTE.md](HANDHELD_REMOTE.md)** — build the handheld
- **[TODO.md](TODO.md)** — full development roadmap
