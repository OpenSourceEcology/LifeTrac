# LifeTrac v25 Controller Build Guide

> **Heads up!** This is a build guide, not a design document. The "why" lives in [`../DESIGN-CONTROLLER/`](../DESIGN-CONTROLLER/). The "how" lives here. If you find a contradiction between the two, the design folder wins — open an issue and we'll fix the build guide.

## Introduction

This guide walks you through building the wireless control system for a LifeTrac v25 tractor end-to-end. By the time you're done, you'll have:

- A **Tractor Node** mounted in an IP65 enclosure on the tractor, driving the hydraulic valves and listening on 915 MHz LoRa.
- A **Base Station** in your shop or office, hosting a web UI you can drive a joystick from in any browser on the LAN.
- An optional **Handheld Remote** for walk-around control with a real joystick and an E-stop.

The architecture is documented in [`DESIGN-CONTROLLER/MASTER_PLAN.md`](../DESIGN-CONTROLLER/MASTER_PLAN.md) — short version: LoRa-only RF link, AES-128-GCM authenticated frames, Arduino Opta + D1608S SSR for the industrial I/O layer, Portenta X8 + onboard STM32H747 co-MCU on both tractor and base.

This guide is written in the style of a SparkFun hookup tutorial. Each section starts with what you need, walks through the steps in order, and ends with a known-good test you can run before moving on.

## Suggested Reading

Before you start cutting cable, skim:

- [`DESIGN-CONTROLLER/MASTER_PLAN.md`](../DESIGN-CONTROLLER/MASTER_PLAN.md) — the canonical scope and pinned hardware decisions (especially §8.2, §8.5, §8.9, §8.17, §8.18).
- [`DESIGN-CONTROLLER/ARCHITECTURE.md`](../DESIGN-CONTROLLER/ARCHITECTURE.md) — block diagrams and the failsafe model.
- [Arduino Portenta Max Carrier user manual](https://docs.arduino.cc/hardware/portenta-max-carrier/) — pin-out and connector locations.
- [Arduino Opta + Expansions user manual](https://docs.arduino.cc/hardware/opta/) — Modbus and I/O behavior.
- SparkFun's [Working with Wire](https://learn.sparkfun.com/tutorials/working-with-wire) and [How to Solder: Through-Hole Soldering](https://learn.sparkfun.com/tutorials/how-to-solder-through-hole-soldering) tutorials if you're new to wiring.

## Build Order

Build the three nodes in this order. Each section assumes the previous one is done so you can bench-test as you go.

| # | Step | What you'll do | Time |
|---|---|---|---|
| 1 | [Bill of Materials](01_bill_of_materials.md) | Order parts from DigiKey, Mouser, and the Arduino Store. | 30 min ordering, 1–4 weeks lead time |
| 2 | [Tractor Node Assembly](02_tractor_node_assembly.md) | Build the Max Carrier + X8 + Opta stack in an IP65 enclosure. | 1 day |
| 3 | [Base Station Assembly](03_base_station_assembly.md) | Build the second Max Carrier + X8, mount the mast antenna. | 1 day + mast install |
| 4 | [Handheld Assembly](04_handheld_assembly.md) *(optional)* | Build the MKR WAN 1310 handheld with joysticks and E-stop. | 1 day |
| 5 | [Firmware & Software Installation](05_firmware_installation.md) | Flash the Arduino sketches, install the Docker stack on the base X8. | 2–3 hours |
| 6 | [Bring-Up & Testing](06_bringup_and_testing.md) | Bench-pair the three nodes, drive valves with LED stand-ins, run the safety tests. | 1 day |

## Tools You'll Need

You probably already own most of this. The starred items are non-negotiable.

- **Soldering iron** ★ — temperature-controlled, fine tip. Hakko FX-888D or similar.
- **Solder + flux** ★ — 60/40 leaded or SAC305 lead-free, 0.5–0.8 mm.
- **Wire strippers** ★ — for 22–14 AWG.
- **Crimp tool** for Deutsch DT and JST-XH connectors.
- **Multimeter** ★ — continuity, DC volts, current.
- **Cordless drill + step bit** for enclosure cable glands.
- **Heat-shrink assortment** + heat gun.
- **Cable lugs + ferrule kit** for the 12 V power feed.
- **Torx + Phillips bit set**.
- **DIN-rail cutters** if you're trimming a rail to fit the enclosure.
- *(Optional but recommended)* **Logic analyzer** (Saleae or clone) for Modbus debugging.
- *(Optional but recommended)* **NanoVNA** for SWR-checking the LoRa antennas.

## Conventions Used in This Guide

- **Bold** is a part name, button name, or setting that appears on a screen.
- `Inline code` is a file name, command, or value to type literally.
- Block quotes prefixed with **Heads up!** are safety warnings — don't skip them.
- Block quotes prefixed with **Pro tip** are time-savers — easy to skip on a first read.
- Photo placeholders (`![Photo: ...](photos/xxx.jpg)`) will be filled in once the first build is done. Until then, use the wiring diagrams in [`DESIGN-CONTROLLER/TRACTOR_NODE.md`](../DESIGN-CONTROLLER/TRACTOR_NODE.md) and the like.

## Resources & Going Further

- **Repository:** [github.com/OpenSourceEcology/LifeTrac](https://github.com/OpenSourceEcology/LifeTrac)
- **Issues:** open one if you hit a snag — we backport real-build feedback into this guide.
- **Hydraulic build:** [`../BUILD-HYDRAULIC/`](../BUILD-HYDRAULIC/) (parallel guide; controller and hydraulic builds can happen in parallel).
- **Structural build:** [`../BUILD-STRUCTURE/`](../BUILD-STRUCTURE/).

---

*Last reviewed: 2026-04-26. Tracks the canonical [`DESIGN-CONTROLLER/MASTER_PLAN.md`](../DESIGN-CONTROLLER/MASTER_PLAN.md) §8.1–§8.18.*
