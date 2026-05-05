# DESIGN-LORAFIRMWARE — `OSE-LifeTracLORA-MurataFW`

**Owner:** LifeTrac v25 controller team
**Status:** Active design — Method G committed (2026-05-04). Methods A, B, C, D, E, F superseded.
**Firmware product name:** `OSE-LifeTracLORA-MurataFW`
**Firmware version convention:** `vMAJOR.MINOR.PATCH`, starting at `v0.0.0` for initial bring-up builds.
**Target hardware:** STM32L072CZ inside the Murata SiP on the Portenta Max Carrier (LoRa SX1276 die wired to the L072 over an internal SPI bus that is not externally routed).

---

## Why this folder exists

The Portenta Max Carrier exposes the Murata LoRa module only via UART, not via the SX1276 SPI bus. The original v25 LoRa protocol design (per-frame FHSS, adaptive SF, three-profile PHY swap, P0 preempt, custom 16 B ControlFrame, AES-GCM, no-ACK semantics) assumed direct SX1276 register access from the host MCU. That access is physically impossible on this carrier.

The decision (see [00_DECISION_Method_G_Commitment.md](00_DECISION_Method_G_Commitment.md)) is to **write custom firmware that runs on the STM32L072 inside the Murata package** and reaches the SX1276 over the SiP's internal SPI bus — recovering all of the SPI-era features and gaining several more that were never possible from the host.

This folder holds the **design, analysis, and bring-up plans** for that firmware. The firmware source code itself, when it lands, will live in [`../firmware/murata_l072/`](../firmware/) (not yet created). Release artifacts and `VER_URC` identity strings should use the product name `OSE-LifeTracLORA-MurataFW` and the `v0.0.0`-style version convention above.

---

## Documents in this folder

| # | Document | Purpose |
|---|---|---|
| 00 | [Method G Commitment](00_DECISION_Method_G_Commitment.md) | Decision record: why G, why E/F/D are dropped, scope. |
| 01 | [Capabilities Analysis — Custom Firmware](01_Capabilities_Analysis_Custom_Firmware.md) | What new improvements become possible *because* we own the L072 firmware (beyond just recovering SPI-era features). |
| 02 | [Firmware Architecture Plan](02_Firmware_Architecture_Plan.md) | Project structure, fork point, module breakdown, RAM/Flash budget, build & toolchain. |
| 03 | [Bring-up Roadmap](03_Bringup_Roadmap.md) | Phased, no-deadline plan from "today" to "v25 LoRa fully on custom firmware." |
| 04 | [Hardware Interface & Recovery](04_Hardware_Interface_and_Recovery.md) | UART transport spec (H7 ↔ L072), BOOT0/NRST control, brick-recovery design, golden-binary policy. |

---

## Cross-references

- [LifeTrac-v25/AI NOTES/2026-05-04_Murata_L072_Custom_Firmware_Method_G_Copilot_v1_0.md](../../AI%20NOTES/2026-05-04_Murata_L072_Custom_Firmware_Method_G_Copilot_v1_0.md) — original Method G analysis that triggered this folder.
- [LifeTrac-v25/AI NOTES/2026-05-04_LoRa_AT_vs_SPI_Method_Comparison_Copilot_v1_0.md](../../AI%20NOTES/2026-05-04_LoRa_AT_vs_SPI_Method_Comparison_Copilot_v1_0.md) — historical comparison of all six prior methods.
- [LifeTrac-v25/AI NOTES/2026-05-04_Murata_UART_Firmware_Probe_Bench_Results_Copilot_v1_0.md](../../AI%20NOTES/2026-05-04_Murata_UART_Firmware_Probe_Bench_Results_Copilot_v1_0.md) — the bench data that drove the decision.
- [LifeTrac-v25/DESIGN-CONTROLLER/LORA_PROTOCOL.md](../LORA_PROTOCOL.md) — the over-the-air protocol the firmware must implement (largely unchanged by this decision; only the *implementation location* moves from H7 to L072).
- [LifeTrac-v25/DESIGN-CONTROLLER/LORA_IMPLEMENTATION.md](../LORA_IMPLEMENTATION.md) — implementation guide; large portions migrate to the L072 firmware once Method G lands.
- [LifeTrac-v25/DESIGN-CONTROLLER/DECISIONS.md](../DECISIONS.md) — global decision log; will reference [00_DECISION_Method_G_Commitment.md](00_DECISION_Method_G_Commitment.md) once linked.
- [LifeTrac-v25/DESIGN-CONTROLLER/MASTER_PLAN.md](../MASTER_PLAN.md) §8.17 — LoRa subsystem master plan.

---

## Out of scope for this folder

- LoRaWAN MAC compliance and any TTN/Helium gateway integration (rejected per Method B/C analysis).
- External SX1276 hardware (RFM95W, FeatherWing) — was Method E/F insurance, now dropped.
- Image processing, vision pipelines, NPU — H7/X8 concerns; the L072 only frames and transmits bytes.
- Hydraulic/structural design — see other DESIGN-* folders.
