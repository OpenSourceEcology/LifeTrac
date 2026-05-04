# 00 — Decision Record: Commitment to Method G (Custom L072 Firmware)

**Date:** 2026-05-04
**Status:** **Committed.** Methods A, B, C, D, E, F superseded.
**Author:** GitHub Copilot (Claude Opus 4.7), per direction from project lead.
**Driver:** Project lead direction —
> *"We are not in a hurry, we want to get the most performance out of LoRa as possible. Lets skip external modems or boards in method E/F, and focus exclusively on method G."*

---

## 1. Decision

LifeTrac v25 will run **custom firmware on the STM32L072CZ inside the Murata `CMWX1ZZABZ-078` SiP** on the Portenta Max Carrier. The firmware will speak directly to the in-package SX1276 over the SiP's internal SPI bus and expose a thin binary protocol to the Portenta H7 over UART.

No external LoRa hardware will be added. No vendor AT firmware (LoRaWAN, P2P, or otherwise) will be used in production. No LoRaWAN MAC stack will be carried.

## 2. Methods superseded

| Method | Description | Why dropped |
|---|---|---|
| A | Host-side `RadioLib` over SX1276 SPI | Physically impossible — SPI bus not routed off the SiP. |
| B | Stock LoRaWAN-AT firmware, Class A | Wrong MAC, wrong duty model, wrong receive-window timing for v25 control plane. |
| C | LoRaWAN-AT, Class C continuous RX | Same MAC issues as B; no per-frame FHSS, no per-frame SF, no preempt. |
| D | Raw-P2P AT firmware | Best of the AT options but adds 10–20 ms host↔modem tax per frame; depends on a third-party P2P binary; loses per-frame PHY tricks. |
| E | External SX1276 module on a routed SPI bus | Adds permanent BOM, extra antenna, per-unit mechanical work. Project is not time-constrained, so the BOM/mech burden is unjustified. |
| F | E + Murata-as-LoRaWAN-AT for telemetry | Same BOM/mech burden as E plus the AT-tax for telemetry; only attractive as a "ship-sooner" hedge, which the schedule does not require. |

The full historical analysis is preserved in [the comparison note](../../AI%20NOTES/2026-05-04_LoRa_AT_vs_SPI_Method_Comparison_Copilot_v1_0.md) and the [original Method G analysis](../../AI%20NOTES/2026-05-04_Murata_L072_Custom_Firmware_Method_G_Copilot_v1_0.md).

## 3. Scope of "custom firmware"

In scope for the L072 firmware:

- SX1276 driver (register-level, no LoRaMac).
- LoRa PHY: SF{6..12}, BW{125,250,500} kHz, CR{4/5..4/8}, payload up to ~250 B.
- v25 over-the-air protocol: 16 B ControlFrame, telemetry frame, image-fragment frame, frame header, AES-128-GCM payload, replay window, sequence numbers.
- Per-frame FHSS (8 channels in the chosen ISM sub-band).
- Adaptive SF ladder, per-frame SF selection.
- Three-profile per-frame PHY swap (e.g. SF7/BW250 control / SF9/BW125 telemetry / SF10/BW125 image).
- Priority queue with P0 preempt of in-flight P2/P3 fragments via direct radio reset.
- Failsafe heartbeat watchdog.
- Binary host transport (COBS-framed, length-prefixed, CRC-checked) — see [04_Hardware_Interface_and_Recovery.md](04_Hardware_Interface_and_Recovery.md).
- Brick-resistant boot path: persistent UART safe-mode command, BOOT0/NRST control retained by H7, golden-binary recovery (see [04](04_Hardware_Interface_and_Recovery.md) §3).
- Diagnostics: register read/write passthrough, RSSI/SNR per packet, FHSS dwell counters, RX/TX/error counters, queue depths.

Out of scope for the L072 firmware:

- LoRaWAN MAC, ADR, OTAA/ABP, MIC computation per LoRaWAN spec.
- Any OS networking stack (no LWIP, no TCP/IP).
- Any image processing — the L072 only frames pre-encrypted bytes from the H7.
- Any cellular, BLE, Wi-Fi handling.
- USB device stack on the L072 (the L072 USB pins on the Murata are not exposed on the Max Carrier).

## 4. What this commitment buys us

See [01_Capabilities_Analysis_Custom_Firmware.md](01_Capabilities_Analysis_Custom_Firmware.md) for the full enumeration. Highlights:

1. **All SPI-era v25 features recovered** with sub-millisecond host↔radio latency floor (binary UART, no AT parser).
2. **Zero added BOM, zero per-unit mechanical work** vs. stock hardware.
3. **Per-frame register-level radio control** (~1 ms retune, ~100 µs P0-preempt radio reset).
4. **New capabilities not previously planned** — see §3 of [01](01_Capabilities_Analysis_Custom_Firmware.md): listen-before-talk, real-time spectrum scanning, channel-quality-aware FHSS, deep-sleep telemetry, dual-mode emergency beacon, etc.
5. **Single hardware bill of materials forever** — every future LifeTrac unit is identical hardware. Firmware investment amortizes across the platform lifetime.

## 5. Costs we accept

- ~4–6 weeks of focused L072 firmware work (single contributor, full-time-equivalent estimate).
- Carrying our own SX1276 driver and our own LoRa PHY scheduler.
- Establishing brick-recovery procedures and a golden-binary policy from day 1.
- Building & maintaining a separate toolchain and CI target for the L072 binary.
- HIL bench rig must be expanded to include modem-side fault injection (forced reset, intentional brick).

## 6. Risks accepted, with mitigations

| Risk | Mitigation |
|---|---|
| Brick a Murata module during development | Always preserve H7-driven BOOT0 + NRST. UART safe-mode command in firmware. Golden-binary in H7 Flash. Both bench boards must have the bootloader pipeline qualified before any custom binary is flashed. |
| L072 Flash exhaustion (192 KB) | Resource budget in [02_Firmware_Architecture_Plan.md](02_Firmware_Architecture_Plan.md) §4 shows ~75 KB used. Keep no LoRaMac, prefer tinycrypt over mbedTLS, avoid C++ exceptions/RTTI, link-time-optimize. |
| L072 RAM exhaustion (20 KB) | Budget shows ~8 KB used. Use static allocation; no `malloc` in hot paths; pre-sized circular buffers. |
| SX1276 driver bugs | Start from a vetted driver (Semtech reference or the one inside `hardwario/lora-modem`); unit-test against the same driver running on a Linux host with mocked SPI. |
| Schedule slippage | Project is explicitly not time-constrained. No mitigation needed beyond honest tracking in [03_Bringup_Roadmap.md](03_Bringup_Roadmap.md). |
| Regulatory (FCC §15.247) | Per-frame FHSS over ≥50 channels with ≤0.4 s dwell satisfies §15.247(a)(1). v25 already targets 8 channels — revisit channel count if needed. PHY profile swap is allowed within §15.247 limits. |

## 7. Acceptance criteria for "Method G complete"

The custom firmware is considered done when, on a HIL bench:

1. All `R-1`..`R-7` LoRa requirements pass with margin.
2. `L1` (≤25 ms per-fragment cap) holds at p99 over a 1-hour run.
3. `L3` (≤500 ms failsafe trip) holds with 100% success over 1000 induced link drops.
4. `L-V11` (handheld→valve ≤150 ms p99, base→valve ≤250 ms p99) holds at all three SF profiles.
5. Round-trip H7↔L072 binary UART latency p99 ≤ 3 ms at 921600 baud.
6. P0-preempt latency p99 ≤ 5 ms (request-to-air-quiet).
7. Brick-recovery procedure executes successfully on both bench boards.
8. 24-hour soak test with random packet loss (10% PER) shows no firmware crash, no memory growth, no missed heartbeats.
9. The Arduino LoRaWAN-AT golden binary still flashes successfully after a custom-firmware run (proves the bootloader pipeline isn't damaged).

## 8. What this decision does *not* change

- v25 over-the-air [LoRa Protocol](../LORA_PROTOCOL.md): unchanged. The bytes on the air are the same; only the MCU that emits them moves from H7 to L072.
- v25 application logic on H7 / X8: unchanged in intent; the H7 keeps the priority queue, AES-GCM (host-side encryption is the default — see [01](01_Capabilities_Analysis_Custom_Firmware.md) §4.2), failsafe state machine, image fragmenter, telemetry scheduler. The L072 receives pre-formed frames over UART and transmits them.
- Handheld controller firmware: unchanged.
- Base-station bridge: unchanged on the LoRa side.
- HIL test program: gains a "modem firmware" tier (W4-pre) before the existing W4-00.

## 9. Reversibility

This decision is reversible at low cost up until the end of [03 Phase 4](03_Bringup_Roadmap.md). If the L072 firmware effort hits an unforeseen wall (e.g. an undocumented Murata pin strap that prevents flashing in production), the fallback is to add an external SX1276 module (the previously-rejected Method E) on a small daughterboard. We do not need to commit to that fallback today; we only need to keep the H7-side host code structured so a different physical radio could be swapped in without rewriting the application layer. [02_Firmware_Architecture_Plan.md](02_Firmware_Architecture_Plan.md) §6 captures this constraint.

## 10. One-line summary

> **LifeTrac v25 commits to writing custom firmware for the STM32L072 inside the Murata SiP, recovering every SPI-era feature, gaining several new ones (see [01](01_Capabilities_Analysis_Custom_Firmware.md)), at the cost of ~4–6 weeks of focused firmware work and a brick-resistance discipline that begins on day 1.**
