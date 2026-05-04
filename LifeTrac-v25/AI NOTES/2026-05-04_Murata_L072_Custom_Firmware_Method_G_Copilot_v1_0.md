# Method G — Custom Firmware on the Murata STM32L072

**Date:** 2026-05-04
**Author:** GitHub Copilot (Claude Opus 4.7)
**Version:** v1.0
**Status:** Decision support — answers the user's question:
> *"If we build our own custom firmware, could we implement some of the items we had planned to implement with SPI? like channel frequency hopping?"*

**Short answer:** **Yes. All of them.** Custom firmware running on the
Murata's internal STM32L072 has the same direct register-level access
to the SX1276 die that our (impossible) host-side `RadioLib` plan
assumed. This recovers per-frame FHSS, the adaptive-SF ladder,
per-frame PHY-profile swap, P0 fragment preemption, AES-GCM payload
crypto, the custom 16 B ControlFrame, and the no-ACK / no-retry
semantics that v25 was designed around. It is the **highest-
engineering-cost, highest-reward** of the options on the table, and it
slots into the existing comparison as **Method G**.

This note evaluates Method G against the same scorecards used in
[2026-05-04_LoRa_AT_vs_SPI_Method_Comparison_Copilot_v1_0.md](2026-05-04_LoRa_AT_vs_SPI_Method_Comparison_Copilot_v1_0.md).

---

## 1. What the Murata SiP actually is

The CMWX1ZZABZ-078 is a System-in-Package with:

- **STM32L072CZ** (Cortex-M0+ @ 32 MHz, **192 KB Flash, 20 KB RAM**, USART/SPI/I²C/USB-FS, hardware AES-128 in some die revisions, RNG, RTC).
- **Semtech SX1276** LoRa radio die, wired to the L072 via an **internal SPI bus** that is *not* exposed on any external pin. *This is the very SPI bus we wanted but couldn't reach from the H7.*
- TCXO, switch, LNA, PA, all internal.

The L072 is user-programmable and is the same MCU the stock Murata
LoRaWAN-AT firmware runs on. The factory firmware is just one of many
possible programs.

## 2. Available development paths

| Path | Maturity | Effort to bring up | Notes |
|---|---|---|---|
| **STM32CubeIDE + I-CUBE-LRWAN** | Production | Medium | STMicro reference. Includes `AT_Slave` (P2P-capable), `LoRaWAN_End_Node`, and a HAL-based template. Best for fully custom apps. |
| **`hardwario/lora-modem`** ([github](https://github.com/hardwario/lora-modem)) | Production, MIT licensed | **Lowest** | Open-source LoRaWAN-AT modem replacement; well-tested, clean Cube-HAL build, exposes a richer AT than stock. Good *fork point* for our protocol. |
| **Arduino-CMWX core** (Thomas Roell / GrumpyOldPizza) | Mature, community | Low–medium | Lets us write sketches that run on the L072. Good for prototyping; less control than bare HAL. |
| **Zephyr RTOS** | Production | Higher | Has SX1276 + STM32L0 drivers, decent LoRa-PHY support. Heavier, but gives us a real RTOS and proven crypto. |
| **From-scratch on Cube HAL** | Production | High | Full control, no abstractions. Only worth it if we hit a wall in `hardwario`/I-CUBE. |

**Recommended starting point: fork [`hardwario/lora-modem`](https://github.com/hardwario/lora-modem)** and rip out its LoRaMac stack, keeping the SX1276 driver, the OS-abstraction (FreeRTOS), the build system, and the UART transport. Drop in our `lora_proto.c` (or its L072-resident equivalent).

## 3. Flashing & recovery (the critical risk axis)

The STM32L072 has a **mask-ROM system bootloader at `0x1FF00000`** that is *always* present and is entered when the BOOT0 pin is pulled high at reset. The `MKRWANFWUpdate_standalone` sketch already proves the Max Carrier wires both **BOOT0 and NRST** to H7-controllable lines — that's the only way the H7 can drive the L072 into bootloader mode without USB. The same pin sequence flashes any binary, not just Arduino's.

**Brick recovery procedure** (must be designed-in from day 1):

1. **Never overwrite the ROM bootloader** — it lives outside user Flash, can't be touched.
2. **Keep BOOT0 / NRST control verified** in our H7 firmware: a single command from the H7 must always be able to hold BOOT0 high and pulse NRST to enter bootloader, regardless of what the L072 firmware is doing.
3. **Always include a UART-triggered safe-mode entry** in our L072 firmware: any 16 B "panic frame" on UART within the first 500 ms of boot causes the L072 to call the ROM bootloader directly (`SystemMemoryBoot()`), so we don't need BOOT0 cycling.
4. **Keep a known-good golden binary** (latest `MKRWANFWUpdate_standalone` LoRaWAN-AT) in the H7's external Flash so we can always restore the modem to a probable-working state.
5. **Stage 1 of any custom-firmware bring-up is to flash the Arduino LoRaWAN-AT first** (Method D Step 1, see [the bench-probe note](2026-05-04_Murata_UART_Firmware_Probe_Bench_Results_Copilot_v1_0.md)) to confirm the entire flash pipeline works *before* we put our own binary on the modem.

If items 2–4 are designed correctly, the brick risk is **near-zero** in practice — the same risk profile as updating any other STM32 in our stack.

## 4. Resource budget on the L072

| Resource | L072 capacity | v25 needs (estimated) | Margin |
|---|---|---|---|
| Flash | 192 KB | SX1276 driver (~25 KB) + LoRa PHY/MAC stub (~10 KB) + AES-GCM (~12 KB w/ tinycrypt) + KISS framer (~2 KB) + FHSS table & RNG (~3 KB) + replay window (~1 KB) + UART transport (~5 KB) + FreeRTOS (~15 KB) + bootloader hooks (~2 KB) + headroom = **~75 KB** | **~117 KB free** |
| RAM | 20 KB | RTOS stacks 4 × 1 KB, queues 2 × 256 B, SX1276 page buffer 256 B, AES-GCM context ~512 B, UART DMA buffers 2 × 512 B, app state ~1 KB = **~7–8 KB** | **~12 KB free** |
| CPU @ 32 MHz | ~32 DMIPS | AES-GCM throughput on M0+ (tinycrypt) ≈ 60–100 KB/s. ControlFrame is 16 B at 20 Hz = 320 B/s. Telemetry/image at ~5 KB/s peak. **All fit by 10× margin.** | comfortable |
| Hardware AES | Yes on L072CZ rev Y onward | Optional — software fits anyway | bonus |
| RNG | Yes | FHSS shuffle seed, GCM nonces | OK |
| Internal SPI to SX1276 | Yes, dedicated, not shared | Per-frame retune ≪ 1 ms | excellent |

**Conclusion:** the L072 is comfortably big enough. The thing that
*doesn't* fit is anything image-related — image processing stays on the
H7/X8; the L072 only frames and transmits the bytes.

## 5. Feature recovery vs. the original SPI plan

Re-running [the comparison-note §4 scorecards](2026-05-04_LoRa_AT_vs_SPI_Method_Comparison_Copilot_v1_0.md) for Method G:

| Capability | A (SPI/RadioLib) | D (P2P AT) | E (external SX1276) | **G (custom L072 fw)** |
|---|---|---|---|---|
| 50 ms / 20 Hz ControlFrame cadence | ✅ | ⚠️ ~56 ms typ. | ✅ | ✅ — *and* lower jitter than A (no host-MCU OS scheduler in the path) |
| Handheld→valve ≤150 ms p99 | ✅ | ✅ | ✅ | ✅ |
| Failsafe ≤500 ms | ✅ | ✅ | ✅ | ✅ |
| ≤25 ms per-fragment cap (L1) | ✅ | ⚠️ ~40 ms | ✅ | ✅ |
| Custom 16 B ControlFrame | ✅ | ✅ | ✅ | ✅ |
| AES-128-GCM on payload | ✅ | ✅ | ✅ | ✅ — moved inside the L072 |
| KISS framing | ✅ | ✅ | ✅ | ✅ — or skip entirely (we own both ends of the UART link) |
| FHSS, 8 channels per frame | ✅ | ⚠️ slow | ✅ | ✅ — **register-level retune <1 ms** |
| Adaptive SF ladder (per-frame) | ✅ | ⚠️ per-burst | ✅ | ✅ |
| Three-profile per-frame PHY swap | ✅ | ❌ | ✅ | ✅ |
| P0 preempt of P2/P3 fragments | ✅ | ⚠️ needs `AT+TXABORT` | ✅ | ✅ — direct radio reset, ~100 µs |
| No-ACK / no-retry | ✅ | ✅ | ✅ | ✅ |
| FCC §15.247 hopping argument intact | ✅ | ⚠️ | ✅ | ✅ |
| Host MCU SPI bus for control plane | consumed | free | consumed | **free** |
| Host MCU CPU cycles for radio I/O | high | medium | high | **near-zero** |
| Host↔radio latency floor | ~0.1 ms | ~10–20 ms | ~0.1 ms | ~1–2 ms (UART transport only, no AT processing) |

**Method G dominates everywhere except engineering effort.**

## 6. UART transport between H7 and L072 (with custom firmware)

Once the L072 firmware is ours, the UART protocol can be *much* thinner than AT. A reasonable design:

```
Frame on UART (binary, length-prefixed, COBS-framed):

  0x00 (COBS delimiter)
  <COBS-encoded payload:>
    1 B  type   (CTRL_TX / CTRL_RX_URC / TELEM_TX / IMG_TX / CFG / ERR / PING)
    1 B  flags  (priority, ACK-requested, encrypted-already, etc.)
    2 B  seq    (host-assigned, echoed in URC)
    2 B  payload_len
    N B  payload bytes (already AES-GCM encrypted by host *or* plaintext to be encrypted on L072 — configurable)
    2 B  CRC-16
  0x00 (COBS delimiter)
```

Properties:
- COBS framing means **no escape characters in the payload**, no SLIP-style overhead drama.
- Length-prefixed → the L072 knows exactly how many bytes to expect, so DMA-on-IDLE works perfectly. No "AT command parser" state machine.
- `seq` → host can pipeline frames without waiting for response.
- **Baud:** start at 921600 (L072 USART easily handles it; H7 too); fall back to 460800 if EMI-on-PCB-traces is a problem. AT-baud calculations from the previous note become irrelevant.

At **921600 8N1**, a 44 B encrypted ControlFrame frame on the wire (~50 B with COBS+CRC+headers) takes:

```
50 B × 10 bits / 921600 ≈ 0.54 ms one-way
```

Round-trip H7↔L072 is well under 2 ms. **The host↔modem latency tax that ruled out AT effectively vanishes.**

## 7. Engineering cost — honest accounting

| Phase | Effort | Notes |
|---|---|---|
| Phase 0: De-risk | 1–2 days | Flash Arduino LoRaWAN-AT first ([Method D Step 1](2026-05-04_Murata_UART_Firmware_Probe_Bench_Results_Copilot_v1_0.md)); confirm flash pipeline works; baseline measurements. Required regardless. |
| Phase 1: Fork & build | 2–3 days | Clone `hardwario/lora-modem`, set up STM32CubeIDE / `cmake-stm32` build, flash it, verify it answers AT. |
| Phase 2: Strip & re-host | 3–5 days | Remove LoRaMac, keep SX1276 driver + RTOS + UART, define the binary UART protocol, write a minimal "echo-payload-over-LoRa" loopback. |
| Phase 3: Port `lora_proto.c` | 5–7 days | Bring the v25 frame format, AES-GCM, KISS (or replace with COBS), FHSS, replay window into the L072. Adapt for 32-bit-aligned-ish memory and small RAM. |
| Phase 4: Adaptive SF + per-frame retune | 3–5 days | Implement the SF ladder and the three-profile per-frame swap with measured retune cost. |
| Phase 5: Priority queue + P0 preempt | 3–5 days | Implement the queue and the radio-abort path; measure preempt latency. |
| Phase 6: HIL bring-up + R-series tests | 5–10 days | Re-run R-1 through R-7, L1, L3, L-V11. Probably some firmware iteration. |
| Phase 7: Hardening | ongoing | Watchdog, brown-out, brick-recovery commands, factory-reset path, OTA update protocol, signed binaries (eventually). |

**Total: ~4–6 weeks of focused firmware work** before HIL is fully green. Compare to:

- Method E (external SX1276): ~1–2 weeks (HW integration + RadioLib reuse) but adds permanent BOM cost and an extra antenna / mechanical task on every unit.
- Method D Step 1 (LoRaWAN-AT only): ~1 day, but unusable for control plane.
- Method F (E + LoRaWAN-AT-Murata for telemetry): ~2–3 weeks; survives any firmware variant.

**The right scaling decision is probably Method G *if* this project is a long-lived platform**, because every future LifeTrac unit is then identical hardware (no extra LoRa board), uses one antenna, and the firmware investment amortizes. **Method E or F is the right scaling decision *if* the priority is shipping a working tractor before doing a multi-week firmware project**.

## 8. The hybrid path — recommended sequencing

Don't pick now. Walk it in stages. Each stage is a no-regrets investment:

1. **Today (~1 hour):** Flash `MKRWANFWUpdate_standalone` (Method D Step 1) on Board2. Confirm flash pipeline + post-flash AT works. *This is required for both Method D and Method G — it's the qualification of the bootloader path.*
2. **Week 1:** Order an external SX1276 module (RFM95W or Adafruit LoRa FeatherWing) as Method E insurance. Lead time runs in parallel with everything else.
3. **Week 1–2:** Bring up [`hardwario/lora-modem`](https://github.com/hardwario/lora-modem) on Board2 (Method G Phase 1). This gives us a clean code base to work from and re-confirms we can flash whatever we want.
4. **Decision gate:** With (1) and (3) green and (2) on the bench, decide whether to commit to G (full custom) or pivot to F (E + LoRaWAN-AT-Murata for telemetry). The only thing this decision waits on is whether the project timeline can absorb 4–6 weeks of L072 firmware work.
5. **Either way:** Method E hardware on the bench is useful as an independent sanity radio for HIL tests of the chosen path. Money not wasted.

## 9. Recommended decision

If the v25 design intent (per-frame FHSS, adaptive SF, three-profile swap, 25 ms fragment cap, 20 Hz cadence) is **non-negotiable** for the protocol's safety/regulatory argument, **Method G is the only option that preserves all of it without adding hardware**. It is also the cheapest in BOM cost forever after.

If the v25 design intent is **negotiable** and we'd rather ship sooner with a known-good radio path, **Method F** (external SX1276 for control + Murata in LoRaWAN-AT for telemetry) is the pragmatic choice.

**Methods B and C remain rejected** for the control plane regardless. **Method D Step 1 is a prerequisite for everything else** because it qualifies the modem-flash pipeline.

## 10. Doc updates this decision will trigger

- [LORA_PROTOCOL.md](../DESIGN-CONTROLLER/LORA_PROTOCOL.md): if Method G is chosen, no protocol changes; if F or E, only the host↔radio interface section changes.
- [LORA_IMPLEMENTATION.md](../DESIGN-CONTROLLER/LORA_IMPLEMENTATION.md): under Method G, `tractor_h7.ino` shrinks dramatically — most of `lora_proto.c` migrates to a new `firmware/murata_l072/` subdirectory; the H7 keeps only the COBS-framed UART transport and its existing application logic.
- [DECISIONS.md](../DESIGN-CONTROLLER/DECISIONS.md): record the chosen method (E / F / G) and the brick-recovery design.
- [MASTER_TEST_PROGRAM.md](../MASTER_TEST_PROGRAM.md): add W4-pre tier for the L072 firmware bring-up (loopback, AT-passthrough, single-frame TX/RX) before W4-00.
- New top-level: [LifeTrac-v25/DESIGN-CONTROLLER/firmware/murata_l072/](../DESIGN-CONTROLLER/firmware/) (placeholder until we commit to G).

## 11. One-line summary

> **Yes — custom firmware on the Murata's internal STM32L072 recovers every SPI-era feature (per-frame FHSS, adaptive SF, three-profile swap, P0 preempt, AES-GCM, custom frame format, no-ACK semantics) with sub-millisecond host↔radio latency. Cost is 4–6 weeks of focused L072 firmware work using `hardwario/lora-modem` as the fork point. Brick risk is near-zero if the H7 always retains BOOT0/NRST control. The pragmatic sequencing is: flash the Arduino LoRaWAN-AT (Method D Step 1) tomorrow → start a parallel `hardwario/lora-modem` fork → decide G vs F at the end of week 2 with both options on the bench.**
