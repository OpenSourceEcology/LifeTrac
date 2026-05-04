# LoRa Transport Method Comparison — Murata `CMWX1ZZABZ-078` on Portenta Max Carrier

**Date:** 2026-05-04
**Author:** GitHub Copilot (Claude Opus 4.7)
**Version:** v1.0
**Status:** Decision support — answers the user's question:
> *"Make a full comparison of the available methods."*

**Trigger context:** Bench finding (2026-05-04) confirmed the Murata SiP's
SX1276 SPI bus is **not routed** to the Portenta high-density connectors
on the Max Carrier. The host (H7 M7 or X8 Linux) can only talk to the
module via its UART (`Serial3` on H7 / `/dev/ttymxc3` on X8 Linux). The
question is: which transport method should LifeTrac v25 commit to, and
what does each cost?

This note **does not** replace the existing protocol docs; it ranks the
options against the v25 budgets so the next bench session can make a
decision before any firmware refactor.

---

## 1. Reference budgets we are measuring against

From [LORA_PROTOCOL.md](../DESIGN-CONTROLLER/LORA_PROTOCOL.md),
[LORA_IMPLEMENTATION.md](../DESIGN-CONTROLLER/LORA_IMPLEMENTATION.md),
[MASTER_PLAN.md](../DESIGN-CONTROLLER/MASTER_PLAN.md) and
[DECISIONS.md](../DESIGN-CONTROLLER/DECISIONS.md):

| Symbol | Budget | Source |
|---|---|---|
| ControlFrame TOA (SF7/BW250/CR4-5, 44 B encrypted) | ~46 ms | LORA_PROTOCOL §PHY |
| ControlFrame cadence | 50 ms (20 Hz) | LORA_PROTOCOL §control |
| Handheld→tractor valve, p99 | ≤150 ms | L-V11 |
| Base→tractor valve, p99 | ≤250 ms | L-V11 |
| LoRa-leg, end-to-end | ≤100 ms | KR-E3 |
| Failsafe trip | ≤500 ms | L3 |
| Per-fragment airtime cap | ≤25 ms | L1 / R-6 |
| Per-frame retune cost | ≤5 ms | R-7 |
| FHSS dwell per channel | ~12.5% | LORA_PROTOCOL §FHSS |

---

## 2. Methods on the table

The user's research surfaces three orthogonal axes. The interesting
combinations (after eliminating physically-impossible ones) are:

| # | Host↔modem | LoRa mode | Library | Status |
|---|---|---|---|---|
| **A** | Direct SPI | Raw P2P | RadioLib | **Physically impossible** — SPI not routed. (Current `tractor_h7.ino` baseline.) |
| **B** | UART AT @ 19 200 | LoRaWAN Class A | MKRWAN (stock Murata) | Possible, design-incompatible. |
| **C** | UART AT @ 19 200 | LoRaWAN Class C | MKRWAN (stock Murata, `+CLASS=C`) | Possible, partially-compatible. |
| **D** | UART AT @ 115 200 | Raw P2P | Custom AT firmware (Semtech `AT_Slave`, `MKRWAN_LoraConsole`, etc.) | Possible, **design-compatible** if firmware exists. |
| **E** | External SX1276 over a routed SPI | Raw P2P | RadioLib | Possible via add-on board (e.g. RFM95 on free SPI). Re-introduces design-A behaviour. |
| **F** | Two-radio split: keep Murata for telemetry/image, add external SX1276 for control | Mixed | Both | "Belt and suspenders" — survives any AT firmware variant. |

Methods A is dead. Below we score B–F.

---

## 3. UART overhead — the constant tax on B/C/D

For a 44 B encrypted ControlFrame, AT framing overhead is approximately:

```
"AT+UTX=44\r"            → 11 B
44 B payload (binary or hex)
URC response "+OK\r\n"   → ~5 B
```

| Encoding | Bytes on wire (TX cmd) | Bytes (RX URC for 44 B) |
|---|---|---|
| Binary `AT+UTX=N` | ~60 B | ~60 B |
| Hex-encoded `AT+SEND=...` | ~104 B | ~104 B |

UART byte time at 8N1 is 10 bits / byte (8N2 is 11):

| Baud | 60 B (binary) | 104 B (hex) |
|---|---|---|
| 19 200 8N2 | **34 ms** | **60 ms** |
| 57 600 8N1 | 10 ms | 18 ms |
| 115 200 8N1 | **5.2 ms** | **9.0 ms** |
| 230 400 8N1 (if FW supports) | 2.6 ms | 4.5 ms |

**Modem command-handling time** (state machine inside the STM32L0 in the
Murata SiP): typically **5–30 ms** for `+UTX`, longer for `+JOIN` or
`+CFG` mutators. Add ~10 ms in budget calculations.

So the **per-frame UART tax** added on top of LoRa airtime:

| Baud | Per-frame UART+modem tax | Total (with 46 ms TOA) |
|---|---|---|
| 19 200, hex | ~70 ms | **~116 ms** — blows 50 ms slot |
| 19 200, binary | ~44 ms | **~90 ms** — blows 50 ms slot |
| 115 200, binary | ~15 ms | **~61 ms** — still over 50 ms |
| 115 200, binary, modem fast (5 ms) | ~10 ms | **~56 ms** — borderline |
| 230 400, binary | ~12 ms | **~58 ms** — borderline |

**Implication for the 20 Hz / 50 ms control cadence:** even at 115 200
baud with a fast P2P AT firmware, the slot is over by ~10–15 %. The
realistic ControlFrame cadence is **10–12 Hz (≈80–100 ms slot)** for any
AT-based method.

Throughput ceiling at 20 Hz × ~60 B = **1200 B/s sustained, ~1500 B/s
peak**. Comfortably below 115 200's ~11.5 KB/s; uncomfortably above 19
200's ~1.92 KB/s once telemetry/image share the same UART.

---

## 4. Method-by-method scorecard

Legend: ✅ meets, ⚠️ marginal / requires re-derivation, ❌ violated, ⛔ impossible.

### 4.1 Per-frame latency / cadence

| Capability | A (SPI/RadioLib) | B (LoRaWAN A) | C (LoRaWAN C) | D (P2P AT @115k2) | E (External SX1276) | F (Two-radio split) |
|---|---|---|---|---|---|---|
| Physically possible on Max Carrier | ⛔ | ✅ | ✅ | ✅ (needs firmware flash) | ✅ (needs board) | ✅ |
| Meets 50 ms / 20 Hz ControlFrame cadence | ✅ | ❌ (RX1/RX2 windows ≈ 1–5 s on downlink) | ⚠️ (uplink ~60 ms; downlink ≪ when modem RX is open) | ⚠️ (~56 ms typical; needs cadence relaxed to 100 ms / 10 Hz) | ✅ | ✅ on the SX1276 leg |
| Meets handheld→valve ≤150 ms p99 | ✅ | ❌ | ⚠️ | ✅ | ✅ | ✅ |
| Meets base→tractor ≤250 ms p99 | ✅ | ❌ | ⚠️ | ✅ | ✅ | ✅ |
| Meets failsafe ≤500 ms | ✅ | ⚠️ | ✅ | ✅ | ✅ | ✅ |
| Meets ≤25 ms per-fragment cap (L1) | ✅ | ❌ (no host-side preempt) | ❌ | ⚠️ (cap likely → 40 ms) | ✅ | ✅ on control leg |

### 4.2 Protocol features

| Feature | A | B | C | D | E | F |
|---|---|---|---|---|---|---|
| Custom 16 B ControlFrame | ✅ | ❌ (LoRaWAN MAC wraps payload) | ❌ | ✅ | ✅ | ✅ |
| AES-128-GCM on payload (our keys, replay window 64) | ✅ | ❌ (LoRaWAN uses its own AppSKey/NwkSKey) | ❌ | ✅ | ✅ | ✅ |
| KISS framing | ✅ | n/a | n/a | ✅ | ✅ | ✅ |
| FHSS, 8 channels per frame | ✅ | ❌ (LoRaWAN channel plan) | ❌ | ⚠️ — each retune is an AT cmd; realistic ≥100 ms hop | ✅ | ✅ on control leg |
| Adaptive SF ladder (per-frame) | ✅ | ❌ (ADR only, slow) | ❌ | ⚠️ — per-burst, not per-frame | ✅ | ✅ |
| Three-profile per-frame swap (control/telemetry/image PHY) | ✅ | ❌ | ❌ | ❌ — pick one PHY per session | ✅ | ✅ via dedicated radios |
| P0 ControlFrame preemption of P2/P3 image fragments | ✅ | ❌ | ❌ | ⚠️ — needs an `AT+TXABORT` if firmware exposes it | ✅ | ✅ (separate radios → no contention) |
| No-ACK / no-retry semantics | ✅ | ❌ (Class A confirmed/unconfirmed) | ❌ | ✅ | ✅ | ✅ |
| FCC §15.247 hopping argument intact | ✅ | n/a (LoRaWAN regional plan) | n/a | ⚠️ — slower hop, may force §15.249 path | ✅ | ✅ |

### 4.3 Engineering cost

| Cost axis | A | B | C | D | E | F |
|---|---|---|---|---|---|---|
| Code refactor in `tractor_h7.ino` + `lora_proto/*` | none | large (rip out crypto/FHSS, integrate LoRaWAN stack semantics) | large | **medium** (replace SPI HAL with AT driver; protocol layer mostly unchanged) | small (port instead of rewrite) | medium + extra board bring-up |
| Firmware flash on Murata required | no | no (stock) | no (stock + `AT+CLASS=C`) | **yes** (must source/flash P2P-capable AT firmware) | no | yes (for Murata leg) |
| BOM impact | n/a | none | none | none | **+1 SX1276 board, +antenna, +SPI wiring** | +1 SX1276 board + antenna |
| Regulatory re-cert risk | low | medium (different MAC) | medium | medium (lose per-frame FHSS argument) | low | low |
| Power impact | baseline | low (Class A) | **high** (RX always on) | baseline | baseline | baseline + extra radio |
| Time-to-bench-validate | n/a | days | days | **~1 day** if P2P firmware available | ~1 week (HW + driver) | ~1.5 weeks |

---

## 5. Verdict per method

### B — LoRaWAN Class A (stock MKRWAN)
**Reject.** Already excluded by [LORA_PROTOCOL.md "Why not LoRaWAN"](../DESIGN-CONTROLLER/LORA_PROTOCOL.md). RX1/RX2 windows make
sub-second downlink impossible; ADR/duty-cycle break the no-retry
control loop; LoRaWAN MAC supplants our crypto and frame format.

### C — LoRaWAN Class C
**Reject for control, possibly viable for telemetry/image only.** Class
C fixes the *downlink* latency problem (RX is always open, so
gateway→node ≈ TOA + UART tax). But the **uplink** path is still
LoRaWAN-confirmed/unconfirmed, the MAC layer still wraps everything,
and our AES-GCM/KISS/FHSS design is incompatible. Power draw also
roughly doubles vs Class A. Could host telemetry uplinks if we abandon
the custom protocol there too — large redesign.

### D — Raw P2P AT @ 115 200
**Best fit if a P2P-capable AT firmware exists for the Murata SiP.**
Preserves the entire `lora_proto.c` frame format, KISS, AES-GCM, replay
window. Requires:
1. Identifying / sourcing P2P AT firmware (Semtech `AT_Slave` ported to
   STM32L072 inside the Murata, or equivalent — bench `AT+VER?` first).
2. Flashing the Murata's STM32L0 (likely via the Murata's own programming pads or via the X8's GPIO if exposed). **This may not be field-flashable** — investigate before committing.
3. Relaxing the ControlFrame cadence from **20 Hz → 10 Hz** (slot 50 ms → 100 ms).
4. Reducing or removing per-frame FHSS hopping; switch to per-burst hopping.
5. Picking **one PHY profile per session** instead of per-frame swap; or accepting a profile-change handshake delay of ~50 ms.
6. Increasing L1 fragment cap from 25 ms → ~40 ms; or implementing P0 preempt via `AT+TXABORT` if exposed.

### E — External SX1276 on a routed SPI bus
**Best engineering fit; medium hardware cost.** Add an off-the-shelf
SX1276 module (e.g. RFM95W, Adafruit LoRa FeatherWing) to a free SPI
bus on the Max Carrier or H7 breakout. RadioLib + current `lora_proto.c`
work unchanged. All v25 budgets survive intact. Cost: BOM, antenna, and
mechanical integration. The Murata SiP becomes vestigial unless used
for the gateway/telemetry leg.

### F — Two-radio split
**Maximum robustness, highest BOM/engineering cost.** Use the external
SX1276 for the **control plane** (preserves all v25 invariants) and the
Murata in LoRaWAN-Class-C or P2P-AT mode for **telemetry / image**
(where 100 ms-class latency is acceptable). This also resolves the
P0-preempts-P3 contention that L1 fights against by giving each
priority class its own radio.

---

## 6. Recommended decision sequence (gates the next bench session)

1. **`AT+VER?` and `AT+DEV?` on Board1 + Board2** (run [`tools/at_probe.py`](../tools/at_probe.py)). Identify firmware family. *This single result decides D vs E.*
2. **If P2P AT firmware is present or flashable:** prototype Method **D**. Re-derive cadence (likely 10 Hz), measure UART+modem tax with `micros()` scope on `Serial3`, update L1/R-7 thresholds in [LORA_IMPLEMENTATION.md](../DESIGN-CONTROLLER/LORA_IMPLEMENTATION.md).
3. **If only LoRaWAN AT firmware is present and not flashable:** commit to Method **E** (external SX1276). The Murata stays present for future LoRaWAN telemetry but is not on the control path.
4. **If field reliability margin is needed regardless:** commit to Method **F**.

Method **B/C alone** should not be carried forward as the control transport.

---

## 7. Doc updates this decision will trigger (deferred until choice is made)

- [LORA_PROTOCOL.md](../DESIGN-CONTROLLER/LORA_PROTOCOL.md): cadence and FHSS sections.
- [LORA_IMPLEMENTATION.md](../DESIGN-CONTROLLER/LORA_IMPLEMENTATION.md): L1 (fragment cap), R-7 (retune cost), §3.1 / §3.2 (adaptive SF, FHSS), §4 (priority queue with new preempt mechanism).
- [MASTER_PLAN.md](../DESIGN-CONTROLLER/MASTER_PLAN.md): §8.17 LoRa interface revision; add an "AT-modem latency budget" row to the test program.
- [MASTER_TEST_PROGRAM.md](../MASTER_TEST_PROGRAM.md): new W4-pre AT-firmware-identification step before W4-00.
- [DECISIONS.md](../DESIGN-CONTROLLER/DECISIONS.md): new entry capturing the chosen method and rationale.
- [TODO.md](../TODO.md) and [DESIGN-CONTROLLER/TODO.md](../DESIGN-CONTROLLER/TODO.md): replace the "blocked on AT rewrite" wording with the concrete chosen path.

---

## 8. One-line summary

> **Method D (raw-P2P AT at 115 200) is the best fit *if* the Murata can be flashed with a P2P-capable AT firmware; otherwise Method E (external SX1276) is the only path that keeps the v25 protocol invariants intact. Method F is the robust insurance policy. Methods B and C cannot host the control plane.**

---

## 9. Bench update (2026-05-04, post-probe)

A live UART probe on both Portenta X8 boards (full results in
[2026-05-04_Murata_UART_Firmware_Probe_Bench_Results_Copilot_v1_0.md](2026-05-04_Murata_UART_Firmware_Probe_Bench_Results_Copilot_v1_0.md))
established:

- The Murata UART is wired and bidirectional at **19200 8N1** (not 8N2,
  not 115200).
- Reset via `gpio163` works.
- The currently-flashed firmware **does not respond to any known AT
  command set** (Hayes, MKRWAN, RAK, Semtech AT_Slave, RUI3 all
  rejected with the same `Error when receiving / +ERR_RX` URC).
- A board1↔board2 over-air smoke test produced **no received bytes** on
  the listener — the firmware does not even surface received LoRa
  frames on the UART.
- The literal firmware error string does not appear in the local
  Arduino tree, so this is a custom or vendor-customised binary that
  must be replaced.

**Implication for this comparison:** the Method D pre-condition
"P2P-capable AT firmware exists or is field-flashable" has shifted from
*unknown* to *known-false-without-physical-flash*. The next decision
gate is no longer `AT+VER?`; it is **physical SWD access to the Murata
SiP on the Max Carrier**. If SWD is exposed, Method D is still
viable but multi-day, with brick risk; if SWD is not exposed, Method E
becomes the only practical path forward and Method F is the
robust-but-expensive variant.

The decision-sequence in §6 is superseded by §8 of the bench-probe
note.

---

## 10. Bench update (2026-05-04, post-Arduino-doc reading)

**The §9 conclusion above is wrong about the SWD requirement.**

Arduino's official Max Carrier documentation
([TTN tutorial §2.1](https://docs.arduino.cc/tutorials/portenta-max-carrier/connecting-to-ttn),
[LoRa modem firmware update guide](https://support.arduino.cc/hc/en-us/articles/4405107258130-How-to-update-the-LoRa-modem-firmware))
states that the Murata `CMWX1ZZABZ-078` ships with a **stale proprietary
LoRaWAN-AT firmware that must be updated before first use** via the
`MKRWANFWUpdate_standalone` example sketch from the `MKRWAN` library
(with `#define PORTENTA_CARRIER` before `#include <MKRWAN.h>`). That
sketch reflashes the modem **over UART from the host H7**, using the
STM32L072's built-in system bootloader. **No SWD is needed.**

The `Error when receiving / +ERR_RX` URC we observed is exactly the
documented symptom of running the modem with its stale stock firmware
against a newer MKRWAN library / TTN regional plan.

**Revised cost picture:**

| Method | Cost in §4.3 above | Revised cost |
|---|---|---|
| D (raw-P2P AT) — Step 1: bring up LoRaWAN-AT first | "must source/flash P2P firmware" | **~1 hour** to flash known-good LoRaWAN-AT and confirm baseline |
| D — Step 2: replace with raw-P2P binary | included | days, but now optional and de-risked |
| E (external SX1276) | unchanged | unchanged |
| F (two-radio split) | unchanged | unchanged |

**Revised decision sequence (replaces §6):**

1. Flash `MKRWANFWUpdate_standalone` (with `#define PORTENTA_CARRIER`)
   on Board2 first via Arduino IDE / `arduino-cli` targeting
   `arduino:mbed_portenta:envie_m7`. Capture printed pre/post
   versions.
2. Re-run [`tools/at_probe.sh`](../tools/at_probe.sh) +
   [`at_probe2.sh`](../tools/at_probe2.sh). Expect `AT+VER?` →
   real version + `+OK`.
3. If green, repeat on Board1.
4. With a known-good LoRaWAN-AT modem on both boards, measure actual
   round-trip AT latency end-to-end (host → modem → over-air →
   peer-modem → peer-host) at 19200 8N1, and at 115200 8N1 if the new
   firmware exposes a baud-change command.
5. Plug the measured numbers into §3 / §4.1 of this note. The
   Method-B/C reject decision still stands on protocol grounds (LoRaWAN
   MAC, Class A windows, ADR), but Method **D** is now realistically
   on a single-day timeline rather than a multi-day SWD adventure.
6. If after the LoRaWAN-AT flash the modem **still** misbehaves, only
   then revive the SWD / external-SX1276 contingency.

Full bench-side details and the revised next-actions list are in
[2026-05-04_Murata_UART_Firmware_Probe_Bench_Results_Copilot_v1_0.md §7a / §8](2026-05-04_Murata_UART_Firmware_Probe_Bench_Results_Copilot_v1_0.md).


---

## 11. Method G — Custom firmware on the Murata STM32L072 (added 2026-05-04)

**Trigger:** User question — *"If we build our own custom firmware,
could we implement some of the items we had planned to implement with
SPI? like channel frequency hopping?"*

**Short answer: yes, all of them.** Full analysis is in
[2026-05-04_Murata_L072_Custom_Firmware_Method_G_Copilot_v1_0.md](2026-05-04_Murata_L072_Custom_Firmware_Method_G_Copilot_v1_0.md).

The Murata SiP's SX1276 SPI bus is *internal* to the package and
directly wired to the on-die STM32L072. The bus we couldn't reach from
the H7 is fully reachable from custom firmware running on the L072. The
same MKRWANFWUpdate_standalone bootloader-entry path that flashes the
Arduino LoRaWAN-AT binary will flash any binary we want.

### Scorecard delta vs. §4

| Capability | A (SPI/RadioLib) | D (raw-P2P AT) | E (external SX1276) | **G (custom L072 fw)** |
|---|---|---|---|---|
| 50 ms / 20 Hz cadence | ✅ | ⚠️ ~56 ms typ. | ✅ | ✅ |
| ≤25 ms per-fragment cap | ✅ | ⚠️ | ✅ | ✅ |
| Per-frame FHSS | ✅ | ⚠️ | ✅ | ✅ — register-level retune <1 ms |
| Adaptive SF + 3-profile per-frame swap | ✅ | ❌ | ✅ | ✅ |
| AES-GCM payload | ✅ | ✅ | ✅ | ✅ — moved into L072 |
| P0 preempt | ✅ | ⚠️ | ✅ | ✅ — direct radio reset ~100 µs |
| Custom 16 B ControlFrame | ✅ | ✅ | ✅ | ✅ |
| Host MCU SPI bus left free | ❌ consumed | ✅ | ❌ consumed | ✅ |
| Host↔radio latency floor | ~0.1 ms | ~10–20 ms | ~0.1 ms | **~1–2 ms** (UART, but binary, no AT) |
| Adds BOM cost | no | no | yes | **no** |
| Adds antenna / mech work | no | no | yes (per unit) | **no** |
| Engineering effort | impossible (HW) | 1 hr to weeks | 1–2 weeks | **4–6 weeks** |
| Brick risk | n/a | low | n/a | low *if* BOOT0/NRST control is preserved + safe-mode UART command included |
| FCC §15.247 hopping argument intact | ✅ | ⚠️ | ✅ | ✅ |

### How G compares to the prior leaders

- **vs. E (external SX1276):** G saves the BOM cost, the second
  antenna, and the per-unit mechanical integration, at the cost of
  4–6 weeks of L072 firmware work. E is the right call if shipping
  speed dominates; G is the right call if this is a long-lived
  platform.
- **vs. D (raw-P2P AT):** G removes the host↔modem AT tax entirely
  (binary COBS UART at 921600 baud → ~0.5 ms one-way for a 50 B
  frame), preserves all PHY tricks, and doesn't depend on finding a
  third-party P2P AT binary that may or may not exist.
- **vs. F (E + Murata-LoRaWAN-AT):** F is still useful as the
  *interim* state during the G build-out — Murata in LoRaWAN-AT for
  telemetry + external SX1276 for control gets us shipping while
  firmware is being written. F → G is a no-regret migration path
  because the same lora_proto.c runs in both.

### Recommended starting point

Fork [hardwario/lora-modem](https://github.com/hardwario/lora-modem)
(MIT, well-maintained Cube-HAL build, exposes a richer-than-stock AT
interface). Strip its LoRaMac stack, keep the SX1276 driver, RTOS, and
UART transport, drop in our lora_proto.c adapted for L072 RAM/Flash
(see resource budget in the focused note).

### Hybrid sequencing (replaces §10 decision sequence)

1. **Today (~1 h):** Flash MKRWANFWUpdate_standalone on Board2
   (Method D Step 1). Required for **any** future modem-firmware path
   because it qualifies the bootloader pipeline.
2. **Week 1:** Order an external SX1276 (RFM95W / Adafruit LoRa
   FeatherWing) as Method E/F insurance — long lead time runs in
   parallel.
3. **Week 1–2:** Build & flash unmodified hardwario/lora-modem on
   Board2 (G Phase 1). Re-confirms we can flash arbitrary binaries.
4. **Decision gate at end of week 2:** with (1) green, (3) green, and
   (2) on the bench, choose **F** (ship sooner, 2–3 wk effort) vs
   **G** (full v25 protocol on existing HW, 4–6 wk effort). F → G is a
   non-breaking migration if we go F first.
5. Methods **B and C remain rejected** for the control plane regardless.

### Updated one-line summary (overrides §8)

> **Method G (custom firmware on the Murata's internal STM32L072)
> recovers every SPI-era feature including per-frame FHSS, adaptive
> SF, three-profile PHY swap, P0 preempt, AES-GCM, custom frame
> format, and no-ACK semantics, with sub-millisecond host↔radio
> latency, zero added BOM, and near-zero brick risk if BOOT0/NRST
> control and a UART safe-mode command are designed in. Cost: 4–6
> weeks of focused L072 firmware work using hardwario/lora-modem
> as the fork point. The pragmatic sequencing is: flash Arduino
> LoRaWAN-AT (Method D Step 1) tomorrow → start a parallel
> hardwario/lora-modem fork → decide G vs F at the end of week 2
> with both options on the bench.**
