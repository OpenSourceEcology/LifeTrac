# MicroTrac Controller v17.10 — Optimization Roadmap

> Companion to the v25 wireless deep-dive in
> [WIRELESS_OPTIONS.md](WIRELESS_OPTIONS.md#prior-ose-implementation-microtrac-controller-v1710-xbee)
> and the chassis description in
> [`Microtrac-v17.10/README.md`](../../Microtrac-v17.10/README.md).

This document catalogs **incremental upgrade options** for the existing
MicroTrac v17.10 wireless control system — the Arduino Uno + XBee-PRO 900HP
stack documented on the
[OSE wiki](https://wiki.opensourceecology.org/wiki/MicroTrac_Controller_v17.10).
Options are organized from **lowest-effort / lowest-cost** (firmware tweaks,
no new hardware) up to **full architectural replacement**, with the
expected latency, reliability, and cost impact of each step so they can be
mixed and matched.

The end-to-end target metric is **button-to-relay latency** at the operator's
hand. The v17.10 baseline we measured during bench testing was roughly
**150–250 ms typical** with occasional **>500 ms** spikes when retries
fired — well above the ~50 ms threshold below which a tracked vehicle feels
"direct" to drive.

---

## Table of Contents

1. [Baseline (v17.10 As-Built)](#baseline-v1710-as-built)
2. [Tier 1 — Firmware & Radio-Settings Tweaks (No New Hardware)](#tier-1--firmware--radio-settings-tweaks-no-new-hardware)
3. [Tier 2 — Same Radios, Smarter Protocol](#tier-2--same-radios-smarter-protocol)
4. [Tier 3 — Microcontroller Upgrade (Keep XBee)](#tier-3--microcontroller-upgrade-keep-xbee)
5. [Tier 4 — Radio Hardware Swap (Keep MCU)](#tier-4--radio-hardware-swap-keep-mcu)
6. [Tier 5 — Full Replacement (LifeTrac v25 Architecture)](#tier-5--full-replacement-lifetrac-v25-architecture)
7. [Cumulative Improvement Chart](#cumulative-improvement-chart)
8. [Recommended Upgrade Paths](#recommended-upgrade-paths)
9. [References](#references)

---

## Baseline (v17.10 As-Built)

| Component | Spec |
|-----------|------|
| Remote MCU | Arduino Uno (ATmega328P, 16 MHz, 8-bit) |
| Tractor MCU | Arduino Uno + relay shield (4 ch) |
| Radio (both ends) | Digi XBee-PRO 900HP (XBP9B-DPST-001) on Wireless SD shield |
| Serial baud | 9600 |
| RF data rate | 10 kbps (XBee default profile) |
| Mode | Transparent (AT) mode, broadcast addressing |
| Packet format | 1-byte ASCII command per state change, no sequence/ack at app layer |
| Watchdog | 8 s software stop on loss-of-signal |
| Inputs | Momentary push-buttons (binary on/off) |

**Measured behavior:**

- Typical button-to-relay latency: **~180 ms**
- Worst-case (single retry): **~350–500 ms**
- Failure mode of concern: dropped "release" byte → relay stays energized
  until next state change or watchdog fires

**Root causes of the latency budget** (see WIRELESS_OPTIONS.md §3 for full breakdown):

| Stage | Typical contribution |
|-------|----------------------|
| Button debounce / poll loop on Uno | 10–30 ms |
| Serial TX at 9600 baud (1–8 byte msg) | 1–10 ms |
| XBee packetization timeout (`RO=3` chars @ 9600) | ~3 ms |
| XBee RF air time (transparent, 10 kbps, ACK on) | 30–80 ms |
| Receiver re-serialize at 9600 baud | 1–10 ms |
| Tractor Uno parse + relay drive | 5–15 ms |
| Mechanical relay actuation | 5–10 ms |
| **Total typical** | **~150–250 ms** |
| **Total with one retry** | **+50–100 ms** |

---

## Tier 1 — Firmware & Radio-Settings Tweaks (No New Hardware)

Cost: **$0**. Time: **a few hours of bench work + XCTU.**
These changes alone typically cut latency by **3–5×** and dramatically
tighten worst-case jitter.

### 1.1 Raise the serial baud

- Reconfigure both XBees with `BD = 7` (115 200 baud) via XCTU.
- Update both Arduino sketches to `Serial.begin(115200)`.
- Per-byte serial time drops from ~1 ms to ~0.087 ms.

**Expected gain:** −5 to −15 ms; eliminates serial as a bottleneck.

### 1.2 Tune the packetization timeout (`RO`)

- Default `RO = 3` (3 character times) makes the XBee wait for a quiet
  serial line before sending. With short fixed-length commands, set
  `RO = 0` so the packet ships on the first byte, or send a fixed-length
  framed message (e.g., 4 bytes incl. checksum) and rely on the radio's
  internal length field.

**Expected gain:** −3 to −10 ms typical, less jitter.

### 1.3 Switch the RF data rate profile

- The 900HP supports 10 kbps (long range, ~–101 dBm RX sensitivity) and
  200 kbps (shorter range, ~–96 dBm). For ranges under ~1 km LoS the
  200 kbps profile cuts air time by **~20×** at the cost of ~5 dB link
  budget — still hundreds of meters in practice with stock antennas.
- Set `BR = 1` (200 kbps) on both radios.

**Expected gain:** −20 to −60 ms (the largest single-step improvement).

### 1.4 Disable retries for time-critical commands; add app-layer state

- In transparent mode you can disable MAC retries (`RR = 0`,
  `MT = 0`) to bound worst-case latency.
- Compensate at the application layer: send the **current button state**
  on a 20–50 Hz timer rather than just on edges. A dropped packet is
  recovered by the next periodic frame instead of retried by the radio.

**Expected gain:** Eliminates the +50–100 ms retry tail.
Worst-case drift bounded to one period (20–50 ms).

### 1.5 Tighten the input loop

- Replace any `delay()` calls in the remote sketch with a non-blocking
  `millis()` scheduler.
- Use hardware pin-change interrupts for button edges, debounce in
  software with a 5–8 ms window.

**Expected gain:** −10 to −25 ms, removes "sticky" feel.

### 1.6 Add a fast-stop deadman

- Reserve one bit in the periodic frame as a hardware deadman driven by a
  dedicated grip switch. On the tractor side, a missing or zero deadman
  bit triggers immediate relay-off **before** parsing the rest of the frame.
- Drop the watchdog from 8 s to **300–500 ms**; the periodic frame
  guarantees you'll see updates more often than that.

**Expected gain:** Safety, not latency — failsafe goes from "8 s of
runaway" to "half a second of runaway" worst case.

**Tier 1 cumulative target: ~30–60 ms typical, <100 ms worst case.**

---

## Tier 2 — Same Radios, Smarter Protocol

Cost: **$0–$50** (optional small OLED / RSSI LED on remote).
Time: **a weekend of firmware work.**

### 2.1 Switch XBee to API mode

- API mode (`AP = 1` or `AP = 2`) wraps every TX/RX in a framed packet
  with addresses, frame ID, RSSI, and ACK status. You get:
  - Per-packet RSSI for free (the `DB` AT command or the API RX frame).
  - Explicit TX status frames so the remote knows when a frame failed.
  - Easier multi-node routing if a relay node is ever added.
- Latency cost is essentially zero vs. transparent at the same `RO`/`BR`.

### 2.2 Define a binary protocol with sequence + checksum

```
[STX][SEQ][BTNS_LO][BTNS_HI][DEADMAN][CRC8]   // 6 bytes, 50 Hz
```

- 16-bit button bitmap covers all current and future inputs.
- 1-byte sequence number lets the receiver detect drops and reordering.
- CRC-8 (Dallas/Maxim, polynomial 0x31) catches single-bit RF errors
  cheaply on an Uno.

### 2.3 Bidirectional telemetry on the same link

- Send a small status frame (RSSI, watchdog state, relay states, error
  codes, optional GPS) from tractor → remote at 5–10 Hz.
- Display link quality on an OLED or a 4-LED bargraph on the remote so
  the operator sees degraded link **before** control becomes unsafe.

### 2.4 Graceful degradation

- If RSSI < threshold for >250 ms, restrict the tractor to "creep"
  speed by limiting which relays may be actuated.
- Below a second threshold or on total link loss, hard stop.

**Tier 2 cumulative target: ~25–45 ms typical, deterministic worst case,
operator awareness of link health.**

---

## Tier 3 — Microcontroller Upgrade (Keep XBee)

Cost: **$10–$40 per end** (vs. ~$25 for an Uno).
Time: **a weekend to port firmware.**

The Uno's 8-bit core, 2 KB SRAM, single hardware UART, and lack of
hardware float make several v17.10 limitations structural rather than
fixable in firmware. Modern boards remove those limits without
abandoning the existing XBee or wiring harness.

| Board | MCU | Why It Helps | Approx. Cost |
|-------|-----|--------------|--------------|
| **Arduino Nano Every** | ATmega4809 @ 20 MHz, 6 KB SRAM, 4 UARTs | Drop-in pin-compatible replacement, frees `Serial` for debug while XBee uses `Serial1`. | $13 |
| **Teensy 4.0** | ARM Cortex-M7 @ 600 MHz, 1 MB RAM | Microsecond-deterministic loop, plenty of RAM for buffering and telemetry logging. Overkill but cheap. | $24 |
| **Arduino MKR series** | SAMD21 @ 48 MHz, 32 KB SRAM | Native 3.3 V (matches XBee logic levels — removes the 5 V→3.3 V level shifter on the Uno). | $30–$40 |
| **ESP32 (DevKit / WROOM)** | Dual-core Xtensa @ 240 MHz, 520 KB SRAM, Wi-Fi + BLE | Free upgrade path to MQTT, BLE, OTA firmware updates; XBee on UART2 while Wi-Fi handles telemetry/logging. | $5–$10 |
| **Raspberry Pi Pico W** | Dual-core Cortex-M0+ @ 133 MHz, 264 KB SRAM, Wi-Fi | Cheap, two PIO blocks for tight I/O timing, Wi-Fi for parallel telemetry. | $6 |

**Direct latency win is small** (Uno isn't actually the slow link), but
secondary benefits matter:

- **Remove level shifter** on MKR/Pico/ESP32 → fewer parts, less ringing.
- **Hardware UART for XBee + USB-serial for debug** in parallel.
- **Floating-point math** on ARM/ESP32 → trivial RSSI smoothing, EMA filters,
  curve-shaped joystick mapping when analog sticks are added.
- **Wi-Fi or BLE side channel** on ESP32/Pico W → operator can monitor
  link health from a phone without touching the XBee path.

**Expected gain:** −5 to −10 ms typical, plus a foundation for analog
inputs, OTA updates, and parallel transports.

---

## Tier 4 — Radio Hardware Swap (Keep MCU)

Cost: **$20–$200 per end.**
The single largest latency lever after Tier 1. Choose based on the
control surface: discrete buttons can stay on XBee/LoRa with Tier 1+2
optimizations; analog joysticks really want an RC-style link.

### 4.1 XBee 900HP @ 200 kbps + API mode (Tier 1+2 done well)

- Already covered above. Realistic floor is **~25 ms typical** with stock
  hardware. Cannot go lower without changing the radio.

### 4.2 LoRa point-to-point (RFM95W, E22, Heltec, etc.)

- 868/915 MHz, ~$10–$20/module, range comparable to XBee-PRO at SF7.
- **Latency: 30–80 ms** at SF7/BW500 — *not* an improvement over a
  well-tuned XBee for control, but cheaper and FOSS-friendly.
- Best as a **secondary telemetry channel**, not a primary control link.

### 4.3 nRF24L01+ / nRF52 ESB

- 2.4 GHz, ~$3/module. Air-time per 32-byte payload at 2 Mbps is
  **~150 µs**; full round-trip with auto-ack typically **2–5 ms**.
- Range realistically **100–500 m LoS** with PA+LNA modules ($8 each).
- Great fit if the tractor is always within sight on a worksite.
- **Expected end-to-end latency: 8–15 ms.**

### 4.4 ExpressLRS (ELRS) 900 MHz or 2.4 GHz

- Open-source, designed for RC, runs on STM32/ESP32 modules ($25–$60).
- Packet rates **50/100/250/333/500 Hz** selectable at runtime.
- Native CRSF telemetry back-channel; mature failsafe behavior.
- **Expected end-to-end latency: 4–20 ms**, range comparable to or better
  than XBee-PRO depending on antenna and band.
- Drop-in for joystick/proportional control once the v17.10 sketch is
  ported to read CRSF channels instead of polling buttons.

### 4.5 Wi-Fi (ESP-NOW / UDP)

- ESP-NOW peer-to-peer between two ESP32s: **3–8 ms typical** at short
  range, no router/AP required.
- Wi-Fi station + UDP via a Pi/router: **5–20 ms** typical, more
  jitter, range limited by AP placement.
- Excellent on-site, weak under heavy 2.4 GHz interference, no real
  long-range story without external APs/antennas.

### 4.6 Comparison of radio swaps

| Radio | End-to-end latency | Range (LoS) | Cost / pair | Notes |
|-------|-------------------|-------------|-------------|-------|
| XBee-PRO 900HP, optimized | 25–45 ms | ~10 km | $250 | Drop-in, no rewire |
| LoRa SX1276 (RFM95W) | 30–80 ms | 5–10 km | $30 | Better as telemetry |
| nRF24L01+ PA/LNA | 8–15 ms | 100–500 m | $20 | Cheapest fast option |
| ESP-NOW (2× ESP32) | 3–8 ms | 100–300 m | $15 | Forces MCU swap too |
| ExpressLRS 900 MHz | 4–20 ms | 5–15 km | $80–$120 | Best fit for joysticks |
| ExpressLRS 2.4 GHz | 4–10 ms | 1–3 km | $60–$100 | Smaller antennas |

---

## Tier 5 — Full Replacement (LifeTrac v25 Architecture)

Cost: **$300–$600**. Time: **rebuild remote + tractor controller.**
This is what the LifeTrac v25 program is doing in parallel — see
[WIRELESS_OPTIONS.md](WIRELESS_OPTIONS.md)
for the full design rationale and the rest of this
[DESIGN-CONTROLLER/](.) folder for the implementation.

Key elements that make the v25 stack qualitatively better than v17.10
even after Tier 1–4 patches:

- **Arduino Opta** industrial PLC on the tractor (IP20, 24 V native,
  proper relay outputs, M4+M7 dual-core, Ethernet + RS-485 + Wi-Fi/BLE).
- **ELRS or BLE primary control link** with **MQTT-over-Wi-Fi/Ethernet**
  fallback when on-site infrastructure exists.
- **Three input modalities** (BLE DroidPad, ESP32 handheld, Pi web UI)
  selectable by a hardware mode switch — no firmware reflash to swap.
- **Proportional flow valves** instead of bang-bang relays, so the
  control link bandwidth and resolution actually buy you smoother motion.
- **CI-tested firmware** (`.github/workflows/arduino-ci.yml`) and
  documented failsafe behavior.

**Expected end-to-end latency: 4–25 ms** depending on input modality;
proportional control replaces on/off buttons; field-serviceable industrial
hardware throughout.

---

## Cumulative Improvement Chart

Bars are **typical** button-to-relay latency. Each tier assumes the
previous tier is also applied. Values are bench-test estimates — measure
your own system before claiming numbers in the field.

```
Tier 0 — Baseline v17.10 as-built
  ████████████████████████████████████████████████  ~180 ms typical
                                                    ~350–500 ms worst

Tier 1 — Firmware + XBee settings (BD=115200, RO=0, BR=200 kbps,
         non-blocking loop, 50 Hz periodic frames)
  ████████████  ~45 ms typical, ~80 ms worst

Tier 2 — XBee API mode + binary framed protocol w/ CRC + telemetry +
         graceful degradation
  ██████████  ~35 ms typical, deterministic worst case

Tier 3 — Drop-in MCU upgrade (Nano Every / ESP32 / Teensy)
  █████████  ~30 ms typical
  (small direct gain; large secondary benefits)

Tier 4a — Same MCU, swap XBee → ExpressLRS 900 MHz
  ████  ~12 ms typical, similar range

Tier 4b — Same MCU, swap XBee → nRF24L01+ PA/LNA (short range only)
  ███  ~10 ms typical, ≤500 m range

Tier 4c — ESP32 + ESP-NOW (forces Tier 3 too)
  ██  ~6 ms typical

Tier 5 — LifeTrac v25 architecture (Opta + ELRS / BLE / MQTT)
  ██  ~4–25 ms typical, proportional control, multi-modal inputs
```

### Latency vs. cost (paired ends)

| Cumulative tier | Typical latency | Hardware delta vs. v17.10 |
|-----------------|----------------|---------------------------|
| Tier 0 (baseline) | ~180 ms | $0 |
| + Tier 1 (firmware + radio settings) | ~45 ms | $0 |
| + Tier 2 (smarter protocol) | ~35 ms | $0–$50 (optional OLED) |
| + Tier 3 (MCU upgrade) | ~30 ms | $20–$60 |
| + Tier 4a (ELRS 900 MHz) | ~12 ms | +$80–$120 |
| + Tier 4b (nRF24L01+ short range) | ~10 ms | +$20 |
| + Tier 4c (ESP-NOW, supersedes Tier 4a/b) | ~6 ms | +$15 |
| + Tier 5 (v25 Opta architecture) | ~4–25 ms | +$300–$600 |

### Reliability/feature improvements per tier

| Tier | Latency floor | Worst-case bounded? | Link-quality awareness | Proportional control | Failsafe response |
|------|--------------|---------------------|------------------------|----------------------|-------------------|
| 0 | ~180 ms | ❌ retry tail | ❌ | ❌ | 8 s watchdog |
| 1 | ~45 ms | ✅ via periodic frames | ❌ | ❌ | 300–500 ms |
| 2 | ~35 ms | ✅ | ✅ RSSI/OLED | ❌ | tiered (creep → stop) |
| 3 | ~30 ms | ✅ | ✅ + side channel | ⚠️ possible w/ analog inputs | ditto |
| 4 (any) | 4–20 ms | ✅ | ✅ CRSF telemetry | ✅ practical | radio-native failsafe |
| 5 | 4–25 ms | ✅ | ✅ multi-channel | ✅ flow valves | industrial PLC |

---

## Recommended Upgrade Paths

Pick based on goals and budget:

- **"I just want it to feel responsive, today, for free."**
  → **Tier 1 only.** A few hours in XCTU + sketch edits gets you from
  ~180 ms to ~45 ms with no parts. This alone makes the existing system
  usable for slow utility work.

- **"I want it safe and observable, still cheap."**
  → **Tier 1 + Tier 2.** Adds RSSI-aware degradation, deterministic
  failsafe, and an operator-visible link-quality readout for under $50.

- **"I want analog joysticks and modern firmware, on the existing radios."**
  → **Tier 1 + Tier 2 + Tier 3.** Swap the Unos for ESP32 (or MKR if you
  prefer the Arduino IDE story) and add proportional inputs; keep XBee.

- **"I want it to feel like an RC vehicle."**
  → **Tier 1 + Tier 3 + Tier 4a (ELRS 900 MHz).** ~$200 total upgrade,
  10× latency improvement, mature failsafe, room to grow.

- **"I'm rebuilding from scratch anyway."**
  → **Tier 5 / adopt the LifeTrac v25 stack** and inherit all of the
  ongoing v25 work (multiple input modalities, Opta firmware,
  proportional valves, CI). Use the v17.10 chassis and hydraulics as the
  test bed.

---

## References

- [MicroTrac Controller v17.10 (OSE wiki)](https://wiki.opensourceecology.org/wiki/MicroTrac_Controller_v17.10)
- [MicroTrac Controller v17.10 / Code (OSE wiki)](https://wiki.opensourceecology.org/wiki/MicroTrac_Controller_v17.10/Code)
- [Digi XBee-PRO 900HP datasheet](https://www.digi.com/resources/documentation/digidocs/pdfs/90002173.pdf)
- [LifeTrac v25 wireless options analysis](WIRELESS_OPTIONS.md)
- [LifeTrac v25 controller folder](.)
- [ExpressLRS project](https://www.expresslrs.org/)
- [`Microtrac-v17.10/README.md`](../../Microtrac-v17.10/README.md)
