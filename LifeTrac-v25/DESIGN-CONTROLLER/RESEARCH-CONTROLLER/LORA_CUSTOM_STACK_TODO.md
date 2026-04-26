# LoRa Custom Stack — Implementation TODO

A working plan for building a custom LoRa + KISS protocol stack on SparkFun hardware (LoRa Thing Plus expLoRaBLE, Pro RF, **LoRa 1W Breakout 915M30S** + ESP32-S3 host, LoRa Gateway 1-Channel) for LifeTrac v25 telemetry and secondary command use.

> **Note on the previous SparkX Pro RF 1W (SPX-15836):** that board is **retired**. Its replacement is the SparkFun **LoRa 1W Breakout — 915M30S** (EBYTE E22-900M30S module, SX1262 + 1 W PA), which requires a separate host MCU (ESP32-S3 Thing Plus is the recommended pairing). All references below to "Pro RF 1W" should be read as "1W Breakout 915M30S + ESP32-S3 host."

For context and background, see [WIRELESS_OPTIONS.md](WIRELESS_OPTIONS.md), particularly:

- [Custom LoRa Stack vs XBee Baseline — Range and Throughput](WIRELESS_OPTIONS.md#custom-lora-stack-vs-xbee-baseline--range-and-throughput)
- [SparkFun LoRa Hardware deep-dive](WIRELESS_OPTIONS.md#sparkfun-lora-hardware-explorable-pro-rf-pro-rf-1w-lora-gateway)
- [MQTT Over Long-Range Radio (XBee / LoRa Bridges)](WIRELESS_OPTIONS.md#mqtt-over-long-range-radio-xbee--lora-bridges)

---

## Goals

A custom LoRa stack for LifeTrac must:

1. **Beat the XBee baseline** — ≥3× useful throughput at the long-range setting and ≥2× LoS range with the same regulatory class. (See the [throughput table](WIRELESS_OPTIONS.md#throughput-comparison) in WIRELESS_OPTIONS.md.)
2. **Bound worst-case latency** — primary command channel ≤200 ms round-trip, with hardware failsafe ≤500 ms after link loss.
3. **Bridge cleanly to MQTT** — telemetry frames must land on the existing v25 Mosquitto broker with topic-level fidelity (no protocol-private dashboard work).
4. **Be loss-tolerant** — no ACK/retry on the control path (latency kills retries); FEC on the telemetry path.
5. **Stay legal** — automatic EIRP cap based on configured antenna gain; default to FCC §15.247 limits.
6. **Be portable across the four SparkFun boards** with a pin-map header per board, no protocol changes.

## Non-Goals

- LoRaWAN compatibility (we're skipping the join/ADR/duty-cycle stack on purpose)
- Mesh routing (use Meshtastic for that — different firmware, same silicon)
- Primary joystick control link (use ELRS 900 MHz; LoRa is the *secondary* path)
- Encryption certification — AES-128 payload encryption is in scope, but no FIPS work

---

## Architecture

```
Application layer       │ joystick / telemetry / MQTT-SN topics
─────────────────────── │
Frame layer (KISS)      │ FEND/FESC byte stuffing, sequence #, CRC-16
─────────────────────── │
Session layer           │ link ID, AES-128-GCM, role (master/slave)
─────────────────────── │
MAC layer               │ "talk-when-quiet" CSMA, channel-busy detect
─────────────────────── │
PHY (RadioLib)          │ SX1276 / SX1262, configurable SF/BW/CR/TX power
```

The application layer carries one of two payload types:

- **Control frame** (16–24 bytes) — joystick axes (4 × int8), button bitmap (1 byte), heartbeat counter (1 byte), sequence (2 bytes), CRC (2 bytes). No retry, no ACK.
- **Telemetry frame** (32–128 bytes) — MQTT-SN PUBLISH wrapper carrying topic-ID + payload. QoS-0 by default, optional QoS-1 with single retry for config pushes.

---

## TODO List

### Phase 0 — Hardware bring-up

- [ ] Buy 1× SparkFun **LoRaSerial Kit** (pair of 915 MHz radios, pre-flashed transparent-UART firmware) — proves the link budget end-to-end before any custom firmware is written, and serves as a XBee-equivalent fallback
- [ ] Buy 2× SparkFun LoRa Thing Plus expLoRaBLE for handheld remote prototypes
- [ ] Buy 1× SparkFun LoRa Gateway 1-Channel (ESP32) for tractor-side bridge prototype
- [ ] Buy 1× SparkFun **LoRa 1W Breakout — 915M30S** + 1× SparkFun **Thing Plus ESP32-S3** (host MCU) for long-range link-budget testing
- [ ] Buy 4× 3 dBi 900 MHz vertical antennas (SMA-male, RP-SMA where required)
- [ ] Buy 2× LMR-240 SMA→RP-SMA pigtails (for Gateway → external antenna mount)
- [ ] Set up Arduino IDE / PlatformIO with [SparkFun Apollo3 board package](https://github.com/sparkfun/Arduino_Apollo3) and ESP32 Arduino core
- [ ] Verify all boards run a "hello world" sketch and basic LoRa send/receive with [`RadioLib`](https://github.com/jgromes/RadioLib)
- [ ] Document the working pin maps for each board (SPI, IRQ, RST, BUSY pins) in `lora_pinmap.h`

#### Opta integration approach (gateway-box pattern)

The Arduino Opta PLC has no native long-range radio. Rather than try to add a LoRa shield to the Opta directly, the recommended pattern is a **gateway box** alongside the Opta:

- **Gateway hardware:** ESP32-S3 Thing Plus + 1W Breakout 915M30S in a small DIN-rail or wall-mount enclosure, powered from the same 24 V supply as the Opta.
- **Opta ↔ Gateway link:** Modbus RTU over RS-485 (Opta has it natively) **or** Ethernet/MQTT if the Opta is the WiFi/Ethernet variant. Both are well-supported by the existing v25 controller code in [arduino_opta_controller/](arduino_opta_controller/).
- **Why a gateway box, not a shield:** keeps the safety-critical PLC firmware untouched, isolates the radio's RF noise from the Opta's I/O, allows the radio to be replaced/upgraded without re-certifying the PLC code, and lets the same gateway serve a fleet of Optas if that ever happens.
- **Mirrors the existing v25 architecture:** the Raspberry Pi already plays this gateway role for WiFi/MQTT; the LoRa gateway box is the same pattern, one tier further out.

### Phase 1 — Framing layer

- [ ] Implement KISS framer (FEND=0xC0, FESC=0xDB, TFEND=0xDC, TFESC=0xDD) — ~100 lines of C
- [ ] Add CRC-16/CCITT over the unstuffed frame
- [ ] Add 2-byte sequence number with rollover detection
- [ ] Unit-test the framer with hand-crafted boundary cases (frames containing FEND/FESC bytes, zero-length frames, max-length frames)
- [ ] Bench-test SF7/BW500 air time with `oscilloscope + GPIO toggle` to validate the 15–25 ms expectation

### Phase 2 — Control link

- [ ] Define `ControlFrame` struct (16 bytes packed) — see Architecture above
- [ ] Implement TX side: pack frame from joystick state, send at 20 Hz fixed rate
- [ ] Implement RX side: parse frame, validate CRC, drop on sequence-rollback or sequence-skip > N
- [ ] Implement watchdog: if no valid control frame in 250 ms → drive all valves to neutral (mirror v17.10 `WDT_vect` pattern, but at 250 ms not 8 s)
- [ ] Bench-test failsafe: power off TX while bench rig is running, confirm valves drop within 300 ms
- [ ] Measure end-to-end latency (TX joystick wiggle → RX relay state) with logic analyzer; target ≤150 ms RT
- [ ] Field-test range at SF7/BW500 with 3 dBi vertical at both ends; record packet loss vs distance to validate the [link-budget table](WIRELESS_OPTIONS.md#link-budget--distance-comparison)

### Phase 3 — MQTT-SN telemetry

- [ ] Choose gateway: [Eclipse RSMB](https://github.com/eclipse/mosquitto.rsmb) on the Pi (recommended), or `paho.mqtt-sn.embedded-c`
- [ ] Define topic-ID table (statically pre-registered): `lifetrac/v25/telemetry/gps`, `.../engine`, `.../battery`, `.../mode`, `.../errors`
- [ ] Implement MQTT-SN PUBLISH framing inside the KISS payload
- [ ] On the tractor-side Gateway 1-Channel ESP32: bridge LoRa KISS ⇆ MQTT-SN ⇆ MQTT (Mosquitto over WiFi) — no PC needed
- [ ] Verify telemetry shows up unchanged on the existing v25 web dashboard
- [ ] Stress-test: 5 Hz GPS + 1 Hz battery + 0.2 Hz status, run for 1 hour, confirm zero broker-side topic loss

### Phase 4 — Security

- [ ] Add AES-128-GCM payload encryption (key shared between TX and RX at provisioning)
- [ ] Add 4-byte nonce in the session header to prevent replay
- [ ] Verify encrypted-frame size still fits in a single SF7/BW500 packet (≤256 bytes)
- [ ] Document key provisioning procedure (BLE-pair the expLoRaBLE remote to the tractor on first boot, exchange keys over BLE, store in Apollo3 flash)

### Phase 5 — Forward error correction (telemetry only)

- [ ] Add Reed-Solomon (255,223) FEC on the telemetry path (control path stays uncoded for latency)
- [ ] Measure throughput cost of RS — expect ~12% overhead
- [ ] Field-test packet loss with and without FEC at SF8/BW250 and 5 km foliage; FEC should recover ~10–20% loss without retry

### Phase 6 — Multi-node / channel-hopping (optional)

- [ ] Add support for 2 LoRa modems on the tractor side, each on a different channel, for parallel TX/RX
- [ ] Implement simple round-robin scheduling across modems
- [ ] Validate ~2× throughput improvement on the bench

### Phase 7 — Regulatory compliance

- [ ] Implement EIRP cap: refuse to TX above (FCC max EIRP) − (configured antenna gain)
- [ ] Add a one-line config option `country = US|EU|...` that selects band + EIRP rules
- [ ] Document the test procedure for verifying compliance with a borrowed RF spectrum analyzer
- [ ] Apply for an FCC ID **only if** OSE distributes pre-built radio modules (DIY kits are exempt)

### Phase 8 — Field validation

- [ ] Bench latency: ≤150 ms RT at SF7/BW500 (target)
- [ ] Bench failsafe: ≤300 ms valve-drop after TX power off
- [ ] Field range: ≥10 km LoS at 100 mW (expLoRaBLE), ≥25 km LoS at 1 W (1W Breakout 915M30S + ESP32-S3 host)
- [ ] Field foliage: ≥3 km NLoS at 100 mW, ≥8 km NLoS at 1 W
- [ ] 24-hour soak test: zero unrecovered link drops, zero spurious failsafes

### Phase 9 — Documentation & release

- [ ] Write a Hookup Guide in [LifeTrac-v25/DESIGN-CONTROLLER/](.) covering wiring, antennas, and pairing
- [ ] Open-source the firmware under GPLv3 (matching ELRS / Meshtastic licensing norms)
- [ ] Add to the LifeTrac v25 BOM with SparkFun part numbers
- [ ] Cross-link this TODO from the v25 main [TODO.md](../TODO.md)

---

## Open Questions

- **AES key provisioning:** Pair-on-boot via BLE (only expLoRaBLE has BLE) vs USB-cable provisioning (works on all four boards). BLE is more elegant but locks us to the expLoRaBLE on the remote side.
- **Heartbeat rate:** 20 Hz costs ~5 kbps continuous. Could we drop to 10 Hz with a 200 ms watchdog instead of 250 ms? Tradeoff: bench-test perceived responsiveness.
- **Single-channel vs FHSS:** FCC §15.247 has different bandwidth/power rules for FHSS vs DTS. LoRa is DTS; we lose the slight FHSS advantage but avoid the hop-table complexity. Confirm with FCC application notes before pushing past 1 W.
- **Drop-in `LoRaSerial` firmware:** SparkFun ships the [**LoRaSerial Kit**](https://www.sparkfun.com/sparkfun-loraserial-kit-915mhz-1w.html) pre-flashed with [`LoRaSerial`](https://github.com/sparkfun/SparkFun_LoRaSerial) firmware as a ready-made transparent-serial replacement for XBee. **Phase 0 buys this kit first** so we have a working long-range link before any custom firmware exists, and so we can A/B-compare our custom stack against a known-good baseline. The same firmware ports cleanly to the 1W Breakout 915M30S + ESP32-S3 combo if we want to skip writing our own framer entirely.

---

## Risk Register

| Risk | Likelihood | Impact | Mitigation |
|---|---|---|---|
| Custom firmware introduces a control-path bug that causes runaway implement | Medium | **Critical** | Hardware E-stop independent of MCU; 250 ms watchdog; bench-test before every field session |
| FCC EIRP exceeded at 1 W with a 6 dBi+ antenna | Medium | High (regulatory) | Software EIRP cap; document antenna gain in config |
| LoRa bandwidth insufficient for combined control + telemetry + thumbnail | High | Medium | Reserve LoRa for control + telemetry only; thumbnails go over WiFi/cellular (see [VIDEO_OPTIONS.md](VIDEO_OPTIONS.md)) |
| SparkFun discontinues a board mid-project | Low | Medium | All four boards use commodity Semtech silicon; pin-map header keeps firmware portable |
| 1 W TX brown-outs the ESP32-S3 driving the 1W Breakout 915M30S | Medium | Low | LiPo with adequate C-rating; bulk capacitor on 3.3 V rail; consider separate buck regulator for the radio module |

---

## References

- [RadioLib documentation](https://github.com/jgromes/RadioLib/wiki)
- [SparkFun LoRaSerial firmware](https://github.com/sparkfun/SparkFun_LoRaSerial) — reference for transparent-serial mode
- [Meshtastic firmware](https://github.com/meshtastic/firmware) — reference for a mature LoRa stack with FEC, encryption, mesh
- [Eclipse RSMB (Really Small Message Broker)](https://github.com/eclipse/mosquitto.rsmb) — MQTT-SN ⇆ MQTT bridge
- [LoRa AirTime calculator](https://www.semtech.com/design-support/lora-calculator)
- [FCC 47 CFR §15.247](https://www.ecfr.gov/current/title-47/chapter-I/subchapter-A/part-15/subpart-C/section-15.247)
- [Tock OS LoRa driver on expLoRaBLE](https://www.alistair23.me/2023/08/09/lora-on-sparkfun-board/)
