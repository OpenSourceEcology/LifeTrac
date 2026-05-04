# 03 — Bring-up Roadmap

**Date:** 2026-05-04
**Status:** Plan — concrete, phased, no calendar deadlines (project is explicitly not time-constrained per [00 §1](00_DECISION_Method_G_Commitment.md)).
**Author:** GitHub Copilot (Claude Opus 4.7)

---

## How to read this

Phases are gated, not dated. Each phase has explicit **entry criteria**, **deliverables**, and **exit criteria**. Don't start phase N+1 until phase N's exit criteria are green. Within a phase, work items can run in parallel.

Capability IDs (R-XX, N-XX) reference [01 Capabilities Analysis](01_Capabilities_Analysis_Custom_Firmware.md).

---

## Phase 0 — Qualify the bootloader pipeline (prerequisite for everything)

**Entry:** Both bench boards present; UART probe results captured (already done — see [the bench-results note](../../AI%20NOTES/2026-05-04_Murata_UART_Firmware_Probe_Bench_Results_Copilot_v1_0.md)).

**Work items:**
- W0-1: Install Arduino IDE and the `arduino:mbed_portenta` core (or set up `arduino-cli` equivalent) on the dev workstation.
- W0-2: Install the `MKRWAN` library (v1, *not* MKRWAN_v2).
- W0-3: Open `MKRWANFWUpdate_standalone` example, add `#define PORTENTA_CARRIER` before `#include <MKRWAN.h>`.
- W0-4: Set BOOT DIP switch SW1 = OFF on Board2's Max Carrier.
- W0-5: Compile for `arduino:mbed_portenta:envie_m7`, upload to Board2.
- W0-6: Open serial monitor, capture pre/post version strings.
- W0-7: Re-run [`tools/at_probe.sh`](../../tools/at_probe.sh) and [`at_probe2.sh`](../../tools/at_probe2.sh) — confirm `AT+VER?` returns a real version + `+OK`.
- W0-8: Repeat W0-3..W0-7 on Board1.

**Exit criteria:**
- Both boards respond to `AT+VER?` with a sensible version + `+OK`.
- Captured pre/post versions documented in the [bench-results note](../../AI%20NOTES/2026-05-04_Murata_UART_Firmware_Probe_Bench_Results_Copilot_v1_0.md).
- Confidence: we can flash arbitrary binaries via the H7 ↔ STM32L072 ROM-bootloader UART path.

**Why this still happens despite committing to G:** the *flashing pipeline* the Arduino sketch uses is exactly the one our custom firmware will rely on for development and field updates (N-25). Confirming it on stock-vendor binary first means any later flash failure is a regression caused by our binary, not a hardware/wiring problem.

---

## Phase 1 — Hello-world & brick-resistance foundation

**Entry:** Phase 0 green. Toolchain (`arm-none-eabi-gcc` 13.2 + Make + optionally PlatformIO) installed. Repository folder [`../firmware/murata_l072/`](../firmware/) created with the layout from [02 §2](02_Firmware_Architecture_Plan.md) (skeleton files only).

**Work items:**
- W1-1: Minimal `main.c` that toggles a debug GPIO at 1 Hz and emits `"L072 hello v0.0.1\r\n"` on USART2 at 921600 baud once per second. No HAL beyond clock + GPIO + USART.
- W1-2: Linker script `stm32l072cz_flash.ld` matching [02 §4.3](02_Firmware_Architecture_Plan.md) (single slot for now; A/B comes in Phase 6).
- W1-3: Makefile producing a stripped `firmware.bin` ready for the STM32 ROM bootloader UART protocol.
- W1-4: Helper script (PowerShell-friendly) that drives the H7 to: (a) hold BOOT0 high, (b) pulse NRST, (c) speak STM32 ROM-bootloader protocol over `Serial3`, (d) flash `firmware.bin`. Reuse the open-source `stm32flash` source as reference; we want this working *without* needing the Arduino sketch.
- W1-5: Implement **N-22 UART safe-mode listener** as the very first thing `main()` does — listen on USART2 for a fixed magic byte sequence for 500 ms; on match, call `__set_MSP(...)`-and-jump-to ROM bootloader at `0x1FF00000`. Test by sending the magic and verifying we re-enter the bootloader without needing BOOT0/NRST cycling.
- W1-6: Implement **N-21 H7 hard-reset path** smoke test — H7 toggles NRST, L072 reboots, prints hello again.
- W1-7: Implement **N-17 register dump** as a temporary debug command — read all SX1276 registers via SPI1, print them as hex. Confirms the SiP's internal SPI bus actually works from custom firmware. **This is the moment we definitively prove Method G is viable.**
- W1-8: Confirm Q4 from [02 §8](02_Firmware_Architecture_Plan.md): is DIO3 routed?

**Exit criteria:**
- Custom binary boots, prints hello, dumps SX1276 registers (non-zero values matching datasheet defaults).
- Safe-mode magic recovers any "bricked" custom binary without external hardware.
- H7 NRST-driven reset is reliable.
- Flashing tooling works without the Arduino sketch.

**Risk gate:** If W1-7 returns all zeros or the SPI bus is unresponsive, **stop**. That would mean the SX1276 SPI bus on this revision of the Murata SiP isn't where we think it is, and Method G needs hardware re-investigation before going further. (We do not expect this — the same SPI is used by all known vendor firmware — but we explicitly check.)

---

## Phase 2 — Single-frame TX/RX & host transport

**Entry:** Phase 1 green; the L072 SPI bus to the SX1276 is confirmed working from our binary.

**Work items:**
- W2-1: Port a minimal SX1276 driver into `radio/sx1276.c` (forked from the Semtech reference or the slimmer copy inside `hardwario/lora-modem`). Init, set frequency, set SF/BW/CR, set TX power, write FIFO, trigger TX, RX continuous, FIFO read.
- W2-2: Hello-world TX: transmit `"hello"` at SF7/BW250/CR4-5 on a fixed channel from Board1; receive on Board2 via raw `radio/` API; print payload + RSSI/SNR. **First over-air bytes from our firmware.**
- W2-3: Implement `host/host_uart.c` with COBS framing per [04 Hardware Interface](04_Hardware_Interface_and_Recovery.md). RX via DMA-on-IDLE on USART2, TX via blocking-then-DMA-once-busy.
- W2-4: Implement initial host command set: `PING`, `VER`, `UID` (N-29), `RESET`, `TX_RAW`, `RX_RAW_URC`, `REG_READ`, `REG_WRITE`, `STATS_RESET`, `STATS_DUMP`. Document each in [04](04_Hardware_Interface_and_Recovery.md).
- W2-5: Port v25 frame format (`proto/lora_proto.c`) — ControlFrame layout, telemetry layout, fragment layout, header structure.
- W2-6: Port AES-GCM (`proto/crypto_gcm.c`) using tinycrypt initially. Wire `proto/replay.c` with the existing v25 nonce/replay-window scheme.
- W2-7: Implement **N-13** — derive per-frame GCM nonce from L072 hardware RNG.
- W2-8: Implement **N-20** — IWDG independent watchdog with a sensible window (e.g. 100 ms) and `iwdg_kick()` calls in the main loop.
- W2-9: Implement **R-11 / failsafe** heartbeat watchdog (separate from IWDG; tracks H7 liveness via UART activity).

**Exit criteria:**
- Two boards exchange a v25-formatted ControlFrame end-to-end, AES-GCM-encrypted, replay-window enforced.
- The H7 can drive a TX from start to finish via the COBS-framed UART protocol; the receiving H7 sees the decoded URC.
- Host PC unit tests for `proto/` and `host/cobs` pass.
- The IWDG fires correctly when the main loop is intentionally hung.

---

## Phase 3 — All recovered (R-XX) features

**Entry:** Phase 2 green; we can transmit and receive a v25 frame.

**Work items (mandatory, R-XX from [01 §1](01_Capabilities_Analysis_Custom_Firmware.md)):**
- W3-1 (R-01): Per-frame FHSS — implement hop-table generator, validate retune cost ≤1 ms.
- W3-2 (R-02): Adaptive SF — per-frame SF selection driven by host-supplied or firmware-decided value.
- W3-3 (R-03): Three-profile per-frame PHY swap — control / telemetry / image profiles with zero-cost switching.
- W3-4 (R-04): P0 preempt — `RADIO_RESET` API, measured latency target ≤5 ms request-to-air-quiet.
- W3-5 (R-08, R-09, R-10): 50 ms slot scheduler with priority queue, `≤25 ms` per-fragment cap, `≤5 ms` retune accounting.
- W3-6 (N-15, recommended in launch set): Per-frame radio metadata in the URC.
- W3-7: Spectrum-bench test fixture (single board + RTL-SDR or HackRF) to verify FHSS dwell statistics, frequency error, and TX power across all channels.

**Exit criteria:**
- Two-board HIL run: 1 hour at 20 Hz cadence; PER < 1% on a clean bench; per-fragment airtime histogram shows ≤25 ms p99; per-frame retune histogram shows ≤5 ms p99.
- P0-preempt latency p99 ≤ 5 ms (1000 induced preemptions).
- Spectrum capture confirms all 8 (or eventual ≥50) channels actually carry traffic in expected proportions.
- Three-profile swap round-trip works under the host-driven test harness.

---

## Phase 4 — Strongly-recommended new (N-XX) features

**Entry:** Phase 3 green; v25 protocol working end-to-end on custom firmware.

**Work items:**
- W4-1 (N-04): Listen-before-talk — CCA before P0 ControlFrame TX, configurable threshold per channel, exposed in stats.
- W4-2 (N-06): Channel-quality-aware FHSS — per-channel quality store (RSSI floor, recent CRC pass rate, recent CCA-busy rate), weighted hop selection that satisfies the regulatory uniformity constraint.
- W4-3 (N-07): Per-frame TX-power adaptation — RX-side carries a target-TX-power hint back; TX side honors it within configurable bounds.
- W4-4 (N-19): RSSI-watchdog fault flag.
- W4-5 (N-24): Last-frame replay on link recovery.
- W4-6 (N-08, opt): Symbol-level CR hopping if it makes scoreboard improvements.
- W4-7: HIL test rig expansion — interference injection (a third radio TXing on a chosen channel) to verify N-04/N-06/N-19 actually do what we say.
- W4-8: Resolve [02 §8 Q5](02_Firmware_Architecture_Plan.md) — produce a validated regulatory channel list for the project's intended deployment region(s).

**Exit criteria:**
- LBT enabled: PER on an intentionally-busy bench drops by ≥30%; LBT-aborted-TX count visible in stats.
- Quality-aware FHSS demonstrably avoids a deliberately-jammed channel within ~10 s of jam onset.
- TX-power adaptation: bench-measured average TX power drops by ≥3 dB at 1 m link distance vs. fixed-max.
- All `R-1..R-7`, `L1`, `L3`, `L-V11` HIL requirements green per [00 §7](00_DECISION_Method_G_Commitment.md).

This is the **launch-quality milestone.** The firmware at the end of Phase 4 is a credible v25-launch candidate.

---

## Phase 5 — Power & autonomous operation

**Entry:** Phase 4 green and stable for 24 h on a soak run.

**Work items:**
- W5-1 (N-09): Deep-sleep RX-window scheduler. STOP mode @ ~1.5 µA between windows; RTC alarm + DIO0 wake.
- W5-2 (N-10): Scheduled wake-on-LoRa beacon pattern.
- W5-3 (N-11): Whole-modem deep sleep with H7-driven wake.
- W5-4 (N-30): Autonomous emergency beacon on failsafe-trip — independent of H7 state.
- W5-5 (N-23): Brown-out detection + persistent-buffer fault record.
- W5-6 (N-16): Histogram counters dumped via `STATS_DUMP`.
- W5-7 (N-31, opt): LoRa-only "wake-up tractor" command — L072 GPIO asserts H7/X8 power-rail enable.
- W5-8 (N-34, opt): Time-sync broadcast.

**Exit criteria:**
- Handheld power profile: idle current with deep-sleep enabled ≤10 mA average (H7 + L072 combined) over a 1 h trace.
- Emergency beacon TX'd autonomously when H7 UART goes silent; confirmed received by a second board on the beacon channel.
- 7-day soak with random link drops, brown-out events, and forced resets shows no memory growth, no missed beacons.

---

## Phase 6 — Field-update & manufacturing readiness

**Entry:** Phase 5 green; firmware is in v25-deployment-ready territory.

**Work items:**
- W6-1 (N-25): H7-driven field update — a documented procedure where the H7 reflashes the L072 from a tablet UI command, no PC required.
- W6-2 (N-26): A/B firmware slots in L072 Flash — the second slot at `0x08018000` per [02 §4.3](02_Firmware_Architecture_Plan.md), with `slot_select.c` choosing the active slot at boot, "try B" flag with auto-revert on failed first-boot heartbeat.
- W6-3 (N-14): Signed firmware images — ED25519 verify at boot using a pinned public key in the bootloader region.
- W6-4 (N-27): Manufacturing self-test mode (`MFGTEST`).
- W6-5 (N-28): Per-unit calibration storage in the reserved Flash page.
- W6-6 (N-12, opt): Move AES-GCM into the L072 hardware AES engine as a build-time profile.
- W6-7 (N-18): In-firmware packet-capture ring (`PCAP_DUMP`).
- W6-8 (N-02): Hardware-timed TX scheduling via TIM2.
- W6-9 (N-05): Real-time spectrum scan / waterfall command.

**Exit criteria:**
- A/B update demonstrated: one bench unit takes a known-bad image, reverts cleanly, takes the next image, commits cleanly.
- Signed-boot rejects an unsigned image; accepts a properly signed image.
- Manufacturing test runs in ≤30 s with deterministic pass/fail.
- Brick-recovery procedure ([04 §3](04_Hardware_Interface_and_Recovery.md)) tested and documented end-to-end.

This is the **commercial-readiness milestone.** A unit shipped at this point can be safely updated by an end user.

---

## Phase 7 — Long-horizon platform features (post-launch)

These are explicitly out of scope for v25 launch. They become possible at zero hardware cost because of Method G; we list them so the architecture in [02](02_Firmware_Architecture_Plan.md) doesn't accidentally close them off.

- W7-1 (N-32): Mesh / repeater mode for multi-tractor sites.
- W7-2 (N-33): Autonomous "lost link, return to dock" coordination.
- Anything that emerges as field experience accumulates.

---

## Cross-phase ongoing work

These run continuously across phases; not gated:

- **Documentation** — every UART command added in any phase is documented in [04](04_Hardware_Interface_and_Recovery.md). Every protocol behavior added is reflected in [LORA_PROTOCOL.md](../LORA_PROTOCOL.md). [DECISIONS.md](../DECISIONS.md) gets an entry per phase boundary.
- **Tests** — every new feature lands with at least one unit-tier test (host PC) and at least one HIL-tier test.
- **CI** — keep the nightly loopback HIL run green. A red CI is a phase-blocker.
- **Repo memory** — milestone notes go into `/memories/repo/` so that future agent sessions don't re-discover known facts.

---

## Decision-reversal escape hatches

Per [00 §9](00_DECISION_Method_G_Commitment.md), the architecture stays reversible until **end of Phase 4**. Practically:

- Through Phase 1: trivial — we've only written boot + safe-mode + register dump.
- Phase 2–3: still reversible — `proto/` and `host/` would migrate to the H7 in any fallback design.
- Phase 4 onwards: the LBT, quality-aware FHSS, and emergency-beacon features only make sense if we own the radio MCU. Reversing after Phase 4 is a strategic re-plan, not a tactical pivot.

If a Phase ≤4 reversal becomes necessary, the fallback is: external SX1276 module on a small daughterboard wired to a routed H7 SPI bus. Same `proto/` code runs on H7 instead of L072.

---

## One-line summary

> **Seven phases, gated on capability not calendar: (0) qualify the bootloader pipeline, (1) hello-world + brick-resistance foundation + prove SPI-to-SX1276 from our binary, (2) single-frame TX/RX + host transport + crypto, (3) all recovered SPI-era features, (4) launch-quality new features (LBT, quality-aware FHSS, TX-power adapt) — this is the v25-launch milestone, (5) power & autonomous operation, (6) field-update + signed boot + manufacturing — commercial-readiness milestone, (7) long-horizon platform features. Reversal escape hatch stays open through Phase 4.**
