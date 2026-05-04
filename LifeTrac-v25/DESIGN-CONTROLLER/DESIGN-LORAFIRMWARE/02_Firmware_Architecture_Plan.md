# 02 — Firmware Architecture Plan

**Date:** 2026-05-04
**Status:** Plan — feeds module breakdown, build system, and resource budgets into [03 Bring-up Roadmap](03_Bringup_Roadmap.md).
**Author:** GitHub Copilot (Claude Opus 4.7)

---

## 1. Target

- **MCU:** STMicro STM32L072CZ (Cortex-M0+ @ 32 MHz, 192 KB Flash, 20 KB RAM, true RNG, hardware AES-128 on rev Y+ silicon, USART, SPI, I²C, USB-FS, RTC, IWDG, TIM2/3/21/22, DMA1).
- **Radio die:** Semtech SX1276 in same package, on internal SPI (`SPI1`), with DIO0/DIO1/DIO2/DIO3 wired to L072 GPIOs and a dedicated TCXO/PA/LNA/switch.
- **Host link:** L072 USART2 ↔ Portenta H7 `Serial3` (and X8 `/dev/ttymxc3` when the H7 isn't booted).
- **Reset/boot:** L072 NRST + BOOT0 driven by H7 GPIOs (NRST proven via `gpio163` from the X8 Linux bench; BOOT0 path is what the `MKRWANFWUpdate_standalone` sketch exercises).

## 2. Source layout

The firmware lives in [`../firmware/murata_l072/`](../firmware/) (folder will be created in roadmap [03 Phase 1](03_Bringup_Roadmap.md)):

```
firmware/murata_l072/
├── README.md                    # build & flash instructions, links back to DESIGN-LORAFIRMWARE
├── Makefile                     # top-level (or CMakeLists.txt — see §5)
├── platformio.ini               # optional alternate build entry point
├── ld/
│   └── stm32l072cz_flash.ld    # custom linker script with A/B slot regions (see §4.3)
├── boot/
│   ├── safe_mode.c             # UART safe-mode listener (N-22) — runs first thing in main()
│   ├── golden_jump.c           # last-resort jump to ROM bootloader
│   └── slot_select.c           # A/B slot selection (post-Phase-6)
├── hal/
│   ├── stm32l0xx_hal_conf.h    # Cube HAL configuration (only modules we use)
│   ├── system_stm32l0xx.c      # clock tree, SystemInit
│   ├── gpio.c / gpio.h         # pin map (DIO0..3, NSS, MOSI, MISO, SCK, RFSW, NRST, etc.)
│   ├── spi.c / spi.h           # SX1276 SPI bus only — dedicated to radio
│   ├── usart.c / usart.h       # host UART (USART2), DMA-on-IDLE pattern
│   ├── timer.c / timer.h       # TIM2 32-bit free-running timestamp + scheduled-TX trigger (N-02)
│   ├── rng.c / rng.h           # true RNG (N-13)
│   ├── aes.c / aes.h           # hardware AES-128 wrapper (used only if N-12 is enabled)
│   ├── rtc.c / rtc.h           # RTC alarms for deep-sleep wake (N-09)
│   └── pwr.c / pwr.h           # STOP/SLEEP entry, brown-out (N-23)
├── radio/
│   ├── sx1276.c / sx1276.h     # register-level driver (forked from Semtech reference, slimmed)
│   ├── sx1276_fhss.c / .h      # FHSS hop-table generator and per-channel quality store (N-06)
│   ├── sx1276_lbt.c / .h       # listen-before-talk (N-04)
│   ├── sx1276_metrics.c / .h   # per-frame metadata extraction (N-15)
│   └── sx1276_safety.c / .h    # P0 preempt via direct radio reset (R-04)
├── proto/
│   ├── lora_proto.c / .h       # v25 over-the-air frame format (ControlFrame, telemetry, fragment)
│   ├── replay.c / .h           # AES-GCM replay window
│   ├── crypto_gcm.c / .h       # tinycrypt or HW-AES-backed GCM (N-12 controlled)
│   └── failsafe.c / .h         # failsafe heartbeat watchdog (R-11)
├── host/
│   ├── host_uart.c / .h        # host transport state machine (COBS framer + dispatch)
│   ├── host_cmd.c / .h         # command table (TX, RX_URC, CFG, REG, STATS, PCAP, MFGTEST, ...)
│   └── host_cobs.c / .h        # COBS encoder/decoder (re-uses public-domain reference)
├── app/
│   ├── scheduler.c / .h        # priority queue + 50 ms slot scheduler (R-08, R-09, R-10)
│   ├── beacon.c / .h           # autonomous emergency beacon (N-30)
│   ├── deepsleep.c / .h        # STOP-mode RX-window scheduler (N-09)
│   └── stats.c / .h            # histograms (N-16)
├── diag/
│   ├── reg_dump.c / .h         # SX1276 register dump (N-17)
│   ├── pcap.c / .h             # in-firmware packet capture ring (N-18)
│   └── mfgtest.c / .h          # factory self-test (N-27)
├── main.c                      # init sequence, task dispatcher (no RTOS for launch — see §3)
├── config.h                    # build-time options (FEATURE_* defines)
├── version.h                   # generated at build time from git SHA + tag
└── tests/
    ├── host_loopback/          # PC-side Python test harness, talks to the host UART
    └── unit/                   # native (host-PC) unit tests for proto/, host/cobs/, replay/
```

Notes on layout choices:

- **No vendor SDK in-tree** beyond the few CMSIS / HAL files we actually use — keeps the tree small and reviewable.
- **`hal/` only contains what we use.** Cube HAL pulls in dozens of peripherals by default; we hand-pick.
- **`radio/` and `proto/` are deliberately separate.** A future swap to a different radio die (the Method E fallback in [00 §9](00_DECISION_Method_G_Commitment.md)) only touches `radio/` and `hal/spi.c`.
- **`host/` is the public ABI.** Document it in [04_Hardware_Interface_and_Recovery.md](04_Hardware_Interface_and_Recovery.md). Anything else can be refactored without coordinating with the H7 team.

## 3. Concurrency model

**Bare-metal cooperative scheduler. No RTOS for v25 launch.**

Reasons:
- M0+ context-switch overhead is non-trivial (~3–5 µs per switch); we want deterministic 100 µs control loops.
- 20 KB RAM doesn't comfortably hold 4× FreeRTOS task stacks plus our buffers.
- The application is naturally event-driven: 50 ms slot tick, DIO0/DIO1 IRQs from radio, USART2 IDLE IRQ, RTC alarm. A single main loop with an event queue handles all of it.

Structure:

```
main():
  hal_init();
  safe_mode_listen(500 ms);      // brick-resistance window (N-22)
  if (slot_select() == GOLDEN) golden_jump();
  proto_init(); host_init(); radio_init();
  scheduler_init();              // arms TIM2 50 ms tick

  for (;;) {
    event_t e = event_queue_pop_blocking_with_wfi();
    switch (e.type) {
      case EV_TICK:        scheduler_tick(); break;
      case EV_RADIO_RXD:   radio_rxd_handler(e.payload); break;
      case EV_RADIO_TXD:   radio_txd_handler(); break;
      case EV_HOST_RX:     host_dispatch(e.payload); break;
      case EV_RTC_ALARM:   deepsleep_wake_handler(); break;
      case EV_FAULT:       failsafe_trip(e.fault); break;
    }
    iwdg_kick();
  }
```

ISRs only enqueue events; no business logic runs in interrupt context. This makes timing analysis tractable and unit tests possible.

When N-09 (deep sleep) lands, the `wfi` becomes `pwr_enter_stop()`; wake sources stay the same.

## 4. Resource budget

### 4.1 Flash budget (192 KB total)

| Region | Size | Notes |
|---|---|---|
| Boot / safe-mode / slot-select | ~4 KB | Must be in a fixed location — see §4.3 |
| Cube HAL (pruned) | ~12 KB | Only enabled modules |
| `hal/` | ~6 KB | Our wrappers |
| `radio/sx1276.c` | ~14 KB | Forked from Semtech reference, ~30% smaller than original |
| `radio/` everything else | ~6 KB | FHSS table, LBT, metrics, safety |
| `proto/` | ~12 KB | Frame format, replay, GCM (tinycrypt ~9 KB or HW-AES ~3 KB), failsafe |
| `host/` | ~6 KB | UART + COBS + command dispatch |
| `app/` | ~10 KB | Scheduler, beacon, deepsleep, stats |
| `diag/` | ~4 KB | Register dump, packet ring, MFG test |
| `main.c` + glue | ~2 KB |  |
| Headroom | ~16 KB | For new features in Phase 5+ |
| **Subtotal — Slot A** | **~92 KB** | rounds to a 96 KB slot |
| **Slot B** | 96 KB | Mirror of slot A (post-Phase-6, N-26) |
| **Bootloader / recovery / config** | ~4 KB | Last 4 KB of Flash; never overwritten by app updates |

Until A/B slots land, the firmware can use the full ~188 KB of user Flash. The constraint above is what we plan for once N-26 ships.

### 4.2 RAM budget (20 KB total)

| Allocation | Size | Notes |
|---|---|---|
| `.data` + `.bss` (statics) | ~3 KB | Globals, FHSS table, replay window, channel-quality scores |
| Host UART RX DMA buffer (×2 ping-pong) | 2 × 256 B | DMA-on-IDLE |
| Host UART TX queue | 1 KB | Outbound URC/response frames |
| Radio TX/RX page buffer | 2 × 256 B | SX1276 FIFO mirrors |
| Event queue | 32 entries × 16 B | 512 B |
| AES-GCM context | 512 B | Only allocated when crypto active |
| Packet capture ring (N-18, post-launch) | 2 KB | 32 frames × 64 B |
| Histogram counters (N-16) | 1 KB | Per-channel + per-SF |
| Stack | 2 KB | Single main stack + ISR stack |
| Headroom | ~6 KB | |
| **Total** | **~14 KB** | comfortable |

### 4.3 Linker / memory map

```
0x08000000  +-------------------------+
            | Vector table & boot     |  4 KB  (always-resident, never overwritten by app updates)
0x08001000  +-------------------------+
            | Slot A (active app)     | 92 KB
0x08018000  +-------------------------+
            | Slot B (inactive/spare) | 92 KB   (Phase 6+; until then, available to slot A)
0x08030000  +-------------------------+
            | Reserved / config page  |  4 KB   (per-unit calibration N-28, slot-select flags)
0x08030400  +-------------------------+   end of Flash @ 192 KB

0x20000000  +-------------------------+
            | .data / .bss            |  ~3 KB
            | DMA buffers             |  ~1.5 KB
            | App heap (static pool)  |  ~6 KB
            | Stack (top-down)        |  ~2 KB
            | Headroom                |  rest
0x20005000  +-------------------------+   end of RAM @ 20 KB
```

## 5. Build system & toolchain

**Primary toolchain:** `arm-none-eabi-gcc` 13.2 (matches what `arduino:mbed_portenta` core uses for the H7; reduces "but it builds for me" issues).

**Build entry points (we ship both):**

1. **GNU Make** — minimal, no external dependencies beyond `arm-none-eabi-*`. Fast incremental builds. Primary CI target.
2. **PlatformIO** — drops into VS Code with no setup, gives a one-click flash. For developers who don't want to install the toolchain manually.

Both invoke the same source files and the same linker script.

**No STM32CubeIDE in CI.** Cube IDE is fine for one-off debugging but its project file is not a sensible source of truth.

**Flashing during development:** primary path is the `MKRWANFWUpdate_standalone`-style approach (H7 drives BOOT0/NRST, runs the STM32 ROM bootloader UART protocol over `Serial3`). Reference: the [Arduino MKRWAN library `MKRWANFWUpdate_standalone` example](https://docs.arduino.cc/libraries/mkrwan/) — already proven on this hardware. Secondary: ST-Link via SWD pins on a custom adapter (only if a board is bricked beyond UART-recovery, which the safe-mode design is intended to prevent).

## 6. Stable interfaces (the things we don't break casually)

Two interfaces are *contracts* and must be versioned explicitly:

1. **Over-the-air frame format** — defined in [LORA_PROTOCOL.md](../LORA_PROTOCOL.md). Carries a 4-bit `proto_version` field; firmware refuses frames it doesn't understand.
2. **Host UART protocol** — defined in [04_Hardware_Interface_and_Recovery.md](04_Hardware_Interface_and_Recovery.md). Carries a 1-byte protocol-version in the COBS-framed header. Firmware refuses commands it doesn't understand and emits a versioned `ERR_PROTO` URC.

A third interface exists implicitly: the radio-driver-to-protocol-stack split. Keeping `radio/sx1276*.c` independent of `proto/*` is a reversibility insurance policy — see [00 §9](00_DECISION_Method_G_Commitment.md).

## 7. Testing strategy

| Tier | Where it runs | What it covers |
|---|---|---|
| Unit | Host PC (x86_64 GCC) | `proto/`, `host/cobs`, `replay`, `crypto_gcm` — pure logic, no MCU |
| Loopback | Host PC ↔ real Murata over USB serial via H7 | `host/` ABI conformance: every UART command exercises a documented path |
| HIL pair | Two real Murata boards, on the bench | Over-the-air protocol; FHSS dwell stats; LBT stats; failsafe; preempt latency |
| HIL soak | Two boards × 24 h | Memory leaks, watchdog stability, statistics drift |
| Spectrum | Single board + RTL-SDR on the bench | TX-power calibration, FHSS pattern verification, FCC §15.247 dwell-time evidence |

CI runs the unit tier on every push, the loopback tier nightly against a single bench Murata wired permanently to a CI runner, and HIL tiers on demand.

## 8. Open questions deferred to roadmap

These get resolved during the roadmap phases listed; flagging them here so they aren't forgotten.

- **Q1:** Cube HAL vs. LL drivers vs. roll-our-own? — decide in Phase 1, after baseline build size measured. Default: Cube HAL with LTO.
- **Q2:** Tinycrypt vs. HW-AES for GCM? — measure in Phase 2 once we have a board flashable with our binary. Default: tinycrypt for portability.
- **Q3:** What's the exact maximum DMA-friendly host UART burst size? — measure in Phase 2 against H7's `Serial3` driver. Default assumption: 256 B fits in one IDLE interrupt window at 921600 baud.
- **Q4:** Does the Max Carrier route DIO3 (CAD-done, useful for LBT)? — confirm with multimeter in Phase 1.
- **Q5:** Which channels are usable in our regulatory domain at full 100 mW with our duty model? — produce a validated channel list in Phase 4 before locking N-06's quality-aware FHSS tables.

## 9. One-line summary

> **Bare-metal cooperative scheduler on the L072, Cube HAL pruned to what we use, source split into hal/radio/proto/host/app/diag/, two stable interfaces (over-the-air frame + host UART), Make + PlatformIO build entry points, testing in four tiers from host-PC unit tests up to spectrum measurement. Flash budget ~92 KB per slot with two slots planned, RAM ~14 KB used of 20 KB. The firmware source lives in `../firmware/murata_l072/`; this folder holds its design.**
