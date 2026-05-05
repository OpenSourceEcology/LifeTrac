# LoRa Firmware — Tranche 5 Plan: Channel Hygiene & Link Adaptation

**Author:** GitHub Copilot (Claude Opus 4.7)
**Date:** 2026-05-05
**Version:** v1.2 (Copilot §16 self-review applied — PR#0 rewritten against real STM32L072 RCC, FHSS epoch sync added, TX-power adaptation moved to H7 to preserve Crypto Profile A boundary, FHSS labeled bench-only pending regulatory baseline, T5-A split into RSSI + CAD-preamble cases, golden STATS vectors mandated, existing `sx1276_tx_estimate_toa_us` reused for airtime, CFG-table hygiene tightened, H7 build-system assumption flagged for verification)
**Status:** Draft for review
**Predecessor:** [2026-05-05_LoRa_Firmware_Tranche_4_Bringup_TX_H7Driver_MaxCarrier_Plan_Copilot_v1_0.md](2026-05-05_LoRa_Firmware_Tranche_4_Bringup_TX_H7Driver_MaxCarrier_Plan_Copilot_v1_0.md) §12 (Tranche 5 sketch)

---

## 0. Context — what we're inheriting from Tranche 4

Tranche 4 implementation is partially landed. Per user notes 2026-05-05:

| Item | Status as of T5 kickoff |
|---|---|
| PR#1 CI cross-compile gate | ✅ landed |
| PR#2 BOOT_URC.reset_cause + FAULT_URC | ✅ landed (with deviation — see below) |
| PR#3 radio TX path (C3) + TX_DONE_URC + sole-writer guard | ✅ landed |
| PR#4 minimal `tractor_h7` host driver | ⚠ deferred — carryover into T5 |
| PR#5 host-side unit/loopback tests for wire + TX_DONE parsing | ⚠ deferred — carryover into T5 |
| PR#6 `BRINGUP_MAX_CARRIER.md` + openocd config | ⚠ deferred — carryover into T5 |

**Deviation from plan — fault replay storage:**
- **Plan said:** RTC backup registers BKP0–BKP4 with magic `0xFA075DEC`, survives PoR.
- **Implemented:** retained `.noinit` RAM record + reset/replay path. Survives soft reset and IWDG, **does not survive PoR or VBAT loss**.
- **Why this matters:** for bench bring-up this is fine — every fault we'll see in T5 is software-induced from a running session. For battery-powered field deployment (Tranche 8 territory), PoR-survivable fault replay becomes load-bearing for diagnosing wake-from-deep-sleep failures.
- **Decision for T5:** *do not retrofit RTC BKP yet.* Track as **deferred-tech-debt T5-D1**, schedule for Tranche 8 alongside RTC LSE wake configuration (RTC peripheral has to be initialized for wake anyway — the BKP write costs ~6 lines of code at that point). If a field unit ships before T8, escalate.

**Implication for T5 scope:** we have to absorb three unfinished T4 PRs (driver, tests, runbook) before we can validate any new T5 work on hardware. They become T5 PR#1–#3 — *prerequisite gate* — and the channel-hygiene work proper begins at T5 PR#4.

---

## 1. Theme & success criterion

**Theme:** Make the radio link survive a real RF environment. Today the L072 transmits on a fixed channel at fixed power and would happily step on any neighbor in the 902–928 MHz ISM band. Tranche 5 introduces:
1. **Listen-Before-Talk** (regulatory hygiene + reduced collisions),
2. **Channel Activity Detection** as a faster complement to LBT,
3. **TX-power adaptation** driven by ACK link metrics,
4. **Frequency-hopping spread spectrum** (deterministic schedule, no learning yet),
5. **Air-time accounting** to enforce US 915 MHz Part 15.247 duty-cycle equivalent (400 ms in any 1 s per 500 kHz channel — being conservative; the actual rule is more nuanced but this is a safe ceiling).

**Success criterion:** All four bench tests pass:
- **T5-A1 (RSSI LBT path):** With a deliberate **CW interferer** (signal generator at +0 dBm CW on the channel, conducted/shielded with attenuators to protect the RX front-end), L072 backs off via RSSI-based LBT — measured `radio_tx_abort_lbt` increment count matches injected interferer duty cycle ±10 %.
- **T5-A2 (CAD path — added per §16.6):** With a **known-good LoRa preamble/packet interferer** on the same channel (e.g. an HTCC-AB01 transmitting fixed preambles), CAD asserts `CadDetected`, fallback path executes correctly. Validates that CAD code is not silently dead behind the RSSI-only T5-A1.
- **T5-B:** TX power tracks RX SNR — drive the H7 to deliberately attenuate the link (RF cable + 30 dB pad), observe TX power step up via `STATS_URC.radio_state` interpretation and on-air spectrum-analyzer readings. **Note:** under §16.4 amendment, the EWMA decision lives in the H7, not the L072 — this test now exercises the H7 → `CFG_SET tx_power_dbm` → L072 apply path.
- **T5-C:** Two-board ping-pong with FHSS enabled across all 16 channels for 30 minutes, zero stuck-on-channel events (max consecutive frames per channel ≤ ceil(30 min × frame_rate / 16) + 3σ). **Requires:** PR#0 HSE clock + epoch-sync mechanism per §16.3 (see PR#6 detail). If epoch sync is not implemented in PR#6, T5-C downgrades to a **one-way distribution test** that only verifies the transmitter's channel rotation cadence and dwell accounting — the bidirectional ping-pong is deferred to whatever tranche lands epoch sync.
- **T5-D:** Air-time guard rejects intentional overshoot — set frame rate × payload to exceed the configured per-channel budget on a single channel, observe `radio_tx_abort_airtime` (new counter) increment instead of TX_DONE. **Note:** the budget is **engineering-conservative bench limit**, not a Part 15.247 compliance claim — see §16.5.

---

## 2. Out of scope (deferred to later tranches)

| Item | Tranche |
|---|---|
| Quality-aware FHSS (per-channel learned SNR/RSSI/loss EWMA) | T7 (N-06) |
| True random channel hop with PRNG seeded from session nonce | T7 |
| Adversarial-resistance / replay fuzzing | T7 |
| RTC BKP fault persistence (T5-D1 deferred-tech-debt) | T8 |
| Deep sleep, RX windowing | T8 |
| H7 driver migration to default; expanded command surface | T6 |

---

## 3. PR sequence

Seven PRs. PR#0 is a clock-domain gate (added in v1.1 per Gemini review #2 — **must land before any FHSS work**). PRs #1–#3 are T4 carryover. PRs #4–#6 are T5 proper.

| # | Branch | Title | Gate |
|---|---|---|---|
| 0 | `chore/clock-source-hse` | **Gate (Gemini review #2):** verify SYSCLK runs off the Murata SiP's 32 MHz TCXO/HSE; switch from HSI if needed | `RCC_CR.HSEON=1`; SYSCLK source = PLL-fed-by-HSE (or HSE direct); SysTick drift ≤ 5 ppm measured against ST-LINK's reference; documented in BOOT_URC's first-byte payload (new `clock_source_id` field reserved at end of BOOT_URC) |
| 1 | `feat/h7-host-driver-min` | T4 carryover: minimal `tractor_h7` Method G driver | Compiles in H7 build under `LIFETRAC_USE_METHOD_G_HOST=1`; legacy KISS path still compiles when flag off |
| 2 | `chore/h7-host-driver-tests` | T4 carryover: host-CC unit tests + PTY loopback for COBS+CRC + TX_DONE parsing + vendor-drift check + offset-equivalence check | `make check-host-cc` + `make check-stats-offsets` green in CI |
| 3 | `docs/max-carrier-bringup` | T4 carryover: `BRINGUP_MAX_CARRIER.md` + openocd cfg | Reviewed + dry-walked on dev machine (not yet executed on hardware) |
| 4 | `feat/radio-lbt-cad` | C5 LBT + C6 CAD; integrate ahead of `sx1276_tx_begin` | `radio_tx_abort_lbt` increments on bench with injected interferer; CI guard `check-lbt-owner` (sole writer of LBT thresholds) |
| 5 | `feat/radio-tx-power-adapt-airtime` | C7 TX-power adaptation + C10 air-time accounting | `radio_tx_abort_airtime` exists; T5-B + T5-D pass on bench |
| 6 | `feat/radio-fhss-deterministic` | C8 deterministic FHSS hop schedule + B5 four new CFG keys | T5-C passes; **requires PR#0 landed**; PROTOCOL_VER + WIRE_SCHEMA_VER **unchanged** at 1/1 (additive payload only) |

### 3.5 PR#0 detail — `chore/clock-source-hse` (clock-domain gate)

**Why this exists:** Gemini review comment #2 — at HSI's worst-case ~1% accuracy, two boards' `now_ms` clocks drift ~200 ms apart in **as little as 10 s** (worst case differential), well below the 200 ms half-dwell threshold that desyncs deterministic FHSS. T5-C's 30-min ping-pong target would fail in the first ~100 s of any run.

**Fix:** Murata CMWX1ZZABZ-078 SiP includes a **32 MHz TCXO** wired to the STM32L072's OSC_IN (shared between MCU SYSCLK and SX1276 RF reference). Spec ±2 ppm typical → 30-min cumulative drift ≤ 3.6 ms, two-orders-of-magnitude headroom under the 200 ms threshold.

**Implementation work:**
- Inspect current boot RCC config (`SystemClock_Config()` or equivalent in T4-landed boot code).
- If currently MSI/HSI-based: enable HSE bypass mode (TCXO drives directly), wait for `HSERDY`, configure PLL with HSE source (PLLM=1, PLLN=8, PLLR=2 → 128 MHz VCO → 32 MHz SYSCLK), switch SYSCLK via `RCC_CFGR.SW`, deinit MSI/HSI to save ~100 µA.
- New `FAULT_URC` sub-code `0x08 CLOCK_HSE_FAILED` if HSERDY times out → bootloader falls back to HSI and asserts FAULT so STATS reflects the degraded mode (rather than silently desyncing later).
- Reserve a single byte at the end of BOOT_URC payload (`clock_source_id`: 0=HSE-OK, 1=HSI-fallback, 2=MSI-fallback) so the H7 can refuse to enable FHSS if the L072 reports a degraded clock.
- Add bench check to `BRINGUP_MAX_CARRIER.md` Stage 1: with an oscilloscope or frequency counter on MCO pin, verify SYSCLK = 32.000 MHz ±100 Hz (the TCXO will be inside ±64 Hz at room temp; HSI would read ±320 kHz and is the smoking-gun signal).

**Files modified:**
- `boot/clock.c` (or equivalent — verify location during PR drafting)
- `host/host_types.h` — add `clock_source_id` byte to BOOT_URC payload (additive, no schema bump — tail byte)
- `host/host_cmd.c` — emit it in `BOOT_URC` builder
- `BRINGUP_MAX_CARRIER.md` — Stage 1 MCO frequency check
- New CI guard `check-no-hsi-default`: grep that boot code never selects HSI as SYSCLK source

---

## 4. Detailed work — PR#1 `feat/h7-host-driver-min` (T4 carryover)

**Restate from T4 §6:** create `LifeTrac-v25/DESIGN-CONTROLLER/firmware/tractor_h7/src/murata_host/` containing:

```
murata_host/
├── mh_cobs.h / mh_cobs.cpp        -- vendored from L072 cobs.c, identical algorithm
├── mh_crc16.h / mh_crc16.cpp      -- vendored from L072 crc16.c
├── mh_wire.h / mh_wire.cpp        -- frame encode/decode; matches host_types.h type IDs
├── mh_uart.h / mh_uart.cpp        -- Serial3 @ 921600 8N1 wrapper
├── mh_driver.h / mh_driver.cpp    -- public API: ping/getStats/sendFrame/onURC
└── mh_types.h                     -- mirror of L072 host_types.h (drift-checked in PR#2)
```

**Public API (minimal — full surface lands T6):**
```cpp
class MurataHost {
public:
    bool begin();                                  // init UART, send PING, expect BOOT_URC
    bool sendFrame(const uint8_t* p, uint16_t n);  // queues TX_FRAME_REQ
    void poll();                                   // pumps RX, dispatches URCs to callbacks
    void onRxFrame(std::function<void(const RxFrame&)>);
    void onTxDone(std::function<void(uint8_t seq, uint8_t status)>);
    void onFault(std::function<void(uint8_t code, const uint8_t* ctx, uint8_t n)>);
    const HostStats& stats() const;                // last STATS_URC payload
};
```

**Compile gate:** wrap in `#ifdef LIFETRAC_USE_METHOD_G_HOST`. Legacy KISS path stays compiled when flag is off — both must coexist for one full release cycle (per T4 §6 + T6 H1).

**Build-system reality check (per §16.10):** the v1.0 plan referenced `tractor_h7/platformio.ini`, but the current `tractor_h7` folder is Arduino-sketch style and may not contain that file. Before drafting PR#1, **inspect the actual `tractor_h7` build entry point**:
- If Arduino-sketch + arduino-cli: add the `-DLIFETRAC_USE_METHOD_G_HOST=1` flag via `arduino-cli compile --build-property build.extra_flags=...` and document the two build invocations in `tractor_h7/README.md`.
- If `platformio.ini`: add a build variant.
- If neither/something else: choose the least-invasive option that supports two build variants in CI.

Acceptance amendment: **both** the legacy KISS build and the Method G host-driver build must compile in the **current repository build path**, not a proposed-but-nonexistent PlatformIO path.

**Files modified:**
- New: 13 files under `murata_host/`
- `tractor_h7/<actual build entry>` — add `-DLIFETRAC_USE_METHOD_G_HOST=1` build variant
- `tractor_h7/src/main.cpp` (or `.ino`) — `#ifdef`-guarded init wiring (no behavior change to legacy path)

---

## 5. Detailed work — PR#2 `chore/h7-host-driver-tests` (T4 carryover)

**Two test surfaces:**

### 5.1 Host-CC unit tests (run in CI, no hardware)

- Reuse the existing `LifeTrac-v25/DESIGN-CONTROLLER/firmware/murata_l072/bench/host_proto/` test pattern (Method G already has `cfg_contract.c`, `cfg_wire_vectors.c`).
- New file `bench/host_proto/wire_roundtrip.c`: encodes a frame via L072's `cobs.c` + `crc16.c`, decodes via H7's `mh_cobs.cpp` + `mh_crc16.cpp` (vendored copies must produce byte-identical results), asserts equality.
- New file `bench/host_proto/tx_done_parse.c`: feeds a synthetic TX_DONE_URC byte sequence into H7 driver, asserts callback fires with correct seq + status.
- New `Makefile` target `check-host-cc` invokes a host gcc/g++ build of these tests.

### 5.2 PTY loopback (dev workstation, not CI by default)

- New `tools/murata_host_loopback.py`:
  - Opens two ends of a Linux pseudo-tty pair (or a Windows com0com pair — document both).
  - One end runs a Python implementation of the L072 wire (mirrors `host_types.h`).
  - Other end runs the H7 `MurataHost` driver compiled host-native.
  - Validates: PING → PONG, TX_FRAME_REQ → STATS sees `host_dropped` unchanged, synthetic RX injection → `onRxFrame` fires.

**Transport-abstraction file split (Gemini review #3 — required for host build to succeed):** the H7 driver cannot directly include `Arduino.h` from any header that the host build also consumes. Layout:

```
murata_host/
├── mh_uart.h              -- pure interface only: `class IUartTransport { virtual ssize_t read/write/available; };` NO Arduino.h, NO HardwareSerial.
├── mh_uart_arduino.cpp    -- TARGET-ONLY. Wraps Serial3 (HardwareSerial) into IUartTransport. Excluded from host build.
├── mh_uart_posix.cpp      -- HOST-ONLY. Wraps a POSIX file descriptor / Win32 HANDLE into IUartTransport. Excluded from target build.
├── mh_driver.{h,cpp}      -- references only IUartTransport*; portable to both targets.
```

- Build-system selector: PlatformIO `build_src_filter` (or arduino-cli equivalent) excludes `mh_uart_arduino.cpp` from host-CC builds; the host Makefile excludes `mh_uart_posix.cpp` from target builds.
- CI guard `check-no-arduino-in-host-headers` (added in §10): greps `mh_*.h` for any reference to `Arduino.h`, `HardwareSerial`, or `Stream` and fails the build. This is the single tripwire that makes the abstraction load-bearing.
- The `MurataHost` constructor takes an `IUartTransport&`; the H7 sketch passes `MhUartArduino(Serial3)`, the PTY loopback passes `MhUartPosix(pty_fd)`.

### 5.3 Vendor-drift check

- New `Makefile` target `check-host-vendor-drift`:
  - Computes SHA-256 of `murata_l072/host/cobs.c` + `cobs.h`, compares to the same of H7's `mh_cobs.cpp` + `mh_cobs.h` (with a header-comment normalization preprocessor pass — strip `#include` and namespace differences).
  - Same for `crc16.{c,h}` ↔ `mh_crc16.{cpp,h}`.
  - Same for the type-ID block from `host_types.h` ↔ the mirror in `mh_types.h`.
  - Fails CI if any of the three diff. Forces deliberate update on the H7 side any time the L072 wire moves.

**CI integration:** add `check-host-cc` and `check-host-vendor-drift` to the existing `firmware-l072-static-checks` job in `.github/workflows/arduino-ci.yml`.

---

## 6. Detailed work — PR#3 `docs/max-carrier-bringup` (T4 carryover)

New file `LifeTrac-v25/DESIGN-CONTROLLER/firmware/murata_l072/BRINGUP_MAX_CARRIER.md`. Content sections:

1. **Hardware checklist** — Max Carrier rev, MKR WAN 1310 carrier-modification notes (if any), 12 V supply spec, antenna (50 Ω SMA-male, ≥3 dBi, ≥10 cm clear of metal), spectrum analyzer or SDR for T5 validation.
2. **SWD pinout** — STM32L072CZ SWDIO/SWCLK/NRST/GND/3V3 to ST-LINK V2 / V3 mini.
3. **`openocd.cfg`** — embedded — uses `interface/stlink.cfg` + `target/stm32l0.cfg`, sets `adapter speed 480`, defines `flash_app`/`flash_boot` proc that respects the memory map.
4. **First-flash procedure** — bootloader (currently empty 4 KB stub — load anyway to claim the BOOT region), then app, then verify with `mdw 0x08001000 4` (read first 4 words of vector table).
5. **Bench procedure — three stages:**
   - **Stage 1 — single-board boot:** flash one L072, attach H7 over Serial3, run `MurataHost.begin()` from a minimal sketch, expect `BOOT_URC` within 200 ms with `reset_cause=0x01` (POR). Confirm `STATS_URC` arrives at 1 Hz.
   - **Stage 2 — known-good RX:** with TX disabled on the L072 under test, transmit a CW + LoRa packet from a separate known-good radio (e.g. an HTCC-AB01 dev board running a fixed transmitter sketch), observe `RX_FRAME_URC` on the bench L072 with valid SNR/RSSI.
   - **Stage 3 — two-board ping-pong:** flash a second L072 + H7 pair, run handshake at 1 Hz for 10 minutes, success criterion = zero `FAULT_URC` and `radio_rx_ok` count matches `radio_tx_ok` count of the peer ±2.
6. **Recovery procedures** — what to do if BOOT_URC never arrives (check 3V3 rail, check SWD connectivity, check NRST, mass-erase recipe), what to do if STATS_URC arrives but `radio_state` stays UNINIT (RF switch GPIO config — board-specific), what to do if a `FAULT_URC` arrives (decode the taxonomy 0x01–0x07 against the source-of-truth in `host_types.h`).
7. **Bring-up checklist sign-off** — yes/no boxes for each Stage. Append filled-in copies as bench artifacts under `LifeTrac-v25/AI NOTES/bringup_logs/` (new subdir) once executed.

---

## 7. Detailed work — PR#4 `feat/radio-lbt-cad` (T5 proper begins)

### 7.1 C5 — Listen-Before-Talk

**New file:** `radio/sx1276_lbt.c` + `radio/sx1276_lbt.h`.

**API:**
```c
typedef enum {
    LBT_RESULT_CLEAR,        // RSSI below threshold; safe to TX
    LBT_RESULT_BUSY,         // RSSI above threshold; back off
    LBT_RESULT_DISABLED,     // cfg.lbt_enable == 0
    LBT_RESULT_ERROR,        // SPI / mode-transition failure
} sx1276_lbt_result_t;

sx1276_lbt_result_t sx1276_lbt_check(uint32_t now_ms);
uint32_t sx1276_lbt_backoff_ms(uint32_t now_ms);  // returns ms-to-retry on BUSY
```

**Implementation:**
- Honors `cfg.lbt_enable` (key 0x03) and `cfg.lbt_threshold_dbm` (key 0x04, signed int8 in dBm, default -90).
- Procedure: transition to RX_CONT via `sx1276_modes_to_rx_cont`, wait `cfg.cad_symbols × symbol_time` (currently hardcoded — becomes configurable in PR#6), read `RegRssiValue` (single byte; convert: `RSSI_dBm = -157 + raw` for HF band), compare to threshold, transition back to STANDBY.
- **Sole-writer-of-OpMode discipline:** all mode transitions go through `sx1276_modes_*`. LBT module never touches RegOpMode directly.
- **Backoff schedule:** binary exponential backoff per [02 Phase 3 W3-2](../DESIGN-CONTROLLER/DESIGN-LORAFIRMWARE/02_Firmware_Architecture_Plan.md), capped at `cfg.lbt_max_backoff_ms` (new key 0x10, default 500 ms).
- Increments `host_stats_radio_tx_abort_lbt` on every BUSY result that prevents a queued TX.

**Integration:** `sx1276_tx_begin` (created in T4 PR#3) gains an early call to `sx1276_lbt_check` when `cfg.lbt_enable`. On BUSY, returns `SX1276_TX_RESULT_DEFERRED`; caller re-queues with the backoff delay.

**Files modified:**
- New: `radio/sx1276_lbt.{c,h}`, `bench/host_proto/lbt_vectors.c`
- `radio/sx1276_tx.c` — add LBT pre-check
- `host/host_cfg_keys.h` — add key 0x10
- `host/host_cfg.c` — add validator + apply-hook for key 0x10 (range 10–5000 ms)
- `Makefile` — new target `check-lbt-owner` (CI guard: `radio_tx_abort_lbt` is incremented only from `sx1276_lbt.c` and the existing reserved-counter slot)

### 7.2 C6 — Channel Activity Detection

**New file:** `radio/sx1276_cad.c` + `radio/sx1276_cad.h`.

**API:**
```c
typedef enum {
    CAD_RESULT_DETECTED,     // LoRa preamble present
    CAD_RESULT_CLEAR,
    CAD_RESULT_PENDING,      // CAD_DONE not yet asserted
    CAD_RESULT_ERROR,
} sx1276_cad_result_t;

void sx1276_cad_begin(void);                  // arms CAD, returns immediately
sx1276_cad_result_t sx1276_cad_poll(void);    // poll RegIrqFlags for CAD_DONE | CAD_DETECTED
```

**Implementation:**
- Uses `sx1276_modes_to_cad` (already exists from T2 C1 modes extraction).
- Reads `RegIrqFlags` bits CadDone (bit 2) + CadDetected (bit 0), clears via W1C.
- Faster than LBT for LoRa-vs-LoRa contention (CAD: ~1 symbol; LBT RSSI-only: ~`cad_symbols` symbols and misses preamble below noise floor).
- LBT module's `sx1276_lbt_check` may opportunistically use CAD first, fall back to RSSI.
- **Race-window note (Gemini review #4 — accepted as design limit):** there is a ~100 µs gap between CAD/RSSI evaluation and the `sx1276_modes_to_tx` transition during which an interferer can key up undetected. This is inherent to any non-MAC RF protocol and is acceptable: at SF7 BW125 the interferer's 8.2 ms preamble would also have painted CAD with a partial detect, so the race-loss probability is bounded. T5-A's ±10% tolerance on `radio_tx_abort_lbt` count absorbs race-induced collisions as expected noise; a >10% mismatch indicates a real bug, not a race-window artifact.

**Files modified:**
- New: `radio/sx1276_cad.{c,h}`
- `radio/sx1276_lbt.c` — call CAD before RSSI when `cfg.cad_enable` (uses existing fhss_quality_aware-adjacent decision pattern; note: CAD enable is implicit via `cfg.cad_symbols > 0` per key 0x11)

---

## 8. Detailed work — PR#5 `feat/radio-tx-power-adapt-airtime`

### 8.1 C7 — TX-power adaptation (architecture revised per §16.4)

**v1.0 plan said:** EWMA + decision logic on the L072.
**v1.2 amendment:** **EWMA + decision logic moves to the H7.** This preserves the Crypto Profile A boundary at launch (H7 owns AES-GCM; L072 transports authenticated bytes and cannot inspect application payloads to identify ACKs).

**Revised contract:**
- **L072 side (this tranche):**
  - Continues to populate `RX_FRAME_URC` with `snr` and `rssi` per-frame (already implemented).
  - Honors `CFG_SET tx_power_dbm` (key 0x01, already validated/clamped 2..17 dBm in T2).
  - **No new logic on L072.** Removes the previously-planned 8-sample EWMA state and decision rule from `host_cmd.c`.
  - Removes the previously-planned `tx_power_step_db` key (0x12) — the H7 manages step size.
- **H7 side (this tranche, in PR#1's `MurataHost` driver):**
  - New optional helper class `TxPowerAdapter` consumes `onRxFrame` callbacks, maintains 8-sample EWMA of ACK SNR, applies the decision rule (SNR > +10 dB → step down; < +3 dB → step up; floor +2 dBm, ceiling +17 dBm), 5-ACK hysteresis.
  - H7 application opts in by instantiating `TxPowerAdapter(host, ackSeqMatcher)` where `ackSeqMatcher` is an application-supplied predicate that identifies which RX frames are ACKs (the H7 *can* see this because it owns decryption).
  - `TxPowerAdapter` calls `host.cfgSet(TX_POWER_DBM, new_dbm)` to apply.
- **Defer to T6/T7 if needed:** if even the H7-side adapter is too coupled to ACK semantics that aren't yet defined, ship T5 with **manual** `tx_power_dbm` control only and move adaptation to T6 after the application-layer ACK surface is formalized.

**Out of scope:** dynamic spreading-factor adaptation, dynamic bandwidth.

**Files modified (v1.2):**
- `tractor_h7/src/murata_host/tx_power_adapter.{h,cpp}` — new optional H7 helper.
- *(removed from L072 scope: no changes to `host_cmd.c` for adaptation logic.)*

### 8.2 C10 — Air-time accounting

**New file:** `radio/sx1276_airtime.c` + `radio/sx1276_airtime.h`.

**API:**
```c
typedef enum {
    AIRTIME_RESULT_OK,           // budget available
    AIRTIME_RESULT_OVER_BUDGET,  // would exceed cap; reject TX
    AIRTIME_RESULT_DISABLED,     // cfg.airtime_enforce == 0 (key TBD; for T5, always-on)
} sx1276_airtime_result_t;

sx1276_airtime_result_t sx1276_airtime_reserve(uint16_t payload_bytes,
                                               uint8_t  channel_idx,
                                               uint32_t now_ms);
void sx1276_airtime_commit(uint8_t channel_idx, uint32_t actual_us);  // called from TX_DONE handler
```

**Implementation:**
- Per-channel sliding-window credit: configured per-channel budget in any 1 000 ms window per channel. **Default 400 000 µs** as an engineering-conservative bench limit. **Not a Part 15.247 compliance claim** — see §16.5; the regulatory baseline (channel count, BW, dwell rule) is unresolved and will be addressed in a future tranche with an explicit `REG_PROFILE_*` enum.
- 16 channels × (uint32 used_us + uint32 window_start_ms) = 128 B RAM. Acceptable.
- **Reuse the existing `sx1276_tx_estimate_toa_us()` (per §16.8)** — do not invent a second formula. Refactor it from `sx1276_tx.c` into `sx1276_airtime.c` and have **both** TX_DONE reporting and budget reservation call the same implementation. Prevents drift between estimated-and-reserved vs. estimated-and-reported.
- **Reservation rollback semantics (per §16.8):**
  - LBT rejects before TX → no airtime reserved (reservation deferred until LBT-clear).
  - TX starts and times out → commit estimated airtime conservatively (RF energy may have been emitted).
  - TX aborts before `sx1276_modes_to_tx` → release the reservation.
  - TX_DONE clean → commit actual airtime if measured, else estimated.
- New STATS counter `radio_tx_abort_airtime` — requires adding 4 B to `host_stats_wire_t`. **This bumps the payload from 64 B to 68 B.** Decision: payload-additive only, no `HOST_WIRE_SCHEMA_VER` bump (the schema-version table in T4 §12 said T8 is the first wire bump — keeping that promise by strict-additive in T5 means readers of v1 schema simply ignore trailing bytes). Confirm this is accepted by re-reading the wire-versioning policy in `host_types.h` header comment; if it isn't strictly additive-tolerant, escalate and reconsider.

**Files modified:**
- New: `radio/sx1276_airtime.{c,h}`, `bench/host_proto/airtime_vectors.c`, `bench/host_proto/stats_golden_v1_64.bin` (pre-PR#5 STATS golden), `bench/host_proto/stats_golden_v1_68.bin` (post-PR#5 STATS golden — per §16.7)
- `radio/sx1276_tx.c` — call `sx1276_airtime_reserve` after LBT clear, before `sx1276_modes_to_tx`; **move** `sx1276_tx_estimate_toa_us()` into `sx1276_airtime.c` (single source of truth).
- `host/host_stats.c` — extend manual serializer (`put_u32_le`-style — the wire is *not* a struct memcpy; per §16.7) to append `radio_tx_abort_airtime`; update size assertion.
- `host/host_stats.h` — bump `HOST_STATS_PAYLOAD_BYTES` from 64 to 68; add explicit `HOST_STATS_OFFSET_*` constants per field for parser use.
- `host/host_types.h` — document the additive-tail policy: while `HOST_WIRE_SCHEMA_VER == 1`, parsers MUST accept STATS payloads of ≥64 B and ignore trailing bytes they don't recognize.
- H7 driver: parser tests covering both 64 B and 68 B STATS payloads (golden-vector replay).
- Add CI guard `check-airtime-counter-owner` (mirrors existing rx/tx counter owner guards).

**Padding & alignment discipline (Gemini review #1 + Copilot §16.7 amendment — wire integrity):**

The biggest STATS-drift risk is **not** ARM padding on the wire — current L072 code already serializes STATS manually with `put_u32_le()`, not a struct memcpy. The risk is **H7 mirror drift and parser expectations**.

**Decisions:**
- **Reject `__attribute__((packed))`** on STATS structs (Gemini #1 was correct in spirit; §16.7 confirms this supersedes the original packed-struct recommendation).
- **Manual serialization is the source of truth.** The C struct is a convenience for in-RAM accounting; the wire is whatever `host_stats_serialize()` writes. Therefore the discipline is on the **serializer**, not on struct layout.
- **Explicit `HOST_STATS_OFFSET_*` constants in `host_types.h`** for every field, used by both serializer (L072) and parser (H7). Drift in either side = test failure.
- **Golden-vector tests (per §16.7):**
  - `stats_golden_v1_64.bin`: a known-good 64 B STATS payload from pre-PR#5 firmware. Both L072 serializer and H7 parser must produce/consume it byte-identically.
  - `stats_golden_v1_68.bin`: a known-good 68 B STATS payload from post-PR#5 firmware. H7 parser tests cover **both** golden vectors under `HOST_WIRE_SCHEMA_VER == 1` (additive-tail tolerance).
- **New CI guard `check-stats-offsets`** (added to §10): host-CC compiles a tiny program that prints every `HOST_STATS_OFFSET_*` constant, diffs against a checked-in baseline, and additionally verifies the golden vectors round-trip through both serializer and parser. Strictly stronger than SHA-based vendor-drift.

---

## 9. Detailed work — PR#6 `feat/radio-fhss-deterministic-bench` (renamed per §16.5)

**Scope label (per §16.5):** This is **deterministic bench FHSS**, not a regulatory compliance feature. The 16-channel + 200 kHz spacing layout is engineering hygiene to validate the hop machinery, **not** a Part 15.247 channel plan. Production FHSS with an approved regional channel table + dwell ledger is deferred to a future tranche once the regulatory profile is formally chosen. New CFG concept (deferred): `reg_profile` enum (`REG_PROFILE_BENCH_16`, `REG_PROFILE_US915_PRODUCTION_TBD`, ...) — documented here, **not** added to `host_cfg_keys.h` yet to avoid premature commitment.

### 9.1 C8 — Deterministic FHSS

**New file:** `radio/sx1276_fhss.c` + `radio/sx1276_fhss.h`.

**API:**
```c
void     sx1276_fhss_init(void);
uint8_t  sx1276_fhss_channel_for(uint64_t epoch_ms, uint32_t now_ms);  // returns 0..15
uint32_t sx1276_fhss_freq_hz(uint8_t channel_idx);                     // 902.3 + 0.2*idx MHz, BENCH layout
void     sx1276_fhss_set_epoch(uint64_t epoch_ms);                     // peer epoch sync (§16.3)
```

**Implementation:**
- Honors `cfg.fhss_enable` (key 0x05) — when off, returns fixed channel 0.
- Honors `cfg.fhss_channel_mask` (key 0x07) — 16-bit mask of which channels are in the rotation.
- Honors `cfg.fhss_dwell_ms` (key 0x13, default 400 ms).
- Schedule: deterministic round-robin over `__builtin_popcount(mask)` enabled channels indexed by `((now_ms - epoch_ms) / fhss_dwell_ms) mod popcount(mask)`. **No PRNG, no learning** — both deferred to T7.
- `cfg.fhss_quality_aware` (key 0x06) — apply-hook continues to reject (returns `APPLY_FAILED`); unblock lands T7.

### 9.1.1 Epoch synchronization (added per §16.3 — required for T5-C bidirectional)

A shared 32 MHz TCXO removes clock-rate drift but does **not** make two boards agree on the schedule phase. Deterministic FHSS needs both peers to compute the same `(now - epoch) / dwell` index. Three options were considered (§16.3); the plan picks **option B (sequence-derived hopping)** for T5 because it requires no new wire surface and the L072 already maintains a TX/RX sequence counter that survives reboots within a session:

- **Picked: Sequence-derived hopping.** Channel index = `((tx_seq_counter) mod popcount(mask))`. Both peers increment the counter on every successful TX/RX and converge after one bidirectional exchange. The receiver's RX-arm uses the *expected next* sequence number to choose the channel.
- **Bootstrap:** on PR#6 boot, both peers start at `tx_seq_counter = 0` and listen on **channel 0 only** for the first beacon (or first frame). Once one frame arrives, both lock to the sequence-derived schedule.
- **Loss recovery:** if RX misses N consecutive expected frames (default `N = 4`, settable via TBD CFG key in a follow-up), the receiver reverts to **channel-0 listen** until the next bootstrap frame.
- **Why not option A (master epoch handshake):** requires defining a new wire message and the H7 doesn't yet own time-of-day in T5.
- **Why not option C (RX scan window):** much higher latency on first lock; harder to bench-validate cleanly.

**T5-C downgrade clause:** if epoch sync via sequence-derived hopping cannot be made to work in the bench window, T5-C downgrades to a **one-way distribution test** — transmitter rotates channels, receiver scans all and counts; we verify cadence + dwell + air-time accounting without requiring a closed loop. Bidirectional ping-pong then defers to whatever tranche lands a real epoch handshake.

- T8 deep-sleep work needs RTC-based epoch sync that survives sleep (sequence counters reset on wake). Recorded as **deferred-tech-debt T5-D2** (refined: peer-to-peer epoch sync that survives sleep, beyond TCXO accuracy and beyond sequence-derived hopping).

### 9.2 B5 — Three new CFG keys (revised per §16.4 — was four, drop `tx_power_step_db`)

Add to `host_cfg_keys.h`:
- `0x10 lbt_max_backoff_ms` — uint16, range 10–5000, default 500
- `0x11 cad_symbols` — uint8, range 1–10, default 4 (number of LoRa symbols to listen for in CAD)
- ~~`0x12 tx_power_step_db`~~ — **removed per §16.4** (TX-power adaptation moved to H7; step size is an H7-side parameter, not a wire-visible CFG key)
- `0x13 fhss_dwell_ms` — uint16, range 50–2000, default 400

**CFG-table hygiene (per §16.9):**
- `0x0F` remains an **unused gap** in the descriptor table — document this in `host_cfg_keys.h` with a comment so a future engineer doesn't accidentally re-use it for a semantically-different key.
- Update `CFG_KEY_COUNT` to match the actual descriptor table count (was 17; T5 adds 3 keys with a gap at 0x0F, so verify against descriptor count, not max-key-value).
- `cfg_contract.c` coverage extends to: 0x0F (must reject as `KEY_UNKNOWN`), all three new keys (good values), bad lengths, out-of-range values, readback after apply.
- `cfg_wire_vectors.c` golden TLVs for the three new keys.

For each new key: validator (range check), apply-hook (atomic update of in-RAM cfg), addition to `cfg_wire_vectors.c` test vectors. Per existing T2/T3 pattern.

**Files modified:**
- New: `radio/sx1276_fhss.{c,h}`, `bench/host_proto/fhss_vectors.c`
- `radio/sx1276_tx.c` — invoke FHSS channel selection (sequence-counter input)
- `radio/sx1276_rx.c` — invoke FHSS channel selection on rx-arm (expected-next-seq input)
- `host/host_cfg_keys.h` — add three keys + 0x0F gap comment + updated `CFG_KEY_COUNT`
- `host/host_cfg.c` — three validators + apply-hooks
- `bench/host_proto/cfg_contract.c` — extend coverage to new keys + 0x0F unknown-key case
- `bench/host_proto/cfg_wire_vectors.c` — golden TLVs for new keys

---

## 10. CI / static-check additions

| New target | What it guards |
|---|---|
| `check-host-cc` | host-CC unit tests for COBS+CRC roundtrip + TX_DONE parsing pass |
| `check-host-vendor-drift` | H7-vendored `mh_*` files match L072 originals (modulo header-comment normalization) |
| `check-stats-offsets` (Gemini review #1) | host-CC builds of `host_types.h` and `mh_types.h` produce identical `offsetof` outputs for every STATS member; catches semantic-equivalence violations stronger than SHA-diff |
| `check-lbt-owner` | `radio_tx_abort_lbt` written only from `sx1276_lbt.c` |
| `check-airtime-counter-owner` | `radio_tx_abort_airtime` written only from `sx1276_airtime.c` |
| `check-fhss-owner` | `RegFrf*` set only via `sx1276_modes_set_freq_hz`; FHSS channel decision only in `sx1276_fhss.c` |
| `check-stats-payload-size` | enforce `HOST_STATS_PAYLOAD_BYTES == 68` post-T5 (catches accidental shrink) |
| `check-no-arduino-in-host-headers` (Gemini review #3) | grep guard: no `#include <Arduino.h>` or `HardwareSerial` reference in any `mh_*.h` header |

All wired into the existing `firmware-l072-static-checks` job in `.github/workflows/arduino-ci.yml`.

---

## 11. Risks & mitigations

| Risk | Likelihood | Mitigation |
|---|---|---|
| **HSI clock drift desyncs FHSS in <100 s on T5-C bench (Gemini #2)** | **H if SYSCLK on HSI; ~Nil if on HSE/TCXO** | **PR#0 `chore/clock-source-hse` is a hard gate before PR#6.** Switch SYSCLK to HSE-direct (TCXO ±2 ppm); STM32L072-specific RCC code (per §16.2 — not L4 PLLM/PLLN/PLLR); MCO or GPIO-toggle frequency check in BRINGUP Stage 1; `HOST_FAULT_CODE_CLOCK_HSE_FAILED = 0x08` + `clock_source_id` byte at existing reserved BOOT_URC offset (not byte 0) for in-band visibility. |
| **FHSS phase desync even with shared TCXO (§16.3)** | **H if no epoch sync** | PR#6 implements **sequence-derived hopping** (channel index from shared TX/RX seq counter). T5-C downgrades to one-way distribution test if epoch sync fails. Loss-recovery: receiver reverts to channel-0 listen after N missed frames. |
| **TX-power adaptation crosses Crypto Profile A boundary (§16.4)** | H (architectural) | **EWMA + decision logic moved to H7** (`tractor_h7/src/murata_host/tx_power_adapter.{h,cpp}`). L072 only applies `CFG_SET tx_power_dbm`. ACK identification uses H7's decrypted view, never L072 inspection of payloads. |
| 16-channel FHSS misread as Part 15.247 compliance (§16.5) | M | PR#6 explicitly labelled "deterministic bench FHSS"; `REG_PROFILE_*` enum sketched but **not** committed to wire; airtime guard documented as engineering-conservative bench limit, not regulatory enforcement. Production regulatory tranche scheduled separately. |
| CAD ships untested behind RSSI-only T5-A (§16.6) | M | T5-A split into **T5-A1 (CW → RSSI path)** and **T5-A2 (LoRa preamble → CAD path)**. Both are PR#4 acceptance gates. |
| Vendored `mh_types.h` drifts silently from `host_types.h` (Gemini #1 + §16.7) | M | Manual serializer is source of truth; explicit `HOST_STATS_OFFSET_*` constants; golden vectors `stats_golden_v1_64.bin` + `stats_golden_v1_68.bin` round-trip through both serializer and parser; `check-stats-offsets` CI guard. |
| H7 driver's headers leak `Arduino.h` and break host-CC build (Gemini #3) | L (caught at first host build) | Strict file split: `mh_uart.h` interface-only, `mh_uart_arduino.cpp` target-only, `mh_uart_posix.cpp` host-only; CI guard `check-no-arduino-in-host-headers`. |
| H7 build entry point is not actually `platformio.ini` (§16.10) | M | PR#1 first inspects the real entry point and chooses the least-invasive flag-injection path; CI must build both variants from the actual repo build path. |
| Airtime estimator drifts from TX_DONE accounting (§16.8) | M | Refactor existing `sx1276_tx_estimate_toa_us()` into `sx1276_airtime.c`; both reservation and TX_DONE reporting call the same function. Documented rollback semantics for LBT-reject / TX-timeout / TX-abort cases. |
| Strict-additive STATS payload bump from 64→68 B violated by a downstream parser | L | Re-audit `host_types.h` policy comment; if non-tolerant, escalate before merge of PR#5. Worst case: T5-D3 → bump `HOST_WIRE_SCHEMA_VER` to 2 *here* (would re-order the T4 §12 schedule by one tranche). |
| LBT/CAD adds enough latency to push frame budget into airtime overshoot | M | Air-time module accounts for the actual on-air time, not LBT overhead — should be fine. Validate in T5-D bench test. |
| CAD→TX race window admits interferer collision (Gemini #4) | L | ~100 µs window; inherent to non-MAC RF; absorbed by T5-A1's ±10% tolerance. Accepted as design limit. |
| H7 driver vendor-drift check is a treadmill | M | Accept the cost: every L072 wire change must be deliberate on H7 side. If it gets painful, we know the wire is unstable and that itself is a finding. |
| TX-power adaptation oscillates near a sharp gain cliff | L | 5-ACK hysteresis + 1 dB step (H7-side). If observed in T5-B bench test, increase hysteresis to 8 ACKs before merge. |
| RTC BKP fault persistence (T5-D1) bites us in field before T8 | L (no field deploys planned this tranche) | Track in TODO; any pre-T8 field deploy escalates to a hot-patch PR. |
| CAD→TX race window admits interferer collision (Gemini #4) | L | ~100 µs window; inherent to non-MAC RF; absorbed by T5-A's ±10% tolerance. Accepted as design limit. |
| H7 driver vendor-drift check is a treadmill | M | Accept the cost: every L072 wire change must be deliberate on H7 side. If it gets painful, we know the wire is unstable and that itself is a finding. |
| TX-power adaptation oscillates near a sharp gain cliff | L | 5-ACK hysteresis + 1 dB step. If observed in T5-B bench test, increase hysteresis to 8 ACKs before merge. |
| RTC BKP fault persistence (T5-D1) bites us in field before T8 | L (no field deploys planned this tranche) | Track in TODO; any pre-T8 field deploy escalates to a hot-patch PR. |

---

## 12. Deferred-tech-debt log (created this tranche)

| ID | Item | Tranche to resolve |
|---|---|---|
| T5-D1 | Move fault replay storage from `.noinit` RAM to RTC BKP0–4 | T8 (RTC LSE wake init pulls in BKP write for free) |
| T5-D2 | Peer-to-peer epoch sync that survives sleep (beyond TCXO accuracy and beyond sequence-derived hopping) | T8 (deep-sleep RTC sync) |
| T5-D3 (contingent) | If STATS payload-additive isn't strictly tolerated, bump `HOST_WIRE_SCHEMA_VER` to 2 in T5 instead of T8 | T5 PR#5 (escalation only) |
| T5-D4 | If PR#0 cannot land in T5, T5-C is voided and FHSS validation defers | escalation only — should not happen |
| T5-D5 (per §16.5) | Production regulatory FHSS profile (`REG_PROFILE_US915_PRODUCTION_TBD`): formal channel table, dwell ledger, BW choice, regulatory rule citation | future tranche, before any field deploy |
| T5-D6 (per §16.4) | If H7-side `TxPowerAdapter` proves too coupled to undefined ACK semantics, fall back to manual `tx_power_dbm` only and defer adaptation to T6/T7 after ACK surface formalization | T5 PR#5 escalation → T6/T7 |

---

## 13. Acceptance review checklist (for the reviewer of T5 plan)

- [ ] **PR#0 `chore/clock-source-hse` is gated before PR#6 (FHSS) — non-negotiable per Gemini review #2.**
- [ ] PR#0 implementation uses STM32L072-specific RCC fields (PLLMUL/PLLDIV, not L4 PLLM/PLLN/PLLR); HSE-direct preferred over PLL when 32 MHz HSE is valid (per §16.2).
- [ ] PR#0 does not overwrite BOOT_URC byte 0 (`reset_cause`); `clock_source_id` placed at existing reserved tail offset.
- [ ] PR#6 includes epoch-sync mechanism (sequence-derived hopping) **or** T5-C is downgraded to one-way distribution test (per §16.3).
- [ ] TX-power adaptation logic lives in H7 (`tx_power_adapter.{h,cpp}`), not L072 (per §16.4); L072 only applies `CFG_SET tx_power_dbm`.
- [ ] PR#6 labelled "deterministic bench FHSS"; airtime guard documented as engineering bench limit, not Part 15.247 (per §16.5).
- [ ] T5-A split into T5-A1 (RSSI/CW) and T5-A2 (CAD/LoRa preamble); both are PR#4 acceptance gates (per §16.6).
- [ ] STATS golden vectors `stats_golden_v1_64.bin` + `stats_golden_v1_68.bin` exist; H7 parser tests cover both under `HOST_WIRE_SCHEMA_VER == 1` (per §16.7).
- [ ] PR#5 reuses `sx1276_tx_estimate_toa_us()` (refactored into `sx1276_airtime.c`), not a second formula (per §16.8); rollback semantics documented for LBT-reject / TX-timeout / TX-abort.
- [ ] PR#6 updates `CFG_KEY_COUNT` to descriptor-table count, not max-key-value; documents 0x0F gap; `cfg_contract.c` covers 0x0F as `KEY_UNKNOWN` (per §16.9).
- [ ] PR#1 inspects actual `tractor_h7` build entry point (Arduino vs PlatformIO) and adds Method G variant there; both variants compile from current repo build path (per §16.10).
- [ ] Carryover scope (PR#1–#3) acknowledged and gated before T5-proper PRs.
- [ ] No T7 work (quality-aware FHSS, replay fuzzer) leaked into T5.
- [ ] No T8 work (deep sleep, RTC BKP fault persistence) leaked into T5.
- [ ] STATS payload bump 64→68 B is strictly additive and does not bump `HOST_WIRE_SCHEMA_VER`.
- [ ] STATS struct is natural-aligned (no `__attribute__((packed))`); manual serializer is source of truth; explicit `HOST_STATS_OFFSET_*` constants exist.
- [ ] H7 driver headers contain no `Arduino.h` / `HardwareSerial` references; transport split between `mh_uart_arduino.cpp` and `mh_uart_posix.cpp` is enforced.
- [ ] Three new CFG keys (0x10, 0x11, 0x13 — 0x12 dropped per §16.4) follow the T2/T3 validator + apply-hook + cfg_wire_vector pattern.
- [ ] All new modules respect the sole-writer disciplines (OpMode → modes; LBT counter → lbt; airtime counter → airtime; FRF → modes).
- [ ] CI gains eight new check targets, all wired into the existing static-checks job.
- [ ] Bench success criteria T5-A1/A2 through T5-D are measurable on hardware planned for T4 PR#6 bring-up; BRINGUP Stage 1 includes frequency verification of HSE/TCXO.
- [ ] Deferred-tech-debt log is honest about what we're not doing.

---

## 14. One-line summary

> **Land PR#0 `chore/clock-source-hse` first (STM32L072-specific HSE-direct switch — PLLMUL/PLLDIV not L4 PLLM/PLLN/PLLR — with `HOST_FAULT_CODE_CLOCK_HSE_FAILED = 0x08` and `clock_source_id` placed at the existing BOOT_URC reserved tail offset, never overwriting `reset_cause` byte 0); close out T4 carryover (PR#1 minimal H7 Method G driver behind `LIFETRAC_USE_METHOD_G_HOST=1` with the actual `tractor_h7` build entry point inspected and a strict `mh_uart.h`/`mh_uart_arduino.cpp`/`mh_uart_posix.cpp` transport split, PR#2 host-CC unit tests + PTY loopback + SHA vendor-drift + `check-stats-offsets` semantic-equivalence guard + golden STATS vectors `stats_golden_v1_64.bin` and `stats_golden_v1_68.bin` round-tripped through both serializer and parser, PR#3 `BRINGUP_MAX_CARRIER.md` + openocd cfg with frequency verification of HSE in Stage 1); then in T5 proper land four PRs (PR#4 LBT + CAD with binary-exponential backoff and the CAD path validated by **T5-A2** LoRa-preamble interferer test in addition to **T5-A1** CW/RSSI; PR#5 air-time accounting reusing the existing `sx1276_tx_estimate_toa_us()` refactored into `sx1276_airtime.c` with documented LBT-reject/TX-timeout/TX-abort rollback, plus a strictly-additive natural-aligned 68 B STATS payload guarded by manual `HOST_STATS_OFFSET_*` constants — **TX-power adaptation lives in the H7's new `TxPowerAdapter` class, not on the L072, preserving the Crypto Profile A boundary**; PR#6 deterministic **bench-only** FHSS over `cfg.fhss_channel_mask` with **sequence-derived epoch sync** so two boards converge after one bidirectional exchange, three new CFG keys 0x10/0x11/0x13 with 0x0F gap documented and `CFG_KEY_COUNT` matched to descriptor table, regulatory profile deliberately deferred as `T5-D5`); bench validation = T5-A1 (RSSI), T5-A2 (CAD), T5-B (H7-driven TX-power tracks attenuation via `CFG_SET tx_power_dbm`), T5-C (30-min FHSS ping-pong, downgrades to one-way distribution test if epoch sync fails), T5-D (air-time guard rejects intentional overshoot — engineering bench limit, not Part 15.247 compliance); `HOST_PROTOCOL_VER` and `HOST_WIRE_SCHEMA_VER` stay at 1/1 (T8 remains the first and only wire-schema bump in the roadmap); six deferred-tech-debt items filed (T5-D1 RTC BKP fault persistence → T8, T5-D2 peer epoch sync that survives sleep → T8, T5-D3 contingent wire-schema bump → escalation only, T5-D4 contingent if PR#0 cannot land → escalation only, T5-D5 production regulatory FHSS profile → future tranche before field deploy, T5-D6 contingent fallback to manual TX-power if H7 adapter is too coupled to undefined ACK semantics → T6/T7).**

---

## 15. Review & System Architecture Analysis (Gemini 3.1 Pro)

**Review Date:** 2026-05-05
**Reviewer:** Gemini 3.1 Pro (Preview)

The Tranche 5 channel hygiene and link adaptation plan defines a rigid, highly protective layer over the RF link. Consolidating the T4 carryovers as hard PR gates ensures the fundamental host driver works before you saturate the environment with frequency hops and dynamic TX adjustments.

Here are the key technical implications and architectural flags for this phase:

### 1. The 64B $\rightarrow$ 68B Payload Bump & Struct Alignment
Moving `host_stats_wire_t` from exactly 64 bytes to 68 bytes introduces a critical cross-architecture struct padding risk. 
*   **The Hazard:** The Portenta H7 features a Cortex-M7 core, standardizing on 32-bit or 64-bit alignments. The Murata STM32L072 is a Cortex-M0+, using a more constrained alignment. If the 68-byte struct (which drops a nice power-of-two alignment) contains `uint8_t` or `uint16_t` interlaced with `uint32_t`, the H7 receiver might implicitly pad the struct differently than the L072 sender emitted it over COBS.
*   **Recommendation:** Strictly enforce `__attribute__((packed))` on the structs in `host_types.h` and explicitly evaluate the payload via CI `check-size` constraints.

### 2. T5-D2 Clock-Drift Desync during 30-Minute Bench Test
The T5-C bench criterion requires a two-board ping-pong over 16 channels for 30 minutes without losing synchronization. T5 defers formal clock synchronization (T5-D2) to Tranche 8.
*   **The Hazard:** Pure `now_ms` based deterministic FHSS relies on `HAL_GetTick()`. If you derive `now_ms` from an uncalibrated HSI (1% error max at room temp), the clocks could theoretically drift by up to **18 seconds** over a 30-minute span. With a `fhss_dwell_ms` of 400ms, the nodes will diverge and become permanently stuck on disjoint channels.
*   **Recommendation:** To pass T5-C out-of-the-box, ensure the `SysTick` generating `now_ms` relies on the highly accurate Murata TCXO (externally driving the PLL) rather than the default internal HSI RC oscillator.

### 3. H7 PTY Loopback Test viability
The PTY loopback strategy for the H7 `MurataHost` driver is an excellent pattern for hardware-in-the-loop (HIL) reduction. 
*   **The Hazard:** The Portenta H7 Arduino Core heavily bakes `Arduino.h` (like `HardwareSerial` and `Stream`) into driver classes. Compiling this natively on a Linux/Windows host workstation with GCC/Clang will fail immediately on missing headers.
*   **Recommendation:** Define a pure virtual transport interface (`IUartTransport`) inside `mh_uart.h` that the `MurataHost` class consumes. At compile-time, inject the `HardwareSerial` wrapper for the H7 target, and a POSIX/Win32 file-descriptor wrapper for the Native Host loopback target.

### 4. CAD vs RSSI Race Visibility
Placing CAD right before LBT RSSI is conceptually sound but note the temporal blind spot. CAD takes multiple symbols. If CAD evaluates clear, but an interferer keys up *during* the transition from CAD evaluation to RSSI read, the L072 might transmit anyway. This is completely standard behavior for asynchronous RF collision detection and is acceptable, but be aware of the race condition limit when viewing bench test T5-A logs.

---

## 16. Review & System Architecture Analysis (GitHub Copilot v1.0)

**Review Date:** 2026-05-05
**Reviewer:** GitHub Copilot
**Scope:** Tranche 5 plan checked against the current `murata_l072` tree, especially `host_types.h`, `host_cmd.c`, `host_stats.c`, `host_uart.c`, `sx1276_tx.c`, `sx1276_rx.c`, `sx1276_modes.c`, and `platform.c`.

### 16.1 Executive Verdict

Approve the T5 direction with amendments. The plan correctly gates channel hygiene behind the unfinished H7 host driver and correctly adds PR#0 as a clock-domain gate before deterministic FHSS. The current firmware tree already contains more T4 work than the plan text implies: centralized type IDs, `RADIO_IRQ_URC = 0xC0`, `REG_WRITE_ACK_URC = 0xB1`, binary `VER_URC`, reset-cause `BOOT_URC`, HT/TC DMA servicing, CFG contract tests, TX/RX/modes modules, and a 64-byte STATS payload.

The most important T5 corrections are:

1. PR#0 must be rewritten against the real STM32L072 RCC model and the actual Murata clock routing, not generic PLLM/PLLN/PLLR terminology.
2. Deterministic FHSS needs an epoch/schedule-sync mechanism for T5-C; a TCXO fixes drift but not startup phase alignment.
3. TX-power adaptation must not make the L072 parse encrypted application ACKs if the launch role remains a thin modem with H7-owned crypto.
4. The 16-channel FHSS and 400 ms/s/channel language must be labeled bench/engineering hygiene until the regulatory channel plan is formally chosen.

### 16.2 Blocker - PR#0 Clock Gate Is Correct, But the Implementation Detail Is Not Yet Safe

PR#0 is the right hard gate before FHSS. Current `platform.c` still runs `platform_clock_init_hsi16()` with `s_core_hz = 16000000`, so the desync risk is real. However, the proposed fix uses STM32L4-style PLL terms (`PLLM`, `PLLN`, `PLLR`) and assumes the Murata 32 MHz TCXO is directly usable as the L072 SYSCLK source.

Before coding PR#0:

- Verify from the Murata CMWX1ZZABZ-078 datasheet/schematic whether the 32 MHz source is wired to STM32L072 `OSC_IN` as HSE, whether it requires BYPASS mode, and whether the host can observe it on any MCO-capable pin actually routed on the Max Carrier.
- Replace the generic PLL recipe with STM32L072-specific RCC fields and valid frequency limits. If 32 MHz HSE direct is valid and stable, prefer direct HSE over an unnecessary PLL.
- Do not overwrite `BOOT_URC` byte 0. Current firmware uses byte 0 for `reset_cause`; the new `clock_source_id` should use the existing reserved tail byte at offset 5 or a clearly versioned tail extension.
- Add `HOST_FAULT_CODE_CLOCK_HSE_FAILED = 0x08` to `host_types.h` and treat `clock_source_id != HSE_OK` as a hard H7-side veto for FHSS enable.

Acceptance gate amendment: PR#0 must prove both `platform_core_hz()` and SysTick-derived `platform_now_ms()` are based on the selected external clock. A register-level RCC assertion alone is not enough.

### 16.3 Blocker - Deterministic FHSS Needs Epoch Sync, Not Only Low Drift

The TCXO solves long-run drift. It does not make two boards agree on `now_ms / fhss_dwell_ms` unless they share an epoch. The plan says T5 assumes boards are started "within seconds," but with `fhss_dwell_ms = 400`, the peers need to agree within roughly `dwell/4` (~100 ms) by the plan's own tolerance.

PR#6 needs one of these before T5-C can be meaningful:

- **Master epoch handshake:** fixed bootstrap channel, base/tractor exchanges `epoch_ms`, follower applies a signed offset to its FHSS schedule.
- **Sequence-derived hopping:** channel is derived from shared frame sequence/hop counter, not local uptime. This is closer to the existing `lp_fhss_channel_index(key_id, hop_counter)` helpers.
- **RX scan window:** receiver scans all enabled channels around expected dwell boundaries until it locks, then tracks the peer.

If none of those lands in T5, change T5-C from "two-board ping-pong" to a one-way distribution test that only verifies the transmitter's channel rotation and dwell accounting. Otherwise the bench test can fail even with perfect clocks.

### 16.4 Blocker - TX-Power Adaptation Crosses the Thin-Modem Boundary

The launch architecture still says H7 owns AES-GCM and the L072 transports authenticated bytes. Under that model the L072 cannot inspect application payloads to identify ACK frames or match ACK sequence numbers unless the ACK metadata is outside the encrypted on-air payload.

T5 should choose one of these contracts:

- **Recommended for launch:** H7 owns TX-power adaptation. L072 reports RX SNR/RSSI in `RX_FRAME_URC`; H7 computes EWMA and sends `CFG_SET tx_power_dbm`. The L072 only applies the configured power.
- **Alternative:** add unencrypted modem metadata to `TX_FRAME`/`RX_FRAME_URC` that lets the L072 correlate ACK-like events without reading application payloads. This needs a host-protocol contract and threat-model review.
- **Defer:** keep TX-power adaptation manual/config-only in T5 and move automatic adaptation to T6/T7 after ACK semantics are formalized.

As written, PR#5 says the adaptation logic lives in `host/host_cmd.c` plus `radio/sx1276_tx.c`, but it depends on ACK semantics that are explicitly "to be defined." That should be a blocking precondition for T5-B, not an implementation placeholder.

### 16.5 High - 16-Channel FHSS Is a Bench Feature Until the Regulatory Baseline Is Closed

The plan moves to 16 channels at `902.3 + 0.2*idx MHz`, while earlier code and review history centered on an 8-channel helper and a separate unresolved `>=50`-channel FCC 15.247 question. A 200 kHz channel spacing also overlaps at BW250/BW500 and does not map cleanly to the plan's "400 ms in any 1 s per 500 kHz channel" language.

Recommendation:

- Rename PR#6's target to **deterministic bench FHSS** unless a regulatory profile table is approved first.
- Put the regulatory operating mode in config: `REG_PROFILE_BENCH_16`, `REG_PROFILE_US915_PRODUCTION_TBD`, etc.
- Do not describe the T5 airtime guard as enforcing Part 15.247 compliance. It is a conservative engineering limiter until the channel count, bandwidth, dwell window, and rule basis are frozen.
- If production FHSS is still the goal, add a later work item that replaces bench 16-channel hopping with the approved region table and dwell ledger.

This does not block T5 as a bench-validation tranche, but it blocks any claim that T5 closes regulatory compliance.

### 16.6 High - LBT/CAD Bench Tests Need Two Interference Cases

T5-A uses a +0 dBm CW interferer. That is a useful RSSI/LBT test, but it does not validate CAD because SX1276 CAD detects LoRa preambles, not arbitrary CW energy.

Amend PR#4 acceptance:

- **T5-A1 RSSI LBT:** conducted or shielded CW/noise interferer, with attenuators so the radio front-end and test equipment are not overdriven.
- **T5-A2 CAD path:** known-good LoRa preamble/packet interferer on the same channel, proving `CadDetected` handling and fallback behavior.

The plan's CAD race note is good. Keep it, but add the second test so CAD does not ship as untested code behind an RSSI-only bench result.

### 16.7 High - Current Wire Serialization Reduces, But Does Not Eliminate, STATS Drift Risk

The current L072 code serializes STATS manually with `put_u32_le()` rather than memcpying a C struct. That is good: the biggest risk is not ARM padding on the wire, but H7 mirror drift and parser expectations.

Keep the v1.1 decision to reject `__attribute__((packed))`. Also add:

- A golden 64-byte STATS vector before PR#5 and a golden 68-byte vector after PR#5.
- H7 parser tests that accept 64-byte and 68-byte STATS payloads while `HOST_WIRE_SCHEMA_VER == 1`, ignoring only documented trailing fields.
- Explicit field-offset constants in `host_types.h` or a generated manifest. Do not rely only on `host_stats_wire_t` because the wire function is manual serialization.

This also means the Gemini §15 packed-struct recommendation should be treated as superseded by the v1.1 natural-alignment + explicit-offset discipline.

### 16.8 Medium - Reuse the Existing Airtime Estimator

`sx1276_tx.c` already contains `sx1276_tx_estimate_toa_us()`, which reads the active modem registers and computes estimated LoRa airtime. PR#5 should not create a second formula that can drift from TX_DONE behavior.

Refactor the existing estimator into `sx1276_airtime.c` and make both TX_DONE reporting and airtime budget reservation call the same implementation. Also specify reservation rollback semantics:

- If LBT rejects before TX, no airtime is reserved.
- If TX starts and times out, either commit the estimated airtime conservatively or release only after proving no RF energy was emitted.
- If TX aborts before `sx1276_modes_to_tx`, release the reservation.

### 16.9 Medium - CFG Key Additions Need Table-Count and Gap Hygiene

Current `host_cfg_keys.h` ends normal runtime keys at `0x0E`, with read-only keys at `0x80` and `CFG_KEY_COUNT = 17`. Adding `0x10` to `0x13` leaves `0x0F` unused and changes table count expectations.

This is fine, but PR#6 should explicitly update:

- `CFG_KEY_COUNT` to match the actual descriptor table count, not the max key value.
- `cfg_contract.c` coverage for missing `0x0F`, all new keys, bad lengths, out-of-range values, and readback after apply.
- `cfg_wire_vectors.c` golden TLVs for the four new keys.

### 16.10 Medium - PR#1 H7 Driver Build-System Assumption Needs Verification

The plan references `tractor_h7/platformio.ini`, but the current `tractor_h7` folder is Arduino-sketch style and does not obviously contain that file. PR#1 should start by choosing the actual H7 build entry point and adding the Method G variant there.

Acceptance amendment: both the legacy KISS build and the Method G host-driver build must compile in the current repository build path, not only in a proposed PlatformIO path.

### 16.11 Bottom Line

Proceed with T5 after tightening the gates. The most important changes are: make PR#0 hardware-accurate for STM32L072, add FHSS epoch synchronization or relax T5-C, move automatic TX-power adaptation to the H7 unless new modem metadata is defined, and label 16-channel FHSS as bench-only until the regulatory baseline is closed. With those amendments, T5 is a useful and well-bounded channel-hygiene tranche rather than a premature production RF compliance claim.
