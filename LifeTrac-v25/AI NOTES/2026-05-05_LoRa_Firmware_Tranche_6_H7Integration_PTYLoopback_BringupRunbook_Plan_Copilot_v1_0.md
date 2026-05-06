# LoRa Firmware Tranche 6 — H7 Runtime Integration, PTY Loopback Harness, Bring-up Runbook (Copilot v1.2)

Date: 2026-05-05
Status: REVISED — Gemini §15 + Copilot self-review §16 + ABX00043 Max Carrier schematic trace
Author: GitHub Copilot (Claude Opus 4.7)

> **Predecessor docs:** `2026-05-05_LoRa_Firmware_Tranche_5_ChannelHygiene_LinkAdaptation_Plan_Copilot_v1_0.md` (v1.2 plan), `2026-05-05_LoRa_Firmware_Tranche_5_Implementation_Copilot_v1_0.md` (L072 pass), `2026-05-05_LoRa_Firmware_Tranche_5_Implementation_Copilot_v1_1.md` (H7 host-wire pass).

### Change log

- **v1.2 (this revision)** — ABX00043 Max Carrier schematic trace folded in (sheets 1-14): LoRa SiP `SERIAL` net (USART2 PA2/PA3 + RTS/CTS, 4-wire HW flow control) leaves sheet 6 and lands on one of `SERIAL{0,1,2}` to the HD connector on sheet 1; **`SERIAL3` is reserved by the on-carrier Cellular Modem (sheet 5) — NEVER set `LIFETRAC_MH_SERIAL=Serial3`**; Arduino mbed `Serial1` is already consumed by the X8↔H747 link in `tractor_h7.ino` line 1788 — also excluded; viable bench candidates narrow to `Serial2`/`Serial4`/`Serial5` (final selection requires sheet-1 zoom + mbed `variant.cpp` cross-ref). **CN2 SWD debug header is DNP** (depopulated by default per X marks on sheet 6) — ST-Link path requires soldering 2x5 1.27 mm header OR using BOOT0/DFU which is wired via R19. **On-carrier USB debugger** (sheet 3 STM32F405 with `UART_SNIFF1..5` channels) provides passive UART snooping without extra cables — added to runbook hardware checklist. **`LORA_RST` shares NRST with STSAFE secure element** (sheet 6 pin 34 = `MCU_NRST/STSAFE_RST`) — chip-erase via OpenOCD also resets STSAFE; new R10. **`SHARED PINS on H7` warning** (sheet 2 HD connector note) — chosen `SERIALn` may multiplex with another H7 peripheral; new R11 mandating mbed-core pinmap audit. RS-485 fieldbus path (sheet 13 `SERIAL-485` → SP335 transceiver) confirmed independent of `SERIALn` (uses USART1 PA9/PA10 via `RS485.setPins()`); does not contend with Method G.
- **v1.1** — Folded in Gemini §15 (symbol-set guard pulled into PR#3, Serial2 schematic verification, cross-platform mock noted as future) and Copilot §16 (12 amendments): real Method G ABI names everywhere (`PING_REQ` echo, `STATS_DUMP_REQ`, `CFG_DATA_URC`, `TX_FRAME_REQ`, `RX_FRAME_URC`), mock imports from `mh_wire.h` and asserts `ver == HOST_PROTOCOL_VER`, Method G runtime made mutually exclusive with legacy RadioLib via `#if/#else` around `radio.begin`/`poll_radio`/`poll_link_ladder`/`tx_pump`, API-lift `mh_stream` layer added to PR#2 (stream unframer + seq allocator + request helpers), CI build glue (inline `cc` invocation; H7 `arduino-cli` job decision documented), symbol-set guard `check-mh-runtime-off-symbols` added in PR#3, `clock_source_id=0` corrected for HSE OK, MCO check reframed as conditional, Stage 2/3 rewritten around real `RX_FRAME_URC` and `TX_FRAME_REQ`/`TX_DONE_URC` (no `RX_START`, no PING_ACK over-air metric), bench-visibility surface in PR#3, partial-read/write loopback cases, 45-constant count, T6_bringup evidence path, `LIFETRAC_MH_SERIAL` schematic-confirmation gate.
- **v1.0** — Initial T6 plan covering three PRs: docs/max-carrier-bringup, feat/h7-pty-loopback-harness, feat/h7-runtime-method-g-host.

## 0. Why this tranche, why now

T5 implementation landed two halves:

- **L072 half** (T5-impl v1.0): PR#0 HSE clock gate, LBT/CAD core, airtime hygiene, additive 68 B STATS, three new CFG keys (`0x10` LBT_MAX_BACKOFF_MS, `0x11` CAD_SYMBOLS, `0x13` FHSS_DWELL_MS, `0x0F` reserved gap), `check-lbt-owner` + `check-airtime-counter-owner` CI guards.
- **H7 half** (T5-impl v1.1): tracked `firmware/tractor_h7/murata_host/` with `mh_wire.h`, `mh_crc16.{h,c}`, `mh_cobs.{h,c}`, `murata_host.{h,c}` inner-frame builder/parser, `mh_stats.{h,c}` 64/68 B tolerant parser; bench tests `mh_cobs_crc_unit.c` + `mh_stats_vectors.c`; `tools/check_mh_wire_sync.py` drift guard; new CI job `h7-host-driver-tests`.

What T5 explicitly **did not** ship — verbatim from the impl notes:

1. *"Integrate `murata_host` into active `tractor_h7.ino` runtime path behind `LIFETRAC_USE_METHOD_G_HOST` flag."*
2. *"Add PTY-based loopback harness (`tools/murata_host_loopback.py`) for end-to-end request/URC smoke tests."*
3. *"Add bring-up runbook docs for Max Carrier SWD/OpenOCD flow."*
4. (From T5-impl v1.0) *"Formal deterministic FHSS bench profile module and epoch-sync plumbing beyond current key/airtime groundwork."*

Items 1-3 are the user's three explicit asks for T6. Item 4 (FHSS PR#6 from T5-plan v1.2) remains carryover — it depends on a working H7↔L072 link to validate, so we land **after** items 1-3 in this tranche or push to T7. Decision: **carryover to T7** (T6 is already three substantial PRs, FHSS deterministic-bench can wait one more cycle and benefits from the runbook + PTY harness being in place first).

## 1. Theme and success criteria

**Theme:** *Make the H7-side Method G driver real* — wire it into the H7 main loop, prove it round-trips against a mocked L072 over PTY without hardware, and write the runbook that lets a bench operator actually flash and validate a real Murata board.

**Success criteria (5):**

1. **S1 — Compile-flag exclusivity (revised per §16.3):** With `LIFETRAC_USE_METHOD_G_HOST=0` (default), the H7 build is byte-identical to pre-T6 in `loop()` AND no `mh_`/`murata_host_` symbols appear in the linked ELF. With `=1`, `murata_host` is initialized in `setup()` and polled in `loop()` AND the legacy RadioLib path (`radio.begin`, `poll_radio`, `poll_link_ladder`, `tx_pump`) is compiled out — Method G is an exclusive replacement, not a sidecar.
2. **S2 — PTY loopback green (revised per §16.2):** A new `tools/murata_host_loopback.py` runs against a `host-cc` build of the H7 driver linked to a POSIX UART transport, and round-trips `HOST_TYPE_PING_REQ` → echoed `PING_REQ` (same seq/flags/payload) + `HOST_TYPE_CFG_GET_REQ` → `HOST_TYPE_CFG_DATA_URC` + `HOST_TYPE_STATS_DUMP_REQ` → `HOST_TYPE_STATS_URC` (both 64 B legacy and 68 B additive payloads) over a `pty.openpty()` pair with a Python L072 mock that imports its type/version constants from `mh_wire.h` (no second manual table).
3. **S3 — Runbook executable (revised per §16.6/16.7):** `BRINGUP_MAX_CARRIER.md` lands with SWD pinout, OpenOCD config files, three flash recipes (SWD via ST-Link, SWD via J-Link, MAX CARRIER USB DFU), three-stage bench procedure that matches **current** L072 behavior — Stage 1: `BOOT_URC.clock_source_id == 0` (HSE OK; `==1` is HSI fallback and is a STOP); Stage 2: `BOOT_URC.radio_ok == 1` + observe `HOST_TYPE_RX_FRAME_URC` from a known-good peer (RX is already armed during L072 boot, no `RX_START` command exists); Stage 3: Board A `TX_FRAME_REQ` → `TX_DONE_URC`, Board B `RX_FRAME_URC` with expected payload (no implied L072 echo). MCO frequency-verification step is documented as a **conditional** check (only valid against a debug L072 artifact that configures MCO; not a hard T6 acceptance gate).
4. **S4 — CI extends, never regresses:** New CI job `h7-host-driver-loopback` runs the PTY harness headless. New CI step `check-mh-runtime-off-symbols` (per §15.1 + §16.5) asserts no `mh_`/`murata_host_` symbols in the default-off ELF. Existing T5 jobs (`h7-host-driver-tests`, `check-stats-offsets`, `check-lbt-owner`, `check-airtime-counter-owner`, `check-cfg-wire-owner`) all stay green.
5. **S5 — No wire-schema bump:** `HOST_PROTOCOL_VER=1`, `HOST_WIRE_SCHEMA_VER=1` unchanged. T8 remains the first/only wire-schema bump in the roadmap.

## 2. What we are explicitly NOT doing in T6

- No FHSS implementation (PR#6 from T5-plan v1.2 → T7 with sequence-derived epoch sync).
- No TX-power adapter on H7 side (T5-plan §8.1 → T7 once link is hot).
- No production regulatory profile (T5-D5 → future tranche).
- No RTC BKP fault persistence (T5-D1 → T8).
- No new CFG keys.
- No Crypto Profile A boundary changes.
- No new STATS fields.
- No changes to `mh_wire.h` constants — `check_mh_wire_sync.py` must continue to pass with the existing **45-constant** census (corrected per §16.10).
- No L072 C/header/linker/Makefile behavior changes (PR#1 *does* add OpenOCD `.cfg` files under `firmware/murata_l072/openocd/` — that is bench tooling, not firmware behavior; phrasing corrected per §16.10).

## 3. PR sequence (3 PRs, ordered by dependency)

| # | Branch | Scope | Gate / dependency |
|---|---|---|---|
| 1 | `docs/max-carrier-bringup` | T4/T5 carryover: docs + bench tooling — `BRINGUP_MAX_CARRIER.md` + OpenOCD cfg + Stage-1 conditional clock check | First — bench operator needs this to flash even the existing T5 firmware |
| 2 | `feat/h7-pty-loopback-harness` | New `tools/murata_host_loopback.py` + Python L072 mock that imports from `mh_wire.h` + `host-cc` POSIX transport + **`mh_stream.{h,c}` API-lift layer** (COBS unframer, seq allocator, `PING_REQ`/`CFG_GET_REQ`/`STATS_DUMP_REQ` helpers, request/URC dispatch with reject counters) + new CI job `h7-host-driver-loopback` | After PR#1; pure host-CC, no Arduino deps |
| 3 | `feat/h7-runtime-method-g-host` | Wire `murata_host` into `tractor_h7.ino` `setup()` + `loop()` **mutually exclusive** with legacy RadioLib via `#if/#else` behind `LIFETRAC_USE_METHOD_G_HOST=1`; add `mh_uart_arduino.cpp` transport; periodic `PING_REQ` + `STATS_DUMP_REQ` scheduler; bench-visibility serial logger; new `check-mh-runtime-off-symbols` symbol-set CI guard | Last — depends on PR#2's `mh_stream` API + harness to prove driver behavior before hardware |

Rationale for this order: runbook first because **a board not on the bench can't validate anything**, harness next because **PTY proves correctness without burning hardware time**, runtime hookup last because **it's the lowest-confidence change and benefits most from the prior two**.

## 4. PR #1 — `docs/max-carrier-bringup`

### 4.1 Files

New:
- `LifeTrac-v25/DESIGN-CONTROLLER/firmware/murata_l072/BRINGUP_MAX_CARRIER.md`
- `LifeTrac-v25/DESIGN-CONTROLLER/firmware/murata_l072/openocd/stm32l0_swd.cfg`
- `LifeTrac-v25/DESIGN-CONTROLLER/firmware/murata_l072/openocd/stlink.cfg`

Modified: none (pure docs PR).

### 4.2 Runbook structure

Sections in `BRINGUP_MAX_CARRIER.md`:

1. **Hardware checklist** — Murata CMWX1ZZABZ-078 dev board (or MAX CARRIER), ST-Link V2 (or J-Link EDU), 3.3 V serial-USB cable for USART2 (PA2/PA3), 50 Ω attenuators (20–40 dB), spectrum analyzer or RTL-SDR for Stage 1 verification, oscilloscope or frequency counter for MCO check.
2. **SWD pinout** — table mapping Murata SiP pins to ST-Link V2 connector (VCC, GND, SWDIO, SWCLK, NRST). Explicit warning: do **not** power Murata from ST-Link 3.3 V if the carrier board is also externally powered.
3. **OpenOCD config files** — copy of `stm32l0_swd.cfg` and `stlink.cfg`, including `set CHIPNAME stm32l072cz` and FLASH bank definition.
4. **Flash recipes** (three):
   - **A. ST-Link via OpenOCD** — `openocd -f stlink.cfg -f stm32l0_swd.cfg -c "program build/firmware.elf verify reset exit"`.
   - **B. J-Link via JLinkExe** — script with `loadfile`, `r`, `g`.
   - **C. MAX CARRIER USB DFU** — `dfu-util -a 0 -s 0x08000000:leave -D firmware.bin` (bootloader entry sequence documented).
5. **Three-stage bench procedure** (corrected per §16.6/16.7 to match current L072 behavior):
   - **Stage 1 — Single-board boot + clock verification.** Power on, attach USART2 at 921600 8N1, expect `BOOT_URC` with `clock_source_id == 0` (`PLATFORM_CLOCK_SOURCE_HSE_OK`) and `reset_cause` decoded. `clock_source_id == 1` is `PLATFORM_CLOCK_SOURCE_HSI_FALLBACK` — **STOP and inspect TCXO**, do not proceed to RF stages. *Optional MCO frequency check (only valid if you flashed a debug L072 artifact that configures MCO):* SYSCLK÷16 ≈ 2.000 MHz ±100 Hz on the configured MCO pin. Production T5 firmware does not configure MCO; this step is therefore **not** a hard T6 acceptance gate.
   - **Stage 2 — Known-good RX from external TX.** Confirm `BOOT_URC.radio_ok == 1`. RX is already armed during L072 boot after `sx1276_init()` succeeds — no `RX_START` host command exists. Use a known LoRa transmitter (handheld_mkr at 100 mW, 30 dB attenuator pad) on the same channel/SF/BW. Expect `HOST_TYPE_RX_FRAME_URC` events with sane RSSI/SNR plus `radio_rx_ok` STATS counter increments.
   - **Stage 3 — Two-board RF round-trip.** Two Murata boards back-to-back, attenuated coax. Board A: H7 issues `HOST_TYPE_TX_FRAME_REQ` once per second, expects `HOST_TYPE_TX_DONE_URC`. Board B: expects `HOST_TYPE_RX_FRAME_URC` with the matching payload, then optionally H7-on-Board-B issues its own `TX_FRAME_REQ` to send a reply (echo lives in H7 bench code, not in L072 firmware). Expect ≥99% TX_DONE rate and ≥99% matching RX_FRAME rate over 5 minutes, with no `radio_tx_abort_airtime` increments. Do **not** report a host-`PING_REQ` round-trip rate as an over-air metric — host PING only proves the H7↔L072 UART path.
6. **Failure-mode index** — table mapping each `BOOT_URC` `reset_cause` value, each `FAULT_URC` code (incl. T5's `HOST_FAULT_CODE_CLOCK_HSE_FAILED=0x08`), and each common bench symptom (no boot, garbled UART, no RF, runaway airtime aborts) to a remediation step.
7. **Reference data capture** — instructions to save `bench-evidence/T6_bringup_<date>/` with serial logs (BOOT_URC + first 5 minutes of STATS), optional MCO screenshot if available, two-board RF round-trip CSV (TX_DONE/RX_FRAME timestamps and RSSI/SNR).

### 4.3 Acceptance

- File exists, links from `LifeTrac-v25/DESIGN-CONTROLLER/README.md` and `LifeTrac-v25/DESIGN-CONTROLLER/firmware/murata_l072/README.md`.
- All three OpenOCD/JLink/dfu-util command lines syntactically lint with their respective dry-run flags on the dev machine (no hardware required).
- Markdown lint passes existing repo conventions.

## 5. PR #2 — `feat/h7-pty-loopback-harness`

### 5.1 Files

New:
- `LifeTrac-v25/DESIGN-CONTROLLER/tools/murata_host_loopback.py` — main harness.
- `LifeTrac-v25/DESIGN-CONTROLLER/tools/murata_host_l072_mock.py` — Python implementation of the L072 wire side (COBS + CRC16 + frame builder; responds to `HOST_TYPE_PING_REQ` by echoing the same seq/flags/payload, to `HOST_TYPE_CFG_GET_REQ` with `HOST_TYPE_CFG_DATA_URC`, to `HOST_TYPE_STATS_DUMP_REQ` with `HOST_TYPE_STATS_URC`). Constants are imported from `mh_wire.h` via a generated `mh_wire_constants.py` (see §5.2.1) — no second manual type table.
- `LifeTrac-v25/DESIGN-CONTROLLER/tools/gen_mh_wire_py.py` — small generator script that parses `mh_wire.h` `#define` lines and emits `mh_wire_constants.py`. Run in CI before the loopback step; output is `.gitignore`d to keep the source-of-truth single.
- `LifeTrac-v25/DESIGN-CONTROLLER/firmware/tractor_h7/murata_host/mh_uart.h` — abstract transport interface.
- `LifeTrac-v25/DESIGN-CONTROLLER/firmware/tractor_h7/murata_host/mh_uart_posix.c` — host-CC POSIX implementation (open, read, write, close on a file descriptor).
- **`LifeTrac-v25/DESIGN-CONTROLLER/firmware/tractor_h7/murata_host/mh_stream.{h,c}` — API-lift layer (per §16.4):** COBS-delimited stream unframer over `mh_uart_read`, monotonic seq allocator, `mh_stream_send_ping_req()`, `mh_stream_send_cfg_get_req(key)`, `mh_stream_send_stats_dump_req()`, URC dispatch table with per-reason reject counters (CRC, length, version, unknown-type), and a single `mh_stream_poll(now_ms)` entry point. Both `loopback_driver` (PR#2) and `mh_runtime` (PR#3) sit on top of this layer — neither re-implements stream framing.
- `LifeTrac-v25/DESIGN-CONTROLLER/firmware/tractor_h7/bench/h7_host_proto/loopback_driver.c` — host-CC binary that opens a PTY slave passed via argv, drives `mh_stream` API, and exits non-zero on any mismatch.
- `LifeTrac-v25/DESIGN-CONTROLLER/firmware/tractor_h7/bench/h7_host_proto/Makefile` (or inline `cc` recipe — see §5.4) — host-CC build glue for `loopback_driver`, `mh_cobs_crc_unit`, `mh_stats_vectors`.

Modified:
- `.github/workflows/arduino-ci.yml` — add `h7-host-driver-loopback` job (needs `h7-host-driver-tests`).
- `LifeTrac-v25/DESIGN-CONTROLLER/ARDUINO_CI.md` — document the new job.
- `LifeTrac-v25/DESIGN-CONTROLLER/tools/requirements.txt` — stdlib only (`os`+`pty`+`select`+`subprocess`); no new pip deps.

### 5.2 Transport split discipline (from T5-plan §16.10)

`murata_host.c` already does **not** depend on Arduino. The split this PR cements:

```
mh_uart.h            — abstract: mh_uart_open(cfg), mh_uart_write(buf,len), mh_uart_read(buf,maxlen,timeout_ms), mh_uart_close()
mh_uart_arduino.cpp  — Arduino HardwareSerial wrapper (lands in PR#3)
mh_uart_posix.c      — host-CC POSIX wrapper using read/write on an fd (lands in PR#2)
```

Acceptance: building `loopback_driver.c` with `gcc` does **not** include `Arduino.h`. Building `tractor_h7.ino` with `arduino-cli` does **not** include `mh_uart_posix.c`.

#### 5.2.1 Mock-constants generator (per §16.2)

`gen_mh_wire_py.py` parses `mh_wire.h` for `#define HOST_TYPE_*`, `#define HOST_PROTOCOL_VER`, `#define HOST_WIRE_SCHEMA_VER`, STATS offset constants, and CFG_KEY constants and emits a `.py` module with the same names + values. The mock then does:

```python
from mh_wire_constants import (HOST_PROTOCOL_VER, HOST_TYPE_PING_REQ,
                               HOST_TYPE_CFG_GET_REQ, HOST_TYPE_CFG_DATA_URC,
                               HOST_TYPE_STATS_DUMP_REQ, HOST_TYPE_STATS_URC, ...)

def on_inner_frame(frame):
    if frame.ver != HOST_PROTOCOL_VER:
        stats['rejected_version'] += 1
        return                                # never dispatch on wrong ver
    ...
```

Rationale: prevents the silent-pass failure mode where a mock with a parallel name table speaks the wrong types and CI stays green while real L072 ignores the H7 driver.

### 5.3 Loopback harness behavior

`murata_host_loopback.py`:

1. Calls `os.openpty()` → returns `(master_fd, slave_fd)` and `slave_path` (`/dev/pts/N` on Linux, equivalent on macOS; Windows runs in WSL only — documented limitation per §15.3, with cross-platform TCP transport `mh_uart_tcp.c` filed as **T6-D7** for a future tranche).
2. Spawns `loopback_driver` (host-CC binary) with `<slave_path>` argv as subprocess.
3. Drives `master_fd` with `murata_host_l072_mock`:
   - Encodes `HOST_TYPE_STATS_URC` with both 64 B and 68 B payloads (alternating), sends framed to driver.
   - Receives `HOST_TYPE_PING_REQ` / `HOST_TYPE_CFG_GET_REQ` / `HOST_TYPE_STATS_DUMP_REQ` from driver, validates frame structure (COBS unwrap → CRC16 verify → inner-frame parse → `ver == HOST_PROTOCOL_VER`), responds with: echoed `PING_REQ` (same seq/flags/payload, per real L072 behavior); `HOST_TYPE_CFG_DATA_URC` for valid keys or `HOST_TYPE_CFG_OK_URC` with non-zero status for bad keys; `HOST_TYPE_STATS_URC`.
   - Validates that driver's emitted requests reach the mock with no corruption.
4. Asserts (per §16.9, expanded from v1.0):
   - At least N=20 `PING_REQ` round-trips (request + echo) complete inside 5 s.
   - Driver correctly decodes both 64 B and 68 B STATS without misalignment.
   - Driver rejects a deliberately-corrupted CRC frame (CRC reject counter increments, emits no spurious URC).
   - Driver rejects a payload-length mismatch frame (length reject counter increments).
   - Driver rejects a wrong-version frame (`ver != HOST_PROTOCOL_VER`; version reject counter increments).
   - **Partial-read robustness:** mock writes one byte at a time, then random chunk sizes (1–512 B), then two COBS frames in a single write — driver must produce identical URC sequence in all three modes.
   - **Partial-write robustness:** simulated short `write()` returns from the mock side cause driver to retry without dropping or duplicating bytes.
   - Per-API-call host wall-time budget: no `mh_stream_*` call exceeds 5 ms on the host (asserts the contract that maps to the M7 watchdog budget on Arduino).
5. Exits 0 on all green, non-zero with diff on any failure.

### 5.4 CI job

The existing CI workflow style compiles host-CC bench binaries inline with `cc` rather than via a per-directory Makefile (per §16.5). PR#2 follows the same style; a Makefile is added only as a developer convenience and is not the source of truth for CI.

```yaml
h7-host-driver-loopback:
  needs: h7-host-driver-tests
  runs-on: ubuntu-latest
  steps:
    - uses: actions/checkout@v4
    - run: sudo apt-get install -y gcc python3
    - name: Generate Python mock constants from mh_wire.h
      run: cd LifeTrac-v25/DESIGN-CONTROLLER && python3 tools/gen_mh_wire_py.py
             firmware/tractor_h7/murata_host/mh_wire.h
             tools/mh_wire_constants.py
    - name: Build loopback_driver (host-CC, no Arduino)
      run: cd LifeTrac-v25/DESIGN-CONTROLLER &&
             cc -std=gnu11 -Wall -Wextra -Werror
               -Ifirmware/tractor_h7/murata_host
               -o firmware/tractor_h7/bench/h7_host_proto/loopback_driver
               firmware/tractor_h7/bench/h7_host_proto/loopback_driver.c
               firmware/tractor_h7/murata_host/murata_host.c
               firmware/tractor_h7/murata_host/mh_crc16.c
               firmware/tractor_h7/murata_host/mh_cobs.c
               firmware/tractor_h7/murata_host/mh_stream.c
               firmware/tractor_h7/murata_host/mh_uart_posix.c
    - name: Run PTY loopback
      run: cd LifeTrac-v25/DESIGN-CONTROLLER &&
             python3 tools/murata_host_loopback.py
               --driver firmware/tractor_h7/bench/h7_host_proto/loopback_driver
               --iterations 50
```

PTY support on `ubuntu-latest` is stdlib (`os.openpty()` works out of the box); no extra packages.

### 5.5 Acceptance

- `python3 tools/murata_host_loopback.py` runs locally on a Linux/macOS dev box and prints `[PASS] loopback: 50 iterations, 0 errors`.
- New CI job is green; can be marked required protection on `main`.
- Building `loopback_driver` with `gcc` succeeds without `Arduino.h` (proves transport split).
- Existing `h7-host-driver-tests` job still green (no regression in unit tests).

## 6. PR #3 — `feat/h7-runtime-method-g-host`

### 6.1 Files

New:
- `LifeTrac-v25/DESIGN-CONTROLLER/firmware/tractor_h7/murata_host/mh_uart_arduino.cpp` — Arduino HardwareSerial implementation of `mh_uart.h`. Non-blocking only: `mh_uart_read()` uses `Serial.available()` + `Serial.read()` capped at 256 B/call; `mh_uart_write()` checks `Serial.availableForWrite()` and returns short-write counts rather than blocking (per §16.9 — same byte/time budget contract as the read path).
- `LifeTrac-v25/DESIGN-CONTROLLER/firmware/tractor_h7/murata_host/mh_runtime.h` + `mh_runtime.c` — thin glue: `mh_runtime_setup()`, `mh_runtime_poll(now)`, periodic `PING_REQ` (1 Hz) + `STATS_DUMP_REQ` (0.2 Hz) scheduler sitting on top of `mh_stream` from PR#2. URC dispatch fills a `mh_runtime_health_t` struct (see §6.2.1) and emits a periodic serial log line for bench visibility (per §16.8).

Modified:
- `LifeTrac-v25/DESIGN-CONTROLLER/firmware/tractor_h7/tractor_h7.ino`:
  - In `setup()` near line 1772 (after `Serial1.begin(921600)`, before `boot_trace_mark(9)` closes setup): **mutually exclusive** `#if/#else` block (per §16.3) — the `=1` arm calls `mh_runtime_setup()`; the `=0` arm calls the existing legacy RadioLib init (`radio.begin(...)`, `radio.startReceive()`, etc., whatever `setup()` already does for the radio path).
  - In `loop()` adjacent to `poll_radio()` (around line 1800): **mutually exclusive** `#if/#else` block — the `=1` arm calls `mh_runtime_poll(now)`; the `=0` arm calls `poll_radio()`, `poll_link_ladder()`, and `tx_pump(now)` exactly as today. The legacy radio path is **not** invoked when Method G is enabled.
  - Diff target ≤60 lines (raised from v1.0's ≤30 because the if/else split touches more sites than the v1.0 sidecar approach).
- `LifeTrac-v25/DESIGN-CONTROLLER/BUILD_CONFIG.md` — document `LIFETRAC_USE_METHOD_G_HOST` (default 0), `LIFETRAC_MH_SERIAL` (no default — bench operator MUST set this explicitly until the Max Carrier UART routing is confirmed against schematic, per §15.4 + §16.10), and the mutually-exclusive runtime contract.
- `LifeTrac-v25/DESIGN-CONTROLLER/ARCHITECTURE.md` — one-paragraph note on the optional Method G runtime path and its exclusivity with the legacy RadioLib path.

### 6.2 Runtime hookup contract

`mh_runtime_setup()`:
- Picks a HardwareSerial from build-time macro `LIFETRAC_MH_SERIAL` (no implicit default — must be set per §15.4/§16.10 once Max Carrier UART routing is schematic-verified; build fails if undefined).
- Calls `mh_uart_open()` at 921600 8N1 (matches L072 USART2).
- Allocates static `mh_stream_t` + parser state in `.bss` — no heap.
- Records first-call wallclock for periodic scheduler.

`mh_runtime_poll(uint32_t now)`:
- Calls `mh_stream_poll(now)` which drains `mh_uart_read()` non-blockingly (max 256 B per call).
- Stream layer feeds bytes through COBS unframer → `murata_host_parse_inner_frame` → version check → URC dispatch.
- Dispatches parsed URCs into a local `static mh_runtime_health_t` struct (see §6.2.1).
- Every 1000 ms, calls `mh_stream_send_ping_req()`. Every 5000 ms, calls `mh_stream_send_stats_dump_req()`.
- Exposes `mh_runtime_get_health(mh_runtime_health_t *out)` for future telemetry consumers (not wired into `shared_mem.h` this tranche per T6-D1; just compiles and is callable from serial-debug paths).

#### 6.2.1 Bench-visibility surface (per §16.8)

`mh_runtime_health_t` minimum fields exposed this tranche:

```c
typedef struct {
    /* Last BOOT_URC */
    uint8_t  boot_reset_cause;
    uint8_t  boot_clock_source_id;
    uint8_t  boot_radio_ok;
    uint32_t boot_seen_at_ms;
    /* Last FAULT_URC */
    uint8_t  fault_code;
    uint8_t  fault_sub;
    uint32_t fault_seen_at_ms;
    /* Last parsed STATS snapshot (raw bytes; consumers parse via mh_stats) */
    uint8_t  last_stats[68];
    uint8_t  last_stats_len;     /* 0, 64, or 68 */
    uint32_t last_stats_seen_at_ms;
    /* PING liveness */
    uint32_t ping_round_trips;
    uint32_t ping_timeouts;
    uint32_t ping_last_rtt_us;
    /* Stream reject counters */
    uint32_t reject_crc;
    uint32_t reject_length;
    uint32_t reject_version;
    uint32_t reject_unknown_type;
} mh_runtime_health_t;
```

At 1 Hz, `mh_runtime_poll()` emits a single-line `Serial.printf` summary of these fields when `LIFETRAC_MH_DEBUG_LOG=1`. Default off in production; on for the first bench session per §16.8.

### 6.3 Default-off discipline

With `LIFETRAC_USE_METHOD_G_HOST` undefined or `0`:
- The `#if/#else` arm selecting the legacy RadioLib path is the **only** code compiled into `setup()`/`loop()`.
- No `mh_*` or `murata_host_*` translation units are linked into the final ELF (their files are still in the source tree but no caller exists in the default-off arm).

### 6.4 CI guards for default-off (revised per §15.1 + §16.5)

Two guards, with the symbol-set guard being the strict one and the section-size guard being a smoke test:

**Primary: `check-mh-runtime-off-symbols` (new, mathematically rigorous)**
- Runs `arm-none-eabi-nm --defined-only tractor_h7.ino.elf | awk '{print $3}'` on the default-off build.
- `grep -E '^(mh_|murata_host_)'` → must be empty.
- Compiler/LTO-invariant: any new `mh_*` symbol means the default-off arm is accidentally linking Method G code.

**Secondary: `check-mh-runtime-off-noop` (smoke test, ≤32 B tolerance)**
- Compares `.text` + `.data` + `.rodata` section sizes vs PR merge-base.
- ≤32-byte string-pool drift tolerated.
- Documented as advisory rather than blocking until R3 (LTO drift) is confirmed quiet over multiple PRs. If section-size guard false-positives once, demote to advisory permanently and rely on symbol-set guard alone.

**CI build job dependency:** both guards require an `arduino-cli` H7 build job. The current CI does not have one as a blocking gate. PR#3 lands the symbol-set guard wired to whatever H7 build exists at merge time:
- If the H7 `arduino-cli` build is already a blocking gate by then → both guards run as required.
- If not → land the symbol-set guard as a **manual-review checklist item** in the PR template (per §16.5), wired to a non-blocking CI job, and promote to required once the H7 compile job is real. T6-D5 then becomes "promote `check-mh-runtime-off-symbols` to required" rather than "add the symbol-set guard".

This prevents accidental "always-on" mistakes during refactors and keeps the legacy radio path the source of truth until `=1` is intentional.

### 6.5 Acceptance

- `arduino-cli compile` succeeds with `LIFETRAC_USE_METHOD_G_HOST=0` (default) and resulting binary is size-equivalent to pre-T6.
- `arduino-cli compile -DLIFETRAC_USE_METHOD_G_HOST=1` succeeds; binary is larger by the expected `murata_host` + `mh_runtime` cost (target ≤4 kB ROM, ≤512 B RAM).
- Loopback harness from PR#2 still passes (no regression in `mh_runtime` because it only adds, doesn't modify, `murata_host`).
- Bench dry-run: with a USB-serial cable on `Serial2` connected to a Linux box running `murata_host_l072_mock.py` in PTY-server mode, the H7 board emits one PING/sec and one STATS_GET/5 sec for 5 minutes with zero CRC errors.

## 7. CI guards summary (T6 net-new)

| Guard | Type | Triggers fail when… |
|---|---|---|
| `h7-host-driver-loopback` | CI job | PTY round-trip count below threshold; any CRC/length/version reject; partial-read or partial-write divergence; per-call wall-time >5 ms |
| `check-mh-runtime-off-symbols` | CI step (primary, per §15.1 + §16.5) | Any `mh_*` or `murata_host_*` symbol present in default-off ELF |
| `check-mh-runtime-off-noop` | CI step (secondary smoke test) | Default-off `.text+.data+.rodata` grows more than 32 bytes; demotable to advisory if R3 fires |
| `gen_mh_wire_py.py` clean run | CI step (per §16.2) | `mh_wire.h` parse fails or generated `mh_wire_constants.py` is empty |
| `check_mh_wire_sync.py` extension | Existing T5 guard | New constants in `host_types.h` not mirrored to `mh_wire.h` (45-constant census per §16.10) |

T5 guards retained: `check-stats-offsets`, `check-lbt-owner`, `check-airtime-counter-owner`, `check-cfg-wire-owner`, `check-cfg-contract`, `check-opmode-owner`, `check-rx-counter-owner`, `check-tx-counter-owner`.

## 8. Risk register

| # | Risk | Likelihood | Impact | Mitigation |
|---|---|---|---|---|
| R1 | PTY behavior differs subtly between Linux GitHub Actions and macOS dev boxes | Med | Med | Use stdlib `os.openpty()` only; avoid `pty.spawn()`; pin Python ≥3.10 in CI; document Windows = WSL only |
| R2 | Adding `mh_runtime_poll()` to `loop()` exceeds 200 ms M4 watchdog if `mh_uart_read()` blocks | Low | High | `mh_uart_arduino.cpp` MUST use `Serial.available()` + `Serial.read()` only (non-blocking); cap bytes drained per call to 256; add unit assertion in `loopback_driver` that no API call exceeds 5 ms wall time on the host |
| R3 | Default-off section-size byte-compare is too strict, fails on benign LTO drift (per §15.1) | High | Low | **Symbol-set guard is now primary** — compiler/LTO-invariant `nm` filter for `mh_*`/`murata_host_*`. Section-size guard is secondary and demotable to advisory if it false-positives even once. |
| R4 | Bench operator follows runbook on a board with HSI fallback active and doesn't notice | Med | High | Stage-1 `clock_source_id == 0` check is the mandatory STOP gate (per §16.6); failure-mode index points `clock_source_id == 1` directly at TCXO inspection. MCO check is conditional/optional, not a blocking gate. |
| R5 | `mh_runtime` competes with another peripheral on the chosen HardwareSerial (per §15.4 + §16.10) | Med | High | `LIFETRAC_MH_SERIAL` has **no implicit default** — build fails if undefined. **Per ABX00043 schematic trace (v1.2):** exclude `Serial1` (X8↔H747 link, busy in `tractor_h7.ino` line 1788) and `Serial3` (on-carrier Cellular Modem, sheet 5). Bench candidates = `Serial2`/`Serial4`/`Serial5`; final selection requires sheet-1 zoom + Arduino mbed `variants/PORTENTA_H7_M7/variant.cpp` cross-reference. R5 retired only when schematic page + chosen `SerialN` captured in `bench-evidence/T6_bringup_<date>/serial_routing.md`. |
| R6 | DFU recipe in runbook fails if MAX CARRIER bootloader version differs | Med | Low | Document the bootloader version probe (`dfu-util -l`); add fallback to ST-Link path |
| R7 | Loopback harness Python mock drifts from real L072 firmware behavior | Med | Med | `murata_host_l072_mock.py` is documented as **frame-shape only**, not behavior-equivalent; Stage 2/3 bench proves real behavior; mock is for wire-protocol regression only. Type/version constants are imported from `mh_wire.h` via generator (per §16.2) so name drift cannot produce a silent-pass. |
| R8 | Method G runtime accidentally runs alongside legacy RadioLib path (per §16.3) | Med | High | Hookup is `#if/#else` mutually-exclusive on `LIFETRAC_USE_METHOD_G_HOST`; symbol-set guard catches accidental linking of Method G symbols in the `=0` build. |
| R9 | API drift between `loopback_driver` and `mh_runtime` if each owns its own stream pump (per §16.4) | High | High | Both sit on top of shared `mh_stream.{h,c}` from PR#2; neither re-implements COBS unframing, seq allocation, or dispatch. Stream layer is exercised by the loopback harness before PR#3 even branches. |
| R10 | OpenOCD chip-erase on the L072 also resets STSAFE secure element (sheet 6: pin 34 `MCU_NRST/STSAFE_RST` is shared) | Med | Med | Runbook §2 + §6 add explicit STSAFE warning. If/when Method G firmware ever uses STSAFE provisioned material, document a chip-erase recovery procedure. T6 firmware does not use STSAFE so impact is informational only. |
| R11 | Chosen `SERIALn` HD-connector pins multiplex with another H7 peripheral via mbed-core defaults (sheet 2 `SHARED PINS on H7` note) | Med | High | PR#3 acceptance adds a one-time mbed-core pinmap audit: enumerate all HD-pin consumers in `variants/PORTENTA_H7_M7/PeripheralPins.c`, confirm chosen `SerialN` HD pins are not also claimed by `Wire`, `SPI`, ADC, or PWM in any default-on path. Capture audit notes in `bench-evidence/T6_bringup_<date>/pinmap_audit.md`. |
| R12 | CN2 SWD debug header on Max Carrier is DNP (sheet 6 X marks) — operator may attempt to flash and find no SWD pads soldered | High | Low | Runbook §2 + §4 explicitly state CN2 is DNP and route operators to either (a) solder the 2x5 1.27 mm header per the Max Carrier silkscreen, or (b) use the BOOT0/DFU path (R19 wired). |

## 9. Deferred-tech-debt log

| ID | Item | Target tranche |
|---|---|---|
| T6-D1 | Wire `mh_runtime` STATS into actual H7 telemetry / shared_mem.h consumers | T7 (after FHSS) |
| T6-D2 | Move PING/STATS_GET cadence from hard-coded constants to CFG keys | T7 |
| T6-D3 | TX-power adapter on H7 (`tractor_h7/src/murata_host/tx_power_adapter.{h,cpp}`) per T5-plan §8.1 | T7 |
| T6-D4 | FHSS deterministic-bench PR (T5-plan v1.2 PR#6) with sequence-derived epoch sync | T7 |
| T6-D5 | Promote `check-mh-runtime-off-symbols` from advisory to required CI gate once H7 `arduino-cli` build is a blocking job (per §16.5; symbol-set guard itself moved into PR#3 per §15.1) | When H7 compile job is required |
| T6-D6 | Real bench-evidence capture from first hardware bring-up under `bench-evidence/T6_bringup_<date>/` | After first physical session |
| T6-D7 | Cross-platform mock transport `mh_uart_tcp.c` so Windows-native developers can run loopback without WSL (per §15.3) | T7 or T8 |
| T6-D8 | MCO configuration in a debug L072 firmware artifact so the runbook's Stage-1 MCO check becomes a routine gate rather than conditional (per §16.6) | When a bench operator needs the higher-confidence clock check |

## 10. Acceptance checklist

- [ ] `BRINGUP_MAX_CARRIER.md` lands with all 7 sections; OpenOCD `.cfg` files lint-clean; Stage 1 expects `clock_source_id == 0`; Stage 2/3 use `RX_FRAME_URC` / `TX_FRAME_REQ` / `TX_DONE_URC` (no `RX_START`, no PING_ACK over-air metric).
- [ ] `tools/gen_mh_wire_py.py` parses `mh_wire.h` cleanly; `mh_wire_constants.py` regenerates in CI.
- [ ] `tools/murata_host_loopback.py` runs locally and in CI, ≥50 round-trip iterations, 0 errors, including partial-read/partial-write/wrong-version cases.
- [ ] `loopback_driver` compiles with inline `cc` (no Makefile dependency in CI) and **without** `Arduino.h`.
- [ ] `mh_uart_arduino.cpp` compiles with `arduino-cli` and **without** POSIX headers.
- [ ] `mh_stream.{h,c}` is the single COBS unframer + dispatch + seq allocator used by both `loopback_driver` and `mh_runtime`.
- [ ] `tractor_h7.ino` diff ≤60 lines, mutually-exclusive `#if/#else` around `setup()` radio init AND `loop()` `poll_radio`/`poll_link_ladder`/`tx_pump` sites.
- [ ] `check-mh-runtime-off-symbols` passes — no `mh_*` / `murata_host_*` symbols in default-off ELF.
- [ ] `check-mh-runtime-off-noop` passes (advisory or required per the §6.4 promotion rule).
- [ ] `check_mh_wire_sync.py` continues to pass (45 constants unchanged).
- [ ] All T5 CI guards remain green.
- [ ] `HOST_PROTOCOL_VER=1`, `HOST_WIRE_SCHEMA_VER=1` unchanged.
- [ ] No new CFG keys.
- [ ] No new STATS fields.
- [ ] `BUILD_CONFIG.md` documents `LIFETRAC_USE_METHOD_G_HOST` AND requires explicit `LIFETRAC_MH_SERIAL` (no implicit default).
- [ ] `ARCHITECTURE.md` references the optional Method G runtime path AND its mutual exclusivity with the legacy RadioLib path.
- [ ] `mh_runtime_health_t` exposes BOOT/FAULT/STATS/ping/reject-counter fields per §6.2.1 with bench-visibility serial logging behind `LIFETRAC_MH_DEBUG_LOG=1`.
- [ ] T7 plan can begin from a clean state with FHSS as PR#1.

## 11. Out-of-scope confirmations

- T6 does **not** change Crypto Profile A boundaries.
- T6 does **not** modify any L072 firmware C/header/linker/Makefile behavior (PR#1 *does* add OpenOCD `.cfg` files under `firmware/murata_l072/openocd/` but those are bench tooling artifacts, not firmware behavior — phrasing corrected per §16.10).
- T6 does **not** introduce hardware-specific timing assumptions beyond what Stage 1 of the runbook validates (and Stage 1's MCO check is documented as conditional, not gating, per §16.6).
- T6 does **not** require any new third-party Python dependency — stdlib only (`os`+`pty`+`select`+`subprocess`).
- T6 does **not** invent new L072 host commands. Every wire-side type used by mock, runbook, and runtime exists in the current `host_types.h`.

## 12. Rollback plan

- PR#3 is reversible by setting `LIFETRAC_USE_METHOD_G_HOST=0` (the default); no `git revert` needed for runtime safety.
- PR#2 is purely additive (new files + new CI job); revert by deleting files + removing job stanza.
- PR#1 is docs-only; revert is `git revert` of the doc commit.
- Combined revert path: revert PR#3 → PR#2 → PR#1 leaves repo at exact post-T5-impl-v1.1 state.

## 13. Sequencing within the tranche

```
PR#1 docs/max-carrier-bringup
   │  (lands first; unblocks bench operator on existing T5 firmware)
   ▼
PR#2 feat/h7-pty-loopback-harness
   │  (proves murata_host correctness over PTY before any hardware coupling)
   ▼
PR#3 feat/h7-runtime-method-g-host
   │  (final hookup; safe-by-default with compile flag)
   ▼
T6 closed; T7 opens with FHSS PR#1 (sequence-derived epoch sync)
```

## 14. One-line summary

> **Land three PRs that finish what T5 deferred, with v1.1 amendments folded in from Gemini §15 and Copilot §16 — PR#1 ships `BRINGUP_MAX_CARRIER.md` (docs + bench tooling, not pure docs) with SWD pinout, three flash recipes (ST-Link/J-Link/MAX CARRIER DFU), and a three-stage bench procedure aligned to **current L072 behavior** (Stage 1 expects `BOOT_URC.clock_source_id == 0` for HSE OK, `==1` is HSI fallback STOP gate; MCO frequency check is conditional/optional, deferred to T6-D8; Stage 2 confirms `BOOT_URC.radio_ok == 1` and observes `HOST_TYPE_RX_FRAME_URC` because RX is auto-armed at boot — there is no `RX_START` host command; Stage 3 measures `TX_FRAME_REQ`/`TX_DONE_URC`/`RX_FRAME_URC` rates over RF, never host-`PING_REQ` rate as an over-air metric); PR#2 adds `tools/murata_host_loopback.py` + `murata_host_l072_mock.py` (constants imported from `mh_wire.h` via new `tools/gen_mh_wire_py.py`, frames rejected if `ver != HOST_PROTOCOL_VER`) + `mh_uart_posix.c` host-CC transport + new shared **`mh_stream.{h,c}` API-lift layer** (COBS unframer + seq allocator + `PING_REQ`/`CFG_GET_REQ`/`STATS_DUMP_REQ` helpers + dispatch with CRC/length/version/unknown-type reject counters) + `loopback_driver` host-CC binary, all wired into a new `h7-host-driver-loopback` CI job (built inline with `cc`, no Makefile dependency) that round-trips real Method G ABI types (echoed `PING_REQ`, `CFG_DATA_URC`, `STATS_URC` 64 B and 68 B) over a stdlib `os.openpty()` pair with partial-read/partial-write/wrong-version coverage and a 5 ms per-call wall-time budget, without any Arduino dependency; PR#3 wires `mh_runtime` (sitting on `mh_stream`) into `tractor_h7.ino` `setup()` (near line 1772) and `loop()` (next to `poll_radio()` at line 1800) **mutually exclusive** with the legacy RadioLib path via `#if/#else` behind `LIFETRAC_USE_METHOD_G_HOST=1` (default 0; `LIFETRAC_MH_SERIAL` has no implicit default per §15.4/§16.10 — bench operator must schematic-verify the Max Carrier UART), with a non-blocking Arduino HardwareSerial transport in `mh_uart_arduino.cpp`, a 1 Hz `PING_REQ` + 0.2 Hz `STATS_DUMP_REQ` scheduler, an `mh_runtime_health_t` bench-visibility surface (BOOT/FAULT/STATS/ping/reject counters) logged at 1 Hz behind `LIFETRAC_MH_DEBUG_LOG=1`, and a new **primary** `check-mh-runtime-off-symbols` CI guard (compiler/LTO-invariant `nm` filter for `mh_*`/`murata_host_*` symbols in the default-off ELF, pulled into PR#3 per §15.1) plus a secondary advisory `check-mh-runtime-off-noop` section-size smoke test; all T5 guards remain green, no wire-schema bump, no new CFG keys, no new STATS fields, no L072 firmware behavior changes — eight deferred items filed (T6-D1 STATS-into-shared_mem → T7, T6-D2 cadence-as-CFG → T7, T6-D3 H7 `TxPowerAdapter` → T7, T6-D4 FHSS deterministic-bench → T7, T6-D5 promote symbol-set guard to required gate once H7 `arduino-cli` job is required, T6-D6 real bench-evidence capture, T6-D7 cross-platform mock transport `mh_uart_tcp.c` per §15.3, T6-D8 debug-L072 MCO configuration so Stage-1 MCO check becomes routine per §16.6).**

## 15. Review & System Architecture Analysis (Gemini 3.1 Pro)

**Review Date:** 2026-05-05
**Reviewer:** Gemini 3.1 Pro (Preview)

This is a well-scoped and highly defensive tranche. Focusing entirely on testing harnesses, runbooks, and runtime isolation before pushing new wireless features (like FHSS) into the mix is excellent engineering practice. 

Here is my architectural analysis and risk assessment of the Tranche 6 plan:

### 1. The LTO and Binary Diff Fragility (R3 / T6-D5)
The `check-mh-runtime-off-noop` CI guard is a very strong conceptual safeguard. However, comparing raw `.text`/`.data`/`.rodata` sizes from `arduino-cli` is extremely brittle. GCC's Link-Time Optimization (LTO), which the Arduino core often employs, can shift section sizes by a few bytes unpredictably based on build environments or minor standard library differences between the CI runner and a local machine. 
*   **Assessment:** The ≤32-byte tolerance might absorb this, but false positives are highly likely. You should strongly consider pulling T6-D5 (the `arm-none-eabi-nm` symbol-set diff) into PR#3 directly rather than waiting for it to fail. Asserting that no *new* `mh_` symbols exist in the final binary is mathematically rigorous and compiler-invariant.

### 2. MCO HSE Verification (Stage 1 Bring-up)
*   **Assessment:** Outstanding step. The STM32L072 will silently fall back to the internal 16 MHz RC oscillator (HSI16) if the Murata TCXO fails to start. The firmware will "boot" perfectly, but the RF frequency will be off by tens of kilohertz, causing all LoRa packets to fail to decode at the receiver. Probing MCO (SysClk/16 = 2.000 MHz) is the exact hardware ground-truth needed to prove the TCXO is driving the PLL.

### 3. PTY Harness OS Limitations (R1)
*   **Assessment:** Standard Python `os.openpty()` does not exist natively on Windows. Many Arduino developers compile natively on Windows rather than inside WSL. While documenting that Windows users must use WSL is an acceptable stopgap (R1), it forms a friction point for contributors. For a future tranche, consider adding a cross-platform TCP-socket transport implementation (`mh_uart_tcp.c`) alongside the POSIX file-descriptor implementation to make the mock universally runnable.

### 4. Serial2 Contention (R5)
*   **Assessment:** The plan correctly notes that the legacy SPI `RFM` radio driver doesn't use `Serial2`. However, you must verify against the *complete* Portenta H7 Max Carrier schematic to ensure `Serial2` isn't assigned to GPS, RS485, or another peripheral. If `Serial2` is purely broken out to the Murata SiP header, your `LIFETRAC_MH_SERIAL` default is safe.

### 5. Architectural Clearance
You are clear to proceed with Tranche 6 execution. The state-machine isolation and compile-flag toggles will keep the `main` branch completely stable while introducing the Method G bridge.

## 16. Review & Implementation Analysis (GitHub Copilot v1.0)

**Review Date:** 2026-05-05
**Reviewer:** GitHub Copilot
**Scope:** T6 plan checked against the current H7 and L072 tree, especially `firmware/tractor_h7/tractor_h7.ino`, `firmware/tractor_h7/murata_host/*`, `firmware/tractor_h7/bench/h7_host_proto/*`, `tools/check_mh_wire_sync.py`, `firmware/murata_l072/include/host_types.h`, `host_cmd.c`, `platform.c`, and the active CI workflow.

### 16.1 Executive verdict

Proceed with T6, but tighten the plan before implementation. The tranche is correctly scoped: no new RF features, no schema bump, no L072 behavior changes, and the right emphasis on runbook + loopback + default-off H7 runtime. That is the right bridge between the T5 wire-layer work and T7 FHSS/link-adaptation work.

The main corrections are practical rather than strategic:

1. The loopback mock and runbook must use the actual Method G ABI names and behaviors.
2. The Method G runtime path must be mutually exclusive with the legacy RadioLib path when enabled.
3. The CI/build steps for `loopback_driver` and `check-mh-runtime-off-noop` need real build glue in the repo.
4. The runbook's clock and bench-stage details need to match the current L072 implementation.

### 16.2 Blocker - The loopback mock must match the current wire ABI exactly

The plan uses several intuitive names that are not the current wire contract:

| Plan wording | Current source-of-truth ABI |
|---|---|
| `PING_ACK` | L072 echoes `HOST_TYPE_PING_REQ` with the same seq/flags/payload |
| `STATS_GET` | `HOST_TYPE_STATS_DUMP_REQ` |
| `CFG_VALUE_URC` | `HOST_TYPE_CFG_DATA_URC`; bad reads return `HOST_TYPE_CFG_OK_URC` with status |
| `TX_REQ` | `HOST_TYPE_TX_FRAME_REQ` |
| `RX_DONE_URC` | `HOST_TYPE_RX_FRAME_URC` |
| `RX_START` | No current host command; L072 arms RX during boot when radio init succeeds |

PR#2 should make the Python mock import or generate its constants from `mh_wire.h` rather than carrying a second manual type table. It should also assert `frame.ver == HOST_PROTOCOL_VER` before dispatching, because `murata_host_parse_inner_frame()` currently parses the version byte but does not reject a wrong version by itself.

This is a blocker because a mock that speaks the wrong names can pass CI while the real L072 rejects or ignores the H7 driver's traffic.

### 16.3 Blocker - Method G runtime must not run beside legacy RadioLib

`tractor_h7.ino` currently initializes the legacy RadioLib path in `setup()` and services `poll_radio()`, `poll_link_ladder()`, and `tx_pump(now)` in `loop()`. The T6 plan says PR#3 should add `mh_runtime_poll(now)` adjacent to `poll_radio()`, but Method G is not an additional radio; it is the replacement host path for the Murata L072 custom firmware.

When `LIFETRAC_USE_METHOD_G_HOST=1`, PR#3 should either:

- compile out the legacy RadioLib init/poll/TX path and use only `mh_runtime`, or
- add an explicit bench-only mode that documents the legacy path is still active and does not claim Method G runtime isolation.

The preferred launch shape is an either/or compile-time split:

```c
#if defined(LIFETRAC_USE_METHOD_G_HOST) && LIFETRAC_USE_METHOD_G_HOST
   mh_runtime_setup();
#else
   radio.begin(...);
   radio.startReceive();
#endif
```

and similarly in `loop()` around `poll_radio()`, `poll_link_ladder()`, and `tx_pump(now)`. Default-off remains the safety path, but enabled Method G should not leave two radio stacks competing for the same product role.

### 16.4 High - The H7 driver still needs an API-lift layer

The current `murata_host` library is a low-level inner-frame builder/parser plus COBS/CRC/STATS helpers. It does not yet provide request helpers, a COBS-delimited stream unframer, seq allocation, request/URC dispatch, or a runtime state machine.

PR#2 and PR#3 should make that explicit:

- PR#2 adds the host-native stream driver: COBS frame ingest, CRC/length/version rejection counters, seq matching, and helpers for `PING_REQ`, `CFG_GET_REQ`, and `STATS_DUMP_REQ`.
- PR#3 adds the Arduino transport and scheduler on top of that stream driver.

Do not hide this inside `loopback_driver.c` only. If the loopback binary owns the first real request/URC pump, PR#3 will reimplement it for Arduino and the two paths can drift immediately.

### 16.5 High - CI build glue is missing today

The proposed CI job runs:

```bash
make -C firmware/tractor_h7/bench/h7_host_proto loopback_driver
```

but there is currently no `Makefile` in `firmware/tractor_h7/bench/h7_host_proto/`. T6 should either add that Makefile in PR#2 or keep the workflow's existing style and compile the loopback binary inline with `cc -std=gnu11 -Wall -Wextra -Werror -Imurata_host ...`.

Similarly, `check-mh-runtime-off-noop` depends on an automated `tractor_h7.ino` Arduino build, but the current CI docs still say the Tractor H747 M7/M4 FQBN needs local verification before becoming a blocking compile gate. PR#3 acceptance should include one of these decisions:

- first land reliable H7 `arduino-cli` build glue in CI, then enforce the default-off guard, or
- mark `check-mh-runtime-off-noop` as a local/manual review gate until the H7 compile job is real.

I agree with Gemini §15.1: pull the symbol-set guard into PR#3 now. A section-size comparison can remain a smoke test, but the stronger gate is "no linked `mh_`/`murata_host_` symbols when `LIFETRAC_USE_METHOD_G_HOST=0`."

### 16.6 High - Runbook clock semantics need correction

Current L072 constants are:

```c
#define PLATFORM_CLOCK_SOURCE_HSE_OK        0U
#define PLATFORM_CLOCK_SOURCE_HSI_FALLBACK  1U
#define PLATFORM_CLOCK_SOURCE_MSI_FALLBACK  2U
```

So Stage 1 should expect `BOOT_URC.clock_source_id == 0` for HSE OK, not `1`. A `clock_source_id` of `1` is the HSI fallback path and should stop FHSS/bench RF validation.

The plan also says MCO will output `SYSCLK / 16 = 2.000 MHz`, but the current L072 tree does not appear to configure MCO. Because T6 explicitly says it will not modify L072 firmware behavior, the runbook must phrase MCO as either:

- a check available only when using a T5 artifact that already exports MCO, or
- a follow-up debug-firmware requirement outside T6.

Do not make MCO verification a hard T6 acceptance item unless the supporting L072 firmware change already exists in the exact artifact being flashed.

### 16.7 High - The bring-up stages should test real current behavior

Stage 2 should not say "Issue `RX_START`" because no such host command exists. Current L072 firmware arms continuous RX during boot after `sx1276_init()` succeeds, then emits `RX_FRAME_URC` on valid packets. The runbook should instead instruct the operator to confirm `BOOT_URC.radio_ok=1`, then transmit from a known-good peer and observe `RX_FRAME_URC` plus STATS deltas.

Stage 3 should not report a `PING_ACK` rate as an over-air ping-pong result. Host `PING_REQ` only proves the H7-to-L072 UART path. A real two-board RF stage should require:

- Board A sends `TX_FRAME_REQ` and receives `TX_DONE_URC`.
- Board B receives `RX_FRAME_URC` with the expected payload.
- Optional echo is implemented by the H7 bench runtime on Board B sending its own `TX_FRAME_REQ` back, not by an implied L072 firmware echo path.

That distinction matters because T6's purpose is to connect the H7 runtime to the real L072 modem boundary, not to invent an unplanned L072 radio test command.

### 16.8 Medium - Runtime visibility should not be entirely deferred

T6-D1 defers wiring Method G STATS into the real H7 telemetry/shared-memory consumers, which is acceptable. Still, PR#3 should expose a minimal bench-visible health surface in this tranche:

- last `BOOT_URC` fields,
- last `FAULT_URC` code/sub,
- last parsed STATS snapshot,
- ping RTT / timeout count,
- parser CRC/length/version reject counters.

Serial logging is enough for T6 if shared-memory wiring waits until T7. Without this, an enabled Method G runtime can be "working" while the bench operator has no direct signal beyond absence of crashes.

### 16.9 Medium - Transport contract needs partial-read/write tests

The POSIX PTY harness should not only send full frames in one write. UARTs and PTYs can split frames arbitrarily. Add loopback cases for:

- one byte at a time,
- random chunk sizes,
- two COBS frames in one read,
- partial writes from the host transport,
- corrupted CRC and bad payload length with no spurious URC dispatch.

For the Arduino transport, define whether `mh_uart_write()` may block and how it handles a full TX buffer. `mh_runtime_poll()` is on the M7 watchdog path, so any blocking serial write should have a byte/time budget just like reads.

### 16.10 Medium - Naming and scope cleanups

Small document fixes to apply while implementing:

- `check_mh_wire_sync.py` currently checks 45 constants, not 44.
- T6 says "zero edits under `firmware/murata_l072/`" but PR#1 adds docs/config files under that directory. Rephrase to "no L072 C/header/linker/Makefile behavior changes."
- PR#1 is not purely docs if it adds OpenOCD config files; call it docs + bench tooling.
- Reference evidence should use `bench-evidence/T6_bringup_<date>/`, not `T5_bringup_<date>`.
- Keep `LIFETRAC_MH_SERIAL` explicit for bench builds until the Max Carrier UART routing is confirmed. `Serial1` is already the X8 link; `Serial2` may be correct, but the runbook should prove it from the carrier schematic rather than relying on an implicit default.

### 16.11 Bottom line

T6 is the correct next tranche and should stay narrow. Before execution, revise the plan so the mock speaks the exact current ABI, Method G is an exclusive runtime path when enabled, CI has real build glue for both host loopback and default-off enforcement, and the bring-up runbook reflects current L072 behavior: HSE OK is `clock_source_id=0`, RX is already armed after boot, and RF ping-pong is `TX_FRAME_REQ`/`TX_DONE_URC`/`RX_FRAME_URC`, not host `PING_REQ`.
