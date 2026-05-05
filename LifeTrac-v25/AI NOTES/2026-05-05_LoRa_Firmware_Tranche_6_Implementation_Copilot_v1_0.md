# LoRa Firmware Tranche 6 Implementation (Copilot v1.0)

Date: 2026-05-05
Scope: Implement T6 carryover/runtime tasks after plan-review pass.

## Implemented

### 1) Bring-up runbook and OpenOCD configs

Added:
- `DESIGN-CONTROLLER/firmware/murata_l072/BRINGUP_MAX_CARRIER.md`
- `DESIGN-CONTROLLER/firmware/murata_l072/openocd/stlink.cfg`
- `DESIGN-CONTROLLER/firmware/murata_l072/openocd/stm32l0_swd.cfg`

Updated links in:
- `DESIGN-CONTROLLER/README.md`
- `DESIGN-CONTROLLER/firmware/murata_l072/README.md`

Runbook alignment details:
- Stage 1 checks `clock_source_id == 0` as HSE OK.
- Stage 2 uses real behavior (`RX` auto-armed after boot; no `RX_START` command).
- Stage 3 validates `TX_FRAME_REQ` -> `TX_DONE_URC` and peer `RX_FRAME_URC`.

### 2) H7 stream layer + transport abstraction + PTY harness

Added H7 host stream and transport files under `firmware/tractor_h7/murata_host/`:
- `mh_uart.h`, `mh_uart.c`
- `mh_uart_posix.c`
- `mh_stream.h`, `mh_stream.c`

Key behavior implemented:
- COBS stream unframing over delimiter `0x00`
- Monotonic seq allocator
- Request helpers for `PING_REQ`, `CFG_GET_REQ`, `STATS_DUMP_REQ`
- Reject counters for CRC/length/version/unknown-type
- Write retry loop with short write budget

Added host loopback artifacts:
- `firmware/tractor_h7/bench/h7_host_proto/loopback_driver.c`
- `tools/gen_mh_wire_py.py`
- `tools/murata_host_l072_mock.py`
- `tools/murata_host_loopback.py`

Notes:
- Loopback script now handles non-POSIX platforms gracefully (prints a clear error and exits).
- Generated `tools/mh_wire_constants.py` is ignored via `.gitignore`.

### 3) Method G runtime wiring in tractor_h7

Added runtime/Arduino transport:
- `firmware/tractor_h7/murata_host/mh_uart_arduino.cpp`
- `firmware/tractor_h7/murata_host/mh_runtime.h`
- `firmware/tractor_h7/murata_host/mh_runtime.c`

Updated `firmware/tractor_h7/tractor_h7.ino`:
- Added compile-time gate for Method G:
  - `LIFETRAC_USE_METHOD_G_HOST=1`
  - required `LIFETRAC_MH_SERIAL`
- `setup()` and `loop()` now use compile-time-exclusive `#if/#else` path:
  - Method G enabled: `mh_runtime_begin()` / `mh_runtime_loop()`
  - Method G disabled: legacy path unchanged

### 4) CI and documentation updates

Updated workflow:
- Added `h7-host-driver-loopback` job:
  - builds `loopback_driver`
  - runs PTY harness
- Added `tractor-h7-method-g-guards` job:
  - compiles default-off build and checks for leaked `mh_runtime_*` / `mh_uart_arduino_*` symbols
  - compiles Method G enabled build and asserts runtime symbols are present

Updated docs:
- `DESIGN-CONTROLLER/ARDUINO_CI.md` (new loopback + runtime guard checks)
- `DESIGN-CONTROLLER/BUILD_CONFIG.md` (Method G flags and exclusivity)
- `DESIGN-CONTROLLER/ARCHITECTURE.md` (Tractor M7 integration mode note)

### 5) Wire-sync census update

Added mirror constant to H7 wire header and sync check:
- `HOST_FAULT_CODE_CLOCK_HSE_FAILED`

`check_mh_wire_sync.py` now validates 45 constants (up from 44).

## Local validation run

Executed and passed:
- `python tools/check_mh_wire_sync.py` -> `[PASS] mh_wire sync: 45 constants match`
- `python -m py_compile` on updated/new tools scripts
- `gcc` build + run:
  - `mh_cobs_crc_unit` -> pass
  - `mh_stats_vectors` -> pass
- `gcc -c` object compile check for:
  - `mh_uart.c`, `mh_stream.c`, `mh_stats.c`, `murata_host.c`

Platform limitation observed locally (Windows shell):
- `mh_uart_posix.c` host build requires POSIX headers (`termios.h`), so full PTY driver/harness execution is not runnable natively in this Windows environment.
- This path is expected to run in Linux CI (`ubuntu` jobs).

## Deferred / follow-up

- Full PTY runtime exercise remains CI/Linux-validated due local platform limits.
- Bench hardware evidence capture remains pending first physical session under:
  - `DESIGN-CONTROLLER/bench-evidence/T6_bringup_<date>/`
