# LoRa Firmware Tranche 5 Implementation (Copilot v1.1)

Date: 2026-05-05
Purpose: Continuation pass after v1.0, focused on H7-side host-wire compatibility and tests.

## What was added in this continuation

### 1) H7 host-wire module (tracked, not CI-staged scratch)
Added a new tracked directory under `firmware/tractor_h7/murata_host/`:

- `mh_wire.h`
  - Mirrored wire constants from L072 `host_types.h`.
  - Includes STATS offsets and explicit legacy compatibility constants:
    - legacy payload length 64
    - legacy `radio_state` offset 60
    - additive payload length 68 with `radio_tx_abort_airtime` + shifted `radio_state`.

- `mh_crc16.{h,c}`
  - CRC16-CCITT implementation (`poly=0x1021`, init `0xFFFF`).

- `mh_cobs.{h,c}`
  - Allocation-free COBS encode/decode helpers.

- `murata_host.{h,c}`
  - Inner-frame builder/parser for Method G host frames.
  - Parses `ver/type/flags/seq/payload_len/payload/crc` and validates CRC.

- `mh_stats.{h,c}`
  - STATS payload parser tolerant to both:
    - legacy 64-byte payload (state at offset 60)
    - additive 68-byte payload (airtime-abort at 60, state at 64)

### 2) Host-run vector tests for compatibility lock
Added `firmware/tractor_h7/bench/h7_host_proto/` tests:

- `mh_cobs_crc_unit.c`
  - CRC reference check (`"123456789" -> 0x29B1`)
  - COBS encode/decode round-trip vectors.

- `mh_stats_vectors.c`
  - STATS 64-byte legacy vector parse.
  - STATS 68-byte additive vector parse.
  - short-payload rejection.
  - full inner-frame build/parse + STATS decode for both lengths.

### 3) Wire drift guard script
Added `tools/check_mh_wire_sync.py`:

- Compares key constants between:
  - `firmware/murata_l072/include/host_types.h`
  - `firmware/tractor_h7/murata_host/mh_wire.h`
- Fails CI on mismatch to prevent silent host/firmware drift.

### 4) CI integration
Updated `.github/workflows/arduino-ci.yml` with new job:

- `h7-host-driver-tests` (needs `firmware-l072-cross-compile`)
  - runs `check_mh_wire_sync.py`
  - builds/runs `mh_cobs_crc_unit`
  - builds/runs `mh_stats_vectors`

Updated `ARDUINO_CI.md` to document the new job and recommend it as a required check.

## Local validation in this continuation

Executed and passed:

- `python tools/check_mh_wire_sync.py`
- `gcc ... mh_cobs_crc_unit.c ... && mh_cobs_crc_unit.exe`
- `gcc ... mh_stats_vectors.c ... && mh_stats_vectors.exe`

Observed outputs:

- `[PASS] mh_wire sync: 44 constants match`
- `[PASS] mh_cobs_crc_unit: 4 checks`
- `[PASS] mh_stats_vectors: 4 vectors`

## Remaining follow-ups (not done in this pass)

- Integrate `murata_host` into active `tractor_h7.ino` runtime path behind `LIFETRAC_USE_METHOD_G_HOST` flag.
- Add PTY-based loopback harness (`tools/murata_host_loopback.py`) for end-to-end request/URC smoke tests.
- Add bring-up runbook docs for Max Carrier SWD/OpenOCD flow (if not already landed in separate tranche).
