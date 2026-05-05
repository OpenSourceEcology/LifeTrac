# LoRa Firmware Tranche 5 Implementation (Copilot v1.0)

Date: 2026-05-05
Scope: L072 tranche implementation pass after v1.2 plan review.

## Implemented in this pass

### 1) PR0 clock-source gate and BOOT/FAULT telemetry
- Added RCC HSE register and SYSCLK switch definitions.
- Updated platform clock init to try HSE first (32 MHz), then fall back to HSI16 if HSE is unavailable.
- Added `platform_clock_source_id()` API and source IDs in `platform.h`.
- BOOT_URC reserved byte now carries `clock_source_id`.
- Added `HOST_FAULT_CODE_CLOCK_HSE_FAILED` and emit FAULT_URC at boot if HSE is not active.

### 2) LBT/CAD and airtime hygiene core
- Added `radio/sx1276_cad.c` + `include/sx1276_cad.h` for CAD begin/poll.
- Added `radio/sx1276_lbt.c` + `include/sx1276_lbt.h` for LBT check/backoff using CAD + RSSI sampling.
- Added `radio/sx1276_airtime.c` + `include/sx1276_airtime.h` for shared ToA estimate and per-channel airtime budget reservation.
- `sx1276_tx.c` now:
  - runs LBT before TX,
  - reserves airtime before entering TX mode,
  - releases reservation on pre-TX failure,
  - counts airtime budget aborts.

### 3) STATS payload additive extension
- Added `radio_tx_abort_airtime` counter in host stats.
- Bumped STATS payload from 64 to 68 bytes, additive tail only.
- Added explicit `HOST_STATS_OFFSET_*` constants including the new field offset.

### 4) CFG extensions (with explicit key-gap discipline)
- Added CFG keys:
  - `0x10` `CFG_KEY_LBT_MAX_BACKOFF_MS` (u16)
  - `0x11` `CFG_KEY_CAD_SYMBOLS` (u8)
  - `0x13` `CFG_KEY_FHSS_DWELL_MS` (u16)
- Preserved `0x0F` as intentionally unused.
- Added defaults and validation ranges in `host_cfg.c`.
- Expanded CFG contract and wire-vector tests for the new keys.

### 5) CI/static ownership checks
- Added Makefile static checks:
  - `check-lbt-owner`
  - `check-airtime-counter-owner`
- Wired both checks into `.github/workflows/arduino-ci.yml`.
- Documented both checks in `ARDUINO_CI.md`.

## Validation run in this pass
- `mingw32-make check`
- `mingw32-make check-cfg-contract`
- `mingw32-make check-opmode-owner`
- `mingw32-make check-cfg-wire-owner`
- `mingw32-make check-rx-counter-owner`
- `mingw32-make check-tx-counter-owner`
- `mingw32-make check-lbt-owner`
- `mingw32-make check-airtime-counter-owner`

All above passed in local host-check environment.

## Deferred from full Tranche 5 plan
- H7-side minimal Method G host driver carryover and dedicated parser/golden-vector coverage for 64 B + 68 B STATS coexistence.
- Formal deterministic FHSS bench profile module and epoch-sync plumbing beyond current key/airtime groundwork.
- Bring-up runbook artifacts for MAX CARRIER + OpenOCD scripting were not added in this pass.

## Notes
- Existing dirty worktree state from prior tranche work was preserved.
- Source-only changes were kept; generated host test binaries were restored.
