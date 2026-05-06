# LoRa Firmware Hardware Test Blockers (Copilot v1.1)

Date: 2026-05-05
Status: **Not ready yet**. Gate 2 is closed; Gates 1 and 3 remain open.
Scope: Current hard gates before powering Portenta X8 + Max Carrier for Method G hardware test start.

## 1. Executive Delta From v1.0

This update reflects direct source inspection plus local host/runtime regression execution completed in this session.

- Gate 2 (firmware defects in runtime parser/handling/visibility) is **closed**.
- Gate 1 (toolchain and production-target compile evidence) is **still open**.
- Gate 3 (operator evidence package) is **still open**.

## 2. Gate Status

### Gate 1 - Toolchain / production-target compile path

Status: **OPEN**

Observed local toolchain state:

- `arduino-cli` version: `1.4.1`
- Installed cores include:
  - `arduino:mbed_portenta 4.5.0`
  - `arduino:mbed_opta 4.5.0`
- FQBNs listed include `arduino:mbed_portenta:portenta_x8` and `arduino:mbed_portenta:envie_m7`.

Why still open:

- Production readiness requires successful compile evidence for the X8 target path and transcript capture into bench evidence.
- This session confirms core presence and target visibility, but does not yet establish the final production compile artifact + logged transcript closure for all required preflight variants.
- `DESIGN-CONTROLLER/bench-evidence/T6_bringup_2026-05-05/compile_preflight.log` currently records local toolchain probe output; full production compile transcript is still pending.

Required closure action:

1. Run production-target compile preflight with `--fqbn arduino:mbed_portenta:portenta_x8`.
2. Save transcript to `DESIGN-CONTROLLER/bench-evidence/T6_bringup_<date>/compile_preflight.log`.
3. Confirm required preflight matrix entries pass (default and Method G variants used for Day 1 gate).

### Gate 2 - Runtime firmware defects from Plan §12.2

Status: **CLOSED**

#### 2.1 BOOT_URC parser alignment

Fixed in `firmware/tractor_h7/murata_host/mh_runtime_health.c`:

- `boot_radio_ok` is parsed from `payload[1]`.

#### 2.2 TX_DONE and RX_FRAME handling in runtime path

Implemented in `firmware/tractor_h7/murata_host/mh_runtime_health.c`:

- `HOST_TYPE_TX_DONE_URC` parsed and tracked (`tx_done_count`, tx metadata).
- `HOST_TYPE_RX_FRAME_URC` parsed and tracked (`rx_frame_count`, rx metadata, payload copy).

#### 2.3 Bench-visibility logger

Implemented and wired:

- Runtime log sink API and frame/summary emit path in `firmware/tractor_h7/murata_host/mh_runtime.c`.
- Sketch setup wiring and optional USB serial sink in `firmware/tractor_h7/tractor_h7.ino` under `LIFETRAC_MH_BENCH_LOG=1`.

#### 2.4 Regression evidence (executed this session)

Host C vectors:

- `[PASS] mh_cobs_crc_unit: 4 checks`
- `[PASS] mh_stats_vectors: 4 vectors`
- `[PASS] mh_runtime_health_vectors: 4 vectors`

Windows TCP loopback integration:

- `loopback verdict pass: iterations=150 ping=150 cfg=150 stats=150 legacy=75 additive=75 rej_crc=1 rej_len=1 rej_ver=1 rej_unknown=0`

Conclusion for Gate 2:

- The three listed firmware defects are already fixed in-tree and currently regressed green.
- No additional Gate 2 code patch is required before bench start.

### Gate 3 - Operator evidence package

Status: **OPEN**

Still required before hardware power-up:

1. `serial_routing.md`: schematic/probe proof of selected `LIFETRAC_MH_SERIAL` route (`Serial2`/`Serial4`/`Serial5`).
2. `pinmap_audit.md`: mbed `PeripheralPins.c` audit for chosen mapping.
3. Flash-path proof for exact board:
   - CN2-soldered SWD + OpenOCD verify transcript, or
   - `dfu-util -l` enumeration + flash evidence.

## 3. Practical Go/No-Go

Current verdict: **No-go for bench power-up**.

Reason:

- Gate 2 is complete, but Gate 1 compile-evidence closure and Gate 3 operator hardware-path evidence are not yet closed.

## 4. Immediate Next Actions

1. Execute and archive X8 production compile preflight transcript (Gate 1).
2. Finish serial routing + pinmap + flash-path evidence bundle (Gate 3).
3. Re-issue readiness verdict after both evidence sets are attached.
