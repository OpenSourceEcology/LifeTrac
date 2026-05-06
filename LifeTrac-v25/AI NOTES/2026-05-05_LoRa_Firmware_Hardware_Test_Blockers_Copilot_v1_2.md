# LoRa Firmware Hardware Test Blockers (Copilot v1.2)

Date: 2026-05-05
Status: **Not ready yet**. Gate 2 remains closed. Gates 1 and 3 remain open with updated evidence.
Scope: Current hard gates before Method G hardware bring-up on Portenta X8 + Max Carrier.

## 1. Executive Delta From v1.1

This update includes direct Gate 1 compile-matrix execution and Gate 3 evidence-file creation completed in this session.

- Gate 1 compile preflight was executed and archived, and failed deterministically on local stock X8 core.
- Gate 3 desk-evidence files are now present (`serial_routing.md`, `pinmap_audit.md`, `flash_path_proof.md`).
- Gate 3 physical-board proof is still pending.

## 2. Gate Status

### Gate 1 - Production compile preflight matrix

Status: **OPEN**

Evidence file:
- `LifeTrac-v25/DESIGN-CONTROLLER/bench-evidence/T6_bringup_2026-05-05/compile_preflight.log`

Observed matrix outcomes:
- `x8-default`: `ExitCode: 1`
- `x8-methodg-serial2`: `ExitCode: 1`
- `x8-methodg-serial2-benchlog`: initial run interrupted (`ExitCode: -1`), rerun note added

Deterministic root cause (first two complete cases):
- Compile fails inside stock core file:
  - `.../variants/PORTENTA_X8/pins_arduino.h:87` (`PIN_SPI_MOSI (A6)`) and
  - `.../variants/PORTENTA_X8/pins_arduino.h:88` (`PIN_SPI_SCK (A5)`)
- Error text:
  - `error: initializer element is not constant`
- Failure occurs while compiling ArduinoModbus C source (`modbus.c`) through `Arduino.h`.

Interpretation:
- This is a local toolchain/core issue on the stock `arduino:mbed_portenta 4.5.0` X8 core path, not a Method G runtime regression.
- Gate 1 cannot be closed until the X8 compile path is corrected (patched X8 core/toolchain alignment) and the matrix is rerun clean.

Required closure action:
1. Install/use the expected patched X8 core path for this repo's production target.
2. Re-run the same preflight matrix and obtain `ExitCode: 0` for all required variants.

### Gate 2 - Runtime firmware defects (plan section 12.2)

Status: **CLOSED**

No change from v1.1.

Validated previously in this session:
- `mh_cobs_crc_unit` pass
- `mh_stats_vectors` pass
- `mh_runtime_health_vectors` pass
- Windows TCP loopback integration pass

### Gate 3 - Operator evidence package

Status: **OPEN (partially advanced)**

New evidence files now present:
- `LifeTrac-v25/DESIGN-CONTROLLER/bench-evidence/T6_bringup_2026-05-05/serial_routing.md`
- `LifeTrac-v25/DESIGN-CONTROLLER/bench-evidence/T6_bringup_2026-05-05/pinmap_audit.md`
- `LifeTrac-v25/DESIGN-CONTROLLER/bench-evidence/T6_bringup_2026-05-05/flash_path_proof.md`

What was closed in this session:
- Desk-audit serial-route selection record created (candidate: `Serial2` with documented constraints/exclusions).
- Pinmap conflict audit file created from local source/core inspection.
- Flash-path readiness note created; OpenOCD installed via winget and absolute path recorded.

What remains open:
1. Physical serial-route proof on the real board (schematic trace capture and/or UART_SNIFF/logic capture).
2. Physical flash-path proof for target board:
   - successful SWD verify transcript on hardware, or
   - successful DFU enumeration/flash transcript on hardware.
3. `dfu-util` is still missing on this workstation.

## 3. Practical Go/No-Go

Current verdict: **No-go for bench power-up**.

Reason:
- Gate 1 has hard compile failures on the current local X8 toolchain/core.
- Gate 3 lacks physical-board proof artifacts even though desk-evidence docs now exist.

## 4. Immediate Next Actions

1. Fix local X8 compile environment (patched core/toolchain alignment), then rerun Gate 1 matrix to all-pass.
2. Capture physical serial-route and flash-path evidence on the target board and archive under `T6_bringup_2026-05-05`.
3. Reissue readiness verdict after both sets are complete.
