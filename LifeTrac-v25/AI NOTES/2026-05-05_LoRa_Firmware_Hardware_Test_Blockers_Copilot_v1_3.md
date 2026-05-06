# LoRa Firmware Hardware Test Blockers (Copilot v1.3)

Date: 2026-05-05
Status: **Gate 1 closed locally; Gate 2 remains closed; Gate 3 remains open for physical-board proof.**
Scope: Current hard gates before Method G hardware bring-up on Portenta X8 + Max Carrier.

## 1. Executive Delta From v1.2

This update records the Method G architecture correction and the successful X8 compile preflight rerun.

- Method G H7 builds no longer include or link H7-side RadioLib, ArduinoRS485, ArduinoModbus, or SPI library output.
- Method G bench diagnostics are compiled in by default, so the bench-log case no longer injects a global `LIFETRAC_MH_BENCH_LOG=1` flag that forces a full mbed-core rebuild.
- Gate 1 preflight rerun passed all archived cases:
  - `x8-default`: `ExitCode: 0`
  - `x8-methodg-serial1`: `ExitCode: 0`
  - `x8-methodg-serial1-benchlog-built-in`: `ExitCode: 0`
- Method G compile databases for both Method G cases contain no references to `RadioLib`, `ArduinoRS485`, `ArduinoModbus`, or `libraries\SPI`.
- The accidental `tool_test.txt` evidence artifact was removed.

## 2. Gate Status

### Gate 1 - Production compile preflight matrix

Status: **CLOSED on this workstation**

Evidence file:
- `LifeTrac-v25/DESIGN-CONTROLLER/bench-evidence/T6_bringup_2026-05-05/compile_preflight.log`

Archived summary:
- `Summary: gate1-summary default=0 methodg=0 benchlog=0`

Build architecture result:
- Default build remains the legacy path.
- Method G build routes `setup()` and `loop()` through `murata_host/mh_runtime`.
- Method G build compiles out the legacy RadioLib/RS-485/Modbus sketch body.
- The custom L072 firmware owns SX1276 control; the H7 is only a Method G UART host.

Important nuance:
- The preflight uses `LIFETRAC_MH_SERIAL=Serial1` only as a CI/compile token. `Serial1` is still rejected for physical Max Carrier Method G routing because it is consumed by the X8-H747 link.

### Gate 2 - Runtime firmware defects (plan section 12.2)

Status: **CLOSED**

No change from v1.2.

Validated earlier in this session:
- `mh_cobs_crc_unit` pass
- `mh_stats_vectors` pass
- `mh_runtime_health_vectors` pass
- Windows TCP loopback integration pass

Runtime fixes remain in-tree:
- BOOT `radio_ok` parsed from payload byte 1.
- `TX_DONE_URC` and `RX_FRAME_URC` parsed and tracked.
- Method G bench log sink emits decoded URC and summary lines.

### Gate 3 - Operator evidence package

Status: **OPEN**

Desk evidence exists:
- `LifeTrac-v25/DESIGN-CONTROLLER/bench-evidence/T6_bringup_2026-05-05/serial_routing.md`
- `LifeTrac-v25/DESIGN-CONTROLLER/bench-evidence/T6_bringup_2026-05-05/pinmap_audit.md`
- `LifeTrac-v25/DESIGN-CONTROLLER/bench-evidence/T6_bringup_2026-05-05/flash_path_proof.md`

Still required before hardware power-up:
1. Physical serial-route proof on the real boards, preferably UART_SNIFF or logic-analyzer capture at 921600 8N1 on the selected route.
2. Physical flash-path proof for the target board, either successful SWD verify transcript or successful DFU enumeration/flash transcript.
3. Final selected hardware UART must be a real Max Carrier route, not the compile-only `Serial1` token.

## 3. Current Go/No-Go

Current verdict: **No-go for bench RF power-up until Gate 3 physical proof is archived.**

Reason:
- Gate 1 and Gate 2 are now closed locally.
- Gate 3 still lacks physical-board serial-route and flash-path proof artifacts.

## 4. Immediate Next Actions

1. Use the two connected X8 + Max Carrier boards to capture the real Method G UART route evidence.
2. Prove the L072 flash path on at least one target board with OpenOCD/SWD or DFU transcript.
3. Rebuild Method G with the physically proven `LIFETRAC_MH_SERIAL=<SerialN>` value and append that compile command/result to the evidence package.
4. Reissue the readiness verdict after Gate 3 artifacts are complete.