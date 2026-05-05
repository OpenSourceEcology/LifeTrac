# Arduino CI for LifeTrac v25

The active v25 Arduino firmware is the LoRa/Modbus controller tree under `DESIGN-CONTROLLER/firmware/`:

- `firmware/handheld_mkr/handheld_mkr.ino` — MKR WAN 1310 handheld LoRa controller
- `firmware/tractor_h7/tractor_h7.ino` (M7) and `firmware/tractor_h7_m4/tractor_h7_m4.ino` (M4) — Portenta X8 onboard H747 co-MCU tractor firmware (split across two sketch folders for arduino-cli; see Round 17 in the Implementation Status memo)
- `firmware/tractor_opta/opta_modbus_slave.ino` — Opta Modbus-RTU hydraulic I/O slave
- `firmware/bench/lora_retune_bench/lora_retune_bench.ino` — LoRa retune timing bench sketch

The old `arduino_opta_controller` and `esp32_remote_control` sketches are archived under `RESEARCH-CONTROLLER/` and are intentionally excluded from the active CI gate.

## What The Workflow Should Compile

| Target | FQBN | Status |
|---|---|---|
| Handheld MKR WAN 1310 | `arduino:samd:mkrwan1310` | Ready for CI once shared C compilation is wired into the sketch/build step. |
| Opta Modbus slave | `arduino:mbed_opta:opta` | Ready for CI; expansion-bus functions are placeholders until hardware bring-up. |
| Tractor H747 M7/M4 | Portenta/X8 H747 core FQBN | Needs local FQBN verification before making CI blocking. |
| LoRa retune bench | same board as radio under test | Manual bench tool; not required on every PR. |

## L072 Static Gates

The `firmware-l072-static-checks` workflow job validates host-buildable invariants for
`firmware/murata_l072` without requiring the cross toolchain:

- `make check` (memory map preprocess + static asserts)
- `make check-opmode-owner` (only `radio/sx1276_modes.c` may own `RegOpMode`)
- `make check-cfg-wire-owner` (CFG URC payload layout owned by `host_cfg_wire`)
- `make check-rx-counter-owner` (RX stats increment ownership in `radio/sx1276_rx.c`)
- `make check-cfg-contract` (CFG policy/unit + wire-vector host tests)

Repository maintainers should mark `L072 firmware static checks` as a required branch-protection check on `main`.

## Local Smoke Test

```bash
arduino-cli core update-index
arduino-cli core install arduino:samd
arduino-cli core install arduino:mbed_opta
arduino-cli core install arduino:mbed_portenta

arduino-cli lib install RadioLib
arduino-cli lib install ArduinoRS485
arduino-cli lib install ArduinoModbus
arduino-cli lib install OptaController

arduino-cli compile --fqbn arduino:samd:mkrwan1310 LifeTrac-v25/DESIGN-CONTROLLER/firmware/handheld_mkr
arduino-cli compile --fqbn arduino:mbed_opta:opta LifeTrac-v25/DESIGN-CONTROLLER/firmware/tractor_opta
```

The first hardware compile pass may still need sketch-folder build glue for `firmware/common/lora_proto/*.c`. Keep that fix in the active tree; do not copy code back out to `RESEARCH-CONTROLLER/`.

## Python Protocol Gate

The base-station protocol tests are standard-library `unittest` tests and can run before hardware is available:

```bash
cd LifeTrac-v25/DESIGN-CONTROLLER/base_station
python -m unittest discover -s tests
```

These tests verify CRC/KISS packing, ControlFrame layout, FHSS channel generation, and the `CMD_ENCODE_MODE` hysteresis controller.