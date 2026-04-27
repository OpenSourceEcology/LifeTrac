# Arduino CI for LifeTrac v25

The active v25 Arduino firmware is the LoRa/Modbus controller tree under `DESIGN-CONTROLLER/firmware/`:

- `firmware/handheld_mkr/handheld.ino` — MKR WAN 1310 handheld LoRa controller
- `firmware/tractor_h7/tractor_m7.ino` and `tractor_m4.cpp` — Portenta X8 onboard H747 co-MCU tractor firmware
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