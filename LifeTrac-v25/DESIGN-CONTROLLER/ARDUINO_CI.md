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
- `make check-tx-counter-owner` (TX stats increment ownership in `radio/sx1276_tx.c`)
- `make check-lbt-owner` (LBT abort counter ownership in `radio/sx1276_lbt.c`)
- `make check-airtime-counter-owner` (airtime abort counter ownership in `radio/sx1276_tx.c`)
- `make check-cfg-contract` (CFG policy/unit + wire-vector host tests)

The dependent `firmware-l072-cross-compile` job then runs `make all` with `arm-none-eabi-gcc` on pinned `ubuntu-24.04`, enforces APP/RAM budgets via `tools/check_size_budget.py`, and uploads ELF/BIN/HEX/MAP artifacts.

The `h7-host-driver-tests` job validates the tracked `firmware/tractor_h7/murata_host` wire layer:

- verifies mirrored host-wire constants via `tools/check_mh_wire_sync.py`
- runs COBS + CRC16 unit tests (`mh_cobs_crc_unit`)
- runs STATS parser vectors for both legacy 64-byte and additive 68-byte payloads (`mh_stats_vectors`)
- runs runtime-health vectors for BOOT field mapping and `TX_DONE_URC`/`RX_FRAME_URC` handling (`mh_runtime_health_vectors`)

The `h7-host-driver-loopback` job adds an integration harness for the same layer:

- builds `bench/h7_host_proto/loopback_driver.c` against `mh_stream` + POSIX/TCP UART transport adapters
- runs `tools/murata_host_loopback.py` in PTY mode on Linux, where the mock L072 peer injects chunked I/O and malformed frames
- verifies request/response flow (`PING`, `CFG_GET`, `STATS_DUMP`) while asserting reject counters for CRC/length/version errors

For local non-WSL Windows runs, the same harness can use TCP loopback mode (`--transport tcp`) with the same mock peer and verdict checks.

The `tractor-h7-method-g-guards` job enforces compile-time exclusivity for Method G runtime wiring:

- compiles default `tractor_h7` build (Method G disabled) and fails if `mh_runtime_*`/`mh_uart_arduino_*` symbols leak in
- compiles Method G enabled build (`LIFETRAC_USE_METHOD_G_HOST=1`, `LIFETRAC_MH_SERIAL=Serial1`) and asserts runtime symbols are present

Repository maintainers should mark `L072 firmware static checks`, `L072 cross-compile (arm-none-eabi-gcc)`, `H7 host driver wire tests`, `H7 host driver PTY loopback`, and `tractor_h7 Method G runtime guards` as required branch-protection checks on `main`.

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

## Local Host Loopback (Windows, Non-WSL)

```bash
cd LifeTrac-v25/DESIGN-CONTROLLER/firmware/tractor_h7
mkdir -p build
gcc -std=gnu11 -Wall -Wextra -Werror -Wpedantic -Imurata_host \
	bench/h7_host_proto/loopback_driver.c \
	murata_host/mh_cobs.c \
	murata_host/mh_crc16.c \
	murata_host/murata_host.c \
	murata_host/mh_stats.c \
	murata_host/mh_uart.c \
	murata_host/mh_uart_tcp.c \
	murata_host/mh_stream.c \
	-lws2_32 \
	-o build/loopback_driver_tcp.exe

python ../../tools/murata_host_loopback.py \
	--transport tcp \
	--driver build/loopback_driver_tcp.exe \
	--iterations 150
```

## Python Protocol Gate

The base-station protocol tests are standard-library `unittest` tests and can run before hardware is available:

```bash
cd LifeTrac-v25/DESIGN-CONTROLLER/base_station
python -m unittest discover -s tests
```

These tests verify CRC/KISS packing, ControlFrame layout, FHSS channel generation, and the `CMD_ENCODE_MODE` hysteresis controller.