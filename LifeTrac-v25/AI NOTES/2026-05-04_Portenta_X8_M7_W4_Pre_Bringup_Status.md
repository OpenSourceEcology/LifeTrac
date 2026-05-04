# Portenta X8 M7 W4-pre Bring-up Status - 2026-05-04

## Scope

Goal was to flash the `tractor_h7` M7 firmware at `0x08040000` to two Portenta X8 boards on Max Carriers and advance toward W4-pre/W4-00 gates.

- Board 1: ADB `2D0A1209DABC240B`, COM11
- Board 2: ADB `2E2C1209DABC240B`, COM12
- Arduino FQBN: `arduino:mbed_portenta:envie_m7`
- Latest diagnostic build flags: `-DLIFETRAC_USE_REAL_CRYPTO -DLIFETRAC_X8_NO_USB_SERIAL -DLIFETRAC_X8_BOOT_TRACE -DLIFETRAC_BENCH_RADIO_COUNTERS`
- Flash address: `0x08040000`

## Firmware Bring-up Changes Validated

- Board 2 now reaches `loop()` instead of stalling in Mbed SPI or TRNG startup.
- The X8 RadioLib path uses a custom `LifeTracX8RadioHal` with mode-0 bit-banged SPI to avoid the Mbed H7 SPI polling path that stalled in `spi_master_write()` waiting for RXP.
- The X8/no-USB nonce path avoids Arduino/Mbed `random()` because it entered Mbed `trng_init()` and spun waiting on an unavailable/incorrect hardware-ready condition. The replacement keeps the existing 12-byte nonce shape and uses a local xorshift generator seeded from STM32 UID plus `millis()`/`micros()` only for `ARDUINO_PORTENTA_X8` or `LIFETRAC_X8_NO_USB_SERIAL` builds.
- Earlier X8 startup pieces remain required: patched `libmbed_x8.a`, `x8_no_usb_main.cpp`, `sysclock_x8.c`, `us_ticker_override.c`, and `lp_ticker_override.c`.

## Image/Symbols

Earlier validated binary:

- Path: `%TEMP%\lifetrac_m7_build\tractor_h7.ino.bin`
- Size: `200120` bytes
- Timestamp: `2026-05-04 10:57`
- Key symbols from the validated ELF:
  - `setup = 0x080413ec`
  - `loop = 0x0804170c`
  - `main = 0x08041ad0`
  - `Reset_Handler = 0x08057790`

Latest diagnostic binary:

- Path: `%TEMP%\lifetrac_m7_build\tractor_h7.ino.bin`
- Size: `216524` bytes
- Timestamp: `2026-05-04 14:49:11`
- MSP: `0x24080000`
- Reset vector: `0x0805CC5D`

## Flash Results

Board 2 flashed and verified normally with sudo-backed OpenOCD through ADB.

Board 1 first flash attempt timed out in the STM32H7 flash algorithm while halted in the old app. Retrying with explicit `reset halt` before `flash write_image erase` succeeded and verified.

## W4-pre Evidence Collected

SRAM4 contract used for checks:

- `v(0)`: shared version, expected `2`
- `v(2)`: `alive_tick_ms`
- `v(3)`: E-stop magic if latched
- `v(4)`: loop counter
- CFSR: `0xE000ED28`
- HFSR: `0xE000ED2C`

Board 2:

- Hit `loop()` breakpoint at `0x0804170c`.
- 10 s sample: `v(0)=2`, `v(2)=10344`, `v(3)=0`, `v(4)=14`, CFSR/HFSR `0`.
- Post-run sample after more than 60 s: `v(0)=2`, `v(2)=69631`, `v(4)=43`, CFSR/HFSR `0`.
- `v(3)=0xA5A5A5A5` after the longer run because the Opta/RS-485 Modbus target is not attached; the firmware's existing 10-consecutive-Modbus-fail safety path latches E-stop. This is expected for this two-Max-Carrier-only bench, not a CPU fault.

Board 1:

- Hit `loop()` breakpoint at `0x0804170c`.
- 10 s sample: `v(0)=2`, `v(2)=10344`, `v(3)=0`, `v(4)=14`, CFSR/HFSR `0`.
- Post-run sample near 60 s: `v(0)=2`, `v(2)=59424`, `v(4)=38`, CFSR/HFSR `0`.
- `v(3)=0xA5A5A5A5` after the longer run for the same missing-Opta reason.

## W4-00 Status

W4-00 was not run. The current repo has a W4-00 recording harness at `DESIGN-CONTROLLER/hil/w4-00_lora_dual_portenta.ps1`, but it records operator-entered metrics. The runbook assumes two USB-CDC consoles and a base-side 1000-ControlFrame burst sender. That sender/console path is not implemented for the current X8 no-USB M7 build.

Later low-power radio diagnostics with antennas attached did enqueue telemetry frames, but TX did not start. The `LIFETRAC_BENCH_RADIO_COUNTERS` image wrote `RAD0` counters into SRAM4:

| Board | `TX_ENQ` | `TX_START` | `TX_FAIL` | `TX_DONE` | Reg packs | `stageMode(TX)` | Overall TX meta |
| --- | ---: | ---: | ---: | ---: | --- | --- | --- |
| Board 1 `2D0A1209DABC240B` | 3 | 0 | 3 | 0 | `0`, `0` | `0xBA04FFF0` | `0xBA03FFF0` |
| Board 2 `2E2C1209DABC240B` | 2 | 0 | 2 | 0 | `0`, `0` | `0xBA04FFF0` | `0xBA03FFF0` |

`0xBA04FFF0` decodes as diagnostic stage 4 with RadioLib state `-16` (`RADIOLIB_ERR_SPI_WRITE_FAILED`) from `stageMode(TX)`. The failure happens before `launchMode()` and before the TX-start counter increments. Direct SX127x register snapshots came back zero on both boards, so this is no longer behaving like a simple Mbed SPI timing issue.

## Max Carrier LoRa Interface Finding

**Definitive Confirmation:** The direct M7 raw-SPI assumption is definitively incorrect. The Max Carrier does not route the Murata module's SPI pins to the Portenta's high-density connectors. The module is a standalone modem that exposes an AT command interface over UART (`Serial3` on H7, or `/dev/ttymxc3` on X8 Linux). Raw SPI communication (e.g. `RadioLib` P2P) is physically impossible.

Live X8 Linux probes on both boards found:
- `/dev/ttymxc3` and `/dev/gpiochip5` are present and match the Arduino gateway example.

Conclusion: W4-00 is blocked and fundamentally requires an architectural rewrite. The next implementation path MUST replace the raw-SPI `RadioLib` code with an AT command driver over the appropriate serial endpoint. Standard AT commands (e.g. `AT`, `AT+VER?`, `AT+APPEUI`, `AT+JOIN`, etc.) will be required to control the radio.

## Practical Commands That Worked

Board-side OpenOCD requires sudo from ADB:

```powershell
$adb = "$env:LOCALAPPDATA\Arduino15\packages\arduino\tools\adb\32.0.0\adb.exe"
& $adb -s 2E2C1209DABC240B shell "echo fio | sudo -S openocd -f /tmp/openocd_noreset.cfg -c 'init; reset halt; flash write_image erase /tmp/tractor_h7_x8_nonce_prng.bin 0x08040000 bin; verify_image /tmp/tractor_h7_x8_nonce_prng.bin 0x08040000 bin; reset run; shutdown'"
```

Reliable SRAM4 snapshot pattern:

```sh
echo fio | sudo -S openocd -f /tmp/openocd_noreset.cfg -c "init; halt; mem2array v 32 0x38000000 8; parray v; mem2array c 32 0xE000ED28 1; parray c; mem2array h 32 0xE000ED2C 1; parray h; shutdown" 2>&1
```

Avoid `pkill -f openocd`; use `pkill -x openocd` so the shell command does not self-match.