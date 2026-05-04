# W4-pre Bench Evidence Notes - 2026-05-04

## Status

W4-pre is still open. This file records software-visible evidence from the two-Portenta-X8 desk setup and the remaining physical checks needed before W4-pre can be signed off.

No confirmed LoRa RF TX start was observed during this evidence capture. Do not plug in the USB camera yet.

## Hardware Observed

- Board 1: ADB `2D0A1209DABC240B`, Windows `COM11`
- Board 2: ADB `2E2C1209DABC240B`, Windows `COM12`
- Both boards were visible in `adb devices -l` as `device`.
- Windows Device Manager / PnP reported two present `USB Serial Device` ports: `COM11` and `COM12`.

## Firmware Image

- Sketch: `DESIGN-CONTROLLER/firmware/tractor_h7/tractor_h7.ino`
- Flash address: `0x08040000`
- Latest diagnostic build flags: `-DLIFETRAC_USE_REAL_CRYPTO -DLIFETRAC_X8_NO_USB_SERIAL -DLIFETRAC_X8_BOOT_TRACE -DLIFETRAC_BENCH_RADIO_COUNTERS`
- Bench TX power stayed at the default `LIFETRAC_BENCH_TX_DBM=2`.
- Latest diagnostic artifact: `%TEMP%\lifetrac_m7_build\tractor_h7.ino.bin`
  - Size: `216524` bytes
  - Timestamp: `2026-05-04 14:49:11`
  - MSP: `0x24080000`
  - Reset vector: `0x0805CC5D`
- Earlier W4-pre liveness samples used a prior validated reset vector table at `0x08040000`:
  - MSP: `0x24080000`
  - Reset vector: `0x08057791`

## SRAM4 Liveness Evidence

SRAM4 contract:

- `v(0)`: shared version, expected `2`
- `v(2)`: `alive_tick_ms`
- `v(3)`: `estop_request`
- `v(4)`: loop counter
- `c(0)`: CFSR at `0xE000ED28`
- `h(0)`: HFSR at `0xE000ED2C`

After explicit app launch from the `0x08040000` vector, both boards reached a clean early-boot loop state:

| Board | Mode / PC | `v(0)` | `v(1)` | `v(2)` | `v(3)` | `v(4)` | CFSR | HFSR |
| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| Board 1 `2D0A1209DABC240B` | Thread, PC `0x08051610` | 2 | 28 | 10344 | 0 | 14 | 0 | 0 |
| Board 2 `2E2C1209DABC240B` | Thread, PC `0x080513f2` | 2 | 28 | 10344 | 0 | 14 | 0 | 0 |

Interpretation: both M7 cores entered the app, updated the shared SRAM4 liveness fields, reached `loop()`, had no active E-stop latch in the early sample, and had no Cortex fault flags at capture time.

After the LoRa antennas were attached, a non-reset snapshot showed both boards still alive in thread mode with CFSR/HFSR clear:

| Board | Mode / PC | `v(0)` | `v(1)` | `v(2)` | `v(3)` | `v(4)` | CFSR | HFSR |
| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| Board 1 `2D0A1209DABC240B` | Thread, PC `0x08050a30` | 2 | 318 | 306863 | `0xA5A5A5A5` | 159 | 0 | 0 |
| Board 2 `2E2C1209DABC240B` | Thread, PC `0x080504fa` | 2 | 296 | 284365 | `0xA5A5A5A5` | 148 | 0 | 0 |

The E-stop magic in this later snapshot is the expected missing-Opta/RS-485 latch from the two-carrier-only bench shape, not a Cortex fault.

## Reset Caveat Found During Capture

Generic OpenOCD `reset run` is not a reliable way to relaunch this X8 M7 app image on the current bench. During this capture, Board 1 entered HardFault after a generic reset-run:

- PC: `0x08001018`
- CFSR: `0x00000100`
- HFSR: `0x40000000`

Board 1 recovered after an explicit app-vector launch sequence:

```text
init;
reset halt;
mww 0xE000ED08 0x08040000;
mww 0xE000ED28 0xFFFFFFFF;
mww 0xE000ED2C 0xFFFFFFFF;
reg xpsr 0x01000000;
reg msp 0x24080000;
reg psp 0x24080000;
reg pc 0x08057791;
resume;
shutdown
```

Use the explicit vector-launch path for repeatable W4-pre firmware-boot sampling unless the X8 boot/reset path is changed.

For the latest diagnostic artifact, the explicit launch PC is `0x0805CC5D`; use the reset vector that matches the binary being sampled.

## W4-pre Physical Checks Still Open

Do not mark W4-pre PASS until these are captured:

Physical-check status for this session: no USB replug-cycle count, DMM rail measurements, blink/echo result, or dual-core handshake result was available. Treat all physical sub-gates below as still open.

- [ ] USB enumeration/replug: 10 replug cycles per carrier, 20 total successful cycles, with distinct stable ports.
- [ ] Rails: DMM readings for Max Carrier 3V3 and 5V0 on each board under no-RF load. Target 3V3 range: 3.135 V to 3.465 V. Target 5V0 range: 4.75 V to 5.25 V.
- [ ] Blink/echo: flash/run an appropriate USB-CDC echo or stock bring-up sketch and record worst-case echo round-trip under 50 ms. The current `LIFETRAC_X8_NO_USB_SERIAL` image cannot provide a normal USB-CDC console echo.
- [ ] Dual-core handshake: run the stock Portenta M7<->M4 dual-core example for 60 s and record zero M4 counter gaps.
- [x] Firmware boot/liveness: partial evidence captured via ADB/OpenOCD SRAM4 snapshot above.

## LoRa And Camera Hold Points

- Do not run W4-00 yet; the current harness records metrics but does not generate radio traffic.
- Current RF attachment status: user reports 915 MHz LoRa antennas are attached to both boards. This clears the unloaded-SMA TX hold for a low-power first-light sanity check, but it does not close W4-pre or W4-00.
- Current firmware limitation: the no-USB tractor image can enqueue 1 Hz `TOPIC_SOURCE_ACTIVE` telemetry and publish SRAM4 bench counters, but the assumed direct M7-to-SX1276 SPI path fails before TX start. Treat this as a blocked radio-interface diagnostic, not a LoRa RF pass.
- Before any LoRa TX, attach 915 MHz antennas or 50 ohm dummy loads to both Max Carriers, keep radiating antennas at least 30 cm apart, and keep TX power at the bench default `LIFETRAC_BENCH_TX_DBM=2`.
- Do not plug in the USB camera until W4-pre is closed and the low-power radio first-light path is stable.

## Informal LoRa Diagnostic Observations

With both antennas attached, the validated ELF symbols were used to sample internal TX state over OpenOCD. `g_telem_seq` lives at `0x24001c8c`; it increments when `emit_topic()` builds an encrypted telemetry frame and queues it for the async TX pump.

| Board | Sample 1 `g_telem_seq` | Sample 2 `g_telem_seq` | Interpretation |
| --- | ---: | ---: | --- |
| Board 1 `2D0A1209DABC240B` | 270 | 276 | Source-active telemetry frames are being generated and queued. |
| Board 2 `2E2C1209DABC240B` | 257 | 261 | Source-active telemetry frames are being generated and queued. |

The latest `LIFETRAC_BENCH_RADIO_COUNTERS` image exported radio diagnostic counters in SRAM4. `sh(5)` held the `RAD0` magic (`0x52414430` / `1380009008`) on both boards. The relevant counter map is:

- `sh(6)`: TX enqueue
- `sh(7)`: TX start
- `sh(8)`: TX fail
- `sh(9)`: TX done
- `sh(11)`: diagnostic SX127x register pack 1
- `sh(12)`: diagnostic SX127x register pack 2
- `sh(13)`: `stageMode(TX)` status meta
- `sh(14)`: `launchMode()` status meta
- `sh(15)`: overall TX status meta

Latest diagnostic sample:

| Board | `TX_ENQ` | `TX_START` | `TX_FAIL` | `TX_DONE` | Reg pack 1 | Reg pack 2 | `stageMode(TX)` | Overall TX meta |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | --- | --- |
| Board 1 `2D0A1209DABC240B` | 3 | 0 | 3 | 0 | 0 | 0 | `0xBA04FFF0` | `0xBA03FFF0` |
| Board 2 `2E2C1209DABC240B` | 2 | 0 | 2 | 0 | 0 | 0 | `0xBA04FFF0` | `0xBA03FFF0` |

Decode: `0xBA04FFF0` means diagnostic stage 4, state `-16` (`RADIOLIB_ERR_SPI_WRITE_FAILED`) from `stageMode(TX)`. The failure happens before `launchMode()` and before any TX start counter increments. The raw SX127x register snapshots are all zero on both boards, which makes the assumed direct M7 raw-SPI path suspect.

## Max Carrier LoRa Interface Finding

Arduino's Portenta Max Carrier documentation identifies the onboard LPWAN module as Murata `CMWX1ZZABZ-078`. The Portenta X8 multi-protocol gateway example does not use a raw SX1276 SPI device; it opens `/dev/ttymxc3` at 19200 baud and drives an AT-command LoRa modem, with reset on `/dev/gpiochip5` line 3 (`gpio163` in the example comments).

Live X8 Linux probes on both boards found:

- USB enumerates the Max Carrier hub (`0424:2514`) but no separate LoRa USB device.
- No `/dev/spidev*` device was present.
- `/dev/ttymxc3` and `/dev/gpiochip5` are present and match the Arduino gateway example.
- No LoRa/SX127/Murata device-tree node was found in the simple probe.
- A non-reset AT ping attempt on Board 2 returned a modem-style `+ERR_RX` response, but post-reset identity queries (`AT`, `AT+DEV?`, `AT+VER?`, `AT+DEVEUI?`) did not yet return clean responses. Treat live AT identity as inconclusive, not failed.

Conclusion: W4-00 is blocked on radio-interface revalidation. The next implementation path should identify and prove the real Murata module control path, likely X8 Linux `/dev/ttymxc3` AT commands, before any 1000-frame burst or camera work.
