# Method G Hardware Test Status (Copilot v1.2)

Date: 2026-05-05
Status: Serial2 compile/link blocker is resolved in firmware; Serial2 build was flashed to both X8 boards. Linux-side passive endpoint scans still show no observable bytes on `/dev/ttymxc1..3`.
Scope: Delta update after v1.1 focused on the Serial2 route unblock attempt.

## 1. Delta from v1.1

This update records work done after the prior state that reported `_UART2_` link failures for `LIFETRAC_MH_SERIAL=Serial2`.

## 2. Firmware change applied

File updated:
- `LifeTrac-v25/DESIGN-CONTROLLER/firmware/tractor_h7/tractor_h7.ino`

Change summary:
- Added an X8-only Serial2 shim for stock `portenta_x8` cores that expose only `SERIAL_HOWMANY=1`.
- Guard used:
  - `#if defined(ARDUINO_PORTENTA_X8) && (SERIAL_HOWMANY < 2)`
- Injected `_UART2_` object (Arduino core alias target of `Serial2`) using H7 Serial2 mapping:
  - `SERIAL2_TX PA_15`
  - `SERIAL2_RX PF_6`
  - `SERIAL2_RTS PF_8`
  - `SERIAL2_CTS PF_9`

Intent:
- Make `-DLIFETRAC_MH_SERIAL=Serial2` link on stock local X8 core without changing non-X8 behavior.

## 3. Compile result with Serial2

Command profile:
- FQBN: `arduino:mbed_portenta:portenta_x8`
- Flags include:
  - `-DLIFETRAC_USE_METHOD_G_HOST=1`
  - `-DLIFETRAC_MH_SERIAL=Serial2`

Observed result:
- Full compile and link succeeded.
- Artifacts generated in build path:
  - `tractor_h7.ino.elf`
  - `tractor_h7.ino.bin`
  - `tractor_h7.ino.hex`

Notable message (non-fatal in this run context):
- Bootloader file warning from X8 core package path did not prevent build artifact generation.

## 4. Flash result

Serial2 Method G build flashed successfully to both boards:

1. Board A
- Target: `COM11`
- ADB serial: `2D0A1209DABC240B`
- Upload result: file pushed, new upload port reported as `COM11`.

2. Board B
- Target: `COM12`
- ADB serial: `2E2C1209DABC240B`
- Upload result: file pushed, new upload port reported as `COM12`.

## 5. Post-flash Linux-side passive UART scan

ADB-visible devices:
- `2D0A1209DABC240B`
- `2E2C1209DABC240B`

Passive checks attempted on each board for:
- `/dev/ttymxc1`
- `/dev/ttymxc2`
- `/dev/ttymxc3`

Observed outputs:
- byte count samples returned `0` for all tested endpoints during this pass.
- shell emitted utility/TTY warnings during scripted probe (`stty`/`timeout` invocation differences on target userspace), but final sampled counts remained zero.

## 6. Current interpretation

What is now closed:
1. The specific compile/link blocker for `Serial2` on this workstation is resolved in-source.
2. Serial2 Method G firmware can be built and flashed to both connected boards.

What remains open:
1. Runtime route observability/proof on Linux endpoints is still not established from passive `/dev/ttymxc*` sampling.
2. Physical route validation still needs either:
   - proven endpoint capture path that shows live Method G bytes, or
   - external UART sniff/logic capture mapped to the selected serial route.

## 7. Recommended next actions

1. Perform targeted active probe on the selected route (known payload marker + endpoint monitor) with command forms compatible with target userspace tools.
2. If Linux endpoint visibility remains zero, switch to external `UART_SNIFF`/logic analyzer route-proof capture.
3. Once route bytes are visible, archive BOOT/VER evidence for both boards under `bench-evidence` and issue updated readiness verdict.
