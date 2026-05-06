# Method G Hardware Test Status (Copilot v1.3)

Date: 2026-05-05
Status: Software bridge mode is implemented, build-validated, and flashed to one X8 board. Runtime observability is still blocked by endpoint/access constraints (no bytes visible on host COM ports; Linux H7 bridge node requires elevated access).
Scope: Delta update after v1.2 focused on the software bridge validation path ("Option 1").

## 1. Delta from v1.2

This update records work done after the architecture diagnosis in v1.2 Section 8 and executes the software-first observability path.

## 2. Firmware changes applied

File updated:
- `LifeTrac-v25/DESIGN-CONTROLLER/firmware/tractor_h7/tractor_h7.ino`

New behavior added (compile-time gated):
1. Added `LIFETRAC_MH_SOFT_BRIDGE` (default `0`).
2. In Method G build, when bridge mode is enabled:
- `setup()` starts a debug serial endpoint and `LIFETRAC_MH_SERIAL` (921600), then enters bridge mode instead of `mh_runtime_begin()`.
- `loop()` calls `mh_soft_bridge_pump()` to forward bytes both directions:
  - `LIFETRAC_MH_SERIAL -> bridge debug serial`
  - `bridge debug serial -> LIFETRAC_MH_SERIAL`
3. Disabled X8 no-op Serial remap when bridge mode is enabled.
4. Added selectable bridge debug endpoint:
- Default on X8: `SerialRPC`
- Default elsewhere: `Serial`
- New override hook: `-DLIFETRAC_MH_BRIDGE_DEBUG_SERIAL=...`
5. Auto-disabled Method G bench log when bridge mode is on (`LIFETRAC_MH_BENCH_LOG=0` default in that mode).

Intent:
- Keep normal Method G runtime unchanged by default.
- Enable software-only traffic forwarding/inspection when explicitly requested.

## 3. Build validation results

### Build A: Bridge mode with X8 default debug endpoint (SerialRPC)
Flags:
- `-DLIFETRAC_USE_METHOD_G_HOST=1`
- `-DLIFETRAC_MH_SERIAL=Serial2`
- `-DLIFETRAC_MH_SOFT_BRIDGE=1`

Result:
- Compile and link succeeded.
- Artifacts generated (`.elf/.bin/.hex`).

### Build B: Bridge mode with explicit debug override to `Serial`
Extra flag:
- `-DLIFETRAC_MH_BRIDGE_DEBUG_SERIAL=Serial`

Result:
- Compile and link also succeeded.
- Artifacts generated in separate build path.

## 4. Flash attempt results

Target board:
- COM: `COM11`
- ADB serial: `2D0A1209DABC240B`

Observed upload behavior:
1. Initial upload attempts failed with:
- `adb: error: failed to get feature set: more than one device/emulator`
2. Resolution:
- Set `ANDROID_SERIAL=2D0A1209DABC240B` for upload command context.
3. After forcing serial selection:
- Upload succeeded for Build A (SerialRPC default).
- Upload succeeded for Build B (Serial override).

## 5. Runtime observability checks executed

### Host-side serial checks
Read attempts at `115200` on:
- `COM11`
- `COM3`

Observed result:
- `NO_DATA` on both checks.

### Linux-side checks via ADB shell (same board)
- Enumerated `/dev` tty nodes, including `ttyX0`, `ttyGS0`, and `ttymxc1..3`.
- Attempted to probe `ttyX0` directly.

Observed result:
- Access denied to `ttyX0` as non-root user (`fio`).
- `adb root` unavailable on production image (`adbd cannot run as root in production builds`).
- `sudo -n` confirms password is required for privileged tty access.

## 6. Current interpretation

Closed in this pass:
1. Software bridge functionality is implemented and source-integrated.
2. Both bridge routing variants compile cleanly on X8 toolchain.
3. Bridge firmware was successfully flashed to board `2D0A1209DABC240B`.

Still open:
1. Verified byte-level observability evidence from the active bridge path is not yet captured.
2. The likely Linux-visible H7 path (`/dev/ttyX0`) is currently blocked by privilege constraints in this environment.

## 7. Immediate next actions

1. Obtain privileged read access for `ttyX0` (or a one-time `sudo` session) and run directed capture while bridge firmware is active.
2. If privileged Linux capture is not available, proceed with physical capture path already recommended in v1.2 Section 8:
- UART_SNIFF route
- external USB-UART tap
- logic analyzer
3. Once any capture path is active, inject a deterministic marker payload and archive evidence with timestamp and board serial.

## 8. Practical command notes from this pass

- Upload to one of multiple attached X8 boards required selecting ADB target explicitly via environment variable:
  - `ANDROID_SERIAL=<board_adb_serial>`
- Without this, uploader fails with multi-device ADB ambiguity even when a COM port is supplied.
