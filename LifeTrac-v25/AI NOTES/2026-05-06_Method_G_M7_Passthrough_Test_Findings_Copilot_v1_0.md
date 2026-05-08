# Method G M7 Passthrough Test Findings (Copilot v1.0)

Date: 2026-05-06  
Author: GitHub Copilot  
Scope: Continued Phase 1 bench testing after installing Arduino CLI and creating the Portenta M7 L072 passthrough/ping bridge sketch.

---

## 1. Summary

The Portenta M7 bridge sketch now builds for both target profiles and was uploaded successfully to both connected Portenta X8 boards.

However, the expected direct Windows USB serial-monitor path is not available on these X8 boards:
- COM11 opens but write attempts time out with `The semaphore timeout period has expired`.
- COM12 is locked by another process and cannot be opened from this session.
- `SerialRPC` output is not observable as a normal Windows COM byte stream.
- `m4_proxy` TCP ports can be reached through ADB forwarding, but passive and safe injected `tty` probes produced no decoded bridge output.

Fallback Linux-side probing over `/dev/ttymxc3` confirmed that board `2E2C1209DABC240B` still shows the known stale Murata AT firmware symptom at 19200 8N1: repeated `Error when receiving` / `+ERR_RX`. Board `2D0A1209DABC240B` produced no printable AT responses at 19200, 115200, 921600, or 9600 8N1.

Result: Phase 1 live AT validation of the custom L072 firmware is still blocked. The bridge sketch itself is now compile/upload-ready, but Portenta X8 USB/SerialRPC observability is not a usable monitor path in this bench state.

Correction after the Max Carrier debugger DIP switch was enabled: the Micro-USB debug path now enumerates cleanly as J-Link CDC UART ports, but the earlier `Serial1` PC-side assumption is invalid for Method G on Max Carrier. Existing project routing evidence says `Serial1` is the X8<->H747 link; Method G bench candidates remain `Serial2`/`Serial4`/`Serial5`, with `Serial2` as the only locally mapped candidate. The newly visible J-Link CDC ports have not yet shown traffic from the bridge or route-probe sketches.

---

## 2. Build and Upload Results

Sketch:
`LifeTrac-v25/DESIGN-CONTROLLER/firmware/portenta_m7_l072_passthrough_ping/portenta_m7_l072_passthrough_ping.ino`

Validated compile targets:
- `arduino:mbed_portenta:envie_m7`: PASS
- `arduino:mbed_portenta:portenta_x8`: PASS

Patch applied during testing:
- Added the X8-only `Serial2`/`_UART2_` instantiation used by the existing `tractor_h7` firmware pattern.
- Changed the X8 PC/debug side default from `SerialRPC` to `Serial1` for the Max Carrier debug Micro-USB / STLink virtual COM port path.
- Added AT baud scanning across `115200`, `19200`, `921600`, and `9600`.
- Disabled local command parsing after `!boot` so future `stm32flash` byte streams are not corrupted by bridge command interception.

Later correction:
- The `Serial1` PC/debug-side bridge assumption did not survive Max Carrier route proof. `Serial1` is reserved for the X8<->H747 link in the main controller firmware and must not be treated as the Method G/Max Carrier debug path.
- The existing `x8_uart_route_probe` sketch was repurposed to emit labeled `LT_ROUTE_PROBE` lines on `Serial1`, `Serial2`, and `Serial3` at 115200 8N1 for J-Link CDC route discovery.

Upload results:
- Board `2D0A1209DABC240B`: upload PASS through Arduino CLI / ADB; latest bridge uses `Serial1` as PC side on X8.
- Board `2E2C1209DABC240B`: upload PASS through Arduino CLI / ADB; latest bridge uses `Serial1` as PC side on X8.

---

## 3. Port Mapping

Arduino CLI JSON mapping:

| COM Port | X8 Serial Number | Notes |
|---|---|---|
| COM11 | `2E2C1209DABC240B` | Accessible for open; writes timeout |
| COM12 | `2D0A1209DABC240B` | Access denied from this session |

ADB devices:
- `2D0A1209DABC240B` -> `portenta-x8-2d0a1209dabc240b`
- `2E2C1209DABC240B` -> `portenta-x8-2e2c1209dabc240b`

After enabling the Max Carrier debugger DIP switch, Windows also enumerated two J-Link CDC UART ports:

| COM Port | J-Link Serial | USB ID | Notes |
|---|---:|---|---|
| COM13 | `1078222309` | `USB\VID_1366&PID_0105\001078222309` | `JLink CDC UART Port` |
| COM8 | `1078180658` | `USB\VID_1366&PID_0105\001078180658` | `JLink CDC UART Port` |

`JLink.exe ShowEmuList` reports both probes as `J-Link-OB-STM32F4405-Arduino`.

---

## 4. Windows COM Probe Results

COM11 after bridge upload:
- Open at 115200 8N1: PASS
- Startup text: none captured
- `!help`, `!ping`, `AT`, `ATI`, `AT+VER?`: write failed with semaphore timeout

COM12:
- Open at 115200 8N1: FAIL
- Error: access denied

Interpretation:
- These Portenta X8 USB serial interfaces are not acting as a direct M7 serial-monitor path for the uploaded bridge sketch in this state.
- The desired direct PC USB-C to M7 to L072 bridge workflow is likely valid for standalone Portenta H7, but not observable through these X8 COM endpoints without additional X8-side bridge support.

---

## 5. SerialRPC / m4_proxy Probe Results

For board `2E2C1209DABC240B`, ADB forwarding was configured:
- Host `tcp:15020` -> X8 `tcp:5000`
- Host `tcp:15021` -> X8 `tcp:5001`

Passive probe:
- Connected to both forwarded ports.
- Received `raw_rx=0`, `decoded_rx=0` on both.
- Expected text `bridge` not found.

Safe `tty` injection probe:
- Sent a msgpack-rpc `tty` notification containing:
  - `!help`
  - `!ping`
  - `AT`
  - `ATI`
  - `AT+VER?`
- Port `15021` peer closed.
- Port `15020` received no decoded output.
- Expected text `bridge` not found.

Post-probe state:
- `/usr/bin/m4_proxy` remained running.
- Linux listeners still present on `0.0.0.0:5001` and `:::5000`.
- No `/dev/ttyRPMSG*` or `/dev/rpmsg*` nodes were present.

Interpretation:
- The X8 `SerialRPC` path is still opaque and does not provide a practical software monitor path for this immediate Method G bench gate.

---

## 6. Linux `/dev/ttymxc3` AT Probe Results

Probe path:
- UART: `/dev/ttymxc3`
- Reset GPIO: `gpio163`
- Commands: `AT`, `ATI`, `AT+VER?`, `AT+RADIO?`, `AT+STAT?`
- Bauds: `19200`, `115200`, `921600`, `9600` 8N1

Board `2E2C1209DABC240B`:
- `19200 8N1`: repeated `Error when receiving` / `+ERR_RX`
- `19200 8N2`: repeated `+ERR`
- `115200 8N1`: no printable response / null bytes in full probe
- `921600 8N1`: no data
- `9600 8N1`: no useful printable response

Board `2D0A1209DABC240B`:
- `19200 8N1`: no data
- `115200 8N1`: no data
- `921600 8N1`: no data
- `9600 8N1`: no data

Interpretation:
- Board `2E2C...` is alive on the old stock Murata AT path but still has the known broken/stale firmware behavior.
- Board `2D0A...` did not show a live AT endpoint on `/dev/ttymxc3` during this run.

---

## 7. Tool Availability

Arduino CLI:
- Installed/resolved at `C:\Users\dorkm\AppData\Local\Programs\ArduinoCLI\arduino-cli.exe`
- Version: `1.4.1`

STM32 UART flash tools:
- `stm32flash` / `stm32flash.exe`: not found on host PATH
- No `stm32flash` or `stm32loader` found on either checked X8 Linux image

---

## 8. Current Blocker

The immediate blocker is not L072 firmware build quality. It is the lack of a working end-to-end flash/monitor transport:

1. Windows COM endpoints on Portenta X8 are not usable as direct M7 bridge monitors in this state.
2. X8 `SerialRPC` through `m4_proxy` is reachable but not yielding bridge text or command-response traffic.
3. `stm32flash` is not installed on the Windows host or X8 Linux side.
4. One board still responds like stale Murata AT firmware; the other is silent on the known `/dev/ttymxc3` path.

Planned bypass: use the Max Carrier debug Micro-USB / STLink virtual COM port, which should enumerate as a separate Windows COM port wired to M7 `Serial1`. The bridge now uses `Serial1` as the X8 PC side by default.

Update after connecting both Max Carrier Micro-USB debug ports through a USB hub:
- Windows sees two present `J-Link` devices, not STLink devices:
  - `USB\VID_1366&PID_0101\001078222309`
  - `USB\VID_1366&PID_0101\001078180658`
- Both report `CM_PROB_FAILED_INSTALL`, so the debug probe/VCOM driver is not bound.
- No new active COM ports appeared beyond COM11/COM12 from the X8 USB-C interfaces.
- Winget found package `NordicSemiconductor.JLink` version `8.18.0`, but silent install did not complete; the installer was downloaded to `C:\Users\dorkm\AppData\Local\Temp\JLink_Windows.exe` and likely needs elevated/manual installation.

Update after manual SEGGER installation:
- Both J-Link devices bind successfully as `J-Link driver` with `CM_PROB_NONE`.
- Installed tools were found under `C:\Program Files\SEGGER\JLink_V942`.
- J-Link Commander identifies the probes as `J-Link-OB-STM32F4405-Arduino` with serial numbers `1078222309` and `1078180658`.
- Even after driver installation, no J-Link virtual COM ports appear under Windows `Win32_SerialPort` or the PnP `Ports` class.
- Attempts to open a selected probe with J-Link Commander trigger/stall during the probe firmware/capability-read path. Directly connecting a single Micro-USB debug cable leaves one healthy `J-Link driver` USB device present, but still no VCOM port and Commander still stalls at `Connecting to J-Link via USB...`.
- Conclusion: the Max Carrier Micro-USB debug interface is currently usable only as a J-Link USB debug probe from Windows, not as a proven `Serial1` VCOM transport for the M7 bridge.

Update after retrying USB with the hub disconnected:
- The bridge sketch was confirmed to already use `Serial1` as the PC-side serial path on `ARDUINO_PORTENTA_X8`; a blind `Serial.` to `Serial1.` replacement would be incorrect because `Serial2` is still the L072 UART.
- The current X8 bridge sketch compiled for `arduino:mbed_portenta:portenta_x8` and was reuploaded successfully to both USB-C targets after setting `ANDROID_SERIAL` to disambiguate the two connected ADB devices:
  - COM11 / `2E2C1209DABC240B`
  - COM12 / `2D0A1209DABC240B`
- With the hub disconnected, Windows no longer enumerated a `VID_1366` J-Link probe. Instead, the Micro-USB debug connection appeared as `Unknown USB Device (Device Descriptor Request Failed)`, `USB\VID_0000&PID_0002\6&233BB915&0&1`, problem code 43 / `CM_PROB_FAILED_POST_START`.
- Reseating the Micro-USB cable, trying the other Max Carrier debug port, power-cycling one carrier, and reconnecting one debug cable through the hub all continued to produce the same descriptor failure.
- Because the probe never reaches `VID_1366` enumeration in this state, `JLink.exe` cannot connect and `vcom enable` cannot be applied.

Update after enabling the Max Carrier debugger DIP switch:
- The debugger USB problem is no longer the active blocker. Both debug probes now enumerate as healthy `VID_1366&PID_0105` composite devices with J-Link CDC UART ports:
  - COM13 -> J-Link serial `1078222309` / `001078222309`
  - COM8 -> J-Link serial `1078180658` / `001078180658`
- The passthrough bridge was uploaded successfully to the currently visible X8 USB-C target COM12 / `2D0A1209DABC240B` using `ANDROID_SERIAL` to disambiguate ADB.
- Passive/command probes on COM8 and COM13 at 115200 8N1 produced no bridge banner or response to `!help` / `!ping`.
- Project routing docs and pinmap evidence now override the earlier assumption: `Serial1` is the X8<->H747 link, not the Method G/Max Carrier debug route.
- The route-probe sketch was changed to emit labeled `LT_ROUTE_PROBE` lines on `Serial1`, `Serial2`, and `Serial3` at 115200 8N1, then uploaded to COM12 / `2D0A1209DABC240B`.
- COM8 and COM13 captured no route-probe bytes with DTR/RTS deasserted or asserted.

Update after schematic-guided SERIAL0 probe expansion:
- The first SERIAL0 attempt used `Serial.begin(115200)` and `Serial.println(...)` in the route-probe sketch, but X8 compile failed by design: `ErrorSerialClass::begin` requires including `SerialRPC` for `Serial` on this core, so `Serial` is not a direct hardware UART handle in this configuration.
- The route-probe sketch was updated instead to instantiate explicit X8 UART candidates and emit labeled markers on each:
  - `SERIAL0_PA2PA3` via `arduino::UART _UART0_(PA_2, PA_3, NC, NC)`
  - `SERIAL0_PD5PD6` via `arduino::UART _UART0_ALT_(PD_5, PD_6, PD_4, PD_3)`
  - existing `Serial1`, `Serial2`, `Serial3` probes remained active
- The expanded probe compiled and uploaded successfully to COM12 / `2D0A1209DABC240B`.
- Capture on COM8 and COM13 at 115200 8N1 still showed no route-probe bytes with DTR/RTS both deasserted and asserted.

Update after SEGGER RTT integration attempt:
- RTT support was integrated directly into `x8_uart_route_probe.ino` using local SEGGER sources (`SEGGER_RTT.c/.h`, `SEGGER_RTT_printf.c`, `SEGGER_RTT_Conf.h`) copied into the sketch root so Arduino CLI links them deterministically.
- The probe now emits periodic `LT_RTT_PROBE ...` messages with `SEGGER_RTT_printf()` while still emitting UART route markers.
- RTT-enabled sketch compile for `arduino:mbed_portenta:portenta_x8` is PASS.
- Flash path blockers prevented getting the RTT firmware onto target in this state:
  - X8 upload path (`arduino:mbed_portenta:portenta_x8`) failed because ADB target `2D0A1209DABC240B` was not present (`adb: ... device ... not found`).
  - Direct M7 fallback (`arduino:mbed_portenta:envie_m7` on COM12) failed with `No DFU capable USB device available`.
  - J-Link SWD flash attempt (`STM32H747XI_M7`, USB serial `1078222309`) repeatedly failed to attach target/DAP (`InitTarget() ... error code -1`).
- RTT tool checks in current bench state:
  - `JLinkRTTLogger` on probe `1078222309` can reach the J-Link probe but reports `RTT Control Block not found`.
  - `JLinkRTTLogger` on probe `1078180658` reports `Could not connect to target`.

Current interpretation:
- J-Link CDC enumeration is solved.
- The J-Link CDC ports are not yet proven to be wired to the tested M7 UART candidates (`SERIAL0_PA2PA3`, `SERIAL0_PD5PD6`, `Serial1`, `Serial2`, `Serial3`) at 115200 8N1.
- RTT firmware plumbing is now implemented and build-valid, but runtime RTT visibility is blocked by target flashing/attach transport failures, not by missing code support.
- If these ports expose the Max Carrier `UART_SNIFF` feature, they may require a carrier-side mux/DIP setting or exact schematic mapping to select the desired sniff channel.
- If they are passive sniffers, writes such as `!help` may never reach M7; the route-probe transmit test is the right next diagnostic pattern.

---

## 9. Recommended Next Step

Fastest path to unblock Phase 1 now that J-Link CDC enumerates:

1. Keep the route-probe sketch loaded while adjusting any Max Carrier debugger/UART switch or jumper settings; monitor COM8 and COM13 at 115200 for `LT_ROUTE_PROBE serial=<SerialN>` lines after each hardware change.
2. Get the ABX00043 Max Carrier schematic page or board documentation that maps `UART_SNIFF1..5` to H7 HD-connector pins and J-Link CDC channels.
3. Restore one working flash transport to get the RTT-enabled sketch on target:
  - preferred: recover X8 ADB visibility for `2D0A1209DABC240B`, or
  - alternate: put board in DFU-capable state for `envie_m7`, or
  - alternate: resolve J-Link SWD attach (`InitTarget`/DAP failures) on the intended probe.
4. Once flashed, open `JLinkRTTLogger` or RTT Viewer on channel 0 and confirm `LT_RTT_PROBE` lines before further UART-route work.
5. Treat COM8/COM13 as passive or unmapped until proven otherwise. Do not rely on `!help`/`!ping` writes reaching M7 through those ports yet.
6. If no carrier-side mux route exposes the desired UART, use an external USB-to-UART adapter or logic analyzer on the selected `Serial2` pins (`PA_15`/`PF_6`, optional RTS/CTS `PF_8`/`PF_9`) to prove traffic.
7. After the host/debug UART is proven, restore the passthrough bridge sketch and set its PC-side serial to the proven route. Keep the L072 side on `Serial2` unless schematic evidence proves another Method G route.
8. Install `stm32flash` on Windows, or provide its path, so the BOOT0/reset flash workflow can be executed once a working UART transport is proven.
9. Keep `/dev/ttymxc3` AT probing as a fallback diagnostic only; it proves stock/stale modem state but does not flash or validate the custom Method G L072 firmware unless BOOT0/reset control is also solved from Linux or an external adapter.
