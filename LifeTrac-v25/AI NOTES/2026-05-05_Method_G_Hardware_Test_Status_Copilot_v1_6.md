# Method G Hardware Test Status (Copilot v1.6)

Date: 2026-05-05
Status: Heartbeat-enabled deterministic observability tests completed on both X8 boards. Firmware build and flash succeeded, but all tested Linux serial endpoints remained zero-byte during bounded capture windows.
Scope: Delta update after v1.5 focused on route-proof heartbeat validation.

## 1. Delta from v1.5

v1.5 identified endpoint-route uncertainty and recommended a compile-gated heartbeat beacon.
This pass executes that recommendation end-to-end.

## 2. Heartbeat test objective

Goal:
- Prove whether the Linux-visible serial path is carrying any bridge debug output at all, independent of Murata traffic.

Method:
- Enable software-bridge heartbeat print in firmware using compile flag:
  - `-DLIFETRAC_MH_SOFT_BRIDGE_HEARTBEAT_MS=1000`
- Flash to board A and board B.
- Perform bounded privileged capture and endpoint sweep.

## 3. Execution summary

Boards under test:
- Board A: `2D0A1209DABC240B` (`COM11`)
- Board B: `2E2C1209DABC240B` (`COM12`)

### 3.1 Heartbeat build result
- Build directory: `%TEMP%/lifetrac_methodg_softbridge_serial2_hb`
- Artifact confirmed:
  - `tractor_h7.ino.elf`
  - size: `6,952,488` bytes

### 3.2 Heartbeat flash result
- Upload used explicit target disambiguation:
  - `ANDROID_SERIAL=2D0A1209DABC240B`
  - `ANDROID_SERIAL=2E2C1209DABC240B`
- Push/upload completed successfully for heartbeat artifact on both boards.

### 3.3 Primary endpoint capture (`/dev/ttyX0`)
Capture method:
- Privileged bounded capture (`sudo` + `timeout`) with binary save, hex preview, and printable string extraction.

Observed result:
- `BYTE_COUNT:0`
- `STRINGS:` empty

Interpretation:
- No heartbeat banner or any other bytes were observed on `/dev/ttyX0` in this run.

### 3.4 Heartbeat-era endpoint sweep (board A)
Bounded privileged reads (`timeout 5 cat | wc -c`) on:
- `/dev/ttyX0` -> `0`
- `/dev/ttyGS0` -> `0`
- `/dev/ttymxc1` -> `0`
- `/dev/ttymxc2` -> `0`
- `/dev/ttymxc3` -> `0`

### 3.5 Heartbeat-era endpoint sweep (board B)
Bounded privileged reads (`timeout 5 cat | wc -c`) on:
- `/dev/ttyX0` -> `0`
- `/dev/ttyGS0` -> `0`
- `/dev/ttymxc1` -> `0`
- `/dev/ttymxc2` -> `0`
- `/dev/ttymxc3` -> `0`

## 4. Current interpretation

Closed in this pass:
1. Deterministic beacon strategy implemented and exercised on hardware.
2. Build and flash workflow for heartbeat image validated.
3. Endpoint sweeps repeated on both boards under known periodic-output firmware.

Still open:
1. Any positive byte-level evidence on tested Linux serial nodes.
2. Definitive identification of the Linux endpoint actually bridged to M4 debug output in this image/runtime configuration.

## 5. Updated blocker statement

The blocker is now strongly narrowed to endpoint routing/visibility, not payload timing and not permission path execution.
Even with deterministic 1 Hz heartbeat enabled in software-bridge mode, `/dev/ttyX0`, `/dev/ttyGS0`, and `/dev/ttymxc1..3` remained silent in bounded privileged captures on both boards.

## 6. Recommended next step

High-confidence next actions:
1. Enumerate and test additional candidate nodes in this image (for example `ttyAMA*`, `ttyRPMSG*`, and any board-specific virtual UART aliases exposed at runtime).
2. If still silent, proceed to physical route proof (`UART_SNIFF` / logic analyzer) as primary evidence to map actual bridge path.

Operational note:
- Continue using bounded capture windows and explicit board targeting (`ANDROID_SERIAL`) for reproducible evidence collection.

## 7. Diagnosis of Endpoint Errors (Gemini 3.1 Pro)

The current blocker where `/dev/ttyX0`, `/dev/ttyGS0`, and `/dev/ttymxc1..3` are entirely silent during the heartbeat beacon test is highly likely due to the internal architecture of the Portenta X8's Remote Procedure Call (RPC) link:

1. **`SerialRPC` Maps to `ttyRPMSG`**: On the Portenta X8, `SerialRPC` uses the OpenAMP framework to pass messages between the STM32H7 and the i.MX8 Linux side via the shared memory buffer. In Linux, these OpenAMP channels present themselves as `/dev/ttyRPMSG0`, `/dev/ttyRPMSG1`, etc., **not** `ttyX0` or `ttyGS0`.
2. **`m4_proxy` Dependency**: The `SerialRPC` library expects a daemon on the Linux side (typically `m4_proxy` or an `mtty` service) to actively bind the RPMSG channel to a pseudo-terminal (pty). If this service is inactive or misconfigured, the kernel will not surface the bytes to any standard `/dev/tty*` file.
3. **M7 vs M4 Core Routing**: By default, the firmware flashes to the M7 core of the STM32H747. However, the OpenAMP physical link to the Linux i.MX8 is often governed by the M4 core on the Portenta X8. If the M4 core is not running the forwarder firmware, the M7 `SerialRPC` calls simply buffer out into the void.

**Immediate Resolution Steps:**
- Probe `/dev/ttyRPMSG0` and `/dev/ttyRPMSG1` using `sudo cat /dev/ttyRPMSG0`.
- Verify the OpenAMP/RPC daemon is running on Linux: `adb shell "sudo systemctl status m4_proxy"` (or similar).
- If `ttyRPMSG*` does not exist or yields 0 bytes, abandon `SerialRPC` and move directly to a physical logic analyzer / `UART_SNIFF` tap. The Portenta X8 software bridges are too opaque for rapid unblocking at this tier.
