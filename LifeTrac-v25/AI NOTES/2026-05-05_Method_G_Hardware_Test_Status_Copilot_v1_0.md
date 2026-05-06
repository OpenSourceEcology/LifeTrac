# Method G Hardware Test Status (Copilot v1.0)

Date: 2026-05-05
Status: In progress; firmware updates and board flashing succeeded, but live Method G Murata route is still blocked by UART/core routing mismatch.
Scope: Current bench status for two connected Portenta X8 + Max Carrier boards while validating Method G host runtime (including VER request/response path).

## 1. Bench inventory and targets

- Workspace target: LifeTrac-v25 Method G host runtime on `tractor_h7`.
- Connected boards:
  - Board A: COM11, ADB 2D0A1209DABC240B
  - Board B: COM12, ADB 2E2C1209DABC240B
- Arduino core in use for X8 builds: `arduino:mbed_portenta` 4.5.0.

## 2. What was completed in this run

1. Implemented Method G version-reporting path on H7 host runtime:
   - VER request send helper added.
   - VER_URC parser/storage added to runtime health.
   - Runtime now periodically requests VER and logs decoded VER line.
   - Host vectors updated and passing.

2. Built and validated host-side tests:
   - `mh_runtime_health_vectors` pass (6 vectors).
   - Loopback driver still compiles.

3. Built Method G X8 firmware artifact with version-reporting changes:
   - Build path: `C:/Users/dorkm/AppData/Local/Temp/lifetrac_methodg_ver_hwtest`
   - Artifact example: `tractor_h7.ino.elf`.

4. Flashed both connected X8 boards successfully:
   - Upload to COM11: exit 0
   - Upload to COM12: exit 0

## 3. Runtime evidence captured after flashing

1. USB CDC serial visibility:
   - COM11/COM12 at 115200 and 921600 both produced 0 bytes during passive capture windows.

2. ADB board visibility:
   - Both boards visible and online through local adb tool.

3. `m4_proxy` status:
   - Running on both boards and listening on TCP 5001.
   - Local forwarded sockets connected, but passive reads returned 0 bytes.

4. Linux-side Murata UART probes (`/dev/ttymxc3`) with elevated privileges:
   - Passive byte count: 0 at 19200 and 921600 on both boards.
   - AT probe commands (`AT`, `AT+VER?`) returned no bytes on both boards during this run.

## 4. Routing test and hard blocker found

A direct test compile with Method G on `LIFETRAC_MH_SERIAL=Serial2` was attempted to move off the compile-token route.

Result:
- Link failure: undefined reference to `_UART2_` from `tractor_h7.ino`.
- This indicates the currently installed X8 core/config exposed to this build does not provide the required Serial2 object for Method G routing on this workstation state.

Observed implication:
- `Serial1` builds and flashes, but does not provide observable Method G Murata traffic for this bench setup.
- `Serial2` is not linkable in current core state.

## 5. Current assessment

- Firmware change set for VER request/response handling is implemented and test-validated.
- Hardware flashing path to both boards is proven.
- Live host<->Murata route validation remains blocked by UART route/core mismatch, not by the Method G runtime code changes.

## 6. Immediate next steps

1. Discover/install/select the X8 core variant/config that exposes the real Method G UART route expected by bench docs.
2. Rebuild Method G with the physically valid `LIFETRAC_MH_SERIAL` value.
3. Reflash both boards with that build.
4. Re-run runtime capture for BOOT/VER evidence and archive logs under bench evidence.

## 7. Gate impact (as of this note)

- Gate 1 (compile matrix): closed for current stock X8 compile-token cases.
- Gate 2 (host/runtime software correctness): closed for current implemented scope.
- Gate 3 (physical route/flash proof): still open until live Method G route is observed on actual hardware path.
