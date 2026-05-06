# M4 Proxy Msgpack-RPC Probe Results (Copilot v1.0)

Date: 2026-05-06
Status: Python probe implemented and tested. The software-only observability path is not confirmed; passive msgpack-rpc capture received zero bytes, and discovery requests can restart `m4_proxy` on this image.
Scope: Follow-up to `2026-05-06_Python_Msgpack_RPC_Plan.md`.

## 1. Implemented tool

Added:
- `LifeTrac-v25/tools/m4_proxy_msgpack_probe.py`

Design choices:
- Dependency-free Python; no `mprpc`, `msgpack`, or `pip` required.
- Implements minimal MessagePack encode/decode for msgpack-rpc frames.
- Supports passive listening, optional discovery requests, optional `tty` injection, and heartbeat text matching.

## 2. Tested host-side path

ADB forwarding:
- `tcp:15000` -> board `tcp:5000`
- `tcp:15001` -> board `tcp:5001`

Command family:
- `python LifeTrac-v25/tools/m4_proxy_msgpack_probe.py --ports 15000,15001 --timeout 12 --expect-text MH_SOFT_BRIDGE_HB`

Observed result:
- Connected to both forwarded ports.
- `raw_rx=0` on both ports.
- `decoded_rx=0` on both ports.
- `MH_SOFT_BRIDGE_HB` not found.

Interpretation:
- Keeping a client connected to the m4_proxy TCP listeners is not enough to receive SerialRPC heartbeat traffic as a passive stream.

## 3. Discovery-mode result

Command family:
- `python LifeTrac-v25/tools/m4_proxy_msgpack_probe.py --ports 15000,15001 --timeout 8 --discover`

Observed result:
- The probe sent standard msgpack-rpc requests such as `system.listMethods`, `list`, `services`, and `ping`.
- Both peers closed without returning decoded responses.
- Privileged journal showed `m4_proxy` restarted after `system.listMethods` with:
  - `panic: reflect: call of reflect.Value.Type on zero Value`

Earlier logs also showed m4_proxy panics from malformed or unexpected one-byte input:
- `panic: runtime error: index out of range [1] with length 1`

Interpretation:
- Discovery mode is unsafe on this production image unless the exact valid method set is known.
- m4_proxy does not behave like a robust generic msgpack-rpc introspection server.

## 4. On-device Python result

Checked board A:
- `python3` exists: `/usr/bin/python3`
- Version: `Python 3.10.4`

Pushed the probe to:
- `/home/fio/m4_proxy_msgpack_probe.py`

Observed result when running on-device:
- `ModuleNotFoundError: No module named 'socket'`

Interpretation:
- The X8 rootfs Python is stripped and cannot run even a stdlib socket client in the current state.
- The practical Python path is host-side execution through `adb forward`, not on-device execution, unless a full Python stdlib is installed.

## 5. Current conclusion

The Python script is useful as a controlled probe, but it did not prove the SerialRPC heartbeat path.

What it did prove:
1. m4_proxy TCP ports accept connections through ADB forwarding.
2. Passive client attachment yields zero bytes during the heartbeat window.
3. Generic msgpack-rpc discovery can crash/restart m4_proxy.
4. On-device Python is currently too stripped for socket-based probing.

## 6. Recommended next step

Do not spend more bench time guessing m4_proxy methods from the outside.

Recommended path for Method G hardware unblocking:
1. Use `UART_SNIFF` / logic analyzer for direct physical proof of Serial2/LoRa traffic.
2. Keep `m4_proxy_msgpack_probe.py` as a diagnostic tool, but only use `--discover` when the exact m4_proxy API is known or on a disposable test image.
3. If software observability remains required, obtain or rebuild `m4_proxy` source with logging/instrumentation rather than probing opaque production sockets.
