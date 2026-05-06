# Method G Hardware Test Status (Copilot v1.7)

Date: 2026-05-05
Status: SerialRPC endpoint diagnosis validated and corrected. The added diagnosis helps explain why `/dev/ttyX0` and ordinary Linux tty reads stayed silent, but the live board/core evidence points to `m4_proxy`/msgpack-rpc TCP routing rather than a directly readable `/dev/ttyRPMSG*` node.
Scope: Delta update after v1.6 focused on validating the SerialRPC/OpenAMP/m4_proxy hypothesis.

## 1. What the new diagnosis gets right

The diagnosis is useful because it correctly challenges the old assumption that `SerialRPC` output should appear on `/dev/ttyX0`, `/dev/ttyGS0`, or `/dev/ttymxc*`.

Verified from the installed Arduino core:
- `SerialRPC.write()` does not write to a UART-like Linux character device.
- It calls `RPC.send("tty", tx_buffer)`.
- `RPC` uses OpenAMP/RPMsg endpoints named `raw` and `rpc`.

Therefore, the previous zero-byte reads on ordinary tty devices are not strong evidence that the heartbeat was never generated. They are strong evidence that those nodes are the wrong observation surface for `SerialRPC`.

## 2. Important correction

The appended external diagnosis states that our firmware flashes to M7 and then depends on M4 to broker the Linux connection.
That is not correct for the build path used in this test.

Observed build/upload facts:
- The X8 FQBN build used Cortex-M4 flags (`-mcpu=cortex-m4`).
- The upload pushed `tractor_h7.ino.elf` as `/tmp/arduino/m4-user-sketch.elf`.

So, for `arduino:mbed_portenta:portenta_x8`, the heartbeat test image is an M4 user sketch talking to Linux through OpenAMP/RPMsg. This is not an M7 -> M4 -> Linux chain in the current tested configuration.

## 3. Live board evidence

Board A: `2D0A1209DABC240B`

### 3.1 RPMsg character devices

Checked:
- `/dev/ttyRPMSG*`
- `/dev/rpmsg*`
- `/sys/bus/rpmsg/devices`

Observed:
- No `/dev/ttyRPMSG*` nodes.
- No `/dev/rpmsg*` nodes.
- `/sys/bus/rpmsg/devices` exists but is empty in the tested state.

Interpretation:
- The immediate next step is not simply `cat /dev/ttyRPMSG0`; that node is not present on this image/state.

### 3.2 m4_proxy status

Observed:
- `/usr/bin/m4_proxy` is running.
- `m4-proxy.service` is loaded and active.
- Service definition: `/usr/lib/systemd/system/m4-proxy.service`
- Service dependency: `stm32h7-program.service`

This corrects another part of the diagnosis: the proxy is not missing or inactive on board A.

### 3.3 m4_proxy exposed interfaces

Observed listening sockets:
- TCP `0.0.0.0:5001`
- TCP `:::5000`

Embedded strings in `/usr/bin/m4_proxy` indicate:
- Go binary using `github.com/msgpack-rpc/msgpack-rpc-go/proxy`
- RPC server/listener behavior
- The string `Registering service on port`

Passive bounded reads against ports 5000 and 5001 with `nc` produced no bytes.

Interpretation:
- m4_proxy is likely not a raw byte stream or tty bridge.
- It likely requires a msgpack-rpc client/protocol interaction to observe `SerialRPC` notifications such as `tty`.

## 4. Updated conclusion

Yes, the added diagnosis helps solve the issue, but only after correction:

1. It correctly explains why tty endpoint sweeps stayed silent.
2. It incorrectly predicts `/dev/ttyRPMSG*` availability on this image.
3. It incorrectly describes the current tested firmware as M7-routed; the X8 FQBN is producing an M4 user sketch here.
4. The actual next software-observability target is m4_proxy's msgpack-rpc TCP interface, not ordinary tty reads.

## 5. Recommended next step

Choose one of two paths:

1. Protocol path: build a small msgpack-rpc client/probe for m4_proxy ports 5000/5001 and subscribe/call the service path that receives `SerialRPC` `tty` notifications.
2. Fast bench path: stop spending time on SerialRPC observability and move to physical UART proof (`UART_SNIFF` / logic analyzer), because the proxy layer is now confirmed to be non-trivial and not a simple character-device read.

For rapid Method G hardware unblocking, the fast bench path is still recommended unless we specifically need to characterize the X8 SerialRPC stack.
