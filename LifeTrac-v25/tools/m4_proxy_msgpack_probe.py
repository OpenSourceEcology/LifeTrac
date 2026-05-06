#!/usr/bin/env python3
"""Probe Portenta X8 m4_proxy msgpack-rpc traffic.

The Portenta X8 Arduino ``SerialRPC`` class does not expose a normal Linux
tty. In Arduino core 4.5.0, ``SerialRPC.write()`` sends a msgpack-rpc
notification named ``tty`` through the OpenAMP/RPMsg RPC path. The Linux
``m4_proxy`` process exposes TCP listeners, usually on ports 5000 and 5001.

This tool is intentionally dependency-free. It implements just enough
MessagePack and msgpack-rpc framing to:

* keep a TCP client connected while heartbeat traffic is expected;
* decode inbound request/response/notification frames;
* optionally send harmless discovery requests; and
* optionally inject bytes through the ``tty`` notification for loopback tests.

Typical host-side use with ADB forwarding:

    adb -s 2D0A1209DABC240B forward tcp:15000 tcp:5000
    adb -s 2D0A1209DABC240B forward tcp:15001 tcp:5001
    python LifeTrac-v25/tools/m4_proxy_msgpack_probe.py --ports 15000,15001 \
        --timeout 12 --discover --expect-text MH_SOFT_BRIDGE_HB

Typical on-device use:

    python3 /home/fio/m4_proxy_msgpack_probe.py --host 127.0.0.1 \
        --ports 5000,5001 --timeout 12 --discover
"""

from __future__ import annotations

import argparse
import binascii
import struct
import sys
import time
from dataclasses import dataclass, field
from typing import Any

try:
    import socket
except ModuleNotFoundError:  # pragma: no cover - depends on stripped target rootfs
    socket = None


DEFAULT_DISCOVERY_METHODS = (
    "system.listMethods",
    "system.describe",
    "system.methodHelp",
    "list",
    "help",
    "services",
    "version",
    "ping",
)


class NeedMore(Exception):
    """Raised when a partial MessagePack object needs more bytes."""


class MsgpackError(Exception):
    """Raised when unsupported or malformed MessagePack is encountered."""


def _pack_int(value: int) -> bytes:
    if 0 <= value <= 0x7F:
        return bytes([value])
    if -32 <= value < 0:
        return bytes([0x100 + value])
    if 0 <= value <= 0xFF:
        return b"\xcc" + struct.pack(">B", value)
    if 0 <= value <= 0xFFFF:
        return b"\xcd" + struct.pack(">H", value)
    if 0 <= value <= 0xFFFFFFFF:
        return b"\xce" + struct.pack(">I", value)
    if 0 <= value <= 0xFFFFFFFFFFFFFFFF:
        return b"\xcf" + struct.pack(">Q", value)
    if -0x80 <= value < 0:
        return b"\xd0" + struct.pack(">b", value)
    if -0x8000 <= value < 0:
        return b"\xd1" + struct.pack(">h", value)
    if -0x80000000 <= value < 0:
        return b"\xd2" + struct.pack(">i", value)
    return b"\xd3" + struct.pack(">q", value)


def _pack_str(value: str) -> bytes:
    data = value.encode("utf-8")
    size = len(data)
    if size <= 31:
        return bytes([0xA0 | size]) + data
    if size <= 0xFF:
        return b"\xd9" + struct.pack(">B", size) + data
    if size <= 0xFFFF:
        return b"\xda" + struct.pack(">H", size) + data
    return b"\xdb" + struct.pack(">I", size) + data


def _pack_bytes(value: bytes) -> bytes:
    size = len(value)
    if size <= 0xFF:
        return b"\xc4" + struct.pack(">B", size) + value
    if size <= 0xFFFF:
        return b"\xc5" + struct.pack(">H", size) + value
    return b"\xc6" + struct.pack(">I", size) + value


def pack_msgpack(value: Any) -> bytes:
    if value is None:
        return b"\xc0"
    if value is False:
        return b"\xc2"
    if value is True:
        return b"\xc3"
    if isinstance(value, int):
        return _pack_int(value)
    if isinstance(value, bytes):
        return _pack_bytes(value)
    if isinstance(value, bytearray):
        return _pack_bytes(bytes(value))
    if isinstance(value, str):
        return _pack_str(value)
    if isinstance(value, (list, tuple)):
        size = len(value)
        if size <= 15:
            prefix = bytes([0x90 | size])
        elif size <= 0xFFFF:
            prefix = b"\xdc" + struct.pack(">H", size)
        else:
            prefix = b"\xdd" + struct.pack(">I", size)
        return prefix + b"".join(pack_msgpack(item) for item in value)
    if isinstance(value, dict):
        size = len(value)
        if size <= 15:
            prefix = bytes([0x80 | size])
        elif size <= 0xFFFF:
            prefix = b"\xde" + struct.pack(">H", size)
        else:
            prefix = b"\xdf" + struct.pack(">I", size)
        chunks = [prefix]
        for key, item in value.items():
            chunks.append(pack_msgpack(key))
            chunks.append(pack_msgpack(item))
        return b"".join(chunks)
    raise TypeError(f"unsupported MessagePack value: {type(value)!r}")


def _need(data: bytearray, offset: int, size: int) -> bytes:
    if offset + size > len(data):
        raise NeedMore
    return bytes(data[offset : offset + size])


def _decode_at(data: bytearray, offset: int = 0) -> tuple[Any, int]:
    first = _need(data, offset, 1)[0]
    offset += 1

    if first <= 0x7F:
        return first, offset
    if first >= 0xE0:
        return first - 0x100, offset
    if 0x80 <= first <= 0x8F:
        return _decode_map(data, offset, first & 0x0F)
    if 0x90 <= first <= 0x9F:
        return _decode_array(data, offset, first & 0x0F)
    if 0xA0 <= first <= 0xBF:
        return _decode_str(data, offset, first & 0x1F)

    if first == 0xC0:
        return None, offset
    if first == 0xC2:
        return False, offset
    if first == 0xC3:
        return True, offset
    if first == 0xC4:
        size = struct.unpack(">B", _need(data, offset, 1))[0]
        offset += 1
        return _decode_bytes(data, offset, size)
    if first == 0xC5:
        size = struct.unpack(">H", _need(data, offset, 2))[0]
        offset += 2
        return _decode_bytes(data, offset, size)
    if first == 0xC6:
        size = struct.unpack(">I", _need(data, offset, 4))[0]
        offset += 4
        return _decode_bytes(data, offset, size)
    if first == 0xCA:
        return struct.unpack(">f", _need(data, offset, 4))[0], offset + 4
    if first == 0xCB:
        return struct.unpack(">d", _need(data, offset, 8))[0], offset + 8
    if first == 0xCC:
        return struct.unpack(">B", _need(data, offset, 1))[0], offset + 1
    if first == 0xCD:
        return struct.unpack(">H", _need(data, offset, 2))[0], offset + 2
    if first == 0xCE:
        return struct.unpack(">I", _need(data, offset, 4))[0], offset + 4
    if first == 0xCF:
        return struct.unpack(">Q", _need(data, offset, 8))[0], offset + 8
    if first == 0xD0:
        return struct.unpack(">b", _need(data, offset, 1))[0], offset + 1
    if first == 0xD1:
        return struct.unpack(">h", _need(data, offset, 2))[0], offset + 2
    if first == 0xD2:
        return struct.unpack(">i", _need(data, offset, 4))[0], offset + 4
    if first == 0xD3:
        return struct.unpack(">q", _need(data, offset, 8))[0], offset + 8
    if first == 0xD9:
        size = struct.unpack(">B", _need(data, offset, 1))[0]
        offset += 1
        return _decode_str(data, offset, size)
    if first == 0xDA:
        size = struct.unpack(">H", _need(data, offset, 2))[0]
        offset += 2
        return _decode_str(data, offset, size)
    if first == 0xDB:
        size = struct.unpack(">I", _need(data, offset, 4))[0]
        offset += 4
        return _decode_str(data, offset, size)
    if first == 0xDC:
        size = struct.unpack(">H", _need(data, offset, 2))[0]
        offset += 2
        return _decode_array(data, offset, size)
    if first == 0xDD:
        size = struct.unpack(">I", _need(data, offset, 4))[0]
        offset += 4
        return _decode_array(data, offset, size)
    if first == 0xDE:
        size = struct.unpack(">H", _need(data, offset, 2))[0]
        offset += 2
        return _decode_map(data, offset, size)
    if first == 0xDF:
        size = struct.unpack(">I", _need(data, offset, 4))[0]
        offset += 4
        return _decode_map(data, offset, size)

    raise MsgpackError(f"unsupported MessagePack prefix 0x{first:02x}")


def _decode_bytes(data: bytearray, offset: int, size: int) -> tuple[bytes, int]:
    return _need(data, offset, size), offset + size


def _decode_str(data: bytearray, offset: int, size: int) -> tuple[str, int]:
    raw = _need(data, offset, size)
    return raw.decode("utf-8", errors="replace"), offset + size


def _decode_array(data: bytearray, offset: int, size: int) -> tuple[list[Any], int]:
    items = []
    for _ in range(size):
        item, offset = _decode_at(data, offset)
        items.append(item)
    return items, offset


def _decode_map(data: bytearray, offset: int, size: int) -> tuple[dict[Any, Any], int]:
    items: dict[Any, Any] = {}
    for _ in range(size):
        key, offset = _decode_at(data, offset)
        value, offset = _decode_at(data, offset)
        try:
            items[key] = value
        except TypeError:
            items[repr(key)] = value
    return items, offset


class StreamDecoder:
    def __init__(self) -> None:
        self.buffer = bytearray()

    def feed(self, chunk: bytes) -> list[Any]:
        self.buffer.extend(chunk)
        objects = []
        while self.buffer:
            try:
                obj, used = _decode_at(self.buffer, 0)
            except NeedMore:
                break
            objects.append(obj)
            del self.buffer[:used]
        return objects


@dataclass
class PortState:
    port: int
    sock: socket.socket
    decoder: StreamDecoder = field(default_factory=StreamDecoder)
    raw_rx: int = 0
    decoded_rx: int = 0


def printable(data: bytes) -> str:
    return "".join(chr(byte) if 32 <= byte <= 126 or byte in (9, 10, 13) else "." for byte in data)


def render(value: Any, limit: int = 160) -> str:
    if isinstance(value, bytes):
        hex_text = binascii.hexlify(value[:64]).decode("ascii")
        suffix = "..." if len(value) > 64 else ""
        return f"bytes[{len(value)}] ascii={printable(value)!r} hex={hex_text}{suffix}"
    text = repr(value)
    if len(text) > limit:
        return text[: limit - 3] + "..."
    return text


def coerce_payload_bytes(value: Any) -> bytes | None:
    if isinstance(value, bytes):
        return value
    if isinstance(value, bytearray):
        return bytes(value)
    if isinstance(value, str):
        return value.encode("utf-8", errors="replace")
    if isinstance(value, list) and all(isinstance(item, int) and 0 <= item <= 255 for item in value):
        return bytes(value)
    return None


def iter_payloads(value: Any):
    payload = coerce_payload_bytes(value)
    if payload is not None:
        yield payload
        return
    if isinstance(value, (list, tuple)):
        for item in value:
            yield from iter_payloads(item)
    elif isinstance(value, dict):
        for item in value.values():
            yield from iter_payloads(item)


def describe_rpc_object(port: int, obj: Any) -> list[str]:
    lines = [f"[{port}] RX {render(obj)}"]
    if isinstance(obj, list) and len(obj) >= 3 and isinstance(obj[0], int):
        msg_type = obj[0]
        if msg_type == 0 and len(obj) >= 4:
            lines.append(f"[{port}] RX request id={obj[1]!r} method={obj[2]!r}")
        elif msg_type == 1 and len(obj) >= 4:
            lines.append(f"[{port}] RX response id={obj[1]!r} error={render(obj[2])} result={render(obj[3])}")
        elif msg_type == 2 and len(obj) >= 3:
            lines.append(f"[{port}] RX notify method={obj[1]!r}")
            if obj[1] == "tty":
                for payload in iter_payloads(obj[2]):
                    lines.append(f"[{port}] RX tty payload ascii={printable(payload)!r} hex={binascii.hexlify(payload).decode('ascii')}")
    return lines


def parse_ports(text: str) -> list[int]:
    ports = []
    for part in text.split(","):
        part = part.strip()
        if not part:
            continue
        port = int(part, 10)
        if not 0 < port <= 65535:
            raise ValueError(f"invalid TCP port: {port}")
        ports.append(port)
    if not ports:
        raise ValueError("at least one port is required")
    return ports


def connect_ports(host: str, ports: list[int], connect_timeout: float) -> list[PortState]:
    states = []
    for port in ports:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(connect_timeout)
        try:
            sock.connect((host, port))
        except OSError as exc:
            print(f"[{port}] connect failed: {exc}", file=sys.stderr)
            sock.close()
            continue
        sock.setblocking(False)
        print(f"[{port}] connected to {host}:{port}")
        states.append(PortState(port=port, sock=sock))
    return states


def send_frame(state: PortState, obj: Any) -> None:
    data = pack_msgpack(obj)
    state.sock.sendall(data)
    print(f"[{state.port}] TX {render(obj)} ({len(data)} bytes)")


def send_discovery(states: list[PortState], methods: list[str]) -> None:
    print(
        "WARNING: --discover sends msgpack-rpc requests. On the tested Portenta X8 "
        "image, unsupported methods can restart m4_proxy.",
        file=sys.stderr,
    )
    msg_id = 1
    for state in states:
        for method in methods:
            send_frame(state, [0, msg_id, method, []])
            msg_id += 1


def send_tty_injection(states: list[PortState], payload: bytes) -> None:
    for state in states:
        send_frame(state, [2, "tty", [payload]])


def listen(states: list[PortState], timeout_s: float, expect_text: str | None) -> bool:
    deadline = time.monotonic() + timeout_s
    expected = expect_text.encode("utf-8") if expect_text else None
    found_expected = False

    while states and time.monotonic() < deadline:
        for state in list(states):
            try:
                chunk = state.sock.recv(4096)
            except BlockingIOError:
                continue
            except OSError as exc:
                print(f"[{state.port}] read failed: {exc}", file=sys.stderr)
                states.remove(state)
                state.sock.close()
                continue

            if not chunk:
                print(f"[{state.port}] peer closed")
                states.remove(state)
                state.sock.close()
                continue

            state.raw_rx += len(chunk)
            if expected and expected in chunk:
                found_expected = True
            print(f"[{state.port}] RX raw {len(chunk)} bytes hex={binascii.hexlify(chunk[:96]).decode('ascii')}")

            try:
                objects = state.decoder.feed(chunk)
            except MsgpackError as exc:
                print(f"[{state.port}] decode error: {exc}", file=sys.stderr)
                continue

            for obj in objects:
                state.decoded_rx += 1
                for line in describe_rpc_object(state.port, obj):
                    print(line)
                if expected:
                    for payload in iter_payloads(obj):
                        if expected in payload:
                            found_expected = True

        time.sleep(0.02)

    for state in states:
        pending = len(state.decoder.buffer)
        print(f"[{state.port}] summary raw_rx={state.raw_rx} decoded_rx={state.decoded_rx} pending_bytes={pending}")
        state.sock.close()

    if expect_text:
        print(f"expect_text={expect_text!r} found={found_expected}")
    return found_expected


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="127.0.0.1", help="m4_proxy host or forwarded host")
    parser.add_argument("--ports", default="5000,5001", help="comma-separated TCP ports")
    parser.add_argument("--timeout", type=float, default=10.0, help="listen window in seconds")
    parser.add_argument("--connect-timeout", type=float, default=3.0)
    parser.add_argument("--discover", action="store_true", help="send common msgpack-rpc discovery requests")
    parser.add_argument(
        "--method",
        action="append",
        default=[],
        help="additional discovery method to call with empty params; may be repeated",
    )
    parser.add_argument(
        "--inject-tty",
        default=None,
        help="send a tty notification containing this text after connecting (can write toward M4 SerialRPC RX)",
    )
    parser.add_argument("--expect-text", default=None, help="exit 0 only if this text appears in raw or decoded payloads")
    args = parser.parse_args()

    if socket is None:
        print(
            "Python socket module is unavailable in this interpreter. "
            "Run this tool on the host with adb forward, or install a full Python stdlib on the X8.",
            file=sys.stderr,
        )
        return 2

    if args.timeout <= 0:
        print("timeout must be > 0", file=sys.stderr)
        return 2

    try:
        ports = parse_ports(args.ports)
    except ValueError as exc:
        print(str(exc), file=sys.stderr)
        return 2

    states = connect_ports(args.host, ports, args.connect_timeout)
    if not states:
        return 1

    if args.discover or args.method:
        methods = list(DEFAULT_DISCOVERY_METHODS if args.discover else ())
        methods.extend(args.method)
        send_discovery(states, methods)

    if args.inject_tty is not None:
        send_tty_injection(states, args.inject_tty.encode("utf-8"))

    found = listen(states, args.timeout, args.expect_text)
    if args.expect_text and not found:
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())