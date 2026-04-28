#!/usr/bin/env python3
"""Pair the base-station with a fresh handheld controller over USB-CDC.

Per CYBERSECURITY_CASE.md the pairing flow:

  1. Operator plugs the handheld into the base-station laptop via USB-C.
  2. The handheld presents a USB-CDC ACM device (Arduino bootloader leaves
     it on /dev/ttyACM0 by default).
  3. This script:
       a. opens the port at 115 200 baud,
       b. sends a "HELLO" frame with the operator's name,
       c. waits for the handheld's serial number + firmware version,
       d. derives a fresh 256-bit AES-GCM key from os.urandom(),
       e. writes it to the base-station keystore (config/keys/<sn>.bin),
       f. ships it back to the handheld over USB-CDC where the Opta M4
          burns it into the secure element.
  4. After the handheld confirms key write, the script logs the pairing to
     audit_log.jsonl and exits 0. Any error path exits non-zero so the
     calling shell wrapper can surface it.

This file deliberately does no networking — pairing is line-of-sight,
USB-only, with a physical button-press confirmation on the handheld.
"""
from __future__ import annotations

import argparse
import json
import os
import secrets
import sys
import time
from dataclasses import dataclass
from typing import Optional

try:
    import serial                                # type: ignore
except ImportError:                              # pragma: no cover
    serial = None                                # type: ignore

DEFAULT_PORT = "/dev/ttyACM0"
DEFAULT_BAUD = 115_200
DEFAULT_TIMEOUT_S = 5.0
DEFAULT_KEYSTORE = "config/keys"
DEFAULT_AUDIT_LOG = "logs/audit_log.jsonl"

HELLO = b"LIFETRAC-PAIR-HELLO\n"
KEY_WRITE = b"LIFETRAC-PAIR-KEY "       # followed by 64 hex chars + b"\n"
KEY_OK = b"LIFETRAC-PAIR-KEY-OK"


@dataclass
class HandheldIdent:
    serial_number: str
    firmware_version: str


def _read_line(port, timeout_s: float) -> bytes:
    deadline = time.monotonic() + timeout_s
    buf = bytearray()
    while time.monotonic() < deadline:
        chunk = port.read(1)
        if not chunk:
            continue
        buf += chunk
        if chunk == b"\n":
            return bytes(buf).rstrip(b"\r\n")
    raise TimeoutError("handheld did not respond in time")


def handshake(port, operator: str, timeout_s: float) -> HandheldIdent:
    port.write(HELLO + operator.encode("utf-8") + b"\n")
    port.flush()
    line = _read_line(port, timeout_s)
    # Expected format: "LIFETRAC-PAIR-IDENT <serial> <fw_version>"
    parts = line.decode("utf-8", "replace").split()
    if len(parts) < 3 or parts[0] != "LIFETRAC-PAIR-IDENT":
        raise RuntimeError(f"unexpected handshake response: {line!r}")
    return HandheldIdent(serial_number=parts[1], firmware_version=parts[2])


def push_key(port, key: bytes, timeout_s: float) -> None:
    if len(key) != 32:
        raise ValueError("key must be 32 bytes")
    port.write(KEY_WRITE + key.hex().encode("ascii") + b"\n")
    port.flush()
    line = _read_line(port, timeout_s)
    if line != KEY_OK:
        raise RuntimeError(f"handheld rejected key: {line!r}")


def write_keystore(directory: str, ident: HandheldIdent, key: bytes) -> str:
    os.makedirs(directory, exist_ok=True)
    path = os.path.join(directory, f"{ident.serial_number}.bin")
    fd = os.open(path, os.O_WRONLY | os.O_CREAT | os.O_TRUNC, 0o600)
    try:
        os.write(fd, key)
        os.fsync(fd)
    finally:
        os.close(fd)
    return path


def append_audit(path: str, payload: dict) -> None:
    os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
    with open(path, "a", encoding="utf-8") as fh:
        fh.write(json.dumps(payload, separators=(",", ":")) + "\n")


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default=DEFAULT_PORT)
    parser.add_argument("--baud", type=int, default=DEFAULT_BAUD)
    parser.add_argument("--timeout", type=float, default=DEFAULT_TIMEOUT_S)
    parser.add_argument("--operator", required=True,
                        help="Logged in audit_log.jsonl alongside the pairing event.")
    parser.add_argument("--keystore", default=DEFAULT_KEYSTORE)
    parser.add_argument("--audit-log", default=DEFAULT_AUDIT_LOG)
    args = parser.parse_args(argv)

    if serial is None:
        print("pyserial is not installed; cannot open USB-CDC port", file=sys.stderr)
        return 2

    try:
        port = serial.Serial(args.port, args.baud, timeout=0.1)
    except (serial.SerialException, OSError) as exc:
        print(f"cannot open {args.port}: {exc}", file=sys.stderr)
        return 3

    try:
        ident = handshake(port, args.operator, args.timeout)
        key = secrets.token_bytes(32)
        push_key(port, key, args.timeout)
        keystore_path = write_keystore(args.keystore, ident, key)
        append_audit(args.audit_log, {
            "ts": int(time.time()),
            "event": "handheld_paired",
            "operator": args.operator,
            "serial_number": ident.serial_number,
            "firmware_version": ident.firmware_version,
            "keystore_path": keystore_path,
        })
        print(f"OK paired serial={ident.serial_number} fw={ident.firmware_version} "
              f"key_written={keystore_path}")
        return 0
    except (RuntimeError, TimeoutError, ValueError) as exc:
        print(f"pairing failed: {exc}", file=sys.stderr)
        return 4
    finally:
        port.close()


if __name__ == "__main__":               # pragma: no cover
    sys.exit(main())
