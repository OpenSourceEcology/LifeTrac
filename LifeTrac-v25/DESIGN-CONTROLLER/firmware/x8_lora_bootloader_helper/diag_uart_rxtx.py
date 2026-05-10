#!/usr/bin/env python3
"""
diag_uart_rxtx.py — live diagnostic for X8 ttymxc3 <-> Murata LPUART1.
Prints current termios baud, then sends ATI + binary VER_REQ and listens
for any response bytes for 2 seconds each.

Run directly on the X8 while the Murata is in the main loop:
  python3 /tmp/lifetrac_p0c/diag_uart_rxtx.py
"""
import argparse
import os
import struct
import subprocess
import time


def configure_port(dev, baud=921600, hwflow=False):
    # Use stty to configure the port (applied globally to the device).
    flow_flag = "crtscts" if hwflow else "-crtscts"
    r = subprocess.run(
        ["stty", "-F", dev, str(baud), "cs8", "-parenb", "-cstopb", "raw", "-echo", flow_flag],
        capture_output=True, text=True)
    if r.returncode != 0:
        print(f"  stty error: {r.stderr.strip()}")
    # Report current settings
    r2 = subprocess.run(["stty", "-F", dev, "-a"], capture_output=True, text=True)
    first_line = r2.stdout.strip().split("\n")[0] if r2.stdout else "(stty -a failed)"
    print(f"  stty -a: {first_line}")

def drain(fd, secs=1.5):
    deadline = time.monotonic() + secs
    data = bytearray()
    while time.monotonic() < deadline:
        try:
            chunk = os.read(fd, 256)
            if chunk:
                data.extend(chunk)
        except BlockingIOError:
            time.sleep(0.01)
    return bytes(data)

def crc16_ccitt(data):
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc

def cobs_encode(data):
    out = bytearray([0])
    code_index = 0
    code = 1
    for byte in data:
        if byte == 0:
            out[code_index] = code
            code_index = len(out)
            out.append(0)
            code = 1
            continue
        out.append(byte)
        code += 1
        if code == 0xFF:
            out[code_index] = code
            code_index = len(out)
            out.append(0)
            code = 1
    out[code_index] = code
    return bytes(out)

def build_ver_req():
    HOST_PROTOCOL_VER = 1
    HOST_TYPE_VER_REQ = 0x01
    header = struct.pack("<BBBHH", HOST_PROTOCOL_VER, HOST_TYPE_VER_REQ, 0, 1, 0)
    inner = header
    inner += struct.pack("<H", crc16_ccitt(inner))
    return b"\x00" + cobs_encode(inner) + b"\x00"

def main():
    parser = argparse.ArgumentParser(description="UART RX/TX diagnostics for Murata host link")
    parser.add_argument("--dev", default="/dev/ttymxc3", help="serial device path")
    parser.add_argument("--baud", type=int, default=921600, help="serial baud")
    parser.add_argument(
        "--hwflow",
        choices=["off", "on"],
        default="off",
        help="host UART RTS/CTS hardware flow control",
    )
    parser.add_argument("--listen-secs", type=float, default=2.0, help="passive listen window")
    args = parser.parse_args()

    hwflow = args.hwflow == "on"
    print(f"=== UART RX/TX diagnostic on {args.dev} ===")
    print(f"mode: baud={args.baud} hwflow={args.hwflow}")
    print()

    fd = os.open(args.dev, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    print(f"Opened fd={fd}")
    configure_port(args.dev, baud=args.baud, hwflow=hwflow)

    print()
    print(f"--- Passive listen {args.listen_secs:.1f}s (should be empty since BOOT_URC long gone) ---")
    passive = drain(fd, args.listen_secs)
    if passive:
        print(f"  Got {len(passive)} bytes: {passive.hex()}")
    else:
        print("  (no bytes)")

    print()
    print("--- Sending ATI<CR><LF> ---")
    os.write(fd, b"ATI\r\n")
    time.sleep(0.05)
    reply_ati = drain(fd, 1.5)
    if reply_ati:
        printable = ''.join(chr(b) if 0x20 <= b <= 0x7E else '.' for b in reply_ati)
        print(f"  Got {len(reply_ati)} bytes: {reply_ati.hex()}")
        print(f"  txt: {printable}")
    else:
        print("  (no bytes — LPUART1 RX may not be reaching Murata)")

    print()
    print("--- Sending binary VER_REQ (type=0x01) ---")
    req = build_ver_req()
    print(f"  Sending {len(req)} bytes: {req.hex()}")
    os.write(fd, req)
    time.sleep(0.05)
    reply_ver = drain(fd, 2.0)
    if reply_ver:
        print(f"  Got {len(reply_ver)} bytes: {reply_ver.hex()}")
    else:
        print("  (no bytes — VER_URC not received)")

    print()
    print("--- Sending PING_REQ (type=0x00) ---")
    header = struct.pack("<BBBHH", 1, 0x00, 0, 2, 0)
    inner = header + struct.pack("<H", crc16_ccitt(header))
    ping = b"\x00" + cobs_encode(inner) + b"\x00"
    print(f"  Sending {len(ping)} bytes: {ping.hex()}")
    os.write(fd, ping)
    time.sleep(0.05)
    reply_ping = drain(fd, 2.0)
    if reply_ping:
        print(f"  Got {len(reply_ping)} bytes: {reply_ping.hex()}")
    else:
        print("  (no bytes)")

    os.close(fd)
    print()
    print("Done.")


if __name__ == "__main__":
    main()
