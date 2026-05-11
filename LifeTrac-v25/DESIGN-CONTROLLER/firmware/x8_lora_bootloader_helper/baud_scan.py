#!/usr/bin/env python3
"""Test AN3155 bootloader sync and multiple baud rates."""
import os, sys, time

dev = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttymxc3"

def test_baud(baud_str, send_bytes, label):
    os.system(f"stty -F {dev} {baud_str} cs8 -parenb -cstopb raw -echo 2>/dev/null")
    fd = os.open(dev, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    
    # drain
    t0 = time.time()
    while time.time() - t0 < 0.3:
        try: os.read(fd, 256)
        except BlockingIOError: time.sleep(0.005)
    
    os.write(fd, send_bytes)
    time.sleep(0.3)
    
    buf = bytearray()
    t0 = time.time()
    while time.time() - t0 < 0.7:
        try: buf.extend(os.read(fd, 256))
        except BlockingIOError: time.sleep(0.005)
    
    os.close(fd)
    
    hex_str = buf.hex() if buf else "(none)"
    nonzero = [f"{i}:{hex(b)}" for i, b in enumerate(buf) if b != 0]
    print(f"[{baud_str}] {label}: sent={send_bytes.hex()} got={len(buf)}bytes hex={hex_str[:60]} nonzero={nonzero}")
    return buf

# AN3155 sync byte — should get 0x79 ACK from ROM bootloader
print("=== AN3155 sync test (0x7F) ===")
test_baud("9600",   b"\x7f", "sync")
test_baud("19200",  b"\x7f", "sync")
test_baud("38400",  b"\x7f", "sync")
test_baud("57600",  b"\x7f", "sync")
test_baud("115200", b"\x7f", "sync")
test_baud("230400", b"\x7f", "sync")
test_baud("460800", b"\x7f", "sync")
test_baud("921600", b"\x7f", "sync")

print("\n=== ATI at various bauds ===")
test_baud("9600",   b"ATI\r\n", "ATI")
test_baud("19200",  b"ATI\r\n", "ATI")
test_baud("38400",  b"ATI\r\n", "ATI")
test_baud("115200", b"ATI\r\n", "ATI")
test_baud("921600", b"ATI\r\n", "ATI")
