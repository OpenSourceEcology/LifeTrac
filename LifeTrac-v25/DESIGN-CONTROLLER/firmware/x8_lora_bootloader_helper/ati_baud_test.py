#!/usr/bin/env python3
"""Send ATI at a given baud and read response. Also try binary VER_REQ."""
import os, sys, time

dev = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttymxc3"
baud = sys.argv[2] if len(sys.argv) > 2 else "19200"

os.system(f"stty -F {dev} {baud} cs8 -parenb -cstopb raw -echo 2>/dev/null")
fd = os.open(dev, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)

def drain(fd, t=0.05):
    buf = bytearray()
    t0 = time.time()
    while time.time() - t0 < t:
        try:
            buf.extend(os.read(fd, 256))
        except BlockingIOError:
            time.sleep(0.005)
    return bytes(buf)

def read_for(fd, seconds, max_bytes=512):
    buf = bytearray()
    t0 = time.time()
    while time.time() - t0 < seconds and len(buf) < max_bytes:
        try:
            buf.extend(os.read(fd, 256))
        except BlockingIOError:
            time.sleep(0.005)
    return bytes(buf)

# Drain any pending
pre = drain(fd, 0.3)
print(f"pre-drain: {len(pre)} bytes  hex={pre.hex()[:40]}")

# Send ATI
print(f"sending ATI at {baud} baud...")
os.write(fd, b"ATI\r\n")
time.sleep(0.2)
reply = read_for(fd, 0.8)
print(f"reply: {len(reply)} bytes  hex={reply.hex()[:80]}")
all_zero = all(b == 0 for b in reply) if reply else True
nonzero = [(i, hex(b)) for i, b in enumerate(reply) if b != 0]
print(f"all_zero={all_zero} nonzero_count={len(nonzero)} first_nonzero={nonzero[:5]}")

# Drain
drain(fd, 0.1)

# Send AT+VER?
print(f"\nsending AT+VER? at {baud} baud...")
os.write(fd, b"AT+VER?\r\n")
time.sleep(0.2)
reply2 = read_for(fd, 0.8)
print(f"reply: {len(reply2)} bytes  hex={reply2.hex()[:80]}")

os.close(fd)
