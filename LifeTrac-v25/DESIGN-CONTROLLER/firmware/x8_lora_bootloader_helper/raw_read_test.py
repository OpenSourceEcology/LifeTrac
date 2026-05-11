#!/usr/bin/env python3
"""Raw read test: reads from /dev/ttymxc3 for 2s without sending anything."""
import os, sys, time

dev = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttymxc3"
baud = sys.argv[2] if len(sys.argv) > 2 else "921600"

os.system(f"stty -F {dev} {baud} cs8 -parenb -cstopb raw -echo 2>/dev/null")
fd = os.open(dev, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)

t0 = time.time()
buf = bytearray()
while time.time() - t0 < 2.0:
    try:
        chunk = os.read(fd, 256)
        buf.extend(chunk)
    except BlockingIOError:
        time.sleep(0.01)

os.close(fd)
print(f"bytes={len(buf)}")
if buf:
    print(f"hex={buf.hex()}")
    print(f"all_zero={all(b == 0 for b in buf)}")
    # Try to find non-zero bytes
    nz = [(i, b) for i, b in enumerate(buf) if b != 0]
    print(f"nonzero_count={len(nz)} first_10={nz[:10]}")
else:
    print("no bytes received")
