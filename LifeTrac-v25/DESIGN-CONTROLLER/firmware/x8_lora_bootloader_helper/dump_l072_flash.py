#!/usr/bin/env python3
"""dump_l072_flash.py — read N bytes of L072 flash and write to file."""
import sys, os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import importlib.util
spec = importlib.util.spec_from_file_location("flasher", "/tmp/flash_l072_via_uart.py")
m = importlib.util.module_from_spec(spec); spec.loader.exec_module(m)

length = int(sys.argv[1], 0) if len(sys.argv) > 1 else 16440
out_path = sys.argv[2] if len(sys.argv) > 2 else "/tmp/l072_flash_dump.bin"
fd = m.open_serial()
try:
    m.do_probe(fd)
    print(f"Reading {length} bytes from 0x08000000 ...")
    data = m.do_read(fd, 0x08000000, length)
    with open(out_path, "wb") as fp:
        fp.write(data)
    print(f"Wrote {len(data)} bytes to {out_path}")
finally:
    os.close(fd)
