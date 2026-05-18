#!/usr/bin/env python3
"""
l072_probe_get.py — Run AN3155 GET on the L072 ROM, print supported command
list verbatim so we can tell whether Extended Erase (0x44) is actually
advertised. Run while OpenOCD is holding the L072 in ROM.
"""
import os, sys, time, termios

PORT = "/dev/ttymxc3"; BAUD = termios.B19200
ACK = b'\x79'; NACK = b'\x1f'

def open_port():
    fd = os.open(PORT, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    iflag, oflag, cflag, lflag, ispeed, ospeed, cc = termios.tcgetattr(fd)
    cflag &= ~(termios.PARENB | termios.PARODD | termios.CSIZE | termios.CSTOPB | termios.CRTSCTS)
    cflag |= (termios.CS8 | termios.PARENB | termios.CREAD | termios.CLOCAL)
    iflag &= ~(termios.IXON | termios.IXOFF | termios.IXANY | termios.INLCR | termios.IGNCR | termios.ICRNL)
    oflag &= ~termios.OPOST
    lflag &= ~(termios.ICANON | termios.ECHO | termios.ECHOE | termios.ISIG)
    termios.tcsetattr(fd, termios.TCSANOW, [iflag, oflag, cflag, lflag, BAUD, BAUD, cc])
    os.set_blocking(fd, False)
    return fd

def rd(fd, n, t=2.0):
    end = time.time() + t; data = b''
    while len(data) < n:
        if time.time() > end: raise TimeoutError(f"got {len(data)}/{n}: {data.hex()}")
        try:
            c = os.read(fd, n - len(data))
            if c: data += c
        except BlockingIOError:
            time.sleep(0.005)
    return data

def main():
    fd = open_port()
    # sync
    os.write(fd, b'\x7F')
    b = rd(fd, 1, 1.5)
    print(f"sync: {b.hex()} ({'ACK' if b==ACK else 'NACK' if b==NACK else 'other'})")
    # GET
    os.write(fd, b'\x00\xFF')
    b = rd(fd, 1)
    print(f"GET cmd ack: {b.hex()}")
    if b != ACK: return 1
    n = ord(rd(fd, 1)); ver = ord(rd(fd, 1))
    print(f"GET: N={n} version=0x{ver:02X}")
    cmds = rd(fd, n)
    print(f"supported commands: {' '.join(f'{c:02X}' for c in cmds)}")
    final = rd(fd, 1)
    print(f"GET final ack: {final.hex()}")
    print(f"\n  ExtErase 0x44 supported? {'YES' if 0x44 in cmds else 'NO'}")
    print(f"  Erase    0x43 supported? {'YES' if 0x43 in cmds else 'NO'}")
    print(f"  Wr/Rd Unprotect: {'WP-set ' if 0x73 in cmds else ''}{'RDP-set ' if 0x92 in cmds else ''}")
    # GET ID
    os.write(fd, b'\x02\xFD')
    b = rd(fd, 1)
    print(f"GETID cmd ack: {b.hex()}")
    if b == ACK:
        n2 = ord(rd(fd, 1))
        pid = rd(fd, n2+1)
        rd(fd, 1)
        print(f"PID = 0x{int.from_bytes(pid,'big'):0{2*(n2+1)}X}  (STM32L072 expected 0x447)")
    return 0

if __name__ == "__main__":
    sys.exit(main())
