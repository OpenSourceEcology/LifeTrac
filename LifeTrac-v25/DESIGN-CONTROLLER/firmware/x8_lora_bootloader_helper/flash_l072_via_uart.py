#!/usr/bin/env python3
"""
flash_l072_via_uart.py — minimal STM32 AN3155 UART bootloader client.

Specifically tuned for the LifeTrac Murata L072 reached over the Portenta X8
x8h7 bridge on /dev/ttymxc3 at 19200 8E1.

KEY INSIGHT (2026-05-13): the ROM bootloader on this board is *already*
auto-bauded by the time we open /dev/ttymxc3 (a glitch byte during NRST
release consumes the auto-baud slot). Sending the standard 0x7F sync byte
returns 0x1F NACK, which we used to mis-read as "ROM dead". Skipping 0x7F
and going straight to GET (0x00 0xFF) succeeds.

This client therefore:
  * NEVER sends 0x7F.
  * Opens at 19200 8E1.
  * Verifies ROM by sending GET first.
  * Implements GET, GET_ID, READ_MEMORY, WRITE_MEMORY, ERASE_NO_STRETCH.
  * Default action: read-only ID + first 256 bytes of flash, dump as hex.

Usage:
  flash_l072_via_uart.py probe            # GET + GET_ID
  flash_l072_via_uart.py read [addr] [n]  # READ_MEMORY (default 0x08000000, 256)
  flash_l072_via_uart.py erase            # mass-erase
  flash_l072_via_uart.py write <bin>      # ERASE + WRITE + verify
  flash_l072_via_uart.py go [addr]        # GO (default 0x08000000)
  flash_l072_via_uart.py wunprot          # WRITE_UNPROTECT (cmd 0x73) - clears WRP, auto-resets
  flash_l072_via_uart.py runprot          # READOUT_UNPROTECT (cmd 0x92) - mass-erase + OPT defaults + reset

Run via sudo because /dev/ttymxc3 is root-only on LmP without dialout.
"""
import os, sys, time, struct, argparse, subprocess, select

DEV = "/dev/ttymxc3"
ACK = 0x79
NACK = 0x1F

def configure_uart_via_stty():
    """LmP Python lacks termios; use system stty."""
    cmd = ["stty", "-F", DEV, "19200", "cs8", "parenb", "-parodd",
           "-cstopb", "raw", "-echo", "-ixon", "-ixoff",
           "-inpck", "-istrip"]
    subprocess.check_call(cmd)

def open_serial():
    configure_uart_via_stty()
    fd = os.open(DEV, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    return fd

def write_all(fd, data):
    n = 0
    while n < len(data):
        try:
            w = os.write(fd, data[n:])
            if w <= 0:
                time.sleep(0.005); continue
            n += w
        except BlockingIOError:
            time.sleep(0.005)

def read_n(fd, n, timeout_s=2.0):
    out = b""
    deadline = time.time() + timeout_s
    while len(out) < n:
        remain = deadline - time.time()
        if remain <= 0:
            break
        r, _, _ = select.select([fd], [], [], min(remain, 0.2))
        if r:
            try:
                chunk = os.read(fd, n - len(out))
            except BlockingIOError:
                chunk = b""
            if chunk:
                out += chunk
    return out

def expect_ack(fd, ctx, timeout_s=1.0):
    b = read_n(fd, 1, timeout_s)
    if not b:
        raise IOError(f"{ctx}: no response")
    if b[0] == NACK:
        raise IOError(f"{ctx}: NACK (0x1F)")
    if b[0] != ACK:
        raise IOError(f"{ctx}: unexpected 0x{b[0]:02x}")

def cmd(fd, code, timeout_s=1.0):
    """Send command byte + complement."""
    write_all(fd, bytes([code, code ^ 0xFF]))
    expect_ack(fd, f"cmd 0x{code:02x}", timeout_s)

def do_write_unprotect(fd):
    """AN3155 cmd 0x73: clears WRPROT1/WRPROT2 then chip auto-resets.
    WARNING: On STM32L072 ROM Bootloader v3.1, this command fails to correctly
    rewrite the complement values (nRDP/nUSER) causing a permanent Option Byte
    Mismatch Lockout (hardware brick). 
    DO NOT USE ON L072. DEPRECATED.
    """
    print("!!! WARNING: Executing cmd 0x73 (wunprot) on STM32L072 is known to brick the device !!!")
    cmd(fd, 0x73, timeout_s=2.0)
    print("WRITE_UNPROTECT ACK \u2014 chip is auto-resetting; OPT WRP should now be 0xFFFF")

def do_readout_unprotect(fd):
    """AN3155 cmd 0x92: RDP\u2192AA + mass erase + OPT defaults + reset.
    Per stm32flash: STM32_MASSERASE_TIMEOUT = 35s for the second ACK.
    Note: spec says one ACK before erase + one ACK after; some chips only emit
    a single ACK and reset. We accept either.
    """
    cmd(fd, 0x92, timeout_s=2.0)  # first ACK (command accepted)
    print("READOUT_UNPROTECT first ACK \u2014 waiting up to 35s for completion ACK ...")
    try:
        b = read_n(fd, 1, 35.0)
        if b and b[0] == ACK:
            print("READOUT_UNPROTECT second ACK \u2014 chip wiped + reset")
        elif b:
            print(f"READOUT_UNPROTECT: post-erase byte 0x{b[0]:02x} (chip may have reset before ACK)")
        else:
            print("READOUT_UNPROTECT: no second ACK before timeout (chip may have reset already)")
    except IOError as e:
        print(f"READOUT_UNPROTECT: {e} (likely chip reset)")

def addr_with_csum(addr):
    a = struct.pack(">I", addr)
    cs = a[0] ^ a[1] ^ a[2] ^ a[3]
    return a + bytes([cs])

def do_probe(fd):
    cmd(fd, 0x00)  # GET
    n = read_n(fd, 1)[0]
    payload = read_n(fd, n + 1)  # version + n command bytes
    final = read_n(fd, 1)
    if not final or final[0] != ACK:
        raise IOError("GET: no trailing ACK")
    ver = payload[0]
    cmds = payload[1:]
    print(f"GET: bootloader v{ver>>4}.{ver&0xF}  cmds={ ' '.join(f'{c:02x}' for c in cmds) }")
    cmd(fd, 0x02)  # GET_ID
    nid = read_n(fd, 1)[0]
    pid = read_n(fd, nid + 1)
    final = read_n(fd, 1)
    if not final or final[0] != ACK:
        raise IOError("GET_ID: no trailing ACK")
    pid_int = int.from_bytes(pid, "big")
    print(f"GET_ID: PID=0x{pid_int:0{(nid+1)*2}x}")
    return ver, cmds, pid_int

def do_read(fd, addr, length):
    out = bytearray()
    while length > 0:
        chunk = min(length, 256)
        cmd(fd, 0x11)  # READ_MEMORY
        write_all(fd, addr_with_csum(addr))
        expect_ack(fd, "READ_MEMORY addr")
        write_all(fd, bytes([chunk - 1, (chunk - 1) ^ 0xFF]))
        expect_ack(fd, "READ_MEMORY len")
        data = read_n(fd, chunk, 3.0)
        if len(data) != chunk:
            raise IOError(f"READ_MEMORY: short read {len(data)}/{chunk} at 0x{addr:08x}")
        out += data
        addr += chunk
        length -= chunk
    return bytes(out)

def do_erase_mass(fd):
    # Extended erase (0x44) — L072 supports no-stretch variant 0x44.
    cmd(fd, 0x44)
    # Mass erase: send 0xFFFF + checksum 0x00.
    write_all(fd, b"\xff\xff\x00")
    expect_ack(fd, "ERASE mass")
    print("Mass erase OK")

def do_write(fd, addr, data):
    while data:
        chunk = data[:256]
        data = data[256:]
        cmd(fd, 0x31)  # WRITE_MEMORY
        write_all(fd, addr_with_csum(addr))
        expect_ack(fd, "WRITE_MEMORY addr")
        n = len(chunk) - 1
        payload = bytes([n]) + chunk
        cs = 0
        for b in payload:
            cs ^= b
        write_all(fd, payload + bytes([cs]))
        expect_ack(fd, "WRITE_MEMORY data")
        addr += len(chunk)
        if addr & 0x3FF == 0:
            print(f".. 0x{addr:08x}")

def do_go(fd, addr):
    cmd(fd, 0x21)
    write_all(fd, addr_with_csum(addr))
    expect_ack(fd, "GO")
    print(f"GO 0x{addr:08x} OK")

def hexdump(data, base=0):
    for i in range(0, len(data), 16):
        row = data[i:i+16]
        hexes = " ".join(f"{b:02x}" for b in row)
        ascii_ = "".join(chr(b) if 32 <= b < 127 else "." for b in row)
        print(f"{base+i:08x}  {hexes:<48}  {ascii_}")

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("action", choices=["probe", "read", "erase", "write", "go", "wunprot", "runprot"], 
                    help="WARNING: 'wunprot' corrupts Option Bytes on L072. Use 'runprot' to repair.")
    ap.add_argument("rest", nargs="*")
    args = ap.parse_args()
    fd = open_serial()
    try:
        do_probe(fd)
        if args.action == "probe":
            return
        if args.action == "read":
            addr = int(args.rest[0], 0) if len(args.rest) > 0 else 0x08000000
            length = int(args.rest[1], 0) if len(args.rest) > 1 else 256
            data = do_read(fd, addr, length)
            hexdump(data, base=addr)
            return
        if args.action == "erase":
            do_erase_mass(fd)
            return
        if args.action == "write":
            if not args.rest:
                sys.exit("write needs <bin>")
            with open(args.rest[0], "rb") as fp:
                payload = fp.read()
            print(f"Writing {len(payload)} bytes to 0x08000000 ...")
            do_erase_mass(fd)
            do_write(fd, 0x08000000, payload)
            print("Verifying ...")
            rb = do_read(fd, 0x08000000, len(payload))
            if rb == payload:
                print("VERIFY OK")
            else:
                # find first differing byte
                for i, (a, b) in enumerate(zip(payload, rb)):
                    if a != b:
                        print(f"VERIFY FAIL at offset {i}: wrote {a:02x} read {b:02x}")
                        sys.exit(2)
            return
        if args.action == "go":
            addr = int(args.rest[0], 0) if args.rest else 0x08000000
            do_go(fd, addr)
            return
        if args.action == "wunprot":
            do_write_unprotect(fd)
            return
        if args.action == "runprot":
            do_readout_unprotect(fd)
            return
    finally:
        os.close(fd)

if __name__ == "__main__":
    main()
