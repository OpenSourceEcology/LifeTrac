#!/usr/bin/env python3
# stm32_an3155_flasher.py
#
# Self-contained AN3155 flasher for STM32 system bootloader over UART.
# Targets STM32L072 (Murata CMWX1ZZABZ-078) on Portenta X8 + Max Carrier
# accessible at /dev/ttymxc3 once BOOT0 is held HIGH and NRST has been pulsed.
#
# Stdlib only: os, sys, struct, time, select, argparse.
# (Portenta X8 LmP Python ships without termios/fcntl C-extensions, so we
# rely on the caller to configure the UART externally with `stty` BEFORE
# launching this script. Required stty: `stty -F <port> 19200 cs8 parenb
# -parodd -cstopb raw -echo`.)
#
# Usage:
#   stty -F /dev/ttymxc3 19200 cs8 parenb -parodd -cstopb raw -echo
#   python3 stm32_an3155_flasher.py /dev/ttymxc3 mlm32l07x01.bin [--verify] [--no-erase]

import argparse
import os
import select
import struct
import sys
import time

BAUD = 19200
BITS_PER_BYTE = 11.0  # 1 start + 8 data + 1 parity + 1 stop
BYTE_TIME_S = BITS_PER_BYTE / BAUD  # ~0.573 ms

ACK = 0x79
NACK = 0x1F
CHIP_ID_L072 = 0x0447
FLASH_BASE = 0x08000000
BLOCK = 256


# ---------------------------------------------------------------------------
# UART helpers (no termios)
# ---------------------------------------------------------------------------

def open_uart(path):
    return os.open(path, os.O_RDWR | os.O_NOCTTY)


def write_bytes(fd, data):
    n = 0
    while n < len(data):
        n += os.write(fd, data[n:])
    # Wait for the bytes to clear the UART hardware (no tcdrain available).
    time.sleep(len(data) * BYTE_TIME_S + 0.01)


def read_exact(fd, count, timeout):
    deadline = time.monotonic() + timeout
    buf = bytearray()
    while len(buf) < count:
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            raise TimeoutError(
                "read_exact: got %d/%d bytes in %.2fs (have=%s)"
                % (len(buf), count, timeout, buf.hex()))
        r, _, _ = select.select([fd], [], [], remaining)
        if not r:
            continue
        try:
            chunk = os.read(fd, count - len(buf))
        except BlockingIOError:
            continue
        if not chunk:
            continue
        buf.extend(chunk)
    return bytes(buf)


def drain_input(fd, settle=0.05):
    deadline = time.monotonic() + settle
    while True:
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            return
        r, _, _ = select.select([fd], [], [], remaining)
        if not r:
            return
        try:
            chunk = os.read(fd, 4096)
        except BlockingIOError:
            return
        if not chunk:
            return


# ---------------------------------------------------------------------------
# AN3155 primitives
# ---------------------------------------------------------------------------

def _xor(buf):
    x = 0
    for b in buf:
        x ^= b
    return x


def expect_ack(fd, what, timeout=1.0):
    b = read_exact(fd, 1, timeout)[0]
    if b == ACK:
        return
    if b == NACK:
        raise RuntimeError("%s: NACK (0x1F)" % what)
    raise RuntimeError("%s: unexpected response 0x%02X" % (what, b))


def send_sync(fd):
    drain_input(fd)
    write_bytes(fd, b"\x7f")
    b = read_exact(fd, 1, timeout=2.0)[0]
    if b == ACK:
        return
    if b == NACK:
        raise RuntimeError("sync: NACK (bootloader busy or already-synced)")
    raise RuntimeError("sync: unexpected 0x%02X (expected 0x79)" % b)


def send_cmd(fd, opcode):
    write_bytes(fd, bytes([opcode, opcode ^ 0xFF]))
    expect_ack(fd, "cmd 0x%02X" % opcode, timeout=1.0)


def cmd_get(fd):
    send_cmd(fd, 0x00)
    n = read_exact(fd, 1, 1.0)[0]
    ver = read_exact(fd, 1, 1.0)[0]
    cmds = read_exact(fd, n, 1.0)
    expect_ack(fd, "Get tail", 1.0)
    return ver, list(cmds)


def cmd_get_id(fd):
    send_cmd(fd, 0x02)
    n = read_exact(fd, 1, 1.0)[0]
    pid = read_exact(fd, n + 1, 1.0)
    expect_ack(fd, "Get ID tail", 1.0)
    if len(pid) == 2:
        return (pid[0] << 8) | pid[1]
    return int.from_bytes(pid, "big")


def cmd_read_memory(fd, addr, count):
    if not (1 <= count <= 256):
        raise ValueError("Read Memory count must be 1..256")
    send_cmd(fd, 0x11)
    addr_b = struct.pack(">I", addr)
    write_bytes(fd, addr_b + bytes([_xor(addr_b)]))
    expect_ack(fd, "Read Memory addr", 1.0)
    n_minus_1 = count - 1
    write_bytes(fd, bytes([n_minus_1, n_minus_1 ^ 0xFF]))
    expect_ack(fd, "Read Memory count", 1.0)
    return read_exact(fd, count, 5.0)


def cmd_extended_erase_mass(fd, timeout_s=45.0):
    send_cmd(fd, 0x44)
    payload = b"\xff\xff"
    write_bytes(fd, payload + bytes([_xor(payload)]))
    expect_ack(fd, "Extended Erase mass", timeout=timeout_s)


def cmd_extended_erase_pages(fd, start_page, count, timeout_s=30.0):
    """Per-page Extended Erase. count <= 124 to fit in payload comfortably."""
    if count < 1 or count > 124:
        raise ValueError("page count must be 1..124, got %d" % count)
    send_cmd(fd, 0x44)
    n_minus_1 = count - 1
    body = bytearray()
    body.append((n_minus_1 >> 8) & 0xFF)
    body.append(n_minus_1 & 0xFF)
    for p in range(start_page, start_page + count):
        body.append((p >> 8) & 0xFF)
        body.append(p & 0xFF)
    body.append(_xor(body))
    write_bytes(fd, bytes(body))
    expect_ack(fd, "Extended Erase pages %d..%d" % (start_page, start_page + count - 1),
               timeout=timeout_s)


def cmd_write_unprotect(fd):
    """0x73 Write Unprotect. Two ACKs then chip resets.
    Caller must re-sync (BOOT0 must still be HIGH for re-entry to bootloader).
    """
    send_cmd(fd, 0x73)
    expect_ack(fd, "Write Unprotect tail", timeout=5.0)


def cmd_readout_unprotect(fd):
    """0x92 Readout Unprotect. Two ACKs, chip resets, MASS ERASE.
    Use only if Read Memory returns garbage (RDP active).
    """
    send_cmd(fd, 0x92)
    expect_ack(fd, "Readout Unprotect tail", timeout=30.0)


def cmd_write_memory(fd, addr, data):
    if not (1 <= len(data) <= 256):
        raise ValueError("Write Memory data length must be 1..256")
    if addr % 4 != 0:
        raise ValueError("Write Memory address must be 4-byte aligned: 0x%X" % addr)
    if len(data) % 4 != 0:
        raise ValueError("Write Memory length must be 4-byte aligned: %d" % len(data))
    send_cmd(fd, 0x31)
    addr_b = struct.pack(">I", addr)
    write_bytes(fd, addr_b + bytes([_xor(addr_b)]))
    expect_ack(fd, "Write Memory addr", 1.0)
    n_minus_1 = len(data) - 1
    payload = bytes([n_minus_1]) + data
    write_bytes(fd, payload + bytes([_xor(payload)]))
    expect_ack(fd, "Write Memory data", timeout=5.0)


# ---------------------------------------------------------------------------
# Top-level flow
# ---------------------------------------------------------------------------

def pad_to_word(data):
    if len(data) % 4 == 0:
        return data
    return data + b"\xff" * (4 - (len(data) % 4))


def flash(fd, image, do_erase=True, do_verify=False):
    print("[1/6] sync (0x7F)")
    send_sync(fd)

    print("[2/6] Get")
    ver, cmds = cmd_get(fd)
    print("       bootloader version=0x%02X cmds=%s"
          % (ver, " ".join("%02X" % c for c in cmds)))
    for needed in (0x00, 0x02, 0x11, 0x31, 0x44):
        if needed not in cmds:
            raise RuntimeError("bootloader does not advertise opcode 0x%02X" % needed)

    print("[3/6] Get ID")
    pid = cmd_get_id(fd)
    print("       PID=0x%04X" % pid)
    if pid != CHIP_ID_L072:
        raise RuntimeError(
            "PID 0x%04X != expected 0x%04X (STM32L0x2 cat 5). "
            "Wrong chip or wrong UART." % (pid, CHIP_ID_L072))

    print("[4/6] Read Memory probe @ 0x%08X" % FLASH_BASE)
    sample = cmd_read_memory(fd, FLASH_BASE, 16)
    print("       first16=" + " ".join("%02X" % b for b in sample))

    if do_erase:
        # L072: 192KB flash @ 128 B/page = 1536 pages. We only need pages
        # covering the image. Round UP to whole pages.
        PAGE_SIZE = 128
        npages = (len(image) + PAGE_SIZE - 1) // PAGE_SIZE
        print("[5/6] Erase: try mass first, fall back to per-page (%d pages needed)"
              % npages)
        t0 = time.monotonic()
        try:
            cmd_extended_erase_mass(fd)
            print("       mass erase OK in %.1f s" % (time.monotonic() - t0))
        except RuntimeError as e:
            if "NACK" not in str(e):
                raise
            print("       mass erase NACKed -> Write Unprotect, re-sync, page-erase")
            try:
                cmd_write_unprotect(fd)
            except (TimeoutError, RuntimeError) as ue:
                print("       (Write Unprotect tail miss: %s) -- proceeding to re-sync" % ue)
            time.sleep(1.0)
            drain_input(fd, settle=0.5)
            send_sync(fd)
            # Re-issue Get to be safe (some bootloaders need it after reset)
            cmd_get(fd)
            BATCH = 100
            page = 0
            while page < npages:
                n = min(BATCH, npages - page)
                cmd_extended_erase_pages(fd, page, n, timeout_s=max(30.0, n * 0.5))
                page += n
                print("       erased pages 0..%d (%.1f s elapsed)"
                      % (page - 1, time.monotonic() - t0))
            print("       page erase OK in %.1f s" % (time.monotonic() - t0))
    else:
        print("[5/6] Erase SKIPPED (--no-erase)")

    nblocks = (len(image) + BLOCK - 1) // BLOCK
    print("[6/6] Write Memory: %d bytes in %d blocks of <=%d B"
          % (len(image), nblocks, BLOCK))
    t0 = time.monotonic()
    written = 0
    for i in range(nblocks):
        chunk = image[i * BLOCK:(i + 1) * BLOCK]
        chunk = pad_to_word(chunk)
        addr = FLASH_BASE + i * BLOCK
        cmd_write_memory(fd, addr, chunk)
        written += len(chunk)
        if (i + 1) % 16 == 0 or i + 1 == nblocks:
            elapsed = time.monotonic() - t0
            rate = written / max(elapsed, 1e-3)
            print("       block %4d/%4d  %6d B  %.1f s  %.0f B/s"
                  % (i + 1, nblocks, written, elapsed, rate))
    print("       write OK in %.1f s" % (time.monotonic() - t0))

    if do_verify:
        print("[+]  Verify (read-back compare)")
        t0 = time.monotonic()
        mismatches = 0
        for i in range(nblocks):
            expected = pad_to_word(image[i * BLOCK:(i + 1) * BLOCK])
            addr = FLASH_BASE + i * BLOCK
            got = cmd_read_memory(fd, addr, len(expected))
            if got != expected:
                mismatches += 1
                bad = next((j for j in range(len(expected)) if got[j] != expected[j]), -1)
                print("       MISMATCH block %d @ 0x%08X offset %d: got %02X expected %02X"
                      % (i, addr, bad, got[bad] if bad >= 0 else 0,
                         expected[bad] if bad >= 0 else 0))
        if mismatches:
            raise RuntimeError("verify failed: %d block mismatch(es)" % mismatches)
        print("       verify OK in %.1f s" % (time.monotonic() - t0))

    print("DONE. Drop BOOT0 (release PA_11) and pulse NRST (PF_4) "
          "from the openocd holder shell to boot the new firmware.")


def main():
    ap = argparse.ArgumentParser(description="Stdlib AN3155 flasher for STM32L072")
    ap.add_argument("port", help="serial device, e.g. /dev/ttymxc3")
    ap.add_argument("image", help="binary image to flash, e.g. mlm32l07x01.bin")
    ap.add_argument("--no-erase", action="store_true", help="skip mass erase")
    ap.add_argument("--verify", action="store_true", help="read-back verify after write")
    args = ap.parse_args()

    with open(args.image, "rb") as f:
        image = f.read()
    print("loaded %s: %d bytes" % (args.image, len(image)))
    print("(Caller MUST have run: stty -F %s 19200 cs8 parenb -parodd -cstopb raw -echo)"
          % args.port)

    fd = open_uart(args.port)
    try:
        flash(fd, image, do_erase=not args.no_erase, do_verify=args.verify)
    finally:
        os.close(fd)


if __name__ == "__main__":
    try:
        main()
    except (RuntimeError, TimeoutError, ValueError) as e:
        print("FATAL: %s" % e, file=sys.stderr)
        sys.exit(1)
