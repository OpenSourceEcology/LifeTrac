#!/usr/bin/env python3
"""
l072_unprotect.py

Removes flash WRITE_PROTECT and (if set) READOUT_PROTECT on the Murata
CMWX1ZZABZ-078 (STM32L072) via AN3155 ROM bootloader commands on /dev/ttymxc3.

PRECONDITION: caller has already driven BOOT0 (H7 PA11) HIGH and pulsed NRST
(H7 PF4) low->high so the L072 ROM bootloader is listening at 19200 8E1, AND
the caller is continuing to assert PA11 HIGH for the duration of this script
(because both WRITE_UNPROTECT and READOUT_UNPROTECT trigger a system reset on
completion, after which the chip re-enters ROM only if BOOT0 is still HIGH).

Sequence per AN3155 §3:
  1. Send 0x7F -> ACK            (autobaud / sync)
  2. WRITE_UNPROTECT (0x73 0x8C) -> ACK -> ACK -> [system reset]
  3. Re-sync 0x7F -> ACK
  4. READOUT_UNPROTECT (0x92 0x6D) -> ACK -> ACK -> [system reset + mass erase
     if RDP was non-zero]
  5. Re-sync 0x7F -> ACK to prove chip is back in ROM, unprotected.

Exit codes:
  0 = unprotect completed (resync OK after step 4)
  1 = WRITE_UNPROTECT failed
  2 = READOUT_UNPROTECT failed
  3 = final resync failed
  4 = initial sync failed
"""
import os
import sys
import time
import termios

PORT = "/dev/ttymxc3"
BAUD = termios.B19200
ACK = b'\x79'
NACK = b'\x1f'


def open_port():
    fd = os.open(PORT, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    iflag, oflag, cflag, lflag, ispeed, ospeed, cc = termios.tcgetattr(fd)
    cflag &= ~(termios.PARENB | termios.PARODD | termios.CSIZE |
               termios.CSTOPB | termios.CRTSCTS)
    cflag |= (termios.CS8 | termios.PARENB | termios.CREAD | termios.CLOCAL)
    iflag &= ~(termios.IXON | termios.IXOFF | termios.IXANY |
               termios.INLCR | termios.IGNCR | termios.ICRNL)
    oflag &= ~termios.OPOST
    lflag &= ~(termios.ICANON | termios.ECHO | termios.ECHOE | termios.ISIG)
    termios.tcsetattr(fd, termios.TCSANOW,
                      [iflag, oflag, cflag, lflag, BAUD, BAUD, cc])
    os.set_blocking(fd, False)
    return fd


def drain(fd, dur=0.2):
    end = time.time() + dur
    while time.time() < end:
        try:
            chunk = os.read(fd, 256)
            if not chunk:
                time.sleep(0.01)
        except BlockingIOError:
            time.sleep(0.01)


def read_one(fd, timeout):
    end = time.time() + timeout
    while time.time() < end:
        try:
            b = os.read(fd, 1)
            if b:
                return b
        except BlockingIOError:
            pass
        time.sleep(0.005)
    return b''


def wait_ack(fd, label, timeout=5.0):
    b = read_one(fd, timeout)
    if b == ACK:
        print(f"  [{label}] ACK")
        return True
    if b == NACK:
        print(f"  [{label}] NACK")
        return False
    print(f"  [{label}] unexpected={b.hex() if b else 'TIMEOUT'}")
    return False


def sync(fd, label="sync", attempts=8):
    for i in range(1, attempts + 1):
        drain(fd, 0.05)
        os.write(fd, b'\x7F')
        b = read_one(fd, 1.5)
        if b == ACK:
            print(f"  [{label}#{i}] 0x7F -> ACK (0x79)")
            return True
        # ROM may NACK if it was already synced; treat as already-in-sync.
        if b == NACK:
            print(f"  [{label}#{i}] 0x7F -> NACK (already initialised) - OK")
            return True
        print(f"  [{label}#{i}] 0x7F -> {b.hex() if b else 'TIMEOUT'}")
        time.sleep(0.3)
    return False


def write_unprotect(fd):
    print("--- WRITE_UNPROTECT (0x73 0x8C) ---")
    os.write(fd, b'\x73\x8C')
    if not wait_ack(fd, "WU.cmd", timeout=2.0):
        return False
    # Bootloader executes unprotect (a few ms - tens of ms) then sends 2nd ACK.
    if not wait_ack(fd, "WU.done", timeout=5.0):
        return False
    print("  WRITE_UNPROTECT issued; chip resetting...")
    # Give the L072 time to come back through ROM reset (BOOT0 must be HIGH).
    time.sleep(0.5)
    return True


def readout_unprotect(fd):
    print("--- READOUT_UNPROTECT (0x92 0x6D) ---")
    os.write(fd, b'\x92\x6D')
    if not wait_ack(fd, "RU.cmd", timeout=2.0):
        return False
    # If RDP was level 1, this triggers a full mass erase -> can take seconds.
    if not wait_ack(fd, "RU.done", timeout=30.0):
        return False
    print("  READOUT_UNPROTECT issued; chip resetting (mass erase if RDP>0)...")
    time.sleep(1.0)
    return True


def main():
    print(f"Opening {PORT} at 19200 8E1 ...")
    fd = open_port()
    try:
        print("=== Step 1: initial sync ===")
        if not sync(fd, "init"):
            print("ERROR: initial 0x7F sync failed")
            return 4

        print("=== Step 2: WRITE_UNPROTECT ===")
        if not write_unprotect(fd):
            print("ERROR: WRITE_UNPROTECT failed")
            return 1

        print("=== Step 3: re-sync post WRITE_UNPROTECT reset ===")
        # The chip resets; UART glitches may appear. Drain then resync.
        drain(fd, 0.5)
        if not sync(fd, "post-WU", attempts=10):
            print("ERROR: post WRITE_UNPROTECT resync failed")
            return 3

        print("=== Step 4: READOUT_UNPROTECT ===")
        if not readout_unprotect(fd):
            print("ERROR: READOUT_UNPROTECT failed")
            return 2

        print("=== Step 5: final resync ===")
        drain(fd, 0.5)
        if not sync(fd, "post-RU", attempts=15):
            print("ERROR: final resync failed (chip may be unprotected but not"
                  " back in ROM - verify BOOT0 is being held HIGH)")
            return 3

        print("OK: WRITE_UNPROTECT and READOUT_UNPROTECT completed; L072 is in"
              " ROM, unprotected, ready for Extended Erase.")
        return 0
    finally:
        try:
            os.close(fd)
        except Exception:
            pass


if __name__ == "__main__":
    sys.exit(main())
