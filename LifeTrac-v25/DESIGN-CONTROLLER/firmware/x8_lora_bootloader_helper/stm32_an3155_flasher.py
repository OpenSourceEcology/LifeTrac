#!/usr/bin/env python3
import os
import sys
import time
import struct

# Portenta X8 LmP Python 3.10 ships without `termios` (see repo memory
# lifetrac-x8-python-stripped). The bash wrapper (run_flash_l072.sh)
# already runs `stty -F /dev/ttymxc3 19200 cs8 parenb -parodd -cstopb
# raw -echo` before invoking us, so termios.tcsetattr() here is
# redundant on that platform. Make it optional so the flasher runs on
# both stripped (X8) and full (dev workstation) Python stdlib.
try:
    import termios  # type: ignore[import-not-found]
    _HAVE_TERMIOS = True
except ImportError:
    termios = None  # type: ignore[assignment]
    _HAVE_TERMIOS = False

PORT = "/dev/ttymxc3"
BAUD = termios.B19200 if _HAVE_TERMIOS else 0
ACK = b'\x79'
NACK = b'\x1f'

def open_port():
    print(f"Opening {PORT} at 19200 8E1...")
    fd = os.open(PORT, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)

    if _HAVE_TERMIOS:
        # Configure termios
        # 8E1: CS8, PARENB (parity enable), INPCK (input parity checking). CLEAR PARODD for Even.
        iflag, oflag, cflag, lflag, ispeed, ospeed, cc = termios.tcgetattr(fd)

        cflag &= ~(termios.PARENB | termios.PARODD | termios.CSIZE | termios.CSTOPB | termios.CRTSCTS)
        cflag |= (termios.CS8 | termios.PARENB | termios.CREAD | termios.CLOCAL)

        iflag &= ~(termios.IXON | termios.IXOFF | termios.IXANY | termios.INLCR | termios.IGNCR | termios.ICRNL)
        oflag &= ~termios.OPOST
        lflag &= ~(termios.ICANON | termios.ECHO | termios.ECHOE | termios.ISIG)

        termios.tcsetattr(fd, termios.TCSANOW, [iflag, oflag, cflag, lflag, BAUD, BAUD, cc])
    else:
        # termios unavailable (e.g. Portenta X8 LmP stripped stdlib).
        # The bash wrapper run_flash_l072.sh runs `stty -F $DEV 19200
        # cs8 parenb -parodd -cstopb raw -echo` before calling us, so
        # the port is already correctly configured. Proceed.
        print("  (termios unavailable; relying on prior stty configuration)")

    # Clear NOCTTY/NONBLOCK for blocking IO with timeout
    os.set_blocking(fd, False)
    return fd

def read_exact(fd, length, timeout=5.0): # Default extended slightly for erase wait
    start = time.time()
    data = b''
    while len(data) < length:
        if time.time() - start > timeout:
            raise TimeoutError(f"Timeout waiting for {length} bytes. Got {len(data)}")
        try:
            chunk = os.read(fd, length - len(data))
            if chunk:
                data += chunk
        except BlockingIOError:
            time.sleep(0.01)
    return data

def write_cmd(fd, data):
    os.write(fd, data)

def wait_ack(fd, timeout=5.0):
    try:
        res = read_exact(fd, 1, timeout=timeout)
        if res == ACK:
            return True
        elif res == NACK:
            print("Received NACK!")
            return False
        else:
            print(f"Unexpected response: {res.hex()}")
            return False
    except TimeoutError:
        print("Timeout waiting for ACK")
        return False

def sync(fd):
    print("Sending sync byte 0x7F...")
    write_cmd(fd, b'\x7F')
    return wait_ack(fd, timeout=2.0)

def get_command(fd):
    print("Sending GET command (0x00 0xFF)...")
    write_cmd(fd, b'\x00\xFF')
    if wait_ack(fd):
        n = ord(read_exact(fd, 1))
        version = ord(read_exact(fd, 1))
        print(f"Bootloader version: {version:02X}")
        cmds = read_exact(fd, n)
        wait_ack(fd)
        return cmds
    return None

def xor_checksum(data):
    c = 0
    for b in data:
        c ^= b
    return c

def erase_extended_pages(fd, num_pages):
    """Page-by-page Extended Erase. Required on STM32L0 ROM where mass erase
    (0xFFFF) is unreliable on a clean/RDP=0 chip."""
    if num_pages < 1 or num_pages > 0xFFFB:
        print(f"  page count out of range: {num_pages}")
        return False
    # N is (number_of_pages - 1) as a big-endian halfword
    n_minus_1 = num_pages - 1
    payload = bytes([(n_minus_1 >> 8) & 0xFF, n_minus_1 & 0xFF])
    for p in range(num_pages):
        payload += bytes([(p >> 8) & 0xFF, p & 0xFF])
    chk = xor_checksum(payload)
    payload += bytes([chk])
    write_cmd(fd, b'\x44\xBB')
    if not wait_ack(fd, timeout=2.0):
        print("  Extended Erase command NACK on cmd byte")
        return False
    write_cmd(fd, payload)
    # L0 erases ~3-4 ms per 128B page; budget generously.
    return wait_ack(fd, timeout=max(30.0, num_pages * 0.05))

def erase_memory(fd, firmware_len=None):
    print("Erasing memory... (This may take up to 60 seconds)")
    # Try Extended Erase mass-erase (0xFFFF) first — works when RDP just
    # transitioned 1->0, fast path for some devices.
    write_cmd(fd, b'\x44\xBB')
    if wait_ack(fd, timeout=2.0):
        write_cmd(fd, b'\xFF\xFF\x00')
        if wait_ack(fd, timeout=30.0):
            print("Extended Erase complete! (mass-erase via 0xFFFF)")
            return True
        # Mass erase NACK'd — fall through to page-by-page.
        print("  mass-erase parameter NACK; falling back to page-by-page erase")
        # Drain any stray bytes before next command.
        try:
            while os.read(fd, 64):
                pass
        except BlockingIOError:
            pass
        # STM32L072 page size is 128 bytes; erase enough pages to cover
        # the firmware (rounded up). Default to 256 pages (32KB) if size
        # is unknown — comfortably covers our 16592B builds.
        L072_PAGE_SIZE = 128
        if firmware_len is not None:
            num_pages = (firmware_len + L072_PAGE_SIZE - 1) // L072_PAGE_SIZE
        else:
            num_pages = 256
        print(f"  page-by-page Extended Erase: {num_pages} pages "
              f"({num_pages * L072_PAGE_SIZE} bytes)")
        if erase_extended_pages(fd, num_pages):
            print(f"Extended Erase complete! ({num_pages} page batches complete via page-by-page fallback)")
            return True
    else:
        print("Extended Erase not supported or failed. Attempting standard Erase (0x43)...")
        try:
            os.read(fd, 1024)
        except Exception:
            pass
        write_cmd(fd, b'\x43\xBC')
        if wait_ack(fd, timeout=2.0):
            write_cmd(fd, b'\xFF\x00')
            if wait_ack(fd, timeout=25.0):
                print("Standard Erase complete!")
                return True

    print("Erase failed!")
    return False

def write_memory(fd, address, data):
    length = len(data)
    if length == 0 or length > 256:
        print(f"Invalid write block length: {length}")
        return False
        
    write_cmd(fd, b'\x31\xCE')
    if not wait_ack(fd, timeout=2.0):
        return False
        
    addr_bytes = struct.pack('>I', address)
    chk = xor_checksum(addr_bytes)
    write_cmd(fd, addr_bytes + bytes([chk]))
    if not wait_ack(fd, timeout=2.0):
        return False
        
    n_minus_1 = length - 1
    payload = bytes([n_minus_1]) + data
    chk = xor_checksum(payload)
    write_cmd(fd, payload + bytes([chk]))
    
    if not wait_ack(fd, timeout=2.0):
        return False
        
    return True

def flash_file(fd, filepath, start_address=0x08000000):
    print(f"Reading payload {filepath}...")
    try:
        with open(filepath, 'rb') as f:
            firmware = f.read()
    except Exception as e:
        print(f"Failed to open file: {e}")
        return False
        
    firmware_len = len(firmware)
    print(f"Firmware size: {firmware_len} bytes")
    
    # 1. Erase
    if not erase_memory(fd, firmware_len=firmware_len):
        return False
        
    # 2. Flash
    print("Starting flash process...")
    start_time = time.time()
    
    address = start_address
    offset = 0
    
    while offset < firmware_len:
        end = min(offset + 256, firmware_len)
        chunk = firmware[offset:end]
        
        # STM32 ROM bootloader requires writes to be multiple of 4 bytes
        if len(chunk) % 4 != 0:
            chunk += b'\xFF' * (4 - (len(chunk) % 4))
            
        retry = 0
        success = False
        while retry < 3 and not success:
            if write_memory(fd, address, chunk):
                success = True
            else:
                print(f"\nWrite failed at offset {offset:08X}, retrying...")
                retry += 1
                # Try to re-sync buffer
                try:
                    os.read(fd, 1024)
                except:
                    pass
                
        if not success:
            print(f"\nFailed to write at address 0x{address:08X} after 3 retries.")
            return False
            
        address += len(chunk)
        offset += len(chunk)
        
        # Primitive progress bar
        progress = offset / firmware_len
        bar_len = 40
        filled = int(progress * bar_len)
        bar = '=' * filled + '-' * (bar_len - filled)
        sys.stdout.write(f"\r[{bar}] {min(100.0, progress*100):.1f}% (0x{offset:X} / 0x{firmware_len:X})")
        sys.stdout.flush()
        
    elapsed = time.time() - start_time
    if elapsed == 0: elapsed = 0.1
    print(f"\n\nFlash complete in {elapsed:.1f} seconds! ({firmware_len / elapsed / 1024:.1f} KB/s)")
    return True

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print(f"Usage: {sys.argv[0]} <firmware.bin>")
        sys.exit(1)

    filepath = sys.argv[1]
    fd = open_port()
    exit_code = 1

    try:
        # Drain any pending RX
        try:
            os.read(fd, 1024)
        except:
            pass

        print("Connecting to Bootloader...")
        synced = sync(fd)

        if not synced:
            print("Sync failed (got NACK or Timeout). The autobaud might already be locked.")
            print("Attempting to proceed anyway...")

        if get_command(fd) is not None:
            try:
                flash_file(fd, filepath, 0x08000000)
                exit_code = 0
            except Exception as ex:
                print(f"flash_file raised: {ex}")
                exit_code = 1
        else:
            print("Failed to communicate with the STM32 Bootloader. Reset the Murata chip and try again.")
            exit_code = 1

    finally:
        os.close(fd)

    sys.exit(exit_code)
