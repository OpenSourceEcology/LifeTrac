#!/usr/bin/env python3
import os
import sys
import time
import struct

PORT = "/dev/ttymxc3"
ACK = b'\x79'
NACK = b'\x1f'
FLASH_BASE = 0x08000000
FLASH_PAGE_SIZE = 128
ERASE_CHUNK_PAGES = 64

def open_port():
    """
    Open /dev/ttymxc3 for raw I/O. UART is pre-configured by stty.
    Note: This script assumes stty has already configured the port to 19200 8E1 raw.
    """
    print(f"Opening {PORT} (stty-preconfigured)...")
    # Open with O_NOCTTY to prevent TTY control, O_NONBLOCK for async reads
    fd = os.open(PORT, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
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

def page_numbers_for_span(start_address, length):
    if length <= 0:
        return []

    first_page = (start_address - FLASH_BASE) // FLASH_PAGE_SIZE
    last_address = start_address + length - 1
    last_page = (last_address - FLASH_BASE) // FLASH_PAGE_SIZE
    return list(range(first_page, last_page + 1))

def erase_pages_extended(fd, pages):
    for index in range(0, len(pages), ERASE_CHUNK_PAGES):
        batch = pages[index:index + ERASE_CHUNK_PAGES]
        print(f"Trying Extended Erase page batch {batch[0]}..{batch[-1]}...")
        write_cmd(fd, b'\x44\xBB')
        if not wait_ack(fd, timeout=2.0):
            print("Extended Erase command was NACKed during page fallback.")
            return False

        count_minus_1 = len(batch) - 1
        payload = bytearray(struct.pack('>H', count_minus_1))
        for page in batch:
            payload.extend(struct.pack('>H', page))
        payload.append(xor_checksum(payload))

        write_cmd(fd, bytes(payload))
        if not wait_ack(fd, timeout=25.0):
            print("Extended Erase page payload was NACKed.")
            return False

    print("Extended Erase page batches complete!")
    return True

def erase_pages_standard(fd, pages):
    if not pages:
        return True
    if max(pages) > 0xFF:
        print("Standard Erase page fallback unavailable: page index exceeds 255.")
        return False

    for index in range(0, len(pages), ERASE_CHUNK_PAGES):
        batch = pages[index:index + ERASE_CHUNK_PAGES]
        print(f"Trying Standard Erase page batch {batch[0]}..{batch[-1]}...")
        write_cmd(fd, b'\x43\xBC')
        if not wait_ack(fd, timeout=2.0):
            print("Standard Erase command was NACKed during page fallback.")
            return False

        payload = bytearray([len(batch) - 1])
        payload.extend(batch)
        payload.append(xor_checksum(payload))

        write_cmd(fd, bytes(payload))
        if not wait_ack(fd, timeout=25.0):
            print("Standard Erase page payload was NACKed.")
            return False

    print("Standard Erase page batches complete!")
    return True

def erase_memory(fd, start_address, firmware_len):
    print("Erasing memory... (This may take up to 20 seconds)")
    pages = page_numbers_for_span(start_address, firmware_len)
    if pages:
        print(f"Firmware span covers flash pages {pages[0]}..{pages[-1]} ({len(pages)} pages @ {FLASH_PAGE_SIZE} bytes/page)")

    # We will try Extended Erase (0x44) first. STM32L0 often uses standard Erase (0x43)
    # but the bootloader may support both.
    write_cmd(fd, b'\x44\xBB')
    if wait_ack(fd, timeout=2.0):
        # Global Ext Erase command: 0xFF 0xFF 0x00
        write_cmd(fd, b'\xFF\xFF\x00')
        if wait_ack(fd, timeout=25.0):
            print("Extended Erase complete!")
            return True
        print("Extended Erase payload was NACKed. Trying Extended Erase page fallback...")
        if erase_pages_extended(fd, pages):
            return True
        print("Extended Erase page fallback failed. Falling back to standard Erase (0x43)...")
    else:
        print("Extended Erase not supported or failed. Attempting standard Erase (0x43)...")

    # Clear any leftover buffered bytes before retrying with standard erase.
    try:
        os.read(fd, 1024)
    except Exception:
        pass

    write_cmd(fd, b'\x43\xBC')
    if wait_ack(fd, timeout=2.0):
        # Global Erase command: 0xFF 0x00
        write_cmd(fd, b'\xFF\x00')
        if wait_ack(fd, timeout=25.0):
            print("Standard Erase complete!")
            return True
        print("Standard Erase payload was NACKed. Trying Standard Erase page fallback...")
        if erase_pages_standard(fd, pages):
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
    if not erase_memory(fd, start_address, firmware_len):
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

        rc = 1
        if get_command(fd) is not None:
            rc = 0 if flash_file(fd, filepath, 0x08000000) else 1
        else:
            print("Failed to communicate with the STM32 Bootloader. Reset the Murata chip and try again.")
            rc = 1

    finally:
        os.close(fd)

    sys.exit(rc)
