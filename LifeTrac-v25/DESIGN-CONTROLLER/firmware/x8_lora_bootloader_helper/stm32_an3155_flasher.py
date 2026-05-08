#!/usr/bin/env python3
import os
import sys
import time
import termios
import struct

PORT = "/dev/ttymxc3"
BAUD = termios.B19200
ACK = b'\x79'
NACK = b'\x1f'

def open_port():
    print(f"Opening {PORT} at 19200 8E1...")
    fd = os.open(PORT, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    
    # Configure termios
    # 8E1: CS8, PARENB (parity enable), INPCK (input parity checking). CLEAR PARODD for Even.
    iflag, oflag, cflag, lflag, ispeed, ospeed, cc = termios.tcgetattr(fd)
    
    cflag &= ~(termios.PARENB | termios.PARODD | termios.CSIZE | termios.CSTOPB | termios.CRTSCTS)
    cflag |= (termios.CS8 | termios.PARENB | termios.CREAD | termios.CLOCAL)
    
    iflag &= ~(termios.IXON | termios.IXOFF | termios.IXANY | termios.INLRCR | termios.IGNCR | termios.ICRNL)
    oflag &= ~termios.OPOST
    lflag &= ~(termios.ICANON | termios.ECHO | termios.ECHOE | termios.ISIG)
    
    termios.tcsetattr(fd, termios.TCSANOW, [iflag, oflag, cflag, lflag, BAUD, BAUD, cc])
    
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

def erase_memory(fd):
    print("Erasing memory... (This may take up to 20 seconds)")
    
    # We will try Extended Erase (0x44) first. STM32L0 often uses standard Erase (0x43)
    # but the bootloader may support both.
    write_cmd(fd, b'\x44\xBB')
    if wait_ack(fd, timeout=2.0):
        # Global Ext Erase command: 0xFF 0xFF 0x00
        write_cmd(fd, b'\xFF\xFF\x00')
        if wait_ack(fd, timeout=25.0):
            print("Extended Erase complete!")
            return True
    else:
        print("Extended Erase not supported or failed. Attempting standard Erase (0x43)...")
        # Clear buffer
        try:
            os.read(fd, 1024)
        except:
            pass
        
        write_cmd(fd, b'\x43\xBC')
        if wait_ack(fd, timeout=2.0):
            # Global Erase command: 0xFF 0x00
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
    if not erase_memory(fd):
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
            
        if get_command(fd) is not None:
            flash_file(fd, filepath, 0x08000000)
        else:
            print("Failed to communicate with the STM32 Bootloader. Reset the Murata chip and try again.")
            
    finally:
        os.close(fd)
