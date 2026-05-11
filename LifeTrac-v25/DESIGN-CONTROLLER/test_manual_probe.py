#!/usr/bin/env python3
"""Manual probe test to capture diagnostics."""

import sys
import time
import subprocess
import serial
import struct
import os

# Check if firmware binary exists
fw_path = "firmware/murata_l072/build/firmware.bin"
if not os.path.exists(fw_path):
    print(f"ERROR: firmware not found at {fw_path}")
    sys.exit(1)

# Push firmware to X8
print("=== Pushing firmware ===")
adb_serial = "2E2C1209DABC240B"
result = subprocess.run([
    "adb", "-s", adb_serial, "push", fw_path, "/tmp/lifetrac_p0c/firmware.bin"
], capture_output=True, text=True)
print(result.stdout.strip() if result.stdout else result.stderr.strip())

# Flash via bootloader
print("\n=== Flashing via AN3155 bootloader ===")
flash_script = "firmware/x8_lora_bootloader_helper/stm32_an3155_flasher.py"
adb_flasher = "firmware/x8_lora_bootloader_helper/run_method_g_stage1.sh"

result = subprocess.run([
    "adb", "-s", adb_serial, "shell", f"bash /tmp/lifetrac_p0c/{adb_flasher}"
], capture_output=True, text=True, timeout=120)

flash_output = result.stdout + result.stderr
print(flash_output[-500:] if len(flash_output) > 500 else flash_output)

if "FATAL" in flash_output or result.returncode != 0:
    print(f"Flash failed (exit code {result.returncode})")
    sys.exit(1)

print("\n=== Waiting for firmware to boot ===")
time.sleep(2)

# Capture early bytes at 921600
print("\n=== Capturing early bytes at 921600 ===")
result = subprocess.run([
    "adb", "-s", adb_serial, "shell", """
    stty -F /dev/ttymxc3 921600
    timeout 2 dd if=/dev/ttymxc3 bs=1 count=512 2>/dev/null | xxd -g1 -u
    """
], capture_output=True, text=True, timeout=10)

print(result.stdout)
if result.stderr:
    print("STDERR:", result.stderr)

# Parse and show what we got
hex_output = result.stdout.strip()
if hex_output:
    print("\n=== Analysis ===")
    # Extract hex pairs
    lines = hex_output.split('\n')
    for line in lines:
        if ':' in line:
            parts = line.split(':')
            if len(parts) > 1:
                hex_part = parts[1].split('|')[0].strip()
                ascii_part = parts[1].split('|')[1] if '|' in parts[1] else ""
                print(f"Hex: {hex_part} | ASCII: {ascii_part}")

# Try to send VER_REQ manually
print("\n=== Attempting manual VER_REQ probe ===")
cmd = """
python3 << 'EOF'
import serial
import time
import struct

ser = serial.Serial('/dev/ttymxc3', 921600, timeout=0.5)
time.sleep(0.5)

# COBS encoded VER_REQ: type=0x01, flags=0, seq=1, payload_len=0
# Inner frame: [1, 1, 0, 0, 1, 0, crc_lo, crc_hi]
# CRC16-CCITT of [1, 1, 0, 0, 1, 0] = 0x....
crc_val = 0xCBC6  # pre-calculated for this frame
inner = bytes([1, 1, 0, 0, 1, 0, crc_val & 0xFF, (crc_val >> 8) & 0xFF])

# COBS encode
def cobs_encode(data):
    out = bytearray()
    block = bytearray()
    for byte in data:
        if byte == 0:
            out.append(len(block) + 1)
            out.extend(block)
            block = bytearray()
        else:
            block.append(byte)
    if block or not out:
        out.append(len(block) + 1)
        out.extend(block)
    return bytes(out)

encoded = cobs_encode(inner)
frame = b'\\x00' + encoded + b'\\x00'

print(f"Sending VER_REQ frame: {frame.hex()}")
ser.write(frame)

# Wait for response
time.sleep(0.5)
response = ser.read(256)
if response:
    print(f"Response: {response.hex()}")
    print(f"Response (ASCII): {response}")
else:
    print("No response")

ser.close()
EOF
"""

result = subprocess.run([
    "adb", "-s", adb_serial, "shell", cmd
], capture_output=True, text=True, timeout=10)

print(result.stdout)
if result.stderr:
    print("STDERR:", result.stderr)
