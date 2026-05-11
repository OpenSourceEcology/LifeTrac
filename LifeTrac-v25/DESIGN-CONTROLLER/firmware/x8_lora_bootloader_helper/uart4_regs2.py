#!/usr/bin/env python3
import os, struct

UART4_BASE = 0x30A60000

def rd(fd, off):
    os.lseek(fd, UART4_BASE + off, 0)
    return struct.unpack("<I", os.read(fd, 4))[0]

def bit(v, n):
    return (v >> n) & 1

fd = os.open("/dev/mem", os.O_RDONLY | os.O_SYNC)

ucr1 = rd(fd, 0x80)
ucr2 = rd(fd, 0x84)
ucr4 = rd(fd, 0x8C)
ufcr = rd(fd, 0x90)
uts  = rd(fd, 0xB4)
ubir = rd(fd, 0xA4)
ubmr = rd(fd, 0xA8)

os.close(fd)

print("=== i.MX8MM UART4 Registers ===")
print(f"UCR1=0x{ucr1:08X}  UARTEN={bit(ucr1,0)}")
print(f"UCR2=0x{ucr2:08X}  RXEN={bit(ucr2,1)} TXEN={bit(ucr2,2)} CTS_pin={bit(ucr2,12)} CTSC={bit(ucr2,13)} IRTS={bit(ucr2,14)}")
if bit(ucr2, 13):
    print("  => CTS FLOW CONTROL ENABLED (CTSC=1) - TX gated by CTS pin")
    if not bit(ucr2, 12):
        print("  => CTS pin NOT asserted => TX IS BLOCKED")
    else:
        print("  => CTS pin asserted => TX allowed")
else:
    print("  => CTS flow control disabled (CTSC=0)")

if bit(ucr2, 14):
    print("  => IRTS=1: RX pin IGNORED (internal loopback or RS-485 dir ctrl)")
else:
    print("  => IRTS=0: RX connected to external pin (normal)")

print(f"UCR4=0x{ucr4:08X}  INVR={bit(ucr4,9)}")
print(f"UFCR=0x{ufcr:08X}  TXTL={(ufcr>>10)&0x3F}  RXTL={(ufcr>>0)&0x3F}  RFDIV={(ufcr>>7)&7}")
print(f"UTS =0x{uts:08X}   TXEMPTY={bit(uts,6)} RXEMPTY={bit(uts,5)} TXFULL={bit(uts,4)}")
print(f"UBIR=0x{ubir:08X}  UBMR=0x{ubmr:08X}")
rfdiv_tab = [6,5,4,3,2,1,7,7]
rfdiv = rfdiv_tab[(ufcr >> 7) & 7]
if ubmr > 0:
    baud_est = 80_000_000 // rfdiv * (ubir + 1) // ((ubmr + 1) * 16)
    print(f"  => Baud est @ 80MHz uart_clk, rfdiv={rfdiv}: {baud_est}")
print("=== end ===")
