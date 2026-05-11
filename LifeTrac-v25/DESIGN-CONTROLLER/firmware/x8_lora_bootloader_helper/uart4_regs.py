#!/usr/bin/env python3
"""
uart4_regs.py - Dump i.MX8MM UART4 control registers from /dev/mem.

UART4 base address: 0x30A60000
UCR1 offset: 0x80   UCR2 offset: 0x84   UCR4 offset: 0x8C
UFCR offset: 0x90   UTS  offset: 0xB4

Run on X8 as root.  Prints register values with key bits decoded.
"""
import os
import mmap
import struct

UART4_BASE = 0x30A60000
PAGE_SIZE  = 4096
MAP_SIZE   = PAGE_SIZE

UCR1_OFF = 0x80
UCR2_OFF = 0x84
UCR4_OFF = 0x8C
UFCR_OFF = 0x90
UTS_OFF  = 0xB4
UBIR_OFF = 0xA4
UBMR_OFF = 0xA8
ONEMS_OFF= 0xB0

def rd(m, off):
    return struct.unpack("<I", m[off: off + 4])[0]

def bit(v, n):
    return (v >> n) & 1

fd = os.open("/dev/mem", os.O_RDONLY | os.O_SYNC)
m  = mmap.mmap(fd, MAP_SIZE, mmap.MAP_SHARED, mmap.PROT_READ,
               offset=UART4_BASE)

ucr1 = rd(m, UCR1_OFF)
ucr2 = rd(m, UCR2_OFF)
ucr4 = rd(m, UCR4_OFF)
ufcr = rd(m, UFCR_OFF)
uts  = rd(m, UTS_OFF)
ubir = rd(m, UBIR_OFF)
ubmr = rd(m, UBMR_OFF)
onems= rd(m, ONEMS_OFF)

m.close()
os.close(fd)

print(f"=== i.MX8MM UART4 Register Dump (base=0x{UART4_BASE:08X}) ===")
print(f"UCR1 0x{ucr1:08X}  UARTEN={bit(ucr1,0)}  TXMPTYEN={bit(ucr1,6)}  "
      f"TRDYEN={bit(ucr1,13)}  ADBR={bit(ucr1,14)}  ADEN={bit(ucr1,15)}")
print(f"UCR2 0x{ucr2:08X}  SRST={bit(ucr2,0)}  RXEN={bit(ucr2,1)}  "
      f"TXEN={bit(ucr2,2)}  ATEN={bit(ucr2,3)}  "
      f"CTS={bit(ucr2,12)}  CTSC={bit(ucr2,13)}  "
      f"IRTS={bit(ucr2,14)}  ESCI={bit(ucr2,15)}")
print(f"  => CTS flow ctrl: {'ENABLED (CTSC=1, TX gated by CTS pin)' if bit(ucr2,13) else 'disabled (CTSC=0)'}")
print(f"  => CTS pin state: {'asserted=OK to TX' if bit(ucr2,12) else 'NOT asserted => TX BLOCKED if CTSC=1'}")
print(f"  => Loopback (IRTS): {'RX disconnected from ext pin' if bit(ucr2,14) else 'RX from ext pin'}")
print(f"UCR4 0x{ucr4:08X}  INVR={bit(ucr4,9)}  DREN={bit(ucr4,0)}")
print(f"UFCR 0x{ufcr:08X}  TXTL={(ufcr>>10)&0x3F}  RXTL={(ufcr>>0)&0x3F}  "
      f"RFDIV={(ufcr>>7)&7}")
print(f"UTS  0x{uts:08X}  TXEMPTY={bit(uts,6)}  RXEMPTY={bit(uts,5)}  "
      f"TXFULL={bit(uts,4)}  RXFULL={bit(uts,3)}  LOOPIR={bit(uts,12)}")
print(f"UBIR 0x{ubir:08X}  UBMR 0x{ubmr:08X}  ONEMS 0x{onems:08X}")
if ubmr > 0:
    # Reference clock to module = rfdiv factor * module_clk
    # baud = ref_clk * (UBIR+1) / (UBMR+1) / 16
    # (approximation — actual ref_clk depends on device tree config)
    rfdiv_tab = [6,5,4,3,2,1,7,7]
    rfdiv = rfdiv_tab[(ufcr >> 7) & 7]
    # Assume 80 MHz uart_clk (common on i.MX8MM)
    uart_clk_hz = 80_000_000
    ref_clk = uart_clk_hz // rfdiv
    baud_est = ref_clk * (ubir + 1) // ((ubmr + 1) * 16)
    print(f"  => Estimated baud (assuming 80 MHz uart_clk, rfdiv={rfdiv}): {baud_est}")
print("=== end ===")
