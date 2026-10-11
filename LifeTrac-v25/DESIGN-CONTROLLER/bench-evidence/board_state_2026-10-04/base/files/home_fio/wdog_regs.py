# Read-only dump of i.MX8MM WDOG1 registers via /dev/mem (root).
# WCR@0x00: WDE bit2 (enabled), WT bits15:8 (timeout = (WT+1)/2 s)
# WSR@0x02 service reg; WRSR@0x04: SFTW bit0, TOUT bit1, POR bit4
import mmap, os, struct, sys
BASE = 0x30280000
fd = os.open("/dev/mem", os.O_RDONLY | os.O_SYNC)
m = mmap.mmap(fd, 0x1000, mmap.MAP_SHARED, mmap.PROT_READ, offset=BASE)
def r16(off):
    return struct.unpack("<H", m[off:off+2])[0]
wcr, wsr, wrsr, wicr, wmcr = r16(0), r16(2), r16(4), r16(6), r16(8)
wt = (wcr >> 8) & 0xFF
print("WDOG1 WCR=0x%04x WDE=%d WT=%d (%.1f s) WDT=%d WDZST=%d WDBG=%d | WSR=0x%04x | WRSR=0x%04x SFTW=%d TOUT=%d POR=%d | WICR=0x%04x WMCR=0x%04x" % (
    wcr, (wcr >> 2) & 1, wt, (wt + 1) / 2.0, (wcr >> 3) & 1, wcr & 1, (wcr >> 1) & 1,
    wsr, wrsr, wrsr & 1, (wrsr >> 1) & 1, (wrsr >> 4) & 1, wicr, wmcr))
m.close(); os.close(fd)
