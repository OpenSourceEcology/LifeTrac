# wdog_regs.py -- READ-ONLY dump of the i.MX8M Mini WDOG1 registers.
#
# What it reads: WDOG1 at physical 0x30280000 through /dev/mem, opened
# O_RDONLY and mapped PROT_READ -- it writes nothing and cannot pet, stop or
# arm the watchdog. One line out:
#   WCR  (0x00) WDE bit2 = enabled, WT bits15:8 -> timeout (WT+1)/2 s,
#               WDT bit3 = WDOG_B asserted on timeout (PMIC reset on the X8),
#               WDZST bit0, WDBG bit1
#   WSR  (0x02) service register
#   WRSR (0x04) reset reason: SFTW bit0, TOUT bit1, POR bit4
#   WICR (0x06), WMCR (0x08)
# Expected on both bench boards (u-boot arms it, FLASH_RUNBOOK section 3):
#   WCR=0x773d -> WDE=1 WT=119 (60.0 s), WDT=1. WRSR reads POR even after a
#   watchdog reset, because WDOG_B power-cycles the board through the PMIC --
#   so WRSR cannot tell a watchdog reboot from a power-on.
#
# Where it runs: ON THE BOARD, inside a docker image, because the LmP host
# python has no `mmap` module and the image has no `devmem`/sysfs watchdog
# attributes. Needs root and the /dev/mem device:
#   base (ssh):     echo fio | sudo -S -p '' docker run --rm --privileged \
#                     -v /dev/mem:/dev/mem -v /tmp/lifetrac_strict:/work \
#                     --entrypoint python3 lifetrac-v25:latest /work/wdog_regs.py
#   tractor (adb):  same, with image
#                     hub.foundries.io/arduino/arduino-ootb-python-devel:738bc44
# (push this file to /tmp/lifetrac_strict first; /tmp is tmpfs).
#
# Provenance: until 2026-10-10 this script existed only in /home/fio on the
# two boards (identical copies). Recovered from the read-only captures
# bench-evidence/board_state_2026-10-04/base/files/home_fio/wdog_regs.py and
# board_state_2026-10-10/tractor/; the code below is unchanged.
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
