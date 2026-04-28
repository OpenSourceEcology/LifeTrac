#!/usr/bin/env python3
"""
LifeTrac v25 — tractor X8 time service.

Disciplines the system clock from the u-blox NEO-M9N PPS pin (when present)
and broadcasts the disciplined time-of-day to the H747 M7 over the X8↔H747
UART so the M7 RTC stays within ~1 ms of GPS time.

Per MASTER_PLAN.md §8.10 (X8 Linux sidecar role) and §8.17 (FHSS hop counter
needs synchronised time-of-day across base + tractor + handheld).

Hardware contract:
  * /dev/pps0    — Linux PPS API device (CONFIG_PPS_CLIENT_GPIO).
  * /dev/ttymxc0 — X8↔H747 UART, 921 600 8N1; M7 reads TIME_SYNC ticks.

Wire-format of the 1 Hz tick (little-endian, 6 bytes):
    [u32 unix_seconds | u16 millis_since_second]

Discipline policy:
  * If /dev/pps0 is available, run a tight loop reading PPS edges via the
    PPS_FETCH ioctl. Each edge yields the kernel-stamped timestamp.
  * If /dev/pps0 is missing (bench / no GPS yet), fall back to a plain
    1 Hz tick using CLOCK_REALTIME — still useful so the M7 stays near
    wall-clock time for log correlation.

Safety:
  * UART writes are best-effort; a write failure logs and continues so a
    transient cable wiggle does not crash the daemon.
  * No clock stepping is performed here — chrony/ntpd handle cold-start
    sync; this service only emits the periodic tick.
"""

from __future__ import annotations

import errno
import fcntl
import logging
import os
import struct
import sys
import time
from pathlib import Path

LOG = logging.getLogger("time_service")

PPS_DEVICE  = Path(os.environ.get("LIFETRAC_PPS_DEVICE",  "/dev/pps0"))
UART_DEVICE = Path(os.environ.get("LIFETRAC_TIME_UART",   "/dev/ttymxc0"))

# struct pps_fdata layout from <linux/pps.h>: two pps_ktime (16 B each),
# two u32 sequence numbers, then a timeout pps_ktime (also 16 B).
_PPS_KTIME = struct.Struct("=qiI")           # sec(i64) + nsec(i32) + flags(u32)
_PPS_FDATA = struct.Struct("=16s16sII16s")

# PPS_FETCH = _IOWR('p', 0xa4, struct pps_fdata) on x86_64.
# Stable across kernels because magic 'p' (0x70) and dir/size are fixed.
PPS_FETCH = 0xC038_7003


def _write_tick(uart_fd: int, sec: int, ms: int) -> None:
    payload = struct.pack("<IH", sec & 0xFFFFFFFF, ms & 0xFFFF)
    try:
        os.write(uart_fd, payload)
    except OSError as exc:
        LOG.warning("time_service: UART write failed: %s", exc)


def _open_uart() -> int | None:
    if not UART_DEVICE.exists():
        LOG.warning("time_service: UART %s missing; running tick-only mode", UART_DEVICE)
        return None
    try:
        return os.open(str(UART_DEVICE), os.O_WRONLY | os.O_NOCTTY)
    except OSError as exc:
        LOG.warning("time_service: cannot open UART %s: %s", UART_DEVICE, exc)
        return None


def _open_pps() -> int | None:
    if not PPS_DEVICE.exists():
        LOG.info("time_service: %s missing — falling back to wall-clock tick", PPS_DEVICE)
        return None
    try:
        return os.open(str(PPS_DEVICE), os.O_RDWR)
    except OSError as exc:
        LOG.warning("time_service: cannot open PPS %s: %s", PPS_DEVICE, exc)
        return None


def _wait_pps_edge(pps_fd: int) -> tuple[int, int] | None:
    """Block on PPS_FETCH; return (sec, nsec) of the assert edge, or None."""
    buf = bytearray(_PPS_FDATA.size)
    try:
        fcntl.ioctl(pps_fd, PPS_FETCH, buf, True)
    except OSError as exc:
        if exc.errno == errno.EINTR:
            return None
        LOG.warning("time_service: PPS_FETCH failed: %s", exc)
        return None
    assert_tu, _, _, _, _ = _PPS_FDATA.unpack(bytes(buf))
    sec, nsec, _flags = _PPS_KTIME.unpack(assert_tu)
    return sec, nsec


def run() -> int:
    logging.basicConfig(level=logging.INFO,
                        format="%(asctime)s %(levelname)s %(message)s")
    uart_fd = _open_uart()
    pps_fd  = _open_pps()
    LOG.info("time_service: starting (uart=%s, pps=%s)",
             "yes" if uart_fd is not None else "no",
             "yes" if pps_fd  is not None else "no")
    try:
        while True:
            if pps_fd is not None:
                edge = _wait_pps_edge(pps_fd)
                if edge is None:
                    continue
                sec, nsec = edge
                ms = nsec // 1_000_000
            else:
                now = time.time()
                time.sleep(max(0.0, 1.0 - (now - int(now))))
                sec = int(time.time())
                ms = 0
            if uart_fd is not None:
                _write_tick(uart_fd, sec, ms)
            LOG.debug("tick %d.%03d", sec, ms)
    except KeyboardInterrupt:
        LOG.info("time_service: shutting down")
    finally:
        if uart_fd is not None:
            os.close(uart_fd)
        if pps_fd is not None:
            os.close(pps_fd)
    return 0


if __name__ == "__main__":
    sys.exit(run())
