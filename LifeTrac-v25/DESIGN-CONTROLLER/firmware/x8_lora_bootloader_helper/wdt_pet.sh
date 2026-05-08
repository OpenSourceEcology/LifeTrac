#!/bin/bash
# wdt_pet.sh — keep /dev/watchdog0 alive during long-running operations
# (e.g. MKRWAN flash which takes ~62 s, exceeding the 60 s imx2+ WDT timeout).
#
# Usage:
#   bash wdt_pet.sh start        # spawns a background petter, writes PID to /tmp/lifetrac_p0c/wdt_pet.pid
#   bash wdt_pet.sh stop         # kills the petter
#   bash wdt_pet.sh status       # prints whether petter is running
#
# Why this is needed:
#   When openocd halts H7, the SPI/x8h7 path stalls and the kernel
#   [watchdogd] thread that normally pets /dev/watchdog0 either gets
#   blocked or stops scheduling for long enough that the 60 s
#   imx2+ HW watchdog fires and reboots the i.MX. A userspace petter
#   takes over while the bridge is offline.
#
# NOTE: imx2+ watchdog does NOT support the magic-close 'V' character
# (`wdctl` reports MAGICCLOSE STATUS=0). So once we open /dev/watchdog0
# we MUST keep petting it; closing without 'V' is harmless because the
# kernel [watchdogd] resumes feeding once the bridge is back.

set -u
PIDFILE=/tmp/lifetrac_p0c/wdt_pet.pid

case "${1:-}" in
  start)
    if [ -e "$PIDFILE" ] && kill -0 "$(cat $PIDFILE)" 2>/dev/null; then
      echo "wdt_pet already running pid=$(cat $PIDFILE)"
      exit 0
    fi
    # Background loop: open /dev/watchdog0, pet every 20 s.
    # We use a heredoc to keep the python in this single file.
    nohup python3 -u -c "
import os, time, signal, sys
def bye(*a):
    os._exit(0)
signal.signal(signal.SIGTERM, bye)
signal.signal(signal.SIGINT,  bye)
fd = os.open('/dev/watchdog0', os.O_WRONLY)
try:
    while True:
        os.write(fd, b'\\0')
        time.sleep(20)
finally:
    os.close(fd)
" > /tmp/lifetrac_p0c/wdt_pet.log 2>&1 &
    echo $! > "$PIDFILE"
    sleep 0.3
    if kill -0 "$(cat $PIDFILE)" 2>/dev/null; then
      echo "wdt_pet started pid=$(cat $PIDFILE)"
      exit 0
    else
      echo "wdt_pet failed to start; see /tmp/lifetrac_p0c/wdt_pet.log"
      cat /tmp/lifetrac_p0c/wdt_pet.log 2>/dev/null
      exit 2
    fi
    ;;
  stop)
    if [ -e "$PIDFILE" ]; then
      PID=$(cat "$PIDFILE")
      kill -TERM "$PID" 2>/dev/null && echo "wdt_pet stopped pid=$PID"
      rm -f "$PIDFILE"
    else
      echo "wdt_pet not running"
    fi
    ;;
  status)
    if [ -e "$PIDFILE" ] && kill -0 "$(cat $PIDFILE)" 2>/dev/null; then
      echo "wdt_pet running pid=$(cat $PIDFILE)"
    else
      echo "wdt_pet NOT running"
    fi
    ;;
  *)
    echo "usage: $0 {start|stop|status}"
    exit 1
    ;;
esac
