#!/bin/bash
# wdt_pet.sh — keep /dev/watchdog0 alive during long-running operations
# (e.g. an L072 flash which takes ~62 s, exceeding the 60 s imx2+ WDT timeout).
#
# Usage:
#   bash wdt_pet.sh start        # spawns a background petter, writes PID to /tmp/lifetrac_p0c/wdt_pet.pid
#   bash wdt_pet.sh stop         # asks the petter to hand the watchdog back and exit
#   bash wdt_pet.sh status       # prints whether petter is running
#
# 2026-09-12 CORRECTION (the previous header was wrong and cost a reboot):
#   The imx2+ driver DOES advertise WDIOF_MAGICCLOSE, and the kernel
#   watchdog core honours it. Closing /dev/watchdog0 WITHOUT first
#   writing 'V' leaves the watchdog "active" with no petter: the core
#   logs "watchdog did not stop!" and the hardware fires 60 s after the
#   last pet. The tractor (kernel 5.10.93) rebooted exactly that way on
#   2026-09-12 after a start/stop test. Writing 'V' before close hands
#   the still-running hardware watchdog back to the kernel keepalive
#   worker (WDOG_HW_RUNNING, handle_boot_enabled=Y), which pets it
#   indefinitely, exactly as it does from boot before anyone opens it.
#   The WDOG is enabled by u-boot on both boards (WCR.WDE=1, WT=60 s,
#   WDOG_B -> PMIC, so a timeout shows up as POR in WRSR, not TOUT).
#
# Every pet is logged with a monotonic timestamp and fsync'd to
# $WDT_PET_LOG (default /home/fio/wdt_pet.log — persistent, so the log
# survives the reboot it is meant to diagnose). /tmp is tmpfs.

set -u
PIDFILE=/tmp/lifetrac_p0c/wdt_pet.pid
LOGFILE=${WDT_PET_LOG:-/home/fio/wdt_pet.log}
PET_S=${WDT_PET_INTERVAL_S:-10}

case "${1:-}" in
  start)
    if [ -e "$PIDFILE" ] && kill -0 "$(cat $PIDFILE)" 2>/dev/null; then
      echo "wdt_pet already running pid=$(cat $PIDFILE)"
      exit 0
    fi
    nohup python3 -u - "$LOGFILE" "$PET_S" > /tmp/lifetrac_p0c/wdt_pet.stderr 2>&1 <<'PY' &
import os, sys, time, signal
logpath, pet_s = sys.argv[1], float(sys.argv[2])
log = open(logpath, "a")
def note(msg):
    log.write("%.3f %s\n" % (time.monotonic(), msg))
    log.flush(); os.fsync(log.fileno())
stop = False
def on_sig(*a):
    global stop
    stop = True
signal.signal(signal.SIGTERM, on_sig)
signal.signal(signal.SIGINT, on_sig)
fd = os.open("/dev/watchdog0", os.O_WRONLY)
note("OPEN pid=%d interval=%.1fs" % (os.getpid(), pet_s))
n = 0
try:
    while not stop:
        os.write(fd, b"\0")
        n += 1
        note("PET %d" % n)
        t_end = time.monotonic() + pet_s
        while not stop and time.monotonic() < t_end:
            time.sleep(0.2)
finally:
    os.write(fd, b"V")
    os.close(fd)
    note("CLOSE-WITH-MAGIC after %d pets" % n)
    log.close()
PY
    echo $! > "$PIDFILE"
    sleep 0.5
    if kill -0 "$(cat $PIDFILE)" 2>/dev/null; then
      echo "wdt_pet started pid=$(cat $PIDFILE) log=$LOGFILE"
      exit 0
    else
      echo "wdt_pet failed to start; see /tmp/lifetrac_p0c/wdt_pet.stderr"
      cat /tmp/lifetrac_p0c/wdt_pet.stderr 2>/dev/null
      exit 2
    fi
    ;;
  stop)
    if [ -e "$PIDFILE" ]; then
      PID=$(cat "$PIDFILE")
      if kill -TERM "$PID" 2>/dev/null; then
        for i in 1 2 3 4 5 6 7 8 9 10; do
          kill -0 "$PID" 2>/dev/null || break
          sleep 0.3
        done
        echo "wdt_pet stopped pid=$PID ($(tail -1 "$LOGFILE" 2>/dev/null))"
      else
        echo "wdt_pet pid=$PID was not running"
      fi
      rm -f "$PIDFILE"
    else
      echo "wdt_pet not running"
    fi
    ;;
  status)
    if [ -e "$PIDFILE" ] && kill -0 "$(cat $PIDFILE)" 2>/dev/null; then
      echo "wdt_pet running pid=$(cat $PIDFILE) last: $(tail -1 "$LOGFILE" 2>/dev/null)"
    else
      echo "wdt_pet NOT running"
    fi
    ;;
  *)
    echo "usage: $0 {start|stop|status}"
    exit 1
    ;;
esac
