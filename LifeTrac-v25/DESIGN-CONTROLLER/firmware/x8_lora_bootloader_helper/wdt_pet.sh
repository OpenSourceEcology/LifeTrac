#!/bin/bash
# wdt_pet.sh — keep /dev/watchdog0 alive during long-running operations
# (e.g. an L072 flash which takes ~62 s, exceeding the 60 s imx2+ WDT timeout).
#
# Usage:
#   bash wdt_pet.sh start        # spawns a background petter, writes PID to $WDT_PET_DIR/wdt_pet.pid
#   bash wdt_pet.sh stop         # asks the petter to hand the watchdog back and exit
#   bash wdt_pet.sh status       # prints whether petter is running
#
# Environment (defaults are the board's; the SIL test
# base_station/tests/test_x8_watchdog_ownership_sil.py overrides them):
#   WDT_DEV               watchdog device                (/dev/watchdog0)
#   WDT_PET_DIR           pidfile and petter stderr      (/tmp/lifetrac_p0c)
#   WDT_PET_LOG           fsync'd pet log, see below     (/home/fio/wdt_pet.log)
#   WDT_PET_INTERVAL_S    seconds between pets           (10)
#   WDT_PET_START_POLLS   0.2 s polls for OPEN/BUSY      (15, i.e. ~3 s; not a positive integer -> 15)
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
# 2026-10-17 SYSTEMD MAY OWN THE DOG (L2 of
# X8_HEALTH_AND_RECOVERY/REBOOT_AND_POWER_CYCLE.md, TODO.md PWR-2):
#   With firmware/x8_recovery's 90-lifetrac-watchdog.conf drop-in installed
#   (on no board yet), PID 1 opens /dev/watchdog0 from the next daemon-reload
#   or boot with RuntimeWatchdogSec=30s: the 60 s above becomes 30 s at
#   runtime, systemd pets it about every 15 s, and the device is single-open,
#   so our open() gets EBUSY. Without this change that EBUSY would kill the
#   petter and `start` would print a misleading "failed to start" (inferred
#   from the code, never seen: L2 has not run on a board).
#   full_flash_pipeline.sh ignores start's exit code, so a flash would still
#   be covered by systemd's petting. Now `start`:
#     1. asks systemd first (`systemctl show -p RuntimeWatchdogUSec --value`).
#        Anything but empty or "0" means PID 1 owns the dog: "30s", "2min",
#        and "infinity" too (RuntimeWatchdogSec=default: armed at the
#        hardware's own timeout). It says so, shows who holds the device
#        (fuser; never opens it), writes no pidfile, appends one fsync'd
#        "RELY systemd RuntimeWatchdogUSec=<v>" line to $WDT_PET_LOG (so the
#        log says why there are no pets) and exits 0 without forking a petter;
#     2. otherwise forks the petter as before. If its open() still gets EBUSY
#        (an owner systemd does not report), the petter logs "BUSY: <dev> held
#        by another owner" and exits 0, and `start` reports "owned elsewhere,
#        relying on it" (exit 0, pidfile removed);
#     3. waits up to ~3 s for this petter's OPEN or BUSY line in the log
#        instead of the old fixed 0.5 s liveness check, so a petter that dies
#        late is not reported as started. A petter still silent at the end is
#        sent SIGTERM (safe before or after its open: the handler is installed
#        first and the close path always writes 'V'); never SIGKILL, which
#        would close without 'V'.
#   `stop` and `status` are unchanged; with systemd as owner there is no
#   pidfile, so `stop` prints "wdt_pet not running".
#
# Every pet is logged with a monotonic timestamp and fsync'd to
# $WDT_PET_LOG (default /home/fio/wdt_pet.log — persistent, so the log
# survives the reboot it is meant to diagnose). /tmp is tmpfs.

set -u
WDT_DEV=${WDT_DEV:-/dev/watchdog0}
WDT_PET_DIR=${WDT_PET_DIR:-/tmp/lifetrac_p0c}
PIDFILE=$WDT_PET_DIR/wdt_pet.pid
ERRFILE=$WDT_PET_DIR/wdt_pet.stderr
LOGFILE=${WDT_PET_LOG:-/home/fio/wdt_pet.log}
PET_S=${WDT_PET_INTERVAL_S:-10}
START_POLLS=${WDT_PET_START_POLLS:-15}
# a non-number would make the poll-count test fail every pass and the wait never
# end; 0 would give up on a healthy petter at the first poll
case $START_POLLS in ''|*[!0-9]*|0) START_POLLS=15 ;; esac

# systemd's RuntimeWatchdogUSec, or nothing (no systemctl, not PID 1, or PID 1
# too wedged to answer within 5 s)
systemd_runtime_wdt() {
  command -v systemctl >/dev/null 2>&1 || return 0
  if command -v timeout >/dev/null 2>&1; then
    timeout 5 systemctl show -p RuntimeWatchdogUSec --value 2>/dev/null | head -n 1
  else
    systemctl show -p RuntimeWatchdogUSec --value 2>/dev/null | head -n 1
  fi
}

# who holds $WDT_DEV, by pid and command name. fuser stats the path and reads
# /proc/*/fd; it never opens the device.
show_holder() {
  command -v fuser >/dev/null 2>&1 || return 0
  local who="" p
  for p in $(fuser "$WDT_DEV" 2>/dev/null); do   # pids on stdout, "dev:" label on stderr
    p=${p%%[!0-9]*}
    [ -n "$p" ] && who="$who $p($(cat "/proc/$p/comm" 2>/dev/null))"
  done
  echo "wdt_pet: fuser $WDT_DEV:${who:- no holder visible}"
}

case "${1:-}" in
  start)
    if [ -e "$PIDFILE" ] && kill -0 "$(cat "$PIDFILE")" 2>/dev/null; then
      echo "wdt_pet already running pid=$(cat "$PIDFILE")"
      exit 0
    fi
    RWD=$(systemd_runtime_wdt)
    case "$RWD" in
      ""|0|0s|off) ;;
      *)
        echo "wdt_pet: systemd owns $WDT_DEV (RuntimeWatchdogUSec=$RWD); relying on it, no petter started"
        show_holder
        rm -f "$PIDFILE"   # a stale one from before the drop-in went live
        # One fsync'd line in the persistent log, so it says why this run has no
        # pets, and run_flash_bench.sh (which deletes the log before the pipeline
        # and tails it after) finds a file. Time: /proc/uptime, the petter's
        # monotonic clock on a board that never suspends.
        if { echo "$(cut -d' ' -f1 /proc/uptime 2>/dev/null) RELY systemd RuntimeWatchdogUSec=$RWD, no petter started" >> "$LOGFILE"; } 2>/dev/null; then
          { sync "$LOGFILE" || sync; } 2>/dev/null   # sync FILE: coreutils >= 8.24
        fi
        exit 0
        ;;
    esac
    mkdir -p "$WDT_PET_DIR"
    # Byte offset of the persistent log before this start: only lines after it
    # count, so an old OPEN or BUSY line cannot answer for this petter.
    LOG0=0
    [ -f "$LOGFILE" ] && LOG0=$(wc -c < "$LOGFILE")
    nohup python3 -u - "$LOGFILE" "$PET_S" "$WDT_DEV" > "$ERRFILE" 2>&1 <<'PY' &
import os, sys, time, signal
logpath, pet_s, dev = sys.argv[1], float(sys.argv[2]), sys.argv[3]
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
try:
    fd = os.open(dev, os.O_WRONLY)
except OSError as e:
    # 16 is EBUSY: the device is single-open and someone else (systemd with
    # RuntimeWatchdogSec=) holds it, so they are petting it. A literal, not
    # errno.EBUSY: import nothing new, the LmP host python lacks modules.
    if e.errno == 16:
        note("BUSY: %s held by another owner (petter pid=%d)" % (dev, os.getpid()))
        log.close()
        sys.exit(0)
    note("OPEN-FAILED %s errno=%s (petter pid=%d)" % (dev, e.errno, os.getpid()))
    raise
n = 0
try:
    # from here on every exit path writes 'V' first (2026-09-12 above)
    note("OPEN pid=%d interval=%.1fs" % (os.getpid(), pet_s))
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
    PID=$!
    echo "$PID" > "$PIDFILE"
    # Wait for this petter's verdict (bounded): OPEN = started, BUSY = someone
    # else owns the dog, dead or silent = failed.
    STATE=""
    i=0
    while :; do
      NEW=$(tail -c +$((LOG0 + 1)) "$LOGFILE" 2>/dev/null)
      case "$NEW" in
        *"OPEN pid=$PID "*) STATE=open ;;
        *"BUSY: "*"(petter pid=$PID)"*) STATE=busy ;;
      esac
      [ -n "$STATE" ] && break
      if ! kill -0 "$PID" 2>/dev/null; then
        # it may have logged BUSY and exited between the read above and kill -0
        NEW=$(tail -c +$((LOG0 + 1)) "$LOGFILE" 2>/dev/null)
        case "$NEW" in
          *"BUSY: "*"(petter pid=$PID)"*) STATE=busy ;;
          *) STATE=dead ;;
        esac
        break
      fi
      i=$((i + 1))
      if [ "$i" -ge "$START_POLLS" ]; then
        STATE=silent
        break
      fi
      sleep 0.2
    done
    case "$STATE" in
      open)
        if kill -0 "$PID" 2>/dev/null; then
          echo "wdt_pet started pid=$PID log=$LOGFILE"
          exit 0
        fi
        STATE="died after OPEN"
        ;;
      busy)
        rm -f "$PIDFILE"
        echo "wdt_pet: $WDT_DEV owned elsewhere (open got EBUSY); relying on it, no petter started"
        show_holder
        exit 0
        ;;
      silent)
        # alive but no OPEN line: ask it to stop (its handler is installed by
        # now, and a late open still closes with 'V'); never SIGKILL it
        kill -TERM "$PID" 2>/dev/null
        for _ in 1 2 3 4 5 6 7 8 9 10; do
          kill -0 "$PID" 2>/dev/null || break
          sleep 0.3
        done
        ;;
    esac
    if kill -0 "$PID" 2>/dev/null; then
      echo "wdt_pet failed to start: pid=$PID never logged OPEN and is still alive; run 'wdt_pet.sh stop' (pidfile kept)"
    else
      rm -f "$PIDFILE"
      echo "wdt_pet failed to start ($STATE); see $ERRFILE"
    fi
    cat "$ERRFILE" 2>/dev/null
    exit 2
    ;;
  stop)
    if [ -e "$PIDFILE" ]; then
      PID=$(cat "$PIDFILE")
      if kill -TERM "$PID" 2>/dev/null; then
        for _ in 1 2 3 4 5 6 7 8 9 10; do
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
    if [ -e "$PIDFILE" ] && kill -0 "$(cat "$PIDFILE")" 2>/dev/null; then
      echo "wdt_pet running pid=$(cat "$PIDFILE") last: $(tail -1 "$LOGFILE" 2>/dev/null)"
    else
      echo "wdt_pet NOT running"
    fi
    ;;
  *)
    echo "usage: $0 {start|stop|status}"
    exit 1
    ;;
esac
