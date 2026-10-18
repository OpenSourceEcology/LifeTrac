#!/bin/sh
# inject_watch.sh -- runs ON THE BASE, as root (it calls docker). Starts a bench
# injector (kf_inject.py, dual_inject.py, ...) in lifetrac-v25:latest when its
# trigger fires, logs the trigger and the injector's output with marker lines,
# and stops both on request. run_live_radio_monitor.ps1 -InjectCmd pushes it to
# /tmp/lifetrac_strict and drives it; it is not meant to be run by hand.
#
# Why on the base: the 2026-10-18 leg W trigger read the harness console, which
# shows two lines of each daemon log every 5 s, and fired 2 min 41 s after the
# first frame. This reads rx_smoke's own log on the board (the W2 trigger,
# bench-evidence/RS_12_15_reverse_delivery_2026-10-18/scripts/inject_on_first_frame.sh)
# and needs no adb polling from the PC while the leg runs.
#
#   sh inject_watch.sh start first_frame <deadline_s> <script.py> [args...]
#        wait for the first "published frame_id" line in rx_smoke's log (polled
#        every 0.5 s), then run the injector. Detaches and prints
#        "INJECT_WATCH_SPAWNED pid=<pid>".
#   sh inject_watch.sh start now:<reason> <deadline_s> <script.py> [args...]
#        run the injector at once (the harness saw the tractor fault it waits
#        for; <reason> is recorded, e.g. fault:code=0x0D,sub=0x83).
#   sh inject_watch.sh reset <label>   stop any earlier watcher/injector and start
#        a fresh log holding one INJECT_ARMED line (fault triggers: nothing runs
#        on the base until the harness calls start now:...)
#   sh inject_watch.sh status          print the marker lines so far
#   sh inject_watch.sh stop            stop the watcher and the injector, print the log
#
# The injector runs as
#   docker run --rm --name bench_inject --network=host --entrypoint python3
#     -v /tmp/lifetrac_strict:/work -w /work -e PYTHONPATH=/work:/work/paho
#     lifetrac-v25:latest -u /work/<script.py> [args...]
# (the WORK_RUN line of lib/bench_env.sh), so <script.py> must be staged in
# /tmp/lifetrac_strict.
#
# Marker lines in the log (UTC from the base clock, then epoch seconds):
#   INJECT_ARMED       <utc> <epoch> <label>
#   INJECT_WATCH       <utc> <epoch> mode=<mode> deadline_s=<n> cmd=<script args>
#   INJECT_TRIGGER     <utc> <epoch> <what fired: the rx_smoke line, or the reason>
#   INJECT_START       <utc> <epoch> <script args>
#   INJECT_EXIT        <utc> <epoch> rc=<injector exit status; 137 = removed by stop>
#   INJECT_NOT_STARTED <utc> <epoch> <why>
#   INJECT_STOP        <utc> <epoch> by=harness injector_running=<0|1> watcher_running=<0|1>
# The injector's own output (KF_INJECT ..., DUAL t=...) sits between START and EXIT.
#
# Test overrides (environment): INJECT_DIR, INJECT_IMAGE, INJECT_NAME, INJECT_RX,
# INJECT_POLL_S, DOCKER.
set -u

DIR=${INJECT_DIR:-/tmp/lifetrac_strict}
LOG=$DIR/bench_inject.log
PIDF=$DIR/bench_inject.pid
IMAGE=${INJECT_IMAGE:-lifetrac-v25:latest}
NAME=${INJECT_NAME:-bench_inject}
RX=${INJECT_RX:-rx_smoke}
POLL=${INJECT_POLL_S:-0.5}
DOCKER=${DOCKER:-docker}

mark() {       # mark TAG TEXT -> stdout
  printf '%s %s %s %s\n' "$1" "$(date -u +%H:%M:%S)" "$(date +%s)" "$2"
}

nap() {        # fractional sleep where supported, else 1 s
  sleep "$1" 2>/dev/null || sleep 1
}

watcher_pid() {   # the pid in the pidfile, only while that process is alive
  _p=$(cat "$PIDF" 2>/dev/null)
  case $_p in ''|*[!0-9]*) return 1 ;; esac
  kill -0 "$_p" 2>/dev/null || return 1
  echo "$_p"
}

injector_running() {
  [ "$($DOCKER inspect -f '{{.State.Running}}' "$NAME" 2>/dev/null)" = true ]
}

own_pidf_rm() {   # the watcher removes the pidfile on exit, if it still names it
  [ "$(cat "$PIDF" 2>/dev/null)" = "$$" ] && rm -f "$PIDF"
  return 0
}

stop_all() {      # stop_all [record]: end any watcher and injector; record=1 logs INJECT_STOP
  wpid=$(watcher_pid) || wpid=""
  wr=0; [ -n "$wpid" ] && wr=1
  ir=0; injector_running && ir=1
  if [ "${1:-0}" = 1 ] && { [ $wr = 1 ] || [ $ir = 1 ]; }; then
    mark INJECT_STOP "by=harness injector_running=$ir watcher_running=$wr" >> "$LOG"
  fi
  # A watcher still waiting for its trigger is killed. One that has logged
  # INJECT_START is left to end by itself, so its INJECT_EXIT line is kept:
  # the container is removed (again on every pass, in case `docker run` was
  # only starting) until the watcher is gone, then it is killed as a last resort.
  if [ $wr = 1 ] && ! grep -q '^INJECT_START ' "$LOG" 2>/dev/null; then kill "$wpid" 2>/dev/null; fi
  $DOCKER rm -f "$NAME" >/dev/null 2>&1
  i=0
  while [ -n "$wpid" ] && kill -0 "$wpid" 2>/dev/null && [ $i -lt 20 ]; do
    nap 0.3; i=$((i + 1))
    $DOCKER rm -f "$NAME" >/dev/null 2>&1
  done
  if [ -n "$wpid" ] && kill -0 "$wpid" 2>/dev/null; then kill "$wpid" 2>/dev/null; fi
  rm -f "$PIDF"
}

cmd=${1:-}
[ $# -gt 0 ] && shift
case $cmd in
  start)
    [ $# -ge 3 ] || { echo "usage: sh $0 start <first_frame|now:REASON> <deadline_s> <script.py> [args...]" >&2; exit 2; }
    mkdir -p "$DIR" || exit 1
    stop_all 0                           # an earlier leg's watcher must not fire into this one
    : > "$LOG"
    nohup sh "$0" run "$@" >> "$LOG" 2>&1 < /dev/null &
    echo $! > "$PIDF"
    echo "INJECT_WATCH_SPAWNED pid=$!"
    ;;
  reset)
    mkdir -p "$DIR" || exit 1
    stop_all 0
    mark INJECT_ARMED "${1:-}" > "$LOG"
    echo "INJECT_RESET_DONE"
    ;;
  run)                                   # internal: the detached watcher started by "start"
    trap '' HUP
    trap own_pidf_rm EXIT
    mode=$1; deadline_s=$2; shift 2
    script=$1; shift
    case $deadline_s in ''|*[!0-9]*) deadline_s=600 ;; esac
    mark INJECT_WATCH "mode=$mode deadline_s=$deadline_s cmd=$script $*"
    end=$(( $(date +%s) + deadline_s ))
    case $mode in
      first_frame)
        # Pin THIS leg's container: if rx_smoke is replaced, the old id's log
        # cannot fire the trigger.
        cid=$($DOCKER inspect -f '{{.Id}}' "$RX" 2>/dev/null)
        if [ -z "$cid" ]; then mark INJECT_NOT_STARTED "no $RX container"; exit 0; fi
        while :; do
          line=$($DOCKER logs "$cid" 2>&1 | grep -a -m1 'published frame_id')
          [ -n "$line" ] && break
          if [ "$($DOCKER inspect -f '{{.State.Running}}' "$cid" 2>/dev/null)" != true ]; then
            mark INJECT_NOT_STARTED "$RX stopped before its first published frame_id"; exit 0
          fi
          if [ "$(date +%s)" -ge "$end" ]; then
            mark INJECT_NOT_STARTED "no published frame_id within $deadline_s s"; exit 0
          fi
          nap "$POLL"
        done
        mark INJECT_TRIGGER "first_frame: $line"
        ;;
      now:*)
        mark INJECT_TRIGGER "${mode#now:}"
        ;;
      *)
        mark INJECT_NOT_STARTED "unknown trigger mode '$mode'"; exit 2
        ;;
    esac
    if [ ! -s "$DIR/$script" ]; then mark INJECT_NOT_STARTED "$DIR/$script is missing"; exit 0; fi
    mark INJECT_START "$script $*"
    $DOCKER run --rm --name "$NAME" --network=host --entrypoint python3 \
      -v "$DIR:/work" -w /work -e PYTHONPATH=/work:/work/paho \
      "$IMAGE" -u "/work/$script" "$@"
    rc=$?
    mark INJECT_EXIT "rc=$rc"
    ;;
  status)
    grep -a '^INJECT_' "$LOG" 2>/dev/null
    echo "INJECT_STATUS_DONE"
    ;;
  stop)
    stop_all 1
    if [ -f "$LOG" ]; then cat "$LOG"; else echo "INJECT_LOG_MISSING"; fi
    echo "INJECT_STOP_DONE"
    ;;
  *)
    echo "usage: sh $0 start|reset|status|stop ... (see the header)" >&2
    exit 2
    ;;
esac
