#!/usr/bin/env bash
# power_up_guard.sh -- keep the L072 radios quiet when the bench boards come up, and
# the end-of-session radio check. Waits for each board; as each one answers:
#   tractor: stop lifetrac-camera.service + the tractor-camera container (they map
#            /dev/ttymxc3, the radio UART, at every boot);
#   both:    report units, containers and /dev/ttymxc3 holders; stage only the
#            probe tools; read the radio with the read-only radio_state.py; if it is
#            not in SLEEP (0x80) -- the L072 boots into RXCONT, listening -- park it
#            with radio_park.py, re-read after 20 s, and retry (60 s apart, 3 tries).
#
# Usage:   bash power_up_guard.sh [options]
#   --timeout MIN        give up waiting after MIN minutes (default 115)
#   --only base|tractor  handle one board
#   --wait-power-cycle   arm BEFORE switching power on: a board already up for more
#                        than 30 min is ignored until it has rebooted
#   --check-only         report and run radio_state.py; stop nothing, park nothing
#                        (the end-of-session check, BENCH_RUNBOOK). It stages the
#                        probe files only if they are missing.
#   --no-park            stop the camera and read the state, but do not park
#   --stop-leg-daemons   also stop leftover tx_smoke / rx_smoke / synth_pub /
#                        camera_svc / rs13_cap* containers (a harness that was
#                        interrupted leaves them running; tx_smoke can transmit).
#                        Without it a board with one running is reported, not probed.
#   --capture            after a board is handled, run the read-only
#                        pull_board_state.sh <role> (adb only; BENCH_BOARDS.md)
# Inputs:  lib/bench_env.sh settings: serials, BASE_TRANSPORT (adb, or ssh when the
#          base does not enumerate on USB), the probe images.
# Writes:  board: /tmp/lifetrac_strict/{probes,radio_park.py,radio_state.py,paho};
#          PC: log $BENCH_SCRATCH/power_up_guard_<UTC date>.log
#          (+ the archive of pull_board_state.sh with --capture, in $ARCHIVE_DIR).
# Boards:  systemctl stop lifetrac-camera / docker stop tractor-camera (tractor, not
#          with --check-only); --rm probe containers with /dev/ttymxc3 mapped.
# Radio:   NEVER transmits. radio_state.py only reads RegOpMode; radio_park.py writes
#          STANDBY -> SLEEP. A probe's HostLink connect can wake the receiver
#          (RXCONT, receive-only). The park readback is not a persistent-off
#          guarantee (BENCH_RUNBOOK): it holds once the firmware's scan state machine
#          has failed out, up to ~2 min after the daemons stop.
# Origin:  bench-evidence/RS_13_vector_scene_2026-09-26/scripts/guard_both.sh,
#          power_up_guard.sh and tractor_guard_capture.sh (historical copies,
#          unchanged), generalised.
set -u
. "$(dirname "${BASH_SOURCE[0]}")/../lib/bench_env.sh" || exit 1

TIMEOUT_MIN=115; ONLY=""; WAIT_PC=0; CHECK=0; PARK=1; STOPD=0; CAPTURE=0
while [ $# -gt 0 ]; do
  case $1 in
    --timeout) TIMEOUT_MIN=${2:?--timeout MIN}; shift ;;
    --only) ONLY=${2:?--only base|tractor}; shift ;;
    --wait-power-cycle) WAIT_PC=1 ;;
    --check-only) CHECK=1; PARK=0 ;;
    --no-park) PARK=0 ;;
    --stop-leg-daemons) STOPD=1 ;;
    --capture) CAPTURE=1 ;;
    -h|--help) bench_usage; exit 0 ;;
    *) die "unknown argument '$1'" ;;
  esac
  shift
done
case $ONLY in ''|base|tractor) ;; *) die "--only base|tractor" ;; esac
case $TIMEOUT_MIN in ''|*[!0-9]*) die "--timeout takes whole minutes" ;; esac
ROLES="tractor base"; [ -n "$ONLY" ] && ROLES=$ONLY
STAMP_LOG="$BENCH_SCRATCH/power_up_guard_$(date -u +%Y-%m-%d).log"
LEG_DAEMONS='^(tx_smoke|rx_smoke|synth_pub|camera_svc|rs13_cap|rs13_cap_base)$'

radio_read() {   # $1 who, $2 image -> the RADIO_STATE / STATE_FAIL line
  board_sh "$1" "$SUDO $PROBE_RUN $2 -u /work/radio_state.py 2>&1 | tail -1"
}
is_sleep() { printf '%s\n' "$1" | grep -q '"reg_op_mode": "0x80"'; }

handle() {   # $1 role; returns 0 when the radio reads SLEEP (or was left as asked)
  local who=$1 img st r attempt holders names daemons
  stamp "$who $(board_serial "$who") is up via $(board_via "$who"): $(board_sh "$who" uptime)"
  if [ "$who" = tractor ]; then
    if [ $CHECK = 0 ]; then
      board_sh "$who" "$SUDO systemctl stop lifetrac-camera.service; $SUDO docker stop tractor-camera" 2>&1 | sed 's/^/   /' | tee -a "$STAMP_LOG"
    fi
    stamp "tractor: lifetrac-camera.service -> $(board_sh "$who" 'systemctl is-active lifetrac-camera.service')"
  fi
  names=$(board_containers "$who")
  stamp "$who containers: [$names]"
  daemons=$(printf '%s\n' $names | grep -E "$LEG_DAEMONS" | tr '\n' ' ' | sed 's/ $//')
  if [ -n "$daemons" ]; then
    if [ $STOPD = 1 ] && [ $CHECK = 0 ]; then
      stamp "$who: stopping leftover leg containers: $daemons"
      board_sh "$who" "$SUDO docker stop -t 3 $daemons" 2>&1 | sed 's/^/   /' | tee -a "$STAMP_LOG"
    else
      stamp "$who: LEG CONTAINER(S) RUNNING: $daemons -- the radio may be in use; not probing (re-run with --stop-leg-daemons)"
      return 1
    fi
  fi
  holders=$(board_uart_holders "$who")
  stamp "$who /dev/ttymxc3 holders (pids): [$holders]"
  case $holders in
    unknown*) stamp "$who: cannot tell whether anything holds the radio UART ($holders) -- not probing"; return 1 ;;
  esac
  if [ -n "$holders" ]; then
    stamp "$who: radio UART busy -- not probing. Identify the holder (ps -p $holders) and stop it."
    return 1
  fi

  # stage only the probe tools (no code tree); --check-only keeps an existing staging
  if [ $CHECK = 0 ] || ! board_sh "$who" "test -f $BOARD_STAGE/radio_state.py && test -f $BOARD_STAGE/method_h_stage2_tx_probe_v2.py && test -d $BOARD_STAGE/paho" > /dev/null; then
    board_sh "$who" "$SUDO mkdir -p $BOARD_STAGE/legs; $SUDO chmod 0777 $BOARD_STAGE $BOARD_STAGE/legs" > /dev/null
    for f in "$HELPER_DIR/method_g_stage1_probe.py" "$HELPER_DIR/method_h_stage2_tx_probe_v2.py" "$HELPER_DIR/rs115_stats_probe.py" \
             "$HELPER_DIR/rs116_health_probe.py" "$BT_DIR/radio_park.py" "$BT_DIR/radio_state.py"; do
      board_push "$who" "$f" "$BOARD_STAGE/" || stamp "PUSH FAILED $f"
    done
    board_sh "$who" "$SUDO rm -rf $BOARD_STAGE/paho" > /dev/null
    board_push "$who" "$REPO_ROOT/_paho_pull/paho" "$BOARD_STAGE/" || stamp "PUSH FAILED paho"
  fi

  img=$(board_image "$who")
  if ! board_has_image "$who" "$img"; then
    if board_has_image "$who" "$TRACTOR_PROBE_IMAGE"; then img=$TRACTOR_PROBE_IMAGE
    elif board_has_image "$who" "$BASE_IMAGE"; then img=$BASE_IMAGE
    else stamp "$who: no probe image ($BASE_IMAGE / $TRACTOR_PROBE_IMAGE) -- radio state unknown (the L072 boots into RXCONT, receive-only)"; return 1
    fi
  fi
  st=$(radio_read "$who" "$img")
  stamp "$who radio: $st"
  is_sleep "$st" && return 0
  if [ $PARK = 0 ]; then stamp "$who: not parked ($([ $CHECK = 1 ] && echo check-only || echo --no-park))"; return 1; fi
  for attempt in 1 2 3; do
    r=$(board_sh "$who" "$SUDO $PROBE_RUN $img -u /work/radio_park.py 2>&1 | tail -1")
    stamp "$who park attempt $attempt: $r"
    sleep 20
    st=$(radio_read "$who" "$img")
    stamp "$who radio after park: $st"
    is_sleep "$st" && return 0
    [ $attempt -lt 3 ] && { stamp "$who: still not 0x80 -- the scan SM may be walking; waiting 60 s"; sleep 60; }
  done
  stamp "$who: PARK NOT CONFIRMED after 3 attempts"
  return 1
}

declare -A done_=() seen_old=()
end=$(( $(date +%s) + TIMEOUT_MIN * 60 ))
rc=0
stamp "guard armed for [$ROLES] ($([ $CHECK = 1 ] && echo check-only || echo park-if-listening)); waiting up to $TIMEOUT_MIN min; log $STAMP_LOG"
while [ "$(date +%s)" -lt $end ]; do
  for who in $ROLES; do
    [ -n "${done_[$who]:-}" ] && continue
    board_present "$who" || continue
    if [ $WAIT_PC = 1 ]; then
      up=$(board_sh "$who" "cut -d. -f1 /proc/uptime" | tr -d ' ')
      if [ "${up:-0}" -gt 1800 ] 2>/dev/null; then
        [ -z "${seen_old[$who]:-}" ] && { stamp "$who visible but up ${up}s (not freshly booted); waiting for the power cycle"; seen_old[$who]=1; }
        continue
      fi
    fi
    handle "$who" || rc=1
    done_[$who]=1
    if [ $CAPTURE = 1 ]; then
      if [ "$(board_via "$who")" = adb ]; then
        stamp "$who capture start (pull_board_state.sh, read-only)"
        ARCHIVE="$ARCHIVE_DIR" bash "$BT_DIR/pull_board_state.sh" "$who" 2>&1 | tee -a "$STAMP_LOG" | tail -8
        stamp "$who capture done"
      else
        stamp "$who: --capture needs adb (pull_board_state.sh is adb-only); skipped"
      fi
    fi
  done
  all=1; for who in $ROLES; do [ -n "${done_[$who]:-}" ] || all=0; done
  if [ $all = 1 ]; then stamp "guard finished (rc $rc)"; exit $rc; fi
  sleep 3
done
for who in $ROLES; do [ -n "${done_[$who]:-}" ] || stamp "TIMEOUT: $who did not appear$([ $WAIT_PC = 1 ] && echo ' (or was not power-cycled)') within $TIMEOUT_MIN min"; done
exit 2
