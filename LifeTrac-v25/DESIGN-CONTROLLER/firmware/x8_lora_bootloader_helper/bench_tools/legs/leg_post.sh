#!/usr/bin/env bash
# leg_post.sh -- after a leg's "[EVIDENCE] archived to <dir>" line: wait for the base
# capture, collect its files, post-brackets on both boards, the leg reports
# (rs12_leg_report.py WITH the base capture -- its "capture seq gaps" line is the
# loss of record, RS-13.1 A14 -- and frag_gap_report.py), the P2/P4 greps, the
# tractor-log, then PARK both radios last (PARK_TRANSIENT -> wait 60 s, park again).
#
# Usage:   bash leg_post.sh <leg> [<harness archive dir>] [--no-park]
#   <leg>           the tag given to leg_prep.sh
#   archive dir     bench-evidence/radio_monitor_<stamp>_<sha> (Windows or MSYS form).
#                   Default: $BENCH_SCRATCH/leg<leg>_archive.txt (written by the
#                   harness wrapper leg_prep.sh generates), else the "[EVIDENCE]
#                   archived to" line of $BENCH_SCRATCH/leg<leg>_harness.txt.
#   --no-park       skip the park (another leg follows at once; park after the last)
# Inputs:  lib/bench_env.sh settings; the harness archive; the running rs13_cap_base.
# Writes:  $EVIDENCE_DIR/leg<leg>_{base.txt,base.jsonl,base.json,post_base,post_tractor,
#          report,gaps,p2p4,tractor_log,params,harness,park,post_transcript}.txt
# Boards:  waits for and removes rs13_cap_base (a re-run finds it gone and keeps the
#          files the first run wrote); then refuses the brackets and the park while a
#          leg daemon (tx_smoke / rx_smoke / synth_pub / camera_svc) still runs or
#          anything holds /dev/ttymxc3 -- an interrupted harness leaves them, and a
#          park would race a live transmitter; rs115 probes and radio_park.py in --rm
#          containers with /dev/ttymxc3 mapped, named lt_post_<board>. If this script
#          is interrupted, its EXIT trap removes those lt_post_* containers (and only
#          those). rs13_cap_base is left to finish on its own (receive-only, no device),
#          so a re-run can still collect it.
# Radio:   never transmits. The brackets' HostLink connect keeps the receiver up
#          (RXCONT); radio_park.py writes SLEEP (0x80) and reads it back. The
#          readback is not a persistent-off guarantee (BENCH_RUNBOOK): re-check
#          later with power_up_guard.sh --check-only.
# Origin:  bench-evidence/RS_13_vector_scene_2026-09-26/scripts/leg_post.sh
#          (historical copy, unchanged). New here: --capture on the leg report,
#          the archive read from the transcript, the DTS-carrier check on
#          params.txt, the park retry.
set -u
. "$(dirname "${BASH_SOURCE[0]}")/../lib/bench_env.sh" || exit 1

LEG=""; ARCH=""; PARK=1
for a in "$@"; do
  case $a in
    --no-park) PARK=0 ;;
    -h|--help) bench_usage; exit 0 ;;
    -*) die "unknown option '$a'" ;;
    *) if [ -z "$LEG" ]; then LEG=$a; elif [ -z "$ARCH" ]; then ARCH=$a; else die "extra argument '$a'"; fi ;;
  esac
done
[ -n "$LEG" ] || { bench_usage; exit 2; }
bench_check_name "leg tag" "$LEG"
HT="$BENCH_SCRATCH/leg${LEG}_harness.txt"
AP="$BENCH_SCRATCH/leg${LEG}_archive.txt"
if [ -z "$ARCH" ] && [ -f "$AP" ]; then ARCH=$(tr -d '\r\n' < "$AP"); fi
if [ -z "$ARCH" ] && [ -f "$HT" ]; then
  # PowerShell wraps the Write-Host line at the console width, so the path may
  # continue on the next line(s): rejoin until the folder holds params.txt.
  mapfile -t _l < <(tr -d '\000' < "$HT" | tr -d '\r' | awk '/\[EVIDENCE\] archived to/ { n = NR; a = $0; b = ""; c = ""; next }
    n && NR == n + 1 { b = $0 } n && NR == n + 2 { c = $0 } END { print a; print b; print c }')
  _p=${_l[0]#*archived to }
  for _i in 1 2 3; do
    if [ -n "$_p" ] && [ -f "$(win_path "$_p")/params.txt" ]; then ARCH=$_p; break; fi
    [ $_i -lt 3 ] && _p="$_p${_l[$_i]:-}"
  done
  unset _l _p _i
fi
[ -n "$ARCH" ] || die "no archive dir given, and none found in $AP or $HT"
ARCH=$(win_path "$ARCH")
[ -f "$ARCH/params.txt" ] || die "not a harness archive (no params.txt): $ARCH"

E=$(bench_evidence_dir) || die "cannot create $EVIDENCE_DIR"
exec > >(tee -a "$E/leg${LEG}_post_transcript.txt") 2>&1
hdr() { echo "# leg $LEG $1, $(date -u +%Y-%m-%dT%H:%M:%SZ) PC clock"; }
param() { tr -d '\r' < "$ARCH/params.txt" | sed 's/^\xEF\xBB\xBF//' | sed -n "s/^$1=//p" | head -1; }

# UART probes run in containers named lt_post_<board>; on an abort (error, Ctrl+C)
# the EXIT trap removes the ones this run started, so none is left holding the UART.
POST_STARTED=""
post_cleanup() {
  local w
  for w in $POST_STARTED; do
    board_sh "$w" "$SUDO docker rm -f lt_post_$w >/dev/null 2>&1" > /dev/null 2>&1
  done
  [ -n "$POST_STARTED" ] && stamp "cleanup: removed the probe container(s) this run started on:$POST_STARTED"
  POST_STARTED=""
}
trap post_cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM
probe_on() {   # probe_on WHO IMAGE SCRIPT: one UART probe in a container named lt_post_<who>
  local who=$1
  case " $POST_STARTED " in *" $who "*) ;; *) POST_STARTED="$POST_STARTED $who" ;; esac
  board_sh "$who" "$SUDO ${PROBE_RUN/docker run /docker run --name lt_post_$who } $2 -u /work/$3 2>&1"
}

stamp "== leg $LEG post (archive $ARCH)"
REGP=$(param reg_profile); FRF=$(param force_frf_hz)
[ "$REGP" = 1 ] && PROF=image_bw250 || PROF=image_bw500

stamp "-- waiting for the base capture (rs13_cap_base) to finish its 480 s"
# Only when the container still exists: a re-run must not overwrite the first run's
# capture log with "No such container".
board_sh base "if $SUDO docker inspect rs13_cap_base >/dev/null 2>&1; then $SUDO docker wait rs13_cap_base >/dev/null 2>&1; $SUDO docker logs rs13_cap_base > $BOARD_STAGE/legs/leg${LEG}_base.txt 2>&1; $SUDO docker rm -f rs13_cap_base >/dev/null 2>&1; else echo '(rs13_cap_base is gone: using the capture files already on the board)'; fi; sed -n '/== vector dry run summary ==/,\$p' $BOARD_STAGE/legs/leg${LEG}_base.txt"
for f in leg${LEG}_base.txt leg${LEG}_base.jsonl leg${LEG}_base.json; do
  board_pull base "$BOARD_STAGE/legs/$f" "$E/$f" || echo "PULL FAILED $f"
done

stamp "-- radio quiet check before the brackets and the park"
busy=""
for who in base tractor; do
  board_sh "$who" "$SUDO docker rm -f lt_post_$who >/dev/null 2>&1" > /dev/null 2>&1   # a leftover of an earlier run
  d=$(printf '%s\n' $(board_containers "$who") | grep -E '^(tx_smoke|rx_smoke|synth_pub|camera_svc)$' | tr '\n' ' ' | sed 's/ $//')
  h=$(board_uart_holders "$who")
  echo "$who: leg daemons [$d] ttymxc3 holders [$h]"
  { [ -n "$d" ] || [ -n "$h" ]; } && busy="$busy $who"
done
if [ -n "$busy" ]; then
  stamp "ABORT before the brackets and the park: not quiet on:$busy (an interrupted harness leaves its daemons running,"
  stamp "       and a park would race a live transmitter). Stop them: bash \"$(win_path "$LEGS_DIR/power_up_guard.sh")\" --stop-leg-daemons"
  stamp "       (it parks too), then re-run: bash \"$(win_path "$LEGS_DIR/leg_post.sh")\" $LEG \"$ARCH\""
  exit 3
fi

stamp "-- post-brackets (rs115) both boards"
{ hdr "post bracket base"; probe_on base "$BASE_IMAGE" rs115_stats_probe.py; } > "$E/leg${LEG}_post_base.txt"
{ hdr "post bracket tractor"; probe_on tractor "$TRACTOR_PROBE_IMAGE" rs115_stats_probe.py; } > "$E/leg${LEG}_post_tractor.txt"
grep -h -E "^radio_(rx_ok|tx_ok|crc_err)=" "$E/leg${LEG}_post_base.txt" | tr '\n' ' '; echo " (base)"
grep -h -E "^radio_(rx_ok|tx_ok|crc_err)=" "$E/leg${LEG}_post_tractor.txt" | tr '\n' ' '; echo " (tractor)"

stamp "-- rs12_leg_report (--capture) + frag_gap_report"
CAP=(); [ -f "$E/leg${LEG}_base.jsonl" ] && CAP=(--capture "$E/leg${LEG}_base.jsonl")
PYTHONIOENCODING=utf-8 bench_py "$DC/tools/rs12_leg_report.py" "$ARCH" --pre "$E/leg${LEG}_pre_base.txt" \
  --post "$E/leg${LEG}_post_base.txt" ${CAP[@]+"${CAP[@]}"} 2>&1 | tee "$E/leg${LEG}_report.txt" | head -8
PYTHONIOENCODING=utf-8 bench_py "$BT_DIR/frag_gap_report.py" "$ARCH" 2>&1 | tee "$E/leg${LEG}_gaps.txt" | head -8

stamp "-- P2 / P4 from the archive"
{ echo "# leg $LEG P2/P4 greps on $ARCH"
  echo "harness: reg_profile=$REGP force_frf_hz=$FRF tx_batch=$(param tx_batch) synth_fps=$(param synth_fps) cam_extra_env=$(param cam_extra_env)"
  if [ "$REGP" = 2 ] && { [ "${FRF:-0}" = 0 ] || [ "$FRF" = 915000000 ]; }; then
    echo "WARNING: this DTS leg flew on 915.000 MHz, the RS-11.6 emitter channel (RS-13.1 A19) -- say so in RESULTS"
  fi
  echo -n "published frame_id lines (rx_daemon.log): "; grep -c "published frame_id" "$ARCH/rx_daemon.log"
  echo -n "tx done K=1 lines: "; grep -cE "done( \(pipelined\))?: 1 fragments ok" "$ARCH/tx_daemon.log"
  echo -n "tx done K>=2 lines: "; grep -cE "done( \(pipelined\))?: ([2-9]|[1-9][0-9]+) fragments ok" "$ARCH/tx_daemon.log"
  echo -n "tx ABORTED lines: "; grep -cE "ABORTED( \(pipelined\))?" "$ARCH/tx_daemon.log"
  echo "max K: $(grep -oE 'done( \(pipelined\))?: [0-9]+ fragments ok' "$ARCH/tx_daemon.log" | grep -oE '[0-9]+ fragments' | sort -n | tail -1)"
  echo -n "camera_service vector_stats lines: "
  if [ -f "$ARCH/camera_service.log" ]; then grep -c vector_stats "$ARCH/camera_service.log"; else echo "no camera_service.log"; fi
  echo "tx drops before air (last tx stats line): $(grep -oE 'drop_(full|stale)=[0-9]+' "$ARCH/tx_daemon.log" | tail -2 | tr '\n' ' ')"
  echo "tractor FHSS authority (post bracket): $(grep -hoE '(tx_stream_streak_max|tx_first_anchor)=[0-9]+' "$E/leg${LEG}_post_tractor.txt" | tr '\n' ' ')"
  echo -n "seq-gap loss (per codec run): "
  if [ -f "$E/leg${LEG}_base.jsonl" ]; then
    PYTHONIOENCODING=utf-8 bench_py "$LEGS_DIR/leg_replay.py" "$LEG" "$PROF" --evidence-dir "$E" 2>&1 | head -1
  else
    echo "no base capture"
  fi
} | tee "$E/leg${LEG}_p2p4.txt"
if [ -f "$ARCH/camera_service.log" ]; then
  PYTHONIOENCODING=utf-8 bench_py "$DC/tools/vector_dry_run.py" tractor-log "$ARCH/camera_service.log" 2>&1 | tee "$E/leg${LEG}_tractor_log.txt"
fi
cp "$ARCH/params.txt" "$E/leg${LEG}_params.txt" 2>/dev/null
if [ -f "$HT" ]; then
  if head -c 2 "$HT" | od -An -tx1 | grep -q 'ff fe'; then
    iconv -f UTF-16 -t UTF-8 "$HT" | tr -d '\r' > "$E/leg${LEG}_harness.txt"
  else
    tr -d '\r' < "$HT" > "$E/leg${LEG}_harness.txt"
  fi
fi

if [ $PARK = 1 ]; then
  stamp "-- park both radios (last)"
  hdr "park" >> "$E/leg${LEG}_park.txt"
  todo="base tractor"
  for attempt in 1 2 3; do
    left=""
    for who in $todo; do
      r=$(probe_on "$who" "$(board_image "$who")" radio_park.py | tail -1)
      printf '%-8s attempt %d: %s\n' "$who:" "$attempt" "$r" | tee -a "$E/leg${LEG}_park.txt"
      case $r in PARK_OK*) ;; *) left="$left $who" ;; esac
    done
    todo=$left
    [ -z "$todo" ] && break
    [ $attempt -lt 3 ] && { stamp "not parked:$todo -- the scan SM may still be walking; waiting 60 s"; sleep 60; }
  done
  [ -n "$todo" ] && stamp "PARK NOT CONFIRMED on:$todo -- check with power_up_guard.sh --check-only before leaving the bench"
fi
POST_STARTED=""                          # finished normally: every probe container ended (--rm)
stamp "== leg $LEG post done (evidence: $E)"
