#!/usr/bin/env bash
# leg2d_switch.sh -- the 2d operator-override switch on the base broker, per
# RS13_VECTOR_LEG.md step 2 (2d): one ack subscriber for the whole leg (-R skips
# the retained replay, %U stamps arrival on the base clock), then at T+60 s publish
# {"mode":"vector","quality":80} and at T+180 s {"mode":"mono_g4"}, each stamped
# on the base clock IN THE SAME board call as the publish (A13). T = the base
# capture's first received frame, so the switch lands 60 s into the flowing leg.
#
# Usage:   bash leg2d_switch.sh [<leg>]     (default leg tag 2d; start it right
#                                           after launching the 2d harness)
# Inputs:  BASE_SERIAL / BASE_TRANSPORT, BASE_BROKER_CONTAINER; rs13_cap_base running
#          (leg_prep.sh started it).
# Writes:  $EVIDENCE_DIR/leg<leg>_switch_times.txt, leg<leg>_acks.txt
# Boards:  base only: a mosquitto_sub inside the broker container (client id
#          lt2d_<leg>_<pid>; only that one is killed at the end), two retained
#          publishes to lifetrac/v25/control/encode_mode_override, and at the end a
#          zero-length retained publish that clears the override again.
# Radio:   THIS IS PART OF A RADIO LEG. rx_smoke relays each publish over the air
#          as a 0x63 command (the base transmits; the tractor acks). Run it only
#          inside a GO'd 2d leg. The final clear sends nothing over the air:
#          image_rx_daemon ignores an empty payload. An EXIT trap clears the
#          override and stops the subscriber on an abort too, so no harness run
#          started later (a direct run, the run_rs12_* wrappers) replays it.
# Origin:  bench-evidence/RS_13_vector_scene_2026-09-26/scripts/leg2d_switch.sh
#          (historical copy, unchanged).
set -u
. "$(dirname "${BASH_SOURCE[0]}")/../lib/bench_env.sh" || exit 1

LEG=${1:-2d}
case $LEG in -h|--help) bench_usage; exit 0 ;; esac
bench_check_name "leg tag" "$LEG"
E=$(bench_evidence_dir) || die "cannot create $EVIDENCE_DIR"
M="docker exec $BASE_BROKER_CONTAINER"
ACKS=/tmp/leg${LEG}_acks.txt                     # inside the broker container (its /tmp is not the host's)
OVR=lifetrac/v25/control/encode_mode_override
SUBID="lt2d_${LEG}_$$"                           # this run's subscriber, and only it, is killed

SUB_UP=0; OVR_SET=0
clear_override() {
  board_sh base "$SUDO $M mosquitto_pub -h 127.0.0.1 -t $OVR -r -n" > /dev/null 2>&1
}
stop_sub() {                                     # no `sh -c` around pkill: it would match itself
  board_sh base "$SUDO $M pkill -f $SUBID" > /dev/null 2>&1
}
leg2d_cleanup() {
  if [ $OVR_SET = 1 ]; then clear_override; OVR_SET=0; stamp "cleanup: retained $OVR cleared"; fi
  if [ $SUB_UP = 1 ]; then stop_sub; SUB_UP=0; stamp "cleanup: ack subscriber $SUBID stopped"; fi
}
trap leg2d_cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

SUB_UP=1
board_sh base "$SUDO $M rm -f $ACKS; $SUDO docker exec -d $BASE_BROKER_CONTAINER sh -c \"mosquitto_sub -i $SUBID -h 127.0.0.1 -R -F '%U %p' -t lifetrac/v25/status/encode_mode > $ACKS\"" > /dev/null 2>&1
stamp "ack subscriber $SUBID running inside $BASE_BROKER_CONTAINER"
t0=$(date +%s)
until [ "$(board_sh base "$SUDO docker logs rs13_cap_base 2>&1 | grep -c '^# *0 '" | tr -d ' ')" != "0" ]; do
  if [ $(( $(date +%s) - t0 )) -gt 240 ]; then stamp "ABORT: no frame reached the base capture within 240 s"; exit 3; fi
  sleep 2
done
stamp "T = first frame at the base capture"
sleep 60
# Stamp and publish in ONE board call: as two calls the return publish once started
# 16.6 s after its stamp (2026-10-03 leg 2d_yt), which read as a 17 s switch delay.
OVR_SET=1
{ echo -n "vector "; board_sh base "date +%s.%N; $SUDO $M mosquitto_pub -h 127.0.0.1 -t $OVR -r -m '{\"mode\":\"vector\",\"quality\":80}' >/dev/null"; } | tee -a "$E/leg${LEG}_switch_times.txt"
stamp "published vector override (T+60)"
sleep 120
{ echo -n "mono_g4 "; board_sh base "date +%s.%N; $SUDO $M mosquitto_pub -h 127.0.0.1 -t $OVR -r -m '{\"mode\":\"mono_g4\"}' >/dev/null"; } | tee -a "$E/leg${LEG}_switch_times.txt"
stamp "published mono_g4 override (T+180)"
sleep 130
board_sh base "$SUDO $M sh -c 'cat $ACKS'" > "$E/leg${LEG}_acks.txt"
stamp "acks collected: $(wc -l < "$E/leg${LEG}_acks.txt") lines -> $E/leg${LEG}_acks.txt"
clear_override; OVR_SET=0
stamp "retained $OVR cleared (empty retained publish; the rx daemon ignores it, nothing goes on the air)"
stop_sub; SUB_UP=0
