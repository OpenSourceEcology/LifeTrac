#!/bin/bash
# leg2d_switch.sh -- the 2d operator-override switch on the base broker per
# RS13_VECTOR_LEG.md: one ack subscriber (-R skips the retained replay, %U stamps
# arrival on the base clock) for the whole leg, then at T+60 s publish
# {"mode":"vector","quality":80} and at T+180 s {"mode":"mono_g4"}, each stamped
# on the base clock. T = the base capture's first received frame (its first
# per-frame row), so the switch lands 60 s into the flowing leg.
set -u
export MSYS_NO_PATHCONV=1
DC="/c/Users/dorkm/Documents/GitHub/LifeTrac/LifeTrac-v25/DESIGN-CONTROLLER"
E="$DC/bench-evidence/RS_13_vector_scene_2026-09-26/legs"
S="echo fio | sudo -S -p ''"
B="adb -s 2D0A1209DABC240B shell"
M="docker exec design-controller-mosquitto-1"
stamp() { echo "$(date -u +%H:%M:%SZ) $*"; }
$B "$S rm -f /tmp/leg2d_acks.txt /tmp/leg2d_switch_times.txt; $S docker exec -d design-controller-mosquitto-1 sh -c \"mosquitto_sub -h 127.0.0.1 -R -F '%U %p' -t lifetrac/v25/status/encode_mode > /tmp/leg2d_acks.txt\"" >/dev/null 2>&1
# the container's /tmp is not the host's: read the acks back through docker exec at the end
stamp "ack subscriber running inside the mosquitto container"
t0=$(date +%s)
until [ "$($B "$S docker logs rs13_cap_base 2>&1 | grep -c '^# *0 '" | tr -d '\r ')" != "0" ]; do
  if [ $(( $(date +%s) - t0 )) -gt 240 ]; then stamp "ABORT: no frame reached the base capture within 240 s"; exit 3; fi
  sleep 2
done
stamp "T = first frame at the base capture"
sleep 60
{ echo -n "vector "; $B "date +%s.%N" | tr -d '\r'; } | tee -a "$E/leg2d_switch_times.txt"
$B "$S $M mosquitto_pub -h 127.0.0.1 -t lifetrac/v25/control/encode_mode_override -r -m '{\"mode\":\"vector\",\"quality\":80}'" | tr -d '\r'
stamp "published vector override (T+60)"
sleep 120
{ echo -n "mono_g4 "; $B "date +%s.%N" | tr -d '\r'; } | tee -a "$E/leg2d_switch_times.txt"
$B "$S $M mosquitto_pub -h 127.0.0.1 -t lifetrac/v25/control/encode_mode_override -r -m '{\"mode\":\"mono_g4\"}'" | tr -d '\r'
stamp "published mono_g4 override (T+180)"
sleep 130
$B "$S $M sh -c 'cat /tmp/leg2d_acks.txt'" | tr -d '\r' > "$E/leg2d_acks.txt"
stamp "acks collected: $(wc -l < "$E/leg2d_acks.txt") lines -> legs/leg2d_acks.txt"
$B "$S $M sh -c 'pkill mosquitto_sub' 2>/dev/null" >/dev/null 2>&1
