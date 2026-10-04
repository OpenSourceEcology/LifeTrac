#!/bin/bash
# link_sample.sh <leg> -- during a leg: the first base link_stats message that has
# seen frames, then the first frame line of the base capture.
LEG=${1:?leg}
export MSYS_NO_PATHCONV=1
cd /c/Users/dorkm/Documents/GitHub/LifeTrac/LifeTrac-v25/DESIGN-CONTROLLER
E=bench-evidence/RS_13_vector_scene_2026-09-26/legs
{ echo "# leg $LEG link_stats (base broker): first message with frames; sampler started $(date -u +%Y-%m-%dT%H:%M:%SZ) PC clock"
  for i in $(seq 1 40); do
    m=$(adb -s 2D0A1209DABC240B shell "echo fio | sudo -S -p '' docker exec design-controller-mosquitto-1 mosquitto_sub -h 127.0.0.1 -C 1 -W 40 -t lifetrac/v25/video/link_stats" | tr -d '\r')
    echo "$m" | grep -q '"rx_frames_seen": [1-9]' && { echo "$(date -u +%H:%M:%SZ) $m"; break; }
    sleep 4
  done; } | tee $E/leg${LEG}_link_stats.txt | tail -1 | cut -c1-420
echo "=== first frame at the base"
adb -s 2D0A1209DABC240B shell "echo fio | sudo -S -p '' docker logs rs13_cap_base 2>&1 | grep '^#' | head -1" | tr -d '\r' | cut -c1-140
