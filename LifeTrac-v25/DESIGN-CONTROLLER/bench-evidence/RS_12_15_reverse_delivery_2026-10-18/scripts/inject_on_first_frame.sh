#!/usr/bin/env bash
# inject_on_first_frame.sh <leg> <injector args...>: start an injector on the base the moment
# the base's rx daemon (rx_smoke) logs its first "published frame_id" -- read from the daemon's
# own log, not the harness's 2-line tail (leg W's lesson).
set -u
. /c/Users/dorkm/Documents/GitHub/LifeTrac-bench-main/LifeTrac-v25/DESIGN-CONTROLLER/firmware/x8_lora_bootloader_helper/bench_tools/lib/bench_env.sh || exit 1
E=$(bench_evidence_dir) || exit 1
LEG=$1; shift; INJ="$*"; T=$BENCH_SCRATCH/leg${LEG}_harness.txt; O=$E/leg${LEG}_inject.txt
end=$(( $(date +%s) + 420 ))
clean() { tr -d '\000' < "$T" 2>/dev/null; }
# The harness removes the previous leg's rx_smoke at its start; only after MONITORING is the
# running rx_smoke this leg's, so the old container's logs cannot fire the trigger.
until [ -f "$T" ] && clean | grep -q "MONITORING RADIO TRANSMISSIONS"; do
  [ $(date +%s) -gt $end ] && { echo "harness never reached MONITORING -- injector NOT started" | tee "$O"; exit 1; }
  clean | grep -q -E "FATAL|harness exit" && { echo "harness ended before MONITORING -- injector NOT started" | tee "$O"; exit 1; }
  sleep 1
done
until MSYS_NO_PATHCONV=1 adb -s "$BASE_SERIAL" shell "echo fio | sudo -S -p '' docker logs rx_smoke 2>&1 | grep -a -m1 'published frame_id'" 2>/dev/null | grep -q "published frame_id"; do
  [ $(date +%s) -gt $end ] && { echo "no published frame_id within 420 s -- injector NOT started" | tee "$O"; exit 1; }
  clean | grep -q -E "FATAL|harness exit" && { echo "harness ended before the first frame -- injector NOT started" | tee "$O"; exit 1; }
  sleep 1
done
echo "first published frame_id in rx_smoke at $(date -u +%T); starting $INJ on the base" | tee "$O"
board_sh base "$SUDO $WORK_RUN $BASE_IMAGE -u /work/$INJ 2>&1" 2>&1 | grep -v -E "post-quantum|store now|openssh.com/pq" | tee -a "$O" | tail -2
