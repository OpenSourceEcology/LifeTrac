#!/bin/bash
# tractor_guard_capture.sh -- wait (<= 115 min) for the tractor to come back after a power
# cycle, then: stop lifetrac-camera (it grabs /dev/ttymxc3 at boot), stage only the probe
# tools, read the L072 state and park it to 0x80 SLEEP if it is listening, then run the
# read-only pull_board_state.sh tractor --images. Never transmits.
export MSYS_NO_PATHCONV=1
T=2E2C1209DABC240B
S="echo fio | sudo -S -p ''"
DC="C:/Users/dorkm/Documents/GitHub/LifeTrac/LifeTrac-v25/DESIGN-CONTROLLER"
H="$DC/firmware/x8_lora_bootloader_helper"
TI=hub.foundries.io/arduino/arduino-ootb-python-devel:738bc44
R="docker run --rm --network=host --entrypoint python3 --device=/dev/ttymxc3 -v /tmp/lifetrac_strict:/work -w /work -e PYTHONPATH=/work:/work/paho"
LOG=/c/Users/dorkm/AppData/Local/Temp/claude/C--Users-dorkm-Documents-GitHub-LifeTrac/5eaec8c2-12ac-4272-80af-b19d1a563f48/scratchpad/tractor_guard.log
stamp() { echo "$(date -u +%H:%M:%SZ) $*" | tee -a "$LOG"; }
end=$(( $(date +%s) + 6900 ))
stamp "tractor guard armed (until $(date -u -d @$end +%H:%M:%SZ))"
until adb devices | awk 'NR>1 && $2=="device"{print $1}' | grep -qx $T && adb -s $T shell true >/dev/null 2>&1; do
  [ $(date +%s) -ge $end ] && { stamp "TIMEOUT: tractor did not appear"; exit 2; }
  sleep 3
done
stamp "tractor up: $(adb -s $T shell uptime | tr -d '\r')"
adb -s $T shell "$S systemctl stop lifetrac-camera.service; $S docker stop tractor-camera" 2>&1 | tr -d '\r' | sed 's/^/   /' | tee -a "$LOG"
stamp "lifetrac-camera: $(adb -s $T shell 'systemctl is-active lifetrac-camera.service' | tr -d '\r'); ttymxc3 holders: [$(adb -s $T shell "$S fuser /dev/ttymxc3 2>/dev/null" | tr -d '\r ')]"
stamp "containers: $(adb -s $T shell "$S docker ps --format '{{.Names}}'" | tr -d '\r' | tr '\n' ' ')"
adb -s $T shell "$S mkdir -p /tmp/lifetrac_strict/legs; $S chmod 0777 /tmp/lifetrac_strict /tmp/lifetrac_strict/legs" >/dev/null
for f in "$H/method_g_stage1_probe.py" "$H/method_h_stage2_tx_probe_v2.py" "$H/rs115_stats_probe.py" "$H/rs116_health_probe.py" "$H/bench_tools/radio_park.py" "$H/bench_tools/radio_state.py"; do
  adb -s $T push "$f" /tmp/lifetrac_strict/ >/dev/null || stamp "PUSH FAILED $f"
done
adb -s $T shell "$S rm -rf /tmp/lifetrac_strict/paho" >/dev/null
adb -s $T push "C:/Users/dorkm/Documents/GitHub/LifeTrac/_paho_pull/paho" /tmp/lifetrac_strict/ >/dev/null
st=$(adb -s $T shell "$S $R $TI -u /work/radio_state.py 2>&1 | tail -1" | tr -d '\r')
stamp "radio before: $st"
if ! echo "$st" | grep -q '"0x80"'; then
  stamp "park: $(adb -s $T shell "$S $R $TI -u /work/radio_park.py 2>&1 | tail -1" | tr -d '\r')"
  sleep 20
  stamp "radio after: $(adb -s $T shell "$S $R $TI -u /work/radio_state.py 2>&1 | tail -1" | tr -d '\r')"
fi
stamp "capture start"
bash "/c/Users/dorkm/Documents/GitHub/LifeTrac/LifeTrac-v25/DESIGN-CONTROLLER/firmware/x8_lora_bootloader_helper/bench_tools/pull_board_state.sh" tractor --images 2>&1 | tee -a "$LOG" | tail -8
stamp "capture done"
stamp "radio at end: $(adb -s $T shell "$S $R $TI -u /work/radio_state.py 2>&1 | tail -1" | tr -d '\r')"
