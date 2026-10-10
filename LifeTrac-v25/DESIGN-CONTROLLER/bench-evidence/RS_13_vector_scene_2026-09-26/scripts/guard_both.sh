#!/bin/bash
# guard_both.sh -- wait (<= 115 min) for the bench X8s to come up after a power cycle. As each
# board appears: (tractor) stop lifetrac-camera, which grabs /dev/ttymxc3 at boot; stage only
# the probe tools; read the L072 and park it to 0x80 SLEEP if it is listening; re-read. Then run
# the read-only pull_board_state.sh tractor --images. Never transmits.
export MSYS_NO_PATHCONV=1
S="echo fio | sudo -S -p ''"
DC="C:/Users/dorkm/Documents/GitHub/LifeTrac/LifeTrac-v25/DESIGN-CONTROLLER"
H="$DC/firmware/x8_lora_bootloader_helper"
R="docker run --rm --network=host --entrypoint python3 --device=/dev/ttymxc3 -v /tmp/lifetrac_strict:/work -w /work -e PYTHONPATH=/work:/work/paho"
LOG=/c/Users/dorkm/AppData/Local/Temp/claude/C--Users-dorkm-Documents-GitHub-LifeTrac/5eaec8c2-12ac-4272-80af-b19d1a563f48/scratchpad/guard_both.log
stamp() { echo "$(date -u +%H:%M:%SZ) $*" | tee -a "$LOG"; }
declare -A done_ seen_old
end=$(( $(date +%s) + 6900 ))
stamp "guard armed for base 2D0A + tractor 2E2C (until $(date -u -d @$end +%H:%M:%SZ))"

handle() {   # $1 serial, $2 role
  local s=$1 role=$2 img=lifetrac-v25:latest st
  [ $role = tractor ] && img=hub.foundries.io/arduino/arduino-ootb-python-devel:738bc44
  stamp "$role $s up: $(adb -s $s shell uptime | tr -d '\r')"
  if [ $role = tractor ]; then
    adb -s $s shell "$S systemctl stop lifetrac-camera.service; $S docker stop tractor-camera" 2>&1 | tr -d '\r' | sed 's/^/   /' | tee -a "$LOG"
    stamp "tractor lifetrac-camera: $(adb -s $s shell 'systemctl is-active lifetrac-camera.service' | tr -d '\r')"
  fi
  stamp "$role containers: $(adb -s $s shell "$S docker ps --format '{{.Names}}'" | tr -d '\r' | tr '\n' ' '); ttymxc3 holders: [$(adb -s $s shell "$S fuser /dev/ttymxc3 2>/dev/null" | tr -d '\r ')]"
  adb -s $s shell "$S mkdir -p /tmp/lifetrac_strict/legs; $S chmod 0777 /tmp/lifetrac_strict /tmp/lifetrac_strict/legs" >/dev/null
  for f in "$H/method_g_stage1_probe.py" "$H/method_h_stage2_tx_probe_v2.py" "$H/rs115_stats_probe.py" "$H/rs116_health_probe.py" "$H/bench_tools/radio_park.py" "$H/bench_tools/radio_state.py"; do
    adb -s $s push "$f" /tmp/lifetrac_strict/ >/dev/null || stamp "PUSH FAILED $f"
  done
  adb -s $s shell "$S rm -rf /tmp/lifetrac_strict/paho" >/dev/null
  adb -s $s push "C:/Users/dorkm/Documents/GitHub/LifeTrac/_paho_pull/paho" /tmp/lifetrac_strict/ >/dev/null
  st=$(adb -s $s shell "$S $R $img -u /work/radio_state.py 2>&1 | tail -1" | tr -d '\r')
  stamp "$role radio: $st"
  if ! echo "$st" | grep -q '"0x80"'; then
    stamp "$role park: $(adb -s $s shell "$S $R $img -u /work/radio_park.py 2>&1 | tail -1" | tr -d '\r')"
    sleep 20
    stamp "$role radio after park: $(adb -s $s shell "$S $R $img -u /work/radio_state.py 2>&1 | tail -1" | tr -d '\r')"
  fi
}

while [ $(date +%s) -lt $end ]; do
  for x in 2E2C1209DABC240B:tractor 2D0A1209DABC240B:base; do
    s=${x%%:*}; role=${x##*:}
    [ -n "${done_[$s]}" ] && continue
    adb devices | awk 'NR>1 && $2=="device"{print $1}' | grep -qx $s || continue
    adb -s $s shell true >/dev/null 2>&1 || continue
    up=$(adb -s $s shell "cut -d. -f1 /proc/uptime" | tr -d '\r ')
    if [ "${up:-0}" -gt 1800 ] 2>/dev/null; then   # up > 30 min: not this power cycle; wait for it to drop and return
      [ -z "${seen_old[$s]}" ] && { stamp "$role visible but up ${up}s (not freshly booted); waiting for the power cycle"; seen_old[$s]=1; }
      continue
    fi
    handle $s $role; done_[$s]=1
    if [ $role = tractor ]; then
      stamp "tractor capture start"
      bash "$H/bench_tools/pull_board_state.sh" tractor --images 2>&1 | tee -a "$LOG" | tail -8
      stamp "tractor capture done; radio at end: $(adb -s $s shell "$S $R hub.foundries.io/arduino/arduino-ootb-python-devel:738bc44 -u /work/radio_state.py 2>&1 | tail -1" | tr -d '')"
    fi
  done
  [ -n "${done_[2E2C1209DABC240B]}" ] && [ -n "${done_[2D0A1209DABC240B]}" ] && break
  sleep 3
done
[ -z "${done_[2E2C1209DABC240B]}" ] && stamp "tractor did not appear before the deadline"
[ -z "${done_[2D0A1209DABC240B]}" ] && stamp "base did not appear (or was not power-cycled) before the deadline"
stamp "guard finished"
