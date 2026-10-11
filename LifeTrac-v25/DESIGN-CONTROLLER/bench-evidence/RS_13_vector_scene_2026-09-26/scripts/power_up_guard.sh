#!/bin/bash
# power_up_guard.sh -- wait for the two bench X8s to come back after a power
# cycle and, as each one appears, keep its L072 radio untouched:
#   tractor: stop lifetrac-camera.service + the tractor-camera container (they
#            map /dev/ttymxc3, the radio UART, at boot; both restart next boot)
#   both:    report containers, units and any /dev/ttymxc3 holder.
# Sends nothing to the radio. Exits when both boards are handled, or after 60 min.
export MSYS_NO_PATHCONV=1
S="echo fio | sudo -S -p ''"
LOG=/c/Users/dorkm/AppData/Local/Temp/claude/C--Users-dorkm-Documents-GitHub-LifeTrac/5eaec8c2-12ac-4272-80af-b19d1a563f48/scratchpad/power_up_guard.log
declare -A done_
end=$(( $(date +%s) + 3600 ))
stamp() { echo "$(date -u +%H:%M:%SZ) $*" | tee -a "$LOG"; }
stamp "guard armed; waiting for 2E2C1209DABC240B (tractor) and 2D0A1209DABC240B (base)"
while [ $(date +%s) -lt $end ]; do
  for s in 2E2C1209DABC240B 2D0A1209DABC240B; do
    [ -n "${done_[$s]}" ] && continue
    adb devices | awk 'NR>1 && $2=="device"{print $1}' | grep -qx "$s" || continue
    adb -s $s shell true >/dev/null 2>&1 || continue
    name=base; [ $s = 2E2C1209DABC240B ] && name=tractor
    stamp "$name $s is up: $(adb -s $s shell uptime | tr -d '\r')"
    if [ $name = tractor ]; then
      adb -s $s shell "$S systemctl stop lifetrac-camera.service; $S docker stop tractor-camera" 2>&1 | tr -d '\r' | sed 's/^/   /' | tee -a "$LOG"
      stamp "tractor: lifetrac-camera.service -> $(adb -s $s shell 'systemctl is-active lifetrac-camera.service' | tr -d '\r')"
    fi
    stamp "$name containers: $(adb -s $s shell "$S docker ps --format '{{.Names}}'" | tr -d '\r' | tr '\n' ' ')"
    stamp "$name /dev/ttymxc3 holders (pids): [$(adb -s $s shell "$S fuser /dev/ttymxc3 2>/dev/null" | tr -d '\r ')]"
    done_[$s]=1
  done
  [ -n "${done_[2E2C1209DABC240B]}" ] && [ -n "${done_[2D0A1209DABC240B]}" ] && { stamp "both boards handled"; exit 0; }
  sleep 3
done
stamp "TIMEOUT: not all boards appeared within 60 min"; exit 2
