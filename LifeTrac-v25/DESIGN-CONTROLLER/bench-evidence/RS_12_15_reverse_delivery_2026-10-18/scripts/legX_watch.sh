#!/usr/bin/env bash
# legX_watch.sh -- RS-12.15 leg X (scan FAILED): keep the command plane silent, log every
# L072 fault 0x0D the tractor's tx daemon reports, and start the base's dual injector
# for 120 s only when the FINAL RX_SCAN_FAILED arrives (sub bit 7 set, attempt nibble 3,
# counted from 0: 0x00, 0x01, 0x02, then 0x83 as seen in leg W). No final fault within 360 s of the harness's
# MONITORING line -> record "final FAIL not reached" and inject nothing.
set -u
. /c/Users/dorkm/Documents/GitHub/LifeTrac-bench-main/LifeTrac-v25/DESIGN-CONTROLLER/firmware/x8_lora_bootloader_helper/bench_tools/lib/bench_env.sh || exit 1
E=$(bench_evidence_dir) || exit 1
T=$BENCH_SCRATCH/legX_harness.txt
F=$E/legX_faults.txt; I=$E/legX_dual_inject.txt
: > "$F"
log() { printf '%s %s\n' "$(date -u +%T)" "$*" | tee -a "$F"; }
# 1. wait for the monitoring window (or a failure before it)
until [ -f "$T" ] && tr -d '\000' < "$T" | grep -q "MONITORING RADIO TRANSMISSIONS"; do
  tr -d '\000' < "$T" 2>/dev/null | grep -q -E "FATAL|harness exit" && { log "harness ended before MONITORING -- nothing to watch"; exit 1; }
  sleep 1
done
t0=$(date +%s); log "MONITORING seen; watching tx_smoke for code=0x0D (deadline +360 s)"
seen=""
while :; do
  now=$(date +%s)
  if [ $((now - t0)) -gt 360 ]; then log "FINAL RX_SCAN_FAILED NOT REACHED within 360 s -- injector NOT started"; echo "not started: final FAIL not reached" > "$I"; exit 0; fi
  tr -d '\000' < "$T" | grep -q "harness exit" && { log "harness exited before a final FAIL -- injector NOT started"; echo "not started: harness exited" > "$I"; exit 0; }
  lines=$(MSYS_NO_PATHCONV=1 adb -s "$TRACTOR_SERIAL" shell "echo fio | sudo -S -p '' docker logs tx_smoke 2>&1 | grep -a 'code=0x0D'" 2>/dev/null | tr -d '\r')
  final=0
  while IFS= read -r l; do
    [ -n "$l" ] || continue
    case $seen in *"|$l|"*) continue ;; esac
    seen="$seen|$l|"; log "FAULT: $l"
    sub=$(printf '%s\n' "$l" | sed -n 's/.*code=0x0D sub=0x\([0-9A-Fa-f][0-9A-Fa-f]\).*/\1/p')
    [ -n "$sub" ] || continue
    v=$((16#$sub))
    if [ $((v & 0x80)) -ne 0 ] && [ $((v & 0x0F)) -eq 3 ]; then final=1; log "-> FINAL RX_SCAN_FAILED (sub=0x$sub, $((now - t0)) s after MONITORING)"; fi
  done <<< "$lines"
  if [ $final = 1 ]; then
    log "starting dual_inject 120 on the base"
    { echo "started $(date -u +%T) after the final RX_SCAN_FAILED"; board_sh base "$SUDO $WORK_RUN $BASE_IMAGE -u /work/dual_inject.py 120 2>&1"; } 2>&1 | grep -v -E "post-quantum|store now|openssh.com/pq" > "$I"
    log "dual_inject finished: $(tail -1 "$I")"
    exit 0
  fi
  sleep 2
done
