#!/bin/bash
# stage_boards.sh -- re-stage /tmp/lifetrac_strict on both boards after tmpfiles
# aging (5 d) or a reboot: the radio probe library + probes + bench tools + broker
# conf + paho (BENCH_RUNBOOK prep 1), then the fix branch's code tree
# (push_fix_to_board.sh). Copies files only; nothing here opens the radio UART.
set -e
export MSYS_NO_PATHCONV=1
DC="C:/Users/dorkm/Documents/GitHub/LifeTrac/LifeTrac-v25/DESIGN-CONTROLLER"
H="$DC/firmware/x8_lora_bootloader_helper"
SP="C:/Users/dorkm/AppData/Local/Temp/claude/C--Users-dorkm-Documents-GitHub-LifeTrac/5eaec8c2-12ac-4272-80af-b19d1a563f48/scratchpad"
S="echo fio | sudo -S -p ''"
for b in 2E2C1209DABC240B 2D0A1209DABC240B; do
  echo "=== $b: helper tooling"
  adb -s $b shell "$S mkdir -p /tmp/lifetrac_strict/legs; $S chmod 0777 /tmp/lifetrac_strict /tmp/lifetrac_strict/legs" >/dev/null
  for f in "$H/method_g_stage1_probe.py" "$H/method_h_stage2_tx_probe_v2.py" "$H/rs115_stats_probe.py" "$H/rs116_health_probe.py" \
           "$H/bench_mqtt.conf" "$H/bench_tools/radio_park.py" "$H/bench_tools/radio_state.py" "$H/bench_tools/clear_retained.py" \
           "$H/bench_tools/clear_retained_host.py" "$H/bench_tools/frag_gap_report.py" "$H/bench_tools/kf_inject.py" \
           "$H/bench_tools/dual_inject.py" "$DC/base_station/image_rx_daemon.py" "$DC/base_station/cmd_timing.py"; do
    adb -s $b push "$f" /tmp/lifetrac_strict/ >/dev/null || { echo "PUSH FAILED $f"; exit 1; }
  done
  # root-owned __pycache__ (python in the containers runs as root) blocks an overwrite
  adb -s $b shell "$S rm -rf /tmp/lifetrac_strict/paho" >/dev/null
  adb -s $b push "C:/Users/dorkm/Documents/GitHub/LifeTrac/_paho_pull/paho" /tmp/lifetrac_strict/ >/dev/null
  echo "=== $b: fix tree"
  BOARD=$b bash "$SP/push_fix_to_board.sh" 2>&1 | grep -E "== push|encode_vector.py|encoder from|has _tick" || true
  echo -n "=== $b: staged entries: "; adb -s $b shell "ls /tmp/lifetrac_strict | wc -l; ls /tmp/lifetrac_strict | tr '\n' ' '" | tr -d '\r'; echo
done
