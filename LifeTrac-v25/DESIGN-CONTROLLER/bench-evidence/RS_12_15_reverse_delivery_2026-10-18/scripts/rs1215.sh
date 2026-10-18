#!/usr/bin/env bash
# session helper: rs1215.sh <health|pre|post|clear|camstop|holders|state|park> [leg]
set -u
. /c/Users/dorkm/Documents/GitHub/LifeTrac-bench-main/LifeTrac-v25/DESIGN-CONTROLLER/firmware/x8_lora_bootloader_helper/bench_tools/lib/bench_env.sh || exit 1
E=$(bench_evidence_dir) || exit 1
what=$1; leg=${2:-}
probe() { board_sh "$1" "$SUDO $PROBE_RUN $2 -u /work/$3 2>&1"; }
case $what in
  camstop) board_sh tractor "$SUDO systemctl stop lifetrac-camera.service; $SUDO docker stop tractor-camera 2>&1; systemctl is-active lifetrac-camera.service" ;;
  holders) for w in tractor base; do echo "$w: [$(board_uart_holders $w)] containers=[$(board_containers $w)]"; done ;;
  health) for w in base tractor; do img=$BASE_IMAGE; [ $w = tractor ] && img=$TRACTOR_PROBE_IMAGE; { echo "### health $w $(date -u +%FT%TZ)"; probe $w $img rs116_health_probe.py; } | tee "$E/session_health_${w}.txt"; done ;;
  clear) { echo "### clear_retained base $(date -u +%FT%TZ)"; board_sh base "$SUDO $WORK_RUN $BASE_IMAGE -u /work/clear_retained.py 2>&1"; } | tee "$E/leg${leg}_clear_retained.txt" ;;
  pre|post) for w in base tractor; do img=$BASE_IMAGE; [ $w = tractor ] && img=$TRACTOR_PROBE_IMAGE; { echo "### $what bracket $w $(date -u +%FT%TZ)"; probe $w $img rs115_stats_probe.py; } > "$E/leg${leg}_${what}_${w}.txt"; echo "$w: $(grep -c . "$E/leg${leg}_${what}_${w}.txt") lines -> leg${leg}_${what}_${w}.txt"; done ;;
  state) for w in tractor base; do img=$BASE_IMAGE; [ $w = tractor ] && img=$TRACTOR_PROBE_IMAGE; echo "$w: $(probe $w $img radio_state.py | tail -1)"; done ;;
  park) for w in tractor base; do img=$BASE_IMAGE; [ $w = tractor ] && img=$TRACTOR_PROBE_IMAGE; echo "$w: $(probe $w $img radio_park.py | tail -1)"; done ;;
  *) echo "usage"; exit 2 ;;
esac
