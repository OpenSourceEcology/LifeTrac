#!/bin/bash
# leg_prep.sh <leg> <image_bw500|image_bw250> [--no-strict] [--min-frames N]
# RS-13.1 step-2 leg prep per BENCH_RUNBOOK "Prep" + RS13_VECTOR_LEG.md: health
# probes (both boards), clear_retained on the base broker, pre-brackets (both),
# then the base-side vector_dry_run capture started DETACHED on the base (docker -d)
# so it outlives this script and covers the harness's 300 s leg (--duration 480).
set -u
LEG=${1:?leg}; PROFILE=${2:?profile}; shift 2
STRICT="--strict"; MINF=500
while [ $# -gt 0 ]; do case "$1" in --no-strict) STRICT="";; --min-frames) MINF=$2; shift;; esac; shift; done
export MSYS_NO_PATHCONV=1
DC="/c/Users/dorkm/Documents/GitHub/LifeTrac/LifeTrac-v25/DESIGN-CONTROLLER"
E="$DC/bench-evidence/RS_13_vector_scene_2026-09-26/legs"; mkdir -p "$E"
S="echo fio | sudo -S -p ''"
B="adb -s 2D0A1209DABC240B shell"; T="adb -s 2E2C1209DABC240B shell"
BI=lifetrac-v25:latest; TI=hub.foundries.io/arduino/arduino-ootb-python-devel:738bc44
R="docker run --rm --network=host --entrypoint python3 --device=/dev/ttymxc3 -v /tmp/lifetrac_strict:/work -w /work -e PYTHONPATH=/work:/work/paho"
stamp() { echo "$(date -u +%H:%M:%SZ) $*"; }
hdr() { echo "# leg $LEG $1, $(date -u +%Y-%m-%dT%H:%M:%SZ) PC clock"; }

stamp "== leg $LEG prep (profile $PROFILE)"
stamp "-- scene: bench YouTube window to the front + camera check (camera only)"
bash "$(dirname "$0")/scene_check.sh" "leg${LEG}" || { stamp "ABORT: scene check failed - fix the screen/aim before this leg"; exit 4; }
stamp "-- production camera unit / UART holders"
$T "systemctl is-active lifetrac-camera.service; $S docker ps --format '{{.Names}}' | grep -E 'tractor-camera|camera_svc|tx_smoke' ; echo -n 'tractor ttymxc3: '; $S fuser /dev/ttymxc3 2>&1; echo" | tr -d '\r'
$B "$S docker ps --format '{{.Names}}' | grep -E 'rx_smoke|rs13_cap' ; echo -n 'base ttymxc3: '; $S fuser /dev/ttymxc3 2>&1; echo" | tr -d '\r'
$B "$S docker rm -f rs13_cap_base >/dev/null 2>&1" >/dev/null 2>&1

stamp "-- health probes (rs116) both boards"
{ hdr "health base"; $B "$S $R $BI -u /work/rs116_health_probe.py 2>&1"; } | tr -d '\r' | tee "$E/leg${LEG}_health_base.txt" | grep -E "STATS-OK|STATS-FAILED|radio_state|RS12-URC" | head -4
{ hdr "health tractor"; $T "$S $R $TI -u /work/rs116_health_probe.py 2>&1"; } | tr -d '\r' | tee "$E/leg${LEG}_health_tractor.txt" | grep -E "STATS-OK|STATS-FAILED|radio_state|RS12-URC" | head -4

stamp "-- clear retained control topics on the base broker"
{ hdr "clear_retained base broker"; $B "$S docker run --rm --network=host --entrypoint python3 -v /tmp/lifetrac_strict:/work -w /work -e PYTHONPATH=/work:/work/paho $BI -u /work/clear_retained.py 2>&1"; } | tr -d '\r' | tee "$E/leg${LEG}_clear_retained.txt" | tail -1

stamp "-- pre-brackets (rs115) both boards"
{ hdr "pre bracket base"; $B "$S $R $BI -u /work/rs115_stats_probe.py 2>&1"; } | tr -d '\r' > "$E/leg${LEG}_pre_base.txt"
{ hdr "pre bracket tractor"; $T "$S $R $TI -u /work/rs115_stats_probe.py 2>&1"; } | tr -d '\r' > "$E/leg${LEG}_pre_tractor.txt"
grep -h -E "^radio_(rx_ok|tx_ok|crc_err)=" "$E/leg${LEG}_pre_base.txt" | tr '\n' ' '; echo " (base)"
grep -h -E "^radio_(rx_ok|tx_ok|crc_err)=" "$E/leg${LEG}_pre_tractor.txt" | tr '\n' ' '; echo " (tractor)"

stamp "-- base-side capture, detached (docker rs13_cap_base): profile $PROFILE, 480 s, min-frames $MINF ${STRICT:-no-strict}"
$B "$S rm -f /tmp/lifetrac_strict/legs/leg${LEG}_base.jsonl /tmp/lifetrac_strict/legs/leg${LEG}_base.json; \
  $S docker run -d --name rs13_cap_base --network=host -v /tmp/lifetrac_strict:/work -w /work \
  -e PYTHONPATH=/work:/work/paho -e PYTHONUNBUFFERED=1 --entrypoint python3 $BI \
  /work/vector_dry_run.py capture --topic lifetrac/v25/video/tile_delta --profile $PROFILE \
  --duration 480 --min-frames $MINF --out /work/legs/leg${LEG}_base.jsonl --json /work/legs/leg${LEG}_base.json $STRICT" | tr -d '\r' | cut -c1-12 | sed 's/^/rs13_cap_base id /'
sleep 3
$B "$S docker logs rs13_cap_base 2>&1 | head -3" | tr -d '\r'
stamp "== leg $LEG prep done -> launch the harness now"
