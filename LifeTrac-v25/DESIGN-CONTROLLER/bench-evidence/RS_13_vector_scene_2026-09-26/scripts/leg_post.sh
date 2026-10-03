#!/bin/bash
# leg_post.sh <leg> <archive_dir_windows_path>
# After "[EVIDENCE] archived to": wait for the base capture, collect its summary +
# files, post-brackets, rs12_leg_report, frag_gap_report, P2/P4 greps, park both
# radios (PARK_OK), copy transcripts into legs/.
set -u
LEG=${1:?leg}; ARCH=${2:?archive dir}
export MSYS_NO_PATHCONV=1
DC="/c/Users/dorkm/Documents/GitHub/LifeTrac/LifeTrac-v25/DESIGN-CONTROLLER"
E="$DC/bench-evidence/RS_13_vector_scene_2026-09-26/legs"
SP="/c/Users/dorkm/AppData/Local/Temp/claude/C--Users-dorkm-Documents-GitHub-LifeTrac/5eaec8c2-12ac-4272-80af-b19d1a563f48/scratchpad"
S="echo fio | sudo -S -p ''"
B="adb -s 2D0A1209DABC240B shell"; T="adb -s 2E2C1209DABC240B shell"
BI=lifetrac-v25:latest; TI=hub.foundries.io/arduino/arduino-ootb-python-devel:738bc44
R="docker run --rm --network=host --entrypoint python3 --device=/dev/ttymxc3 -v /tmp/lifetrac_strict:/work -w /work -e PYTHONPATH=/work:/work/paho"
stamp() { echo "$(date -u +%H:%M:%SZ) $*"; }
hdr() { echo "# leg $LEG $1, $(date -u +%Y-%m-%dT%H:%M:%SZ) PC clock"; }
ARCHU=$(echo "$ARCH" | sed -e 's#\\#/#g' -e 's#^C:#/c#')
ARCHW=$(echo "$ARCH" | sed -e 's#\\#/#g')                      # Windows-style for adb pull and python
EW="C:/Users/dorkm/Documents/GitHub/LifeTrac/LifeTrac-v25/DESIGN-CONTROLLER/bench-evidence/RS_13_vector_scene_2026-09-26/legs"

stamp "== leg $LEG post (archive $ARCHU)"
stamp "-- waiting for the base capture (rs13_cap_base) to finish its 480 s"
$B "$S docker wait rs13_cap_base >/dev/null 2>&1; $S docker logs rs13_cap_base > /tmp/lifetrac_strict/legs/leg${LEG}_base.txt 2>&1; $S docker rm -f rs13_cap_base >/dev/null 2>&1; sed -n '/== vector dry run summary ==/,\$p' /tmp/lifetrac_strict/legs/leg${LEG}_base.txt" | tr -d '\r'
for f in leg${LEG}_base.txt leg${LEG}_base.jsonl leg${LEG}_base.json; do adb -s 2D0A1209DABC240B pull /tmp/lifetrac_strict/legs/$f "$EW/$f" 2>&1 | tail -1; done

stamp "-- post-brackets (rs115) both boards"
{ hdr "post bracket base"; $B "$S $R $BI -u /work/rs115_stats_probe.py 2>&1"; } | tr -d '\r' > "$E/leg${LEG}_post_base.txt"
{ hdr "post bracket tractor"; $T "$S $R $TI -u /work/rs115_stats_probe.py 2>&1"; } | tr -d '\r' > "$E/leg${LEG}_post_tractor.txt"
grep -h -E "^radio_(rx_ok|tx_ok|crc_err)=" "$E/leg${LEG}_post_base.txt" | tr '\n' ' '; echo " (base)"
grep -h -E "^radio_(rx_ok|tx_ok|crc_err)=" "$E/leg${LEG}_post_tractor.txt" | tr '\n' ' '; echo " (tractor)"

stamp "-- rs12_leg_report + frag_gap_report"
( cd "$DC" && PYTHONIOENCODING=utf-8 py -3 tools/rs12_leg_report.py "$ARCHW" --pre "$EW/leg${LEG}_pre_base.txt" --post "$EW/leg${LEG}_post_base.txt" 2>&1 ) | tee "$E/leg${LEG}_report.txt" | head -6
( cd "$DC" && PYTHONIOENCODING=utf-8 py -3 firmware/x8_lora_bootloader_helper/bench_tools/frag_gap_report.py "$ARCHW" 2>&1 ) | tee "$E/leg${LEG}_gaps.txt" | head -8

stamp "-- P2 / P4 from the archive"
{ echo "# leg $LEG P2/P4 greps on $ARCHU"
  echo -n "published frame_id lines (rx_daemon.log): "; grep -c "published frame_id" "$ARCHU/rx_daemon.log"
  echo -n "tx done K=1 lines: "; grep -cE "done( \(pipelined\))?: 1 fragments ok" "$ARCHU/tx_daemon.log"
  echo -n "tx done K>=2 lines: "; grep -cE "done( \(pipelined\))?: ([2-9]|[1-9][0-9]+) fragments ok" "$ARCHU/tx_daemon.log"
  echo -n "tx ABORTED lines: "; grep -cE "ABORTED( \(pipelined\))?" "$ARCHU/tx_daemon.log"
  echo "max K: $(grep -oE 'done( \(pipelined\))?: [0-9]+ fragments ok' "$ARCHU/tx_daemon.log" | grep -oE '[0-9]+ fragments' | sort -n | tail -1)"
  echo -n "camera_service vector_stats lines: "; grep -c vector_stats "$ARCHU/camera_service.log" 2>/dev/null
} | tee "$E/leg${LEG}_p2p4.txt"
( cd "$DC" && PYTHONIOENCODING=utf-8 py -3 tools/vector_dry_run.py tractor-log "$ARCHW/camera_service.log" 2>&1 ) | tee "$E/leg${LEG}_tractor_log.txt"
cp "$ARCHU/params.txt" "$E/leg${LEG}_params.txt" 2>/dev/null
[ -f "$SP/leg${LEG}_harness.txt" ] && cp "$SP/leg${LEG}_harness.txt" "$E/leg${LEG}_harness.txt"

stamp "-- park both radios"
{ hdr "park"; echo -n "base:    "; $B "$S $R $BI -u /work/radio_park.py 2>&1 | tail -1"; echo -n "tractor: "; $T "$S $R $TI -u /work/radio_park.py 2>&1 | tail -1"; } | tr -d '\r' | tee "$E/leg${LEG}_park.txt"
stamp "== leg $LEG post done"
