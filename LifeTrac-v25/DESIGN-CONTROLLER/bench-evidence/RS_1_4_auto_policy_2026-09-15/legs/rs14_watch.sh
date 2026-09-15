#!/bin/bash
# rs14_watch.sh <LEG> <harness-task-output-file> [pause_at_s=90] [pause_len_s=30]
# RS-1.4 live validation of the RS-12.16 auto-policy inputs.
# Instrument: the tractor's FRAME SOURCE (camera_svc) is paused for
# pause_len_s starting pause_at_s after the first published frame. The TX
# daemon stays alive (it can still ACK the profile switch the policy
# commands), so the base sees exactly what a lock loss looks like: dead
# air on the fragment counter with the control plane intact.
# Expected: Auto degrades DTS(2) -> FHSS(1) ~10-15 s into the pause (the
# 60 s min-switch gap since the seed has long passed), the two-phase
# switch completes, frames resume on FHSS after the unpause, and the
# policy promotes back to DTS after its 60 s healthy dwell.
LEG=${1:?leg}; OUT=${2:?task output}; PAUSE_AT=${3:-90}; PAUSE_LEN=${4:-30}
SP="/c/Users/dorkm/AppData/Local/Temp/claude/C--Users-dorkm-Documents-GitHub-LifeTrac/5eaec8c2-12ac-4272-80af-b19d1a563f48/scratchpad"
D="/c/Users/dorkm/Documents/GitHub/LifeTrac/LifeTrac-v25/DESIGN-CONTROLLER"
E="$D/bench-evidence/RS_1_4_auto_policy_2026-09-15/legs"
TOOLS="$D/tools"; BT="$D/firmware/x8_lora_bootloader_helper/bench_tools"
SSH="ssh -i $HOME/.ssh/lifetrac_base_ed25519 -o BatchMode=yes -o ConnectTimeout=8 -o LogLevel=ERROR fio@192.168.1.117"
RUNB="echo fio | sudo -S -p '' docker run --rm --network=host --entrypoint python3 --device=/dev/ttymxc3 -v /tmp/lifetrac_strict:/work -w /work -e PYTHONPATH=/work lifetrac-v25:latest -u"
RUNT="echo fio | sudo -S -p '' docker run --rm --network=host --entrypoint python3 --device=/dev/ttymxc3 -v /tmp/lifetrac_strict:/work -w /work -e PYTHONPATH=/work hub.foundries.io/arduino/arduino-ootb-python-devel:738bc44 -u"
TL="$SP/leg${LEG}_timeline.txt"; : > "$TL"
stamp() { echo "$(date -u +%H:%M:%S.%3N) $*" | tee -a "$TL"; }

t0=$(date +%s)
until grep -a -q 'published frame_id' "$OUT" 2>/dev/null; do
  [ $(( $(date +%s) - t0 )) -gt 280 ] && { stamp "NO FRAME after 280 s"; grep -a -n -i -E "camera|error|Traceback|FATAL" "$OUT" | tail -6 | cut -c1-160; break; }
  sleep 2
done
tf=$(date +%s); stamp "FIRST_FRAME (+$((tf - t0))s after launch)"
stamp "policy state at first frame: $($SSH "echo fio | sudo -S -p '' docker logs bench_webui 2>&1 | grep 'radio_profile' | tail -1" | tr -d '\r' | cut -c1-160)"

sleep $(( PAUSE_AT - ( $(date +%s) - tf ) > 0 ? PAUSE_AT - ( $(date +%s) - tf ) : 0 ))
stamp "PAUSE camera_svc (frame source) for ${PAUSE_LEN}s"
adb -s 2E2C1209DABC240B shell "echo fio | sudo -S -p '' docker pause camera_svc" | tr -d '\r'
for i in $(seq 1 $(( PAUSE_LEN / 5 ))); do
  sleep 5
  stamp "  +$((i*5))s paused | $($SSH "echo fio | sudo -S -p '' docker logs bench_webui --since 6s 2>&1 | grep 'radio_profile\[auto\]:' | tail -1" | tr -d '\r' | cut -c1-150)"
done
stamp "UNPAUSE camera_svc"
adb -s 2E2C1209DABC240B shell "echo fio | sudo -S -p '' docker unpause camera_svc" | tr -d '\r'

# keep sampling the policy until the archive lands
t1=$(date +%s)
until grep -a -q 'archived to' "$OUT" 2>/dev/null; do
  [ $(( $(date +%s) - t1 )) -gt 420 ] && { stamp "TIMEOUT waiting for the archive"; exit 3; }
  sleep 10
  l=$($SSH "echo fio | sudo -S -p '' docker logs bench_webui --since 11s 2>&1 | grep -E 'radio_profile(\[auto\])?: (published|[12] )' | tail -1" | tr -d '\r' | cut -c1-150)
  [ -n "$l" ] && stamp "  POLICY: $l"
done
stamp "ARCHIVED"; sleep 8
ARCH=$(grep -a -o 'archived to .*' "$OUT" | tail -1 | sed 's/archived to //' | tr -d '\r' | sed 's/\x1b\[[0-9;]*m//g')
A=$(cygpath -u "$ARCH"); echo "$ARCH" > "$SP/leg${LEG}_archive.txt"; echo "ARCH=$ARCH"

$SSH "$RUNB /work/rs115_stats_probe.py 2>&1" | tr -d '\r' > "$SP/leg${LEG}_post_base.txt"
adb -s 2E2C1209DABC240B shell "$RUNT /work/rs115_stats_probe.py 2>&1" | tr -d '\r' > "$SP/leg${LEG}_post_tractor.txt"
echo "post-brackets: base $(grep -c '=' "$SP/leg${LEG}_post_base.txt") / tractor $(grep -c '=' "$SP/leg${LEG}_post_tractor.txt")"

echo "=== full policy log (bench_webui)"
$SSH "echo fio | sudo -S -p '' docker logs bench_webui 2>&1 | grep -E 'radio_profile'" | tr -d '\r' | tee "$SP/leg${LEG}_policy_log.txt" | cut -c1-170
echo "=== rx daemon: profile switches commanded / acked / confirmed"
grep -a -E "radio_profile|RADIO_PROFILE|profile.*(ack|ACK|CONF|revert|commanding)" "$A/rx_daemon.log" | cut -c1-170 | tee "$SP/leg${LEG}_daemon_profile.txt" | head -30
echo "=== tractor: profile commands received"
grep -a -E "RADIO_PROFILE|profile" "$A/tx_daemon.log" | cut -c1-150 | head -12
echo "=== leg report + gaps"
py -3 "$TOOLS/rs12_leg_report.py" "$ARCH" --pre "$SP/leg${LEG}_pre_base.txt" --post "$SP/leg${LEG}_post_base.txt" 2>&1 | tee "$SP/leg${LEG}_report.txt" | grep -aE "STALE|log-derived|loss|published" | head -5
py -3 "$BT/frag_gap_report.py" "$ARCH" 2>&1 | tee "$SP/leg${LEG}_gaps.txt" | head -3

mkdir -p "$E"; cp "$SP/leg${LEG}_"*.txt "$E/" 2>/dev/null
echo "evidence copied ($(ls "$E" | grep -c "leg${LEG}_") files)"
