#!/bin/bash
# step1_pass_fix.sh <bw250|bw500> <scene>  -- RS-13.1 step-1 pass with the FIX-BRANCH
# encoder: identical to step1_pass.sh except camera_service runs from the pushed
# /work tree (-w /work, /work/camera_service.py) so /work/x8_image_pipeline (the
# new encoder + vs1_codec) shadows the image's /app copy, and every output file
# carries a _fix suffix. Camera only, no radio (LIFETRAC_USE_LORA_BRIDGE=1 publishes
# to the local broker; nothing keys the L072).
BW=${1:?bw250|bw500}; SCENE=${2:?scene}
PROFILE="image_${BW}"; TAG="${BW}_${SCENE}_fix"
SP="/c/Users/dorkm/AppData/Local/Temp/claude/C--Users-dorkm-Documents-GitHub-LifeTrac/5eaec8c2-12ac-4272-80af-b19d1a563f48/scratchpad"
T="adb -s 2E2C1209DABC240B shell"
L=/tmp/lifetrac_strict/legs
S="echo fio | sudo -S -p ''"
stamp() { echo "$(date -u +%H:%M:%SZ) $*"; }

stamp "== step-1 pass $TAG (profile $PROFILE, encoder from /work)"
if [ "$($T "ls $L/step1_tractor_$TAG.jsonl 2>/dev/null | wc -l" | tr -d '\r ')" != "0" ]; then
  stamp "ABORT: $L/step1_tractor_$TAG.jsonl already exists (would append)"; exit 2
fi
$T "$S docker rm -f camera_svc rs13_cap >/dev/null 2>&1" >/dev/null 2>&1
# a fresh bench broker: a retained encode_mode_override / link_budget left by a radio
# leg (the tractor daemon republishes the base's 0x63 commands here) would flip the
# camera off VECTOR or onto another profile's budget (2026-10-03 ABORTED1)
$T "$S docker rm -f bench_mqtt >/dev/null 2>&1; $S docker run -d --name bench_mqtt --network=host -v /tmp/lifetrac_strict/bench_mqtt.conf:/mosquitto/config/mosquitto.conf eclipse-mosquitto:2 >/dev/null; sleep 2" >/dev/null 2>&1
echo -n "retained on the tractor broker (must be empty): "; $T "$S docker exec bench_mqtt timeout 2 mosquitto_sub -h 127.0.0.1 -t 'lifetrac/#' -v --retained-only 2>/dev/null | wc -l" | tr -d ''

# 1. capture first (the base-station store from /work too: same tree as the encoder)
$T "$S docker run --rm --name rs13_cap --network=host -v /tmp/lifetrac_strict:/work -w /work \
  -e PYTHONPATH=/work:/work/paho -e PYTHONUNBUFFERED=1 --entrypoint python3 lifetrac-tractor-x8:latest \
  /work/vector_dry_run.py capture --topic lifetrac/v25/cmd/image_frame --profile $PROFILE \
  --duration 130 --min-frames 200 --out /work/legs/step1_tractor_$TAG.jsonl \
  --json /work/legs/step1_tractor_$TAG.json --strict 2>&1 | tee $L/step1_tractor_$TAG.txt" \
  > "$SP/step1_$TAG.capture.out" 2>&1 &
CAP_PID=$!

t0=$(date +%s)
until [ "$($T "grep -c subscribing $L/step1_tractor_$TAG.txt 2>/dev/null" | tr -d '\r ')" = "1" ]; do
  if [ $(( $(date +%s) - t0 )) -gt 60 ]; then stamp "ABORT: capture never subscribed"; kill $CAP_PID; exit 3; fi
  sleep 1
done
stamp "capture subscribed after $(( $(date +%s) - t0 )) s"

# 2. then the camera, from /work (new encoder)
$T "$S docker run -d --name camera_svc --network=host --device=/dev/video1 \
  -v /tmp/lifetrac_strict:/work -w /work -e PYTHONPATH=/work:/work/paho --entrypoint python3 \
  -e LIFETRAC_MQTT_HOST=127.0.0.1 -e LIFETRAC_CAMERA_SOURCE=v4l2 -e LIFETRAC_CAMERA_DEVICE=/dev/video1 \
  -e LIFETRAC_CAMERA_FPS=2 -e LIFETRAC_USE_LORA_BRIDGE=1 \
  -e LIFETRAC_ENCODE_MODE=9 -e LIFETRAC_VECTOR_DETAIL=80 \
  -e LIFETRAC_FRAGMENT_BUDGET=1 -e LIFETRAC_FRAGMENT_PROFILE=$PROFILE \
  lifetrac-tractor-x8:latest -u /work/camera_service.py" | tr -d '\r' | cut -c1-12 | sed 's/^/camera_svc id /'
stamp "camera_svc launched"

sleep 15
stamp "boot-log check (+15 s):"
$T "$S docker logs camera_svc 2>&1 | grep -E 'byte_budget|link_budget: ->|lora_cmd|encode_mode|clamp|Traceback|frame build failed' | head -8" \
  | tr -d '\r' | cut -c1-200 | tee "$SP/step1_$TAG.bootcheck.txt"
echo -n "vector_stats lines so far: "; $T "$S docker logs camera_svc 2>&1 | grep -c vector_stats" | tr -d '\r'
echo -n "encoder module in use: "; $T "$S docker exec camera_svc python3 -c \"import x8_image_pipeline.encode_vector as ev; print(ev.__file__, hasattr(ev.VectorEncoder, '_tick_ttl'))\" 2>&1" | tr -d '\r'

# 3. wait for the capture, then encoder timing and teardown
wait $CAP_PID
stamp "capture finished"
$T "$S docker logs camera_svc > $L/step1_camera_service_$TAG.log 2>&1; \
  $S docker run --rm -v /tmp/lifetrac_strict:/work -w /work -e PYTHONPATH=/work --entrypoint python3 \
  lifetrac-tractor-x8:latest /work/vector_dry_run.py tractor-log /work/legs/step1_camera_service_$TAG.log \
  2>&1 | tee $L/step1_tractor_log_$TAG.txt; $S docker rm -f camera_svc >/dev/null 2>&1" \
  | tr -d '\r' > "$SP/step1_$TAG.tractorlog.out"
stamp "== capture summary ($L/step1_tractor_$TAG.txt, from '== vector dry run summary ==')"
$T "sed -n '/== vector dry run summary ==/,\$p' $L/step1_tractor_$TAG.txt" | tr -d '\r'
stamp "== tractor-log ($L/step1_tractor_log_$TAG.txt)"
cat "$SP/step1_$TAG.tractorlog.out"
stamp "== pass $TAG done"
