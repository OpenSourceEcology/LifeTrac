#!/usr/bin/env bash
# step1_pass.sh -- one RS-13 step-1 pass (camera-only dry run on the tractor, NO
# radio), per RS13_VECTOR_LEG.md step 1: scene check, a FRESH bench_mqtt with 0
# retained messages, capture first (wait for its subscribe line), then camera_svc
# with the /work mount + PYTHONPATH, boot-log check, wait for the capture,
# tractor-log, teardown, and the pass files pulled into the evidence folder.
#
# Usage:   bash step1_pass.sh <bw250|bw500> <scene> [--from-work] [--suffix S]
#                             [--scene-min PCT] [--no-scene-check]
#   bw250|bw500     one-fragment budget/profile (image_bw250 = 203 B, image_bw500 = 243 B)
#   <scene>         name in the file tag, e.g. moving, landscape (scene and suffix:
#                   letters, digits, '_', '.', '-' only)
#   --from-work     run camera_service.py and x8_image_pipeline from the pushed
#                   /tmp/lifetrac_strict tree (push_fix_to_board.sh) instead of the
#                   image's /app copy; the tag gets the suffix _work (was
#                   step1_pass_fix.sh, suffix _fix, in RS-13.1)
#   --suffix S      tag suffix instead of the default ("" or _work)
#   --scene-min P   scene-check threshold in % (default 8 when <scene> contains
#                   "moving", "yt" or "youtube", else 0 = record only, for a still)
#   --no-scene-check
# Inputs:  TRACTOR_SERIAL, TRACTOR_APP_IMAGE; /tmp/lifetrac_strict staged
#          (stage_boards.sh: vector_dry_run.py, lora_proto.py, image_pipeline/,
#          bench_mqtt.conf, paho/).
# Writes:  tractor /tmp/lifetrac_strict/legs/step1_*_<tag>.*; PC $EVIDENCE_DIR/
#          step1_<tag>_scene.{txt,jpg}, step1_bench_mqtt_reset.txt (appended),
#          step1_tractor_<tag>.{jsonl,json,txt}, step1_camera_service_<tag>.log,
#          step1_tractor_log_<tag>.txt, step1_<tag>_bootcheck.txt.
# Boards:  tractor only: recreates the bench_mqtt container (127.0.0.1:1883, no
#          persistence), runs the capture (rs13_cap) and camera_svc (/dev/video1)
#          containers, removes them at the end -- and, through an EXIT trap, also
#          when the pass aborts or is interrupted (only the ones it started).
# Radio:   camera only. Refuses to start while anything holds /dev/ttymxc3 or a
#          leg daemon (tx_smoke / rx_smoke / synth_pub) runs on the tractor: a
#          running tx daemon would put the camera frames on the air.
#          LIFETRAC_USE_LORA_BRIDGE=1 publishes to the local broker only.
# Origin:  bench-evidence/RS_13_vector_scene_2026-09-26/scripts/step1_pass.sh and
#          step1_pass_fix.sh (historical copies, unchanged), merged.
set -u
. "$(dirname "${BASH_SOURCE[0]}")/../lib/bench_env.sh" || exit 1

BW=""; SCENE=""; FROM_WORK=0; SUFFIX="unset"; SMIN=""; DO_SCENE=1
while [ $# -gt 0 ]; do
  case $1 in
    --from-work) FROM_WORK=1 ;;
    --suffix) SUFFIX=${2-}; shift ;;
    --scene-min) SMIN=${2:?--scene-min PCT}; shift ;;
    --no-scene-check) DO_SCENE=0 ;;
    -h|--help) bench_usage; exit 0 ;;
    -*) die "unknown option '$1'" ;;
    *) if [ -z "$BW" ]; then BW=$1; elif [ -z "$SCENE" ]; then SCENE=$1; else die "extra argument '$1'"; fi ;;
  esac
  shift
done
[ -n "$BW" ] && [ -n "$SCENE" ] || { bench_usage; exit 2; }
case $BW in bw250|bw500) ;; *) die "first argument must be bw250 or bw500" ;; esac
bench_check_name "scene name" "$SCENE"
[ "$SUFFIX" = unset ] && { [ $FROM_WORK = 1 ] && SUFFIX=_work || SUFFIX=""; }
[ -z "$SUFFIX" ] || bench_check_name "--suffix" "$SUFFIX"
PROFILE="image_${BW}"; TAG="${BW}_${SCENE}${SUFFIX}"
if [ -z "$SMIN" ]; then case $SCENE in *moving*|*yt*|*youtube*) SMIN=8 ;; *) SMIN=0 ;; esac; fi
if [ $FROM_WORK = 1 ]; then WDIR=/work; SCRIPT=/work/camera_service.py; else WDIR=/app; SCRIPT=camera_service.py; fi
L=$BOARD_STAGE/legs
E=$(bench_evidence_dir) || die "cannot create $EVIDENCE_DIR"
OUT="$BENCH_SCRATCH/step1_$TAG"

stamp "== step-1 pass $TAG (profile $PROFILE, camera_service from $WDIR) from $(bench_git_desc)"
board_present tractor || die "tractor ($TRACTOR_SERIAL) is not reachable via adb"
# refuse to append onto an earlier pass (--out appends)
if [ "$(board_sh tractor "ls $L/step1_tractor_$TAG.jsonl 2>/dev/null | wc -l" | tr -d ' ')" != "0" ]; then
  stamp "ABORT: $L/step1_tractor_$TAG.jsonl already exists (would append) -- pick another scene name or --suffix"; exit 2
fi
# radio safety: nothing may hold the radio UART, no leg daemon may run
holders=$(board_uart_holders tractor); running=$(board_containers tractor)
if [ -n "$holders" ] || echo " $running " | grep -qE ' (tx_smoke|rx_smoke|synth_pub|tractor-camera) '; then
  stamp "ABORT: tractor not quiet (ttymxc3 holders [$holders], containers [$running]). Stop them first (power_up_guard.sh --stop-leg-daemons)."
  exit 3
fi
board_sh tractor "$SUDO docker rm -f camera_svc rs13_cap >/dev/null 2>&1" > /dev/null 2>&1

# On an abort ("never subscribed", an error, Ctrl+C) remove the tractor containers
# this pass started -- a leftover camera_svc would keep publishing frames that a
# later tx daemon puts on the air. Names are added just before each launch and
# dropped once the normal teardown has removed them.
CAP_PID=""; STARTED=""
step1_cleanup() {
  if [ -n "$CAP_PID" ]; then kill "$CAP_PID" 2>/dev/null; fi
  if [ -n "$STARTED" ]; then
    stamp "cleanup: removing the tractor container(s) this pass started:$STARTED"
    board_sh tractor "$SUDO docker rm -f$STARTED >/dev/null 2>&1" > /dev/null 2>&1
    STARTED=""
  fi
}
trap step1_cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

if [ $DO_SCENE = 1 ]; then
  bash "$LEGS_DIR/scene_check.sh" "step1_${TAG}" "$SMIN" || { stamp "ABORT: scene check failed"; exit 4; }
fi

# A fresh bench broker: a retained encode_mode_override / link_budget left by a radio
# leg would flip the camera off VECTOR or onto another profile's budget (A12).
{ echo "# step-1 $TAG bench_mqtt reset, $(date -u +%Y-%m-%dT%H:%M:%SZ) PC clock"
  echo "retained before (existing broker, if any):"
  board_sh tractor "$SUDO docker exec bench_mqtt timeout 2 mosquitto_sub -h 127.0.0.1 -t 'lifetrac/#' -v --retained-only 2>/dev/null"
  board_sh tractor "$SUDO docker rm -f bench_mqtt >/dev/null 2>&1; $SUDO docker run -d --name bench_mqtt --network=host -v $BOARD_STAGE/bench_mqtt.conf:/mosquitto/config/mosquitto.conf eclipse-mosquitto:2 >/dev/null; sleep 2" > /dev/null 2>&1
  echo "retained after the recreate (must be 0): $(board_sh tractor "$SUDO docker exec bench_mqtt timeout 2 mosquitto_sub -h 127.0.0.1 -t 'lifetrac/#' -v --retained-only 2>/dev/null | wc -l" | tr -d ' ')"
} | tee -a "$E/step1_bench_mqtt_reset.txt" | tail -1

# 1. capture first
STARTED="$STARTED rs13_cap"
board_sh tractor "$SUDO docker run --rm --name rs13_cap --network=host -v $BOARD_STAGE:/work -w /work \
  -e PYTHONPATH=/work:/work/paho -e PYTHONUNBUFFERED=1 --entrypoint python3 $TRACTOR_APP_IMAGE \
  /work/vector_dry_run.py capture --topic lifetrac/v25/cmd/image_frame --profile $PROFILE \
  --duration 130 --min-frames 200 --out /work/legs/step1_tractor_$TAG.jsonl \
  --json /work/legs/step1_tractor_$TAG.json --strict 2>&1 | tee $L/step1_tractor_$TAG.txt" \
  > "$OUT.capture.out" 2>&1 &
CAP_PID=$!
t0=$(date +%s)
until [ "$(board_sh tractor "grep -c subscribing $L/step1_tractor_$TAG.txt 2>/dev/null" | tr -d ' ')" = "1" ]; do
  if [ $(( $(date +%s) - t0 )) -gt 60 ]; then stamp "ABORT: capture never subscribed"; kill $CAP_PID 2>/dev/null; exit 3; fi
  sleep 1
done
stamp "capture subscribed after $(( $(date +%s) - t0 )) s"

# 2. then the camera
STARTED="$STARTED camera_svc"
board_sh tractor "$SUDO docker run -d --name camera_svc --network=host --device=/dev/video1 \
  -v $BOARD_STAGE:/work -w $WDIR -e PYTHONPATH=/work:/work/paho --entrypoint python3 \
  -e LIFETRAC_MQTT_HOST=127.0.0.1 -e LIFETRAC_CAMERA_SOURCE=v4l2 -e LIFETRAC_CAMERA_DEVICE=/dev/video1 \
  -e LIFETRAC_CAMERA_FPS=2 -e LIFETRAC_USE_LORA_BRIDGE=1 \
  -e LIFETRAC_ENCODE_MODE=9 -e LIFETRAC_VECTOR_DETAIL=80 \
  -e LIFETRAC_FRAGMENT_BUDGET=1 -e LIFETRAC_FRAGMENT_PROFILE=$PROFILE \
  $TRACTOR_APP_IMAGE -u $SCRIPT" | cut -c1-12 | sed 's/^/camera_svc id /'
stamp "camera_svc launched"

sleep 15
stamp "boot-log check (+15 s): expect byte_budget=$([ $BW = bw250 ] && echo 203 || echo 243) B, no 'link_budget: ->', no [lora_cmd]"
board_sh tractor "$SUDO docker logs camera_svc 2>&1 | grep -E 'byte_budget|link_budget: ->|lora_cmd|encode_mode|clamp|Traceback|frame build failed' | head -8" \
  | cut -c1-200 | tee "$E/step1_${TAG}_bootcheck.txt"
echo -n "vector_stats lines so far: "; board_sh tractor "$SUDO docker logs camera_svc 2>&1 | grep -c vector_stats"
echo -n "encoder module in use: "
board_sh tractor "$SUDO docker exec camera_svc python3 -c \"import x8_image_pipeline.encode_vector as ev; print(ev.__file__, hasattr(ev.VectorEncoder, '_tick_ttl'))\" 2>&1" \
  | tee -a "$E/step1_${TAG}_bootcheck.txt"

# 3. wait for the capture, then encoder timing and teardown
wait $CAP_PID
CAP_PID=""
stamp "capture finished"
board_sh tractor "$SUDO docker logs camera_svc > $L/step1_camera_service_$TAG.log 2>&1; \
  $SUDO docker run --rm -v $BOARD_STAGE:/work -w /work -e PYTHONPATH=/work --entrypoint python3 \
  $TRACTOR_APP_IMAGE /work/vector_dry_run.py tractor-log /work/legs/step1_camera_service_$TAG.log \
  2>&1 | tee $L/step1_tractor_log_$TAG.txt; $SUDO docker rm -f camera_svc >/dev/null 2>&1" > "$OUT.tractorlog.out"
STARTED=""                               # torn down: rs13_cap ended (--rm), camera_svc removed
stamp "== capture summary ($L/step1_tractor_$TAG.txt, from '== vector dry run summary ==')"
board_sh tractor "sed -n '/== vector dry run summary ==/,\$p' $L/step1_tractor_$TAG.txt"
stamp "== tractor-log ($L/step1_tractor_log_$TAG.txt)"
cat "$OUT.tractorlog.out"
for f in step1_tractor_$TAG.jsonl step1_tractor_$TAG.json step1_tractor_$TAG.txt step1_camera_service_$TAG.log step1_tractor_log_$TAG.txt; do
  board_pull tractor "$L/$f" "$E/$f" || echo "PULL FAILED $f"
done
stamp "== pass $TAG done (evidence: $E)"
