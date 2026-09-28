#!/bin/bash
# Push the fix branch's tractor-side files to /tmp/lifetrac_strict on the tractor
# (the harness's own push list for the tractor, plus vector_dry_run.py), so a
# camera-only step-1 pass can run /work/camera_service.py with the NEW encoder
# without rebuilding the image. Read-only on the boards otherwise.
set -e
export MSYS_NO_PATHCONV=1
DC="C:/Users/dorkm/Documents/GitHub/LifeTrac/LifeTrac-v25/DESIGN-CONTROLLER"
T="adb -s ${BOARD:-2E2C1209DABC240B}"
S="echo fio | sudo -S -p ''"
stamp() { echo "$(date -u +%H:%M:%SZ) $*"; }
stamp "== push fix tree to board ${BOARD:-2E2C1209DABC240B} /tmp/lifetrac_strict (branch $(git -C "$DC" rev-parse --abbrev-ref HEAD) @ $(git -C "$DC" rev-parse --short HEAD))"
$T shell "$S mkdir -p /tmp/lifetrac_strict; $S chmod 0777 /tmp/lifetrac_strict; $S rm -rf /tmp/lifetrac_strict/x8_image_pipeline /tmp/lifetrac_strict/image_pipeline" >/dev/null
for f in "$DC/firmware/tractor_x8/camera_service.py" "$DC/firmware/tractor_x8/image_tx_daemon.py" \
         "$DC/base_station/lora_proto.py" "$DC/tools/vector_dry_run.py"; do
  $T push "$f" /tmp/lifetrac_strict/ | tail -1
done
$T push "$DC/base_station/image_pipeline" /tmp/lifetrac_strict/ | tail -1
$T push "$DC/firmware/tractor_x8/x8_image_pipeline" /tmp/lifetrac_strict/ | tail -1
$T shell "find /tmp/lifetrac_strict -name __pycache__ -type d -exec rm -rf {} + 2>/dev/null; \
  md5sum /tmp/lifetrac_strict/camera_service.py /tmp/lifetrac_strict/x8_image_pipeline/encode_vector.py \
         /tmp/lifetrac_strict/x8_image_pipeline/vs1_codec.py /tmp/lifetrac_strict/image_pipeline/vector_scene_store.py \
         /tmp/lifetrac_strict/image_pipeline/vector_scene/codec.py /tmp/lifetrac_strict/vector_dry_run.py" | tr -d '\r'
echo "--- local:"
(cd "$DC" && md5sum firmware/tractor_x8/camera_service.py firmware/tractor_x8/x8_image_pipeline/encode_vector.py \
   firmware/tractor_x8/x8_image_pipeline/vs1_codec.py base_station/image_pipeline/vector_scene_store.py \
   base_station/image_pipeline/vector_scene/codec.py tools/vector_dry_run.py)
stamp "== import smoke inside the image, from /work (new encoder must load with its codec mirror)"
$T shell "$S docker run --rm -v /tmp/lifetrac_strict:/work -w /work -e PYTHONPATH=/work:/work/paho --entrypoint python3 \
  lifetrac-tractor-x8:latest -c \"import x8_image_pipeline.encode_vector as ev, inspect; print('encoder from', ev.__file__); print('codec', ev.vs.__file__, 'TTL_FRAMES', ev.vs.TTL_FRAMES); print('has _tick_ttl', hasattr(ev.VectorEncoder, '_tick_ttl'))\"" | tr -d '\r'
