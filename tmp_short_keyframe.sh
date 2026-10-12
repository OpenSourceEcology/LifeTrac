#!/bin/sh
# DO NOT RUN on the bench without an explicit radio GO: edits
# /opt/lifetrac/video-test/docker-compose.yml in place (keyframe period) and
# re-creates the tractor-vtest camera, which publishes the frames that image_tx
# sends from /dev/ttymxc3.
# May-2026 one-off; see BENCH_BOARDS.md "Retired boot units".
set -eu
cd /opt/lifetrac/video-test
grep -nE 'KEYFRAME_PERIOD_S|FPS' docker-compose.yml || true
echo --- sed ---
sed -i 's/LIFETRAC_KEYFRAME_PERIOD_S=60/LIFETRAC_KEYFRAME_PERIOD_S=10/' docker-compose.yml
grep -nE 'KEYFRAME_PERIOD_S' docker-compose.yml || true
echo --- recreate camera ---
docker compose -p tractor-vtest -f docker-compose.yml up -d --no-deps --force-recreate camera
sleep 6
docker logs --tail 8 tractor-camera-v2 2>&1 | tail -8
