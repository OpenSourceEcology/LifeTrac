#!/bin/sh
# DO NOT RUN on the bench without an explicit radio GO: re-creates the
# tractor-vtest camera and image_tx; image_tx opens /dev/ttymxc3 = transmits.
# May-2026 one-off; see BENCH_BOARDS.md "Retired boot units".
set -eu
cd /opt/lifetrac/video-test
docker compose -p tractor-vtest up -d --force-recreate camera image_tx
sleep 12
echo '--- ps ---'
docker ps --format '{{.Names}}\t{{.Status}}' | grep -E v2
echo '--- camera tail 25 ---'
docker logs --tail 25 tractor-camera-v2
echo '--- image_tx tail 50 ---'
docker logs --tail 50 tractor-image-tx-v2
