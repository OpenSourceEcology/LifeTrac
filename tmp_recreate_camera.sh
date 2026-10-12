#!/bin/sh
# DO NOT RUN on a bench board, not even with a radio GO: re-creates the
# tractor-vtest camera and starts the services it depends on. The camera
# publishes the frames that image_tx sends from /dev/ttymxc3.
# May-2026 one-off; see BENCH_BOARDS.md "Retired boot units", rule 6.
set -eu
cd /opt/lifetrac/video-test
docker compose -p tractor-vtest up -d --force-recreate camera
sleep 8
echo '--- ps ---'
docker ps --format '{{.Names}}\t{{.Status}}' | grep -E v2
echo '--- camera logs (tail 40) ---'
docker logs --tail 40 tractor-camera-v2
echo '--- image_tx logs (tail 30) ---'
docker logs --tail 30 tractor-image-tx-v2
