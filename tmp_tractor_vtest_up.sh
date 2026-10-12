#!/bin/sh
# DO NOT RUN on a bench board, not even with a radio GO: stops the
# lifetrac-camera compose services, then starts the tractor-vtest stack, whose
# image_tx opens /dev/ttymxc3 = transmits.
# May-2026 one-off; see BENCH_BOARDS.md "Retired boot units", rule 6.
set -eu
cd /opt/lifetrac/compose-apps/lifetrac-camera
echo '--- BEFORE ---'
docker ps --format '{{.Names}}\t{{.Status}}'
echo '--- stopping camera + mosquitto (production) ---'
docker compose stop camera mosquitto || true
echo '--- starting vtest stack ---'
cd /opt/lifetrac/video-test
docker compose -p tractor-vtest up -d
sleep 4
echo '--- AFTER ---'
docker ps --format '{{.Names}}\t{{.Status}}'
echo '--- image_tx logs ---'
docker logs --tail 50 tractor-image-tx-v2 || true
echo '--- camera logs ---'
docker logs --tail 30 tractor-camera-v2 || true
echo '--- mosquitto logs ---'
docker logs --tail 20 tractor-mosquitto-v2 || true
