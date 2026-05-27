#!/bin/sh
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
