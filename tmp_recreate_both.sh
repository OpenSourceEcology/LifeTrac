#!/bin/sh
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
