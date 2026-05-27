#!/bin/sh
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
