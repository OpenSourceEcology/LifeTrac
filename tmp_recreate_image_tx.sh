#!/bin/sh
set -eu
cd /opt/lifetrac/video-test
docker compose -p tractor-vtest up -d --force-recreate image_tx
sleep 6
echo '--- ps ---'
docker ps --format '{{.Names}}\t{{.Status}}' | grep -E v2
echo '--- image_tx logs ---'
docker logs --tail 80 tractor-image-tx-v2
