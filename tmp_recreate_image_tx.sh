#!/bin/sh
# DO NOT RUN on a bench board, not even with a radio GO: re-creates the
# tractor-vtest image_tx, which opens /dev/ttymxc3 = transmits.
# May-2026 one-off; see BENCH_BOARDS.md "Retired boot units", rule 6.
set -eu
cd /opt/lifetrac/video-test
docker compose -p tractor-vtest up -d --force-recreate image_tx
sleep 6
echo '--- ps ---'
docker ps --format '{{.Names}}\t{{.Status}}' | grep -E v2
echo '--- image_tx logs ---'
docker logs --tail 80 tractor-image-tx-v2
