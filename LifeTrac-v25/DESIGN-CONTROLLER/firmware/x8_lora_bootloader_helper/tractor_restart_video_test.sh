#!/bin/sh
# DO NOT RUN on the bench without an explicit radio GO: starts the
# tractor-vtest mosquitto, camera and image_tx; image_tx opens /dev/ttymxc3 =
# transmits.
# May-2026 one-off; see bench_tools/BENCH_BOARDS.md "Retired boot units".
# Restart tractor video-test services and show image_tx logs.

set -u

if [ "$EUID" != "0" ]; then
  echo "ERROR: run as root"
  exit 1
fi

COMPOSE_DIR=/var/rootdirs/opt/lifetrac/video-test
cd "$COMPOSE_DIR" || exit 2

docker compose -f docker-compose.yml -p tractor-vtest up -d mosquitto camera image_tx
sleep 8

echo "=== containers ==="
docker ps --format 'table {{.Names}}\t{{.Status}}' | grep -E 'tractor-|NAMES' || true

echo "=== image_tx logs ==="
docker logs --tail 160 tractor-image-tx-v2 2>&1