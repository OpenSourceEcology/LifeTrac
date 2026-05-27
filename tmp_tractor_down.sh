#!/bin/sh
set -eu
echo "=== Tractor compose down ==="
cd /opt/lifetrac/video-test
docker compose -p tractor-vtest down --remove-orphans || true
echo
echo "=== Tractor running containers (should be empty) ==="
docker ps --format 'table {{.Names}}\t{{.Status}}'
