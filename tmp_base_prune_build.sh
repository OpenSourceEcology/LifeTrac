#!/bin/sh
set -eu
echo '--- pruning buildkit ---'
docker builder prune -af
echo
echo '--- pruning dangling images ---'
docker image prune -af || true
echo
echo '--- compose build ---'
cd /opt/lifetrac/DESIGN-CONTROLLER
docker compose -p lifetrac-vtest -f docker-compose.yml build 2>&1 | tail -60
echo
echo '--- images ---'
docker images
