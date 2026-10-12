#!/bin/sh
# DO NOT RUN on a bench base: deletes the whole docker build cache and every
# image that no container uses (`docker builder prune -af`,
# `docker image prune -af`), then builds on the board. A build that succeeds
# moves the tag lifetrac-v25:latest off the flown image, and no :previous tag
# is kept. Starts no container. Build with bench_tools/deploy_base.sh instead.
# May-2026 one-off; see BENCH_BOARDS.md "Retired boot units", rule 8.
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
