#!/bin/sh
# DO NOT RUN on a bench base: lists the compose files' build settings, then
# builds on the board (`docker compose build`). A build that succeeds moves
# the tag lifetrac-v25:latest off the flown image, and no :previous tag is
# kept. Starts no container. Build with bench_tools/deploy_base.sh instead.
# May-2026 one-off; see BENCH_BOARDS.md "Retired boot units", rule 8.
set -eu
cd /opt/lifetrac/DESIGN-CONTROLLER
echo '=== docker-compose.yml ==='
grep -nE 'image:|build:|context:|dockerfile' docker-compose.yml || true
echo
echo '=== docker-compose.video-test.yml ==='
grep -nE 'image:|build:|context:|dockerfile' docker-compose.video-test.yml || true
echo
echo '=== Dockerfile present? ==='
ls -la Dockerfile 2>/dev/null || echo NO_DOCKERFILE
echo
echo '=== try compose build with main file ==='
docker compose -p lifetrac-vtest -f docker-compose.yml build 2>&1 | tail -30 || true
echo
echo '=== images ==='
docker images
