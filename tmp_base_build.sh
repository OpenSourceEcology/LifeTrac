#!/bin/sh
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
