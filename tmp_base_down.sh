#!/bin/sh
set -eu
echo "=== Bringing base stack down ==="
cd /opt/lifetrac/DESIGN-CONTROLLER
docker compose -p lifetrac-vtest -f docker-compose.video-test.yml down || true
echo "=== Remaining base containers ==="
docker ps --format '{{.Names}}\t{{.Status}}' | grep -E 'lifetrac|mosquitto|web_ui|image|audit' || echo "(none)"
