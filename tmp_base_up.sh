#!/bin/sh
set -eu
cd /opt/lifetrac/DESIGN-CONTROLLER
echo '--- bringing up video-test stack ---'
docker compose -p lifetrac-vtest -f docker-compose.video-test.yml up -d --remove-orphans 2>&1 | tail -30
echo
echo '--- ps ---'
docker ps --format 'table {{.Names}}\t{{.Status}}\t{{.Image}}'
