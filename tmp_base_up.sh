#!/bin/sh
# DO NOT RUN on a bench board, not even with a radio GO: starts the
# lifetrac-vtest stack, whose image_rx opens /dev/ttymxc3 with a RESET_REQ that
# un-parks the L072 (REG_PROFILE 0, fixed 915 MHz).
# May-2026 one-off; see BENCH_BOARDS.md "Retired boot units", rule 7.
set -eu
cd /opt/lifetrac/DESIGN-CONTROLLER
echo '--- bringing up video-test stack ---'
docker compose -p lifetrac-vtest -f docker-compose.video-test.yml up -d --remove-orphans 2>&1 | tail -30
echo
echo '--- ps ---'
docker ps --format 'table {{.Names}}\t{{.Status}}\t{{.Image}}'
