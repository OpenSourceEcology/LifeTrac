#!/bin/sh
# DO NOT RUN on the bench without an explicit radio GO: runs a bare
# `docker compose down` that removes the bench broker
# (design-controller-mosquitto-1), then starts the lifetrac-vtest stack, whose
# image_rx opens /dev/ttymxc3 with a RESET_REQ that un-parks the L072.
# May-2026 one-off; see BENCH_BOARDS.md "Retired boot units".
# Bring up the video-test stack, replacing the production stack.
set -e
cd /opt/lifetrac/DESIGN-CONTROLLER
echo "=== stop production stack ==="
echo fio | sudo -S docker compose down
echo
echo "=== bring up video-test stack ==="
echo fio | sudo -S docker compose -f docker-compose.video-test.yml -p lifetrac-vtest up -d
echo
sleep 3
echo "=== container status ==="
echo fio | sudo -S docker compose -f docker-compose.video-test.yml -p lifetrac-vtest ps
echo
echo "=== image_rx logs (last 50 lines) ==="
echo fio | sudo -S docker logs --tail 50 lifetrac-vtest-image_rx-1 2>&1
