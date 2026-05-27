#!/bin/sh
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
