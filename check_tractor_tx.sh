#!/bin/sh
echo "=== camera service unit ==="
systemctl is-active lifetrac-camera.service
echo "=== camera containers ==="
docker ps --filter "name=camera" 
echo "=== processes (image_tx, camera_service) ==="
ps -ef | grep -E "image_tx|camera_service" | grep -v grep
echo "=== camera service logs (last 30) ==="
journalctl -u lifetrac-camera.service -n 30 --no-pager | tail -30
echo "=== ttymxc3 tractor counters ==="
grep '30A60000' /proc/tty/driver/IMX-uart 2>/dev/null
echo "=== compose logs (last 30) ==="
cd /opt/lifetrac/compose-apps/lifetrac-camera 2>/dev/null && docker compose logs --tail 30 2>&1 | tail -30
