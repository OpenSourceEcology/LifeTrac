#!/bin/sh
sleep 8
echo "=== unit state ==="
systemctl is-active lifetrac-camera.service
echo "=== camera_service / docker compose procs ==="
ps -ef | grep -E 'camera_service|docker.compose' | grep -v grep
echo "=== ttymxc3 counters ==="
grep 30A60000 /proc/tty/driver/IMX-uart
