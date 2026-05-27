#!/bin/sh
# Inspect basestation state for video-test bringup
echo "=== docker ps ==="
echo fio | sudo -S docker ps --format "{{.Names}}	{{.Status}}"
echo
echo "=== /dev/ttymxc3 owner ==="
echo fio | sudo -S fuser -v /dev/ttymxc3 2>&1 || echo "fuser: nobody"
echo
echo "=== uart counters ==="
cat /proc/tty/driver/IMX-uart 2>/dev/null | grep ttymxc3 || echo "no driver file"
echo
echo "=== compose project at /opt/lifetrac/DESIGN-CONTROLLER ==="
ls -la /opt/lifetrac/DESIGN-CONTROLLER/docker-compose*.yml 2>&1
echo
echo "=== systemd-run base-bringup status ==="
echo fio | sudo -S systemctl status base-bringup.service --no-pager 2>&1 | head -8
echo
echo "=== lifetrac-v25 image ==="
echo fio | sudo -S docker images lifetrac-v25:latest --format "{{.Repository}}:{{.Tag}}	{{.CreatedSince}}	{{.Size}}"
