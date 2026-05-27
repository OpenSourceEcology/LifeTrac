#!/bin/sh
echo "=== IMAGES ==="
docker images
echo "=== FREE ==="
free -m
echo "=== DF ==="
df -h / /var/lib/docker 2>/dev/null
echo "=== DMESG OOM ==="
dmesg | grep -iE 'oom|killed process' | tail -10
echo "=== ENV FILE ==="
ls -la /opt/lifetrac/DESIGN-CONTROLLER/.env 2>/dev/null && cat /opt/lifetrac/DESIGN-CONTROLLER/.env
