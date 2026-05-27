#!/bin/sh
echo === all listening ports ===
ss -tlnp 2>/dev/null
echo === lifetrac units ===
systemctl list-units --no-pager | grep -i lifetrac
echo === docker ===
docker ps --format 'table {{.Names}}\t{{.Status}}\t{{.Ports}}' 2>/dev/null
echo === processes on 8080 ===
fuser -n tcp 8080 2>/dev/null
ls -la /proc/$(fuser -n tcp 8080 2>/dev/null | awk '{print $NF}')/cwd 2>/dev/null
