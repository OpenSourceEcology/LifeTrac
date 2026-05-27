#!/bin/sh
echo "=== UNIT ==="
systemctl is-active base-bringup.service
echo "=== PS ==="
docker ps
echo "=== JOURNAL ==="
journalctl -u base-bringup.service -n 30 --no-pager | tail -30
echo "=== PORT ==="
ss -tln 2>/dev/null | grep 8080 || netstat -tln 2>/dev/null | grep 8080
