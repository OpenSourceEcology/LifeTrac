#!/bin/sh
echo "=== STATE ==="
systemctl is-active lifetrac-base.service
echo "=== DOCKER ==="
docker ps
echo "=== 8080 ==="
ss -tln 2>/dev/null | grep 8080
netstat -tln 2>/dev/null | grep 8080
echo "=== JOURNAL ==="
journalctl -u lifetrac-base.service -n 20 --no-pager
