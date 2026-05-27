#!/bin/sh
set -x
systemctl reset-failed lifetrac-base.service 2>/dev/null
systemctl start --no-block lifetrac-base.service
i=0
while [ $i -lt 150 ]; do
  sleep 2
  state=$(systemctl is-active lifetrac-base.service)
  echo "[$i] state=$state"
  if [ "$state" = "active" ] || [ "$state" = "failed" ]; then break; fi
  i=$((i+2))
done
echo "=== STATUS ==="
systemctl is-active lifetrac-base.service
echo "=== DOCKER ==="
docker ps --format "table {{.Names}}\t{{.Status}}\t{{.Ports}}"
echo "=== 8080 ==="
ss -tlnp 2>/dev/null | grep 8080
netstat -tlnp 2>/dev/null | grep 8080
echo "=== JOURNAL TAIL ==="
journalctl -u lifetrac-base.service -n 25 --no-pager
