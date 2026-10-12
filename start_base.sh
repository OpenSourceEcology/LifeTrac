#!/bin/sh
# DO NOT RUN on the bench without an explicit radio GO: starts
# lifetrac-base.service (retired on the bench base 2026-10-12), which builds on
# the board, retags lifetrac-v25:latest and starts web_ui and lora_bridge.
# May-2026 one-off; see BENCH_BOARDS.md "Retired boot units".
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
