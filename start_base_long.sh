#!/bin/sh
# DO NOT RUN on the bench without an explicit radio GO: re-creates the 1800 s
# TimeoutStartSec drop-in and starts lifetrac-base.service (both retired on the
# bench base 2026-10-12), which builds on the board, retags lifetrac-v25:latest
# and starts web_ui and lora_bridge.
# May-2026 one-off; see BENCH_BOARDS.md "Retired boot units".
set -e
mkdir -p /etc/systemd/system/lifetrac-base.service.d
cat > /etc/systemd/system/lifetrac-base.service.d/timeout.conf <<'EOF'
[Service]
TimeoutStartSec=1800
EOF
systemctl daemon-reload
systemctl reset-failed lifetrac-base.service
systemctl start --no-block lifetrac-base.service
echo "started; polling..."
i=0
while [ $i -lt 1500 ]; do
  sleep 15
  i=$((i+15))
  state=$(systemctl is-active lifetrac-base.service)
  echo "[+${i}s] state=$state"
  if [ "$state" = "active" ] || [ "$state" = "failed" ]; then break; fi
done
echo "=== FINAL ==="
systemctl is-active lifetrac-base.service
docker ps --format "{{.Names}} {{.Status}}"
ss -tln 2>/dev/null | grep 8080 || netstat -tln 2>/dev/null | grep 8080 || echo "no 8080"
journalctl -u lifetrac-base.service -n 10 --no-pager
