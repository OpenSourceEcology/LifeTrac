#!/bin/sh
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
