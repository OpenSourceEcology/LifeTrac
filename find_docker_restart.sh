#!/bin/sh
echo "=== units that Stop/Start docker ==="
# Find what triggered the docker stops
journalctl -n 200 --no-pager | grep -E "Stopping Docker|docker.service|Triggered|Started" | tail -60
echo ""
echo "=== systemd timers ==="
systemctl list-timers --no-pager | head -20
echo ""
echo "=== units that Require docker ==="
systemctl list-dependencies --reverse docker.service --no-pager
echo ""
echo "=== Failing units ==="
systemctl --failed --no-pager
