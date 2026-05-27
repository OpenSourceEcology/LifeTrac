#!/bin/sh
echo "=== recovery unit status ==="
systemctl status compose-apps-early-start-recovery.service --no-pager -l | head -20
echo "=== recovery unit file ==="
systemctl cat compose-apps-early-start-recovery.service 2>/dev/null | head -30
echo "=== related units ==="
systemctl list-units --all | grep -E 'compose-apps|aklite' | head
