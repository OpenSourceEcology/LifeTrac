#!/bin/sh
echo "=== script ==="
cat /usr/bin/compose-apps-early-start-recovery
echo "=== stopping recovery loop ==="
systemctl stop compose-apps-early-start-recovery.service
systemctl mask compose-apps-early-start-recovery.service
echo "=== verify ==="
systemctl is-active compose-apps-early-start-recovery.service
echo "=== also stop the parent early-start unit so it doesn't retrigger recovery ==="
systemctl stop compose-apps-early-start.service 2>/dev/null
systemctl mask compose-apps-early-start.service 2>/dev/null
echo "=== ready ==="
