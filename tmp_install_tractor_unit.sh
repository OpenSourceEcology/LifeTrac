#!/bin/sh
set -eu
install -m 0644 /tmp/lifetrac-tractor-compose.service /etc/systemd/system/lifetrac-tractor-compose.service
systemctl daemon-reload
systemctl enable lifetrac-tractor-compose.service
systemctl is-enabled lifetrac-tractor-compose.service
echo "OK: tractor unit installed and enabled (not starting now — stack is already up)"
