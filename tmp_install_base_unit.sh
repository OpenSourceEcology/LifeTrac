#!/bin/sh
set -eu
install -m 0644 /tmp/lifetrac-base-compose.service /etc/systemd/system/lifetrac-base-compose.service
systemctl daemon-reload
systemctl enable lifetrac-base-compose.service
systemctl is-enabled lifetrac-base-compose.service
echo "OK: base unit installed and enabled (not starting now — stack is already up)"
