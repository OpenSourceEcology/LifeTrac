#!/bin/sh
# DO NOT RUN on the bench without an explicit radio GO: re-installs and enables
# lifetrac-tractor-compose.service, a unit retired 2026-10-12 (image_tx on
# /dev/ttymxc3 at every boot = transmits).
# May-2026 one-off; see BENCH_BOARDS.md "Retired boot units".
set -eu
install -m 0644 /tmp/lifetrac-tractor-compose.service /etc/systemd/system/lifetrac-tractor-compose.service
systemctl daemon-reload
systemctl enable lifetrac-tractor-compose.service
systemctl is-enabled lifetrac-tractor-compose.service
echo "OK: tractor unit installed and enabled (not starting now — stack is already up)"
