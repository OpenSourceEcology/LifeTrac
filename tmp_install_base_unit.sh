#!/bin/sh
# DO NOT RUN on the bench without an explicit radio GO: re-installs and enables
# lifetrac-base-compose.service, a unit retired 2026-10-12 (image_rx on
# /dev/ttymxc3 at every boot, with a RESET_REQ that un-parks the L072).
# May-2026 one-off; see BENCH_BOARDS.md "Retired boot units".
set -eu
install -m 0644 /tmp/lifetrac-base-compose.service /etc/systemd/system/lifetrac-base-compose.service
systemctl daemon-reload
systemctl enable lifetrac-base-compose.service
systemctl is-enabled lifetrac-base-compose.service
echo "OK: base unit installed and enabled (not starting now — stack is already up)"
