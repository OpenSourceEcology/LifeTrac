#!/bin/sh
# DO NOT RUN on a bench board, not even with a radio GO: runs the ExecStart of
# lifetrac-base.service (retired 2026-10-12) as a transient unit, so it works
# without that unit: `docker compose up -d --build --remove-orphans` builds on
# the board, retags lifetrac-v25:latest and starts web_ui and lora_bridge.
# May-2026 one-off; see BENCH_BOARDS.md "Retired boot units", rule 1.
echo "=== stop any half-running bringup ==="
systemctl stop base-bringup.service 2>/dev/null
systemctl reset-failed base-bringup.service 2>/dev/null
systemctl reset-failed lifetrac-base.service 2>/dev/null
echo "=== launching detached compose build+up ==="
cd /opt/lifetrac/DESIGN-CONTROLLER || exit 1
systemd-run --no-block --unit=base-bringup --working-directory=/opt/lifetrac/DESIGN-CONTROLLER /usr/bin/docker compose up -d --build --remove-orphans
echo "=== launched ==="
sleep 5
systemctl is-active base-bringup.service
echo "=== docker.service status (must stay active) ==="
systemctl is-active docker.service
