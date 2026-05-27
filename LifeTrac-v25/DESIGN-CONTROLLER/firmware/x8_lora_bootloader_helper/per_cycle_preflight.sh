#!/bin/bash
# 2026-05-26 Blocker B3 hardening: per-cycle preflight invoked by
# run_stage1_standard_quant_end_to_end.ps1 between flash cycles.
# Evicts /dev/ttymxc3 holders (camera tx/rx daemons, stale flasher),
# kills any stale openocd from a prior cycle (which would otherwise
# hold the H7 SWD GPIOs and make the next DPIDR read 0xdeadbeef),
# re-exports gpio8/10/15 and drives NRST (gpio10) high, then captures
# observable state (PREFLIGHT_HOLDERS / PREFLIGHT_OPENOCD /
# PREFLIGHT_GPIO10) so the launcher.log shows whether the eviction
# actually worked.
#
# Run as root (the launcher invokes via `sudo bash`).
set +e

# 2026-05-26 ROOT CAUSE: lifetrac-camera.service (systemd-managed)
# runs `docker compose ... up` for a camera_service.py container that
# opens /dev/ttymxc3 to talk to the L072 (telemetry/config). Plain
# `pkill camera_tx_daemon` does NOT match the actual process name AND
# systemd respawns the docker stack anyway. Symptom: continuous tx
# activity on /dev/ttymxc3 (~1 KB/s) even after pkill, observable in
# /proc/tty/driver/IMX-uart tx counter deltas; first Python flasher
# os.write returns BlockingIOError (tx FIFO full) and L072 sees
# garbage during the BOOT0+NRST window, never enters ROM cleanly.
# FIX: stop the systemd unit (which stops the docker stack) for the
# duration of the flash. The launcher's POST-flash hook should
# restart it; if it does not, the camera will stay down until next
# boot (acceptable for a flash workflow).
systemctl stop lifetrac-camera.service 2>/dev/null
# Belt-and-suspenders: kill any lingering docker child still holding
# the port after the systemd stop (compose-up can take a moment).
pkill -9 -f camera_service 2>/dev/null
pkill -9 -f camera_tx_daemon 2>/dev/null
pkill -9 -f camera_rx_daemon 2>/dev/null
pkill -9 openocd 2>/dev/null
fuser -k /dev/ttymxc3 2>/dev/null
sleep 0.5

for n in 8 10 15; do
    if [ ! -d /sys/class/gpio/gpio$n ]; then
        echo $n > /sys/class/gpio/export 2>/dev/null
    fi
done
echo out > /sys/class/gpio/gpio10/direction 2>/dev/null
echo 1   > /sys/class/gpio/gpio10/value     2>/dev/null
sleep 0.5

HOLD=$(fuser /dev/ttymxc3 2>&1)
OCD=$(pgrep -a openocd 2>&1)
GPIO10=$(cat /sys/class/gpio/gpio10/value 2>/dev/null)

echo "PREFLIGHT_HOLDERS=$HOLD"
echo "PREFLIGHT_OPENOCD=$OCD"
echo "PREFLIGHT_GPIO10=$GPIO10"
