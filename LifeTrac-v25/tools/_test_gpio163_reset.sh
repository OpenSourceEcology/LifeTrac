#!/bin/bash
# Pulse gpio163 (L072 NRST) then immediately run probe to verify L072
# actually cold-boots (we should see BOOT_URC observed in probe output).
set -u
[ -d /sys/class/gpio/gpio163 ] || echo 163 > /sys/class/gpio/export 2>/dev/null
echo out > /sys/class/gpio/gpio163/direction 2>/dev/null
# Idle high (NRST de-asserted)
echo 1 > /sys/class/gpio/gpio163/value
sleep 0.05
# Assert NRST low for 100ms
echo 0 > /sys/class/gpio/gpio163/value
sleep 0.1
# Release NRST
echo 1 > /sys/class/gpio/gpio163/value
echo "PULSE_DONE_AT=$(date +%s.%N)"
# Give L072 ~150ms to start emitting boot URCs
sleep 0.15
cd /tmp/lifetrac_p0c
python3 -u method_h_stage2_tx_probe_v2.py --dev /dev/ttymxc3 --baud 921600 --probe rx --rx-window 0.5
