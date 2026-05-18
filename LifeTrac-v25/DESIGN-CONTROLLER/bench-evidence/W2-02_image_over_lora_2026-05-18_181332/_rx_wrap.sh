#!/bin/sh
cd /tmp/lifetrac_p0c && echo fio | sudo -S -p '' python3 -u method_h_stage2_tx_probe.py --dev /dev/ttymxc3 --baud 921600 --probe rx_listen --rx-window 108
rc=$?
printf '__METHOD_H_RC__=%s\n' "$rc"
