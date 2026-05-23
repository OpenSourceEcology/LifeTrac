#!/bin/sh
cd /tmp/lifetrac_p0c && echo fio | sudo -S -p '' python3 -u method_h_stage2_tx_probe_v2.py --dev /dev/ttymxc3 --baud 921600 --probe rx_listen --rx-window 179 --progress-file /tmp/lifetrac_p0c/progress_rx.txt
rc=$?
printf '__METHOD_H_RC__=%s\n' "$rc"
