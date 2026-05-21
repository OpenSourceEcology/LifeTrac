#!/bin/sh
echo fio | sudo -S -p '' env LIFETRAC_PROBE_MODE=rx_listen LIFETRAC_RX_WINDOW=66 bash /tmp/lifetrac_p0c/run_method_h_stage2_tx.sh
rc=$?
printf '__METHOD_H_RC__=%s\n' "$rc"
