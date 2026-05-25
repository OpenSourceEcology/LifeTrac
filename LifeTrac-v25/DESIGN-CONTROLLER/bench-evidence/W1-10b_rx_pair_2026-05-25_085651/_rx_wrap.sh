#!/bin/sh
echo fio | sudo -S -p '' env LIFETRAC_REG_PROFILE=0 LIFETRAC_PROBE_MODE=rx_listen LIFETRAC_RX_WINDOW=46 bash /tmp/lifetrac_p0c/run_method_h_stage2_tx.sh
rc=$?
printf '__METHOD_H_RC__=%s\n' "$rc"
