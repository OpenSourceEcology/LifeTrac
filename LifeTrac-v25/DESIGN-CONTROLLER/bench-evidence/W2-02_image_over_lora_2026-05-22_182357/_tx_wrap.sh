#!/bin/sh
cd /tmp/lifetrac_p0c
echo fio | sudo -S -p '' python3 -u /tmp/lifetrac_p0c/w2_02_tx_fragments.py \
    --fragments /tmp/w2_02_fragments.hex \
    --inter-s 0.1
rc=$?
printf '__METHOD_H_RC__=%s\n' "$rc"