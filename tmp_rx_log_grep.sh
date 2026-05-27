#!/bin/sh
echo fio | sudo -S docker logs lifetrac-vtest-image_rx-1 2>&1 | grep -i -E 'fault|err|scan|reject|0x0d|0x91|RX_FRAME|opmode' | head -50
