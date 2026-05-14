#!/bin/bash
# Re-enter ROM (NRST pulse with PA11 already HIGH) and try probing up to 5
# times. Used after a wunprot/runprot to verify OPT bytes.
set -u
GPIO=163
for i in 1 2 3 4 5; do
    echo 0 > /sys/class/gpio/gpio$GPIO/value
    sleep 0.1
    echo 1 > /sys/class/gpio/gpio$GPIO/value
    sleep 0.5
    echo "--- attempt $i ---"
    if python3 /tmp/flash_l072_via_uart.py read 0x1FF80000 32 2>&1; then
        exit 0
    fi
done
echo "ALL ATTEMPTS FAILED"
exit 1
