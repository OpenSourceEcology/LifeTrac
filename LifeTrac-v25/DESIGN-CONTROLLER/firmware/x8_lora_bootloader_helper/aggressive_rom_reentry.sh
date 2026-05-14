#!/bin/bash
# Aggressive ROM re-entry attempt after wunprot/runprot reset.
# Tries longer NRST hold times and multiple autobaud byte patterns.
set -u
DEV=/dev/ttymxc3
GPIO=163

ensure_pwm_high() {
    echo 1000000 > /sys/class/pwm/pwmchip0/pwm4/duty_cycle
    echo 1 > /sys/class/pwm/pwmchip0/pwm4/enable
}

for HOLD in 0.5 1.0 2.0; do
    echo "============================================"
    echo "=== NRST hold=${HOLD}s, then probe ==="
    echo "============================================"
    ensure_pwm_high
    echo 0 > /sys/class/gpio/gpio$GPIO/value
    sleep $HOLD
    echo 1 > /sys/class/gpio/gpio$GPIO/value
    sleep 1.0  # wait for ROM to come up

    # Configure UART
    stty -F $DEV 19200 cs8 parenb -parodd -cstopb raw -echo -ixon -ixoff -inpck -istrip 2>/dev/null

    # Try multiple autobaud strategies
    for MODE in "GET_only" "0x7F_then_GET" "many_0x7F" ; do
        echo "--- mode: $MODE ---"
        rm -f /tmp/_atry.bin
        cat $DEV > /tmp/_atry.bin &
        PID=$!
        sleep 0.1
        case "$MODE" in
            GET_only)        printf '\x00\xff' > $DEV ;;
            0x7F_then_GET)   printf '\x7f' > $DEV; sleep 0.3; printf '\x00\xff' > $DEV ;;
            many_0x7F)       for i in 1 2 3 4 5; do printf '\x7f' > $DEV; sleep 0.05; done ;;
        esac
        sleep 0.6
        kill -9 $PID 2>/dev/null
        SZ=$(stat -c %s /tmp/_atry.bin 2>/dev/null)
        echo "rx=$SZ:"
        xxd /tmp/_atry.bin | head -2
    done
done
