#!/bin/bash
# baud_sweep_at.sh — boot user FW, then send AT at many bauds, hunting the
# rate at which the L072 LPUART1 is actually transmitting.
set -u
DEV=/dev/ttymxc3
LOG=/tmp/lifetrac_p0c/baud_sweep_at.log
mkdir -p /tmp/lifetrac_p0c
: > "$LOG"

GPIO=163
PWMCHIP=/sys/class/pwm/pwmchip0
PWM=$PWMCHIP/pwm4

[ -d $PWM ] || echo 4 > $PWMCHIP/export
echo 0 > $PWM/enable 2>/dev/null || true
echo 1000000 > $PWM/period
echo 0       > $PWM/duty_cycle
echo 1       > $PWM/enable
[ -d /sys/class/gpio/gpio$GPIO ] || echo "$GPIO" > /sys/class/gpio/export 2>/dev/null
echo "out" > /sys/class/gpio/gpio$GPIO/direction
echo "1"   > /sys/class/gpio/gpio$GPIO/value
echo "0" > /sys/class/gpio/gpio$GPIO/value
sleep 0.05
echo "1" > /sys/class/gpio/gpio$GPIO/value
sleep 2.5

# Bauds to try. ttymxc/imx UART4 supports common rates including non-std.
for BAUD in 1843200 1500000 1000000 921600 576000 500000 460800 230400 115200 57600; do
    if ! stty -F $DEV $BAUD cs8 -parenb -cstopb raw -echo -ixon -ixoff 2>/dev/null; then
        echo "skip $BAUD (stty rejected)" | tee -a "$LOG"
        continue
    fi
    actual=$(stty -F $DEV speed)
    OUT=/tmp/lifetrac_p0c/sweep_${BAUD}.bin
    rm -f $OUT
    timeout 0.2 cat $DEV > /dev/null 2>&1 || true
    cat $DEV > $OUT &
    PID=$!
    sleep 0.05
    printf 'AT\r\nAT+VER?\r\nATI\r\n' > $DEV
    sleep 0.6
    kill -9 $PID 2>/dev/null; wait $PID 2>/dev/null
    SZ=$(stat -c %s $OUT)
    echo "" | tee -a "$LOG"
    echo "=== requested=$BAUD actual_speed=$actual rx=$SZ ===" | tee -a "$LOG"
    if [ "$SZ" -gt 0 ]; then
        xxd $OUT | head -6 | tee -a "$LOG"
        STR=$(strings $OUT | head -3)
        if [ -n "$STR" ]; then echo "ASCII: $STR" | tee -a "$LOG"; fi
    fi
done
