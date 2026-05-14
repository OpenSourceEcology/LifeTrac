#!/bin/bash
# user_fw_at_probe.sh — boot user firmware (PA11 LOW + NRST pulse), wait for
# init to complete, then send AT\r\n at multiple bauds and capture response.
set -u
DEV=/dev/ttymxc3
LOG=/tmp/lifetrac_p0c/user_fw_at_probe.log
mkdir -p /tmp/lifetrac_p0c
: > "$LOG"

GPIO=163
PWMCHIP=/sys/class/pwm/pwmchip0
PWM=$PWMCHIP/pwm4

# Drive PA11 LOW
[ -d $PWM ] || echo 4 > $PWMCHIP/export
echo 0 > $PWM/enable 2>/dev/null || true
echo 1000000 > $PWM/period
echo 0       > $PWM/duty_cycle
echo 1       > $PWM/enable
echo "PWM4 BOOT0=LOW" | tee -a "$LOG"

[ -d /sys/class/gpio/gpio$GPIO ] || echo "$GPIO" > /sys/class/gpio/export 2>/dev/null
echo "out" > /sys/class/gpio/gpio$GPIO/direction
echo "1"   > /sys/class/gpio/gpio$GPIO/value

# Pulse NRST
echo "0" > /sys/class/gpio/gpio$GPIO/value
sleep 0.05
echo "1" > /sys/class/gpio/gpio$GPIO/value

# Allow user firmware ~2.5s for safe_mode_listen + clock + radio init
sleep 2.5

for BAUD in 921600 115200 38400 19200 9600; do
    echo "" | tee -a "$LOG"
    echo "=== AT @ $BAUD 8N1 ===" | tee -a "$LOG"
    stty -F $DEV $BAUD cs8 -parenb -cstopb raw -echo -ixon -ixoff 2>&1 | tee -a "$LOG"
    OUT=/tmp/lifetrac_p0c/at_${BAUD}.bin
    rm -f $OUT
    timeout 0.3 cat $DEV > /dev/null 2>&1 || true   # drain
    cat $DEV > $OUT &
    PID=$!
    sleep 0.1
    printf 'AT\r\n' > $DEV
    sleep 0.5
    printf 'AT+VER?\r\n' > $DEV
    sleep 0.7
    kill -9 $PID 2>/dev/null; wait $PID 2>/dev/null
    SZ=$(stat -c %s $OUT)
    echo "rx=$SZ" | tee -a "$LOG"
    if [ "$SZ" -gt 0 ]; then
        xxd $OUT | head -10 | tee -a "$LOG"
        STR=$(strings $OUT | head -3)
        if [ -n "$STR" ]; then echo "ASCII: $STR" | tee -a "$LOG"; fi
    fi
done
