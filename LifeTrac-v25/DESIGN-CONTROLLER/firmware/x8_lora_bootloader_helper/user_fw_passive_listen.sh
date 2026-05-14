#!/bin/bash
# user_fw_passive_listen.sh — boot user FW, then just listen at 921600 8N1
# for 10s without sending anything. The firmware emits boot fault frames
# (CLOCK_HSE_FAILED, RX_SEEN, etc.) early in init.
set -u
DEV=/dev/ttymxc3
LOG=/tmp/lifetrac_p0c/passive_listen.log
mkdir -p /tmp/lifetrac_p0c
: > "$LOG"

GPIO=163
PWMCHIP=/sys/class/pwm/pwmchip0
PWM=$PWMCHIP/pwm4

# PA11 LOW
[ -d $PWM ] || echo 4 > $PWMCHIP/export
echo 0 > $PWM/enable 2>/dev/null || true
echo 1000000 > $PWM/period
echo 0       > $PWM/duty_cycle
echo 1       > $PWM/enable

# NRST gpio
[ -d /sys/class/gpio/gpio$GPIO ] || echo "$GPIO" > /sys/class/gpio/export 2>/dev/null
echo "out" > /sys/class/gpio/gpio$GPIO/direction
echo "1"   > /sys/class/gpio/gpio$GPIO/value

# Configure UART BEFORE pulsing reset so we don't miss boot bytes
stty -F $DEV 921600 cs8 -parenb -cstopb raw -echo -ixon -ixoff 2>&1 | tee -a "$LOG"

OUT=/tmp/lifetrac_p0c/passive_921600.bin
rm -f $OUT
cat $DEV > $OUT &
PID=$!
sleep 0.05

# Pulse NRST
echo "0" > /sys/class/gpio/gpio$GPIO/value
sleep 0.05
echo "1" > /sys/class/gpio/gpio$GPIO/value
echo "[t=0] NRST released" | tee -a "$LOG"

# Listen 10s
sleep 10
kill -9 $PID 2>/dev/null; wait $PID 2>/dev/null

SZ=$(stat -c %s $OUT)
echo "rx bytes = $SZ" | tee -a "$LOG"
xxd $OUT | head -40 | tee -a "$LOG"
echo "" | tee -a "$LOG"
echo "ASCII strings:" | tee -a "$LOG"
strings $OUT | head -20 | tee -a "$LOG"
