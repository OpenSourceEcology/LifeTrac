#!/bin/bash
# flash_and_listen_heartbeat.sh — assumes firmware.bin pushed to /tmp.
# 1) PA11 HIGH, NRST pulse → ROM
# 2) flash_l072_via_uart.py erase + write + verify
# 3) PA11 LOW, NRST pulse → user firmware
# 4) listen 8s at 921600 8N1 for LT_BOOT_HEARTBEAT lines
set -u
DEV=/dev/ttymxc3
BIN=/tmp/firmware.bin
FLASHER=/tmp/flash_l072_via_uart.py
LOG=/tmp/lifetrac_p0c/heartbeat.log
mkdir -p /tmp/lifetrac_p0c
: > "$LOG"

GPIO=163
PWMCHIP=/sys/class/pwm/pwmchip0
PWM=$PWMCHIP/pwm4
PERIOD=1000000

ensure_pwm() {
    [ -d $PWM ] || echo 4 > $PWMCHIP/export
    echo 0 > $PWM/enable 2>/dev/null || true
    echo $PERIOD > $PWM/period
}
boot0_high() { ensure_pwm; echo $PERIOD > $PWM/duty_cycle; echo 1 > $PWM/enable; }
boot0_low()  { ensure_pwm; echo 0       > $PWM/duty_cycle; echo 1 > $PWM/enable; }

ensure_nrst() {
    [ -d /sys/class/gpio/gpio$GPIO ] || echo "$GPIO" > /sys/class/gpio/export 2>/dev/null
    echo "out" > /sys/class/gpio/gpio$GPIO/direction
    echo "1"   > /sys/class/gpio/gpio$GPIO/value
}
pulse_nrst() {
    ensure_nrst
    echo "0" > /sys/class/gpio/gpio$GPIO/value
    sleep 0.05
    echo "1" > /sys/class/gpio/gpio$GPIO/value
}

echo "=== Step 1: BOOT0=HIGH, pulse NRST → ROM ===" | tee -a "$LOG"
boot0_high
pulse_nrst
sleep 0.3

echo "=== Step 2: erase + write + verify ===" | tee -a "$LOG"
python3 $FLASHER write $BIN 2>&1 | tee -a "$LOG"
RC=${PIPESTATUS[0]}
if [ "$RC" != "0" ]; then
    echo "FLASH FAILED rc=$RC" | tee -a "$LOG"
    exit $RC
fi

echo "=== Step 3: BOOT0=LOW, configure UART, then NRST ===" | tee -a "$LOG"
boot0_low
sleep 0.1

stty -F $DEV 921600 cs8 -parenb -cstopb raw -echo -ixon -ixoff 2>&1 | tee -a "$LOG"

OUT=/tmp/lifetrac_p0c/heartbeat_921600.bin
rm -f $OUT
cat $DEV > $OUT &
PID=$!
sleep 0.05

pulse_nrst
echo "[t=0] NRST released" | tee -a "$LOG"

sleep 8
kill -9 $PID 2>/dev/null; wait $PID 2>/dev/null

SZ=$(stat -c %s $OUT)
echo "" | tee -a "$LOG"
echo "rx bytes = $SZ" | tee -a "$LOG"
xxd $OUT | head -40 | tee -a "$LOG"
echo "" | tee -a "$LOG"
echo "ASCII strings:" | tee -a "$LOG"
strings $OUT | head -20 | tee -a "$LOG"
