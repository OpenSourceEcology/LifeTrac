#!/bin/sh
# listen_post_powercycle.sh — passive 921600 listen on /dev/ttymxc3 with
# PA11 driven LOW (boot from main flash). Used after a Board 2 power-cycle
# to detect whether L072 is now executing user firmware (heartbeat).
#
# Usage on X8 (as root via sudo):  sh /tmp/listen_post_powercycle.sh
set -u

PWMCHIP=/sys/class/pwm/pwmchip0
PWM_DIR=$PWMCHIP/pwm4
NRST_GPIO=163
NRST_DIR=/sys/class/gpio/gpio$NRST_GPIO
DEV=/dev/ttymxc3
OUT=/tmp/_listen921k.bin

# ---- ensure PWM4 exists, period set, duty=0 (PA11 LOW) ----
[ -d "$PWM_DIR" ] || echo 4 > $PWMCHIP/export
sleep 0.2
echo 0 > $PWM_DIR/enable 2>/dev/null || true
[ "$(cat $PWM_DIR/period)" -eq 0 ] && echo 1000000 > $PWM_DIR/period
echo 0 > $PWM_DIR/duty_cycle
echo 1 > $PWM_DIR/enable

# ---- ensure NRST gpio exported, output, then pulse LOW->HIGH ----
[ -d "$NRST_DIR" ] || echo $NRST_GPIO > /sys/class/gpio/export
sleep 0.2
echo out > $NRST_DIR/direction
echo 0 > $NRST_DIR/value
sleep 0.25
echo 1 > $NRST_DIR/value
sleep 0.5

# ---- listen ----
stty -F $DEV 921600 raw -echo
rm -f $OUT
timeout 5 cat $DEV > $OUT 2>/dev/null

echo "=== 921600 PA11=LOW heartbeat listen (5s) ==="
printf "bytes received: "
wc -c < $OUT
echo "--- hex ---"
xxd $OUT | head -20
echo "--- printable ASCII ---"
tr -cd '[:print:]\n' < $OUT | head -20
echo "=== done ==="
