#!/bin/bash
# boot_and_listen_hold.sh — boot L072 user app via 09_boot_user_app_hold.cfg
# (openocd stays alive holding H7 halted with PA_11=LOW), and capture UART
# output on /dev/ttymxc3.
#
# Usage: bash boot_and_listen_hold.sh [seconds=8]

LISTEN_SEC=${1:-8}
TOOLDIR=/tmp/lifetrac_p0c
LOG=$TOOLDIR/boot_listen.log
OCD_LOG=$TOOLDIR/openocd_boot.log
RX_BIN=$TOOLDIR/rx.bin

echo "=== killing lingering openocd / cat ==="
pkill -9 openocd 2>/dev/null
pkill -9 -f "cat /dev/ttymxc3" 2>/dev/null
sleep 1

echo "=== UART 19200 8N1 ==="
stty -F /dev/ttymxc3 19200 cs8 -parenb -cstopb raw -echo -ixon -ixoff -icanon -isig

echo "=== start listener BEFORE boot ==="
: > $RX_BIN
( cat /dev/ttymxc3 > $RX_BIN ) &
CAT_PID=$!
sleep 0.3

echo "=== launch openocd boot+hold ==="
nohup openocd -f /usr/arduino/extra/openocd_script-imx_gpio.cfg \
              -f $TOOLDIR/09_boot_user_app_hold.cfg > $OCD_LOG 2>&1 < /dev/null &
OCD_PID=$!
echo "OCD_PID=$OCD_PID"

echo "=== listening for $LISTEN_SEC s ==="
sleep $LISTEN_SEC

echo "=== kill listener + openocd ==="
kill -9 $CAT_PID 2>/dev/null
kill -9 $OCD_PID 2>/dev/null
sleep 0.5

SZ=$(stat -c %s $RX_BIN 2>/dev/null || echo 0)
echo "captured $SZ bytes"

echo ""
echo "--- as text ---"
cat $RX_BIN
echo ""
echo "--- hex (first 256 bytes) ---"
xxd $RX_BIN | head -16

echo ""
echo "--- openocd boot log (last 20 lines) ---"
tail -20 $OCD_LOG

echo "=== done ==="
