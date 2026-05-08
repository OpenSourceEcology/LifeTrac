#!/bin/bash
# boot_and_listen.sh — boot the freshly-flashed L072 user app and capture
# whatever it emits on /dev/ttymxc3 for $1 seconds (default 5).
DEV=/dev/ttymxc3
SECS=${1:-5}
LOG=/tmp/lifetrac_p0c/boot_listen.log
: > $LOG

echo "=== killing lingering openocd ===" | tee -a $LOG
fuser -k $DEV 2>/dev/null || true
pkill -9 openocd 2>/dev/null || true
sleep 1

echo "" | tee -a $LOG
echo "=== UART 19200 8N1 (user app baud) ===" | tee -a $LOG
stty -F $DEV 19200 cs8 -parenb -cstopb raw -echo -ixon -ixoff time 5 min 0 2>&1 | tee -a $LOG

echo "" | tee -a $LOG
echo "=== start listener BEFORE reset (catches boot banner) ===" | tee -a $LOG
rm -f /tmp/listen.bin
cat $DEV > /tmp/listen.bin &
PID=$!
sleep 0.2

echo "" | tee -a $LOG
echo "=== 08_boot_user_app.cfg (BOOT0 LOW + NRST pulse) ===" | tee -a $LOG
openocd -f /usr/arduino/extra/openocd_script-imx_gpio.cfg \
        -f /tmp/lifetrac_p0c/08_boot_user_app.cfg 2>&1 | tee -a $LOG

echo "" | tee -a $LOG
echo "=== listening for ${SECS} s ===" | tee -a $LOG
sleep $SECS
kill -9 $PID 2>/dev/null
wait $PID 2>/dev/null

SZ=$(stat -c %s /tmp/listen.bin)
echo "captured $SZ bytes" | tee -a $LOG
echo "" | tee -a $LOG
echo "--- as text ---" | tee -a $LOG
cat /tmp/listen.bin | tr -c '[:print:]\n\r' '.' | tee -a $LOG
echo "" | tee -a $LOG
echo "--- hex dump (head) ---" | tee -a $LOG
xxd /tmp/listen.bin | head -30 | tee -a $LOG
echo "" | tee -a $LOG
echo "=== done ===" | tee -a $LOG
