#!/bin/bash
# boot_then_at.sh — drop BOOT0+pulse NRST, then immediately probe AT (chip is
# fresh from reset, less time to enter low-power).
DEV=/dev/ttymxc3
LOG=/tmp/lifetrac_p0c/boot_then_at.log
: > $LOG

echo "=== killing lingering openocd ===" | tee -a $LOG
fuser -k $DEV 2>/dev/null || true
pkill -9 openocd 2>/dev/null || true
sleep 1

echo "" | tee -a $LOG
echo "=== 08_boot_user_app.cfg (BOOT0 LOW + NRST pulse) ===" | tee -a $LOG
openocd -f /usr/arduino/extra/openocd_script-imx_gpio.cfg \
        -f /tmp/lifetrac_p0c/08_boot_user_app.cfg 2>&1 | tee -a $LOG

echo "" | tee -a $LOG
echo "=== UART 19200 8N1 ===" | tee -a $LOG
stty -F $DEV 19200 cs8 -parenb -cstopb raw -echo -ixon -ixoff time 5 min 0 2>&1 | tee -a $LOG

# Start cat IMMEDIATELY (don't wait), it will catch the boot banner if any
rm -f /tmp/at_now.bin
cat $DEV > /tmp/at_now.bin &
PID=$!
sleep 0.5
echo "" | tee -a $LOG
echo "=== send AT ===" | tee -a $LOG
printf 'AT\r\n' > $DEV
sleep 1.5
printf 'AT+VER?\r\n' > $DEV
sleep 1.5
printf 'AT+DEV?\r\n' > $DEV
sleep 1.5
kill -9 $PID 2>/dev/null
wait $PID 2>/dev/null
SZ=$(stat -c %s /tmp/at_now.bin)
echo "captured $SZ bytes" | tee -a $LOG
xxd /tmp/at_now.bin | head -20 | tee -a $LOG
echo "" | tee -a $LOG
echo "as text:" | tee -a $LOG
cat /tmp/at_now.bin | tr -c '[:print:]\n\r' '.' | tee -a $LOG
echo "" | tee -a $LOG
echo "=== done ===" | tee -a $LOG
