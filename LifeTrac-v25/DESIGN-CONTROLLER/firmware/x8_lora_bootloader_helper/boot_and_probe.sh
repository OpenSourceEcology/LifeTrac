#!/bin/bash
# boot_and_probe.sh — release BOOT0, reset L072, probe AT response.
#
# Run AFTER successful flash. Sources 08_boot_user_app.cfg via openocd to
# drive PA_11=LOW + pulse PF_4, then issues AT+VER? at 19200 8N1 and prints
# the response. Expect a real version string instead of '+ERR_RX'.

set -e
LOG=/tmp/lifetrac_p0c/boot_probe.log
DEV=/dev/ttymxc3
HELPER_DIR=/tmp/lifetrac_p0c
mkdir -p $HELPER_DIR
: > $LOG

echo "=== killing any lingering openocd ===" | tee -a $LOG
fuser -k /dev/ttymxc3 2>/dev/null || true
pkill -9 openocd 2>/dev/null || true
sleep 1

echo "" | tee -a $LOG
echo "=== running 08_boot_user_app.cfg (release BOOT0 + pulse NRST) ===" | tee -a $LOG
openocd \
  -f /usr/arduino/extra/openocd_script-imx_gpio.cfg \
  -f $HELPER_DIR/08_boot_user_app.cfg 2>&1 | tee -a $LOG

# Give the L072 user firmware time to boot and start its UART
sleep 1.5

echo "" | tee -a $LOG
echo "=== UART configure 19200 8N1 raw ===" | tee -a $LOG
stty -F $DEV 19200 cs8 -parenb -cstopb raw -echo -ixon -ixoff time 5 min 0 2>&1 | tee -a $LOG

echo "" | tee -a $LOG
echo "=== AT probe (8N1) ===" | tee -a $LOG
rm -f /tmp/at_resp.bin
cat $DEV > /tmp/at_resp.bin &
CATPID=$!
sleep 0.3
printf 'AT\r\n' > $DEV
sleep 0.6
printf 'AT+VER?\r\n' > $DEV
sleep 1.2
kill -9 $CATPID 2>/dev/null
wait $CATPID 2>/dev/null || true
SZ=$(stat -c %s /tmp/at_resp.bin)
echo "AT response size = $SZ bytes" | tee -a $LOG
xxd /tmp/at_resp.bin | head -20 | tee -a $LOG

echo "" | tee -a $LOG
echo "=== done ===" | tee -a $LOG
