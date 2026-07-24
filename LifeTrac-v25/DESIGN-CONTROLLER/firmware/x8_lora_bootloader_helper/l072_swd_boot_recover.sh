#!/bin/sh
# l072_swd_boot_recover.sh — hardware-reset the L072 into user firmware via
# the H7 SWD path (OpenOCD imx_gpio), bypassing the wedged x8h7 kernel bridge.
# Pattern per 2026-05-22/26 verified Stage-1 flow + 08_boot_user_app.cfg.
set -u
HELPER=/tmp/lifetrac_p0c
LOG=$HELPER/swd_boot_recover.log
mkdir -p $HELPER
: > $LOG

echo "=== step 0: evict UART users + lingering openocd ===" | tee -a $LOG
fuser -k /dev/ttymxc3 2>/dev/null || true
pkill -9 openocd 2>/dev/null || true
sleep 1

echo "=== step 1: H7 SWD preflight (gpio8/10/15) ===" | tee -a $LOG
for g in 8 10 15; do
  [ -d /sys/class/gpio/gpio$g ] || echo $g > /sys/class/gpio/export 2>/dev/null
done
echo out > /sys/class/gpio/gpio10/direction 2>/dev/null
echo 1 > /sys/class/gpio/gpio10/value 2>/dev/null
sleep 1
# Unexport so openocd's /dev/mem writes are uncontested (2026-05-22 fix).
for g in 8 10 15; do
  echo $g > /sys/class/gpio/unexport 2>/dev/null
done

echo "=== step 2: openocd 08_boot_user_app.cfg (BOOT0 low + NRST pulse) ===" | tee -a $LOG
openocd \
  -f /usr/arduino/extra/openocd_script-imx_gpio.cfg \
  -f $HELPER/08_boot_user_app.cfg 2>&1 | tee -a $LOG
OCD_RC=$?
echo "openocd rc=$OCD_RC" | tee -a $LOG

echo "=== step 3: capture boot chatter at 921600 ===" | tee -a $LOG
stty -F /dev/ttymxc3 921600 cs8 -parenb -cstopb raw -echo -ixon -ixoff clocal
rm -f /tmp/swdrec_boot.bin
timeout 3 cat /dev/ttymxc3 > /tmp/swdrec_boot.bin 2>/dev/null
echo "boot_bytes=$(stat -c %s /tmp/swdrec_boot.bin 2>/dev/null || echo 0)" | tee -a $LOG
od -An -tx1 /tmp/swdrec_boot.bin 2>/dev/null | head -8 | tee -a $LOG
