#!/bin/sh
# fresh_boot_ver.sh — SWD NRST the L072, then IMMEDIATELY raw VER_REQ at
# 921600. Discriminates "firmware wedges over time / on trigger" vs
# "URC TX dead from boot".
DEV=/dev/ttymxc3
HELPER=/tmp/lifetrac_p0c

fuser -k $DEV 2>/dev/null || true
pkill -9 openocd 2>/dev/null || true
sleep 0.5
for g in 8 10 15; do
  [ -d /sys/class/gpio/gpio$g ] || echo $g > /sys/class/gpio/export 2>/dev/null
done
echo out > /sys/class/gpio/gpio10/direction 2>/dev/null
echo 1 > /sys/class/gpio/gpio10/value 2>/dev/null
sleep 1
for g in 8 10 15; do echo $g > /sys/class/gpio/unexport 2>/dev/null; done

stty -F $DEV 921600 cs8 -parenb -cstopb raw -echo -ixon -ixoff clocal
rm -f /tmp/fresh_ver.bin
(timeout 8 cat $DEV > /tmp/fresh_ver.bin 2>/dev/null) &

openocd -f /usr/arduino/extra/openocd_script-imx_gpio.cfg \
        -f $HELPER/08_boot_user_app.cfg > /tmp/ocd_fresh.log 2>&1
echo "ocd_rc=$?"

# VER_REQ immediately (boot takes ~100ms after NRST release), then again 2s later
sleep 0.6
printf '\000\003\001\001\002\001\001\001\003\273\172\000' > $DEV
sleep 2
printf '\000\003\001\001\002\001\001\001\003\273\172\000' > $DEV
sleep 2

echo "captured=$(stat -c %s /tmp/fresh_ver.bin 2>/dev/null || echo 0)"
od -An -tx1 /tmp/fresh_ver.bin | head -14
strings /tmp/fresh_ver.bin | head -6
