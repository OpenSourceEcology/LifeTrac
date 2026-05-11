#!/bin/bash
set +e
pkill -9 -f 'cat /dev/ttymxc3' 2>/dev/null
pkill -9 openocd 2>/dev/null
sleep 0.3
cd /tmp/lifetrac_p0c
echo '=== reset L072 ==='
openocd -f /usr/arduino/extra/openocd_script-imx_gpio.cfg -f 08_boot_user_app.cfg 2>&1 | tail -3
stty -F /dev/ttymxc3 raw 115200 cs8 -parenb -cstopb -ixon -ixoff -crtscts
( timeout 5 cat /dev/ttymxc3 > /tmp/rx_test_capture_115k.bin ) &
CATPID=$!
sleep 1.0
echo '=== bursts at 115200 ==='
printf '\x00\x03\x01\x01\xab\xcd\x00' > /dev/ttymxc3
sleep 0.4
printf 'AT\r\n' > /dev/ttymxc3
sleep 0.4
printf '\x00\x03\x01\x01\xab\xcd\x00' > /dev/ttymxc3
wait $CATPID 2>/dev/null
echo '=== size ==='
wc -c /tmp/rx_test_capture_115k.bin
echo '=== ASCII (sorted unique traces) ==='
strings /tmp/rx_test_capture_115k.bin | sort -u | head -30
echo '=== last 256 bytes hex ==='
xxd /tmp/rx_test_capture_115k.bin | tail -16
