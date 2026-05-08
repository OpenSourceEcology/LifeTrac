#!/bin/bash
TOOLDIR=/home/fio/lifetrac_p0c
echo "=== Killing Lingering OpenOCD ==="
pkill -9 openocd 2>/dev/null
pkill -9 -f "cat /dev/ttymxc3" 2>/dev/null
sleep 1

# Configure 19200 8N2!!! (cstopb means 2 stop bits)
stty -F /dev/ttymxc3 19200 cs8 -parenb cstopb raw -echo -ixoff -crtscts -icanon -isig min 0 time 10

nohup openocd -f /usr/arduino/extra/openocd_script-imx_gpio.cfg \
              -f $TOOLDIR/09_boot_user_app_hold.cfg > $TOOLDIR/openocd_at.log 2>&1 < /dev/null &
OCD_PID=$!

echo "Waiting for module to boot (3s)..."
sleep 3

# Open FD 3 for read/write
exec 3<> /dev/ttymxc3

# Capture output using FD 3
cat <&3 > $TOOLDIR/at_rx.bin &
RX_PID=$!

echo "=== Sending AT string 1 ==="
echo -n -e "AT\r" >&3
sleep 1

echo "=== Sending AT string 2 ==="
echo -n -e "AT+VER=?\r" >&3
sleep 1

kill -9 $RX_PID 2>/dev/null
kill -9 $OCD_PID 2>/dev/null

echo "Response hex:"
xxd $TOOLDIR/at_rx.bin
echo "Response string:"
cat $TOOLDIR/at_rx.bin | tr '\r' '\n'
echo ""
