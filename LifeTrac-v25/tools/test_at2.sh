#!/bin/bash
TOOLDIR=/home/fio/lifetrac_p0c
echo "=== Killing Lingering OpenOCD ==="
pkill -9 openocd 2>/dev/null
pkill -9 -f "cat /dev/ttymxc3" 2>/dev/null
sleep 1

# Note cstopb WITHOUT a minus sign this time! 2 STOP BITS!
stty -F /dev/ttymxc3 19200 cs8 -parenb cstopb raw -echo -ixon -ixoff -icanon -isig

nohup openocd -f /usr/arduino/extra/openocd_script-imx_gpio.cfg \
              -f $TOOLDIR/09_boot_user_app_hold.cfg > $TOOLDIR/openocd_at.log 2>&1 < /dev/null &
OCD_PID=$!

echo "Waiting for module to boot (2s)..."
sleep 2

cat /dev/ttymxc3 > /dev/null &
CAT_PID=$!
sleep 0.5
kill -9 $CAT_PID

for i in {1..3}; do
    echo "=== Sending AT..."
    
    cat /dev/ttymxc3 > $TOOLDIR/at_rx.bin &
    RX_PID=$!
    
    echo -n -e "AT\r" > /dev/ttymxc3
    
    sleep 0.5
    kill -9 $RX_PID 2>/dev/null
    
    echo "Response:"
    xxd $TOOLDIR/at_rx.bin
    cat $TOOLDIR/at_rx.bin | tr -d '\r'
    echo ""
done

kill -9 $OCD_PID 2>/dev/null
