#!/bin/bash
DEV=/dev/ttymxc3
GPIO=163
SERIAL_DEV="30a60000.serial"

echo "1. Asserting BOOT0=HIGH..."
echo 1000000 > /sys/class/pwm/pwmchip0/pwm4/duty_cycle
echo 1 > /sys/class/pwm/pwmchip0/pwm4/enable

echo "2. Unbinding UART (Deafening the line)..."
echo $SERIAL_DEV > /sys/bus/platform/drivers/imx-uart/unbind

echo "3. Resetting L072 via NRST=0..."
echo 0 > /sys/class/gpio/gpio$GPIO/value
sleep 0.5

echo "4. Releasing NRST=1..."
echo 1 > /sys/class/gpio/gpio$GPIO/value
sleep 0.2

echo "5. Rebinding UART..."
echo $SERIAL_DEV > /sys/bus/platform/drivers/imx-uart/bind
sleep 0.1

echo "6. Configuring UART parameters..."
stty -F $DEV 19200 cs8 parenb -parodd -cstopb raw -echo -ixon -ixoff -inpck -istrip 2>/dev/null
sleep 0.1

echo "7. Sending autobaud 0x7F..."
printf '\x7f' > $DEV 
sleep 0.2

echo "8. Probing with GET command (0x00 0xFF)..."
rm -f /tmp/ack.bin
cat $DEV > /tmp/ack.bin &
PID=$!
sleep 0.1

printf '\x00\xff' > $DEV
sleep 0.3

kill -9 $PID 2>/dev/null
echo "Result:"
xxd /tmp/ack.bin
