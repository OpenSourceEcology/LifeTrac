#!/bin/bash
# Try multiple autobaud strategies after a wunprot/runprot reset.
set -u
DEV=/dev/ttymxc3

echo "=== PA11 HIGH (PWM4 duty=period) ==="
echo 1000000 > /sys/class/pwm/pwmchip0/pwm4/duty_cycle
cat /sys/class/pwm/pwmchip0/pwm4/duty_cycle

echo "=== pulse NRST (long) ==="
echo 0 > /sys/class/gpio/gpio163/value
sleep 0.3
echo 1 > /sys/class/gpio/gpio163/value
sleep 1.0

echo "=== open at 19200 8E1, send 0x7F autobaud ==="
stty -F $DEV 19200 cs8 parenb -parodd -cstopb raw -echo -ixon -ixoff -inpck -istrip
rm -f /tmp/_ab.bin
cat $DEV > /tmp/_ab.bin &
PID=$!
sleep 0.1
printf '\x7f' > $DEV
sleep 0.6
kill -9 $PID 2>/dev/null
echo "after 0x7F:"
xxd /tmp/_ab.bin

echo
echo "=== try GET (0x00 0xFF) directly ==="
rm -f /tmp/_get.bin
cat $DEV > /tmp/_get.bin &
PID=$!
sleep 0.1
printf '\x00\xff' > $DEV
sleep 0.6
kill -9 $PID 2>/dev/null
echo "after GET:"
xxd /tmp/_get.bin

echo
echo "=== passive listen 2s at 19200 ==="
rm -f /tmp/_p1.bin
cat $DEV > /tmp/_p1.bin &
PID=$!
sleep 2
kill -9 $PID 2>/dev/null
echo "passive 19200:"
xxd /tmp/_p1.bin | head -3

echo
echo "=== passive listen 2s at 921600 ==="
stty -F $DEV 921600 cs8 -parenb -cstopb raw -echo -ixon -ixoff
rm -f /tmp/_p2.bin
cat $DEV > /tmp/_p2.bin &
PID=$!
sleep 2
kill -9 $PID 2>/dev/null
echo "passive 921600:"
xxd /tmp/_p2.bin | head -3
strings /tmp/_p2.bin | head -3
