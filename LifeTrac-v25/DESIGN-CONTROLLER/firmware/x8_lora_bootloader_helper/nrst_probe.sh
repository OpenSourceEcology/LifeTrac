#!/bin/sh
# nrst_probe.sh — verify whether gpio163 NRST writes actually reset the L072.
# Evidence channels: write rc's, UART rx counter delta, boot chatter capture.
DEV=/dev/ttymxc3
G=/sys/class/gpio/gpio163

[ -d $G ] || echo 163 > /sys/class/gpio/export 2>/dev/null
timeout 3 sh -c "echo out > $G/direction"; echo "dir_rc=$?"

echo "--- pre counters ---"
grep 30A60000 /proc/tty/driver/IMX-uart

stty -F $DEV 921600 cs8 -parenb -cstopb raw -echo -ixon -ixoff clocal 2>&1
timeout 0.5 cat $DEV > /dev/null 2>&1

# receiver
rm -f /tmp/nrst_boot.bin
(timeout 4 cat $DEV > /tmp/nrst_boot.bin 2>/dev/null) &
sleep 0.2

timeout 3 sh -c "echo 0 > $G/value"; echo "w0_rc=$?"
sleep 0.05
timeout 3 sh -c "echo 1 > $G/value"; echo "w1_rc=$?"

sleep 4
echo "--- post counters ---"
grep 30A60000 /proc/tty/driver/IMX-uart
echo "boot_bytes=$(stat -c %s /tmp/nrst_boot.bin 2>/dev/null || echo 0)"
od -An -tx1 /tmp/nrst_boot.bin 2>/dev/null | head -6
strings /tmp/nrst_boot.bin 2>/dev/null | head -5
