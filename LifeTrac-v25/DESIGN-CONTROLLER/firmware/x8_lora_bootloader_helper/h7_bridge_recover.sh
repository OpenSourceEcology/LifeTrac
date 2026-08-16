#!/bin/sh
# h7_bridge_recover.sh — reset the H7 (bridge MCU) via i.MX gpio10, wait for
# x8h7 bridge re-init, then pulse L072 NRST (gpio163) and capture boot bytes.
DEV=/dev/ttymxc3

echo "--- step 1: H7 NRST pulse via i.MX gpio10 ---"
[ -d /sys/class/gpio/gpio10 ] || echo 10 > /sys/class/gpio/export 2>/dev/null
echo out > /sys/class/gpio/gpio10/direction
echo 0 > /sys/class/gpio/gpio10/value
sleep 0.2
echo 1 > /sys/class/gpio/gpio10/value
echo "h7_pulse_done rc=$?"

echo "--- step 2: wait for bridge re-init ---"
sleep 5

echo "--- step 3: gpio163 sanity (read with timeout) ---"
[ -d /sys/class/gpio/gpio163 ] || echo 163 > /sys/class/gpio/export 2>/dev/null
timeout 3 sh -c "echo out > /sys/class/gpio/gpio163/direction"; echo "dir_rc=$?"
timeout 3 cat /sys/class/gpio/gpio163/value; echo "read_rc=$?"

echo "--- step 4: L072 NRST pulse + boot capture at 921600 ---"
stty -F $DEV 921600 cs8 -parenb -cstopb raw -echo -ixon -ixoff clocal
timeout 0.5 cat $DEV > /dev/null 2>&1
rm -f /tmp/h7rec_boot.bin
(timeout 4 cat $DEV > /tmp/h7rec_boot.bin 2>/dev/null) &
sleep 0.2
timeout 3 sh -c "echo 0 > /sys/class/gpio/gpio163/value"; echo "w0_rc=$?"
sleep 0.05
timeout 3 sh -c "echo 1 > /sys/class/gpio/gpio163/value"; echo "w1_rc=$?"
sleep 4
echo "boot_bytes=$(stat -c %s /tmp/h7rec_boot.bin 2>/dev/null || echo 0)"
od -An -tx1 /tmp/h7rec_boot.bin 2>/dev/null | head -6
