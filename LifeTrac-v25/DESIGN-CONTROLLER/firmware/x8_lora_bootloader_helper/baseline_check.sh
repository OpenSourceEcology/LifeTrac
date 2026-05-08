#!/bin/bash
# Quick baseline + bridge health check.
echo "=== L072 AT baseline (8N1) ==="
DEV=/dev/ttymxc3
stty -F $DEV 19200 cs8 -parenb -cstopb raw -echo time 5 min 0 2>/dev/null
rm -f /tmp/at_resp.bin
cat $DEV > /tmp/at_resp.bin &
CATPID=$!
sleep 0.3
printf "AT+VER?\r\n" > $DEV
sleep 1.0
kill -9 $CATPID 2>/dev/null
wait $CATPID 2>/dev/null
SZ=$(stat -c %s /tmp/at_resp.bin)
echo "AT_VER size=$SZ"
xxd /tmp/at_resp.bin | head -5

echo
echo "=== L072 ROM probe (8E1) ==="
stty -F $DEV 19200 cs8 parenb -parodd -cstopb raw -echo time 5 min 0 2>/dev/null
rm -f /tmp/rom_resp.bin
cat $DEV > /tmp/rom_resp.bin &
CATPID=$!
sleep 0.3
printf '\x7f' > $DEV
sleep 0.7
kill -9 $CATPID 2>/dev/null
wait $CATPID 2>/dev/null
SZ=$(stat -c %s /tmp/rom_resp.bin)
echo "ROM_PROBE size=$SZ"
xxd /tmp/rom_resp.bin | head -5

echo
echo "=== bridge health ==="
cat /sys/kernel/x8h7_firmware/version 2>&1
uptime
echo "=== last dmesg ==="
dmesg | tail -10
