#!/bin/sh
# raw_ver_probe.sh — host-side raw VER_REQ probe (no container, no python).
# Writes the precomputed COBS VER_REQ frame and captures any reply.
DEV=/dev/ttymxc3
stty -F $DEV 921600 cs8 -parenb -cstopb raw -echo -ixon -ixoff clocal
timeout 0.5 cat $DEV > /dev/null 2>&1
rm -f /tmp/raw_ver.bin
(timeout 3 cat $DEV > /tmp/raw_ver.bin 2>/dev/null) &
sleep 0.3
# VER_REQ seq=1: 00 03 01 01 02 01 01 01 03 bb 7a 00
printf '\000\003\001\001\002\001\001\001\003\273\172\000' > $DEV
sleep 3
echo "reply_bytes=$(stat -c %s /tmp/raw_ver.bin 2>/dev/null || echo 0)"
od -An -tx1 /tmp/raw_ver.bin 2>/dev/null | head -6
echo "--- uart counters ---"
grep 30A60000 /proc/tty/driver/IMX-uart
