#!/bin/sh
# rx_listen.sh — passive listener at 19200 8N1 on /dev/ttymxc3
# Captures whatever the modem emits for $1 seconds (default 8) into /tmp/rx_capture.txt
set -u
TTY=/dev/ttymxc3
GPIO=163
DUR=${1:-8}
TMP=/tmp/rx_capture.txt

[ -d /sys/class/gpio/gpio${GPIO} ] || echo ${GPIO} > /sys/class/gpio/export 2>/dev/null
echo out > /sys/class/gpio/gpio${GPIO}/direction 2>/dev/null
echo 0 > /sys/class/gpio/gpio${GPIO}/value 2>/dev/null
sleep 0.2
echo 1 > /sys/class/gpio/gpio${GPIO}/value 2>/dev/null
sleep 1

stty -F ${TTY} 19200 cs8 -parenb -parodd -cstopb \
    -ixon -ixoff -crtscts -icrnl -ocrnl -opost -isig -icanon -echo -echoe \
    min 0 time 5 2>/dev/null

: > ${TMP}
echo "LISTENING on ${TTY} for ${DUR}s..."
timeout ${DUR} cat ${TTY} >> ${TMP}
echo "BYTES_CAPTURED=$(wc -c < ${TMP})"
echo "---- printable ----"
tr -d '\000' < ${TMP} | strings -n 2
echo "---- hex (last 256 B) ----"
tail -c 256 ${TMP} | od -An -tx1 -w16
echo "DONE_LISTEN"
