#!/bin/sh
# tx_burst.sh — send a unique marker over the modem's UART to provoke a LoRa TX
set -u
TTY=/dev/ttymxc3
GPIO=163
MARKER="${1:-LIFETRAC_X8_MARKER_$(date +%s)}"

[ -d /sys/class/gpio/gpio${GPIO} ] || echo ${GPIO} > /sys/class/gpio/export 2>/dev/null
echo out > /sys/class/gpio/gpio${GPIO}/direction 2>/dev/null
echo 0 > /sys/class/gpio/gpio${GPIO}/value 2>/dev/null
sleep 0.2
echo 1 > /sys/class/gpio/gpio${GPIO}/value 2>/dev/null
sleep 1

stty -F ${TTY} 19200 cs8 -parenb -parodd -cstopb \
    -ixon -ixoff -crtscts -icrnl -ocrnl -opost -isig -icanon -echo -echoe \
    min 0 time 5 2>/dev/null

echo "TX marker: ${MARKER}"
for i in 1 2 3 4 5; do
    printf '%s\r\n' "${MARKER}_${i}" > ${TTY}
    sleep 0.5
done
echo "DONE_TX"
