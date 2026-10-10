#!/bin/sh
TTY=/dev/ttymxc3
GPIO=163
if [ ! -d /sys/class/gpio/gpio${GPIO} ]; then echo ${GPIO} > /sys/class/gpio/export 2>/dev/null || true; fi
echo out > /sys/class/gpio/gpio${GPIO}/direction 2>/dev/null || true
echo 0 > /sys/class/gpio/gpio${GPIO}/value 2>/dev/null || true
sleep 0.2
echo 1 > /sys/class/gpio/gpio${GPIO}/value 2>/dev/null || true
sleep 0.8
for B in 19200 115200 921600 9600; do
  echo "=== baud ${B} 8N1 ==="
  stty -F ${TTY} ${B} cs8 -parenb -parodd -cstopb -ixon -ixoff -crtscts -icrnl -ocrnl -opost -isig -icanon -echo -echoe min 0 time 5 2>/dev/null
  : > /tmp/lifetrac_compact_at_capture.txt
  timeout 1.3 cat ${TTY} >> /tmp/lifetrac_compact_at_capture.txt &
  R=$!
  sleep 0.1
  for C in AT ATI 'AT+VER?' 'AT+RADIO?' 'AT+STAT?'; do printf "%s\r\n" "$C" > ${TTY}; sleep 0.12; done
  wait ${R} 2>/dev/null
  if [ -s /tmp/lifetrac_compact_at_capture.txt ]; then tr -d '\000' < /tmp/lifetrac_compact_at_capture.txt | strings -n 2; else echo NO_DATA; fi
done