#!/bin/sh
# Pulse H7 NRST via gpio10: drive LOW 200ms then HIGH, leave as output HIGH.
set -e
for n in 8 10 15; do
  [ -d /sys/class/gpio/gpio$n ] || echo $n > /sys/class/gpio/export 2>/dev/null
done
echo out > /sys/class/gpio/gpio10/direction
echo 0 > /sys/class/gpio/gpio10/value
sleep 0.2
echo 1 > /sys/class/gpio/gpio10/value
echo "NRST pulsed; gpio10 now:"
cat /sys/class/gpio/gpio10/value
