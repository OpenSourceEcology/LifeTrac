#!/bin/sh
echo fio | sudo -S -p '' bash -c '
for n in 8 10 15; do
  [ -d /sys/class/gpio/gpio$n ] || echo $n > /sys/class/gpio/export 2>/dev/null
done
echo out > /sys/class/gpio/gpio10/direction 2>/dev/null
echo 1 > /sys/class/gpio/gpio10/value 2>/dev/null
echo "AFTER preflight:"
for g in 8 10 15; do
  if [ -d /sys/class/gpio/gpio$g ]; then
    dir=$(cat /sys/class/gpio/gpio$g/direction 2>/dev/null)
    val=$(cat /sys/class/gpio/gpio$g/value 2>/dev/null)
    echo "gpio${g} EXPORTED dir=$dir val=$val"
  else
    echo "gpio${g} STILL_NOT_EXPORTED"
  fi
done
'
