#!/bin/sh
for g in 8 10 15; do
  if [ -d /sys/class/gpio/gpio$g ]; then
    dir=$(cat /sys/class/gpio/gpio$g/direction 2>/dev/null)
    val=$(cat /sys/class/gpio/gpio$g/value 2>/dev/null)
    echo "gpio${g} EXPORTED dir=$dir val=$val"
  else
    echo "gpio${g} NOT_EXPORTED"
  fi
done
echo "--- listing /sys/class/gpio/ ---"
ls /sys/class/gpio/ | grep -E '^gpio[0-9]+$'
