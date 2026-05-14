#!/bin/sh
# Map gpiochips to labels and find x8h7_gpio
for n in 0 1 2 3 4 5; do
  label=$(cat /sys/class/gpio/gpiochip$(grep -l "" /sys/class/gpio/gpiochip*/label 2>/dev/null | sed -n "$((n+1))p" | tr -d 'a-z/_*' )/label 2>/dev/null)
  echo "gpiochip$n -> ?"
done
echo "==="
for cdev in /dev/gpiochip*; do
  n=$(basename $cdev | tr -d 'gpiochip')
  echo "--- $cdev ---"
  gpioinfo $cdev 2>&1 | head -6
done
echo "==="
echo "device-tree x8h7 gpio info:"
ls /sys/firmware/devicetree/base/x8h7gpio/ 2>/dev/null
echo "==="
echo "device-tree pinctrl gpiox8h7grp:"
strings /sys/firmware/devicetree/base/soc@0/bus@30000000/pinctrl@30330000/imx8mm/gpiox8h7grp/* 2>/dev/null | head
