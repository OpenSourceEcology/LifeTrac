#!/bin/bash
for n in 8 10 15; do [ -d /sys/class/gpio/gpio$n ] || echo $n > /sys/class/gpio/export; done
echo out > /sys/class/gpio/gpio10/direction
echo 1   > /sys/class/gpio/gpio10/value
sleep 1
echo "PRE: gpio10=$(cat /sys/class/gpio/gpio10/value)"
cd /tmp/lifetrac_p0c
openocd -f /usr/arduino/extra/openocd_script-imx_gpio.cfg -f 07_assert_pa11_pf4_long.cfg 2>&1 | head -40
echo "---retry---"
openocd -f /usr/arduino/extra/openocd_script-imx_gpio.cfg -f 07_assert_pa11_pf4_long.cfg 2>&1 | head -40
