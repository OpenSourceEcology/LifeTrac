#!/bin/sh
#
# This script is stored within /usr/arduino/extra .
#
# Use this script to reset the STM32H7 from within the IMX8.
#
# [IMX8] GPIO1_IO10 = [STM32H7] NRST

echo 10 > /sys/class/gpio/export
echo out > /sys/class/gpio/gpio10/direction
echo 0 > /sys/class/gpio/gpio10/value
sleep 0.25
echo 1 > /sys/class/gpio/gpio10/value
