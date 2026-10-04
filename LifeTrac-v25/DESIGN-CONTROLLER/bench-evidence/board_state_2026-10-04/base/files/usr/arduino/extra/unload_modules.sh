#!/bin/sh
#
# This script is stored within /usr/arduino/extra .
#
# The purpose of this script is to unload the kernel
# modules allowing to access the extended IO interfaces
# provided by the STM32H7.

if [ "$(id -u)" != "0" ]; then
  echo "This script must be run as root."
  exit 1
fi

MPATH="/lib/modules/$(uname -r)/extra"
rmmod $MPATH/x8h7_can.ko
rmmod $MPATH/x8h7_gpio.ko
rmmod $MPATH/x8h7_adc.ko
rmmod $MPATH/x8h7_rtc.ko
rmmod $MPATH/x8h7_pwm.ko
rmmod $MPATH/x8h7_uart.ko
rmmod $MPATH/x8h7_ui.ko
rmmod $MPATH/x8h7_h7.ko
rmmod $MPATH/x8h7_drv.ko
rmmod industrialio
