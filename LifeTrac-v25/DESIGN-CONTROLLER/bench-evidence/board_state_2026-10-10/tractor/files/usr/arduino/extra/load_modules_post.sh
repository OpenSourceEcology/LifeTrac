#!/bin/sh
#
# This script is stored within /usr/arduino/extra .
#
# The purpose of this script is to load the kernel
# modules allowing to access the extended IO interfaces
# provided by the STM32H7.

MPATH="/lib/modules/$(uname -r)/extra"
modprobe industrialio
insmod $MPATH/x8h7_can.ko
insmod $MPATH/x8h7_gpio.ko
insmod $MPATH/x8h7_adc.ko
insmod $MPATH/x8h7_rtc.ko
insmod $MPATH/x8h7_pwm.ko
insmod $MPATH/x8h7_uart.ko
insmod $MPATH/x8h7_ui.ko
