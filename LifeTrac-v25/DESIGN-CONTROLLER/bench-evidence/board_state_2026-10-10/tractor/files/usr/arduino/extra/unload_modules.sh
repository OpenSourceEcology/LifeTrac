#!/bin/sh
#
# This script is stored within /usr/arduino/extra .
#
# The purpose of this script is to unload the kernel
# modules allowing to access the extended IO interfaces
# provided by the STM32H7.

rmmod /usr/arduino/extra/x8h7_can.ko
rmmod /usr/arduino/extra/x8h7_gpio.ko
rmmod /usr/arduino/extra/x8h7_adc.ko
rmmod /usr/arduino/extra/x8h7_rtc.ko
rmmod /usr/arduino/extra/x8h7_pwm.ko
rmmod /usr/arduino/extra/x8h7_uart.ko
rmmod /usr/arduino/extra/x8h7_ui.ko
rmmod /usr/arduino/extra/x8h7_h7.ko
rmmod /usr/arduino/extra/x8h7_drv.ko
rmmod industrialio
