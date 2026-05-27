#!/bin/bash
# Sample i.MX UART4 (/dev/ttymxc3) driver counters twice 5s apart.
# Used to detect whether the L072 is currently emitting bytes.
grep "30A60000" /proc/tty/driver/IMX-uart
sleep 5
grep "30A60000" /proc/tty/driver/IMX-uart
