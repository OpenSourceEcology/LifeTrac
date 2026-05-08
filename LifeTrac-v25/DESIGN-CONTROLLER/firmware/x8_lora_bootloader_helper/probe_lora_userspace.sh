#!/bin/bash
echo "=== gpio scripts in /usr ==="
find /usr -name '*lora*' -o -name '*x8h7*' -o -name '*reset.sh' -o -name '*boot0*' 2>/dev/null | head -40
echo
echo "=== m4_led_forwarder ==="
cat /usr/bin/m4_led_forwarder 2>/dev/null
echo
echo "=== /usr/arduino tree ==="
ls -laR /usr/arduino/ 2>/dev/null | head -100
echo
echo "=== /sys/class/gpio export ==="
ls /sys/class/gpio 2>/dev/null
for d in /sys/class/gpio/gpio*; do
    [ -d "$d" ] || continue
    echo "$d label=$(cat $d/label 2>/dev/null) dir=$(cat $d/direction 2>/dev/null) val=$(cat $d/value 2>/dev/null)"
done
echo
echo "=== dmesg gpio/x8h7 ==="
dmesg 2>/dev/null | grep -i -E 'x8h7|lora|murata|gpiochip5' | head -40
echo
echo "=== H7 firmware version ==="
cat /sys/class/x8h7-h7/x8h7_h7.0.auto/x8h7_firmware/version 2>/dev/null
ls /sys/class/x8h7-h7/ 2>/dev/null
