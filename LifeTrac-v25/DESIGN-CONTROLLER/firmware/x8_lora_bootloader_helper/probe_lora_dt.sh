#!/bin/bash
# Find LoRa-related gpio references in the live device tree.
echo "=== /proc/device-tree dirs containing 'lora' or 'sx1' or 'stm32' ==="
find /proc/device-tree -type d 2>/dev/null | grep -i -E 'lora|sx1|stm32|enuc|murata' | head -40
echo
echo "=== UART4 / ttymxc3 (LoRa UART) referenced gpios ==="
# /dev/ttymxc3 = UART4 on iMX8MM. Find the uart4 node.
find /proc/device-tree -name 'serial@30a60000*' 2>/dev/null
ls /proc/device-tree/soc@0/bus@30800000/serial@30a60000/ 2>/dev/null
echo
echo "=== Look for gpio-hog / gpio-line-names anywhere with lora/boot0 ==="
grep -arli -E 'lora|boot0' /proc/device-tree 2>/dev/null | head -20
echo
echo "=== dump x8h7_gpio gpio-line-names ==="
NODE=$(find /proc/device-tree -name 'x8h7_gpio*' 2>/dev/null | head -1)
echo "x8h7_gpio node: $NODE"
ls -la "$NODE" 2>/dev/null
if [ -f "$NODE/gpio-line-names" ]; then
    echo "--- gpio-line-names (raw) ---"
    cat "$NODE/gpio-line-names" | tr '\0' '\n' | nl
fi
echo
echo "=== look for any 'lora' string in /proc/device-tree binary nodes ==="
find /proc/device-tree -type f 2>/dev/null | while read f; do
    if grep -ali 'lora' "$f" >/dev/null 2>&1; then
        echo "HIT: $f"
        cat "$f" | tr '\0' '\n' | head -5
        echo "---"
    fi
done | head -60
echo
echo "=== look at all gpiochip5 nodes with hog assignments ==="
for n in /proc/device-tree/*x8h7*/lora* /proc/device-tree/*lora* /proc/device-tree/*/*lora* /proc/device-tree/*/*/*lora* ; do
    [ -e "$n" ] && echo "FOUND $n"
done
