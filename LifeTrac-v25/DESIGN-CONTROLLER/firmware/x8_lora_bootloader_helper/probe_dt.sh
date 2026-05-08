#!/bin/bash
set -u
echo "=== /boot ==="
ls -la /boot 2>/dev/null
echo
echo "=== /boot/overlays ==="
ls -la /boot/overlays 2>/dev/null
echo
echo "=== fw_printenv full (filtered) ==="
fw_printenv 2>/dev/null | grep -i -E 'overlay|lora|murata|boot0' | head -40
echo
echo "=== find dtbo/dts files referencing lora ==="
find / -iname '*.dtbo' 2>/dev/null | head -40
echo "---"
find / -iname '*lora*' 2>/dev/null | head -40
echo "---"
find /usr/lib /usr/share /opt -iname '*enuc*' 2>/dev/null | head -40
echo
echo "=== ostree deploy roots ==="
find /ostree -maxdepth 5 -name '*.dtbo' 2>/dev/null | head -40
echo
echo "=== applied overlays via configfs ==="
ls -la /sys/kernel/config/device-tree/overlays 2>/dev/null
echo
echo "=== /proc/device-tree probe for lora ==="
find /proc/device-tree -maxdepth 6 2>/dev/null | grep -i -E 'lora|murata|sx126|sx127|stm32' | head -40
echo
echo "=== gpio info ==="
cat /sys/kernel/debug/gpio 2>/dev/null | head -100
