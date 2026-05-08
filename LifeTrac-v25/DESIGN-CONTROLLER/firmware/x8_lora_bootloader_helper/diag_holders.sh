#!/bin/bash
set +e
echo "=== currently exported gpios ==="
ls /sys/class/gpio/ | grep -E '^gpio[0-9]'
echo
echo "=== lsmod x8h7 refcounts ==="
lsmod | grep x8h7
echo
echo "=== open files referencing x8h7 ==="
lsof 2>/dev/null | grep -i x8h7 | head -20
echo
echo "=== processes touching gpio/x8h7 ==="
ps -ef | grep -iE 'gpio|x8h7' | grep -v grep | head -10
echo
echo "=== /dev/x8h7* and /dev/ttyX* ==="
ls -la /dev/x8h7* /dev/ttyX* 2>/dev/null
echo
echo "=== systemd services holding any handle ==="
systemctl list-units --state=active --type=service 2>/dev/null | grep -iE 'gpio|x8h7|carrier|lora' | head
