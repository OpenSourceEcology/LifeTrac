#!/bin/sh
# Focused diagnostic for Board 2 L072 NRST control GPIO (gpio163 / H7 PF4).
set -u

GPIO=163
DIR=/sys/class/gpio/gpio$GPIO

say() { printf '%s\n' "$*"; }

say "=== gpio163 / L072 NRST probe ==="
date || true
uptime || true

say "--- lingering serial/probe processes ---"
ps | grep -E 'sanity|aggressive|ttymxc3|_atry|_sanity' | grep -v grep || true

say "--- export/path ---"
[ -d "$DIR" ] || echo "$GPIO" > /sys/class/gpio/export
sleep 0.1
ls -la "$DIR" 2>&1 || true

say "--- read attributes with 2s timeout ---"
for file in direction value active_low edge; do
    printf '%s=' "$file"
    timeout 2 cat "$DIR/$file" 2>&1
    printf 'rc=%s\n' "$?"
done

say "--- write attributes with 2s timeout ---"
timeout 2 sh -c "echo out > $DIR/direction"
printf 'direction_write_rc=%s\n' "$?"
timeout 2 sh -c "echo 0 > $DIR/value"
printf 'value_write_0_rc=%s\n' "$?"
timeout 2 sh -c "echo 1 > $DIR/value"
printf 'value_write_1_rc=%s\n' "$?"

say "--- read-back after writes ---"
for file in direction value; do
    printf '%s=' "$file"
    timeout 2 cat "$DIR/$file" 2>&1
    printf 'rc=%s\n' "$?"
done

say "--- kernel gpio debug match ---"
mount -t debugfs debugfs /sys/kernel/debug 2>/dev/null || true
grep -n 'gpio-163\|PF4\|LORA\|reset\|nrst' /sys/kernel/debug/gpio 2>&1 | head -30 || true

say "=== done ==="