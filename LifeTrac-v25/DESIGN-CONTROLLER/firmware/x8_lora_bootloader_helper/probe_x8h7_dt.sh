#!/bin/bash
echo "=== nodes named x8h7* ==="
find /proc/device-tree -name 'x8h7*' 2>/dev/null
echo
echo "=== nodes with compatible containing x8h7 ==="
find /proc/device-tree -name compatible 2>/dev/null | while read f; do
    if grep -q x8h7 "$f" 2>/dev/null; then
        echo "$f"
        cat "$f" | tr '\0' '\n' | head -2
        echo "---"
    fi
done
echo
echo "=== all gpio-line-names files ==="
find /proc/device-tree -name 'gpio-line-names' 2>/dev/null | while read f; do
    echo "FILE: $f"
    cat "$f" | tr '\0' '\n' | nl
    echo "---"
done
echo
echo "=== look for nrst, boot0, irq strings in dt names ==="
find /proc/device-tree -type f 2>/dev/null | while read f; do
    if grep -ali -E 'nrst|boot0|murata|sx12|stm32l0' "$f" >/dev/null 2>&1; then
        echo "HIT: $f"
        head -c 200 "$f" | tr '\0' '\n'
        echo
        echo "---"
    fi
done | head -80
