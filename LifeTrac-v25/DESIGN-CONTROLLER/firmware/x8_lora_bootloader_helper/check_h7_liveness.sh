#!/bin/sh
# Check whether H7 is alive via x8h7 bridge (SPI), independent of SWD.
echo "=== /sys/class/x8h7/ ==="
ls -la /sys/class/x8h7/ 2>&1
echo "=== version files ==="
for f in /sys/class/x8h7/*/version /sys/class/x8h7/*/info; do
  if [ -e "$f" ]; then
    echo "$f: $(cat $f 2>&1)"
  fi
done
echo "=== ttymxc3 read attempt (2s) ==="
timeout 2 cat /dev/ttymxc3 < /dev/null 2>&1 | head -5
echo "=== dmesg x8h7/h7-program ==="
dmesg | grep -iE 'x8h7|h7-program|stm32h7' | tail -25
