#!/bin/sh
echo "=== cs42l52 regulator ==="
for r in /sys/class/regulator/regulator.*/name; do
  n=$(cat "$r")
  case "$n" in
    *cs42*|*CS42*) echo "$r = $n" ;;
  esac
done
echo "=== devices matching cs42 ==="
find /sys/devices -iname '*cs42*' 2>/dev/null
echo "=== platform drivers cs42 ==="
ls /sys/bus/platform/drivers/ 2>/dev/null | grep -i cs42
echo "=== i2c drivers cs42 ==="
ls /sys/bus/i2c/drivers/ 2>/dev/null | grep -i cs42
echo "=== spi drivers cs42 ==="
ls /sys/bus/spi/drivers/ 2>/dev/null | grep -i cs42
echo "=== anything 'reg-fixed' with cs42 ==="
for d in /sys/bus/platform/devices/*; do
  if [ -e "$d/of_node/regulator-name" ]; then
    n=$(cat "$d/of_node/regulator-name" 2>/dev/null)
    case "$n" in
      *cs42*|*CS42*) echo "$d -> $n" ;;
    esac
  fi
done
echo "=== gpio-160 consumer detail ==="
cat /sys/kernel/debug/gpio 2>/dev/null | grep -A1 'gpio-160'
