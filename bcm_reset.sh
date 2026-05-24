#!/bin/sh
echo "=== regulators ==="
for r in /sys/class/regulator/regulator.*; do
  n=$(cat "$r/name" 2>/dev/null)
  s=$(cat "$r/state" 2>/dev/null)
  case "$n" in
    *sd*|*wl*|*wifi*|*SD*) echo "$r name=$n state=$s" ;;
  esac
done
echo "=== gpio42 sd1_regulator current state ==="
cat /sys/kernel/debug/gpio 2>/dev/null | grep -E 'sd1_regulator|wl|reg_on'
echo "=== try setting sd1_regulator GPIO LOW (power off) then HIGH (power on) via gpioset ==="
which gpioset
which gpiofind
gpiofind sd1_regulator 2>&1 || echo "no consumer label match"
echo "=== try toggling gpio42 directly via libgpiod ==="
gpioset gpiochip1 10=0
sleep 1
gpioset gpiochip1 10=1
sleep 1
echo "=== trigger mmc rescan ==="
for f in /sys/class/mmc_host/mmc0/device/rescan /sys/devices/platform/soc@0/30800000.bus/30b40000.mmc/rescan; do
  if [ -e "$f" ]; then
    echo "writing 1 to $f"
    echo 1 > "$f"
  fi
done
sleep 4
echo "=== sdio devices after ==="
ls /sys/bus/sdio/devices/
echo "=== iface ==="
ip link show wlan0 2>&1
echo "=== dmesg tail ==="
dmesg | tail -n 25
