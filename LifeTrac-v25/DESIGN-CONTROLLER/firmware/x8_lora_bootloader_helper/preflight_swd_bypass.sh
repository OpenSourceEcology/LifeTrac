#!/bin/sh
# preflight_swd_bypass.sh — probe what we have on this X8 for the bypass path.
set -u
PASS="${SUDO_PASS:-fio}"
sudo_() { echo "$PASS" | sudo -S -p '' "$@"; }

echo "=== gpiochip nodes ==="
for c in /sys/class/gpio/gpiochip*; do
  [ -d "$c" ] || continue
  base=$(cat "$c/base" 2>/dev/null)
  ngpio=$(cat "$c/ngpio" 2>/dev/null)
  label=$(cat "$c/label" 2>/dev/null)
  echo "$c base=$base ngpio=$ngpio label=$label"
done

echo "=== /dev/gpiochip* ==="
ls -l /dev/gpiochip* 2>&1

echo "=== libgpiod tools? ==="
for t in gpioinfo gpioset gpioget gpiomon gpiodetect; do
  p=$(command -v "$t" 2>/dev/null || true)
  echo "$t: ${p:-missing}"
done

echo "=== pwmchip0 ==="
ls /sys/class/pwm/pwmchip0/ 2>&1
echo "npwm: $(cat /sys/class/pwm/pwmchip0/npwm 2>&1)"
echo "exported: $(ls -d /sys/class/pwm/pwmchip0/pwm* 2>/dev/null | tr '\n' ' ')"

echo "=== ttymxc3 ==="
ls -l /dev/ttymxc3 2>&1

echo "=== x8h7 lsmod ==="
lsmod | grep -i x8h7 || echo "(none in lsmod)"

echo "=== consumers (who's using gpiochip5 lines) ==="
sudo_ cat /sys/kernel/debug/gpio 2>&1 | sed -n '1,80p'
