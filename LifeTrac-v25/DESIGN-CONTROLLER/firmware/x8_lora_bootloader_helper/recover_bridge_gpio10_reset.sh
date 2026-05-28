#!/bin/bash
# recover_bridge_gpio10_reset.sh — pulse the i.MX-controlled H7 reset line,
# reload x8h7 modules, and verify the L072 NRST GPIO path.

set +e

if [ "$EUID" -ne 0 ]; then
  echo "ERROR: recover_bridge_gpio10_reset.sh must be run as root."
  exit 1
fi

echo "=== [1/4] pulse i.MX gpio10 / H7 NRST ==="
[ -d /sys/class/gpio/gpio10 ] || echo 10 > /sys/class/gpio/export 2>/dev/null || true
echo out > /sys/class/gpio/gpio10/direction 2>/dev/null
echo 0 > /sys/class/gpio/gpio10/value 2>/dev/null
sleep 0.2
echo 1 > /sys/class/gpio/gpio10/value 2>/dev/null
sleep 2

echo "=== [2/4] unload x8h7 modules ==="
for m in x8h7_ui x8h7_uart x8h7_pwm x8h7_rtc x8h7_adc x8h7_gpio x8h7_can x8h7_h7 x8h7_drv; do
  if lsmod | grep -q "^$m "; then
    rmmod "$m" 2>&1 | sed "s/^/  rmmod $m: /"
  fi
done
rmmod industrialio 2>/dev/null || true

echo "=== [3/4] reload x8h7 modules ==="
bash /usr/arduino/extra/load_modules_pre.sh 2>&1 | sed 's/^/  load_pre: /'
sleep 0.3
bash /usr/arduino/extra/load_modules_post.sh 2>&1 | sed 's/^/  load_post: /'
sleep 1

echo "=== [4/4] verify gpio163 / L072 NRST path ==="
[ -d /sys/class/gpio/gpio163 ] || echo 163 > /sys/class/gpio/export 2>/dev/null || true
echo out > /sys/class/gpio/gpio163/direction 2>/dev/null
echo 1 > /sys/class/gpio/gpio163/value 2>/dev/null
V=$(timeout 3 cat /sys/class/gpio/gpio163/value 2>&1)
RC=$?
echo "  gpio163 read: rc=$RC value='$V'"

echo "=== modules ==="
lsmod | grep '^x8h7' || true

if [ "$RC" -eq 0 ] && { [ "$V" = "0" ] || [ "$V" = "1" ]; }; then
  echo "=== SUCCESS: bridge GPIO path recovered ==="
  exit 0
fi

echo "=== FAIL: bridge GPIO path still timed out ==="
exit 2