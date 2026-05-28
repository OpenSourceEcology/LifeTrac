#!/bin/sh
# Stop tractor UART contenders, force L072 BOOT0 low, pulse L072 NRST, and
# verify the bridge GPIO path before HostLink probes.

set +e

if [ "$EUID" != "0" ]; then
  echo "ERROR: run as root"
  exit 1
fi

DEV=/dev/ttymxc3
PWMCHIP=/sys/class/pwm/pwmchip0
PWM=$PWMCHIP/pwm4
GPIO=163

echo "=== stop UART contenders ==="
systemctl stop lifetrac-camera.service 2>/dev/null || true
timeout 8 docker stop -t 1 tractor-image-tx-v2 tractor-camera-v2 tractor-camera 2>/dev/null || true
pkill -9 -f image_tx_daemon 2>/dev/null || true
pkill -9 -f camera_service 2>/dev/null || true
pkill -9 -f 'cat /dev/ttymxc3' 2>/dev/null || true
sleep 0.5
echo "fuser $DEV: $(fuser $DEV 2>&1 || true)"

echo "=== force BOOT0 low on PWM4 ==="
[ -d "$PWM" ] || echo 4 > "$PWMCHIP/export" 2>/dev/null || true
echo 0 > "$PWM/enable" 2>/dev/null || true
echo 1000000 > "$PWM/period" 2>&1
echo 0 > "$PWM/duty_cycle" 2>&1
echo 1 > "$PWM/enable" 2>&1
for f in enable period duty_cycle polarity; do
  [ -e "$PWM/$f" ] && printf '%s=' "$f" && cat "$PWM/$f"
done

echo "=== verify/pulse L072 NRST gpio$GPIO ==="
[ -d /sys/class/gpio/gpio$GPIO ] || echo "$GPIO" > /sys/class/gpio/export 2>/dev/null || true
echo out > /sys/class/gpio/gpio$GPIO/direction 2>&1
echo 1 > /sys/class/gpio/gpio$GPIO/value 2>&1
V=$(timeout 3 cat /sys/class/gpio/gpio$GPIO/value 2>&1)
RC=$?
echo "gpio$GPIO pre-pulse read rc=$RC value='$V'"
if [ "$RC" -ne 0 ]; then
  echo "FAIL: bridge GPIO path not usable"
  exit 2
fi
echo 0 > /sys/class/gpio/gpio$GPIO/value 2>&1
sleep 0.05
echo 1 > /sys/class/gpio/gpio$GPIO/value 2>&1
sleep 0.5

echo "=== UART config ==="
stty -F "$DEV" 921600 cs8 -parenb -cstopb raw -echo -ixon -ixoff 2>&1
echo "=== READY: L072 should be running user firmware with BOOT0 low ==="