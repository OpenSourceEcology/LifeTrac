#!/bin/sh
echo "=== /etc/udev/rules.d ==="
ls -la /etc/udev/rules.d/ 2>&1

echo
echo "=== applied autosuspend / x8h7 / portenta / lora rule contents ==="
for f in /etc/udev/rules.d/*autosuspend* /etc/udev/rules.d/*x8h7* /etc/udev/rules.d/*portenta* /etc/udev/rules.d/*lora* ; do
  [ -f "$f" ] || continue
  echo "--- $f ---"
  cat "$f"
done

echo
echo "=== device-tree overlays ==="
ls /sys/kernel/config/device-tree/overlays/ 2>&1
ls /proc/device-tree/__symbols__ 2>/dev/null | head -5
echo "--- /proc/device-tree/__overlays__ ---"
ls /proc/device-tree/__overlays__ 2>&1

echo
echo "=== /proc/cmdline ==="
cat /proc/cmdline

echo
echo "=== camera/csi/audio dmesg hits ==="
dmesg | grep -iE 'overlay|cs42l52|imx-csi|imx_csi|ov5640|mipi|csi-mipi|gc2145|imx219' | head -30

echo
echo "=== ttymxc3 udev info ==="
udevadm info /dev/ttymxc3 2>&1 | head -30

echo
echo "=== /sys/kernel/debug/gpio (x8h7) ==="
cat /sys/kernel/debug/gpio 2>&1 | grep -B 1 -A 40 x8h7_gpio | head -60

echo
echo "=== iomuxc state for ttymxc3 / pa11 / pf4 (pinctrl) ==="
ls /sys/kernel/debug/pinctrl/ 2>&1
for d in /sys/kernel/debug/pinctrl/*/ ; do
  echo "--- $d ---"
  cat "$d/pinmux-pins" 2>/dev/null | grep -iE 'uart4|uart_4|ttymxc3|csi|cam|i2s|pdm' | head -20
done

echo
echo "=== ttymxc3 driver / device path ==="
readlink -f /sys/class/tty/ttymxc3
ls /sys/class/tty/ttymxc3/device/ 2>&1

echo
echo "=== USB autosuspend state for x8h7 ==="
for d in /sys/bus/usb/devices/*/ ; do
  v=$(cat "$d/idVendor" 2>/dev/null)
  p=$(cat "$d/idProduct" 2>/dev/null)
  if [ "$v" = "2341" ] || [ "$v" = "2a03" ]; then
    echo "--- $d  ($v:$p) ---"
    cat "$d/power/control" 2>/dev/null
    cat "$d/power/autosuspend_delay_ms" 2>/dev/null
  fi
done
