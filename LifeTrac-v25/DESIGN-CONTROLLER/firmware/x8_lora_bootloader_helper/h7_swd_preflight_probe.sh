#!/bin/sh
# h7_swd_preflight_probe.sh
# Checks GPIO 8/15 (H7 SWCLK/SWDIO) and gpio10 (H7 NRST) state, applies the
# 2026-05-12 preflight (export + gpio10=1), and reports before/after values.
# Run via: adb shell sh /tmp/h7_swd_preflight_probe.sh
set -u
SUDO_PASS="${SUDO_PASS:-fio}"

show() {
  for g in 8 10 15; do
    if [ -d "/sys/class/gpio/gpio$g" ]; then
      dir=$(cat "/sys/class/gpio/gpio$g/direction" 2>/dev/null || echo ?)
      val=$(cat "/sys/class/gpio/gpio$g/value" 2>/dev/null || echo ?)
      echo "  gpio$g exported=Y dir=$dir val=$val"
    else
      echo "  gpio$g exported=N"
    fi
  done
}

echo "=== BEFORE preflight ==="
show

echo "=== applying preflight (export 8/10/15, gpio10=out 1) ==="
echo "$SUDO_PASS" | sudo -S -p '' sh <<'EOSU'
for g in 8 10 15; do
  [ -d "/sys/class/gpio/gpio$g" ] || echo $g > /sys/class/gpio/export 2>/dev/null
done
echo out > /sys/class/gpio/gpio10/direction 2>/dev/null
echo 1   > /sys/class/gpio/gpio10/value
EOSU

echo "=== AFTER preflight ==="
show

echo "=== raw IOMUXC peek for GPIO1_IO10 (i.MX8MM IOMUXC pad mux SW_MUX_CTL_PAD_GPIO1_IO10 @ 0x30330030) ==="
echo "$SUDO_PASS" | sudo -S -p '' sh -c 'devmem2 0x30330030 w 2>/dev/null || busybox devmem 0x30330030 32 2>/dev/null || echo "(no devmem tool)"'
echo "=== raw GPIO1 DR/GDIR @ 0x30200000 ==="
echo "$SUDO_PASS" | sudo -S -p '' sh -c 'busybox devmem 0x30200000 32 2>/dev/null; busybox devmem 0x30200004 32 2>/dev/null'

echo "=== OpenOCD imx_gpio attach (3-second timeout) ==="
OPENOCD="${OPENOCD:-/usr/bin/openocd}"
[ -x "$OPENOCD" ] || OPENOCD=$(which openocd 2>/dev/null)
CFG="/usr/arduino/extra/openocd_script-imx_gpio.cfg"
if [ -x "$OPENOCD" ] && [ -f "$CFG" ]; then
  echo "$SUDO_PASS" | sudo -S -p '' timeout 3 "$OPENOCD" -f "$CFG" -c "init; exit" 2>&1 | head -40
  echo "openocd_rc=$?"
else
  echo "openocd or cfg missing: openocd=$OPENOCD cfg=$CFG"
fi
