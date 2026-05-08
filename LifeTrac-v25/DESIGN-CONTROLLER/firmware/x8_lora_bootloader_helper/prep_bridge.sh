#!/bin/bash
# prep_bridge.sh — prepare X8 for an openocd session by cleanly unloading the
# x8h7 SPI bridge kernel modules WHILE THE BRIDGE IS STILL HEALTHY.
#
# Run as root: echo fio | sudo -S -p '' bash /tmp/lifetrac_p0c/prep_bridge.sh
#
# Why this matters:
#   The x8h7_gpio kernel driver does not handle mid-flight bridge disconnects.
#   When openocd halts H7, in-flight SPI requests time out and the driver leaks
#   refcount. Once that happens, rmmod cannot remove the module and the
#   ONLY recovery is a hardware power-cycle (LmP kernel is built WITHOUT
#   CONFIG_MODULE_FORCE_UNLOAD, confirmed empirically).
#
#   The workaround is: unload the x8h7 modules BEFORE openocd halts the H7,
#   so there's no driver to get stuck. Run revive_bridge.sh after openocd
#   exits to reload them.
#
# What this does:
#   1. Unexport every gpio in the x8h7 range (160-193) -- holds module refs.
#   2. rmmod x8h7_* in reverse-dependency order (by NAME, not path).
#   3. Verify all x8h7_* modules are gone.

set +e
if [ "$EUID" -ne 0 ]; then
  echo "ERROR: prep_bridge.sh must be run as root."
  exit 1
fi

echo "=== [1/4] unexport x8h7 gpios (160..193) ==="
for g in $(seq 160 193); do
  if [ -d /sys/class/gpio/gpio$g ]; then
    echo $g > /sys/class/gpio/unexport 2>/dev/null && echo "  unexport gpio$g OK"
  fi
done
sleep 0.3

# In-kernel consumers of x8h7 gpios (e.g. cs42l52_regulator @ gpio-160)
# hold module refcounts that sysfs unexport cannot release. We must unbind
# the consuming drivers from their devices first. Discovered empirically:
# bench unit had only cs42l52_regulator; if your device-tree binds others
# (sound, etc.) to x8h7 gpios, add them here.
echo "=== [2/4] unbind in-kernel x8h7 gpio consumers ==="
if [ -e /sys/bus/platform/drivers/reg-fixed-voltage/cs42l52_regulator ]; then
  echo cs42l52_regulator > /sys/bus/platform/drivers/reg-fixed-voltage/unbind 2>&1 \
    && echo "  unbind cs42l52_regulator OK" \
    || echo "  unbind cs42l52_regulator FAILED"
fi
sleep 0.3

echo "=== [3/4] rmmod x8h7 modules ==="
for m in x8h7_ui x8h7_uart x8h7_pwm x8h7_rtc x8h7_adc x8h7_can x8h7_gpio x8h7_h7 x8h7_drv; do
  if lsmod | grep -q "^$m "; then
    rmmod $m 2>&1 | sed "s/^/  rmmod $m: /"
  fi
done

echo "=== [4/4] verify ==="
REMAINING=$(lsmod | grep -c '^x8h7')
if [ "$REMAINING" -eq 0 ]; then
  echo "  all x8h7 modules unloaded"
  echo "=== SUCCESS: bridge is now safe to halt with openocd ==="
  exit 0
else
  echo "  WARNING: $REMAINING x8h7 modules still loaded:"
  lsmod | grep '^x8h7' | sed 's/^/    /'
  echo "=== PARTIAL: openocd halt may still wedge the bridge. Power-cycle if it does. ==="
  exit 2
fi
