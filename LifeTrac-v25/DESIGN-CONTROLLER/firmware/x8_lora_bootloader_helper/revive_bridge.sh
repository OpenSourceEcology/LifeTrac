#!/bin/bash
# revive_bridge.sh — recover the X8 x8h7 SPI bridge after an openocd session
# without power-cycling the board.
#
# Run as root: echo fio | sudo -S -p '' bash /tmp/lifetrac_p0c/revive_bridge.sh
#
# Why this is needed:
#   /usr/arduino/extra/openocd_script-imx_gpio.cfg ends with `init; reset halt`,
#   which leaves the H7 frozen in System Bootloader. The x8h7-firmware (SPI
#   bridge) at flash 0x08000000 stops running. Linux x8h7_* drivers then
#   time out, holding stale state that NRST alone cannot clear.
#
# Recovery sequence (5 stages):
#   [1] Kill any leftover openocd / cat-on-ttymxc3 processes.
#   [2] openocd: `init` then `reset run` then `shutdown` -- cold-boots H7
#       from flash into x8h7-firmware while still owning SWD, then cleanly
#       releases SWD on shutdown.
#   [3] Unexport every gpio in the x8h7 range (160-193). Every export holds
#       a refcount on x8h7_gpio that prevents rmmod.
#   [4] rmmod by NAME (note: /usr/arduino/extra/unload_modules.sh has a bug
#       where it uses .ko paths; rmmod expects names) in dependency order,
#       then re-insmod via the upstream load_modules_pre/post.sh scripts.
#   [5] Verify with a real bridge op (write+read gpio163 = LoRa NRST). The
#       /sys/kernel/x8h7_firmware/version probe is unreliable on this
#       OE image even when the bridge is healthy, so we don't use it.

set +e

if [ "$EUID" -ne 0 ]; then
  echo "ERROR: revive_bridge.sh must be run as root."
  exit 1
fi

X8H7_GPIO_BASE=160
X8H7_GPIO_LAST=193   # base + ngpio - 1

echo "=== [1/5] kill stragglers ==="
pkill -9 openocd 2>/dev/null
pkill -9 -f "cat /dev/ttymxc3" 2>/dev/null
sleep 1

echo "=== [2/5] openocd: reset run + shutdown ==="
openocd -f /usr/arduino/extra/openocd_script-imx_gpio.cfg \
        -c "reset run" -c "shutdown" 2>&1 | tail -8
sleep 1

echo "=== [3/5] unexport x8h7 gpios ($X8H7_GPIO_BASE..$X8H7_GPIO_LAST) ==="
for g in $(seq $X8H7_GPIO_BASE $X8H7_GPIO_LAST); do
  if [ -d /sys/class/gpio/gpio$g ]; then
    echo $g > /sys/class/gpio/unexport 2>/dev/null && echo "  unexport gpio$g OK"
  fi
done
sleep 0.3

echo "=== [4/5] rmmod by name + reload ==="
# Reverse dependency order
for m in x8h7_ui x8h7_uart x8h7_pwm x8h7_rtc x8h7_adc x8h7_gpio x8h7_can x8h7_h7 x8h7_drv; do
  if lsmod | grep -q "^$m "; then
    rmmod $m 2>&1 | sed "s/^/  rmmod $m: /"
  fi
done
# industrialio is held by x8h7_adc; once that's gone we can drop it too
if lsmod | grep -q "^industrialio "; then
  rmmod industrialio 2>&1 | sed 's/^/  rmmod industrialio: /'
fi
sleep 0.5

bash /usr/arduino/extra/load_modules_pre.sh  2>&1 | sed 's/^/  load_pre: /'
sleep 0.3
bash /usr/arduino/extra/load_modules_post.sh 2>&1 | sed 's/^/  load_post: /'
sleep 1

# Re-bind in-kernel x8h7-gpio consumers that prep_bridge.sh unbound.
# (Required to restore audio codec power; matches prep_bridge.sh unbind list.)
if [ -e /sys/bus/platform/devices/cs42l52_regulator ] \
   && [ ! -e /sys/bus/platform/drivers/reg-fixed-voltage/cs42l52_regulator ]; then
  echo cs42l52_regulator > /sys/bus/platform/drivers/reg-fixed-voltage/bind 2>&1 \
    && echo "  re-bind cs42l52_regulator OK" \
    || echo "  re-bind cs42l52_regulator FAILED (audio codec power may be off)"
fi

echo "=== [5/5] verify with real bridge op (gpio163 = LoRa NRST) ==="
[ -d /sys/class/gpio/gpio163 ] || echo 163 > /sys/class/gpio/export 2>/dev/null
sleep 0.2
echo out > /sys/class/gpio/gpio163/direction 2>/dev/null
echo 1   > /sys/class/gpio/gpio163/value     2>/dev/null
V=$(timeout 3 cat /sys/class/gpio/gpio163/value 2>&1)
RC=$?
echo "  gpio163 read: rc=$RC value='$V'"

if [ $RC -eq 0 ] && [ "$V" = "1" -o "$V" = "0" ]; then
  echo "=== SUCCESS: bridge revived without power-cycle ==="
  exit 0
else
  echo "=== FAIL: bridge still dead. Power-cycle required. ==="
  exit 2
fi
