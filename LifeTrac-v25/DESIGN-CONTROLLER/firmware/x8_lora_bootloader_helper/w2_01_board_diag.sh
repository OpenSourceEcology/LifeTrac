#!/bin/bash
# W2-01 read-only board diagnostic.
#
# Run this on a Portenta X8 board with the Kurokesu C2 UNPLUGGED. Captures
# everything we need to commit to the Option A / B / C strategy in the
# avoidance doc without writing anything to the bus, the boot environment,
# or any USB device. Output is framed by __W2_01_DIAG_BEGIN__ / __W2_01_DIAG_END__.
#
# Decisions this report drives:
#   - fw_env backend = eMMC vs MTD  -> safe to use fw_setenv? (avoidance doc 6a.1)
#   - ci_hdrc_imx modular vs builtin -> Option B (rmmod recovery) feasible?
#   - /boot layout -> uEnv.txt overlay path vs OSTREE_KERNEL_ARGS only
#   - dr_mode of each USB controller -> hypothesis 6a.4 (VBUS drop)
#   - current usbcore.autosuspend default -> Option A already applied?
#   - per-device power state baseline -> compare before/after Option A
#
# Usage from host:
#   adb -s <SERIAL> push w2_01_board_diag.sh /tmp/
#   adb -s <SERIAL> exec-out bash /tmp/w2_01_board_diag.sh > w2_01_diag_<serial>.log

set -u

echo "__W2_01_DIAG_BEGIN__"
echo "schema=w2_01_diag/1"
echo "utc_start=$(date -u +%Y-%m-%dT%H:%M:%SZ)"
echo "hostname=$(hostname)"
echo "uname=$(uname -a)"

echo "--- LmP version ---"
cat /etc/os-release 2>/dev/null | grep -E '^(NAME|VERSION|PRETTY_NAME|VARIANT)=' || true

echo "--- /proc/cmdline ---"
cat /proc/cmdline 2>/dev/null || true

echo "--- /etc/fw_env.config (eMMC vs MTD) ---"
cat /etc/fw_env.config 2>/dev/null || echo "no_fw_env_config"
echo "--- fw_setenv presence ---"
ls -la /usr/bin/fw_setenv /usr/sbin/fw_setenv 2>/dev/null || echo "no_fw_setenv"

echo "--- /boot layout ---"
ls -la /boot/ 2>/dev/null || true
echo "--- /boot/uEnv.txt (if any) ---"
cat /boot/uEnv.txt 2>/dev/null || echo "no_uenv_txt"
echo "--- /boot/extlinux/extlinux.conf (if any) ---"
cat /boot/extlinux/extlinux.conf 2>/dev/null || echo "no_extlinux_conf"
echo "--- mount | grep boot ---"
mount 2>/dev/null | grep -E 'boot|ostree' || true

echo "--- ci_hdrc / chipidea modular vs builtin ---"
echo "lsmod_chipidea:"
lsmod 2>/dev/null | grep -iE 'ci_hdrc|chipidea' || echo "  (none loaded)"
echo "modules_builtin_chipidea:"
grep -E 'ci_hdrc|chipidea' "/lib/modules/$(uname -r)/modules.builtin" 2>/dev/null || echo "  (none builtin)"
echo "kernel_config_chipidea:"
if [ -r /proc/config.gz ]; then
  zcat /proc/config.gz 2>/dev/null | grep -E '^CONFIG_USB_CHIPIDEA' || echo "  (no chipidea config lines)"
elif [ -r "/boot/config-$(uname -r)" ]; then
  grep -E '^CONFIG_USB_CHIPIDEA' "/boot/config-$(uname -r)" || echo "  (no chipidea config lines)"
else
  echo "  (kernel config not exposed)"
fi

echo "--- usbcore autosuspend default (Option A status) ---"
echo "usbcore.autosuspend=$(cat /sys/module/usbcore/parameters/autosuspend 2>/dev/null || echo '?')"

echo "--- USB controller dr_mode (hypothesis 6a.4) ---"
find /sys/firmware/devicetree -name dr_mode 2>/dev/null \
  | sort \
  | while read f; do
      printf '%s = ' "$f"
      tr -d '\0' < "$f" 2>/dev/null
      echo
    done

echo "--- USB device tree (lsusb -t) ---"
lsusb -t 2>/dev/null || echo "lsusb_t_failed"
echo "--- lsusb (flat) ---"
lsusb 2>/dev/null || echo "lsusb_failed"

echo "--- per-device USB PM snapshot (read-only) ---"
find /sys/bus/usb/devices \( \
    -path '*/power/control' -o \
    -path '*/power/autosuspend_delay_ms' -o \
    -path '*/power/runtime_status' -o \
    -path '*/power/pm_qos_no_power_off' \
  \) -print 2>/dev/null \
  | sort \
  | while read f; do
      printf '%s = ' "$f"
      cat "$f" 2>/dev/null
      echo
    done

echo "--- udev rules referencing 16d0:0ed4 (C2) or autosuspend ---"
grep -rIEn '16d0|0ed4|autosuspend|power/control' /etc/udev/rules.d/ /lib/udev/rules.d/ 2>/dev/null \
  | head -50 || echo "  (no matches)"

echo "--- last 60 dmesg lines mentioning ci_hdrc / usb / uvc ---"
dmesg 2>/dev/null | grep -iE 'ci_hdrc|chipidea|usb |uvc|snd-usb' | tail -60 \
  || echo "dmesg_unavailable"

echo "utc_end=$(date -u +%Y-%m-%dT%H:%M:%SZ)"
echo "__W2_01_DIAG_END__"
