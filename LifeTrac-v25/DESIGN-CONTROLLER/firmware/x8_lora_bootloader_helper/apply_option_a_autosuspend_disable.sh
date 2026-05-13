#!/bin/bash
# W2-01 Option A apply helper: persist `usbcore.autosuspend=-1` in the boot
# cmdline using `fw_setenv`. BENCH-ONLY. The production-clean path is
# Foundries `OSTREE_KERNEL_ARGS:<machine>` in the factory layer; this helper
# is a stopgap for bench iteration only.
#
# Refuses to run unless ALL safety preconditions are met:
#   1. /etc/fw_env.config exists and points to /dev/mmcblk* (NOT /dev/mtd*).
#      The NXP MTD driver bug from kernel 5.4 onward (avoidance doc 6a.1)
#      corrupts the U-Boot environment when written via fw_setenv on raw
#      flash, so we hard-block the MTD case.
#   2. fw_setenv binary is present.
#   3. Current `bootargs` (or printenv equivalent) is captured to a rollback
#      file at /tmp/w2_01_bootargs_rollback_<utc>.txt BEFORE any change.
#   4. Caller passes --i-have-read-the-avoidance-doc to confirm awareness.
#
# Usage on target:
#   sudo bash apply_option_a_autosuspend_disable.sh --i-have-read-the-avoidance-doc
# then `sudo reboot` and verify with:
#   cat /proc/cmdline | grep usbcore.autosuspend
#   cat /sys/module/usbcore/parameters/autosuspend     # expect -1

set -euo pipefail

if [ "${1:-}" != "--i-have-read-the-avoidance-doc" ]; then
  echo "ERROR: Refusing to run without --i-have-read-the-avoidance-doc."
  echo "Read AI NOTES/2026-05-12_W2-01_USB_Wedge_Avoidance_Research_Copilot_v1_0.md"
  echo "(especially sections 6a.1 and 6d.2) before invoking this script."
  exit 2
fi

if [ "$(id -u)" -ne 0 ]; then
  echo "ERROR: must run as root (sudo)."
  exit 2
fi

# Precondition 1: fw_env backend
if [ ! -r /etc/fw_env.config ]; then
  echo "ERROR: /etc/fw_env.config not readable. Cannot verify backend is eMMC."
  exit 3
fi
echo "--- /etc/fw_env.config ---"
cat /etc/fw_env.config
BACKEND_DEV="$(awk 'NF && $1 !~ /^#/ {print $1; exit}' /etc/fw_env.config)"
echo "backend_device=${BACKEND_DEV}"
case "$BACKEND_DEV" in
  /dev/mmcblk*|/mnt/boot/*|/dev/disk/*|/dev/sd*|/dev/nvme*)
    echo "OK: backend appears to be block storage (eMMC/SD/NVMe)."
    ;;
  /dev/mtd*|/dev/ubi*)
    echo "ERROR: backend is raw flash (${BACKEND_DEV}). NXP MTD bug since"
    echo "kernel 5.4 corrupts env via fw_setenv. Use Foundries OSTREE_KERNEL_ARGS"
    echo "instead. ABORTING."
    exit 4
    ;;
  *)
    echo "ERROR: unrecognized backend ${BACKEND_DEV}. Refusing to write."
    echo "If this is genuinely a block-backed env file, edit this script's"
    echo "case statement to allowlist it after manual verification."
    exit 4
    ;;
esac

# Precondition 2: fw_setenv binary
if ! command -v fw_setenv >/dev/null 2>&1; then
  echo "ERROR: fw_setenv not found in PATH."
  exit 5
fi
if ! command -v fw_printenv >/dev/null 2>&1; then
  echo "ERROR: fw_printenv not found in PATH."
  exit 5
fi

# Precondition 3: capture rollback
UTC="$(date -u +%Y%m%dT%H%M%SZ)"
ROLLBACK="/tmp/w2_01_bootargs_rollback_${UTC}.txt"
{
  echo "# W2-01 Option A rollback snapshot ${UTC}"
  echo "# /proc/cmdline at time of change:"
  cat /proc/cmdline
  echo "# fw_printenv (full):"
  fw_printenv 2>&1 || true
} > "$ROLLBACK"
echo "rollback_snapshot=$ROLLBACK"

# Identify the env var that holds the kernel cmdline. On Portenta X8 LmP this
# is typically `bootargs`; some images use a separate `extra_bootargs` or
# `bootcmd_args`. Probe in order.
CANDIDATES="bootargs extra_bootargs bootcmd_args"
TARGET=""
for k in $CANDIDATES; do
  if fw_printenv -n "$k" >/dev/null 2>&1; then
    TARGET="$k"
    break
  fi
done
if [ -z "$TARGET" ]; then
  echo "ERROR: none of [$CANDIDATES] exist in fw_printenv. Manual inspection required."
  echo "Full env saved at $ROLLBACK; do not re-run blindly."
  exit 6
fi
echo "target_env_var=$TARGET"

CURRENT="$(fw_printenv -n "$TARGET" 2>/dev/null || echo '')"
echo "current_${TARGET}=${CURRENT}"

case " $CURRENT " in
  *" usbcore.autosuspend=-1 "*)
    echo "OK: usbcore.autosuspend=-1 already present in ${TARGET}. Nothing to do."
    exit 0
    ;;
  *" usbcore.autosuspend="*)
    echo "ERROR: ${TARGET} already contains a different usbcore.autosuspend setting:"
    echo "  ${CURRENT}"
    echo "Refusing to overwrite. Edit manually after review."
    exit 7
    ;;
esac

NEW="${CURRENT} usbcore.autosuspend=-1"
echo "--- DRY RUN ---"
echo "Will execute: fw_setenv ${TARGET} \"${NEW}\""
echo "Press Ctrl+C in the next 5 seconds to abort."
sleep 5

fw_setenv "$TARGET" "$NEW"
echo "--- post-write verify ---"
fw_printenv -n "$TARGET"

echo
echo "Done. Reboot to take effect:   sudo reboot"
echo "After boot, verify with:"
echo "    cat /proc/cmdline | tr ' ' '\\n' | grep usbcore"
echo "    cat /sys/module/usbcore/parameters/autosuspend     # expect -1"
echo "Rollback snapshot: $ROLLBACK"
