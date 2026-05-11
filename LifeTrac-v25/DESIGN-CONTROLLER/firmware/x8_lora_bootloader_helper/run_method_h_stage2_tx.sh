#!/bin/bash
# run_method_h_stage2_tx.sh — boot the murata_l072 user app (no flash)
# and run the W1-9 Stage 2 TX bring-up probe over /dev/ttymxc3.
#
# Usage on the X8:
#   echo fio | sudo -S -p '' bash /tmp/lifetrac_p0c/run_method_h_stage2_tx.sh
#
# This script intentionally does NOT reflash — the W1-8 firmware is
# expected to already be on the L072. To reflash, run the Stage 1 helper
# first, then this script.

set -u

TOOLDIR=/tmp/lifetrac_p0c
PROBE=$TOOLDIR/method_h_stage2_tx_probe.py
STAGE1_PROBE=$TOOLDIR/method_g_stage1_probe.py
CFG_BOOT=$TOOLDIR/08_boot_user_app.cfg
DEV_LIST_DEFAULT="/dev/ttymxc3 /dev/ttymxc2 /dev/ttymxc1 /dev/ttymxc0"
DEV_LIST=${LIFETRAC_UART_DEV_LIST:-$DEV_LIST_DEFAULT}
LOG=$TOOLDIR/method_h_stage2_tx.log
OCD_LOG=$TOOLDIR/method_h_stage2_tx_ocd.log

: > "$LOG"
: > "$OCD_LOG"

serial_dev_usable() {
  local dev="$1"
  if [ ! -e "$dev" ] || [ ! -c "$dev" ]; then
    return 1
  fi
  if ! stty -F "$dev" 921600 cs8 -parenb -cstopb raw -echo > /dev/null 2>&1; then
    return 1
  fi
  return 0
}

DEV_LIST_EFFECTIVE="$DEV_LIST"
for auto_dev in /dev/ttymxc* /dev/ttyX* /dev/ttyUSB* /dev/ttyACM*; do
  [ -e "$auto_dev" ] || continue
  case " $DEV_LIST_EFFECTIVE " in
    *" $auto_dev "*) ;;
    *) DEV_LIST_EFFECTIVE="$DEV_LIST_EFFECTIVE $auto_dev" ;;
  esac
done

DEV=""
for candidate in $DEV_LIST_EFFECTIVE; do
  if serial_dev_usable "$candidate"; then
    DEV="$candidate"
    break
  fi
done
[ -z "$DEV" ] && DEV=/dev/ttymxc3

for required in "$PROBE" "$STAGE1_PROBE" "$CFG_BOOT"; do
  if [ ! -f "$required" ]; then
    echo "ERROR: missing $required" | tee -a "$LOG"
    exit 2
  fi
done

echo "=== $(date) Method H Stage 2 boot user app ===" | tee -a "$LOG"
openocd -f /usr/arduino/extra/openocd_script-imx_gpio.cfg \
        -f "$CFG_BOOT" \
        > "$OCD_LOG" 2>&1 < /dev/null
BOOT_RC=$?
echo "boot_ocd_rc=$BOOT_RC" | tee -a "$LOG"
cat "$OCD_LOG" | tee -a "$LOG"

if [ "$BOOT_RC" -ne 0 ]; then
  echo "=== ABORT: openocd boot failed ===" | tee -a "$LOG"
  exit "$BOOT_RC"
fi

# Let Reset_Handler bring up LPUART1 + radio_init + the host BOOT_URC.
sleep 0.5
fuser -k "$DEV" 2>/dev/null || true
sleep 0.1

echo "" | tee -a "$LOG"
echo "=== $(date) Stage 2 TX probe on $DEV ===" | tee -a "$LOG"
PYTHONPATH="$TOOLDIR" python3 -u "$PROBE" --dev "$DEV" --baud 921600 2>&1 | tee -a "$LOG"
PROBE_RC=${PIPESTATUS[0]}
echo "probe_rc[$DEV]=$PROBE_RC" | tee -a "$LOG"

echo "" | tee -a "$LOG"
echo "=== DONE: boot=$BOOT_RC probe=$PROBE_RC ===" | tee -a "$LOG"
exit "$PROBE_RC"
