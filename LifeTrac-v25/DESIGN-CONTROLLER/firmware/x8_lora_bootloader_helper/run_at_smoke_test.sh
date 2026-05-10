#!/bin/bash
# run_at_smoke_test.sh — boot L072 into user app and run AT smoke test.
#
# Usage (from X8, after firmware already flashed):
#   echo fio | sudo -S -p '' bash /tmp/lifetrac_p0c/run_at_smoke_test.sh
#
# What this does:
#   1. Launches openocd to boot L072 into user flash (BOOT0 low, NRST pulse)
#   2. Waits for firmware to settle
#   3. Runs at_smoke_test.py over /dev/ttymxc3 @ 115200 8N1
#   4. Kills openocd
#   5. Exits with at_smoke_test.py exit code

set -u
TOOLDIR=/tmp/lifetrac_p0c
CFG_BOOT=$TOOLDIR/08_boot_user_app.cfg
SMOKE=$TOOLDIR/at_smoke_test.py
LOG=$TOOLDIR/at_smoke.log
OCD_LOG=$TOOLDIR/at_smoke_ocd.log
DEV=/dev/ttymxc3

: > "$LOG"
: > "$OCD_LOG"

if [ ! -f "$CFG_BOOT" ]; then
  echo "ERROR: missing $CFG_BOOT" | tee -a "$LOG"; exit 2
fi
if [ ! -f "$SMOKE" ]; then
  echo "ERROR: missing $SMOKE" | tee -a "$LOG"; exit 2
fi

echo "=== $(date) boot L072 into user app ===" | tee -a "$LOG"

# Start openocd to drive BOOT0 LOW + pulse NRST — this boots to user firmware.
nohup openocd -f /usr/arduino/extra/openocd_script-imx_gpio.cfg \
              -f "$CFG_BOOT" \
              > "$OCD_LOG" 2>&1 < /dev/null &
OCD_PID=$!
echo "OCD_PID=$OCD_PID" | tee -a "$LOG"

# Wait for openocd to complete the boot sequence (it runs the TCL script and exits).
sleep 4

echo "" | tee -a "$LOG"
echo "=== openocd log ===" | tee -a "$LOG"
cat "$OCD_LOG" | tee -a "$LOG"

# Kill openocd if still running (it may block waiting for connections).
kill -9 "$OCD_PID" 2>/dev/null || true
sleep 1

# Release UART from any leftover users.
fuser -k "$DEV" 2>/dev/null || true
sleep 0.5

echo "" | tee -a "$LOG"
echo "=== $(date) running AT smoke test ===" | tee -a "$LOG"
python3 -u "$SMOKE" 2>&1 | tee -a "$LOG"
AT_RC=${PIPESTATUS[0]}

echo "" | tee -a "$LOG"
echo "=== AT smoke test exit code: $AT_RC ===" | tee -a "$LOG"
echo "log=$LOG"
exit $AT_RC
