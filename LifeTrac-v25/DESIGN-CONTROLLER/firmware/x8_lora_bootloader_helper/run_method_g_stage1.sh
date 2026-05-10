#!/bin/bash
# run_method_g_stage1.sh — flash a custom murata_l072 binary, boot it,
# then run the Stage 1 host-protocol probe over /dev/ttymxc3.
#
# Usage on the X8:
#   echo fio | sudo -S -p '' bash /tmp/lifetrac_p0c/run_method_g_stage1.sh [image.bin]
#
# Default image is /tmp/lifetrac_p0c/firmware.bin, which should be the
# custom murata_l072 build output copied to the X8 helper directory.

set -u

TOOLDIR=/tmp/lifetrac_p0c
IMAGE=${1:-$TOOLDIR/firmware.bin}
FLASH=$TOOLDIR/run_flash_l072.sh
PROBE=$TOOLDIR/method_g_stage1_probe.py
CFG_BOOT=$TOOLDIR/08_boot_user_app.cfg
DEV=/dev/ttymxc3
LOG=$TOOLDIR/method_g_stage1.log
OCD_LOG=$TOOLDIR/method_g_stage1_ocd.log
EARLY_UART_19200_LOG=$TOOLDIR/method_g_stage1_early_uart_19200.log
EARLY_UART_921600_LOG=$TOOLDIR/method_g_stage1_early_uart_921600.log

: > "$LOG"
: > "$OCD_LOG"
: > "$EARLY_UART_19200_LOG"
: > "$EARLY_UART_921600_LOG"

if [ ! -f "$FLASH" ]; then
  echo "ERROR: missing $FLASH" | tee -a "$LOG"
  exit 2
fi

if [ ! -f "$IMAGE" ]; then
  echo "ERROR: missing $IMAGE" | tee -a "$LOG"
  exit 2
fi

if [ ! -f "$PROBE" ]; then
  echo "ERROR: missing $PROBE" | tee -a "$LOG"
  exit 2
fi

if [ ! -f "$CFG_BOOT" ]; then
  echo "ERROR: missing $CFG_BOOT" | tee -a "$LOG"
  exit 2
fi

echo "=== $(date) Method G Stage 1 flash ===" | tee -a "$LOG"
bash "$FLASH" "$IMAGE" 2>&1 | tee -a "$LOG"
FLASH_RC=${PIPESTATUS[0]}
echo "flash_rc=$FLASH_RC" | tee -a "$LOG"

if [ "$FLASH_RC" -ne 0 ]; then
  echo "=== ABORT: flash failed ===" | tee -a "$LOG"
  exit "$FLASH_RC"
fi

echo "" | tee -a "$LOG"
echo "=== $(date) boot custom user app ===" | tee -a "$LOG"
# Run the boot-phase OpenOCD SYNCHRONOUSLY (no &). The CFG_BOOT script
# drives BOOT0 LOW, pulses NRST 250 ms, releases NRST, then invokes
# 'shutdown'. We must wait for OCD to exit before opening /dev/ttymxc3,
# otherwise the capture window begins during NRST (break condition on the
# floating TX line) rather than after the user app has started.
openocd -f /usr/arduino/extra/openocd_script-imx_gpio.cfg \
         -f "$CFG_BOOT" \
         > "$OCD_LOG" 2>&1 < /dev/null
echo "boot_ocd_rc=$?" | tee -a "$LOG"

cat "$OCD_LOG" | tee -a "$LOG"

# NRST has been released. Give the STM32 a short moment for Reset_Handler
# to start and configure LPUART1 before we open the port.
sleep 0.1
fuser -k "$DEV" 2>/dev/null || true

# Capture at 921600 8N1 immediately — BOOT_URC fires ~500ms after NRST
# while safe_mode_listen holds (500ms window). Starting the capture here
# ensures we see the URC rather than it racing against a sequential 19200
# window that would only show the same bytes as garbage.
stty -F "$DEV" 921600 cs8 -parenb -cstopb raw -echo
# No byte-count limit — rely purely on timeout so the capture stays open
# long enough for BOOT_URC which arrives ~500ms after NRST (after
# safe_mode_listen completes). A count=4096 cap was filling in ~44ms from
# the 19200-baud startup text decoded as break-frames, causing dd to exit
# before the 921600 protocol traffic even started.
if command -v timeout >/dev/null 2>&1; then
  timeout 5 cat "$DEV" > "$EARLY_UART_921600_LOG" || true
else
  dd if="$DEV" bs=4096 status=none 2>/dev/null > "$EARLY_UART_921600_LOG" || true
fi

fuser -k "$DEV" 2>/dev/null || true
sleep 0.2

echo "" | tee -a "$LOG"
echo "=== $(date) early UART capture @921600 (hex) ===" | tee -a "$LOG"
if [ -s "$EARLY_UART_921600_LOG" ]; then
  xxd "$EARLY_UART_921600_LOG" | tee -a "$LOG" || od -A x -t x1z "$EARLY_UART_921600_LOG" | tee -a "$LOG"
  echo "" | tee -a "$LOG"
  echo "early_921600_bytes=$(wc -c < "$EARLY_UART_921600_LOG")" | tee -a "$LOG"
else
  echo "(no bytes captured)" | tee -a "$LOG"
fi

echo "" | tee -a "$LOG"
echo "=== $(date) Stage 1 probe ===" | tee -a "$LOG"
python3 -u "$PROBE" --dev "$DEV" --baud 921600 2>&1 | tee -a "$LOG"
PROBE_RC=${PIPESTATUS[0]}

echo "" | tee -a "$LOG"
echo "=== DONE: flash=$FLASH_RC probe=$PROBE_RC ===" | tee -a "$LOG"
echo "log=$LOG"
exit "$PROBE_RC"