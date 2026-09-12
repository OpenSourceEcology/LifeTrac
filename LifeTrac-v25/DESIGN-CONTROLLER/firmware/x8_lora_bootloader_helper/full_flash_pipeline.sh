#!/bin/bash
# full_flash_pipeline.sh — orchestrate prep + flash + revive end-to-end.
#
# Usage: echo fio | sudo -S -p '' bash /tmp/lifetrac_p0c/full_flash_pipeline.sh [image.bin]
#
# Default image is hello.bin in /tmp/lifetrac_p0c/.
# Logs to /tmp/lifetrac_p0c/pipeline.log

set -u
TOOLDIR=/tmp/lifetrac_p0c
IMAGE=${1:-$TOOLDIR/hello.bin}
LOG=$TOOLDIR/pipeline.log
: > "$LOG"

echo "=== T-1 $(date) wdt_pet start (HW WDT is 60s; flashes >60s reboot otherwise) ===" | tee -a "$LOG"
bash $TOOLDIR/wdt_pet.sh start 2>&1 | tee -a "$LOG"

echo "=== T0 $(date) prep_bridge ===" | tee -a "$LOG"
bash $TOOLDIR/prep_bridge.sh 2>&1 | tee -a "$LOG"
PREP_RC=${PIPESTATUS[0]}
echo "prep_rc=$PREP_RC" | tee -a "$LOG"

if [ "$PREP_RC" -ne 0 ]; then
  echo "=== ABORT: prep_bridge failed ===" | tee -a "$LOG"
  bash $TOOLDIR/wdt_pet.sh stop 2>&1 | tee -a "$LOG"
  exit 2
fi

echo "" | tee -a "$LOG"
echo "=== T1 $(date) flash $IMAGE ===" | tee -a "$LOG"
bash $TOOLDIR/run_flash_l072.sh "$IMAGE" 2>&1 | tee -a "$LOG"
FLASH_RC=${PIPESTATUS[0]}
echo "flash_rc=$FLASH_RC" | tee -a "$LOG"

echo "" | tee -a "$LOG"
echo "=== T2 $(date) revive_bridge ===" | tee -a "$LOG"
# REVIVE_MODE=reboot (2026-09-12): skip the x8h7 module reload, which OOPSes
# the base kernel (6.1.24-lmp) and power-cycles the board anyway. Bring the
# H7 back with openocd reset run, hand the watchdog back, then reboot on
# purpose. /tmp is lost either way; the caller re-pushes tooling.
if [ "${REVIVE_MODE:-full}" = "reboot" ]; then
  REVIVE_MODE=reset_run_only bash $TOOLDIR/revive_bridge.sh 2>&1 | tee -a "$LOG"
  REVIVE_RC=${PIPESTATUS[0]}
  echo "revive_rc=$REVIVE_RC (reset_run_only)" | tee -a "$LOG"
  bash $TOOLDIR/wdt_pet.sh stop 2>&1 | tee -a "$LOG"
  echo "=== T3 $(date) DONE: prep=$PREP_RC flash=$FLASH_RC revive=$REVIVE_RC; REBOOTING (REVIVE_MODE=reboot) ===" | tee -a "$LOG"
  sync
  systemctl reboot
  exit "$FLASH_RC"
fi
bash $TOOLDIR/revive_bridge.sh 2>&1 | tee -a "$LOG"
REVIVE_RC=${PIPESTATUS[0]}
echo "revive_rc=$REVIVE_RC" | tee -a "$LOG"

echo "" | tee -a "$LOG"
echo "=== T2.5 $(date) wdt_pet stop (let kernel [watchdogd] take back over) ===" | tee -a "$LOG"
bash $TOOLDIR/wdt_pet.sh stop 2>&1 | tee -a "$LOG"

echo "" | tee -a "$LOG"
echo "=== T3 $(date) DONE: prep=$PREP_RC flash=$FLASH_RC revive=$REVIVE_RC ===" | tee -a "$LOG"
if [ "$FLASH_RC" -ne 0 ]; then
  exit "$FLASH_RC"
fi
exit "$REVIVE_RC"
