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
bash $TOOLDIR/revive_bridge.sh 2>&1 | tee -a "$LOG"
REVIVE_RC=${PIPESTATUS[0]}
echo "revive_rc=$REVIVE_RC" | tee -a "$LOG"

echo "" | tee -a "$LOG"
echo "=== T2.5 $(date) wdt_pet stop (let kernel [watchdogd] take back over) ===" | tee -a "$LOG"
bash $TOOLDIR/wdt_pet.sh stop 2>&1 | tee -a "$LOG"

echo "" | tee -a "$LOG"
echo "=== T3 $(date) DONE: prep=$PREP_RC flash=$FLASH_RC revive=$REVIVE_RC ===" | tee -a "$LOG"
exit $REVIVE_RC
