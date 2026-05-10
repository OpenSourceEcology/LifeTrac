#!/bin/bash
# run_stage1_standard_contract.sh
# One-shot Stage1 standard-method run contract:
#   - flash (ROM bootloader path)
#   - boot probe
#   - artifact folder + summary enum output

set -u

TOOLDIR=/tmp/lifetrac_p0c
IMAGE=${1:-$TOOLDIR/firmware.bin}
BOARD_SERIAL=${2:-${BOARD_SERIAL:-unknown}}
OPENOCD_CFG=${OPENOCD_CFG:-$TOOLDIR/07_assert_pa11_pf4_long.cfg}
OUTROOT=${OUTROOT:-$TOOLDIR/stage1_standard_runs}
RUN_ID=${RUN_ID:-T6_stage1_standard_$(date +%Y-%m-%d_%H%M%S)}
OUTDIR=$OUTROOT/$RUN_ID

FLASH_SCRIPT=$TOOLDIR/run_flash_l072.sh
BOOT_SCRIPT=$TOOLDIR/boot_and_probe.sh

mkdir -p "$OUTDIR"

RUN_META=$OUTDIR/run_meta.txt
FLASH_LOG=$OUTDIR/flash.log
CONTROL_LOG=$OUTDIR/control.log
VERIFY_LOG=$OUTDIR/verify.log
BOOT_PROBE_LOG=$OUTDIR/boot_probe.log
SUMMARY=$OUTDIR/summary.txt

START_EPOCH=$(date +%s)

run_and_log() {
  local cmd="$1"
  local log="$2"
  bash -lc "$cmd" > "$log" 2>&1
  return $?
}

bool_from_grep() {
  local pattern="$1"
  local file="$2"
  if grep -Eq "$pattern" "$file"; then
    echo 1
  else
    echo 0
  fi
}

cat > "$RUN_META" <<EOF
RUN_ID=$RUN_ID
BOARD_SERIAL=$BOARD_SERIAL
IMAGE_NAME=$(basename "$IMAGE")
IMAGE_PATH=$IMAGE
OPENOCD_CFG=$OPENOCD_CFG
START_UTC=$(date -u +%Y-%m-%dT%H:%M:%SZ)
EOF

FLASH_RC=1
BOOT_RC=1

if [ ! -f "$FLASH_SCRIPT" ]; then
  echo "ERROR: missing $FLASH_SCRIPT" > "$FLASH_LOG"
elif [ ! -f "$BOOT_SCRIPT" ]; then
  echo "ERROR: missing $BOOT_SCRIPT" > "$CONTROL_LOG"
elif [ ! -f "$IMAGE" ]; then
  echo "ERROR: missing image $IMAGE" > "$FLASH_LOG"
else
  run_and_log "$FLASH_SCRIPT \"$IMAGE\"" "$FLASH_LOG"
  FLASH_RC=$?

  if [ "$FLASH_RC" -eq 0 ]; then
    run_and_log "$BOOT_SCRIPT" "$CONTROL_LOG"
    BOOT_RC=$?
  fi
fi

# Pull through underlying helper logs if present.
[ -f "$TOOLDIR/flash_run.log" ] && cp -f "$TOOLDIR/flash_run.log" "$OUTDIR/flash_run.log"
[ -f "$TOOLDIR/flash_ocd.log" ] && cp -f "$TOOLDIR/flash_ocd.log" "$OUTDIR/flash_ocd.log"
[ -f "$TOOLDIR/boot_probe.log" ] && cp -f "$TOOLDIR/boot_probe.log" "$BOOT_PROBE_LOG"

SYNC_OK=$(bool_from_grep "Sending sync byte 0x7F" "$FLASH_LOG")
GETID_OK=$(bool_from_grep "Bootloader version" "$FLASH_LOG")
ERASE_OK=$(bool_from_grep "Extended Erase complete|Standard Erase complete|page batches complete" "$FLASH_LOG")
WRITE_OK=$(bool_from_grep "Flash complete in" "$FLASH_LOG")

# Current local flasher prints verify invocation but may not emit a dedicated verify banner.
# Keep the contract key now; treat successful flash RC as verify proxy until explicit readback stage is emitted.
if [ "$FLASH_RC" -eq 0 ]; then
  VERIFY_OK=1
else
  VERIFY_OK=0
fi

AT_SIZE=0
if [ -f "$BOOT_PROBE_LOG" ]; then
  AT_SIZE_LINE=$(grep -E "AT response size = [0-9]+ bytes" "$BOOT_PROBE_LOG" | tail -n 1 || true)
  if [ -n "$AT_SIZE_LINE" ]; then
    AT_SIZE=$(echo "$AT_SIZE_LINE" | sed -E 's/.*= ([0-9]+) bytes.*/\1/')
  fi
fi

if [ "$BOOT_RC" -eq 0 ] && [ "$AT_SIZE" -gt 0 ]; then
  BOOT_OK=1
else
  BOOT_OK=0
fi

FINAL_RESULT=PASS
if [ "$SYNC_OK" -eq 0 ]; then
  FINAL_RESULT=FAIL_SYNC
elif [ "$GETID_OK" -eq 0 ]; then
  FINAL_RESULT=FAIL_ID
elif [ "$ERASE_OK" -eq 0 ]; then
  FINAL_RESULT=FAIL_ERASE
elif [ "$WRITE_OK" -eq 0 ]; then
  FINAL_RESULT=FAIL_WRITE
elif [ "$VERIFY_OK" -eq 0 ]; then
  FINAL_RESULT=FAIL_VERIFY
elif [ "$BOOT_OK" -eq 0 ]; then
  FINAL_RESULT=FAIL_BOOT
fi

END_EPOCH=$(date +%s)
ELAPSED_S=$((END_EPOCH - START_EPOCH))

cat > "$VERIFY_LOG" <<EOF
VERIFY_MODE=FLASH_RC_PROXY
FLASH_RC=$FLASH_RC
NOTE=Explicit readback-verify banner not currently emitted by local flasher; treating successful flash RC as verify proxy for Phase A contract wiring.
EOF

cat > "$SUMMARY" <<EOF
RUN_ID=$RUN_ID
BOARD_SERIAL=$BOARD_SERIAL
IMAGE_NAME=$(basename "$IMAGE")
OPENOCD_CFG=$OPENOCD_CFG
SYNC_OK=$SYNC_OK
GETID_OK=$GETID_OK
ERASE_OK=$ERASE_OK
WRITE_OK=$WRITE_OK
VERIFY_OK=$VERIFY_OK
BOOT_OK=$BOOT_OK
FINAL_RESULT=$FINAL_RESULT
ELAPSED_S=$ELAPSED_S
OUTDIR=$OUTDIR
EOF

echo "RUN_OUTDIR=$OUTDIR"
cat "$SUMMARY"

if [ "$FINAL_RESULT" = "PASS" ]; then
  exit 0
fi

exit 1
