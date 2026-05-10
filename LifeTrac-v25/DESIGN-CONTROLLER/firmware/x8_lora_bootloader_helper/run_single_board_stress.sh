#!/bin/bash
# run_single_board_stress.sh — run N one-board flash pipeline cycles and
# summarize reliability for Method G bench progression.
#
# Usage:
#   echo fio | sudo -S -p '' bash /tmp/lifetrac_p0c/run_single_board_stress.sh [image.bin] [cycles]
#
# Defaults:
#   image  = /tmp/lifetrac_p0c/hello.bin
#   cycles = 10
#
# Environment knobs passed through to full_flash_pipeline.sh:
#   RUN_POST_LISTEN (default 1)
#   POST_LISTEN_SEC (default 8)
#   RUN_REVIVE (default 0)

set -u

TOOLDIR=/tmp/lifetrac_p0c
PIPELINE=$TOOLDIR/full_flash_pipeline.sh
IMAGE=${1:-$TOOLDIR/hello.bin}
CYCLES=${2:-10}
RUN_POST_LISTEN=${RUN_POST_LISTEN:-1}
POST_LISTEN_SEC=${POST_LISTEN_SEC:-8}
RUN_REVIVE=${RUN_REVIVE:-0}

STAMP=$(date +%Y%m%d_%H%M%S)
OUTDIR=$TOOLDIR/stress_runs/$STAMP
SUMMARY=$OUTDIR/summary.txt
TSV=$OUTDIR/results.tsv

mkdir -p "$OUTDIR"

if [ ! -f "$IMAGE" ]; then
  if [ "$IMAGE" = "$TOOLDIR/hello.bin" ] && [ -f "$TOOLDIR/mlm32l07x01.bin" ]; then
    IMAGE="$TOOLDIR/mlm32l07x01.bin"
  fi
fi

echo "=== single-board stress run start $(date) ===" | tee "$SUMMARY"
echo "outdir=$OUTDIR" | tee -a "$SUMMARY"
echo "image=$IMAGE" | tee -a "$SUMMARY"
echo "cycles=$CYCLES" | tee -a "$SUMMARY"
echo "RUN_POST_LISTEN=$RUN_POST_LISTEN POST_LISTEN_SEC=$POST_LISTEN_SEC RUN_REVIVE=$RUN_REVIVE" | tee -a "$SUMMARY"

if [ ! -f "$PIPELINE" ]; then
  echo "ERROR: missing pipeline script $PIPELINE" | tee -a "$SUMMARY"
  exit 2
fi

if [ ! -f "$IMAGE" ]; then
  echo "ERROR: missing image $IMAGE" | tee -a "$SUMMARY"
  exit 2
fi

echo -e "cycle\tstart_epoch\tend_epoch\trc\tbanner_hit\ttick_hit\trx_bytes\tauto_reboot_seen" > "$TSV"

pass_count=0
fail_count=0
banner_hit_count=0
tick_hit_count=0
auto_reboot_count=0

for i in $(seq 1 "$CYCLES"); do
  CYCLE_DIR=$OUTDIR/cycle_$(printf "%03d" "$i")
  mkdir -p "$CYCLE_DIR"

  echo "" | tee -a "$SUMMARY"
  echo "=== cycle $i/$CYCLES start $(date) ===" | tee -a "$SUMMARY"

  START_EPOCH=$(date +%s)

  RUN_POST_LISTEN="$RUN_POST_LISTEN" \
  POST_LISTEN_SEC="$POST_LISTEN_SEC" \
  RUN_REVIVE="$RUN_REVIVE" \
  bash "$PIPELINE" "$IMAGE" > "$CYCLE_DIR/pipeline.stdout.log" 2>&1
  RC=$?

  END_EPOCH=$(date +%s)

  # Capture generated helper logs when present.
  cp -f "$TOOLDIR/pipeline.log" "$CYCLE_DIR/" 2>/dev/null || true
  cp -f "$TOOLDIR/flash_run.log" "$CYCLE_DIR/" 2>/dev/null || true
  cp -f "$TOOLDIR/flash_ocd.log" "$CYCLE_DIR/" 2>/dev/null || true
  cp -f "$TOOLDIR/boot_listen.log" "$CYCLE_DIR/" 2>/dev/null || true
  cp -f "$TOOLDIR/openocd_boot.log" "$CYCLE_DIR/" 2>/dev/null || true
  cp -f "$TOOLDIR/rx.bin" "$CYCLE_DIR/" 2>/dev/null || true

  # Analyze runtime capture from rx.bin.
  BANNER_HIT=0
  TICK_HIT=0
  RX_BYTES=0
  if [ -f "$CYCLE_DIR/rx.bin" ]; then
    RX_BYTES=$(stat -c %s "$CYCLE_DIR/rx.bin" 2>/dev/null || echo 0)

    # Banner: SX1276 chip detected and initialized (any custom or MKRWAN firmware)
    if strings "$CYCLE_DIR/rx.bin" | grep -qE "SX1276 version=|LIFETRAC L072|ready\.|Role: (PINGER|PONGER)"; then
      BANNER_HIT=1
      banner_hit_count=$((banner_hit_count + 1))
    fi

    # Tick: firmware actively transmitting or counting (MKRWAN pinger, or hello.bin tick)
    if strings "$CYCLE_DIR/rx.bin" | grep -qE "TX PING #|tick=|PING #[0-9]"; then
      TICK_HIT=1
      tick_hit_count=$((tick_hit_count + 1))
    fi
  fi

  # Heuristic: auto reboot usually shows in pipeline output around wdt/uptime reset notes.
  AUTO_REBOOT_SEEN=0
  if grep -Eiq "auto-reboot|uptime=0|uptime: 0 min|reboot" "$CYCLE_DIR/pipeline.stdout.log"; then
    AUTO_REBOOT_SEEN=1
    auto_reboot_count=$((auto_reboot_count + 1))
  fi

  if [ "$RC" -eq 0 ]; then
    pass_count=$((pass_count + 1))
    echo "cycle $i PASS rc=$RC rx_bytes=$RX_BYTES banner=$BANNER_HIT tick=$TICK_HIT" | tee -a "$SUMMARY"
  else
    fail_count=$((fail_count + 1))
    echo "cycle $i FAIL rc=$RC rx_bytes=$RX_BYTES banner=$BANNER_HIT tick=$TICK_HIT" | tee -a "$SUMMARY"
  fi

  echo -e "$i\t$START_EPOCH\t$END_EPOCH\t$RC\t$BANNER_HIT\t$TICK_HIT\t$RX_BYTES\t$AUTO_REBOOT_SEEN" >> "$TSV"

done

PASS_PCT=0
if [ "$CYCLES" -gt 0 ]; then
  PASS_PCT=$((100 * pass_count / CYCLES))
fi

echo "" | tee -a "$SUMMARY"
echo "=== stress summary $(date) ===" | tee -a "$SUMMARY"
echo "pass=$pass_count fail=$fail_count total=$CYCLES pass_pct=${PASS_PCT}%" | tee -a "$SUMMARY"
echo "banner_hits=$banner_hit_count tick_hits=$tick_hit_count auto_reboot_seen=$auto_reboot_count" | tee -a "$SUMMARY"
echo "tsv=$TSV" | tee -a "$SUMMARY"

# Simple gate recommendation aligned with one-board plan.
if [ "$PASS_PCT" -ge 95 ] && [ "$banner_hit_count" -ge $((95 * CYCLES / 100)) ]; then
  echo "recommendation=READY_FOR_NEXT_STAGE" | tee -a "$SUMMARY"
else
  echo "recommendation=STAY_IN_SINGLE_BOARD_HARDENING" | tee -a "$SUMMARY"
fi

exit 0
