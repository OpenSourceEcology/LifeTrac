#!/bin/bash
# run_flash_bench.sh — instrumented wrapper around full_flash_pipeline.sh:
# kernel log tail + monotonic-stamped pipeline log + fsync'd pet log,
# all under /home/fio so they survive a PMIC power-cycle.
#
# Usage (on the board; see FLASH_RUNBOOK.md, or flash_l072.sh from the PC):
#   bash /home/fio/run_flash_bench.sh /tmp/lifetrac_p0c/<image>.bin
# Environment (all optional; defaults are the 2026 bench values):
#   REVIVE_MODE        reboot (default since 2026-10-10) | full
#                      reboot = openocd `reset run`, hand the watchdog back,
#                      `systemctl reboot`. full = re-insert the x8h7 modules,
#                      which OOPSes the base's 6.1.24 kernel; only for 5.10.
#   FLASH_VERIFY_ONLY  1 = read back and compare, write nothing (first contact;
#                      still enters the ROM bootloader and reboots afterwards)
#   FLASH_PREFLIGHT_ONLY 1 = check the staging below and exit; touches nothing
#   BENCH_SUDO_PW      sudo password (default: the LmP default `fio`; the bench
#                      boards have a NOPASSWD sudoers drop-in anyway)
#   BENCH_HOME         where the wrapper, stamp.py, kmsg_log.py and the logs
#                      live (default /home/fio, persistent across reboots)
IMG=${1:?usage: run_flash_bench.sh /tmp/lifetrac_p0c/<image>.bin}
PW=${BENCH_SUDO_PW:-fio}
H=${BENCH_HOME:-/home/fio}
TOOLDIR=/tmp/lifetrac_p0c
REVIVE_MODE=${REVIVE_MODE:-reboot}
FLASH_VERIFY_ONLY=${FLASH_VERIFY_ONLY:-0}
export PATH=/usr/sbin:/sbin:/usr/local/sbin:/usr/bin:/bin:$PATH

case "$REVIVE_MODE" in
  reboot|full) ;;
  *) echo "PREFLIGHT-FAIL: REVIVE_MODE=$REVIVE_MODE (want reboot or full)"; exit 3 ;;
esac

# Preflight (2026-10-10): refuse before ANY hardware is touched when the
# staging is incomplete or CRLF-damaged. /tmp is tmpfs and systemd-tmpfiles
# ages it at 5 d, so the pipeline files vanish with the board up; a CRLF
# script fails in confusing ways (BENCH_RUNBOOK "Board facts").
# Only the files this pipeline executes (08_boot_user_app.cfg belongs to the
# harness's tractor reset and is checked there).
need="$TOOLDIR/full_flash_pipeline.sh $TOOLDIR/run_flash_l072.sh $TOOLDIR/prep_bridge.sh
      $TOOLDIR/revive_bridge.sh $TOOLDIR/wdt_pet.sh $TOOLDIR/stm32_an3155_flasher.py
      $TOOLDIR/07_assert_pa11_pf4_long.cfg $H/stamp.py $H/kmsg_log.py"
missing=""
for f in $need "$IMG"; do
  [ -s "$f" ] || missing="$missing $f"
done
if [ -n "$missing" ]; then
  echo "PREFLIGHT-FAIL: missing or empty:$missing"
  echo "Re-stage /tmp/lifetrac_p0c and $H (FLASH_RUNBOOK section 1). Nothing was touched."
  exit 3
fi
# CRLF breaks bash and the openocd cfg; python tolerates it, so only .sh/.cfg.
# (tr|cmp rather than grep $'\r': same answer on busybox, GNU and Git Bash.)
crlf=""
for f in $need; do
  case "$f" in *.sh|*.cfg) tr -d '\r' < "$f" | cmp -s - "$f" || crlf="$crlf $f" ;; esac
done
if [ -n "$crlf" ]; then
  echo "PREFLIGHT-FAIL: CRLF line endings in:" $crlf
  echo "Re-push LF-clean copies (FLASH_RUNBOOK section 1). Nothing was touched."
  exit 3
fi
echo "PREFLIGHT-OK image=$IMG size=$(stat -c%s "$IMG") md5=$(md5sum "$IMG" | cut -d' ' -f1) REVIVE_MODE=$REVIVE_MODE FLASH_VERIFY_ONLY=$FLASH_VERIFY_ONLY"
if [ "${FLASH_PREFLIGHT_ONLY:-0}" = "1" ]; then
  echo "FLASH_PREFLIGHT_ONLY=1: staging checked, stopping before openocd/UART/watchdog."
  exit 0
fi

rm -f "$H/pipeline_stamped.log" "$H/kmsg_flash.log" "$H/wdt_pet.log"
echo "$PW" | sudo -S -p '' bash -c "nohup python3 -u $H/kmsg_log.py $H/kmsg_flash.log >/dev/null 2>&1 & echo \$! > $H/kmsg_log.pid"
sleep 0.5
echo "kmsg logger pid=$(cat "$H/kmsg_log.pid") uptime_s=$(cut -d' ' -f1 /proc/uptime)"
echo "$PW" | sudo -S -p '' env PATH=$PATH REVIVE_MODE=$REVIVE_MODE FLASH_VERIFY_ONLY=$FLASH_VERIFY_ONLY WDT_PET_LOG=$H/wdt_pet.log bash $TOOLDIR/full_flash_pipeline.sh "$IMG" 2>&1 | python3 -u "$H/stamp.py" "$H/pipeline_stamped.log"
RC=${PIPESTATUS[1]}
echo "PIPELINE-EXIT=$RC uptime_s=$(cut -d' ' -f1 /proc/uptime)"
echo "$PW" | sudo -S -p '' kill "$(cat "$H/kmsg_log.pid")" 2>/dev/null
echo "--- wdt_pet.log tail:"; tail -3 "$H/wdt_pet.log"
echo "--- kmsg tail:"; tail -5 "$H/kmsg_flash.log"
exit $RC
