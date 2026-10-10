#!/bin/bash
# flash_l072.sh -- FLASH_RUNBOOK.md sections 1-2 as one PC-side script (Git
# Bash on the bench PC, or any Linux shell with adb/ssh).
#
#   bash flash_l072.sh <base|tractor> <firmware.bin> [--go] [--verify-only] [--revive reboot|full]
#
# Without --go: copies the flash pipeline LF-clean to /tmp/lifetrac_p0c and the
# wrapper to /home/fio on the board, then runs the wrapper's preflight
# (FLASH_PREFLIGHT_ONLY=1: file presence, CRLF, image md5). Nothing touches
# openocd, the radio UART or the watchdog.
# With --go: also stops the tractor's camera unit/container and runs the flash
# (run_flash_bench.sh -> full_flash_pipeline.sh). A FLASH IS A RADIO-ON EVENT:
# the L072 boots into RXCONT afterwards, and REVIVE_MODE=reboot reboots the X8
# (12-24 s; /tmp is wiped). Use --go only on the operator's explicit GO.
# --verify-only: read back and compare against <firmware.bin>, write nothing
# (still enters the ROM bootloader and reboots; needs --go too).
#
# Configuration: sources $BENCH_ENV (default: bench.env next to this script;
# template bench.env.example) when it exists. Each variable can also come from
# the environment; the defaults are the 2026 bench:
#   BASE_SERIAL     2D0A1209DABC240B  base adb serial (the base is flashed over ssh)
#   TRACTOR_SERIAL  2E2C1209DABC240B  tractor adb serial (adb only, WiFi off)
#   BASE_HOST       192.168.1.117     base ethernet address (DHCP lease)
#   BASE_USER       fio
#   BASE_SSH_KEY    ~/.ssh/lifetrac_base_ed25519
#   BENCH_SUDO_PW   fio               LmP default; both boards have NOPASSWD sudo
#   BOARD_HOME      /home/fio         persistent home for the wrapper and logs
#   REVIVE_MODE     reboot            or full (5.10 kernels only; see FLASH_RUNBOOK)
set -euo pipefail

HERE=$(cd "$(dirname "$0")" && pwd)
HELPER=$(cd "$HERE/.." && pwd)
BENCH_ENV=${BENCH_ENV:-$HERE/bench.env}
if [ -f "$BENCH_ENV" ]; then
  # shellcheck disable=SC1090
  . "$BENCH_ENV"
fi
BASE_SERIAL=${BASE_SERIAL:-2D0A1209DABC240B}
TRACTOR_SERIAL=${TRACTOR_SERIAL:-2E2C1209DABC240B}
BASE_HOST=${BASE_HOST:-192.168.1.117}
BASE_USER=${BASE_USER:-fio}
BASE_SSH_KEY=${BASE_SSH_KEY:-$HOME/.ssh/lifetrac_base_ed25519}
BASE_SSH_KEY=${BASE_SSH_KEY/#\~/$HOME}
BENCH_SUDO_PW=${BENCH_SUDO_PW:-fio}
BOARD_HOME=${BOARD_HOME:-/home/fio}
REVIVE_MODE=${REVIVE_MODE:-reboot}

die() { echo "flash_l072: $*" >&2; exit 2; }
usage() { awk 'NR > 1 && /^#/ { sub(/^# ?/, ""); print; next } NR > 1 { exit }' "$0"; exit 2; }

[ $# -ge 2 ] || usage
BOARD=$1; IMG=$2; shift 2
GO=0; VERIFY_ONLY=0
while [ $# -gt 0 ]; do
  case "$1" in
    --go) GO=1 ;;
    --verify-only) VERIFY_ONLY=1 ;;
    --revive) [ $# -ge 2 ] || die "--revive needs reboot|full"; REVIVE_MODE=$2; shift ;;
    -h|--help) usage ;;
    *) die "unknown option: $1" ;;
  esac
  shift
done
case "$BOARD" in base|tractor) ;; *) die "board must be base or tractor, got '$BOARD'" ;; esac
case "$REVIVE_MODE" in reboot|full) ;; *) die "REVIVE_MODE must be reboot or full, got '$REVIVE_MODE'" ;; esac
[ -s "$IMG" ] || die "image not found or empty: $IMG"

# --- image sanity -----------------------------------------------------------
IMGNAME=$(basename "$IMG")
SIZE=$(wc -c < "$IMG" | tr -d ' ')
MD5=$(md5sum "$IMG" | cut -d' ' -f1)
# 192 KB flash, of which the last 8 KB is the CFG region (include/memory_map.h).
[ "$SIZE" -le $((184 * 1024)) ] || die "$IMGNAME is $SIZE B, larger than the 184 KB APP region"
case "$MD5" in
  0c1bb0a9573f813137f941dfa47177d0) KIND="bench build on both boards since 2026-09-15 (RS-12.15 v2)" ;;
  589c120323c2d5e7ef9f459d7a4ba42d) KIND="PRODUCTION build (main bb4a2071): refuses diag register writes, so -ForceFrfHz and channel_survey_sniff.py fail on it -- bench boards normally run the bench build" ;;
  e8ad842489d5acfc09f204c7807e4661) KIND="RS-12.10 bench build (pre-clock-authority A/B control)" ;;
  5a160e4a8c9296c7d2e49727bdfb8880) KIND="RS-12.15 v2 bench build as flown in legs R and T" ;;
  8c112e6f8b42109aab38a69f2b0aa147) KIND="first RS-12.10 build -- STATS-tail serializer bug, do not fly" ;;
  *) KIND="not a recorded build (see bench-evidence/RS_13_vector_scene_2026-09-26/firmware/README.md)" ;;
esac
echo "image: $IMGNAME  $SIZE B  md5 $MD5"
echo "       $KIND"

# --- LF-clean staging -------------------------------------------------------
PIPE="full_flash_pipeline.sh run_flash_l072.sh prep_bridge.sh revive_bridge.sh wdt_pet.sh
      stm32_an3155_flasher.py 07_assert_pa11_pf4_long.cfg 08_boot_user_app.cfg 99_release_and_reset.cfg"
WRAP="run_flash_bench.sh stamp.py kmsg_log.py"
STAGE=$(mktemp -d)
trap '[ -n "${STAGE:-}" ] && [ -d "$STAGE" ] && rm -rf -- "$STAGE"' EXIT
mkdir -p "$STAGE/p0c" "$STAGE/home"
for f in $PIPE; do tr -d '\r' < "$HELPER/$f" > "$STAGE/p0c/$f"; done
for f in $WRAP; do tr -d '\r' < "$HERE/$f" > "$STAGE/home/$f"; done
cp "$IMG" "$STAGE/p0c/$IMGNAME"   # binary: never line-ending-converted

PW=$BENCH_SUDO_PW
S="echo '$PW' | sudo -S -p ''"
ENVS="REVIVE_MODE=$REVIVE_MODE FLASH_VERIFY_ONLY=$VERIFY_ONLY BENCH_HOME=$BOARD_HOME"
[ "$PW" = "fio" ] || ENVS="$ENVS BENCH_SUDO_PW='$PW'"
RUN="bash $BOARD_HOME/run_flash_bench.sh /tmp/lifetrac_p0c/$IMGNAME"

winpath() { if command -v cygpath >/dev/null 2>&1; then cygpath -m "$1"; else printf '%s\n' "$1"; fi; }

if [ "$BOARD" = "base" ]; then
  command -v ssh >/dev/null && command -v scp >/dev/null || die "ssh/scp not found"
  SSHO=(-i "$BASE_SSH_KEY" -o BatchMode=yes -o ConnectTimeout=10)
  TGT="$BASE_USER@$BASE_HOST"
  echo "=== base $TGT (adb serial $BASE_SERIAL): staging"
  ssh "${SSHO[@]}" "$TGT" "$S mkdir -p /tmp/lifetrac_p0c && $S chmod 0777 /tmp/lifetrac_p0c"
  scp -q "${SSHO[@]}" "$STAGE"/p0c/* "$TGT:/tmp/lifetrac_p0c/"
  scp -q "${SSHO[@]}" "$STAGE"/home/* "$TGT:$BOARD_HOME/"
  echo "=== base: preflight"
  ssh "${SSHO[@]}" "$TGT" "FLASH_PREFLIGHT_ONLY=1 $ENVS $RUN"
  if [ "$GO" = "1" ]; then
    echo "=== base: FLASHING (radio-on event; REVIVE_MODE=$REVIVE_MODE)"
    set +e
    ssh "${SSHO[@]}" "$TGT" "$ENVS $RUN"
    RC=$?
    set -e
  fi
else
  command -v adb >/dev/null || die "adb not found"
  export MSYS_NO_PATHCONV=1
  ADB=(adb -s "$TRACTOR_SERIAL")
  echo "=== tractor $TRACTOR_SERIAL: staging"
  "${ADB[@]}" shell "$S mkdir -p /tmp/lifetrac_p0c; $S chmod 0777 /tmp/lifetrac_p0c" >/dev/null
  for f in "$STAGE"/p0c/*; do "${ADB[@]}" push "$(winpath "$f")" /tmp/lifetrac_p0c/ >/dev/null; done
  for f in "$STAGE"/home/*; do "${ADB[@]}" push "$(winpath "$f")" "$BOARD_HOME/" >/dev/null; done
  echo "=== tractor: preflight"
  "${ADB[@]}" shell "$S env FLASH_PREFLIGHT_ONLY=1 $ENVS $RUN" | tr -d '\r'
  if [ "$GO" = "1" ]; then
    echo "=== tractor: stopping lifetrac-camera.service + tractor-camera (they hold /dev/ttymxc3)"
    "${ADB[@]}" shell "$S systemctl stop lifetrac-camera.service; $S docker stop -t 3 tractor-camera" >/dev/null 2>&1 || true
    echo "=== tractor: FLASHING (radio-on event; REVIVE_MODE=$REVIVE_MODE)"
    set +e
    "${ADB[@]}" shell "$S env $ENVS $RUN" | tr -d '\r'
    RC=${PIPESTATUS[0]}
    set -e
  fi
fi

if [ "$GO" != "1" ]; then
  echo
  echo "Staged and preflighted only. To flash (operator GO required):"
  echo "  bash $0 $BOARD $IMG --go$([ "$VERIFY_ONLY" = 1 ] && echo ' --verify-only')"
  exit 0
fi

echo
echo "flash exit status: ${RC:-?} (a dropped connection, e.g. 255 over ssh, is expected when REVIVE_MODE=reboot reboots the board)"
cat <<EOF
Next (FLASH_RUNBOOK section 4):
  1. When the board is back: success = 'Verify OK' and 'flash_rc=0' in $BOARD_HOME/pipeline_stamped.log.
  2. /tmp was wiped: re-stage /tmp/lifetrac_strict (and /tmp/lifetrac_p0c before another flash).
  3. Tractor: stop lifetrac-camera.service and tractor-camera again; check 'fuser /dev/ttymxc3' is empty.
  4. rs116_health_probe.py on the board, then park the radio (radio_park.py -> PARK_OK 0x80)
     unless a leg follows under the same GO.
EOF
exit "${RC:-0}"
