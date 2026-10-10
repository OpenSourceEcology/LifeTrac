#!/bin/bash
# pull_board_state.sh <base|tractor> [--images] -- PC side (Git Bash). Runs the read-only
# capture_board_state.sh on one bench X8 over adb and pulls the result into the private
# archive OUTSIDE git (default C:/Users/dorkm/Documents/LifeTrac-bench-archive/<date>/).
# Copy only the reviewed text reports into bench-evidence/ (see BENCH_BOARDS.md).
#
#   --images   also stream the board's LifeTrac docker images to the archive (docker save,
#              nothing is written on the board). Large; never commit them.
#
# Nothing here opens /dev/ttymxc3, starts/stops a unit or container, or touches the watchdog.
# On the tractor, stop lifetrac-camera.service first (BENCH_RUNBOOK): it grabs the radio UART
# at boot. adb exec-out output starts with the login shell's terminal-size query
# (ESC 7 ESC[r ESC[999;999H ESC[6n, 19 bytes); strip_to_gzip removes it.
set -u
export MSYS_NO_PATHCONV=1
ROLE=${1:?base|tractor}; IMAGES=0; [ "${2:-}" = --images ] && IMAGES=1
_BT=$(dirname "$0")
[ -f "$_BT/bench.env" ] && eval "$(tr -d '\r' < "$_BT/bench.env")"   # BASE_SERIAL / TRACTOR_SERIAL / ARCHIVE_DIR
case $ROLE in base) SER=${BASE_SERIAL:-2D0A1209DABC240B};; tractor) SER=${TRACTOR_SERIAL:-2E2C1209DABC240B};; *) echo "base|tractor"; exit 2;; esac
HERE=$(cygpath -m "$(dirname "$0")")
ARC=${ARCHIVE:-${ARCHIVE_DIR:-$HOME/Documents/LifeTrac-bench-archive}}/board_state_$(date -u +%Y-%m-%d)
mkdir -p "$ARC/$ROLE" "$ARC/images"
strip_to_gzip() { py -3 -c "import sys;p=sys.argv[1];d=open(p,'rb').read();i=d.find(b'\x1f\x8b\x08');assert 0<=i<64,i;open(p,'wb').write(d[i:])" "$1"; }

tr -d '\r' < "$HERE/capture_board_state.sh" > "$ARC/$ROLE/capture_board_state.sh"     # CRLF trap
adb -s $SER push "$ARC/$ROLE/capture_board_state.sh" /tmp/capture_board_state.sh >/dev/null
adb -s $SER shell "sudo -n sh /tmp/capture_board_state.sh $ROLE" | tr -d '\r' | tee "$ARC/$ROLE/run.txt" | tail -2
TGZ=$(sed -n 's/^CAPTURE_TGZ=//p' "$ARC/$ROLE/run.txt")
[ -n "$TGZ" ] || { echo "capture failed"; exit 1; }
adb -s $SER shell "sudo -n chmod 0644 $TGZ" >/dev/null
adb -s $SER pull "$TGZ" "$ARC/$ROLE/" | tail -1
adb -s $SER exec-out "sudo -n tar czf - -C /var/rootdirs/opt --exclude=lifetrac/DESIGN-CONTROLLER/secrets --exclude=lifetrac/DESIGN-CONTROLLER/.env --exclude=lifetrac/bin/ffmpeg lifetrac 2>/dev/null" > "$ARC/$ROLE/opt_lifetrac_no_secrets.tgz" && strip_to_gzip "$ARC/$ROLE/opt_lifetrac_no_secrets.tgz"

if [ $IMAGES = 1 ]; then
  for ref in $(adb -s $SER exec-out "sudo -n docker images --format '{{.Repository}}:{{.Tag}}' | grep -E '^lifetrac'" | tr -d '\r' | sed 's/\x1b[^a-zA-Z]*[a-zA-Z]//g; s/\x1b[0-9]//g'); do
    id=$(adb -s $SER exec-out "sudo -n docker image inspect -f '{{.Id}}' $ref" | tr -d '\r' | grep -o 'sha256:[0-9a-f]*' | cut -c8-19)
    ls "$ARC/images/"*"_$id.tar.gz" >/dev/null 2>&1 && { echo "have image $id ($ref)"; continue; }   # one file per image id
    out="$ARC/images/${ROLE}_$(echo $ref | tr ':/' '__')_$id.tar.gz"
    adb -s $SER exec-out "sudo -n docker save $ref | gzip -1" > "$out" && strip_to_gzip "$out" && echo "saved $out"
  done
fi
echo "archive: $ARC/$ROLE"
