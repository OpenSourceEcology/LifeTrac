#!/usr/bin/env bash
# scene_check.sh -- before EVERY camera run (each step-1 pass, each leg): bring the
# bench video window to the front (youtube_window.ps1 -Front), grab frames from the
# tractor's /dev/video1 for ~10 s, report the share of pixels changed over
# 1.9 / 5 / 10 s, and save the last frame and the numbers as evidence.
#
# Usage:   bash scene_check.sh <tag> [min_pct] [--no-front]
#   <tag>       file tag: $EVIDENCE_DIR/<tag>_scene.{txt,jpg} (e.g. leg2a, step1_bw250_moving)
#   min_pct     exit 4 when the 10 s change is below it (default 8 = moving content;
#               pass 0 for a still scene, which only records -- RS13_VECTOR_LEG.md "Scene")
#   --no-front  do not touch the PC window (e.g. the still is shown some other way)
# Inputs:  TRACTOR_SERIAL, TRACTOR_APP_IMAGE (has OpenCV), YOUTUBE_URL /
#          FIREFOX_PROFILE_DIR (the window youtube_window.ps1 -Open started).
# Writes:  $EVIDENCE_DIR/<tag>_scene.txt and <tag>_scene.jpg; tractor /tmp/scene_check.jpg.
# Boards:  tractor only: one `docker run --rm --device=/dev/video1` (camera node,
#          nothing else). /dev/video1 must be free: no camera_svc, lifetrac-camera
#          stopped. Exit 3 if the camera is busy.
# Radio:   camera only; never opens /dev/ttymxc3, never transmits.
# Screen:  never kiosk / never a fullscreen the operator cannot leave: the window
#          is a normal window, only the player is fullscreen (Esc leaves it).
# Origin:  bench-evidence/RS_13_vector_scene_2026-09-26/scripts/scene_check.sh
#          (historical copy, unchanged).
set -u
. "$(dirname "${BASH_SOURCE[0]}")/../lib/bench_env.sh" || exit 1

TAG=""; MIN=8; FRONT=1
for a in "$@"; do
  case $a in
    --no-front) FRONT=0 ;;
    -h|--help) bench_usage; exit 0 ;;
    *) if [ -z "$TAG" ]; then TAG=$a; else MIN=$a; fi ;;
  esac
done
[ -n "$TAG" ] || { bench_usage; exit 2; }
bench_check_name "tag" "$TAG"
case $MIN in ''|*[!0-9.]*) die "min_pct must be a number, not '$MIN'" ;; esac
E=$(bench_evidence_dir) || die "cannot create $EVIDENCE_DIR"
board_present tractor || die "tractor ($TRACTOR_SERIAL) is not reachable via adb"

if [ $FRONT = 1 ]; then
  if command -v powershell.exe > /dev/null 2>&1; then
    powershell.exe -NoProfile -ExecutionPolicy Bypass -File "$(win_path "$LEGS_DIR/youtube_window.ps1")" -Front \
      -Url "$YOUTUBE_URL" -ProfileDir "$FIREFOX_PROFILE_DIR" 2>&1 | tr -d '\r'
  else
    echo "(no powershell.exe: bring the bench video window to the front by hand)"
  fi
fi

holders=$(board_sh tractor "$SUDO fuser /dev/video1 2>/dev/null" | tr -s ' \n' ' ' | sed 's/^ //; s/ $//')
if [ -n "$holders" ]; then
  echo "SCENE-CHECK-BUSY: /dev/video1 is held by pid(s) $holders on the tractor (camera_svc running? lifetrac-camera not stopped?)"
  exit 3
fi

out=$( { echo "# $TAG scene check, $(date -u +%Y-%m-%dT%H:%M:%SZ) PC clock (content: $YOUTUBE_URL, normal Firefox window, player fullscreen; min $MIN %)"
  board_sh tractor "$SUDO docker run --rm --device=/dev/video1 -v /tmp:/tmp --entrypoint python3 $TRACTOR_APP_IMAGE -c \"
import cv2, time, numpy as np
c = cv2.VideoCapture('/dev/video1', cv2.CAP_V4L2); fr = []
t0 = time.time()
while time.time() - t0 < 10.5:
    ok, f = c.read()
    if ok: fr.append((time.time(), f))
    time.sleep(0.05)
c.release()
if len(fr) < 10:
    raise SystemExit('only %d frames from /dev/video1' % len(fr))
for gap in (1.9, 5.0, 10.0):
    a = fr[5]; b = min(fr, key=lambda x: abs((x[0] - a[0]) - gap))
    d = np.abs(a[1].astype(np.int16) - b[1].astype(np.int16))
    print('%.1f s apart: mean abs diff %.2f  pct pixels diff>25: %.2f %%' % (b[0] - a[0], d.mean(), 100.0 * (d.max(axis=2) > 25).mean()))
print('mean luma %.1f' % fr[-1][1].mean())
cv2.imwrite('/tmp/scene_check.jpg', fr[-1][1])
\"" 2>&1; } )
printf '%s\n' "$out" | tee "$E/${TAG}_scene.txt"
board_pull tractor /tmp/scene_check.jpg "$E/${TAG}_scene.jpg" 2>/dev/null && echo "frame -> $E/${TAG}_scene.jpg"
pct=$(printf '%s\n' "$out" | grep -E "^(9\.|10\.)[0-9] s apart" | grep -oE "[0-9.]+ %" | grep -oE "[0-9.]+" | head -1)
if [ -z "$pct" ] || ! awk -v p="$pct" -v m="$MIN" 'BEGIN { exit !(p + 0 >= m + 0) }'; then
  echo "SCENE-CHECK-FAIL: 10 s change ${pct:-?} % < $MIN % (paused / behind another window / ad card / camera aim) - look at ${TAG}_scene.jpg"
  exit 4
fi
echo "SCENE-CHECK-OK: 10 s change $pct % (min $MIN %)"
