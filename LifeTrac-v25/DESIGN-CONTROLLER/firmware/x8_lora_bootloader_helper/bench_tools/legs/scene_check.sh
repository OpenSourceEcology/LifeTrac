#!/usr/bin/env bash
# scene_check.sh -- before EVERY camera run (each step-1 pass, each leg): bring the
# bench video window to the front (youtube_window.ps1 -Front), grab frames from the
# tractor's /dev/video1 until 10.5 s after its sixth frame (the one the 1.9 / 5 / 10 s
# pairs are measured from; at most 20 s after the first frame), so a camera that is
# slow to start, or whose first frames trickle in, no longer eats the window
# (2026-10-15 preflight_moving reached only 7.2 s), report the share of pixels
# changed over 1.9 / 5 / 10 s, and save the last frame and the numbers as evidence.
# No first frame within 20 s of opening the camera, or fewer than 10 frames, fails
# with a NO-FRAMES line instead of a missing number.
#
# Usage:   bash scene_check.sh <tag> [min_pct] [--no-front]
#   <tag>       file tag: $EVIDENCE_DIR/<tag>_scene.{txt,jpg} (e.g. leg2a, step1_bw250_moving)
#   min_pct     exit 4 when the 10 s change is below it (default 8 = moving content;
#               pass 0 for a still scene, which only records -- RS13_VECTOR_LEG.md "Scene");
#               exit 4 too when the camera gave no 10 s figure (NO-FRAMES, or no pair
#               9 s or more apart)
#   --no-front  do not touch the PC window (e.g. the still is shown some other way)
#   Exit 5 (SCENE-CHECK-NOT-FRONT) before the camera is opened, with no frame taken,
#   when youtube_window.ps1 -Front fails: the bench window is not confirmed in front
#   after its retry (exit 6 there), or there is none (exit 2). The check is made once,
#   before the grab: a window the operator clicks after it still comes in front of
#   the camera. leg_prep.sh and step1_pass.sh abort on any non-zero exit of this script.
# Inputs:  TRACTOR_SERIAL, TRACTOR_APP_IMAGE (has OpenCV), YOUTUBE_URL /
#          FIREFOX_PROFILE_DIR (the window youtube_window.ps1 -Open started).
# Writes:  $EVIDENCE_DIR/<tag>_scene.txt and <tag>_scene.jpg; tractor /tmp/scene_check.jpg
#          (removed first, so a failed check never pulls an earlier run's frame).
# Boards:  tractor only: one `docker run --rm --device=/dev/video1` (camera node,
#          nothing else). /dev/video1 must be free: no camera_svc, lifetrac-camera
#          stopped. Exit 3 if the camera is busy.
# Radio:   camera only; never opens /dev/ttymxc3, never transmits.
# Screen:  never kiosk / never a fullscreen the operator cannot leave: the window
#          is a normal window, only the player is fullscreen (Esc leaves it).
# Origin:  bench-evidence/RS_13_vector_scene_2026-09-26/scripts/scene_check.sh
#          (historical copy, unchanged). New here: the window counted from the
#          reference (sixth) frame, the NO-FRAMES failures, the stale-frame removal.
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
E=$(bench_evidence_dir) || die "no evidence folder: $EVIDENCE_DIR (see above)"
board_present tractor || die "tractor ($TRACTOR_SERIAL) is not reachable via adb"

if [ $FRONT = 1 ]; then
  if command -v powershell.exe > /dev/null 2>&1; then
    powershell.exe -NoProfile -ExecutionPolicy Bypass -File "$(win_path "$LEGS_DIR/youtube_window.ps1")" -Front \
      -Url "$YOUTUBE_URL" -ProfileDir "$FIREFOX_PROFILE_DIR" 2>&1 | tr -d '\r'
    frc=${PIPESTATUS[0]}
    # No frame unless the bench window is confirmed in front: a frame of another window
    # would be saved here, and on a leg that screen would go on the air into the base
    # capture (2026-10-25). Any non-zero exit (6 = not in front, 2 = no window) stops here.
    if [ "$frc" != 0 ]; then
      echo "SCENE-CHECK-NOT-FRONT: youtube_window.ps1 -Front exit $frc: the bench video window is not confirmed in front, so no frame was taken. Click the video window (or run youtube_window.ps1 -Open), then re-run."
      exit 5
    fi
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
  board_sh tractor "$SUDO rm -f /tmp/scene_check.jpg; $SUDO docker run --rm --device=/dev/video1 -v /tmp:/tmp --entrypoint python3 $TRACTOR_APP_IMAGE -c \"
import cv2, time, numpy as np
c = cv2.VideoCapture('/dev/video1', cv2.CAP_V4L2); fr = []
t_open = time.time()
while not fr and time.time() - t_open < 20.0:
    ok, f = c.read()
    if ok: fr.append((time.time(), f))
    else: time.sleep(0.05)
if not fr:
    c.release()
    print('NO-FRAMES: no frame from /dev/video1 within 20 s of opening it')
    raise SystemExit(1)
t0 = fr[0][0]
while time.time() - fr[min(5, len(fr) - 1)][0] < 10.5 and time.time() - t0 < 20.0:
    ok, f = c.read()
    if ok: fr.append((time.time(), f))
    time.sleep(0.05)
c.release()
print('camera: first frame %.1f s after opening /dev/video1; %d frames, the sixth %.1f s and the last %.1f s after the first' % (t0 - t_open, len(fr), fr[min(5, len(fr) - 1)][0] - t0, fr[-1][0] - t0))
if len(fr) < 10:
    print('NO-FRAMES: only %d frames from /dev/video1' % len(fr))
    raise SystemExit(1)
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
if [ -z "$pct" ]; then
  why=$(printf '%s\n' "$out" | sed -n 's/^NO-FRAMES: //p' | head -1)
  echo "SCENE-CHECK-FAIL: no 10 s figure: ${why:-no frame pair 9 s or more apart (camera stalled, or the check itself failed)} - see ${TAG}_scene.txt"
  exit 4
fi
if ! awk -v p="$pct" -v m="$MIN" 'BEGIN { exit !(p + 0 >= m + 0) }'; then
  echo "SCENE-CHECK-FAIL: 10 s change $pct % < $MIN % (paused / behind another window / ad card / camera aim) - look at ${TAG}_scene.jpg"
  exit 4
fi
echo "SCENE-CHECK-OK: 10 s change $pct % (min $MIN %)"
