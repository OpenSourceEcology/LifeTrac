#!/bin/bash
# scene_check.sh <tag> [min_pct]  -- before every camera run: bring the bench YouTube
# window to the front (youtube_window.ps1 -Front), then grab frames from the tractor's
# /dev/video1 for ~10 s, report the % of pixels changed over 1.9 / 5 / 10 s, save the
# last frame as legs/<tag>_scene.jpg and the numbers as legs/<tag>_scene.txt.
# Exit 4 when the 10 s change is below min_pct (default 8 %): the video is paused,
# behind another window, on an ad card, or the camera is not aimed at it.
# Camera only: nothing here touches the radio.
set -u
TAG=${1:?tag}; MIN=${2:-8}
export MSYS_NO_PATHCONV=1
SP="C:/Users/dorkm/AppData/Local/Temp/claude/C--Users-dorkm-Documents-GitHub-LifeTrac/5eaec8c2-12ac-4272-80af-b19d1a563f48/scratchpad"
EW="C:/Users/dorkm/Documents/GitHub/LifeTrac/LifeTrac-v25/DESIGN-CONTROLLER/bench-evidence/RS_13_vector_scene_2026-09-26/legs"
S="echo fio | sudo -S -p ''"; T="adb -s 2E2C1209DABC240B shell"
powershell.exe -NoProfile -ExecutionPolicy Bypass -File "$SP/youtube_window.ps1" -Front | tr -d '\r'
out=$( { echo "# $TAG scene check, $(date -u +%Y-%m-%dT%H:%M:%SZ) PC clock (railroad video, normal Firefox window, player fullscreen)"
  $T "echo -n 'camera holders: '; $S docker ps --format '{{.Names}}' | grep -c camera_svc; $S docker run --rm --device=/dev/video1 -v /tmp:/tmp --entrypoint python3 lifetrac-tractor-x8:latest -c \"
import cv2, time, numpy as np
c = cv2.VideoCapture('/dev/video1', cv2.CAP_V4L2); fr = []
t0 = time.time()
while time.time() - t0 < 10.5:
    ok, f = c.read()
    if ok: fr.append((time.time(), f))
    time.sleep(0.05)
c.release()
for gap in (1.9, 5.0, 10.0):
    a = fr[5]; b = min(fr, key=lambda x: abs((x[0] - a[0]) - gap))
    d = np.abs(a[1].astype(np.int16) - b[1].astype(np.int16))
    print('%.1f s apart: mean abs diff %.2f  pct pixels diff>25: %.2f %%' % (b[0] - a[0], d.mean(), 100.0 * (d.max(axis=2) > 25).mean()))
print('mean luma %.1f' % fr[-1][1].mean())
cv2.imwrite('/tmp/scene_check.jpg', fr[-1][1])
\"" 2>&1 | tr -d '\r'; } )
echo "$out" | tee "$EW/${TAG}_scene.txt"
adb -s 2E2C1209DABC240B pull /tmp/scene_check.jpg "$EW/${TAG}_scene.jpg" >/dev/null 2>&1 && echo "frame -> legs/${TAG}_scene.jpg"
pct=$(echo "$out" | grep -E "^(9\.|10\.)[0-9] s apart" | grep -oE "[0-9.]+ %" | grep -oE "[0-9.]+")
if [ -z "$pct" ] || [ "$(py -3 -c "print(1 if float('${pct:-0}') >= $MIN else 0)")" != "1" ]; then
  echo "SCENE-CHECK-FAIL: 10 s change ${pct:-?} % < $MIN % (paused / behind / ad card / aim) - look at legs/${TAG}_scene.jpg"; exit 4
fi
echo "SCENE-CHECK-OK: 10 s change $pct %"
