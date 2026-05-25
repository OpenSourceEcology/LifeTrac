#!/bin/bash
DEV=${1:-/dev/video1}
echo "=== device: $DEV ==="
echo fio | sudo -S -p '' docker run --rm --device=$DEV -v /tmp/lifetrac_strict:/work -w /work --entrypoint /work/ffmpeg hub.foundries.io/arduino/arduino-ootb-python-devel:738bc44 -f v4l2 -list_formats all -i $DEV 2>&1 | grep -E "Raw|Compressed|fps|MJPG|YUYV|H264|video size" || true
echo "=== capture default frame ==="
echo fio | sudo -S -p '' rm -f /tmp/lifetrac_strict/probe_frame.jpg
echo fio | sudo -S -p '' docker run --rm --device=$DEV -v /tmp/lifetrac_strict:/work -w /work --entrypoint /work/ffmpeg hub.foundries.io/arduino/arduino-ootb-python-devel:738bc44 -y -f v4l2 -i $DEV -frames:v 1 /work/probe_frame.jpg 2>&1 | tail -15
echo "=== file ==="
ls -la /tmp/lifetrac_strict/probe_frame.jpg 2>&1
