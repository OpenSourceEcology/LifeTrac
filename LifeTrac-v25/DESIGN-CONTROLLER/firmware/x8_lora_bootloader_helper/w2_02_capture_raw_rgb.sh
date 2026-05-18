#!/bin/sh
# W2-02 single-frame capture helper.
# Reads one MJPEG frame from /dev/video1 (Kurokesu C2), scales to 384x256
# rgb24 raw bytes, and writes them to /tmp/w2_02_frame.rgb (294 912 bytes).
# Caller passes the device + width + height + path via env vars.
set -e
DEV="${LT_VIDEO_DEV:-/dev/video1}"
W="${LT_FRAME_W:-384}"
H="${LT_FRAME_H:-256}"
OUT="${LT_FRAME_PATH:-/tmp/w2_02_frame.rgb}"
FFMPEG="${LT_FFMPEG:-/tmp/ffmpeg}"

if [ ! -x "$FFMPEG" ]; then
  echo "FATAL: ffmpeg missing at $FFMPEG" >&2
  exit 2
fi
if [ ! -e "$DEV" ]; then
  echo "FATAL: video device missing at $DEV" >&2
  exit 3
fi

# Capture as fio with sudo (mirrors W2-01) since /dev/video* is root-owned.
echo "W2-02 CAPTURE: dev=$DEV size=${W}x${H} out=$OUT"
echo fio | sudo -S "$FFMPEG" -hide_banner -loglevel error \
  -y -f v4l2 -input_format mjpeg -video_size 1920x1080 -i "$DEV" \
  -vframes 1 -vf "scale=${W}:${H}" -pix_fmt rgb24 \
  -f rawvideo "$OUT"

SZ=$(wc -c < "$OUT")
EXP=$((W * H * 3))
echo "W2-02 CAPTURE: wrote $SZ bytes (expected $EXP)"
if [ "$SZ" != "$EXP" ]; then
  echo "FATAL: raw RGB size mismatch" >&2
  exit 4
fi
echo "__W2_02_CAPTURE_OK__ path=$OUT bytes=$SZ wxh=${W}x${H}"
