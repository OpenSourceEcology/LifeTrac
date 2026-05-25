#!/bin/sh
set -e

DEV="$1"
if [ -z "$DEV" ]; then
  echo "missing device arg"
  exit 2
fi

if ! command -v v4l2-ctl >/dev/null 2>&1; then
  if command -v apk >/dev/null 2>&1; then
    apk add --no-cache v4l-utils >/dev/null 2>&1 || true
  fi
  if ! command -v v4l2-ctl >/dev/null 2>&1 && command -v apt-get >/dev/null 2>&1; then
    apt-get update >/dev/null 2>&1 || true
    apt-get install -y v4l-utils >/dev/null 2>&1 || true
  fi
fi

if ! command -v v4l2-ctl >/dev/null 2>&1; then
  echo "V4L2CTL_MISSING"
  exit 3
fi

echo "DEVICE=$DEV"
v4l2-ctl -d "$DEV" --all || true
echo "__CTRL_LIST__"
v4l2-ctl -d "$DEV" --list-ctrls-menus || true

if v4l2-ctl -d "$DEV" --list-ctrls-menus 2>/dev/null | grep -Eiq 'test.pattern|test_pattern|test-pattern'; then
  echo "TEST_PATTERN_LIKE_CONTROL_FOUND"
  v4l2-ctl -d "$DEV" --set-ctrl=test_pattern=0 || true
  echo "TEST_PATTERN_SET_ATTEMPTED"
else
  echo "NO_TEST_PATTERN_CONTROL"
fi
