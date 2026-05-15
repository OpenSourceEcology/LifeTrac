#!/bin/bash
# W2-01 X8-side camera first-light probe.
#
# Reports the USB tree + every /dev/video* node + driver/dmesg state, then
# invokes w2_01_camera_capture.py against each /dev/videoN until one returns
# verdict=PASS (or we exhaust the list). All output is on stdout, framed by
# __W2_01_PROBE_BEGIN__ / __W2_01_PROBE_END__ so the host orchestrator can
# extract it cleanly even if there are stray kernel messages.
#
# IMPORTANT - permission model:
#   /dev/video* are root:video 0660 + ACL on LmP. The default `fio` user is
#   NOT in the `video` group, so direct open() fails EACCES. The supported
#   one-time fix is:
#       sudo usermod -aG video fio && (logout/login)
#   After that this script runs without sudo.
#
#   If `--use-sudo 1` is passed AND user is not in video group, we attempt
#   sudo -S with the bench-default password 'fio'. WARNING: in earlier bench
#   runs a sudo-driven Python streaming session left the X8 USB-host wedged
#   after a long-running stream, requiring a physical power-cycle. Sudo is
#   therefore opt-in; prefer the usermod fix.
#
# Usage (from host):
#   adb -s 2D0A1209DABC240B exec-out bash /tmp/w2_01_camera_first_light.sh \
#       /tmp/w2_01_camera_capture.py /tmp/w2_01_snap.jpg 1280 720 [use_sudo] [per_attempt_timeout_s]

set -u

CAPTURE_PY="${1:-/tmp/w2_01_camera_capture.py}"
OUT_PATH="${2:-/tmp/w2_01_snap.jpg}"
WIDTH="${3:-1280}"
HEIGHT="${4:-720}"
USE_SUDO="${5:-0}"
PER_ATTEMPT_TIMEOUT_S="${6:-8}"
ENUMERATE_ONLY="${7:-1}"   # 1 = SAFE: only QUERYCAP+ENUM_FMT+ENUM_FRAMESIZES (no STREAMON)
APPLY_QUIRKS="${8:-1}"     # 1 = reload uvcvideo with FIX_BANDWIDTH (quirks=0x80) + nodrop=1
echo "__W2_01_PROBE_BEGIN__"
echo "schema=w2_01_probe/1"
echo "utc_start=$(date -u +%Y-%m-%dT%H:%M:%SZ)"
echo "hostname=$(hostname)"
echo "uname=$(uname -a)"
echo "id=$(id)"
echo "capture_py=$CAPTURE_PY"
echo "out_path=$OUT_PATH"

echo "--- lsusb ---"
lsusb 2>&1 || echo "lsusb_missing"

echo "--- /dev/video* listing ---"
ls -la /dev/video* 2>&1 || echo "no_video_nodes"

echo "--- /dev/media* listing ---"
ls -la /dev/media* 2>&1 || true

echo "--- /dev/v4l/by-id (if present) ---"
ls -la /dev/v4l/by-id/ 2>&1 || echo "no_by_id"

echo "--- modules with uvc/v4l in name ---"
lsmod 2>/dev/null | grep -iE "uvc|videobuf|v4l" || echo "no_v4l_modules"

echo "--- last 30 kmsgs mentioning uvc/usb/video ---"
dmesg 2>/dev/null | grep -iE "uvc|usb 1-|video|camera" | tail -30 || echo "dmesg_unavailable_or_empty"

# Iterate /dev/videoN and try capture on each. Keep the first PASS.
shopt -s nullglob
NODES=( /dev/video* )
shopt -u nullglob
echo "node_count=${#NODES[@]}"
echo "enumerate_only=$ENUMERATE_ONLY"

# In enumerate-only mode we still iterate every node so we get a full report,
# but the python script will skip REQBUFS/STREAMON entirely. In capture mode
# we skip /dev/video0 (the i.MX ISP) because it has zero ENUM_FMT entries
# and previously contributed to the wedge surface.

# Check if we can open a video node directly; if EACCES, optionally escalate
# via sudo (only when --use-sudo 1 was passed, since sudo+ctypes streaming
# previously wedged the USB host on this image - see header note).
NEEDS_SUDO=0
if [ ${#NODES[@]} -gt 0 ]; then
  if ! python3 -c "import os,sys; sys.exit(0 if os.access(sys.argv[1], os.R_OK|os.W_OK) else 1)" "${NODES[0]}"; then
    NEEDS_SUDO=1
  fi
fi
echo "needs_sudo=$NEEDS_SUDO"
echo "use_sudo_flag=$USE_SUDO"
echo "per_attempt_timeout_s=$PER_ATTEMPT_TIMEOUT_S"

if [ "$NEEDS_SUDO" = "1" ] && [ "$USE_SUDO" != "1" ]; then
  echo "ABORTING: fio user lacks /dev/video* access. Run on the X8 ONCE:"
  echo "  echo fio | sudo -S usermod -aG video fio && exit"
  echo "then reconnect and re-run. Or pass --use-sudo to this orchestrator."
  echo "winning_node="
  echo "overall_verdict=FAIL_NEEDS_VIDEO_GROUP"
  echo "utc_end=$(date -u +%Y-%m-%dT%H:%M:%SZ)"
  echo "__W2_01_PROBE_END__"
  exit 0
fi

# Wedge-prevention pre-flight (see AI NOTES/2026-05-12_W2-01_USB_Wedge_Avoidance_Research_Copilot_v1_0.md):
#
# REORDERED 2026-05-12 after wedge #4 (which fired during step (a) and left
# us with no dmesg log because the old order put the tail capture last).
# New order:
#  (a) Start backgrounded `dmesg -wH` tail FIRST so any subsequent step that
#      wedges the host still leaves us a kernel log to inspect after the
#      physical PWR-USB-C unplug.
#  (b) Snapshot /proc/cmdline + global usbcore.autosuspend; this gates whether
#      the rest of the pre-flight is safe to run.
#  (c) GATE: if usbcore.autosuspend != -1 (i.e. Option A from the avoidance
#      doc has not been applied at boot), SKIP the per-device power/control
#      writes and snd-usb-audio unbind. Per the kernel USB-PM doc, writing
#      `power/control=on` to a device whose runtime_status is `suspended` is
#      itself an autoresume operation - the very trigger we are trying to
#      avoid. Without Option A there is no safe time to do these writes.
#  (d) Identify the C2 sysfs path. We still walk /sys/bus/usb/devices because
#      cached descriptor reads (idVendor/idProduct) are documented as not
#      causing USB bus I/O, but we keep this BELOW the autosuspend gate so it
#      only runs once Option A guarantees the device cannot be suspended.
#  (e) Snapshot C2 power state (read-only).
#  (f) For each candidate write target, re-check runtime_status and skip if
#      `suspended`. Only write `power/control=on` to devices already `active`.
#  (g) Unbind snd-usb-audio from C2 audio interfaces (only after gate).
#
# The previous "uvcvideo quirks=0x80" / modprobe-time conf approach is removed:
# the `cannot get freq at ep 0x84` printk was misattributed to uvcvideo - it
# actually originates from snd-usb-audio (kernel.org bug #212771) and is
# benign. FIX_BANDWIDTH only affects uncompressed YUV streams (the C2's
# preferred format is MJPEG = compressed) so the quirk is a no-op for us.
#
# 2026-05-15 REFACTOR: The privileged USB policy work (autosuspend,
# power/control, snd-usb-audio unbind, dmesg tail) is now owned by the
# separate guard script `w2_01_camera_usb_guard.sh`. The orchestrator runs
# the guard FIRST as root, then this script runs in capture mode (ideally
# unprivileged via the `video` group). This block is now a read-only
# VERIFICATION step only - it never writes to sysfs or invokes sudo. That
# keeps capture decoupled from `--use-sudo` and removes the silent-skip
# failure mode where unprivileged capture lost all USB defenses.
PRE_FLIGHT_LOG="/tmp/w2_01_dmesg_during_probe.log"
C2_VID="16d0"
C2_PID="0ed4"
if [ "$APPLY_QUIRKS" = "1" ]; then
  echo "--- pre-flight (a) start backgrounded dmesg tail -> $PRE_FLIGHT_LOG ---"
  : > "$PRE_FLIGHT_LOG" 2>/dev/null || true
  ( dmesg -wH 2>/dev/null > "$PRE_FLIGHT_LOG" 2>&1 & echo $! > /tmp/w2_01_dmesg_pid )
  DMESG_PID="$(cat /tmp/w2_01_dmesg_pid 2>/dev/null || echo '')"
  echo "  dmesg_tail_pid=$DMESG_PID  (log=$PRE_FLIGHT_LOG)"

  echo "--- pre-flight (b) /proc/cmdline + usbcore.autosuspend (read-only) ---"
  cat /proc/cmdline 2>&1 || true
  GLOBAL_AUTOSUSPEND="$(cat /sys/module/usbcore/parameters/autosuspend 2>/dev/null || echo '?')"
  echo "  usbcore.autosuspend=${GLOBAL_AUTOSUSPEND}"

  # Read-only guard verification. We do NOT write to sysfs from here -
  # that's the guard script's job. We only report what's already in place
  # so summary.json can show whether the orchestrator forgot to run guard.
  echo "--- pre-flight (c) read-only guard verification ---"
  if [ "$GLOBAL_AUTOSUSPEND" = "-1" ]; then
    echo "  guard_autosuspend_ok=1"
  else
    echo "  guard_autosuspend_ok=0"
    echo "  guard_autosuspend_note=run_w2_01_camera_usb_guard.sh_first"
  fi

  C2_SYSPATH=""
  for d in /sys/bus/usb/devices/*/; do
    [ -e "${d}idVendor" ] || continue
    v=$(cat "${d}idVendor" 2>/dev/null)
    p=$(cat "${d}idProduct" 2>/dev/null)
    if [ "$v" = "$C2_VID" ] && [ "$p" = "$C2_PID" ]; then
      C2_SYSPATH="$d"
      break
    fi
  done
  echo "c2_syspath=${C2_SYSPATH:-NOT_FOUND}"

  if [ -n "$C2_SYSPATH" ]; then
    C2_BUSDEV="$(basename "$C2_SYSPATH")"
    HUB_BUSDEV="$(echo "$C2_BUSDEV" | sed -E 's/\.[0-9]+$//')"
    HUB_SYSPATH="/sys/bus/usb/devices/${HUB_BUSDEV}/"

    echo "--- pre-flight (e) snapshot C2 + hub power state (read-only) ---"
    for path in "$C2_SYSPATH" "$HUB_SYSPATH"; do
      for f in power/control power/runtime_status power/autosuspend_delay_ms power/runtime_active_time power/runtime_suspended_time; do
        if [ -r "${path}${f}" ]; then
          echo "  ${path}${f}=$(cat "${path}${f}" 2>/dev/null)"
        fi
      done
    done

    # Read-only: report whether snd-usb-audio is still bound to any C2
    # interface. We do NOT unbind here - the guard or the modprobe
    # blacklist owns that.
    echo "--- pre-flight (g) snd-usb-audio bind state (read-only) ---"
    SND_DRV="/sys/bus/usb/drivers/snd-usb-audio"
    SND_BOUND=0
    if [ -d "$SND_DRV" ]; then
      for ifpath in "$SND_DRV"/${C2_BUSDEV}:*; do
        [ -e "$ifpath" ] || continue
        SND_BOUND=$((SND_BOUND + 1))
        echo "  bound=$(basename "$ifpath")"
      done
    fi
    echo "  snd_usb_audio_bound_to_c2=$SND_BOUND"
    if [ "$SND_BOUND" -gt 0 ]; then
      echo "  snd_usb_audio_note=run_guard_or_install_lifetrac-no-usb-audio.conf"
    fi
    echo "  remaining_drivers_bound_to_c2:"
    for ifpath in "$C2_SYSPATH"*:*; do
      [ -e "$ifpath" ] || continue
      ifname="$(basename "$ifpath")"
      drv=""
      if [ -L "${ifpath}driver" ]; then
        drv="$(basename "$(readlink "${ifpath}driver")")"
      fi
      echo "    $ifname -> ${drv:-<unbound>}"
    done
  fi

  sleep 1
  if command -v udevadm >/dev/null 2>&1; then
    udevadm settle --timeout=3 2>&1 || true
  fi
fi

OVERALL_VERDICT="FAIL_NO_PASS"
WINNING_NODE=""
ENUM_ARG=""
if [ "$ENUMERATE_ONLY" = "1" ]; then
  ENUM_ARG="--enumerate-only"
fi
for node in "${NODES[@]}"; do
  # Skip i.MX ISP (/dev/video0) in capture mode - it has no formats and
  # contributes to the USB-host wedge surface.
  if [ "$ENUMERATE_ONLY" != "1" ] && [ "$node" = "/dev/video0" ]; then
    echo "--- skipping $node (i.MX ISP, no UVC formats; capture mode skips it) ---"
    continue
  fi
  echo "--- attempt capture on $node (enumerate_only=$ENUMERATE_ONLY) ---"
  # Always wrap in `timeout` so a wedged ctypes streaming call cannot brick
  # the USB-host stack indefinitely. The timeout binary is part of coreutils
  # and is present on stock LmP.
  if [ "$NEEDS_SUDO" = "1" ]; then
    echo fio | timeout --signal=KILL "${PER_ATTEMPT_TIMEOUT_S}" sudo -S -p '' \
      python3 "$CAPTURE_PY" --device "$node" --width "$WIDTH" --height "$HEIGHT" --out "$OUT_PATH" $ENUM_ARG 2>&1
  else
    timeout --signal=KILL "${PER_ATTEMPT_TIMEOUT_S}" \
      python3 "$CAPTURE_PY" --device "$node" --width "$WIDTH" --height "$HEIGHT" --out "$OUT_PATH" $ENUM_ARG 2>&1
  fi
  rc=$?
  if [ "$NEEDS_SUDO" = "1" ] && [ -e "$OUT_PATH" ]; then
    echo fio | sudo -S -p '' chmod 644 "$OUT_PATH" 2>/dev/null
  fi
  echo "node=$node rc=$rc"
  if [ $rc -eq 0 ]; then
    OVERALL_VERDICT="PASS"
    WINNING_NODE="$node"
    if [ "$ENUMERATE_ONLY" = "1" ]; then
      # In enumerate-only mode keep going so we get every node's capabilities.
      continue
    fi
    break
  fi
done

echo "--- output file stat ---"
ls -la "$OUT_PATH" 2>&1 || echo "no_output_file"

# Stop background dmesg tail (if we started one) and report log path so the
# orchestrator can adb pull it.
if [ -f /tmp/w2_01_dmesg_pid ]; then
  DMESG_PID="$(cat /tmp/w2_01_dmesg_pid 2>/dev/null || echo '')"
  if [ -n "$DMESG_PID" ]; then
    kill "$DMESG_PID" 2>/dev/null || true
  fi
  rm -f /tmp/w2_01_dmesg_pid 2>/dev/null || true
fi
if [ -f "$PRE_FLIGHT_LOG" ]; then
  echo "dmesg_log_path=$PRE_FLIGHT_LOG"
  echo "dmesg_log_size=$(stat -c %s "$PRE_FLIGHT_LOG" 2>/dev/null || echo 0)"
  echo "--- last 40 dmesg lines captured during probe ---"
  tail -40 "$PRE_FLIGHT_LOG" 2>&1 || true
fi

echo "winning_node=$WINNING_NODE"
echo "overall_verdict=$OVERALL_VERDICT"
echo "utc_end=$(date -u +%Y-%m-%dT%H:%M:%SZ)"
echo "__W2_01_PROBE_END__"
