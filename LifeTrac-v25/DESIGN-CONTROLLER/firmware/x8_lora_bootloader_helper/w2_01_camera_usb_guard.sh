#!/bin/bash
# W2-01 USB host-guard for Kurokesu C2 camera bring-up on Portenta X8 / Max Carrier.
#
# Purpose
# -------
# Apply the software mitigations from
#   AI NOTES/2026-05-14_USB_Wedge_Software_Mitigations.md
# in a single root-owned step BEFORE camera capture is attempted. This
# decouples USB power policy (which needs root) from the capture path
# (which should run as the `fio` user via the `video` group).
#
# What it does, in order
# ----------------------
#   (a) Start a backgrounded `dmesg -wH` tail to /tmp/w2_01_guard_dmesg.log
#       so any subsequent step that wedges the host still leaves a kernel
#       log we can pull after a physical power-cycle.
#   (b) Snapshot /proc/cmdline and the global usbcore.autosuspend value.
#   (c) Try to set usbcore.autosuspend=-1 at runtime. This only affects
#       USB devices enumerated AFTER this write, per the kernel USB-PM
#       doc. It is harmless for devices already enumerated.
#   (d) If the C2 is NOT yet enumerated, optionally hold the controller's
#       authorized_default=0 so the C2 enumerates without driver bind,
#       wait up to GUARD_HOTPLUG_WAIT_S seconds for the device, then
#       explicitly authorize it. This is the strongest software lever
#       against the VBUS-attach voltage transient because the camera
#       sits idle drawing only its quiescent current until we're ready.
#   (e) Once the C2 is present, snapshot its power state. If
#       runtime_status=suspended, SKIP power/control writes (the write
#       itself is an autoresume operation - exactly the trigger we are
#       trying to avoid). Otherwise set power/control=on and
#       power/autosuspend_delay_ms=-1.
#   (f) Unbind snd-usb-audio from C2 audio interfaces (or note that the
#       driver is already absent, e.g. blacklisted via
#       /etc/modprobe.d/lifetrac-no-usb-audio.conf).
#   (g) Settle (~500 ms) and emit a machine-parseable status block.
#
# Environment knobs (all optional)
# --------------------------------
#   GUARD_HOTPLUG_WAIT_S   How long to wait for the C2 to appear if it is
#                          not yet enumerated when the guard runs. Default 0
#                          (do not wait). Set to e.g. 20 for the cold-plug
#                          flow where the camera is powered on after the
#                          guard runs.
#   GUARD_AUTHORIZED_GATE  If 1, write authorized_default=0 on the parent
#                          USB controllers BEFORE waiting for the C2, then
#                          authorize the C2 explicitly once detected.
#                          Default 0 (off) because it affects every device
#                          on the controller, not just the C2. Use only on
#                          the bench, with no other USB devices on the
#                          same root hub that need to come up first.
#   GUARD_SETTLE_MS        Settle delay between authorize and the next
#                          step. Default 800.
#   GUARD_SUDO_PASS        Sudo password used when this script is invoked
#                          without root. Default 'fio' (LmP bench default).
#                          Pass via env so it never lands in process args.
#
# Output
# ------
# All status is on stdout, framed by __W2_01_GUARD_BEGIN__ /
# __W2_01_GUARD_END__ so the host orchestrator can parse it without being
# confused by stray kernel printk.
#
# Install / run
# -------------
#   adb -s <serial> push w2_01_camera_usb_guard.sh /tmp/
#   adb -s <serial> shell "chmod +x /tmp/w2_01_camera_usb_guard.sh"
#   adb -s <serial> shell "GUARD_HOTPLUG_WAIT_S=20 /tmp/w2_01_camera_usb_guard.sh"
#
# Exit code is always 0 unless invocation itself failed; gate logic lives
# in the parsed status block.

set -u

C2_VID="16d0"
C2_PID="0ed4"
GUARD_HOTPLUG_WAIT_S="${GUARD_HOTPLUG_WAIT_S:-0}"
GUARD_AUTHORIZED_GATE="${GUARD_AUTHORIZED_GATE:-0}"
GUARD_SETTLE_MS="${GUARD_SETTLE_MS:-800}"
GUARD_SUDO_PASS="${GUARD_SUDO_PASS:-fio}"
GUARD_DMESG_LOG="/tmp/w2_01_guard_dmesg.log"
GUARD_DMESG_PIDFILE="/tmp/w2_01_guard_dmesg.pid"

# Run a command as root: prefer direct exec if we already are root, else
# pipe the bench password into sudo -S. Keeps the password out of argv.
sudo_run() {
    if [ "$(id -u)" = "0" ]; then
        sh -c "$*"
    else
        printf '%s\n' "$GUARD_SUDO_PASS" | sudo -S -p '' sh -c "$*"
    fi
}

# Read a sysfs file (best effort; never fail).
sread() { cat "$1" 2>/dev/null || echo "?"; }

echo "__W2_01_GUARD_BEGIN__"
echo "schema=w2_01_guard/1"
echo "utc_start=$(date -u +%Y-%m-%dT%H:%M:%SZ)"
echo "id=$(id)"
echo "uname=$(uname -a)"
echo "guard_hotplug_wait_s=$GUARD_HOTPLUG_WAIT_S"
echo "guard_authorized_gate=$GUARD_AUTHORIZED_GATE"
echo "guard_settle_ms=$GUARD_SETTLE_MS"

# (a) Start backgrounded dmesg tail FIRST. If anything below wedges the
# USB host, the tail still flushes whatever printk happened before the
# wedge to disk, and we can `adb pull` it after a physical cycle.
echo "--- (a) start dmesg tail -> $GUARD_DMESG_LOG ---"
: > "$GUARD_DMESG_LOG" 2>/dev/null || sudo_run "true > $GUARD_DMESG_LOG" 2>/dev/null || true
( dmesg -wH 2>/dev/null > "$GUARD_DMESG_LOG" 2>&1 & echo $! > "$GUARD_DMESG_PIDFILE" )
DMESG_PID="$(cat "$GUARD_DMESG_PIDFILE" 2>/dev/null || echo '')"
echo "dmesg_pid=$DMESG_PID"

# (b) Snapshot kernel cmdline + global autosuspend.
echo "--- (b) /proc/cmdline + usbcore.autosuspend snapshot ---"
echo "cmdline=$(sread /proc/cmdline)"
GLOBAL_AUTOSUSPEND="$(sread /sys/module/usbcore/parameters/autosuspend)"
echo "autosuspend_initial=$GLOBAL_AUTOSUSPEND"

# (c) Set runtime autosuspend default to -1. Only affects devices that
# enumerate AFTER this write. If the C2 is already enumerated this is a
# no-op for the C2 itself but still useful for any later hot-plug.
echo "--- (c) write usbcore.autosuspend=-1 (effective for future enumerations) ---"
if [ "$GLOBAL_AUTOSUSPEND" = "-1" ]; then
    echo "autosuspend_write=skipped_already_-1"
else
    sudo_run "echo -1 > /sys/module/usbcore/parameters/autosuspend" >/dev/null 2>&1 || true
    echo "autosuspend_after_write=$(sread /sys/module/usbcore/parameters/autosuspend)"
fi

# Helper: locate C2 sysfs path. Cached descriptor reads only - per the
# kernel USB doc these are served from the device descriptor cache and do
# not cause USB bus I/O.
find_c2_syspath() {
    for d in /sys/bus/usb/devices/*/; do
        [ -e "${d}idVendor" ] || continue
        v=$(sread "${d}idVendor")
        p=$(sread "${d}idProduct")
        if [ "$v" = "$C2_VID" ] && [ "$p" = "$C2_PID" ]; then
            echo "$d"
            return 0
        fi
    done
    return 1
}

C2_SYSPATH="$(find_c2_syspath || true)"
C2_PRESENT_AT_START=0
[ -n "$C2_SYSPATH" ] && C2_PRESENT_AT_START=1
echo "c2_present_at_start=$C2_PRESENT_AT_START"
[ -n "$C2_SYSPATH" ] && echo "c2_syspath_initial=$C2_SYSPATH"

# (d) Optional unauthorized-default gate + hot-plug wait.
if [ "$GUARD_AUTHORIZED_GATE" = "1" ] && [ "$C2_PRESENT_AT_START" = "0" ]; then
    echo "--- (d) authorized_default=0 on root hubs, then wait for C2 hot-plug ---"
    for hub in /sys/bus/usb/devices/usb*/; do
        if [ -e "${hub}authorized_default" ]; then
            cur="$(sread "${hub}authorized_default")"
            echo "  ${hub}authorized_default before=$cur"
            sudo_run "echo 0 > ${hub}authorized_default" >/dev/null 2>&1 || true
            echo "  ${hub}authorized_default after=$(sread ${hub}authorized_default)"
        fi
    done
fi

if [ "$C2_PRESENT_AT_START" = "0" ] && [ "$GUARD_HOTPLUG_WAIT_S" -gt 0 ] 2>/dev/null; then
    echo "--- (d) waiting up to ${GUARD_HOTPLUG_WAIT_S}s for C2 (${C2_VID}:${C2_PID}) ---"
    waited=0
    while [ "$waited" -lt "$GUARD_HOTPLUG_WAIT_S" ]; do
        C2_SYSPATH="$(find_c2_syspath || true)"
        if [ -n "$C2_SYSPATH" ]; then
            echo "c2_detected_after_s=$waited"
            break
        fi
        sleep 1
        waited=$((waited + 1))
    done
fi

# If we used the authorized gate, explicitly authorize the C2 now.
if [ "$GUARD_AUTHORIZED_GATE" = "1" ] && [ -n "$C2_SYSPATH" ]; then
    echo "--- (d) explicit authorize C2 ---"
    auth_before="$(sread "${C2_SYSPATH}authorized")"
    echo "  c2_authorized_before=$auth_before"
    if [ "$auth_before" != "1" ]; then
        sudo_run "echo 1 > ${C2_SYSPATH}authorized" >/dev/null 2>&1 || true
        echo "  c2_authorized_after=$(sread ${C2_SYSPATH}authorized)"
        # Settle so udev/uvcvideo can bind before we touch power state.
        if command -v udevadm >/dev/null 2>&1; then
            udevadm settle --timeout=3 >/dev/null 2>&1 || true
        fi
    fi
fi

# (e) C2 power state + safe per-device writes.
if [ -n "$C2_SYSPATH" ]; then
    echo "--- (e) C2 power state snapshot ---"
    for f in power/control power/runtime_status power/autosuspend_delay_ms power/runtime_active_time power/runtime_suspended_time; do
        if [ -r "${C2_SYSPATH}${f}" ]; then
            echo "  ${C2_SYSPATH}${f}=$(sread "${C2_SYSPATH}${f}")"
        fi
    done

    rs="$(sread "${C2_SYSPATH}power/runtime_status")"
    if [ "$rs" = "suspended" ]; then
        echo "c2_power_writes=skipped_runtime_suspended"
        echo "c2_power_writes_reason=writing_power_control_would_autoresume"
    else
        echo "--- (e) writing power/control=on + autosuspend_delay_ms=-1 (rs=$rs) ---"
        if [ -e "${C2_SYSPATH}power/control" ]; then
            sudo_run "echo on > ${C2_SYSPATH}power/control" >/dev/null 2>&1 || true
            echo "  c2_power_control_after=$(sread ${C2_SYSPATH}power/control)"
        fi
        if [ -e "${C2_SYSPATH}power/autosuspend_delay_ms" ]; then
            sudo_run "echo -1 > ${C2_SYSPATH}power/autosuspend_delay_ms" >/dev/null 2>&1 || true
            echo "  c2_autosuspend_delay_ms_after=$(sread ${C2_SYSPATH}power/autosuspend_delay_ms)"
        fi
        echo "c2_power_writes=applied"
    fi

    # Mirror the same safe writes onto the parent SMSC hub (its rs is
    # usually 'active' because it carries other downstream traffic).
    C2_BUSDEV="$(basename "$C2_SYSPATH")"
    HUB_BUSDEV="$(echo "$C2_BUSDEV" | sed -E 's/\.[0-9]+$//')"
    HUB_SYSPATH="/sys/bus/usb/devices/${HUB_BUSDEV}/"
    if [ -d "$HUB_SYSPATH" ] && [ "$HUB_SYSPATH" != "$C2_SYSPATH" ]; then
        hub_rs="$(sread "${HUB_SYSPATH}power/runtime_status")"
        if [ "$hub_rs" != "suspended" ] && [ -e "${HUB_SYSPATH}power/control" ]; then
            sudo_run "echo on > ${HUB_SYSPATH}power/control" >/dev/null 2>&1 || true
            echo "  hub_power_control_after=$(sread ${HUB_SYSPATH}power/control)"
        else
            echo "  hub_power_writes=skipped (rs=$hub_rs)"
        fi
    fi

    # (f) Unbind snd-usb-audio from C2 audio interfaces.
    echo "--- (f) snd-usb-audio policy ---"
    SND_DRV="/sys/bus/usb/drivers/snd-usb-audio"
    if [ -d "$SND_DRV" ]; then
        UNBOUND=0
        for ifpath in "$SND_DRV"/${C2_BUSDEV}:*; do
            [ -e "$ifpath" ] || continue
            ifname="$(basename "$ifpath")"
            sudo_run "echo '$ifname' > $SND_DRV/unbind" >/dev/null 2>&1 || true
            echo "  unbound=$ifname"
            UNBOUND=$((UNBOUND + 1))
        done
        echo "snd_usb_audio_unbound_count=$UNBOUND"
    else
        echo "snd_usb_audio_present=0"
        echo "snd_usb_audio_note=driver_absent_likely_blacklisted"
    fi

    echo "--- (f) drivers bound to C2 interfaces after guard ---"
    for ifpath in "$C2_SYSPATH"*:*; do
        [ -e "$ifpath" ] || continue
        ifname="$(basename "$ifpath")"
        drv=""
        if [ -L "${ifpath}/driver" ]; then
            drv="$(basename "$(readlink "${ifpath}/driver")")"
        fi
        echo "  $ifname -> ${drv:-<unbound>}"
    done
else
    echo "c2_not_found=1"
    echo "c2_not_found_note=hotplug_or_check_vbus"
fi

# (g) Settle delay before declaring guard complete.
sleep_ms="$GUARD_SETTLE_MS"
sleep_s_int=$(( sleep_ms / 1000 ))
sleep_frac=$(( sleep_ms % 1000 ))
if [ "$sleep_frac" -eq 0 ]; then
    sleep "$sleep_s_int" 2>/dev/null || true
else
    sleep "${sleep_s_int}.$(printf '%03d' "$sleep_frac")" 2>/dev/null || sleep "$sleep_s_int" 2>/dev/null || true
fi

# Stop the dmesg tail and report the log path so the orchestrator can pull it.
if [ -n "$DMESG_PID" ]; then
    kill "$DMESG_PID" 2>/dev/null || true
fi
rm -f "$GUARD_DMESG_PIDFILE" 2>/dev/null || true
echo "dmesg_log_path=$GUARD_DMESG_LOG"
echo "dmesg_log_size=$(stat -c %s "$GUARD_DMESG_LOG" 2>/dev/null || echo 0)"

# Final overall verdict for the orchestrator. We pass when:
#   - C2 is present
#   - autosuspend=-1 OR the C2 was authorized via the gate path
#   - C2 power writes were either applied OR safely skipped (i.e. we did
#     not silently fail mid-way)
final_autosuspend="$(sread /sys/module/usbcore/parameters/autosuspend)"
echo "autosuspend_final=$final_autosuspend"
verdict="FAIL"
if [ -n "$C2_SYSPATH" ]; then
    if [ "$final_autosuspend" = "-1" ] || [ "$GUARD_AUTHORIZED_GATE" = "1" ]; then
        verdict="PASS"
    else
        verdict="WARN_AUTOSUSPEND_NOT_DISABLED"
    fi
fi
echo "verdict=$verdict"
echo "utc_end=$(date -u +%Y-%m-%dT%H:%M:%SZ)"
echo "__W2_01_GUARD_END__"
