#!/bin/bash
# W2-01 Section 10.3.V3 reproducibility stress probe.
#
# Goal: with deliverables B+C+D+E in place (modprobe blacklist, udev
# rules, lifetrac-camera.service, and the guard-gate), repeatedly
# exercise the historical wedge trigger pattern on the C2 USB camera
# while monitoring dmesg for ci_hdrc / USB descriptor errors. The
# outcome gates whether deliverable F (OSTREE_KERNEL_ARGS
# usbcore.autosuspend=-1) actually needs to ship.
#
# Trigger pattern per session evidence:
#   1. Cached sysfs descriptor reads on the C2 device path.
#   2. Brief V4L2 device open of /dev/lifetrac-c2 (open + immediate
#      close, no streaming ioctls). This invokes uvc_v4l2_open which is
#      where the historical autoresume race surfaced.
#
# Fail-fast: if dmesg starts emitting USB host errors we stop the loop
# so the bench operator can pull artifacts before the host wedges fully.
#
# Exit code is always 0 (gate is in the verdict line) so the
# orchestrator can collect artifacts even on failure.
set -u

ITERATIONS="${V3_ITERATIONS:-50}"
SLEEP_MS="${V3_SLEEP_MS:-200}"
DEVICE="${V3_DEVICE:-/dev/lifetrac-c2}"
LOG_DIR="${V3_LOG_DIR:-/tmp/w2_01_v3_stress}"
mkdir -p "$LOG_DIR" 2>/dev/null || true

dmesg_pre="$LOG_DIR/dmesg_pre.log"
dmesg_post="$LOG_DIR/dmesg_post.log"
dmesg -T > "$dmesg_pre" 2>/dev/null || dmesg > "$dmesg_pre" 2>/dev/null || true
pre_lines=$(wc -l < "$dmesg_pre" 2>/dev/null || echo 0)

C2_SYSPATH=""
for d in /sys/bus/usb/devices/*/; do
    [ -e "${d}idVendor" ] || continue
    v="$(cat "${d}idVendor" 2>/dev/null || echo)"
    p="$(cat "${d}idProduct" 2>/dev/null || echo)"
    if [ "$v" = "16d0" ] && [ "$p" = "0ed4" ]; then C2_SYSPATH="$d"; break; fi
done

echo "__W2_01_V3_BEGIN__"
echo "schema=w2_01_v3_stress/1"
echo "iterations=$ITERATIONS"
echo "sleep_ms=$SLEEP_MS"
echo "device=$DEVICE"
echo "c2_syspath=$C2_SYSPATH"
echo "dmesg_pre_lines=$pre_lines"
echo "utc_start=$(date -u +%Y-%m-%dT%H:%M:%SZ)"

ok_open=0
fail_open=0
ok_sysfs=0
fail_sysfs=0
wedge_seen=0
wedge_iter=-1

nap() {
    s=$(( SLEEP_MS / 1000 ))
    f=$(( SLEEP_MS % 1000 ))
    if [ "$f" -eq 0 ]; then
        sleep "$s" 2>/dev/null || true
    else
        sleep "${s}.$(printf '%03d' "$f")" 2>/dev/null || sleep "$s" 2>/dev/null || true
    fi
}

WEDGE_RE='ci_hdrc.*error|usb [0-9-]+:.*error -71|device descriptor read.*error|reset (high-speed|SuperSpeed).*-(71|110)|over-current'

i=0
while [ "$i" -lt "$ITERATIONS" ]; do
    i=$((i + 1))

    # Sysfs descriptor reads.
    if [ -n "$C2_SYSPATH" ]; then
        for f in idVendor idProduct product manufacturer bConfigurationValue power/control power/runtime_status; do
            if [ -r "${C2_SYSPATH}${f}" ]; then
                if cat "${C2_SYSPATH}${f}" >/dev/null 2>&1; then
                    ok_sysfs=$((ok_sysfs + 1))
                else
                    fail_sysfs=$((fail_sysfs + 1))
                fi
            fi
        done
    fi

    # Brief V4L2 device open via shell (subshell exits => fd closes =>
    # driver .release fires).
    if [ -e "$DEVICE" ]; then
        if ( exec 3<>"$DEVICE" ) 2>/dev/null; then
            ok_open=$((ok_open + 1))
        else
            fail_open=$((fail_open + 1))
        fi
    else
        fail_open=$((fail_open + 1))
    fi

    # Wedge canary: did the kernel start emitting USB host errors?
    if dmesg 2>/dev/null | tail -n 80 | grep -E -q "$WEDGE_RE"; then
        wedge_seen=1
        wedge_iter=$i
        echo "wedge_detected_at_iter=$i"
        break
    fi

    nap
done

dmesg -T > "$dmesg_post" 2>/dev/null || dmesg > "$dmesg_post" 2>/dev/null || true
post_lines=$(wc -l < "$dmesg_post" 2>/dev/null || echo 0)

echo "iterations_run=$i"
echo "ok_open=$ok_open"
echo "fail_open=$fail_open"
echo "ok_sysfs=$ok_sysfs"
echo "fail_sysfs=$fail_sysfs"
echo "wedge_seen=$wedge_seen"
echo "wedge_iter=$wedge_iter"
echo "dmesg_pre=$dmesg_pre"
echo "dmesg_pre_lines=$pre_lines"
echo "dmesg_post=$dmesg_post"
echo "dmesg_post_lines=$post_lines"
echo "utc_end=$(date -u +%Y-%m-%dT%H:%M:%SZ)"

verdict="PASS"
if [ "$wedge_seen" = "1" ]; then
    verdict="FAIL_WEDGE"
elif [ "$fail_open" -gt 0 ] || [ "$fail_sysfs" -gt 0 ]; then
    verdict="FAIL_IO"
fi
echo "verdict=$verdict"
echo "__W2_01_V3_END__"

exit 0
