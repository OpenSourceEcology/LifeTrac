#!/bin/bash
# W2-01 deliverable E: hard gate around w2_01_camera_usb_guard.sh.
#
# Runs the existing canary, then verifies the policy invariants from
# AI NOTES/2026-05-14_USB_Wedge_Software_Mitigations.md Section 10.1.E:
#
#   - C2 (16d0:0ed4) is present.
#   - /dev/lifetrac-c2 resolves to the C2 video-index0 capture node.
#   - snd_usb_audio is not bound to any C2 interface.
#   - USB power management for the C2 is effectively disabled, either
#     via global autosuspend=-1 (deliverable F) or via per-device
#     power/control=on + power/autosuspend_delay_ms=-1 (deliverable C).
#
# Output is the guard's framed status block followed by a
# __W2_01_GATE_BEGIN__ / __W2_01_GATE_END__ block. Exit code is 0 on
# pass and 1 on any failed invariant.
set -u

C2_VID="16d0"
C2_PID="0ed4"
C2_SYMLINK="/dev/lifetrac-c2"
C2_BY_ID="/dev/v4l/by-id/usb-Kurokesu_C2_KTM-QGFST-video-index0"

SELF_DIR="$(cd "$(dirname "$0")" && pwd)"
GUARD="${GUARD:-$SELF_DIR/w2_01_camera_usb_guard.sh}"

if [ ! -x "$GUARD" ]; then
    echo "__W2_01_GATE_BEGIN__"
    echo "gate=FAIL"
    echo "fail=guard_missing"
    echo "guard_path=$GUARD"
    echo "__W2_01_GATE_END__"
    exit 2
fi

GUARD_OUT="$("$GUARD" 2>&1 || true)"
echo "$GUARD_OUT"

extract() { echo "$GUARD_OUT" | sed -n "s/^$1=\(.*\)$/\1/p" | tail -n1; }
guard_verdict="$(extract verdict)"
final_autosuspend="$(extract autosuspend_final)"
c2_present_at_start="$(extract c2_present_at_start)"

# /dev/lifetrac-c2 -> video-index0?
sym_ok=0
sym_target="?"
sym_index0_target="?"
if [ -e "$C2_SYMLINK" ]; then
    sym_target="$(readlink -f "$C2_SYMLINK" 2>/dev/null || echo '?')"
fi
if [ -e "$C2_BY_ID" ]; then
    sym_index0_target="$(readlink -f "$C2_BY_ID" 2>/dev/null || echo '?')"
fi
if [ "$sym_target" != "?" ] && [ "$sym_target" = "$sym_index0_target" ]; then
    sym_ok=1
fi

# snd_usb_audio bound to any C2 interface?
snd_bound_count=0
SND_DRV="/sys/bus/usb/drivers/snd-usb-audio"
C2_SYSPATH=""
for d in /sys/bus/usb/devices/*/; do
    [ -e "${d}idVendor" ] || continue
    v="$(cat "${d}idVendor" 2>/dev/null || echo)"
    p="$(cat "${d}idProduct" 2>/dev/null || echo)"
    if [ "$v" = "$C2_VID" ] && [ "$p" = "$C2_PID" ]; then
        C2_SYSPATH="$d"
        break
    fi
done
C2_BUSDEV=""
[ -n "$C2_SYSPATH" ] && C2_BUSDEV="$(basename "$C2_SYSPATH")"
if [ -d "$SND_DRV" ] && [ -n "$C2_BUSDEV" ]; then
    for l in "$SND_DRV"/${C2_BUSDEV}:*; do
        [ -e "$l" ] || continue
        snd_bound_count=$((snd_bound_count + 1))
    done
fi

# PM effectively disabled?
pm_ok=0
c2_power_control="?"
c2_power_delay="?"
if [ -n "$C2_SYSPATH" ]; then
    [ -r "${C2_SYSPATH}power/control" ] && c2_power_control="$(cat "${C2_SYSPATH}power/control" 2>/dev/null || echo '?')"
    [ -r "${C2_SYSPATH}power/autosuspend_delay_ms" ] && c2_power_delay="$(cat "${C2_SYSPATH}power/autosuspend_delay_ms" 2>/dev/null || echo '?')"
fi
if [ "$final_autosuspend" = "-1" ]; then
    pm_ok=1
elif [ "$c2_power_control" = "on" ] && [ "$c2_power_delay" = "-1" ]; then
    pm_ok=1
fi

echo "__W2_01_GATE_BEGIN__"
echo "schema=w2_01_gate/1"
echo "guard_verdict=$guard_verdict"
echo "c2_present_at_start=$c2_present_at_start"
echo "lifetrac_c2_target=$sym_target"
echo "lifetrac_c2_index0_target=$sym_index0_target"
echo "lifetrac_c2_index0_ok=$sym_ok"
echo "snd_usb_audio_bound_count=$snd_bound_count"
echo "autosuspend_final=$final_autosuspend"
echo "c2_power_control=$c2_power_control"
echo "c2_power_autosuspend_delay_ms=$c2_power_delay"
echo "pm_effective=$pm_ok"

rc=0
if [ "$c2_present_at_start" != "1" ]; then echo "fail=c2_missing"; rc=1; fi
if [ "$sym_ok" != "1" ]; then echo "fail=lifetrac_c2_symlink_drift"; rc=1; fi
if [ "$snd_bound_count" != "0" ]; then echo "fail=snd_usb_audio_bound"; rc=1; fi
if [ "$pm_ok" != "1" ]; then echo "fail=pm_not_effective"; rc=1; fi

if [ "$rc" = "0" ]; then
    echo "gate=PASS"
else
    echo "gate=FAIL"
fi
echo "__W2_01_GATE_END__"
exit $rc
