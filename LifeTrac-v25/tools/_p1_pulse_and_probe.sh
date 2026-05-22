#!/bin/bash
# _p1_pulse_and_probe.sh — pulse gpio163 (L072 NRST) and immediately run
# the bring-up probe. Used by p1_cold_boot_discriminator.ps1 (Phase 2.1).
#
# Args:
#   $1  probe mode (rx | tx)   — passed to --probe
#   $2  rx window seconds      — passed to --rx-window
#   $3  pre-probe sleep seconds — extra dwell between NRST release and
#                                 probe launch (used by the delay sweep
#                                 discriminator; default 0.05)
#
# Output contract (first line):
#   PULSE_DONE_AT=<unix.ns>     — wall-clock at NRST release (sysfs write)
# Then the full probe stdout follows verbatim.
#
# Reset sequence (SOFT_RESET_INDEX 3.1): NRST is active-low; sysfs
# default after export is input/low, so we set direction=out, drive
# high (de-assert), then low (assert NRST), then high (release).
set -u
PROBE_MODE=${1:-rx}
RX_WINDOW=${2:-0.5}
PRE_PROBE_SLEEP=${3:-0.05}

[ -d /sys/class/gpio/gpio163 ] || echo 163 > /sys/class/gpio/export 2>/dev/null
echo out > /sys/class/gpio/gpio163/direction 2>/dev/null

echo 1 > /sys/class/gpio/gpio163/value   # de-assert NRST (idle high)
sleep 0.02
echo 0 > /sys/class/gpio/gpio163/value   # assert NRST low
sleep 0.10
echo 1 > /sys/class/gpio/gpio163/value   # release NRST -> L072 boots
echo "PULSE_DONE_AT=$(date +%s.%N)"
echo "PRE_PROBE_SLEEP=${PRE_PROBE_SLEEP}"

# Variable pre-probe dwell: this is the knob the discriminator sweeps.
# If RUNTIME_PROFILE_ENUM flips from ERR to OK as this grows, the L072
# firmware is simply not yet able to answer the request when the probe
# opens the port — i.e. firmware-not-ready limb.
sleep "${PRE_PROBE_SLEEP}"

cd /tmp/lifetrac_p0c
exec python3 -u method_h_stage2_tx_probe_v2.py \
    --dev /dev/ttymxc3 \
    --baud 921600 \
    --probe "$PROBE_MODE" \
    --rx-window "$RX_WINDOW"
