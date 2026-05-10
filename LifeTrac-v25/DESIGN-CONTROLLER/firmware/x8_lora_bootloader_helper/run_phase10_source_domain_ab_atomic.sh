#!/bin/sh
# run_phase10_source_domain_ab_atomic.sh
# Atomic Phase 10 runner: require ROM 0x79 baseline first, then run user-halt comparator.

set -eu

WORKDIR=${WORKDIR:-/tmp/lifetrac_p0c}
EVROOT=${EVROOT:-/tmp/lifetrac_p0c/phase10}
MAX_ROM_ATTEMPTS=${MAX_ROM_ATTEMPTS:-5}
OPENOCD_BASE=${OPENOCD_BASE:-/usr/arduino/extra/openocd_script-imx_gpio.cfg}
ROM_CFG=${ROM_CFG:-$WORKDIR/06_assert_pa11_pf4.cfg}
USERHALT_CFG=${USERHALT_CFG:-$WORKDIR/47_phase9_boot_then_halt_000ms.cfg}
VERIFY_SH=${VERIFY_SH:-$WORKDIR/verify_l072_rom.sh}

STAMP=$(date +%Y-%m-%d_%H%M%S)
EVDIR=$EVROOT/T6_phase10_source_domain_ab_atomic_$STAMP
mkdir -p "$EVDIR"

OCD_PID=
cleanup_openocd() {
    if [ -n "${OCD_PID:-}" ]; then
        kill "$OCD_PID" 2>/dev/null || true
        wait "$OCD_PID" 2>/dev/null || true
        OCD_PID=
    fi
}

trap cleanup_openocd EXIT

rom_ok=0
rom_hit_attempt=0
attempt=1
while [ "$attempt" -le "$MAX_ROM_ATTEMPTS" ]; do
    echo "[phase10] ROM baseline attempt $attempt/$MAX_ROM_ATTEMPTS"

    openocd -f "$OPENOCD_BASE" -f "$ROM_CFG" > "$EVDIR/rom${attempt}_openocd_06.txt" 2>&1 &
    OCD_PID=$!
    sleep 5
    sh "$VERIFY_SH" > "$EVDIR/rom${attempt}_verify_l072_rom.txt" 2>&1
    cleanup_openocd

    if grep -q "First byte after 0x7F: 0x79" "$EVDIR/rom${attempt}_verify_l072_rom.txt"; then
        rom_ok=1
        rom_hit_attempt=$attempt
        break
    fi

    attempt=$((attempt + 1))
done

if [ "$rom_ok" -ne 1 ]; then
    {
        echo "PHASE10_ATOMIC_RESULT=FAIL_NO_ROM_BASELINE"
        echo "ROM_ACK_ATTEMPT=0"
        echo "MAX_ROM_ATTEMPTS=$MAX_ROM_ATTEMPTS"
    } > "$EVDIR/summary.txt"

    echo "EVIDENCE_DIR=$EVDIR"
    echo "PHASE10_ATOMIC_RESULT=FAIL_NO_ROM_BASELINE"
    exit 2
fi

openocd -f "$OPENOCD_BASE" -f "$USERHALT_CFG" > "$EVDIR/userhalt_openocd_47.txt" 2>&1
sh "$VERIFY_SH" > "$EVDIR/userhalt_verify_l072_rom.txt" 2>&1

USER_ROM_RESP=$(grep -E "^ROM_RESP_SIZE=" "$EVDIR/userhalt_verify_l072_rom.txt" | tail -1 | cut -d= -f2)
USER_FIRST=$(grep -E "^First byte after 0x7F:" "$EVDIR/userhalt_verify_l072_rom.txt" | tail -1 | awk '{print $5}')
[ -n "$USER_ROM_RESP" ] || USER_ROM_RESP=NA
[ -n "$USER_FIRST" ] || USER_FIRST=NA

{
    echo "PHASE10_ATOMIC_RESULT=OK"
    echo "ROM_ACK_ATTEMPT=$rom_hit_attempt"
    echo "USERHALT_ROM_RESP_SIZE=$USER_ROM_RESP"
    echo "USERHALT_FIRST_BYTE=$USER_FIRST"
} > "$EVDIR/summary.txt"

echo "EVIDENCE_DIR=$EVDIR"
echo "PHASE10_ATOMIC_RESULT=OK"
echo "ROM_ACK_ATTEMPT=$rom_hit_attempt"
echo "USERHALT_ROM_RESP_SIZE=$USER_ROM_RESP"
echo "USERHALT_FIRST_BYTE=$USER_FIRST"
