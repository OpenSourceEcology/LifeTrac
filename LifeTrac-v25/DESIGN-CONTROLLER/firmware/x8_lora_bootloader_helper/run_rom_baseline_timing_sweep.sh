#!/bin/bash
# run_rom_baseline_timing_sweep.sh
# One ROM-hold window, many sync probes.

set -u

CFG=${CFG:-/tmp/lifetrac_p0c/06_assert_pa11_pf4.cfg}
OPENOCD_BASE=${OPENOCD_BASE:-/usr/arduino/extra/openocd_script-imx_gpio.cfg}
DEV=${DEV:-/dev/ttymxc3}
OUTROOT=${OUTROOT:-/tmp/lifetrac_p0c/rom_baseline_timing}
DELAYS_MS=${DELAYS_MS:-"50 100 200 400"}
ATTEMPTS_PER_DELAY=${ATTEMPTS_PER_DELAY:-5}
DELAY_LIST=$(echo "$DELAYS_MS" | tr ',' ' ')

STAMP=$(date +%Y-%m-%d_%H%M%S)
OUTDIR=$OUTROOT/T6_rom_baseline_timing_$STAMP
mkdir -p "$OUTDIR"

OCD_PID=
cleanup() {
    if [ -n "${OCD_PID:-}" ]; then
        kill "$OCD_PID" 2>/dev/null || true
        wait "$OCD_PID" 2>/dev/null || true
    fi
}
trap cleanup EXIT

nohup openocd -f "$OPENOCD_BASE" -f "$CFG" > "$OUTDIR/openocd_06.txt" 2>&1 < /dev/null &
OCD_PID=$!
sleep 5

echo "delay_ms,attempt,resp_size,first_byte,result_class,hex_prefix" > "$OUTDIR/results.csv"

for delay_ms in $DELAY_LIST; do
    for attempt in $(seq 1 "$ATTEMPTS_PER_DELAY"); do
        stty -F "$DEV" 19200 cs8 parenb -parodd -cstopb raw -echo -echoe -echok -ixon -ixoff -icrnl -onlcr time 5 min 0

        rm -f /tmp/rom_timing_resp.bin
        cat "$DEV" > /tmp/rom_timing_resp.bin &
        CATPID=$!

        sleep 0.2
        settle_s=$(awk "BEGIN { printf \"%.3f\", $delay_ms / 1000 }")
        sleep "$settle_s"
        printf '\x7f' > "$DEV"
        sleep 0.8

        kill -9 "$CATPID" 2>/dev/null || true
        wait "$CATPID" 2>/dev/null || true

        size=$(stat -c %s /tmp/rom_timing_resp.bin 2>/dev/null || echo 0)
        if [ "$size" -gt 0 ]; then
            first=$(head -c 1 /tmp/rom_timing_resp.bin | xxd -p | head -c 2)
            hex_prefix=$(xxd -p /tmp/rom_timing_resp.bin | tr -d '\n' | head -c 32)
        else
            first=NA
            hex_prefix=NA
        fi

        if [ "$size" -eq 0 ]; then
            cls=SILENT
        else
            case "$first" in
                79) cls=ACK ;;
                1f) cls=NACK ;;
                00) cls=ZERO ;;
                *)  cls=OTHER ;;
            esac
        fi

        echo "$delay_ms,$attempt,$size,$first,$cls,$hex_prefix" >> "$OUTDIR/results.csv"
    done
done

echo "delay_ms,total,ack,nack,zero,silent,other" > "$OUTDIR/delay_summary.csv"
for delay_ms in $DELAY_LIST; do
    total=$(awk -F, -v d="$delay_ms" 'NR>1 && $1==d {c++} END {print c+0}' "$OUTDIR/results.csv")
    ack=$(awk -F, -v d="$delay_ms" 'NR>1 && $1==d && $5=="ACK" {c++} END {print c+0}' "$OUTDIR/results.csv")
    nack=$(awk -F, -v d="$delay_ms" 'NR>1 && $1==d && $5=="NACK" {c++} END {print c+0}' "$OUTDIR/results.csv")
    zero=$(awk -F, -v d="$delay_ms" 'NR>1 && $1==d && $5=="ZERO" {c++} END {print c+0}' "$OUTDIR/results.csv")
    silent=$(awk -F, -v d="$delay_ms" 'NR>1 && $1==d && $5=="SILENT" {c++} END {print c+0}' "$OUTDIR/results.csv")
    other=$(awk -F, -v d="$delay_ms" 'NR>1 && $1==d && $5=="OTHER" {c++} END {print c+0}' "$OUTDIR/results.csv")
    echo "$delay_ms,$total,$ack,$nack,$zero,$silent,$other" >> "$OUTDIR/delay_summary.csv"
done

total_all=$(awk -F, 'NR>1 {c++} END {print c+0}' "$OUTDIR/results.csv")
ack_all=$(awk -F, 'NR>1 && $5=="ACK" {c++} END {print c+0}' "$OUTDIR/results.csv")
nack_all=$(awk -F, 'NR>1 && $5=="NACK" {c++} END {print c+0}' "$OUTDIR/results.csv")
zero_all=$(awk -F, 'NR>1 && $5=="ZERO" {c++} END {print c+0}' "$OUTDIR/results.csv")
silent_all=$(awk -F, 'NR>1 && $5=="SILENT" {c++} END {print c+0}' "$OUTDIR/results.csv")
other_all=$(awk -F, 'NR>1 && $5=="OTHER" {c++} END {print c+0}' "$OUTDIR/results.csv")

best_line=$(awk -F, 'NR>1 { if ($3 > best_ack) { best_ack=$3; best=$0 } } END { print best }' "$OUTDIR/delay_summary.csv")

{
    echo "OUTDIR=$OUTDIR"
    echo "ATTEMPTS_PER_DELAY=$ATTEMPTS_PER_DELAY"
    echo "DELAYS_MS=$DELAYS_MS"
    echo "TOTAL_PROBES=$total_all"
    echo "ACK_COUNT=$ack_all"
    echo "NACK_COUNT=$nack_all"
    echo "ZERO_COUNT=$zero_all"
    echo "SILENT_COUNT=$silent_all"
    echo "OTHER_COUNT=$other_all"
    echo "BEST_DELAY_LINE=$best_line"
} > "$OUTDIR/summary.txt"

cat "$OUTDIR/summary.txt"
