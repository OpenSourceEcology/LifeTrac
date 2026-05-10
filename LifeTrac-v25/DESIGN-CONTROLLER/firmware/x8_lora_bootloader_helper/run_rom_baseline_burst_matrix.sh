#!/bin/bash
# run_rom_baseline_burst_matrix.sh
# Burst-gated ROM baseline matrix with optional pre/post burst snapshots.

set -u

CFG=${CFG:-/tmp/lifetrac_p0c/06_assert_pa11_pf4.cfg}
OPENOCD_BASE=${OPENOCD_BASE:-/usr/arduino/extra/openocd_script-imx_gpio.cfg}
DEV=${DEV:-/dev/ttymxc3}
OUTROOT=${OUTROOT:-/tmp/lifetrac_p0c/rom_baseline_burst}
DELAYS_MS=${DELAYS_MS:-"30,50"}
BURSTS_PER_DELAY=${BURSTS_PER_DELAY:-10}
ATTEMPTS_PER_BURST=${ATTEMPTS_PER_BURST:-10}
BURST_PASS_MIN_ACK=${BURST_PASS_MIN_ACK:-1}
PRE_BURST_SNAPSHOT_CFG=${PRE_BURST_SNAPSHOT_CFG:-}
POST_FAIL_SNAPSHOT_CFG=${POST_FAIL_SNAPSHOT_CFG:-}
OPENOCD_LIFETIME_S=${OPENOCD_LIFETIME_S:-75}

DELAY_LIST=$(echo "$DELAYS_MS" | tr ',' ' ')
STAMP=$(date +%Y-%m-%d_%H%M%S)
OUTDIR=$OUTROOT/T6_rom_baseline_burst_$STAMP
mkdir -p "$OUTDIR"

OCD_PID=
cleanup() {
    if [ -n "${OCD_PID:-}" ]; then
        kill "$OCD_PID" 2>/dev/null || true
        pkill -P "$OCD_PID" openocd 2>/dev/null || true
        sleep 0.2
        kill -9 "$OCD_PID" 2>/dev/null || true
        wait "$OCD_PID" 2>/dev/null || true
        OCD_PID=
    fi
}
trap cleanup EXIT

run_snapshot() {
    local cfg="$1"
    local out="$2"
    if [ -n "$cfg" ] && [ -f "$cfg" ]; then
        openocd -f "$OPENOCD_BASE" -f "$cfg" > "$out" 2>&1 || true
    fi
}

echo "delay_ms,burst,attempt,resp_size,first_byte,result_class,hex_prefix" > "$OUTDIR/results.csv"
echo "delay_ms,burst,total_attempts,ack_count,nack_count,zero_count,silent_count,other_count,burst_pass" > "$OUTDIR/burst_summary.csv"
echo "[$(date +%H:%M:%S)] start OUTDIR=$OUTDIR DELAYS_MS=$DELAYS_MS BURSTS_PER_DELAY=$BURSTS_PER_DELAY ATTEMPTS_PER_BURST=$ATTEMPTS_PER_BURST" > "$OUTDIR/run.log"

overall_burst_total=0
overall_burst_pass=0
overall_burst_fail=0

for delay_ms in $DELAY_LIST; do
    for burst in $(seq 1 "$BURSTS_PER_DELAY"); do
        overall_burst_total=$((overall_burst_total + 1))

        run_snapshot "$PRE_BURST_SNAPSHOT_CFG" "$OUTDIR/pre_snapshot_d${delay_ms}_b${burst}.txt"

        if command -v timeout >/dev/null 2>&1; then
            nohup timeout "${OPENOCD_LIFETIME_S}s" openocd -c "gdb_port disabled" -f "$OPENOCD_BASE" -f "$CFG" > "$OUTDIR/openocd_d${delay_ms}_b${burst}.txt" 2>&1 < /dev/null &
        else
            nohup openocd -c "gdb_port disabled" -f "$OPENOCD_BASE" -f "$CFG" > "$OUTDIR/openocd_d${delay_ms}_b${burst}.txt" 2>&1 < /dev/null &
        fi
        OCD_PID=$!
        echo "[$(date +%H:%M:%S)] burst_start delay=$delay_ms burst=$burst ocd_pid=$OCD_PID" >> "$OUTDIR/run.log"
        sleep 5

        burst_ack=0
        burst_nack=0
        burst_zero=0
        burst_silent=0
        burst_other=0

        for attempt in $(seq 1 "$ATTEMPTS_PER_BURST"); do
            stty -F "$DEV" 19200 cs8 parenb -parodd -cstopb raw -echo -echoe -echok -ixon -ixoff -icrnl -onlcr time 5 min 0

            rm -f /tmp/rom_burst_resp.bin
            cat "$DEV" > /tmp/rom_burst_resp.bin &
            CATPID=$!

            sleep 0.2
            settle_s=$(awk "BEGIN { printf \"%.3f\", $delay_ms / 1000 }")
            sleep "$settle_s"
            printf '\x7f' > "$DEV"
            sleep 0.8

            kill -9 "$CATPID" 2>/dev/null || true
            wait "$CATPID" 2>/dev/null || true

            size=$(stat -c %s /tmp/rom_burst_resp.bin 2>/dev/null || echo 0)
            if [ "$size" -gt 0 ]; then
                first=$(head -c 1 /tmp/rom_burst_resp.bin | xxd -p | head -c 2)
                hex_prefix=$(xxd -p /tmp/rom_burst_resp.bin | tr -d '\n' | head -c 32)
            else
                first=NA
                hex_prefix=NA
            fi

            if [ "$size" -eq 0 ]; then
                cls=SILENT
                burst_silent=$((burst_silent + 1))
            else
                case "$first" in
                    79)
                        cls=ACK
                        burst_ack=$((burst_ack + 1))
                        ;;
                    1f)
                        cls=NACK
                        burst_nack=$((burst_nack + 1))
                        ;;
                    00)
                        cls=ZERO
                        burst_zero=$((burst_zero + 1))
                        ;;
                    *)
                        cls=OTHER
                        burst_other=$((burst_other + 1))
                        ;;
                esac
            fi

            echo "$delay_ms,$burst,$attempt,$size,$first,$cls,$hex_prefix" >> "$OUTDIR/results.csv"
        done

        cleanup
        echo "[$(date +%H:%M:%S)] burst_done delay=$delay_ms burst=$burst ack=$burst_ack nack=$burst_nack zero=$burst_zero silent=$burst_silent other=$burst_other" >> "$OUTDIR/run.log"

        burst_pass=0
        if [ "$burst_ack" -ge "$BURST_PASS_MIN_ACK" ]; then
            burst_pass=1
            overall_burst_pass=$((overall_burst_pass + 1))
        else
            overall_burst_fail=$((overall_burst_fail + 1))
            run_snapshot "$POST_FAIL_SNAPSHOT_CFG" "$OUTDIR/post_fail_snapshot_d${delay_ms}_b${burst}.txt"
        fi

        echo "$delay_ms,$burst,$ATTEMPTS_PER_BURST,$burst_ack,$burst_nack,$burst_zero,$burst_silent,$burst_other,$burst_pass" >> "$OUTDIR/burst_summary.csv"
    done
done

echo "delay_ms,bursts,pass_bursts,fail_bursts,pass_rate,attempts,ack,nack,zero,silent,other" > "$OUTDIR/delay_summary.csv"
for delay_ms in $DELAY_LIST; do
    bursts=$(awk -F, -v d="$delay_ms" 'NR>1 && $1==d {c++} END {print c+0}' "$OUTDIR/burst_summary.csv")
    pass_bursts=$(awk -F, -v d="$delay_ms" 'NR>1 && $1==d && $9==1 {c++} END {print c+0}' "$OUTDIR/burst_summary.csv")
    fail_bursts=$((bursts - pass_bursts))
    if [ "$bursts" -gt 0 ]; then
        pass_rate=$(awk "BEGIN { printf \"%.4f\", $pass_bursts / $bursts }")
    else
        pass_rate=0
    fi

    attempts=$(awk -F, -v d="$delay_ms" 'NR>1 && $1==d {c++} END {print c+0}' "$OUTDIR/results.csv")
    ack=$(awk -F, -v d="$delay_ms" 'NR>1 && $1==d && $6=="ACK" {c++} END {print c+0}' "$OUTDIR/results.csv")
    nack=$(awk -F, -v d="$delay_ms" 'NR>1 && $1==d && $6=="NACK" {c++} END {print c+0}' "$OUTDIR/results.csv")
    zero=$(awk -F, -v d="$delay_ms" 'NR>1 && $1==d && $6=="ZERO" {c++} END {print c+0}' "$OUTDIR/results.csv")
    silent=$(awk -F, -v d="$delay_ms" 'NR>1 && $1==d && $6=="SILENT" {c++} END {print c+0}' "$OUTDIR/results.csv")
    other=$(awk -F, -v d="$delay_ms" 'NR>1 && $1==d && $6=="OTHER" {c++} END {print c+0}' "$OUTDIR/results.csv")

    echo "$delay_ms,$bursts,$pass_bursts,$fail_bursts,$pass_rate,$attempts,$ack,$nack,$zero,$silent,$other" >> "$OUTDIR/delay_summary.csv"
done

total_attempts=$(awk -F, 'NR>1 {c++} END {print c+0}' "$OUTDIR/results.csv")
ack_all=$(awk -F, 'NR>1 && $6=="ACK" {c++} END {print c+0}' "$OUTDIR/results.csv")
nack_all=$(awk -F, 'NR>1 && $6=="NACK" {c++} END {print c+0}' "$OUTDIR/results.csv")
zero_all=$(awk -F, 'NR>1 && $6=="ZERO" {c++} END {print c+0}' "$OUTDIR/results.csv")
silent_all=$(awk -F, 'NR>1 && $6=="SILENT" {c++} END {print c+0}' "$OUTDIR/results.csv")
other_all=$(awk -F, 'NR>1 && $6=="OTHER" {c++} END {print c+0}' "$OUTDIR/results.csv")

if [ "$overall_burst_total" -gt 0 ]; then
    overall_pass_rate=$(awk "BEGIN { printf \"%.4f\", $overall_burst_pass / $overall_burst_total }")
else
    overall_pass_rate=0
fi

best_line=$(awk -F, 'NR==2 {best=$0; bestv=$3} NR>2 {if ($3 > bestv) {best=$0; bestv=$3}} END {print best}' "$OUTDIR/delay_summary.csv")

{
    echo "OUTDIR=$OUTDIR"
    echo "DELAYS_MS=$DELAYS_MS"
    echo "BURSTS_PER_DELAY=$BURSTS_PER_DELAY"
    echo "ATTEMPTS_PER_BURST=$ATTEMPTS_PER_BURST"
    echo "BURST_PASS_MIN_ACK=$BURST_PASS_MIN_ACK"
    echo "TOTAL_BURSTS=$overall_burst_total"
    echo "PASS_BURSTS=$overall_burst_pass"
    echo "FAIL_BURSTS=$overall_burst_fail"
    echo "BURST_PASS_RATE=$overall_pass_rate"
    echo "TOTAL_PROBES=$total_attempts"
    echo "ACK_COUNT=$ack_all"
    echo "NACK_COUNT=$nack_all"
    echo "ZERO_COUNT=$zero_all"
    echo "SILENT_COUNT=$silent_all"
    echo "OTHER_COUNT=$other_all"
    echo "BEST_DELAY_LINE=$best_line"
} > "$OUTDIR/summary.txt"

cat "$OUTDIR/summary.txt"
echo "[$(date +%H:%M:%S)] done" >> "$OUTDIR/run.log"
