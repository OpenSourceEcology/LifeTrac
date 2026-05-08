#!/bin/sh
# auto_boot0_hunt.sh — automated L072 BOOT0 net discovery on X8 + Max Carrier.
#
# Strategy: for each candidate H747 GPIO pin (excluding bench-critical pins),
#   1. halt M7 via openocd, enable clocks, drive candidate HIGH, pulse PC_7
#      (NRST) low/high, resume and shutdown openocd.
#   2. probe /dev/ttymxc3 with 0x7F at 8E1 — if 0x79 ACK comes back, the
#      candidate is wired to L072 BOOT0 and we stop.
#   3. otherwise move to next candidate. Reset the L072 back to AT firmware
#      between candidates (drive candidate LOW + pulse NRST again via openocd).
#
# Bench-safety pins skipped:
#   PA_11/12 USB OTG (adb), PA_13/14 SWD, PC_7 (used as NRST itself),
#   PC_8..12 SDMMC1, PG_11/13/14 ETH RMII.
#
# Order: try GPIOJ first (PJ_11 is LORA_IRQ in the H7_M7 variant — neighbours
# may include BOOT0 on a Max-Carrier-specific net), then GPIOG (in case PG_7
# was a soldering/header difference), then the rest.
#
# Run as root. Logs to /tmp/p0c_hunt.log.

set -u
LOG=/tmp/p0c_hunt.log
: > $LOG
echo "$(date) auto_boot0_hunt starting" | tee -a $LOG

# Bank base addresses
bank_base() {
    case "$1" in
        A) echo 0x58020000 ;;
        B) echo 0x58020400 ;;
        C) echo 0x58020800 ;;
        D) echo 0x58020c00 ;;
        E) echo 0x58021000 ;;
        F) echo 0x58021400 ;;
        G) echo 0x58021800 ;;
        H) echo 0x58021c00 ;;
        I) echo 0x58022000 ;;
        J) echo 0x58022400 ;;
        K) echo 0x58022800 ;;
    esac
}
bank_bit() {
    case "$1" in
        A) echo 0 ;; B) echo 1 ;; C) echo 2 ;; D) echo 3 ;;
        E) echo 4 ;; F) echo 5 ;; G) echo 6 ;; H) echo 7 ;;
        I) echo 8 ;; J) echo 9 ;; K) echo 10 ;;
    esac
}

# Each candidate is "BANK,PIN".
# Order tuned per script header rationale.
CANDIDATES=""
for P in 0 1 2 3 4 5 6 7 8 9 10 12; do CANDIDATES="$CANDIDATES J,$P"; done   # skip J,11 (LORA_IRQ)
for P in 0 1 2 3 4 5 7 8 9 10 12 15; do CANDIDATES="$CANDIDATES G,$P"; done  # skip G,6 (we already proved G,7 wrong but keep for completeness?), G,11/13/14 ETH
CANDIDATES="$CANDIDATES G,6"   # try G,6 for completeness
for P in 0 1 2 3 4 5 6 ; do CANDIDATES="$CANDIDATES C,$P"; done                # skip C,7 (NRST), C,8..12 (SDMMC1)
for P in 13 14 15; do CANDIDATES="$CANDIDATES C,$P"; done
for P in 0 1 2 3 4 5 6 7 8 9 10 15; do CANDIDATES="$CANDIDATES A,$P"; done   # skip A,11/12 USB, A,13/14 SWD
for B in B D E F H I; do
    for P in 0 1 2 3 4 5 6 7 8 9 10 11 12 13 14 15; do
        CANDIDATES="$CANDIDATES $B,$P"
    done
done

probe_l072_rom() {
    DEV=/dev/ttymxc3
    stty -F $DEV 19200 cs8 parenb -parodd -cstopb raw -echo -echoe -echok -ixon -ixoff -icrnl -onlcr time 5 min 0
    rm -f /tmp/hunt_resp.bin
    ( cat $DEV > /tmp/hunt_resp.bin & ) ; CATPID=$!
    sleep 0.2
    printf '\x7f' > $DEV
    sleep 0.8
    kill $CATPID 2>/dev/null || true
    wait 2>/dev/null || true
    SZ=$(stat -c %s /tmp/hunt_resp.bin 2>/dev/null || echo 0)
    if [ "$SZ" -gt 0 ]; then
        FIRST=$(head -c 1 /tmp/hunt_resp.bin | xxd -p)
        echo "$FIRST"
    else
        echo "00"
    fi
}

write_assert_cfg() {
    BASE="$1"; PIN="$2"; CLK_BIT="$3"; OUT_FILE="$4"
    cat > "$OUT_FILE" <<EOF
proc memread32 {addr} {
    array unset _r32
    mem2array _r32 32 \$addr 1
    return \$_r32(0)
}
proc rmw32 {addr mask val} {
    set cur [memread32 \$addr]
    set new [expr {(\$cur & ~\$mask) | (\$val & \$mask)}]
    mww \$addr \$new
}
# enable GPIOC (NRST bank, bit 2) + candidate bank (bit $CLK_BIT)
rmw32 0x580244E0 [expr {(1 << 2) | (1 << $CLK_BIT)}] [expr {(1 << 2) | (1 << $CLK_BIT)}]
# candidate MODER -> output (RMW bits [2*PIN +: 2] := 01)
rmw32 [expr {$BASE + 0x00}] [expr {0x3 << ($PIN * 2)}] [expr {0x1 << ($PIN * 2)}]
# drive candidate HIGH
mww  [expr {$BASE + 0x18}] [expr {1 << $PIN}]
# NRST MODER -> output
rmw32 0x58020800 [expr {0x3 << (7 * 2)}] [expr {0x1 << (7 * 2)}]
# pulse NRST low/high
mww  0x58020818 [expr {1 << (7 + 16)}]
sleep 60
mww  0x58020818 [expr {1 << 7}]
sleep 50
# re-assert candidate HIGH (paranoia)
mww  [expr {$BASE + 0x18}] [expr {1 << $PIN}]
resume
shutdown
EOF
}

write_recover_cfg() {
    OUT_FILE="$1"
    cat > "$OUT_FILE" <<EOF
proc memread32 {addr} {
    array unset _r32
    mem2array _r32 32 \$addr 1
    return \$_r32(0)
}
proc rmw32 {addr mask val} {
    set cur [memread32 \$addr]
    set new [expr {(\$cur & ~\$mask) | (\$val & \$mask)}]
    mww \$addr \$new
}
# enable GPIOC clock
rmw32 0x580244E0 [expr {(1 << 2)}] [expr {(1 << 2)}]
# NRST MODER -> output
rmw32 0x58020800 [expr {0x3 << (7 * 2)}] [expr {0x1 << (7 * 2)}]
# pulse NRST low/high (no BOOT0 driven by us -> L072 returns to AT firmware)
mww  0x58020818 [expr {1 << (7 + 16)}]
sleep 60
mww  0x58020818 [expr {1 << 7}]
sleep 50
resume
shutdown
EOF
}

OCD_CFG=/usr/arduino/extra/openocd_script-imx_gpio.cfg
ASSERT=/tmp/lifetrac_p0c/_assert_one.cfg
RECOVER=/tmp/lifetrac_p0c/_recover.cfg

write_recover_cfg "$RECOVER"

i=0
for CAND in $CANDIDATES; do
    i=$((i+1))
    BANK=$(echo "$CAND" | cut -d, -f1)
    PIN=$(echo "$CAND" | cut -d, -f2)
    BASE=$(bank_base "$BANK")
    CLK_BIT=$(bank_bit "$BANK")
    LABEL="P${BANK}_${PIN}"
    echo "[$i] Testing $LABEL (base=$BASE clk_bit=$CLK_BIT)" | tee -a $LOG
    write_assert_cfg "$BASE" "$PIN" "$CLK_BIT" "$ASSERT"
    openocd -f "$OCD_CFG" -f "$ASSERT" > /tmp/p0c_hunt_ocd.log 2>&1
    if ! grep -q "shutdown command invoked" /tmp/p0c_hunt_ocd.log; then
        echo "    OCD failed:" | tee -a $LOG
        tail -8 /tmp/p0c_hunt_ocd.log | sed 's/^/      /' | tee -a $LOG
        continue
    fi
    sleep 0.3
    RESP=$(probe_l072_rom)
    echo "    probe response first byte = 0x$RESP" | tee -a $LOG
    if [ "$RESP" = "79" ]; then
        echo ""
        echo "*** FOUND: $LABEL is the L072 BOOT0 net (ROM ACK 0x79 received) ***" | tee -a $LOG
        echo "(Stopping hunt. L072 currently held in bootloader by H7 firmware default state.)" | tee -a $LOG
        exit 0
    fi
    # Recover L072 back to AT firmware for clean next test.
    openocd -f "$OCD_CFG" -f "$RECOVER" > /tmp/p0c_hunt_recover.log 2>&1
    sleep 0.2
done

echo "" | tee -a $LOG
echo "Hunt complete. No candidate produced 0x79 ACK." | tee -a $LOG
exit 1
