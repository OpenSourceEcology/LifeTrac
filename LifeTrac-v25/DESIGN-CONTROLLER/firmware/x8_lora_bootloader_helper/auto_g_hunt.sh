#!/bin/sh
# auto_g_hunt.sh — hunt for L072 NRST within GPIOG, with PG_7 always HIGH (BOOT0).
#
# Insight from x8h7 firmware source:
#   - x8h7 GPIO_pinmap[] uses banks A,B,C,D,E,F (no G entries).
#   - But MX_GPIO_Init() enables GPIOG clock — so GPIOG is reserved for the H7
#     firmware's internal use, almost certainly LoRa control.
#   - Per Portenta H7 variant convention: LORA_BOOT0=PG_7, LORA_RESET=PG_8.
#   - PC_7 turned out to be mapped to PWM channel 0, NOT LoRa NRST.
#
# Strategy: drive PG_7 HIGH (assumed BOOT0), then sweep PG_0..PG_15 (skip
# PG_7 itself and PG_11/13/14 which are Ethernet RMII), pulsing each candidate
# LOW(60ms)/HIGH as a tentative NRST. Probe ttymxc3 for 0x79 ACK after each.

set -u
LOG=/tmp/p0c_ghunt.log
: > $LOG
echo "$(date) auto_g_hunt starting" | tee -a $LOG

GPIOG_BASE=0x58021800
GPIOG_CLK_BIT=6

probe_l072_rom() {
    DEV=/dev/ttymxc3
    stty -F $DEV 19200 cs8 parenb -parodd -cstopb raw -echo -echoe -echok -ixon -ixoff -icrnl -onlcr time 5 min 0 2>/dev/null
    rm -f /tmp/ghunt_resp.bin
    cat $DEV > /tmp/ghunt_resp.bin &
    CATPID=$!
    sleep 0.25
    printf '\x7f' > $DEV
    sleep 0.7
    kill -9 $CATPID 2>/dev/null
    wait $CATPID 2>/dev/null
    SZ=$(stat -c %s /tmp/ghunt_resp.bin 2>/dev/null || echo 0)
    if [ "$SZ" -gt 0 ]; then
        FIRST=$(head -c 1 /tmp/ghunt_resp.bin | xxd -p | head -c 2)
        HEX=$(xxd -p /tmp/ghunt_resp.bin | tr -d '\n' | head -c 32)
        echo "first=${FIRST} sz=${SZ} hex=${HEX}"
    else
        echo "first=__ sz=0"
    fi
}

write_assert_cfg() {
    NRST_PIN="$1"; OUT_FILE="$2"
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
# enable GPIOG clock (bit 6)
rmw32 0x580244E0 [expr {(1 << $GPIOG_CLK_BIT)}] [expr {(1 << $GPIOG_CLK_BIT)}]
# PG_7 MODER -> output, drive HIGH (BOOT0)
rmw32 [expr {$GPIOG_BASE + 0x00}] [expr {0x3 << (7 * 2)}] [expr {0x1 << (7 * 2)}]
mww  [expr {$GPIOG_BASE + 0x18}] [expr {1 << 7}]
# Candidate NRST pin MODER -> output
rmw32 [expr {$GPIOG_BASE + 0x00}] [expr {0x3 << ($NRST_PIN * 2)}] [expr {0x1 << ($NRST_PIN * 2)}]
# pulse candidate LOW then HIGH
mww  [expr {$GPIOG_BASE + 0x18}] [expr {1 << ($NRST_PIN + 16)}]
sleep 60
mww  [expr {$GPIOG_BASE + 0x18}] [expr {1 << $NRST_PIN}]
sleep 80
# re-assert PG_7 HIGH for paranoia
mww  [expr {$GPIOG_BASE + 0x18}] [expr {1 << 7}]
resume
shutdown
EOF
}

OCD_CFG=/usr/arduino/extra/openocd_script-imx_gpio.cfg
ASSERT=/tmp/lifetrac_p0c/_assert_g.cfg

# Skip 7 (BOOT0 itself), 11/13/14 (Ethernet RMII)
for PIN in 0 1 2 3 4 5 6 8 9 10 12 15; do
    LABEL="PG_${PIN}"
    echo "[hunt] testing NRST=$LABEL  (PG_7=BOOT0=HIGH)" | tee -a $LOG
    write_assert_cfg "$PIN" "$ASSERT"
    openocd -f "$OCD_CFG" -f "$ASSERT" > /tmp/p0c_ghunt_ocd.log 2>&1
    if ! grep -q "shutdown command invoked" /tmp/p0c_ghunt_ocd.log; then
        echo "    OCD failed:" | tee -a $LOG
        tail -6 /tmp/p0c_ghunt_ocd.log | sed 's/^/      /' | tee -a $LOG
        continue
    fi
    sleep 0.3
    RESP=$(probe_l072_rom)
    echo "    probe: $RESP" | tee -a $LOG
    FIRST=$(echo "$RESP" | sed -n 's/.*first=\([0-9a-f][0-9a-f]\).*/\1/p')
    if [ "$FIRST" = "79" ]; then
        echo ""
        echo "*** FOUND: PG_7=BOOT0, $LABEL=NRST -> L072 ROM ACK 0x79 received ***" | tee -a $LOG
        exit 0
    fi
done

echo "" | tee -a $LOG
echo "GPIOG hunt complete. No NRST candidate produced 0x79." | tee -a $LOG
exit 1
