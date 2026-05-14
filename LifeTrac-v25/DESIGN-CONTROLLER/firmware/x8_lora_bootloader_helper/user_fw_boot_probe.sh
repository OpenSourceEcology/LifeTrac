#!/bin/bash
# user_fw_boot_probe.sh — drop PA11 LOW (BOOT0 low → boot from flash), pulse
# NRST, then sniff /dev/ttymxc3 at multiple bauds to see if the L072 user
# firmware emits anything (boot heartbeat, fault frame, COBS frames, etc.).
#
# Pre-req: x8h7 bridge alive, gpio-163 exported, pwm4 exported.
#
# Outputs:
#   /tmp/lifetrac_p0c/user_fw_NNNN.bin  — raw RX capture per baud
#   /tmp/lifetrac_p0c/user_fw_boot_probe.log — summary
set -u
DEV=/dev/ttymxc3
LOG=/tmp/lifetrac_p0c/user_fw_boot_probe.log
mkdir -p /tmp/lifetrac_p0c
: > "$LOG"

GPIO=163
PWMCHIP=/sys/class/pwm/pwmchip0
PWM=$PWMCHIP/pwm4

# ---- Step 1: drive PA11 LOW (BOOT0 low → boot from main flash) -----------
if ! [ -d $PWM ]; then
    echo 4 > $PWMCHIP/export 2>/dev/null || true
fi
echo 0 > $PWM/enable 2>/dev/null || true
echo 1000000 > $PWM/period
echo 0       > $PWM/duty_cycle      # 0 duty → output stays LOW
echo 1       > $PWM/enable
echo "PWM4: enable=$(cat $PWM/enable) period=$(cat $PWM/period) duty=$(cat $PWM/duty_cycle)  (BOOT0=LOW)" | tee -a "$LOG"
sleep 0.05

# ---- Step 2: ensure NRST gpio is exported and high -----------------------
if ! [ -d /sys/class/gpio/gpio$GPIO ]; then
    echo "$GPIO" > /sys/class/gpio/export 2>/dev/null || true
fi
echo "out" > /sys/class/gpio/gpio$GPIO/direction
echo "1"   > /sys/class/gpio/gpio$GPIO/value

# ---- Step 3: probe at each baud ------------------------------------------
for BAUD in 921600 115200 19200; do
    echo "" | tee -a "$LOG"
    echo "=== baud $BAUD ===" | tee -a "$LOG"
    stty -F $DEV $BAUD cs8 -parenb -cstopb raw -echo -ixon -ixoff 2>&1 | tee -a "$LOG"

    # Drain
    rm -f /tmp/_drain.bin
    timeout 0.5 cat $DEV > /tmp/_drain.bin 2>/dev/null || true

    # Start receiver
    OUT=/tmp/lifetrac_p0c/user_fw_${BAUD}.bin
    rm -f $OUT
    cat $DEV > $OUT &
    PID=$!
    sleep 0.1

    # Pulse NRST low for 50 ms (chip sees BOOT0 low → flash boot)
    echo "0" > /sys/class/gpio/gpio$GPIO/value
    sleep 0.05
    echo "1" > /sys/class/gpio/gpio$GPIO/value

    # Listen for 1.5 s
    sleep 1.5
    kill -9 $PID 2>/dev/null; wait $PID 2>/dev/null

    SZ=$(stat -c %s $OUT)
    echo "rx bytes = $SZ" | tee -a "$LOG"
    if [ "$SZ" -gt 0 ]; then
        xxd $OUT | head -8 | tee -a "$LOG"
        # Try to detect ASCII
        ASCII=$(strings $OUT 2>/dev/null | head -3)
        if [ -n "$ASCII" ]; then
            echo "ASCII strings:" | tee -a "$LOG"
            echo "$ASCII" | tee -a "$LOG"
        fi
    else
        echo "(silent at $BAUD)" | tee -a "$LOG"
    fi
done

echo "" | tee -a "$LOG"
echo "=== final state: PA11 still LOW, NRST high (chip should be running user FW) ===" | tee -a "$LOG"
