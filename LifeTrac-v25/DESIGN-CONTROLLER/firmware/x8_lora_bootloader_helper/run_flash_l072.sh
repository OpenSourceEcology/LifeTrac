#!/bin/bash
# run_flash_l072.sh — orchestrate the full flash:
#   1) launch openocd holding BOOT0 high, NRST pulsed, target halted (600s window)
#   2) wait 5s for ROM-bootloader entry
#   3) check fuser /dev/ttymxc3 (bail if held)
#   4) run stm32_an3155_flasher.py
#   5) wait for openocd to release at 600s mark (or kill it early on success)
#
# Run from /tmp/lifetrac_p0c/ on the X8.

set -u
TOOLDIR=/tmp/lifetrac_p0c
CFG=$TOOLDIR/07_assert_pa11_pf4_long.cfg
FLASHER=$TOOLDIR/stm32_an3155_flasher.py
IMAGE=${1:-$TOOLDIR/mlm32l07x01.bin}
DEV=/dev/ttymxc3
LOG=$TOOLDIR/flash_run.log
OCD_LOG=$TOOLDIR/flash_ocd.log

: > $LOG
: > $OCD_LOG

echo "=== sanity ===" | tee -a $LOG
ls -la $CFG $FLASHER $IMAGE 2>&1 | tee -a $LOG
HOLDERS=$(fuser $DEV 2>&1 || true)
echo "fuser $DEV: $HOLDERS" | tee -a $LOG

echo "" | tee -a $LOG
echo "=== launching openocd (600s hold) ===" | tee -a $LOG
# Per-cycle gpio preflight: ensure gpio8/10/15 are exported and gpio10 (H7 NRST)
# is driven high and given time to settle BEFORE openocd attaches. Then UNEXPORT
# them so openocd's imx_gpio mmap driver has uncontested access (sysfs-owned gpio
# pins can collide with openocd's direct register writes via /dev/mem mmap).
# Symptom of the collision on the RX X8 (OpenOCD 0.11.0-dirty 2025-07-14):
# SWD DPIDR returns 0xdeadbeef and OCD never reaches `init` / Phase A.
for n in 8 10 15; do
    [ -d /sys/class/gpio/gpio$n ] || echo $n > /sys/class/gpio/export 2>/dev/null
done
echo out > /sys/class/gpio/gpio10/direction 2>/dev/null
echo 1   > /sys/class/gpio/gpio10/value     2>/dev/null
sleep 1
echo "gpio10_value_before_unexport=$(cat /sys/class/gpio/gpio10/value 2>/dev/null)" | tee -a $LOG
for n in 8 10 15; do
    [ -d /sys/class/gpio/gpio$n ] && echo $n > /sys/class/gpio/unexport 2>/dev/null
done

nohup openocd -f /usr/arduino/extra/openocd_script-imx_gpio.cfg \
              -f $CFG \
              > $OCD_LOG 2>&1 < /dev/null &
OCD_PID=$!
echo "OCD_PID=$OCD_PID" | tee -a $LOG

# Poll for READY banner up to 25s. The newer X8 OpenOCD build
# (0.11.0-dirty 2025-07-14) on the RX X8 (e.g. 2D0A1209DABC240B) can take
# 8-12s between gpio10-high settle and SWD-attach completion, vs <5s on
# the older bundled OpenOCD on the TX X8. The previous fixed `sleep 5`
# raced this and falsely reported "openocd did not reach READY phase".
READY_DEADLINE=$(( $(date +%s) + 25 ))
while [ "$(date +%s)" -lt "$READY_DEADLINE" ]; do
    if grep -q "READY: L072 in STM32 ROM bootloader" "$OCD_LOG"; then
        break
    fi
    sleep 1
done

echo "" | tee -a $LOG
echo "=== openocd output so far ===" | tee -a $LOG
cat $OCD_LOG | tee -a $LOG

if ! grep -q "READY: L072 in STM32 ROM bootloader" $OCD_LOG ; then
    echo "" | tee -a $LOG
    echo "ERROR: openocd did not reach READY phase. Aborting flash." | tee -a $LOG
    kill -9 $OCD_PID 2>/dev/null
    exit 2
fi

echo "" | tee -a $LOG
echo "=== configure UART 19200 8E1 raw ===" | tee -a $LOG
stty -F $DEV 19200 cs8 parenb -parodd -cstopb raw -echo
stty -F $DEV -a 2>&1 | head -3 | tee -a $LOG

echo "" | tee -a $LOG
echo "=== running flasher (with --verify) ===" | tee -a $LOG
python3 -u $FLASHER $IMAGE --verify 2>&1 | tee -a $LOG
RC=${PIPESTATUS[0]}
echo "flasher exit code = $RC" | tee -a $LOG

echo "" | tee -a $LOG
echo "=== killing openocd ($OCD_PID) ===" | tee -a $LOG
kill -9 $OCD_PID 2>/dev/null
sleep 1

echo "" | tee -a $LOG
echo "=== final openocd log ===" | tee -a $LOG
tail -30 $OCD_LOG | tee -a $LOG

echo "" | tee -a $LOG
echo "=== overall result: flasher RC=$RC ===" | tee -a $LOG
exit $RC
