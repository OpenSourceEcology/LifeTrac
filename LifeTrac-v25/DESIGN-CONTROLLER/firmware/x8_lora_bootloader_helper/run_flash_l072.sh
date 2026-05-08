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
nohup openocd -f /usr/arduino/extra/openocd_script-imx_gpio.cfg \
              -f $CFG \
              > $OCD_LOG 2>&1 < /dev/null &
OCD_PID=$!
echo "OCD_PID=$OCD_PID" | tee -a $LOG
sleep 5

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
python3 -u $FLASHER $DEV $IMAGE --verify 2>&1 | tee -a $LOG
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
