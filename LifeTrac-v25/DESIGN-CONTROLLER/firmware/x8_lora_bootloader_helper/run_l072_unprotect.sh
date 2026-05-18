#!/bin/bash
# run_l072_unprotect.sh — drive OpenOCD to hold L072 in ROM, then run
# l072_unprotect.py (WRITE_UNPROTECT + READOUT_UNPROTECT) so the L072 will
# accept Extended Erase on the next Stage 1 cycle.
#
# Mirrors run_flash_l072.sh pattern but invokes the unprotect script instead
# of the flasher. Run as root from /tmp/lifetrac_p0c/ on the X8.

set -u
TOOLDIR=/tmp/lifetrac_p0c
CFG=$TOOLDIR/07_assert_pa11_pf4_long.cfg
UNPROT=$TOOLDIR/l072_unprotect.py
DEV=/dev/ttymxc3
LOG=$TOOLDIR/unprotect_run.log
OCD_LOG=$TOOLDIR/unprotect_ocd.log

: > $LOG
: > $OCD_LOG

echo "=== sanity ===" | tee -a $LOG
ls -la $CFG $UNPROT 2>&1 | tee -a $LOG
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
    echo "ERROR: openocd did not reach READY phase. Aborting unprotect." | tee -a $LOG
    kill -9 $OCD_PID 2>/dev/null
    exit 2
fi

echo "" | tee -a $LOG
echo "=== configure UART 19200 8E1 raw ===" | tee -a $LOG
stty -F $DEV 19200 cs8 parenb -parodd -cstopb raw -echo
stty -F $DEV -a 2>&1 | head -3 | tee -a $LOG

echo "" | tee -a $LOG
echo "=== running l072_unprotect.py ===" | tee -a $LOG
python3 -u $UNPROT 2>&1 | tee -a $LOG
RC=${PIPESTATUS[0]}
echo "unprotect exit code = $RC" | tee -a $LOG

echo "" | tee -a $LOG
echo "=== killing openocd ($OCD_PID) ===" | tee -a $LOG
kill -9 $OCD_PID 2>/dev/null
sleep 1

echo "" | tee -a $LOG
echo "=== final openocd log ===" | tee -a $LOG
tail -30 $OCD_LOG | tee -a $LOG

echo "" | tee -a $LOG
echo "=== overall result: unprotect RC=$RC ===" | tee -a $LOG
exit $RC
