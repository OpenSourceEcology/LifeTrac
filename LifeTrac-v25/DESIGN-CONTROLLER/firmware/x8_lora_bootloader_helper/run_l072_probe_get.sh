#!/bin/bash
# run_l072_probe_get.sh — hold L072 in ROM, run l072_probe_get.py
set -u
TOOLDIR=/tmp/lifetrac_p0c
CFG=$TOOLDIR/07_assert_pa11_pf4_long.cfg
PROBE=$TOOLDIR/l072_probe_get.py
DEV=/dev/ttymxc3
OCD_LOG=$TOOLDIR/probe_get_ocd.log
: > $OCD_LOG
nohup openocd -f /usr/arduino/extra/openocd_script-imx_gpio.cfg -f $CFG > $OCD_LOG 2>&1 < /dev/null &
OCD_PID=$!
sleep 5
if ! grep -q "READY: L072 in STM32 ROM" $OCD_LOG; then
  echo "ERROR: openocd not READY"; tail -20 $OCD_LOG; kill -9 $OCD_PID 2>/dev/null; exit 2
fi
stty -F $DEV 19200 cs8 parenb -parodd -cstopb raw -echo
python3 -u $PROBE
RC=$?
kill -9 $OCD_PID 2>/dev/null
exit $RC
