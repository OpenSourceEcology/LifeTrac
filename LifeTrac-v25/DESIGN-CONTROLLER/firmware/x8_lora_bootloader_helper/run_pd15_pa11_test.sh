#!/bin/bash
# run_pd15_pa11_test.sh — same as run_pa11_pd15_test.sh but loads the swapped cfg.
set -u
CFG=/tmp/lifetrac_p0c/05_assert_pd15_pa11.cfg
LOG=/tmp/p0c_pd15_test.log
: > $LOG

echo "=== launching openocd in background ===" | tee -a $LOG
rm -f /tmp/p0c_05_ocd.log
nohup openocd -f /usr/arduino/extra/openocd_script-imx_gpio.cfg \
              -f $CFG \
              > /tmp/p0c_05_ocd.log 2>&1 < /dev/null &
OCD_PID=$!
sleep 5
echo "=== openocd output so far ===" | tee -a $LOG
cat /tmp/p0c_05_ocd.log | tee -a $LOG

echo "" | tee -a $LOG
echo "=== probing /dev/ttymxc3 8E1 for ROM ACK 0x79 ===" | tee -a $LOG
DEV=/dev/ttymxc3
stty -F $DEV 19200 cs8 parenb -parodd -cstopb raw -echo time 5 min 0 2>/dev/null

for ATTEMPT in 1 2 3; do
    echo "--- attempt $ATTEMPT ---" | tee -a $LOG
    rm -f /tmp/rom_${ATTEMPT}_v2.bin
    cat $DEV > /tmp/rom_${ATTEMPT}_v2.bin &
    CATPID=$!
    sleep 0.3
    printf '\x7f' > $DEV
    sleep 0.8
    kill -9 $CATPID 2>/dev/null
    wait $CATPID 2>/dev/null
    SZ=$(stat -c %s /tmp/rom_${ATTEMPT}_v2.bin)
    HEX=$(xxd -p /tmp/rom_${ATTEMPT}_v2.bin | tr -d '\n' | head -c 32)
    echo "  size=$SZ hex=${HEX}" | tee -a $LOG
    if [ "$SZ" -gt 0 ]; then
        FIRST=$(head -c 1 /tmp/rom_${ATTEMPT}_v2.bin | xxd -p | head -c 2)
        if [ "$FIRST" = "79" ]; then
            echo "  *** GOT 0x79 ACK ***" | tee -a $LOG
            break
        fi
    fi
    sleep 0.5
done

echo "" | tee -a $LOG
echo "=== also try AT probe at 8N1 to see if firmware behaviour changed ===" | tee -a $LOG
stty -F $DEV 19200 cs8 -parenb -cstopb raw -echo time 5 min 0 2>/dev/null
rm -f /tmp/at_v2.bin
cat $DEV > /tmp/at_v2.bin &
CATPID=$!
sleep 0.3
printf 'AT\r\n' > $DEV
sleep 1.0
kill -9 $CATPID 2>/dev/null
wait $CATPID 2>/dev/null
SZ=$(stat -c %s /tmp/at_v2.bin)
HEX=$(xxd -p /tmp/at_v2.bin | tr -d '\n' | head -c 64)
echo "  AT size=$SZ hex=${HEX}" | tee -a $LOG
echo "  AT decoded:" | tee -a $LOG
xxd /tmp/at_v2.bin | head -3 | tee -a $LOG

wait $OCD_PID 2>/dev/null
echo "done."
