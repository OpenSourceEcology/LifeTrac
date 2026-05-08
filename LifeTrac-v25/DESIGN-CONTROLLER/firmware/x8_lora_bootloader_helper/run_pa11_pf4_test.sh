#!/bin/bash
# run_pa11_pf4_test.sh — BOOT0=PA_11 (PWM4), NRST=PF_4 (GPIO3).
set -u
CFG=/tmp/lifetrac_p0c/06_assert_pa11_pf4.cfg
LOG=/tmp/p0c_pf4_test.log
: > $LOG

echo "=== launching openocd in background ===" | tee -a $LOG
rm -f /tmp/p0c_06_ocd.log
nohup openocd -f /usr/arduino/extra/openocd_script-imx_gpio.cfg \
              -f $CFG \
              > /tmp/p0c_06_ocd.log 2>&1 < /dev/null &
OCD_PID=$!
sleep 5
echo "=== openocd output so far ===" | tee -a $LOG
cat /tmp/p0c_06_ocd.log | tee -a $LOG

echo "" | tee -a $LOG
echo "=== probing /dev/ttymxc3 8E1 for ROM ACK 0x79 ===" | tee -a $LOG
DEV=/dev/ttymxc3
stty -F $DEV 19200 cs8 parenb -parodd -cstopb raw -echo time 5 min 0 2>/dev/null

ACK_HIT=0
for ATTEMPT in 1 2 3 4; do
    echo "--- attempt $ATTEMPT ---" | tee -a $LOG
    rm -f /tmp/rom_pf4_${ATTEMPT}.bin
    cat $DEV > /tmp/rom_pf4_${ATTEMPT}.bin &
    CATPID=$!
    sleep 0.3
    printf '\x7f' > $DEV
    sleep 0.8
    kill -9 $CATPID 2>/dev/null
    wait $CATPID 2>/dev/null
    SZ=$(stat -c %s /tmp/rom_pf4_${ATTEMPT}.bin)
    HEX=$(xxd -p /tmp/rom_pf4_${ATTEMPT}.bin | tr -d '\n' | head -c 32)
    echo "  size=$SZ hex=${HEX}" | tee -a $LOG
    if [ "$SZ" -gt 0 ]; then
        FIRST=$(head -c 1 /tmp/rom_pf4_${ATTEMPT}.bin | xxd -p | head -c 2)
        if [ "$FIRST" = "79" ]; then
            echo "  *** GOT 0x79 ACK — L072 IS IN STM32 ROM BOOTLOADER ***" | tee -a $LOG
            ACK_HIT=1
            break
        fi
    fi
    sleep 0.5
done

echo "" | tee -a $LOG
echo "=== AT probe at 8N1 (sanity check post-reset) ===" | tee -a $LOG
stty -F $DEV 19200 cs8 -parenb -cstopb raw -echo time 5 min 0 2>/dev/null
rm -f /tmp/at_pf4.bin
cat $DEV > /tmp/at_pf4.bin &
CATPID=$!
sleep 0.3
printf 'AT\r\n' > $DEV
sleep 1.0
kill -9 $CATPID 2>/dev/null
wait $CATPID 2>/dev/null
SZ=$(stat -c %s /tmp/at_pf4.bin)
echo "  AT size=$SZ" | tee -a $LOG
xxd /tmp/at_pf4.bin | head -3 | tee -a $LOG

echo "" | tee -a $LOG
echo "=== ACK_HIT=$ACK_HIT ===" | tee -a $LOG

wait $OCD_PID 2>/dev/null
echo "done."
