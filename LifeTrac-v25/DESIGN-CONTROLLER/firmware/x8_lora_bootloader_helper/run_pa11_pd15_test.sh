#!/bin/bash
# run_pa11_pd15_test.sh — orchestrate openocd hold + verify probe.
set -u
LOG=/tmp/p0c_pa11_test.log
: > $LOG

echo "=== launching openocd in background (60s hold) ===" | tee -a $LOG
rm -f /tmp/p0c_04_ocd.log
nohup openocd -f /usr/arduino/extra/openocd_script-imx_gpio.cfg \
              -f /tmp/lifetrac_p0c/04_assert_pa11_pd15.cfg \
              > /tmp/p0c_04_ocd.log 2>&1 < /dev/null &
OCD_PID=$!
echo "OCD_PID=$OCD_PID" | tee -a $LOG

# Wait for openocd to assert pins and start the hold loop
sleep 5

echo "" | tee -a $LOG
echo "=== openocd output so far ===" | tee -a $LOG
cat /tmp/p0c_04_ocd.log | tee -a $LOG

echo "" | tee -a $LOG
echo "=== probing /dev/ttymxc3 8E1 for ROM ACK 0x79 ===" | tee -a $LOG
DEV=/dev/ttymxc3
stty -F $DEV 19200 cs8 parenb -parodd -cstopb raw -echo time 5 min 0 2>/dev/null

for ATTEMPT in 1 2 3; do
    echo "--- attempt $ATTEMPT ---" | tee -a $LOG
    rm -f /tmp/rom_$ATTEMPT.bin
    cat $DEV > /tmp/rom_$ATTEMPT.bin &
    CATPID=$!
    sleep 0.3
    printf '\x7f' > $DEV
    sleep 0.8
    kill -9 $CATPID 2>/dev/null
    wait $CATPID 2>/dev/null
    SZ=$(stat -c %s /tmp/rom_$ATTEMPT.bin)
    HEX=$(xxd -p /tmp/rom_$ATTEMPT.bin | tr -d '\n' | head -c 32)
    echo "  size=$SZ hex=${HEX}" | tee -a $LOG
    if [ "$SZ" -gt 0 ]; then
        FIRST=$(head -c 1 /tmp/rom_$ATTEMPT.bin | xxd -p | head -c 2)
        if [ "$FIRST" = "79" ]; then
            echo "  *** GOT 0x79 ACK — L072 IS IN STM32 ROM BOOTLOADER ***" | tee -a $LOG
            break
        fi
    fi
    sleep 0.5
done

echo "" | tee -a $LOG
echo "=== final openocd log ===" | tee -a $LOG
cat /tmp/p0c_04_ocd.log | tee -a $LOG

echo "" | tee -a $LOG
echo "=== waiting for openocd to finish ===" | tee -a $LOG
wait $OCD_PID 2>/dev/null
echo "done."
