#!/bin/bash
# probe_at_clean.sh — clean AT probe assuming L072 is already running user FW.
DEV=/dev/ttymxc3
LOG=/tmp/lifetrac_p0c/probe_at_clean.log
: > $LOG

echo "=== UART configure 19200 8N1 raw ===" | tee -a $LOG
stty -F $DEV 19200 cs8 -parenb -cstopb raw -echo -ixon -ixoff time 5 min 0 2>&1 | tee -a $LOG

echo "" | tee -a $LOG
echo "=== drain any pending bytes (3 s) ===" | tee -a $LOG
rm -f /tmp/at_drain.bin
timeout 3 cat $DEV > /tmp/at_drain.bin || true
DRAIN_SZ=$(stat -c %s /tmp/at_drain.bin)
echo "drained $DRAIN_SZ bytes" | tee -a $LOG
xxd /tmp/at_drain.bin | head -8 | tee -a $LOG

echo "" | tee -a $LOG
echo "=== send AT and wait 2 s ===" | tee -a $LOG
rm -f /tmp/at_r1.bin
cat $DEV > /tmp/at_r1.bin &
PID=$!
sleep 0.5
printf 'AT\r\n' > $DEV
sleep 2.0
kill -9 $PID 2>/dev/null; wait $PID 2>/dev/null
echo "AT response = $(stat -c %s /tmp/at_r1.bin) bytes" | tee -a $LOG
xxd /tmp/at_r1.bin | head -10 | tee -a $LOG

echo "" | tee -a $LOG
echo "=== send AT+VER? and wait 2 s ===" | tee -a $LOG
rm -f /tmp/at_r2.bin
cat $DEV > /tmp/at_r2.bin &
PID=$!
sleep 0.5
printf 'AT+VER?\r\n' > $DEV
sleep 2.0
kill -9 $PID 2>/dev/null; wait $PID 2>/dev/null
echo "AT+VER? response = $(stat -c %s /tmp/at_r2.bin) bytes" | tee -a $LOG
xxd /tmp/at_r2.bin | head -10 | tee -a $LOG

echo "" | tee -a $LOG
echo "=== send AT+DEV? and wait 2 s ===" | tee -a $LOG
rm -f /tmp/at_r3.bin
cat $DEV > /tmp/at_r3.bin &
PID=$!
sleep 0.5
printf 'AT+DEV?\r\n' > $DEV
sleep 2.0
kill -9 $PID 2>/dev/null; wait $PID 2>/dev/null
echo "AT+DEV? response = $(stat -c %s /tmp/at_r3.bin) bytes" | tee -a $LOG
xxd /tmp/at_r3.bin | head -10 | tee -a $LOG

echo "" | tee -a $LOG
echo "=== done ===" | tee -a $LOG
