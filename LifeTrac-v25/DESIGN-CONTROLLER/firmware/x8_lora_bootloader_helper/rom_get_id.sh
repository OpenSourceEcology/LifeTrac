#!/bin/bash
# rom_get_id.sh — read STM32 PID via ROM bootloader GET_ID command.
set -u
DEV=/dev/ttymxc3
LOG=/tmp/lifetrac_p0c/rom_get_id.log
mkdir -p /tmp/lifetrac_p0c
: > "$LOG"

stty -F $DEV 19200 cs8 parenb -parodd -cstopb raw -echo -ixon -ixoff -inpck -istrip
rm -f /tmp/id.bin
cat $DEV > /tmp/id.bin &
PID=$!
sleep 0.1
# GET_ID = 0x02, complement 0xFD
printf '\x02\xfd' > $DEV
sleep 0.5
kill -9 $PID 2>/dev/null; wait $PID 2>/dev/null
xxd /tmp/id.bin | tee -a "$LOG"
# Expected reply: 79 01 PID_MSB PID_LSB 79
# L072CZ PID = 0x447
