#!/bin/bash
# rom_get_cmd.sh — assume ROM is post-autobaud at 19200 8E1 and send the
# GET command (0x00 followed by complement 0xFF). If ROM responds with
# 0x79 ACK + N + bootloader version + N command bytes + 0x79, we are in
# command mode and can flash via stm32flash --no-init.
set -u
DEV=/dev/ttymxc3
LOG=/tmp/lifetrac_p0c/rom_get_cmd.log
mkdir -p /tmp/lifetrac_p0c
: > "$LOG"

stty -F $DEV 19200 cs8 parenb -parodd -cstopb raw -echo -ixon -ixoff -inpck -istrip 2>&1 | tee -a "$LOG"

# Drain
rm -f /tmp/rgc_drain.bin
timeout 0.5 cat $DEV > /tmp/rgc_drain.bin 2>/dev/null || true
echo "drain bytes=$(stat -c %s /tmp/rgc_drain.bin)" | tee -a "$LOG"

# Listener
rm -f /tmp/rgc_resp.bin
cat $DEV > /tmp/rgc_resp.bin &
PID=$!
sleep 0.1

# Send GET = 0x00, complement 0xFF
printf '\x00\xff' > $DEV
sleep 0.5
kill -9 $PID 2>/dev/null; wait $PID 2>/dev/null

SZ=$(stat -c %s /tmp/rgc_resp.bin)
echo "GET response size=$SZ" | tee -a "$LOG"
xxd /tmp/rgc_resp.bin | tee -a "$LOG"
FIRST=$(xxd -p -l 1 /tmp/rgc_resp.bin)
case "$FIRST" in
    79) echo "RESULT: ACK + payload — ROM is in command mode."; exit 0 ;;
    1f) echo "RESULT: NACK — chip is in ROM but command framing wrong (mid-state)."; exit 10 ;;
    "") echo "RESULT: silence — chip not responsive on this baud/parity."; exit 20 ;;
    *)  echo "RESULT: unexpected first byte 0x$FIRST"; exit 12 ;;
esac
