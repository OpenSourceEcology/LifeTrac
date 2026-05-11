#!/bin/sh
# Phase D3 - bash-only ingress test (v1.3 plan §9.1).
# Bypasses Python probe to discriminate firmware-side vs. probe-side bug.
# Assumes L072 already running v1.2 firmware on /dev/ttymxc3 @ 921600.
set -u
DEV=${1:-/dev/ttymxc3}
OUT=/tmp/l072_d3_rx.bin
rm -f "$OUT"
stty -F "$DEV" 921600 cs8 -parenb -cstopb raw -echo -ixon -ixoff -ixany -crtscts
# start background reader
( cat "$DEV" > "$OUT" ) &
CAT_PID=$!
sleep 0.3
echo "--- send ATI ---"
printf 'ATI\r\n' > "$DEV"
sleep 0.6
echo "--- send AT+VER? ---"
printf 'AT+VER?\r\n' > "$DEV"
sleep 0.6
echo "--- send AT+STAT? ---"
printf 'AT+STAT?\r\n' > "$DEV"
sleep 0.6
echo "--- send second ATI ---"
printf 'ATI\r\n' > "$DEV"
sleep 0.8
kill "$CAT_PID" 2>/dev/null
wait "$CAT_PID" 2>/dev/null
SIZE=$(wc -c < "$OUT")
echo "--- captured $SIZE bytes ---"
echo "--- xxd dump ---"
xxd "$OUT" || od -A x -tx1z -v "$OUT"
echo "--- printable strings (one per line, splitting on CR/LF) ---"
tr '\r' '\n' < "$OUT" | sed 's/[^[:print:]]/./g' | nl -ba
