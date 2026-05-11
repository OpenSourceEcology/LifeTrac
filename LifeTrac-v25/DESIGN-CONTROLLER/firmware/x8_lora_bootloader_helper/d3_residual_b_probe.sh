#!/bin/sh
# Residual B discriminator: read AT+STAT? before/after a binary VER_REQ.
# Decides whether the frame: never arrived, arrived-but-failed-parse, or
# arrived-parsed-but-no-reply.
PORT=/dev/ttymxc3
BAUD=921600
OUT=/tmp/d3_residual_b.log
HEX=0003010101010101030f0c00

stty -F "$PORT" "$BAUD" cs8 -parenb -cstopb raw -echo -ixon -ixoff -ixany -crtscts
rm -f "$OUT"

# background capture
( cat "$PORT" > "$OUT" ) &
CAT=$!
sleep 0.3
trap "kill $CAT 2>/dev/null" EXIT INT TERM

# Snapshot 1
echo '--- send AT+STAT? (snapshot 1) ---'
printf 'AT+STAT?\r\n' > "$PORT"
sleep 0.6

# Send raw binary frame
echo '--- send binary VER_REQ (12 bytes) ---'
printf '%s' "$HEX" | xxd -r -p > "$PORT"
sleep 0.5

# Snapshot 2
echo '--- send AT+STAT? (snapshot 2) ---'
printf 'AT+STAT?\r\n' > "$PORT"
sleep 0.7

kill "$CAT" 2>/dev/null
wait "$CAT" 2>/dev/null

echo '----- HEX DUMP -----'
xxd "$OUT"
echo '----- ASCII (filtered) -----'
tr -c '[:print:]\r\n=' '.' < "$OUT"
echo
