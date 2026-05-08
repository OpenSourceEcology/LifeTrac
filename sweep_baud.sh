#!/bin/bash
DEV=/dev/ttymxc3
stty -F $DEV 19200 cs8 -parenb -cstopb raw -echo -ixon -ixoff time 5 min 0
# clear
timeout 1 cat $DEV > /dev/null || true
for CMD in "AT\r" "AT\r" "AT\n"; do
  echo "--- Testing $CMD ---"
  rm -f /tmp/sweep.bin
  cat $DEV > /tmp/sweep.bin &
  PID=$!
  sleep 0.5
  printf "$CMD" > $DEV
  sleep 0.5
  kill -9 $PID 2>/dev/null; wait $PID 2>/dev/null
  cat /tmp/sweep.bin | tr -c "[:print:]\n\r" "."
  echo ""
done

