#!/bin/sh
# baud_sweep_ver.sh — send VER_REQ at 921600 (RX side of L072 is fine), then
# capture the reply at several candidate bauds to find where the URC frames
# actually land. The L072 parses our 921600 writes (C:DISPATCH proves it);
# suspicion is its LPUART TX BRR drifted to a slower rate post-boot.
DEV=/dev/ttymxc3
for BAUD in 921600 460800 230400 115200 57600 38400 19200 9600; do
  stty -F $DEV 921600 cs8 -parenb -cstopb raw -echo -ixon -ixoff clocal
  timeout 0.3 cat $DEV > /dev/null 2>&1
  # switch read baud AFTER the write? No — one port, one baud. Write and read
  # both at $BAUD is wrong for TX (L072 RX is at 921600). So: write at 921600
  # first, then quickly retune to $BAUD to catch the reply tail.
  printf '\000\003\001\001\002\001\001\001\003\273\172\000' > $DEV
  stty -F $DEV $BAUD cs8 -parenb -cstopb raw -echo -ixon -ixoff clocal
  rm -f /tmp/sweep_$BAUD.bin
  timeout 1.2 cat $DEV > /tmp/sweep_$BAUD.bin 2>/dev/null
  SZ=$(stat -c %s /tmp/sweep_$BAUD.bin 2>/dev/null || echo 0)
  echo "== baud $BAUD reply_bytes=$SZ"
  [ "$SZ" -gt 0 ] && od -An -tx1 /tmp/sweep_$BAUD.bin | head -4
done
