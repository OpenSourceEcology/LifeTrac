#!/bin/bash
for f in /tmp/at_drain.bin /tmp/at_r1.bin /tmp/at_r2.bin /tmp/at_r3.bin; do
  echo "=== $f ==="
  ls -l $f 2>&1
  xxd $f 2>/dev/null | head -10
done
