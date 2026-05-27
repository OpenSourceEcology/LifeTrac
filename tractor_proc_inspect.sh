#!/bin/sh
echo "=== container Dockerfile ==="
echo fio | sudo -S cat /opt/lifetrac/compose-apps/lifetrac-camera/Dockerfile 2>&1 | head -40
echo
echo "=== container CMD ==="
echo fio | sudo -S docker inspect tractor-camera --format '{{.Config.Cmd}} :: {{.Config.Entrypoint}}'
echo
echo "=== container processes ==="
echo fio | sudo -S docker top tractor-camera
echo
echo "=== current ttymxc3 holder inside container (from host pid namespace) ==="
echo fio | sudo -S sh -c '
for d in /proc/[0-9]*; do
  pid=${d##/proc/}
  for fd in $d/fd/*; do
    tgt=$(readlink "$fd" 2>/dev/null)
    case "$tgt" in
      *ttymxc3*) echo PID=$pid CMD=$(cat $d/comm 2>/dev/null) FD=$tgt ;;
    esac
  done
done
'
echo
echo "=== writing a probe byte to ttymxc3 from camera_service via python ==="
echo fio | sudo -S docker exec tractor-camera python3 -c "
import os, sys
try:
    fh = open('/dev/ttymxc3', 'wb', buffering=0)
    fh.write(b'\xA5\x00\x00\x00\x00\x00')
    print('PROBE_OK wrote 6 bytes to /dev/ttymxc3')
except Exception as e:
    print('PROBE_FAIL', type(e).__name__, e)
"
