#!/bin/sh
# Inspect tractor LoRa TX state
echo "=== camera_service container ==="
echo fio | sudo -S docker ps --filter name=tractor-camera --format "{{.Names}}	{{.Status}}"
echo
echo "=== ttymxc3 counters (current) ==="
ls -l /sys/class/tty/ttymxc3/iomem_reg_shift 2>/dev/null || true
echo fio | sudo -S cat /proc/tty/driver/serial 2>/dev/null | grep -E '^3:|ttymxc3' || true
# fallback via sysfs
for f in /sys/class/tty/ttymxc3/statistics/*; do
    [ -f "$f" ] && echo "  $(basename $f): $(cat $f)"
done
echo
echo "=== tractor-camera logs (last 30 lines, looking for TX_FRAME / ERR_PROTO / RFCO_PERTX / FORBIDDEN) ==="
echo fio | sudo -S docker logs --tail 60 tractor-camera 2>&1 | tail -40
echo
echo "=== service journal (last 30 lines from lifetrac-camera) ==="
echo fio | sudo -S journalctl -u lifetrac-camera.service --no-pager -n 20 2>&1 | tail -20
