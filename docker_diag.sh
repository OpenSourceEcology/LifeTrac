#!/bin/sh
echo "=== docker.service ==="
systemctl status docker.service --no-pager -l | head -15
echo "=== docker journal tail ==="
journalctl -u docker.service -n 40 --no-pager | tail -40
echo "=== containerd ==="
systemctl is-active containerd.service
echo "=== dmesg last 30 ==="
dmesg | tail -30
