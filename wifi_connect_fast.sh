#!/bin/sh
# Wait for brcmfmac firmware load to complete, then connect FAST
set -e
echo "=== waiting for wlan0 to appear ==="
for i in 1 2 3 4 5 6 7 8 9 10 15 20; do
  if ip link show wlan0 >/dev/null 2>&1; then
    echo "wlan0 ready at $i"
    break
  fi
  sleep 1
done
echo "=== rfkill unblock all ==="
rfkill unblock all
echo "=== nmcli wifi on ==="
nmcli radio wifi on
echo "=== rescan ==="
nmcli dev wifi rescan ifname wlan0 2>/dev/null || true
sleep 4
echo "=== connect ==="
nmcli dev wifi connect "I'm a 5 star LAN!" password "9182603083" ifname wlan0
sleep 3
echo "=== status ==="
nmcli -t -f NAME,DEVICE,STATE con show --active
ip -4 addr show wlan0
echo "=== ping ==="
ping -c 3 -W 2 8.8.8.8 || true
