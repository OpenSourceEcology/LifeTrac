#!/bin/sh
echo "=== rescan (12s) ==="
nmcli dev wifi rescan ifname wlan0 2>&1 || true
sleep 12
echo "=== visible nets ==="
nmcli -t -f SSID,SIGNAL dev wifi list ifname wlan0 | head -n 15
echo "=== connect ==="
nmcli dev wifi connect "I'm a 5 star LAN!" password "9182603083" ifname wlan0
sleep 4
echo "=== active connections ==="
nmcli -t -f NAME,DEVICE,STATE con show --active
echo "=== ip ==="
ip -4 addr show wlan0
echo "=== ping ==="
ping -c 3 -W 2 8.8.8.8 || true
echo "=== dmesg tail ==="
dmesg | tail -n 10
