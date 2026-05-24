#!/bin/sh
set -e
SSID="I'm a 5 star LAN!"
PSK="9182603083"
nmcli radio wifi on
nmcli dev wifi rescan 2>/dev/null || true
sleep 3
nmcli dev wifi connect "$SSID" password "$PSK" ifname wlan0
sleep 2
echo "--- connection status ---"
nmcli -t -f NAME,DEVICE,STATE con show --active
echo "--- ip ---"
ip -4 addr show wlan0
echo "--- ping ---"
ping -c 2 -W 2 8.8.8.8 || true
